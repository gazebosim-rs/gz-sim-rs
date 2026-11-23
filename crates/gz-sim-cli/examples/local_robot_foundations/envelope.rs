// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::gripper::{Axis, body, ramp};
use super::*;
use gz_physics::BodyId;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::grasp_guard::{GraspGuard, GraspResponse, PadReading};
use serde::Serialize;

const BASE: &str = include_str!("../../../../examples/local_robot_gripper.sdf");
const END: usize = 5000;
const LIFT_START: usize = 700;
const DT: f64 = 0.001;
const RAIL: f64 = 0.36;

#[derive(Clone, Copy, Debug, Serialize)]
struct Load {
    mass_kg: f64,
    com_x_m: f64,
    friction: f64,
    travel_s: f64,
}

impl Load {
    // Scale the centered box's moments with its mass. The authored COM offset
    // represents a different mass distribution with the same principal moments.
    fn xml(self) -> String {
        with_load(BASE, self.mass_kg, self.com_x_m, self.friction)
    }
}

pub(super) fn with_load(base: &str, mass_kg: f64, com_x_m: f64, friction: f64) -> String {
    let (before, parcel) = base.split_once("<model name=\"parcel\">").unwrap();
    let (parcel, after) = parcel.split_once("</model>").unwrap();
    let (parcel_pose, inertial) = parcel.split_once("<inertial>").unwrap();
    let (_, collision_and_visual) = inertial.split_once("</inertial>").unwrap();
    let parcel = format!(
        "{parcel_pose}<inertial><pose>{} 0 0 0 0 0</pose><mass>{}</mass><inertia><ixx>{}</ixx><iyy>{}</iyy><izz>{}</izz></inertia></inertial>{collision_and_visual}",
        com_x_m,
        mass_kg,
        mass_kg * (0.075_f64.powi(2) + 0.1_f64.powi(2)) / 12.,
        mass_kg * (0.086_f64.powi(2) + 0.1_f64.powi(2)) / 12.,
        mass_kg * (0.086_f64.powi(2) + 0.075_f64.powi(2)) / 12.
    );
    // Vary only the two pads and parcel; the stands retain their original material.
    let (gantry, pads) = before.split_once("<link name=\"left\">").unwrap();
    let pads_and_box = format!("{pads}<model name=\"parcel\">{parcel}")
        .replace("<mu>0.7</mu>", &format!("<mu>{friction}</mu>"))
        .replace("<mu2>0.7</mu2>", &format!("<mu2>{friction}</mu2>"));
    format!("{gantry}<link name=\"left\">{pads_and_box}</model>{after}")
}

pub(super) struct Tactile {
    pub(super) pads: [PadReading; 2],
    pub(super) station_up_ns: [f64; 3],
    pub(super) station_ns: [f64; 3],
}

fn velocity_at(s: BodyState, lever: [f64; 3]) -> DVec3 {
    DVec3::from(s.velocity) + DVec3::from(s.angular_velocity).cross(lever.into())
}

pub(super) fn tactile(
    sim: &Simulation,
    load: BodyId,
    jaws: [BodyId; 2],
    supports: [BodyId; 3],
) -> Result<Tactile> {
    let physics = sim.world().resource::<PhysicsBackend>();
    let solver = physics
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing local solver")?;
    let mut pads = [PadReading::default(); 2];
    let mut station_up_ns = [0.; 3];
    let mut station_ns = [0.; 3];
    for patch in solver.rigid_contact_patches() {
        let load_is_a = patch.a == load;
        let peer = if load_is_a {
            patch.b
        } else if patch.b == load {
            patch.a
        } else {
            continue;
        };
        if let Some(i) = supports.iter().position(|s| *s == peer) {
            let impulse = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
            let up = if load_is_a {
                patch.normal_on_a[2]
            } else {
                -patch.normal_on_a[2]
            };
            station_up_ns[i] += impulse * up;
            station_ns[i] += impulse;
            continue;
        }
        let Some(i) = jaws.iter().position(|p| *p == peer) else {
            continue;
        };
        let load_state = physics.state(load).ok_or("missing parcel state")?;
        let pad_state = physics.state(peer).ok_or("missing jaw state")?;
        let normal = DVec3::from(patch.normal_on_a);
        for point in &patch.points {
            let (load_r, pad_r) = if load_is_a {
                (point.lever_a, point.lever_b)
            } else {
                (point.lever_b, point.lever_a)
            };
            let relative = velocity_at(load_state, load_r) - velocity_at(pad_state, pad_r);
            let shear = (relative - normal * relative.dot(normal)).length();
            let force = point.normal_impulse / sim.step_duration().as_secs_f64();
            pads[i].slip_speed_m_s += shear * force;
            pads[i].normal_force_n += force;
        }
    }
    for pad in &mut pads {
        if pad.normal_force_n > 0. {
            pad.slip_speed_m_s /= pad.normal_force_n;
        }
    }
    Ok(Tactile {
        pads,
        station_ns,
        station_up_ns,
    })
}

#[derive(Copy, Clone)]
struct Motion {
    rail: f64,
    vx: f64,
    height: f64,
    vz: f64,
}
impl Motion {
    fn nominal(t: f64, path: f64) -> Self {
        let (lift, vl) = ramp(t, 0.70, 0.55);
        let (rail, vx) = ramp(t, 1.50, path);
        let (lower, vd) = ramp(t, 2.75, 0.65);
        Self {
            rail: RAIL * rail,
            vx: RAIL * vx,
            height: 0.16 * lift - 0.145 * lower,
            vz: 0.16 * vl - 0.145 * vd,
        }
    }
    fn park(self, t: f64, station: f64) -> Self {
        let (travel, vx) = ramp(t, 0., 0.60);
        let (lower, vd) = ramp(t, 0.60, 0.52);
        Self {
            rail: self.rail + (station - self.rail) * travel,
            vx: (station - self.rail) * vx,
            height: self.height * (1. - lower),
            vz: -self.height * vd,
        }
    }
}

#[derive(Debug, Clone, Serialize, PartialEq)]
struct Outcome {
    outcome: &'static str,
    first_tactile_alarm_tick: Option<usize>,
    tighten_tick: Option<usize>,
    set_down_tick: Option<usize>,
    release_tick: Option<usize>,
    independent_slip_tick: Option<usize>,
    final_m: [f64; 3],
    placement_error_m: f64,
    floor_contact_ticks: usize,
    final_station_loaded_ticks: usize,
    closed_loaded_ticks_of_50: usize,
    final_jaw_loaded_ticks: usize,
    peak_height_loss_m: f64,
    control_step_p95_ns: u64,
    physics_step_p95_ns: u64,
}
fn p95(samples: &mut [u64]) -> u64 {
    let index = (samples.len() * 95).div_ceil(100) - 1;
    *samples.select_nth_unstable(index).1
}

fn cycle(
    r: &mut Recording,
    load: Load,
    feedback: bool,
    capture: bool,
) -> Result<(Outcome, Vec<Value>)> {
    let drives = [
        ("rail", 650., 55., 35.),
        ("lift", 1000., 60., 45.),
        ("close_left", 650., 12., 9.),
        ("close_right", 650., 12., 9.),
    ]
    .into_iter()
    .map(|(name, k, d, limit)| Axis::new(&r.sim, name, k, d, limit))
    .collect::<Result<Vec<_>>>()?;
    let link = |name: &str| entity_id(&r.sim, &format!("default::{name}"));
    let parcel = link("parcel::box")?;
    let palm = link("gripper::palm")?;
    let load_id = body(&r.sim, parcel)?;
    let mut peers = [load_id; 5];
    for (slot, name) in peers.iter_mut().zip([
        "gripper::left",
        "gripper::right",
        "source::body",
        "target::body",
        "floor::body",
    ]) {
        *slot = link(name).and_then(|id| body(&r.sim, id))?;
    }
    let [left_id, right_id, source, target, floor_id] = peers;
    let jaws = [left_id, right_id];
    let supports = [source, target, floor_id];
    let mut guard = GraspGuard::default();
    let mut response = GraspResponse::Hold;
    let mut tightened = None;
    let mut parking: Option<(usize, Motion, f64)> = None;
    let mut released = None;
    let mut supported_for = 0;
    let mut final_station = [0usize; 2];
    let mut floor = 0;
    let mut opposed = 0;
    let mut final_jaws = 0;
    let mut peak_loss = 0_f64;
    let mut initial_relative_z = 0.;
    let mut slipped = None;
    let mut samples = vec![];
    let mut timing = Vec::with_capacity(END);
    if capture {
        r.capture()?;
    }
    for tick in 1..=END {
        let begin = Instant::now();
        let t = (tick - 1) as f64 * DT;
        let mut motion = Motion::nominal(t, load.travel_s);
        if let Some((start, initial, station)) = parking {
            motion = initial.park((tick - start) as f64 * DT, station);
        }
        let (close, vc) = ramp(t, 0.12, 0.33);
        let (open, vo) =
            released.map_or((0., 0.), |start| ramp((tick - start) as f64 * DT, 0., 0.17));
        if let Some(start) = released {
            let (retreat, vr) = ramp((tick - start) as f64 * DT, 0.20, 0.45);
            motion.height += 0.11 * retreat;
            motion.vz += 0.11 * vr;
        }
        let stroke = if feedback && response != GraspResponse::Hold {
            0.0695
        } else {
            0.0615
        };
        let jaw = (stroke * (close - open), stroke * (vc - vo), 0.);
        let commands = [
            (motion.rail, motion.vx, 0.),
            (motion.height, motion.vz, 0.96 * 9.81),
            jaw,
            jaw,
        ];
        for (drive, (position, velocity, bias)) in drives.iter().zip(commands) {
            drive.command(&mut r.sim, position, velocity, bias)?;
        }
        r.advance()?;
        let sense = tactile(&r.sim, load_id, jaws, supports)?;
        if (LIFT_START - 50..LIFT_START).contains(&tick)
            && sense.pads.iter().all(|p| p.normal_force_n >= 0.05)
        {
            opposed += 1;
        }
        if tick > LIFT_START && released.is_none() && !(tick >= 2750 && sense.station_ns[1] > 0.) {
            response = guard.observe(r.sim.step_duration(), sense.pads);
            if response != GraspResponse::Hold {
                tightened.get_or_insert(tick);
            }
            if response == GraspResponse::SetDown && parking.is_none() && feedback {
                parking = Some((
                    tick,
                    motion,
                    if motion.rail < RAIL / 2. { 0. } else { RAIL },
                ));
            }
        }
        if let Some((start, _, station)) = parking {
            let support = usize::from(station > 0.);
            if tick > start + 1120 && sense.station_ns[support] > 0. {
                supported_for += 1;
            } else {
                supported_for = 0;
            }
            if supported_for >= 20 {
                released.get_or_insert(tick);
            }
        } else if tick >= 3450 {
            released.get_or_insert(tick);
        }
        timing.push(begin.elapsed().as_nanos() as u64);
        let pos = state(&r.sim, parcel)?.position;
        let relative = pos[2] - state(&r.sim, palm)?.position[2];
        if tick == LIFT_START {
            initial_relative_z = relative;
        }
        if tick > LIFT_START && released.is_none() && parking.is_none() {
            peak_loss = peak_loss.max(initial_relative_z - relative);
        }
        if tick > LIFT_START && released.is_none() && initial_relative_z - relative > 0.00025 {
            slipped.get_or_insert(tick);
        }
        if tick > END - 200 {
            final_jaws += usize::from(sense.pads.iter().any(|p| p.normal_force_n > 0.));
            for (count, impulse) in final_station.iter_mut().zip(sense.station_ns) {
                *count += usize::from(impulse > 0.);
            }
        }
        floor += usize::from(sense.station_ns[2] > 0.);
        if capture && tick % 50 == 0 {
            r.capture()?;
            samples.push(json!({"tick":tick,"parcel_m":pos,"pad_force_n":sense.pads.map(|s|s.normal_force_n),
            "local_shear_speed_m_s":sense.pads.map(|s|s.slip_speed_m_s),"relative_height_change_m":relative-initial_relative_z,
            "station_impulses_ns":sense.station_ns,"force_command_target_stroke_m":stroke,"response":format!("{response:?}")}));
        }
    }
    let pos = state(&r.sim, parcel)?.position;
    let dest = parking.map_or(RAIL, |(_, _, s)| s);
    let error = DVec3::from(pos).distance(DVec3::new(dest, 0., 0.17));
    let stable = final_station[usize::from(dest > 0.)] >= 190
        && final_jaws == 0
        && opposed == 50
        && floor == 0
        && released.is_some()
        && error < 0.012;
    let outcome = match (stable, parking.is_some(), tightened.is_some() && feedback) {
        (true, true, _) => "safe_set_down",
        (true, false, true) => "recovered_carry",
        (true, false, false) => "carry",
        _ => "miss",
    };
    Ok((
        Outcome {
            outcome,
            first_tactile_alarm_tick: tightened,
            tighten_tick: tightened.filter(|_| feedback),
            set_down_tick: parking.map(|e| e.0),
            release_tick: released,
            independent_slip_tick: slipped,
            final_m: pos,
            placement_error_m: error,
            floor_contact_ticks: floor,
            final_station_loaded_ticks: final_station[usize::from(dest > 0.)],
            closed_loaded_ticks_of_50: opposed,
            final_jaw_loaded_ticks: final_jaws,
            peak_height_loss_m: peak_loss,
            control_step_p95_ns: p95(&mut timing),
            physics_step_p95_ns: p95(&mut r.costs_ns.clone()),
        },
        samples,
    ))
}

fn recording(load: Load) -> Result<Recording> {
    Recording::new(&load.xml(), "local_robot_gripper.sdf", "default")
}

fn verify_reset_frames(original: &[(String, String)], repeat: &[(String, String)]) -> Result<()> {
    if original.len() != repeat.len() {
        return Err("reset changed frame count".into());
    }
    for ((prior, _), (again, _)) in original.iter().zip(repeat) {
        let mut prior: Value = serde_json::from_str(prior)?;
        let again: Value = serde_json::from_str(again)?;
        let epoch = prior["epoch"]
            .as_u64()
            .and_then(|n| n.checked_add(1))
            .ok_or("invalid world epoch")?;
        prior["epoch"] = json!(epoch);
        if prior != again {
            return Err("reset changed physics scene beyond its epoch".into());
        }
    }
    Ok(())
}

pub(super) fn run(out: &Path) -> Result<()> {
    let mut rows = Vec::new();

    for mass_kg in [0.2, 0.45, 0.85] {
        for friction in [0.35, 0.70] {
            for com_x_m in [0., 0.014] {
                for travel_s in [0.55, 0.95] {
                    let load = Load {
                        mass_kg,
                        com_x_m,
                        friction,
                        travel_s,
                    };
                    let (guarded, _) = cycle(&mut recording(load)?, load, true, false)?;
                    let (fixed, _) = cycle(&mut recording(load)?, load, false, false)?;
                    if com_x_m == 0. && friction == 0.35 && travel_s == 0.95 {
                        let (tag, expected) = if mass_kg == 0.2 {
                            ("success", "carry")
                        } else if mass_kg == 0.45 {
                            ("recovery", "recovered_carry")
                        } else {
                            ("overload", "safe_set_down")
                        };
                        if guarded.outcome != expected || (mass_kg > 0.2 && fixed.outcome != "miss")
                        {
                            return Err(format!(
                                "unexpected {tag} outcome: {guarded:?}; control: {fixed:?}"
                            )
                            .into());
                        }
                        let mut r = recording(load)?;
                        let (measured, trace) = cycle(&mut r, load, true, true)?;
                        let path = out.join(tag);
                        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
                        let original = std::mem::take(&mut r.frames);
                        let costs = std::mem::take(&mut r.costs_ns);
                        let (mut repeat, repeated_trace) = cycle(&mut r, load, true, true)?;
                        // Wall-clock costs vary, physical events and observations must reproduce exactly.
                        repeat.control_step_p95_ns = measured.control_step_p95_ns;
                        repeat.physics_step_p95_ns = measured.physics_step_p95_ns;
                        if measured != repeat || trace != repeated_trace {
                            return Err(format!("loaded reset changed measured {tag}").into());
                        }
                        verify_reset_frames(&original, &r.frames)?;
                        r.frames = original;
                        r.costs_ns = costs;
                        r.save(&path)?;
                        write_json(&path.join("load.json"), &json!(load))?;
                        write_json(&path.join("outcome.json"), &json!(measured))?;
                        write_json(&path.join("trace.json"), &json!(trace))?;
                        let event = measured
                            .set_down_tick
                            .or(measured.tighten_tick)
                            .unwrap_or(1950);
                        write_json(
                            &path.join("frames.json"),
                            &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"captures":{"grasp":14,"response":event.div_ceil(50),"carry":39,"placed":100}}),
                        )?;
                    }
                    rows.push(json!({"load":load,"peak_requested_horizontal_acceleration_m_s2":6.*RAIL/(travel_s*travel_s),"guarded":guarded,"fixed_pressure":fixed}));
                }
            }
        }
    }
    write_json(&out.join("campaign.json"), &json!(rows))?;
    Ok(())
}

#[cfg(test)]
#[path = "envelope_tests.rs"]
mod tests;
