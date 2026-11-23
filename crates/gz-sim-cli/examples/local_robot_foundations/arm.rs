// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
use gz_physics::{BodyId, JointState, joint_drive::JointDrive};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{JointSample, set_joint_force};
#[path = "arm/scaling.rs"]
mod scaling;
pub(super) const SCENE: &str = include_str!("../../../../examples/local_manipulation_arm.sdf");
const RADIUS: f64 = 0.32;
const END_US: u64 = 6_100_000;
const TURN: f64 = std::f64::consts::FRAC_PI_2;

fn ramp(t: f64, start: f64, duration: f64) -> (f64, f64) {
    let u = ((t - start) / duration).clamp(0., 1.);
    (u * u * (3. - 2. * u), 6. * u * (1. - u) / duration)
}

pub(super) struct Controller {
    ids: [u64; 4],
    drives: [JointDrive; 4],
}
impl Controller {
    pub(super) fn new(sim: &Simulation) -> Result<Self> {
        let id = |s| entity_id(sim, &format!("default::arm::{s}"));
        let jaw = JointDrive::new(650., 12., 0.55, 9.)?;
        Ok(Self {
            ids: [
                id("shoulder")?,
                id("lift")?,
                id("close_left")?,
                id("close_right")?,
            ],
            drives: [
                JointDrive::new(42., 5., 2.8, 4.)?,
                JointDrive::new(1000., 60., 0.55, 45.)?,
                jaw,
                jaw,
            ],
        })
    }
    /// Independent joint targets, in radians for shoulder and metres for slides.
    /// The low path deliberately drives into the obstacle in the negative control.
    pub(super) fn command(&self, sim: &mut Simulation, t: f64, low: bool) -> Result<[f64; 4]> {
        self.command_with_grip(sim, t, low, 0.)
    }

    pub(super) fn command_with_grip(
        &self,
        sim: &mut Simulation,
        t: f64,
        low: bool,
        extra_stroke: f64,
    ) -> Result<[f64; 4]> {
        let stroke = 0.065 + extra_stroke.clamp(0., 0.015);
        let (yaw, wy) = ramp(t, 1.40, 1.10);
        let (up, vu) = ramp(t, 0.70, 0.55);
        let (down, vd) = ramp(t, 2.67, 0.55);
        let (park, vp) = ramp(t, 3.55, 0.45);
        let (wake, vw) = ramp(t, 5.50, 0.45);
        let (close, vc) = ramp(t, 0.12, 0.33);
        let (open, vo) = ramp(t, 3.32, 0.16);
        let height = if low { 0.045 } else { 0.16 };
        let pinch_bias = (650. * extra_stroke.clamp(0., 0.015)).min(9.) * (close - open);
        let targets = [
            (TURN * yaw, TURN * wy, 0.),
            (
                height * up - (height - 0.025) * down - 0.037 * park + 0.10 * wake,
                height * vu - (height - 0.025) * vd - 0.037 * vp + 0.10 * vw,
                0.96 * 9.81,
            ),
            (stroke * (close - open), stroke * (vc - vo), pinch_bias),
            (stroke * (close - open), stroke * (vc - vo), pinch_bias),
        ];
        self.actuate(sim, targets, !(4.7..5.5).contains(&t))
    }

    pub(super) fn actuate(
        &self,
        sim: &mut Simulation,
        targets: [(f64, f64, f64); 4],
        enabled: bool,
    ) -> Result<[f64; 4]> {
        let mut efforts = [0.; 4];
        for (i, (position, velocity, bias)) in targets.into_iter().enumerate() {
            let state = self.sample(sim, i)?;
            if enabled {
                efforts[i] = self.drives[i].effort(state, position, velocity, bias)?;
            }
            set_joint_force(sim, self.ids[i], efforts[i])?;
        }
        Ok(efforts)
    }
    fn sample(&self, sim: &Simulation, i: usize) -> Result<JointState> {
        let e = entity_by_id(sim.world(), self.ids[i]).ok_or("missing joint")?;
        Ok(sim
            .world()
            .get::<JointSample>(e)
            .ok_or("missing joint measurement")?
            .state)
    }
}
fn handle(sim: &Simulation, id: u64) -> Result<BodyId> {
    let ent = entity_by_id(sim.world(), id).ok_or("missing link")?;
    Ok(sim
        .world()
        .get::<BodyHandle>(ent)
        .ok_or("missing body")?
        .id())
}
fn near_obstacle(p: BodyState, half: DVec3) -> Option<f64> {
    // World-space enclosing box from the body orientation. The reported vertical
    // clearance is conservative even when the two footprints only graze.
    let q = DQuat::from_array(p.orientation);
    let ext = (q * DVec3::X).abs() * half.x
        + (q * DVec3::Y).abs() * half.y
        + (q * DVec3::Z).abs() * half.z;
    let c = DVec3::from_array(p.position);
    ((c.x - 0.22627).abs() <= ext.x + 0.039 && (c.y - 0.22627).abs() <= ext.y + 0.039)
        .then_some(c.z - ext.z - 0.235)
}
fn metrics(r: &mut Recording, low: bool, capture: bool) -> Result<(Value, Vec<Value>)> {
    let controller = Controller::new(&r.sim)?;
    let names = [
        "arm::rotor",
        "arm::palm",
        "arm::left",
        "arm::right",
        "parcel::box",
        "source::body",
        "target::body",
        "floor::body",
        "obstacle::body",
    ];
    let link = names
        .map(|n| entity_id(&r.sim, &format!("default::{n}")))
        .into_iter()
        .collect::<Result<Vec<_>>>()?;
    let ids = link
        .iter()
        .map(|id| handle(&r.sim, *id))
        .collect::<Result<Vec<_>>>()?;
    let dt = r.sim.step_duration().as_micros() as u64;
    let mut obstruction = 0.;
    let mut obstacle_by_link = [0.; 5];
    let mut obstacle_hit_us = [None; 5];
    let mut fixture_impact = 0.;
    let mut unintended_self = 0.;
    let mut floor = 0.;
    let mut stand_during = 0.;
    let mut min_clear = f64::INFINITY;
    let mut max_radial = 0_f64;
    let mut max_height = 0_f64;
    let mut max_stop = 0_f64;
    let mut min_during = f64::INFINITY;
    let mut opposed = 0;
    let mut transfer_ticks = 0;
    let mut anchored = 0_f64;
    let mut source = false;
    let mut final_support = false;
    let mut first_hit = None;
    let mut max_load_speed = 0_f64;
    let mut trace = vec![];
    let bounds = [[-0.05, 1.64], [-0.012, 0.2], [-0.005, 0.08], [-0.005, 0.08]];
    if capture {
        r.capture()?;
    }
    for tick in 1..=END_US / dt {
        let us = tick * dt;
        let t = (us - dt) as f64 * 1e-6;
        let efforts = controller.command(&mut r.sim, t, low)?;
        r.advance()?;
        let mut contact = [0.; 9];
        let backend = r
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        for c in backend.rigid_contact_observations() {
            if c.a == ids[4] || c.b == ids[4] {
                let other = if c.a == ids[4] { c.b } else { c.a };
                if let Some(i) = ids.iter().position(|id| *id == other) {
                    contact[i] += c.normal_impulse;
                }
            }
            if c.a == ids[8] || c.b == ids[8] {
                obstruction += c.normal_impulse;
                let other = if c.a == ids[8] { c.b } else { c.a };
                if c.normal_impulse > 0.
                    && let Some(i) = ids[..5].iter().position(|peer| *peer == other)
                {
                    obstacle_by_link[i] += c.normal_impulse;
                    obstacle_hit_us[i].get_or_insert(us);
                }
                if c.normal_impulse > 0. {
                    first_hit.get_or_insert(us);
                }
            }
            if ids[..4].contains(&c.a) && ids[..4].contains(&c.b) {
                unintended_self += c.normal_impulse;
            }
            if (ids[..4].contains(&c.a) && ids[5..8].contains(&c.b))
                || (ids[..4].contains(&c.b) && ids[5..8].contains(&c.a))
            {
                fixture_impact += c.normal_impulse;
            }
        }
        let rotor = state(&r.sim, link[0])?;
        let qr = DQuat::from_array(rotor.orientation);
        anchored = anchored.max(
            (DVec3::from_array(rotor.position) - qr * DVec3::X * 0.16 - DVec3::Z * 0.67).length(),
        );
        let parcel = state(&r.sim, link[4])?;
        let p = DVec3::from_array(parcel.position);
        max_height = max_height.max(p.z);
        floor += contact[7];
        if us < 120_000 && contact[5] > 0. {
            source = true;
        }
        if (1_400_000..=2_500_000).contains(&us) {
            min_during = min_during.min(p.z);
            transfer_ticks += 1;
            opposed += usize::from(contact[2] > 0. && contact[3] > 0.);
            max_radial = max_radial.max((p.x.hypot(p.y) - RADIUS).abs());
            stand_during += contact[5] + contact[6];
            for (i, half) in [
                (2, [0.012, 0.0425, 0.07]),
                (3, [0.012, 0.0425, 0.07]),
                (4, [0.043, 0.0375, 0.05]),
            ] {
                if let Some(d) = near_obstacle(state(&r.sim, link[i])?, DVec3::from_array(half)) {
                    min_clear = min_clear.min(d);
                }
            }
        }
        if us > 5_800_000 {
            final_support = contact[6] > 0.;
            max_load_speed = max_load_speed.max(DVec3::from_array(parcel.velocity).length());
        }
        let mut q = [0.; 4];
        for (i, [lo, hi]) in bounds.into_iter().enumerate() {
            q[i] = controller.sample(&r.sim, i)?.position;
            max_stop = max_stop.max(lo - q[i]).max(q[i] - hi);
        }
        if us.is_multiple_of(50_000) {
            if capture {
                r.capture()?;
            }
            trace.push(json!({"time_us":us,"q":q,"efforts":efforts,"parcel_center":parcel.position,"parcel_rotation":parcel.orientation,"parcel_velocity":parcel.velocity,"impulse_ns":contact}));
        }
    }
    let p = DVec3::from_array(state(&r.sim, link[4])?.position);
    let error = (p - DVec3::new(0., RADIUS, 0.17)).length();
    let out = json!({"dt_us":dt,"blocked_route":low,"contact_with_obstacle_ns":obstruction,"obstacle_impulses_rotor_palm_left_right_parcel_ns":obstacle_by_link,"first_link_obstacle_hits_us":obstacle_hit_us,"first_obstacle_contact_us":first_hit,"initial_support":source,"final_target_support":final_support,
        "parcel_final_m":p.to_array(),"placement_error_m":error,"peak_parcel_height_m":max_height,"carry_min_height_m":min_during,"opposed_contact_ticks":opposed,"transfer_ticks":transfer_ticks,
        "min_vertical_gap_over_obstacle_m":min_clear,"max_radial_error_m":max_radial,"max_stop_violation":max_stop,"peak_shoulder_anchor_error_m":anchored,
        "carry_station_impulse_ns":stand_during,"parcel_floor_impulse_ns":floor,"arm_station_impulse_ns":fixture_impact,"self_contact_impulse_ns":unintended_self,
        "max_final_parcel_speed_m_s":max_load_speed});
    if low {
        if obstruction <= 0. || first_hit.is_none() {
            return Err(format!("low-arm route missed obstacle: {out}").into());
        }
    } else if !source
        || !final_support
        || error > 0.008
        || obstruction > 0.
        || unintended_self > 0.
        || floor > 0.
        || fixture_impact > 0.
        || stand_during > 0.
        || min_clear < 0.024
        || !min_clear.is_finite()
        || min_during < 0.31
        || max_radial > 0.008
        || anchored > 0.001
        || max_stop > 0.001
        || opposed * 100 < transfer_ticks * 98
        || max_load_speed > 0.005
    {
        return Err(format!("arm transfer failed: {out}").into());
    }
    Ok((out, trace))
}
pub(super) fn run(output: &Path) -> Result<()> {
    let mut table = vec![];
    for dt in [500, 1000, 2000] {
        let xml = SCENE.replace(
            "<max_step_size>0.001</max_step_size>",
            &format!("<max_step_size>{}</max_step_size>", f64::from(dt) * 1.0e-6),
        );
        for low in [false, true] {
            let capture = dt == 1000;
            let mut r = Recording::new(&xml, "local_manipulation_arm.sdf", "default")?;
            let (m, trace) = metrics(&mut r, low, capture)?;
            r.costs_ns.sort_unstable();
            let cost = r.costs_ns[(r.costs_ns.len() * 95).div_ceil(100) - 1];
            table.push(json!({"metrics":m,"step_p95_ns":cost}));
            if capture {
                gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
                let (again, again_trace) = metrics(&mut r, low, false)?;
                if again != m || trace != again_trace {
                    return Err("arm reset changed physical measurements".into());
                }
                r.costs_ns.truncate((END_US / 1000) as usize);
                let d = output.join(if low { "blocked" } else { "clear" });
                r.save(&d)?;
                write_json(&d.join("trajectory.json"), &json!(trace))?;
                write_json(
                    &d.join("frames.json"),
                    &json!({"kind":"authored-archive-v1","camera":"perspective","dt_ns":1,"captures":{"grip":13,"lift":27,"carry":38,"place":65,"end":122}}),
                )?;
            }
        }
    }
    write_json(&output.join("routes.json"), &json!(table))?;
    scaling::run(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn raised_arm_places_the_free_parcel_and_low_path_hits_the_barrier() -> Result<()> {
        let mut r = Recording::new(SCENE, "arm.sdf", "default")?;
        let (clear, trace) = metrics(&mut r, false, false)?;
        assert!(clear["min_vertical_gap_over_obstacle_m"].as_f64().unwrap() > 0.024);
        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
        assert_eq!((clear, trace), metrics(&mut r, false, false)?);
        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
        let (blocked, _) = metrics(&mut r, true, false)?;
        assert!(blocked["contact_with_obstacle_ns"].as_f64().unwrap() > 0.);
        Ok(())
    }
}
