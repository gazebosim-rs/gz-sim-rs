// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, JointState};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{JointSample, set_joint_force};

const SCENE: &str = include_str!("../../../../examples/local_robot_gripper.sdf");
const STROKE: f64 = 0.36;
const END: usize = 4100;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Trial {
    Grasp,
    OpenJaws,
    Frictionless,
}
impl Trial {
    fn recording(self) -> Result<Recording> {
        let smooth;
        let scene = if self == Self::Frictionless {
            smooth = SCENE.replace("<mu>0.7</mu><mu2>0.7</mu2>", "<mu>0</mu><mu2>0</mu2>");
            if smooth == SCENE {
                return Err("frictionless control did not change the surfaces".into());
            }
            &smooth
        } else {
            SCENE
        };
        Recording::new(scene, "local_robot_gripper.sdf", "default")
    }
}

/// Cubic position reference with zero speed at either end.
pub(super) fn ramp(time: f64, start: f64, duration: f64) -> (f64, f64) {
    let u = ((time - start) / duration).clamp(0., 1.);
    (u * u * (3. - 2. * u), 6. * u * (1. - u) / duration)
}

pub(super) struct Axis {
    id: u64,
    stiffness: f64,
    damping: f64,
    limit: f64,
}
impl Axis {
    pub(super) fn new(
        sim: &Simulation,
        name: &str,
        stiffness: f64,
        damping: f64,
        limit: f64,
    ) -> Result<Self> {
        Ok(Self {
            id: entity_id(sim, &format!("default::gripper::{name}"))?,
            stiffness,
            damping,
            limit,
        })
    }
    pub(super) fn command(
        &self,
        sim: &mut Simulation,
        position: f64,
        speed: f64,
        bias: f64,
    ) -> Result<()> {
        let entity = entity_by_id(sim.world(), self.id).ok_or("missing actuator")?;
        let JointState {
            position: actual,
            velocity,
            ..
        } = sim
            .world()
            .get::<JointSample>(entity)
            .ok_or("missing joint state")?
            .state;
        let force =
            (self.stiffness * (position - actual) + self.damping * (speed - velocity) + bias)
                .clamp(-self.limit, self.limit);
        set_joint_force(sim, self.id, force)?;
        Ok(())
    }
}
pub(super) fn body(sim: &Simulation, id: u64) -> Result<BodyId> {
    let entity = entity_by_id(sim.world(), id).ok_or("missing link")?;
    Ok(sim
        .world()
        .get::<BodyHandle>(entity)
        .ok_or("missing body handle")?
        .id())
}

fn cycle(r: &mut Recording, trial: Trial, capture: bool) -> Result<(Value, Vec<Value>)> {
    let grasp = trial == Trial::Grasp;
    let close_jaws = trial != Trial::OpenJaws;
    let rail = Axis::new(&r.sim, "rail", 650., 55., 35.)?;
    let lift = Axis::new(&r.sim, "lift", 1000., 60., 45.)?;
    let left = Axis::new(&r.sim, "close_left", 650., 12., 9.)?;
    let right = Axis::new(&r.sim, "close_right", 650., 12., 9.)?;
    let parcel = entity_id(&r.sim, "default::parcel::box")?;
    let palm = entity_id(&r.sim, "default::gripper::palm")?;
    let load = body(&r.sim, parcel)?;
    let peers = [
        "gripper::left",
        "gripper::right",
        "source::body",
        "target::body",
        "floor::body",
    ]
    .map(|s| entity_id(&r.sim, &format!("default::{s}")).and_then(|id| body(&r.sim, id)))
    .into_iter()
    .collect::<Result<Vec<_>>>()?;
    let mut trace = Vec::new();
    let mut start_ticks = 0;
    let mut settled_ticks = 0;
    let mut grasp_ticks = 0;
    let mut carry_ticks = 0;
    let mut first_release = None;
    let mut landing = None;
    let mut free_flight_ticks = 0;
    let mut floor_impulse = 0.;
    let mut carry_support_impulse = 0.;
    let mut final_jaw_impulse = 0.;
    let mut carry_min_z = f64::INFINITY;
    let mut max_z = 0_f64;
    let mut carry_ref = None;
    let mut carry_slip = 0_f64;
    let mut max_final_speed = 0_f64;
    let mut max_final_spin = 0_f64;
    let mut position_at_open = DVec3::ZERO;
    if capture {
        r.capture()?;
    }
    let dt = r.sim.step_duration().as_secs_f64();
    for tick in 1..=END {
        let t = (tick - 1) as f64 * dt;
        let (cross, vx) = ramp(t, 1.40, 0.85);
        let (up, vu) = ramp(t, 0.70, 0.55);
        let (down, vd) = ramp(t, 2.38, 0.55);
        let (retract, vr) = ramp(t, 3.30, 0.45);
        let (close, vc) = ramp(t, 0.12, 0.33);
        let (open, vo) = ramp(t, 3.00, 0.16);
        rail.command(&mut r.sim, STROKE * cross, STROKE * vx, 0.)?;
        lift.command(
            &mut r.sim,
            0.16 * up - 0.135 * down + 0.12 * retract,
            0.16 * vu - 0.135 * vd + 0.12 * vr,
            0.96 * 9.81,
        )?;
        let stroke = if close_jaws { 0.064 } else { 0. };
        left.command(&mut r.sim, stroke * (close - open), stroke * (vc - vo), 0.)?;
        right.command(&mut r.sim, stroke * (close - open), stroke * (vc - vo), 0.)?;
        r.advance()?;
        let mut impulses = [0.; 5];
        let backend = r
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local physics")?;
        for contact in backend.rigid_contact_observations() {
            let other = if contact.a == load {
                contact.b
            } else if contact.b == load {
                contact.a
            } else {
                continue;
            };
            if let Some(i) = peers.iter().position(|peer| *peer == other) {
                impulses[i] += contact.normal_impulse;
            }
        }
        let [a, b, source, target, floor] = impulses;
        let both = a > 0. && b > 0.;
        let payload = state(&r.sim, parcel)?;
        let position = DVec3::from(payload.position);
        let relative = position - DVec3::from(state(&r.sim, palm)?.position);
        floor_impulse += floor;
        max_z = max_z.max(position.z);
        match tick {
            61..=110 => start_ticks += usize::from(source > 0.),
            601..=700 => grasp_ticks += usize::from(both),
            1401..=2250 => {
                carry_ticks += usize::from(both);
                carry_support_impulse += source + target;
                carry_min_z = carry_min_z.min(position.z);
                let reference = *carry_ref.get_or_insert(relative);
                carry_slip = carry_slip.max(relative.distance(reference));
            }
            3000 => position_at_open = position,
            3001.. if grasp && a + b == 0. && first_release.is_none() => {
                first_release = Some(tick);
            }
            _ => {}
        }
        if tick > 3000 && target > 0. {
            landing.get_or_insert(tick);
        }
        if first_release.is_some() && landing.is_none() && a + b + source + target + floor == 0. {
            free_flight_ticks += 1;
        }
        if tick > 3800 {
            settled_ticks += usize::from(target > 0.);
            final_jaw_impulse += a + b;
            max_final_speed = max_final_speed.max(DVec3::from(payload.velocity).length());
            max_final_spin = max_final_spin.max(DVec3::from(payload.angular_velocity).length());
        }
        if tick % 50 == 0 {
            if capture {
                r.capture()?;
            }
            trace.push(json!({"tick":tick,"position_m":payload.position,"jaw_left_ns":a,"jaw_right_ns":b,
                "source_ns":source,"target_ns":target,"floor_ns":floor,"palm_to_parcel_m":relative.to_array()}));
        }
    }
    let final_pose = state(&r.sim, parcel)?.position;
    let metrics = json!({"jaw_control_enabled":close_jaws,"source_loaded_ticks_of_50":start_ticks,
        "closed_opposed_ticks_of_100":grasp_ticks,"carry_opposed_ticks_of_850":carry_ticks,
        "carry_station_impulse_ns":carry_support_impulse,"floor_impulse_ns":floor_impulse,"min_carry_height_m":carry_min_z,
        "peak_payload_height_m":max_z,"max_relative_motion_during_transfer_m":carry_slip,
        "position_at_open_m":position_at_open.to_array(),"first_tick_both_jaws_unloaded":first_release,"unsupported_release_ticks":free_flight_ticks,"first_destination_contact_after_open_tick":landing,
        "final_position_m":final_pose,"destination_loaded_ticks_of_300":settled_ticks,
        "final_jaw_impulse_ns":final_jaw_impulse,"max_settled_speed_m_s":max_final_speed,"max_settled_spin_rad_s":max_final_spin});
    let destination = DVec3::new(if grasp { STROKE } else { 0. }, 0., 0.17);
    let common = start_ticks >= 45
        && floor_impulse == 0.
        && DVec3::from(final_pose).distance(destination) < 0.005
        && max_final_speed < 0.005
        && max_final_spin < 0.02;
    let passed = if grasp {
        grasp_ticks >= 95
            && carry_ticks >= 800
            && carry_support_impulse == 0.
            && carry_min_z > 0.31
            && carry_slip < 0.005
            && position_at_open.z > 0.18
            && first_release.zip(landing).is_some_and(|(a, b)| a < b)
            && free_flight_ticks >= 20
            && settled_ticks >= 285
            && final_jaw_impulse == 0.
    } else {
        max_z < 0.172
            && (if close_jaws {
                grasp_ticks >= 95
            } else {
                grasp_ticks == 0
            })
            && carry_ticks == 0
            && settled_ticks == 0
    };
    if !(common && passed) {
        return Err(format!("gripper cycle failed: {metrics}").into());
    }
    Ok((metrics, trace))
}

pub(super) fn run(output: &Path) -> Result<()> {
    let mut r = Trial::Grasp.recording()?;
    let (metrics, trace) = cycle(&mut r, Trial::Grasp, true)?;
    let (open, _) = cycle(&mut Trial::OpenJaws.recording()?, Trial::OpenJaws, false)?;
    let (smooth, _) = cycle(
        &mut Trial::Frictionless.recording()?,
        Trial::Frictionless,
        false,
    )?;
    gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
    let (repeat, repeat_trace) = cycle(&mut r, Trial::Grasp, false)?;
    if metrics != repeat || trace != repeat_trace {
        return Err("loaded reset changed gripper cycle".into());
    }
    // Keep the timings and archive for the recorded first cycle only.
    r.costs_ns.truncate(END);
    write_json(
        &output.join("task.json"),
        &json!({"grasp":metrics,"open_jaw_control":open,"zero_friction_control":smooth,"loaded_reset_exact":true}),
    )?;
    write_json(&output.join("trajectory.json"), &json!(trace))?;
    r.save(output)?;
    write_json(
        &output.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,
        "captures":{"grasp":14,"lift":28,"carry":37,"release":60,"placed":82}}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn free_parcel_requires_opposed_contact_for_transfer() -> Result<()> {
        for trial in [Trial::Grasp, Trial::OpenJaws, Trial::Frictionless] {
            cycle(&mut trial.recording()?, trial, false)?;
        }
        Ok(())
    }
}
