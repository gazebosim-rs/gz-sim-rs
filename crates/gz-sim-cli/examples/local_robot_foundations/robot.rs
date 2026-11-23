// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{JointSample, set_joint_force};

const SCENE: &str = include_str!("../../../../examples/local_robot_parcel.sdf");
const WORLD: &str = "default";
const END: usize = 4500;

fn sample(sim: &Simulation, slider: u64) -> Result<gz_physics::JointState> {
    let entity = entity_by_id(sim.world(), slider).ok_or("missing slider")?;
    Ok(sim
        .world()
        .get::<JointSample>(entity)
        .ok_or("missing slider sample")?
        .state)
}
/// Smooth 1 s ramps with a pause for the load to settle at each end.
fn target(time: f64, travel: f64) -> (f64, f64) {
    let ramp = |x: f64| (x * x * (3. - 2. * x), 6. * x * (1. - x));
    if time < 1. {
        let (x, v) = ramp(time);
        (travel * x, travel * v)
    } else if time < 2. {
        (travel, 0.)
    } else if time < 3. {
        let (x, v) = ramp(time - 2.);
        (travel * (1. - x), -travel * v)
    } else {
        (0., 0.)
    }
}
fn drive(r: &mut Recording, travel: f64) -> Result<(Value, Vec<Value>)> {
    let slider = entity_id(&r.sim, "default::robot::slide")?;
    let carriage = entity_id(&r.sim, "default::robot::carriage")?;
    let tool = entity_id(&r.sim, "default::robot::tool")?;
    let parcel = entity_id(&r.sim, "default::parcel::box")?;
    let handle = |id| -> Result<_> {
        Ok(r.sim
            .world()
            .get::<BodyHandle>(entity_by_id(r.sim.world(), id).ok_or("missing entity")?)
            .ok_or("missing body")?
            .id())
    };
    let contact_pair = [handle(tool)?, handle(parcel)?];
    let start_x = state(&r.sim, parcel)?.position[0];
    let dt = r.sim.step_duration().as_secs_f64();
    let mut max_anchor: f64 = 0.;
    let mut max_track: f64 = 0.;
    let mut max_swing: f64 = 0.;
    let mut max_position: f64 = 0.;
    let mut at_goal: f64 = 0.;
    let mut home_hold_error: f64 = 0.;
    let mut first_touch = None;
    let mut total_contact = 0.;
    let mut trajectory = vec![];
    r.capture()?;
    for i in 1..=END {
        let t = (i - 1) as f64 * dt;
        let (position, velocity) = target(t, travel);
        let prior = sample(&r.sim, slider)?;
        let thrust = (160. * (position - prior.position) + 28. * (velocity - prior.velocity))
            .clamp(-22., 22.);
        set_joint_force(&mut r.sim, slider, thrust)?;
        r.advance()?;
        let now = i as f64 * dt;
        let p = sample(&r.sim, slider)?.position;
        let c = state(&r.sim, carriage)?;
        let w = state(&r.sim, tool)?;
        let point = |s: BodyState, local: DVec3| {
            DVec3::from(s.position) + DQuat::from_array(s.orientation) * local
        };
        let gap = point(c, -0.08 * DVec3::Z).distance(point(w, 0.23 * DVec3::Z));
        let swing = DVec3::Z.angle_between(DQuat::from_array(w.orientation) * DVec3::Z);
        let backend = r
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local physics")?;
        let impulse: f64 = backend
            .rigid_contact_observations()
            .iter()
            .filter(|c| [c.a, c.b] == contact_pair || [c.b, c.a] == contact_pair)
            .map(|c| c.normal_impulse)
            .sum();
        if impulse > 0. {
            first_touch.get_or_insert(now);
            total_contact += impulse;
        }
        max_anchor = max_anchor.max(gap);
        max_swing = max_swing.max(swing);
        max_position = max_position.max(p.abs());
        max_track = max_track.max((p - target(now, travel).0).abs());
        if now >= 4.25 {
            home_hold_error = home_hold_error.max(p.abs());
        }
        if i == 2000 {
            at_goal = p;
        }
        if i % 50 == 0 || i == 1 {
            r.capture()?;
            trajectory.push(json!({"time_s":now,"target_m":target(now, travel).0,"slider_m":p,"force_n":thrust,"wrist_swing_rad":swing,
                "anchor_error_m":gap,"parcel_x_m":state(&r.sim,parcel)?.position[0],"tool_parcel_impulse_ns":impulse}));
        }
    }
    let home = sample(&r.sim, slider)?.position;
    let parcel_state = state(&r.sim, parcel)?;
    let pushed = parcel_state.position[0] - start_x;
    let metrics = json!({"max_final_quarter_second_home_error_m":home_hold_error,"travel_target_m":travel,"outbound_settled_m":at_goal,"return_m":home,"parcel_moved_m":pushed,"parcel_final_z_m":parcel_state.position[2],
        "first_tool_parcel_contact_s":first_touch,"summed_tool_parcel_normal_impulse_ns":total_contact,"max_anchor_gap_m":max_anchor,
        "max_joint_excursion_m":max_position,"max_tracking_error_m":max_track,"max_wrist_swing_rad":max_swing});
    if (at_goal - travel).abs() > 0.006
        || home_hold_error > 0.006
        || first_touch.is_none()
        || pushed < 0.08
        || max_anchor > 0.001
        || max_position > 0.4
        || max_swing < 0.03
        || (parcel_state.position[2] - 0.38).abs() > 0.01
    {
        return Err(format!("parcel task failed: {metrics}").into());
    }
    Ok((metrics, trajectory))
}
pub(super) fn run(output: &Path, travel: f64) -> Result<()> {
    if !(0.0..=0.40).contains(&travel) {
        return Err("parcel travel must be between 0 and 0.40 m".into());
    }
    let mut r = Recording::new(SCENE, "local_robot_parcel.sdf", WORLD)?;
    let (metrics, trajectory) = drive(&mut r, travel)?;
    fs::create_dir_all(output)?;
    write_json(&output.join("task.json"), &metrics)?;
    write_json(&output.join("trajectory.json"), &json!(trajectory))?;
    r.save(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn loaded_reset_repeats_the_rail_robot_parcel_push() -> Result<()> {
        let mut run = Recording::new(SCENE, "local_robot_parcel.sdf", WORLD)?;
        let first = drive(&mut run, 0.30)?;
        gz_sim_scene::reset_loaded_world(&mut run.sim, |_| ())?;
        run.frames.clear();
        run.costs_ns.clear();
        assert_eq!(first, drive(&mut run, 0.30)?);
        Ok(())
    }
}
