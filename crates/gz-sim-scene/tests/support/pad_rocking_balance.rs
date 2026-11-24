// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{Rig, near, pad_rocking::pad_impulses};
use glam::{DQuat, DVec3};
use gz_sim_core::EntityId;
use gz_sim_scene::queue_link_world_wrench;

const ROCKING_WORLD: &str = include_str!("../../../../examples/worlds/local_rocking_pad_grasp.sdf");
const GRIP: f64 = 3.0;
const STIFFNESS: f64 = 30_000.0;
const HALF_WIDTH: f64 = 0.015;
const HALF_HEIGHT: f64 = 0.012;

/// Hold the parcel at a known pitch while both force-driven fingers find equilibrium.
/// The two vertical levels on a face each carry half its spring stiffness. When
/// one level lifts, it carries zero load and the supporting level compresses further.
#[test]
fn stationary_tilt_matches_geometric_grip_moment_and_stroke_through_corner_lift() {
    for degrees in [0.0_f64, 0.25, -0.25, 0.47, 0.49, 3.0, -3.0, 14.0, -14.0] {
        let pitch = degrees.to_radians();
        let world = ROCKING_WORLD
            .replace(
                "<model name=\"parcel\">",
                &format!(
                    "<model name=\"parcel\"><static>true</static><pose>0 0 0.30 0 {pitch} 0</pose>"
                ),
            )
            .replace(
                "<link name=\"body\">\n        <pose>0 0 0.30 0 0 0</pose>",
                "<link name=\"body\">",
            );
        let mut rig = Rig::new(&world, 100, 0.0);
        rig.drive([GRIP; 2]);
        rig.sim.step_exact(6_000).unwrap();

        let patches = pad_impulses(&rig);
        let center = DVec3::from(rig.state().position);
        let dt = rig.sim.step_duration().as_secs_f64();
        // The fixed parcel's anchors are absolute. Shift their moment from the
        // world origin to its center before comparing with the geometric law.
        let torque: f64 = patches
            .iter()
            .map(|p| (p.angular - center.cross(p.linear)).y / dt)
            .sum();
        let load = patches.map(|p| p.normal / dt);
        let loaded_points = patches.map(|p| p.loaded);

        // The high corner's overlap exceeds the low corner's by 2 h |sin(theta)|.
        let height_shift = HALF_HEIGHT * pitch.sin().abs();
        let high_load = (0.5 * (GRIP + STIFFNESS * height_shift)).min(GRIP);
        let low_load = GRIP - high_load;
        let corner_bias = (high_load - low_load) * pitch.signum();
        let reference_moment =
            2.0 * (GRIP * HALF_WIDTH * pitch.sin() - corner_bias * HALF_HEIGHT * pitch.cos());
        let tip_depth = 2.0 * high_load / STIFFNESS;
        let reference_travel = HALF_WIDTH * (1.0 - pitch.cos()) - height_shift + tip_depth;
        println!(
            "fixed_tilt_deg={degrees} points={loaded_points:?} measured_torque_nm={torque:.9} reference_torque_nm={reference_moment:.9} travel_m={:?} reference_travel_m={reference_travel:.9} normal_n={load:?}",
            rig.travel()
        );
        for side in 0..2 {
            near(load[side], GRIP, 1e-5);
            assert_eq!(loaded_points[side], if low_load > 0.0 { 4 } else { 2 });
            near(rig.travel()[side], reference_travel, 1e-7);
        }
        near(torque, reference_moment, 1e-6);
    }
}

/// The tall parcel rocks far enough that its two *unloaded* corners leave the
/// collision prediction range. The two compressed corners stay in contact.
#[test]
fn removing_and_reinstating_unloaded_corners_preserves_edge_force_and_torque() {
    const WORLD: &str =
        include_str!("../../../../examples/worlds/local_tall_rocking_pad_grasp.sdf");
    for step_us in [100, 25] {
        let mut rig = Rig::new(WORLD, step_us, 0.0);
        rig.drive([GRIP; 2]);
        rig.sim.step_exact(200_000 / u64::from(step_us)).unwrap();
        let id = rig
            .sim
            .world()
            .get::<EntityId>(super::named(&rig.sim, "default::parcel::body"))
            .unwrap()
            .get();
        let mut previous = pad_impulses(&rig);
        let dt = rig.sim.step_duration().as_secs_f64();
        let mut transitions = [Vec::new(), Vec::new()];
        for tick in 0..250_000 / step_us {
            if tick * step_us < 200 {
                queue_link_world_wrench(&mut rig.sim, id, [0.0; 3], [0.0, 75.0, 0.0]).unwrap();
            }
            rig.sim.step_exact(1).unwrap();
            let current = pad_impulses(&rig);
            let height_axis = DQuat::from_array(rig.state().orientation) * DVec3::Z;
            // Gap difference between the two corner levels along the pad normal.
            let near_prediction_boundary = (0.019..0.021).contains(&(0.160 * height_axis.x.abs()));
            for side in 0..2 {
                let before = previous[side];
                let after = current[side];
                let (old_rows, rows) = (before.selected, after.selected);
                let force_jump = (after.normal - before.normal).abs() / dt;
                let torque_jump = (after.angular.y - before.angular.y).abs() / dt;
                if near_prediction_boundary {
                    // Check every tick around the cutoff, including anchor refreshes
                    // that can precede a change in the number of selected rows.
                    assert_eq!((before.loaded, after.loaded), (2, 2));
                    assert!(force_jump < 10.0 * dt && torque_jump < 0.8 * dt);
                }
                if old_rows != rows {
                    assert!(near_prediction_boundary);
                    println!(
                        "prediction_boundary step_us={step_us} side={side} time_after_kick_us={} rows={old_rows}->{rows} force_change_n={force_jump:.9} torque_change_nm={torque_jump:.9}",
                        (tick + 1) * step_us
                    );
                    transitions[side].push((old_rows, rows));
                }
            }
            previous = current;
        }
        for events in transitions {
            assert_eq!(events, [(4, 2), (2, 4)]);
        }
    }
}
