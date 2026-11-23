// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    Name, PhysicsEngine, UniversalJointSample, WorldPose, load_world_with_sdf_step,
    queue_link_world_wrench, queue_universal_joint_effort, reset_loaded_world,
};
use sdformat::NoResources;
const XML: &str = include_str!("../../../examples/worlds/local_robot_universal.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "universal.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    let name = format!("default::robot::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn sample(sim: &Simulation, joint: u64) -> UniversalJointSample {
    *sim.world()
        .get(entity_by_id(sim.world(), joint).unwrap())
        .unwrap()
}
fn pose(sim: &Simulation, id: u64) -> WorldPose {
    *sim.world()
        .get(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
}
fn centered() -> String {
    XML.replace("0 0 -9.81", "0 0 0")
        .replace("<pose>0 0 0.31 0 0 0</pose>", "<pose>0 0 0 0 0 0</pose>")
        .replace("<izz>0.003</izz>", "<izz>0.03</izz>")
}

#[test]
fn orthogonal_impulses_and_forbidden_torque_in_rotated_frame() {
    let xml = centered();
    for (effort, expected) in [
        ([30., 0.], [1., 0.]),
        ([0., 30.], [0., 1.]),
        ([0., 0.], [0., 0.]),
    ] {
        let mut sim = load(&xml);
        let joint = id(&sim, "wrist");
        let tool = id(&sim, "tool");
        let initial = pose(&sim, tool);
        let frame = DQuat::from_array(initial.rotation);
        queue_universal_joint_effort(&mut sim, joint, effort).unwrap();
        if effort == [0.; 2] {
            queue_link_world_wrench(&mut sim, tool, [0.; 3], (frame * DVec3::Z * 30.).into())
                .unwrap();
        }
        assert!(queue_universal_joint_effort(&mut sim, joint, [1., f64::NAN]).is_err());
        sim.step_exact(100).unwrap();
        let s = sample(&sim, joint).state;
        let observed = DQuat::from_array(pose(&sim, tool).rotation);
        // One millisecond of uniform acceleration, then 99 ms of coasting.
        let equivalent_time = 0.0005 + 0.099;
        let wanted = frame
            * DQuat::from_rotation_x(equivalent_time * expected[0])
            * DQuat::from_rotation_y(equivalent_time * expected[1]);
        eprintln!(
            "applied {effort:?}, observed {s:?}, rotation error {} rad",
            observed.angle_between(wanted)
        );
        for (i, speed) in expected.into_iter().enumerate() {
            assert!((s.velocity[i] - speed).abs() < 1e-7);
            assert!((s.position[i] - equivalent_time * speed).abs() < 0.0001);
        }
        assert!(observed.angle_between(wanted) < 0.0001);
    }
}

#[test]
fn independently_loaded_stops_and_reset_replay() {
    let mut sim = load(XML);
    let joint = id(&sim, "wrist");
    let tool = id(&sim, "tool");
    let base = pose(&sim, tool);
    let frame = DQuat::from_array(base.rotation);
    let anchor = DVec3::from(base.translation) + frame * DVec3::Z * 0.31;
    let mut previous = None;
    for _ in 0..2 {
        assert_eq!(sample(&sim, joint).state.position, [0.; 2]);
        let mut max_gap = 0f64;
        let mut max_twist = 0f64;
        for (loads, goal) in [([6., -6.], [0.55, -0.45]), ([-6., 6.], [-0.8, 0.75])] {
            for _ in 0..1200 {
                queue_universal_joint_effort(&mut sim, joint, loads).unwrap();
                sim.step_exact(1).unwrap();
                let p = pose(&sim, tool);
                let r = DQuat::from_array(p.rotation);
                max_gap =
                    max_gap.max(anchor.distance(DVec3::from(p.translation) + r * DVec3::Z * 0.31));
                max_twist = max_twist.max((frame * DVec3::X).dot(r * DVec3::Y).abs());
            }
            let s = sample(&sim, joint).state;
            eprintln!(
                "limits {goal:?}, angles {:?}, max pivot gap {max_gap} m, max |axis dot| {max_twist}",
                s.position
            );
            for (angle, stop) in s.position.into_iter().zip(goal) {
                assert!((angle - stop).abs() < 0.00001);
            }
        }
        // Require a two-micrometer pivot and one tenth of a degree of squareness
        // throughout both loaded impacts, including transient solver error.
        assert!(max_gap < 2e-6 && max_twist < 0.1_f64.to_radians().sin());
        let final_state = sample(&sim, joint).state;
        if let Some(previous) = previous {
            assert_eq!(final_state, previous);
        }
        previous = Some(final_state);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn missing_parallel_and_unsupported_axes_are_rejected() {
    for xml in [
        XML.replace("<xyz>0 1 0</xyz>", "<xyz>-1 0 0</xyz>"),
        XML.replace(
            "<axis2><xyz>0 1 0</xyz><limit><lower>-0.45</lower><upper>0.75</upper></limit></axis2>",
            "",
        ),
        XML.replace("<upper>0.75</upper>", "<upper>-1</upper>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn one_sided_multiturn_stop_and_zero_lock_survive_reversal() {
    let xml = centered()
        .replace(
            "<lower>-0.80</lower><upper>0.55</upper>",
            "<upper>7.0</upper>",
        )
        .replace(
            "<lower>-0.45</lower><upper>0.75</upper>",
            "<lower>0</lower><upper>0</upper><effort>2</effort>",
        );
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    assert!(gz_sim_scene::queue_joint_effort(&mut sim, joint, 1.).is_err());
    assert!(queue_universal_joint_effort(&mut sim, joint, [0., 3.]).is_err());
    let mut locked = 0.0f64;
    for _ in 0..1400 {
        queue_universal_joint_effort(&mut sim, joint, [1., 1.]).unwrap();
        sim.step_exact(1).unwrap();
        let s = sample(&sim, joint).state;
        locked = locked.max(s.position[1].abs());
        assert!(s.position[0] <= 7.01);
    }
    let stop = sample(&sim, joint).state;
    assert!((stop.position[0] - 7.).abs() < 1e-4 && stop.velocity[0].abs() < 1e-5);
    for _ in 0..1100 {
        queue_universal_joint_effort(&mut sim, joint, [-1., -1.]).unwrap();
        sim.step_exact(1).unwrap();
        locked = locked.max(sample(&sim, joint).state.position[1].abs());
    }
    let released = sample(&sim, joint).state;
    eprintln!(
        "one-sided stop {:.8} rad, reverse {:.5} rad, max locked-axis motion {:.3e} rad",
        stop.position[0], released.position[0], locked
    );
    assert!(released.position[0] < -7. && locked < 1e-5);
}

#[test]
fn internal_torques_conserve_momentum_between_two_rotating_bodies() {
    let xml = centered().replace("<parent>world</parent>", "<parent>base</parent>").replace("<link name=\"tool\">", r#"<link name="base"><pose>0 0 0.70 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.03</ixx><iyy>0.03</iyy><izz>0.03</izz></inertia></inertial>
        <collision name="base"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>
        </link><link name="tool">"#);
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    let base = id(&sim, "base");
    let tool = id(&sim, "tool");
    let initial = DQuat::from_array(pose(&sim, tool).rotation);
    queue_universal_joint_effort(&mut sim, joint, [30., 0.]).unwrap();
    sim.step_exact(80).unwrap();
    assert!((sample(&sim, joint).state.velocity[0] - 2.).abs() < 1e-7);
    queue_universal_joint_effort(&mut sim, joint, [0., 30.]).unwrap();
    let mut peak_momentum = 0f64;
    let mut peak_orientation = 0f64;
    for _ in 0..80 {
        sim.step_exact(1).unwrap();
        let get_omega = |body| {
            DVec3::from(
                sim.world()
                    .get::<gz_sim_scene::Velocity>(entity_by_id(sim.world(), body).unwrap())
                    .unwrap()
                    .angular,
            )
        };
        peak_momentum = peak_momentum.max((0.03 * (get_omega(base) + get_omega(tool))).length());
        let q = sample(&sim, joint).state.position;
        let parent = DQuat::from_array(pose(&sim, base).rotation) * initial;
        let child = DQuat::from_array(pose(&sim, tool).rotation);
        peak_orientation = peak_orientation.max(
            (parent * DQuat::from_rotation_x(q[0]) * DQuat::from_rotation_y(q[1]))
                .angle_between(child),
        );
    }
    let state = sample(&sim, joint).state;
    eprintln!(
        "floating axes {:?} rad, max net angular momentum {peak_momentum:.3e} N m s, max orientation residual {peak_orientation:.3e} rad",
        state.position
    );
    assert!(
        state.position[0] > 0.2
            && state.position[1] > 0.1
            && peak_momentum < 1e-9
            && peak_orientation < 0.001
    );
}

#[test]
fn revolute2_resolves_axes_in_different_named_frames() {
    let named_frame = r#"<model name="robot"><frame name="other" attached_to="tool">
        <pose>0 0 0 0 0 1.5707963267948966</pose></frame>"#;
    let xml = centered()
        .replace(r#"type="universal""#, r#"type="revolute2""#)
        .replace(r#"<model name="robot">"#, named_frame)
        .replace(
            "<xyz>0 1 0</xyz>",
            r#"<xyz expressed_in="other">1 0 0</xyz>"#,
        );
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    let tool = id(&sim, "tool");
    let initial = DQuat::from_array(pose(&sim, tool).rotation);
    queue_universal_joint_effort(&mut sim, joint, [0., 30.]).unwrap();
    sim.step_exact(100).unwrap();
    let actual = DQuat::from_array(pose(&sim, tool).rotation);
    assert!(actual.angle_between(initial * DQuat::from_rotation_y(0.0995)) < 0.0001);
}

#[test]
fn second_axis_lower_stop_and_queued_torques_follow_reset() {
    let xml = centered().replace(
        "<lower>-0.45</lower><upper>0.75</upper>",
        "<lower>-6.5</lower><effort>3</effort>",
    );
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    queue_universal_joint_effort(&mut sim, joint, [0., -2.]).unwrap();
    assert!(queue_universal_joint_effort(&mut sim, joint, [0., -2.]).is_err());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        sample(&sim, joint)
            .state
            .velocity
            .iter()
            .all(|w| w.abs() < 1e-8)
    );
    for _ in 0..900 {
        queue_universal_joint_effort(&mut sim, joint, [0., -2.]).unwrap();
        sim.step_exact(1).unwrap();
        assert!(sample(&sim, joint).state.position[1] >= -6.51);
    }
    let stopped = sample(&sim, joint).state;
    assert!((stopped.position[1] + 6.5).abs() < 1e-4 && stopped.velocity[1].abs() < 1e-5);
}

const SPRINGS_XML: &str =
    include_str!("../../../examples/worlds/local_robot_universal_springs.sdf");

#[test]
fn rotated_wrist_balances_two_springs_against_gravity_and_external_torque() {
    // Isolate the wrist and gravity for the torque balance. Folding the arm
    // against the drawn stand would add an external contact reaction.
    let mut xml = SPRINGS_XML.to_owned();
    xml.replace_range(
        xml.find("<model name=\"stand\">").unwrap()..xml.find("<model name=\"robot\">").unwrap(),
        "",
    );
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    let tool = id(&sim, "tool");
    let initial = DQuat::from_array(pose(&sim, tool).rotation);
    let mut final_state = None;
    for _ in 0..2 {
        for loads in [[0., 0.], [2.1, -1.4]] {
            for _ in 0..5000 {
                queue_universal_joint_effort(&mut sim, joint, loads).unwrap();
                sim.step_exact(1).unwrap();
            }
            let s = sample(&sim, joint).state;
            let frame = DQuat::from_array(pose(&sim, tool).rotation);
            let gravity_torque = (frame * (-0.31 * DVec3::Z)).cross(-9.81 * DVec3::Z);
            let residual = [
                5. * (1.15 - s.position[0]) + gravity_torque.dot(initial * DVec3::X) + loads[0],
                4. * (-0.85 - s.position[1]) + gravity_torque.dot(frame * DVec3::Y) + loads[1],
            ];
            eprintln!(
                "wrist loads={loads:?} angles={:?} rates={:?} torque residuals={residual:?}",
                s.position, s.velocity
            );
            assert!(s.velocity.into_iter().all(|v| v.abs() < 0.0005));
            assert!(residual.into_iter().all(|r| r.abs() < 0.001));
        }
        let state = sample(&sim, joint).state;
        if let Some(prior) = final_state {
            assert_eq!(state, prior);
        }
        final_state = Some(state);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn isolated_axes_follow_the_analytical_multiturn_oscillator_through_both_sdf_spellings() {
    let k: f64 = 0.7;
    let d: f64 = 0.13;
    let reference = -7.5;
    let decay = d / (2. * 0.03);
    let frequency = (k / 0.03 - decay.powi(2)).sqrt();
    let xml = centered()
        .replace("<limit><lower>-0.80</lower><upper>0.55</upper></limit>", "")
        .replace("<limit><lower>-0.45</lower><upper>0.75</upper></limit>", "");
    let law = format!(
        "<dynamics><spring_reference>{reference}</spring_reference><spring_stiffness>{k}</spring_stiffness><damping>{d}</damping></dynamics>"
    );
    for (axis, element, kind) in [(0, "</axis>", "universal"), (1, "</axis2>", "revolute2")] {
        let mut sim = load(
            &xml.replace("type=\"universal\"", &format!("type=\"{kind}\""))
                .replace(element, &format!("{law}{element}")),
        );
        let joint = id(&sim, "wrist");
        let mut peak = 0_f64;
        for tick in 1..=300 {
            sim.step_exact(10).unwrap();
            let t = tick as f64 / 100.;
            let expected = reference
                * (1.
                    - (-decay * t).exp()
                        * ((frequency * t).cos() + decay / frequency * (frequency * t).sin()));
            let state = sample(&sim, joint).state;
            peak = peak.max((state.position[axis] - expected).abs());
            assert!(state.position[1 - axis].abs() < 1e-7);
        }
        eprintln!("wrist axis {axis} peak analytical error {peak:.7} rad");
        assert!(peak < 0.02);
    }
}

#[test]
fn both_angular_speed_caps_hold_under_opposed_torques_and_release_on_reversal() {
    let xml = centered()
        .replace(
            "<lower>-0.80</lower><upper>0.55</upper>",
            "<velocity>0.8</velocity>",
        )
        .replace(
            "<lower>-0.45</lower><upper>0.75</upper>",
            "<velocity>1.3</velocity>",
        );
    let mut sim = load(&xml);
    let joint = id(&sim, "wrist");
    let mut replay = None;
    for _ in 0..2 {
        let mut peak = [0_f64; 2];
        for sign in [1., -1.] {
            for _ in 0..600 {
                queue_universal_joint_effort(&mut sim, joint, [sign * 1.2, -sign * 1.5]).unwrap();
                sim.step_exact(1).unwrap();
                let s = sample(&sim, joint).state;
                for (i, rate) in s.velocity.into_iter().enumerate() {
                    peak[i] = peak[i].max(rate.abs());
                }
            }
            let state = sample(&sim, joint).state;
            eprintln!(
                "capped wrist {sign:+} peaks {peak:?} rates {:?} q {:?}",
                state.velocity, state.position
            );
            for (rate, target) in state.velocity.into_iter().zip([sign * 0.8, -sign * 1.3]) {
                assert!((rate - target).abs() < 0.002);
            }
        }
        assert!(peak[0] < 0.802 && peak[1] < 1.302);
        let state = sample(&sim, joint).state;
        if let Some(prev) = replay {
            assert_eq!(state, prev);
        }
        replay = Some(state);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn a_one_tick_external_impulse_is_capped_and_the_other_axis_can_coast() {
    for (first_cap, expected) in [(0., 0.), (0.8, 0.8)] {
        let xml = centered()
            .replace(
                "<lower>-0.80</lower><upper>0.55</upper>",
                &format!("<velocity>{first_cap}</velocity>"),
            )
            .replace(
                "<lower>-0.45</lower><upper>0.75</upper>",
                "<velocity>1.3</velocity>",
            );
        let mut sim = load(&xml);
        let joint = id(&sim, "wrist");
        let tool = id(&sim, "tool");
        let frame = DQuat::from_array(pose(&sim, tool).rotation);
        queue_link_world_wrench(
            &mut sim,
            tool,
            [0.; 3],
            (frame * DVec3::new(90., 9., 0.)).into(),
        )
        .unwrap();
        sim.step_exact(1).unwrap();
        let impact = sample(&sim, joint).state;
        sim.step_exact(150).unwrap();
        let coast = sample(&sim, joint).state;
        eprintln!(
            "external pulse, cap {first_cap}, impact rates {:?}, coasting rates {:?}",
            impact.velocity, coast.velocity
        );
        for state in [impact, coast] {
            assert!((state.velocity[0] - expected).abs() < 0.0005);
            assert!((state.velocity[1] - 0.3).abs() < 0.0005);
        }
    }
}

#[test]
fn a_speed_limit_on_a_moving_parent_is_rejected_before_construction() {
    let xml=centered().replace("<model name=\"robot\">","<model name=\"robot\"><link name=\"base\"><inertial><mass>2</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name=\"shape\"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision></link>")
        .replace("<parent>world</parent><child>tool</child>","<parent>base</parent><child>tool</child>")
        .replace("<upper>0.75</upper>","<upper>0.75</upper><velocity>0.8</velocity>");
    let err = load_world_with_sdf_step(
        &xml,
        "unsupported.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .err()
    .unwrap();
    assert!(
        err.to_string()
            .contains("angular speed limit requires a world-mounted joint"),
        "{err}"
    );
}
