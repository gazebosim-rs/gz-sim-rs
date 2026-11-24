// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Mixed robot joints through public Bevy state, SDF frames, and actuator commands.
#![allow(deprecated)]
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BallJointSample, JointSample, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    queue_joint_effort, reset_loaded_world, set_joint_force,
};
use sdformat::NoResources;

const XML: &str = include_str!("../../../examples/worlds/local_robot_joints.sdf");

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "robot.sdf",
        "robot_joints",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, local: &str) -> u64 {
    let scoped = format!("robot_joints::robot::{local}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world()
        .get(entity_by_id(sim.world(), id(sim, name)).unwrap())
        .unwrap()
}

#[test]
fn slider_stops_ball_anchor_and_full_reset_survive_reversed_load() {
    let mut sim = load(XML);
    let slide = id(&sim, "slide");
    let ball = id(&sim, "wrist");
    assert!(set_joint_force(&mut sim, ball, 1.).is_err());
    for _ in 0..2 {
        let mut max_gap = 0.0_f64;
        let mut max_swing = 0.0_f64;
        let mut max_cross_axis = 0.0_f64;
        let slider = |sim: &Simulation| {
            sim.world()
                .get::<JointSample>(entity_by_id(sim.world(), slide).unwrap())
                .unwrap()
                .state
        };
        assert_eq!(slider(&sim).position, 0.);
        for sign in [1., -1.] {
            set_joint_force(&mut sim, slide, 12. * sign).unwrap();
            for _ in 0..750 {
                sim.step_exact(1).unwrap();
                let parent = pose(&sim, "carriage");
                let tool = pose(&sim, "tool");
                let cart = DVec3::from(parent.translation);
                let first = cart + DQuat::from_array(parent.rotation) * DVec3::new(0., 0., -0.08);
                let second = DVec3::from(tool.translation)
                    + DQuat::from_array(tool.rotation) * DVec3::new(0., 0., 0.23);
                max_gap = max_gap.max(first.distance(second));
                max_cross_axis = max_cross_axis.max(cart.y.abs()).max((cart.z - 0.84).abs());
                let bs = sim
                    .world()
                    .get::<BallJointSample>(entity_by_id(sim.world(), ball).unwrap())
                    .unwrap();
                max_swing = max_swing
                    .max(DQuat::from_array(bs.state.orientation).angle_between(DQuat::IDENTITY));
                assert!((cart.x - slider(&sim).position).abs() < 1e-8);
                assert!(slider(&sim).position.abs() < 0.403);
            }
            assert!((slider(&sim).position - sign * 0.40).abs() < 1e-4);
        }
        eprintln!(
            "upper/lower stops ±0.4 m, peak spherical anchor gap {max_gap} m, off-axis displacement {max_cross_axis} m, peak swing {max_swing} rad"
        );
        assert!(max_gap < 0.001 && max_cross_axis < 0.0001 && max_swing > 0.25);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
}

#[test]
fn rotated_slider_moves_one_meter_per_second_under_known_impulse() {
    let mut xml = XML.replace("<gravity>0 0 -9.81</gravity>", "<gravity>0 0 0</gravity>");
    let a = xml.find("<link name=\"tool\">").unwrap();
    let b = xml[a..].find("</link>").unwrap() + a + 7;
    xml.replace_range(a..b, "");
    let a = xml.find("<joint name=\"wrist\"").unwrap();
    let b = xml[a..].find("</joint>").unwrap() + a + 8;
    xml.replace_range(a..b, "");
    xml = xml.replace("<xyz>1 0 0</xyz>", "<xyz>0 1 0</xyz>");
    let mut sim = load(&xml);
    let slider = id(&sim, "slide");
    queue_joint_effort(&mut sim, slider, 1000.).unwrap();
    sim.step_exact(100).unwrap();
    let state = sim
        .world()
        .get::<JointSample>(entity_by_id(sim.world(), slider).unwrap())
        .unwrap()
        .state;
    let cart = pose(&sim, "carriage");
    assert!((state.velocity - 1.).abs() < 1e-8 && (cart.translation[1] - 0.1).abs() < 0.001);
    assert!(
        (cart.translation[1] - state.position).abs() < 1e-9 && cart.translation[0].abs() < 1e-8
    );
}

#[test]
fn spherical_joint_accepts_rotation_about_all_three_axes() {
    let mut xml = XML.replace("<gravity>0 0 -9.81</gravity>", "<gravity>0 0 0</gravity>");
    let start = xml.find("<link name=\"carriage\">").unwrap();
    let stop = xml[start..].find("</link>").unwrap() + start + 7;
    xml.replace_range(start..stop, "");
    let start = xml.find("<joint name=\"slide\"").unwrap();
    let stop = xml[start..].find("</joint>").unwrap() + start + 8;
    xml.replace_range(start..stop, "");
    xml = xml
        .replace("<parent>carriage</parent>", "<parent>world</parent>")
        .replace("<pose>0 0 0.23 0 0 0</pose>", "<pose>0 0 0 0 0 0</pose>");
    for (axis, inertia) in [
        (DVec3::X, 0.00554666666667),
        (DVec3::Y, 0.00554666666667),
        (DVec3::Z, 0.00042666666667),
    ] {
        let mut sim = load(&xml);
        let tool = id(&sim, "tool");
        let wrist = id(&sim, "wrist");
        gz_sim_scene::queue_link_world_wrench(
            &mut sim,
            tool,
            [0.; 3],
            (axis * (inertia * 1000.)).into(),
        )
        .unwrap();
        sim.step_exact(100).unwrap();
        let state = sim
            .world()
            .get::<BallJointSample>(entity_by_id(sim.world(), wrist).unwrap())
            .unwrap()
            .state;
        assert!(DVec3::from_array(state.angular_velocity).distance(axis) < 1e-9);
        assert!(
            DQuat::from_array(state.orientation).angle_between(DQuat::from_axis_angle(axis, 0.1))
                < 0.001
        );
        assert!(
            DVec3::from(pose(&sim, "tool").translation).distance(DVec3::new(0., 0., 0.53)) < 1e-10
        );
    }
}

#[test]
fn driving_a_floating_slider_keeps_total_linear_and_angular_momentum() {
    let mut xml = XML.replace("<gravity>0 0 -9.81</gravity>", "<gravity>0 0 0</gravity>");
    // Free the coupled pair from every external body so the commanded impulse is internal.
    let start = xml.find("<model name=\"rail\">").unwrap();
    let stop = xml[start..].find("</model>").unwrap() + start + 8;
    xml.replace_range(start..stop, "");
    let start = xml.find("<joint name=\"slide\"").unwrap();
    let stop = xml[start..].find("</joint>").unwrap() + start + 8;
    xml.replace_range(start..stop, "");
    xml = xml
        .replace(
            "<joint name=\"wrist\" type=\"ball\">",
            "<joint name=\"wrist\" type=\"prismatic\">",
        )
        .replace(
            "<pose>0 0 0.23 0 0 0</pose></joint>",
            "<pose>0 0 0.23 0 0 0</pose><axis><xyz>1 0 0</xyz></axis></joint>",
        );
    let mut previous_drift = f64::INFINITY;
    for scale in [1, 2, 4] {
        let scene = xml.replace(
            "<max_step_size>0.001</max_step_size>",
            &format!("<max_step_size>{}</max_step_size>", 0.001 / scale as f64),
        );
        let mut sim = load(&scene);
        let slide = id(&sim, "wrist");
        set_joint_force(&mut sim, slide, 2.).unwrap();
        sim.step_exact(100 * scale).unwrap();
        let mut linear = DVec3::ZERO;
        let mut angular = DVec3::ZERO;
        for (name, mass, inertia) in [
            (
                "tool",
                0.4,
                DVec3::new(0.00554666666667, 0.00554666666667, 0.00042666666667),
            ),
            (
                "carriage",
                1.,
                DVec3::new(0.00213333333333, 0.00276666666667, 0.00276666666667),
            ),
        ] {
            let entity = entity_by_id(sim.world(), id(&sim, name)).unwrap();
            let velocity = sim.world().get::<gz_sim_scene::Velocity>(entity).unwrap();
            let body = pose(&sim, name);
            let rotation = DQuat::from_array(body.rotation);
            let momentum = mass * DVec3::from_array(velocity.linear);
            linear += momentum;
            angular += DVec3::from(body.translation).cross(momentum)
                + rotation * (inertia * (rotation.inverse() * DVec3::from(velocity.angular)));
        }
        let q = sim
            .world()
            .get::<JointSample>(entity_by_id(sim.world(), slide).unwrap())
            .unwrap()
            .state;
        assert!(q.position > 0.005 && q.velocity > 0.);
        let drift = angular.length();
        eprintln!(
            "free slider dt={} ms, momentum residual {} kg m/s, angular residual {drift} N m s",
            1. / scale as f64,
            linear.length()
        );
        assert!(
            linear.length() < 1e-9 && drift < 4e-5 && drift < 0.6 * previous_drift,
            "P={linear:?}, H={angular:?}"
        );
        previous_drift = drift;
    }
    assert!(previous_drift < 1e-5);
}

#[test]
fn slider_velocity_motor_acts_along_the_linear_coordinate_and_rejects_ball_commands() {
    use gz_physics::{JointVelocityMotor, PhysicsBackend};
    use gz_sim_scene::JointHandle;
    let mut sim = load(XML);
    let lookup = |name| {
        sim.world()
            .get::<JointHandle>(entity_by_id(sim.world(), id(&sim, name)).unwrap())
            .unwrap()
            .id()
    };
    let slide = lookup("slide");
    let ball = lookup("wrist");
    let command = Some(JointVelocityMotor {
        target_velocity: 0.25,
        max_force: 8.,
    });
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    assert!(backend.set_joint_velocity_motor(ball, command).is_err());
    backend.set_joint_velocity_motor(slide, command).unwrap();
    sim.step_exact(250).unwrap();
    let state = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(slide)
        .unwrap();
    // The default velocity motor approaches its target with finite gain.
    assert!((0.02..0.25).contains(&state.velocity) && (0.001..0.0625).contains(&state.position));
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(slide, command)
        .unwrap();
    sim.step_exact(250).unwrap();
    let state = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_state(slide)
        .unwrap();
    assert!(
        (state.velocity - 0.25).abs() < 0.001 && state.position > 0.055,
        "velocity servo {state:?}"
    );
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_motor(slide, None)
        .unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(100).unwrap();
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(slide)
            .unwrap()
            .position
            .abs()
            < 1e-8
    );
}

#[test]
fn one_sided_slider_travel_has_a_stop_only_at_the_authored_end() {
    for (missing, sign) in [("<lower>-0.40</lower>", 1.), ("<upper>0.40</upper>", -1.)] {
        let mut sim = load(&XML.replace(missing, ""));
        let slide = id(&sim, "slide");
        let q = |s: &Simulation| {
            s.world()
                .get::<JointSample>(entity_by_id(s.world(), slide).unwrap())
                .unwrap()
                .state
                .position
        };
        set_joint_force(&mut sim, slide, 15. * sign).unwrap();
        sim.step_exact(700).unwrap();
        assert!((q(&sim) - sign * 0.4).abs() < 0.0001);
        set_joint_force(&mut sim, slide, -15. * sign).unwrap();
        sim.step_exact(900).unwrap();
        assert!(q(&sim) * sign < -0.6);
    }
}
