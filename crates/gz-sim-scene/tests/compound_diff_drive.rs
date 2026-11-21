// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The existing wheel-pair motors drive only a compound canonical chassis.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, JointState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, DiffDriveCommandError, DiffDriveConfiguration, DiffDriveSample, JointHandle,
    JointSample, LoadError, LocalPose, ModelState, Name, PhysicsEngine, WorldEditError, WorldPose,
    diff_drive_binding_matches, initial_diff_drive_configurations_with_identity,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_diff_drive_command, set_model_state,
};
use std::{
    collections::{BTreeMap, VecDeque},
    time::Duration,
};

const M8: &str = include_str!("../../../examples/m8_diff_drive.sdf");
const CHASSIS: &str = "<collision name=\"chassis\"><geometry><box><size>0.6 0.35 0.2</size></box></geometry></collision>";
const WHEEL: &str = "<collision name=\"wheel\"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision>";
const DRIVE: &str = "<plugin filename=\"gz-sim-diff-drive-system\"";

fn members(count: usize, density: f64, compact: bool) -> String {
    (0..count).map(|i| {
        if compact && i == 0 { return CHASSIS.into(); }
        let (position, geometry) = if compact && i == 1 {
            ("0.5 0 0.1".into(), "<sphere><radius>0.1</radius></sphere>")
        } else {
            let geometry = match i % 4 {
                0 => "<sphere><radius>0.5</radius></sphere>",
                1 => "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
                2 => "<box><size>1 0.5 1.5</size></box>",
                _ => "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
            };
            (format!("2 0 {}", i * 4), geometry)
        };
        format!("<collision name='c{i}'><pose>{position} 0 0 0.3</pose><density>{density}</density><geometry>{geometry}</geometry></collision>")
    }).collect()
}

fn robot(name: &str, count: usize, pose: &str, density: f64, compact: bool) -> String {
    let start = M8.find("<model name=\"robot\"").unwrap();
    let end = start + M8[start..].find("</model>").unwrap() + "</model>".len();
    M8[start..end]
        .replace("name=\"robot\"", &format!("name=\"{name}\""))
        .replace(
            "<pose>0 0 0.2 0 0 0</pose>",
            &format!("<pose>{pose}</pose>"),
        )
        .replace(CHASSIS, &members(count, density, compact))
        .replace("/model/robot/cmd_vel", &format!("/m91/{name}/cmd_vel"))
        .replace("/model/robot/odometry", &format!("/m91/{name}/odom"))
        .replace("robot/odom", &format!("{name}/odom"))
        .replace("robot/base_link", &format!("{name}/base"))
}
fn world(models: &str, gravity: &str, ground: bool) -> String {
    let floor = if ground {
        "<model name='ground'><static>true</static><pose>0 0 -0.1 0 0 0</pose><link name='body'><collision name='floor'><geometry><box><size>100 100 0.2</size></box></geometry></collision></link></model>"
    } else {
        ""
    };
    format!(
        "<sdf version='1.12'><world name='w'><gravity>{gravity}</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{floor}{models}</world></sdf>"
    )
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "compound-drive",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn handle(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id()
}
fn native(sim: &Simulation, name: &str) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(handle(sim, name))
        .unwrap()
}
fn wheel(sim: &Simulation, owner: &str, side: &str) -> JointState {
    let entity = named(sim, &format!("w::{owner}::{side}_wheel_joint"));
    sim.world()
        .resource::<PhysicsBackend>()
        .joint_state(sim.world().get::<JointHandle>(entity).unwrap().id())
        .unwrap()
}
fn command(sim: &mut Simulation, owner: &str, linear: f64, angular: f64) {
    let owner = id(sim, &format!("w::{owner}"));
    set_diff_drive_command(sim, owner, linear, angular).unwrap();
}
fn identities(sim: &Simulation) -> BTreeMap<u64, Entity> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| (id.get(), e.id())))
        .collect()
}
fn states(sim: &Simulation) -> BTreeMap<u64, BodyState> {
    sim.world()
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<EntityId>()?.get(),
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(e.get::<BodyHandle>()?.id())?,
            ))
        })
        .collect()
}
fn near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && expected.is_finite() && (actual - expected).abs() < 1e-11,
        "{actual} != {expected}"
    );
}
fn vector(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        near(a, b);
    }
}
fn ray(
    sim: &Simulation,
    origin: DVec3,
    direction: DVec3,
    max: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin.to_array(), direction.to_array(), max, exclude)
        .unwrap()
}

fn check_members(sim: &Simulation, owner: &str, count: usize, frames: bool) {
    let name = format!("w::{owner}::base_link");
    let body = native(sim, &name);
    let p = DVec3::from_array(body.position);
    let q = DQuat::from_array(body.orientation);
    let collision_q = q * DQuat::from_rotation_z(0.3);
    if frames {
        let frame = sim.world().get::<WorldPose>(named(sim, &name)).unwrap();
        vector(frame.translation, body.position);
        vector(
            (DQuat::from_array(frame.rotation) * DVec3::X).to_array(),
            (q * DVec3::X).to_array(),
        );
        vector(
            sim.world()
                .get::<WorldPose>(named(sim, &format!("w::{owner}")))
                .unwrap()
                .translation,
            body.position,
        );
    }
    for i in 0..count {
        let local = DVec3::new(2., 0., 4. * i as f64);
        let center = p + q * local;
        let entity = named(sim, &format!("{name}::c{i}"));
        let recorded = sim.world().get::<LocalPose>(entity).unwrap();
        vector(recorded.0.translation, local.to_array());
        if frames {
            let frame = sim.world().get::<WorldPose>(entity).unwrap();
            vector(frame.translation, center.to_array());
            vector(
                (DQuat::from_array(frame.rotation) * DVec3::Y).to_array(),
                (collision_q * DVec3::Y).to_array(),
            );
        }
        // Adjacent chassis members occupy disjoint four-unit axial lanes.
        // Both singleton wheels remain at local X=0, outside this X>=1 segment.
        let origin = center + collision_q * DVec3::new(4., 0.3, 0.);
        let expected = match i % 4 {
            0 | 1 | 3 => 3.6,
            _ => 3.5,
        };
        let y = if i % 4 == 2 { 0. } else { 0.3 };
        let origin = if i % 4 == 2 {
            center + collision_q * DVec3::new(4., y, 0.)
        } else {
            origin
        };
        let direction = collision_q * -DVec3::X;
        let distance = ray(sim, origin, direction, 5., None).unwrap();
        near(distance, expected);
        // Use the represented hit for inclusive maximum after composed transforms.
        near(
            ray(sim, origin, direction, distance, None).unwrap(),
            expected,
        );
        assert_eq!(
            ray(sim, origin, direction, distance.next_down(), None),
            None
        );
        assert_eq!(
            ray(sim, origin, direction, 5., Some(handle(sim, &name))),
            None
        );
        near(ray(sim, center, direction, 0.125, None).unwrap(), 0.);
        if i % 4 == 1 || i % 4 == 3 {
            let top = if i % 4 == 1 { 1.5 } else { 1.25 };
            near(
                ray(sim, center + q * DVec3::Z * 2., q * -DVec3::Z, 2., None).unwrap(),
                2. - top,
            );
        }
    }
    // This ray passes through the aggregate AABB between c0 and c1.
    assert_eq!(
        ray(sim, p + q * DVec3::new(6., 0., 2.), q * -DVec3::X, 5., None),
        None
    );
}

fn angles(sim: &Simulation, owner: &str) -> [f64; 2] {
    [
        wheel(sim, owner, "left").position,
        wheel(sim, owner, "right").position,
    ]
}
type DriveReading = (Duration, u64, [f64; 3], [f64; 2]);
fn sample(sim: &Simulation, owner: &str) -> Option<DriveReading> {
    sim.world()
        .get::<DiffDriveSample>(named(sim, &format!("w::{owner}")))
        .map(|value| (value.time, value.epoch, value.pose, value.twist))
}
fn equal_response(a: &Simulation, b: &Simulation, owner: &str) {
    assert_eq!(states(a), states(b));
    assert_eq!((a.epoch(), a.elapsed()), (b.epoch(), b.elapsed()));
    for side in ["left", "right"] {
        assert_eq!(wheel(a, owner, side), wheel(b, owner, side));
    }
    assert_eq!(sample(a, owner), sample(b, owner));
}

// An independent wheel-increment SE(2) reference: over one measured interval,
// travel=(dl+dr)/2 and yaw=(dr-dl)/separation. Integrating Rz(t*yaw)*X gives
// local displacement travel*[sin(yaw)/yaw, (1-cos(yaw))/yaw]. The latter is
// evaluated as 2*sin(yaw/2)^2/yaw to avoid cancellation. Quaternion composition
// carries this arc into the previous odometry frame, independently of the
// product's midpoint-bearing formulation. No ideal wheel symmetry or no-slip
// body motion is assumed; every input is a fresh native encoder coordinate.
struct EncoderReference {
    previous: [f64; 2],
    time: Duration,
    radius: f64,
    position: DVec3,
    rotation: DQuat,
    velocities: VecDeque<[f64; 2]>,
}
impl EncoderReference {
    fn new(sim: &Simulation, owner: &str, radius: f64) -> Self {
        Self {
            previous: angles(sim, owner),
            time: sim.elapsed(),
            radius,
            position: DVec3::ZERO,
            rotation: DQuat::IDENTITY,
            velocities: VecDeque::with_capacity(10),
        }
    }
    fn observe_step(&mut self, sim: &Simulation, owner: &str) -> Option<Duration> {
        let now = sim.elapsed();
        assert!(now > self.time);
        let dt = (now - self.time).as_secs_f64();
        let current = angles(sim, owner);
        let dl = (current[0] - self.previous[0]) * self.radius;
        let dr = (current[1] - self.previous[1]) * self.radius;
        let travel = (dl + dr) * 0.5;
        let yaw = (dr - dl) / 0.5;
        let arc = if yaw == 0. {
            DVec3::new(travel, 0., 0.)
        } else {
            DVec3::new(
                travel * yaw.sin() / yaw,
                travel * 2. * (yaw * 0.5).sin().powi(2) / yaw,
                0.,
            )
        };
        self.position += self.rotation * arc;
        self.rotation *= DQuat::from_rotation_z(yaw);
        if self.velocities.len() == 10 {
            self.velocities.pop_front();
        }
        self.velocities.push_back([travel / dt, yaw / dt]);
        self.previous = current;
        self.time = now;
        let actual = sample(sim, owner)?;
        assert!(actual.0 <= now);
        if actual.0 != now {
            return None;
        }
        assert_eq!(actual.1, sim.epoch());
        let forward = self.rotation * DVec3::X;
        let expected_pose = [self.position.x, self.position.y, forward.y.atan2(forward.x)];
        for (axis, (observed, expected)) in actual.2.into_iter().zip(expected_pose).enumerate() {
            assert!(
                observed.is_finite() && expected.is_finite() && (observed - expected).abs() < 1e-11,
                "pose axis {axis}: sample={actual:?}, expected={expected_pose:?}, encoders={current:?}, body={:?}",
                native(sim, &format!("w::{owner}::base_link"))
            );
        }
        for (axis, observed) in actual.3.into_iter().enumerate() {
            let expected = self.velocities.iter().map(|row| row[axis]).sum::<f64>()
                / self.velocities.len() as f64;
            assert!(
                observed.is_finite() && expected.is_finite() && (observed - expected).abs() < 1e-11,
                "twist axis {axis}: sample={actual:?}, expected={expected}, encoders={current:?}, body={:?}",
                native(sim, &format!("w::{owner}::base_link"))
            );
        }
        Some(now)
    }
}

#[test]
fn compound_chassis_motor_motion_advects_every_primitive_frame_and_strict_query() {
    for count in [2, 4, 16] {
        let text = world(
            &robot("robot", count, "0 0 10 0 0 0.4", 1000., false),
            "0 0 0",
            false,
        );
        let mut sim = load(&text);
        let mut quiet = load(&text);
        let owner = id(&sim, "w::robot");
        assert!(diff_drive_binding_matches(&sim, owner));
        check_members(&sim, "robot", count, true);
        let before = states(&sim);
        command(&mut sim, "robot", 0.3, 0.4);
        assert_eq!(states(&sim), before);
        sim.step_exact(1000).unwrap();
        quiet.step_exact(1000).unwrap();
        // Declared targets are 1 and 2 rad/s, but damping-one finite-force
        // motors do not promise one-second setpoint convergence. Actual signed
        // encoder travel is compared with an independently loaded quiet twin.
        for side in ["left", "right"] {
            let measured = wheel(&sim, "robot", side);
            let neutral = wheel(&quiet, "robot", side);
            assert!(measured.position.is_finite() && measured.velocity.is_finite());
            let travel = (measured.position - neutral.position) * 0.2;
            assert!(
                travel > 0.01,
                "{side}: driven={measured:?}, quiet={neutral:?}, travel={travel}"
            );
            assert!(
                measured.velocity > neutral.velocity,
                "{side}: driven={measured:?}, quiet={neutral:?}"
            );
            let observed = sim
                .world()
                .get::<JointSample>(named(&sim, &format!("w::robot::{side}_wheel_joint")))
                .unwrap();
            assert_eq!((observed.time, observed.epoch), (Duration::from_secs(1), 0));
            assert_eq!(observed.state, measured);
        }
        let driven = native(&sim, "w::robot::base_link");
        let neutral = native(&quiet, "w::robot::base_link");
        assert!(
            driven
                .angular_velocity
                .into_iter()
                .zip(neutral.angular_velocity)
                .any(|(a, b)| (a - b).abs() > 1e-7),
            "driven={driven:?}, quiet={neutral:?}"
        );
        check_members(&sim, "robot", count, true);
        check_members(&quiet, "robot", count, true);
    }
}

#[test]
fn compound_drive_odometry_keeps_encoder_frame_cadence_and_last_valid_paused_command() {
    let text = world(
        &robot("robot", 4, "0 0 10 0 0 0.4", 1000., false),
        "0 0 0",
        false,
    );
    let mut sim = load(&text);
    let mut equivalent = load(&text);
    let owner = id(&sim, "w::robot");
    let entity = entity_by_id(sim.world(), owner).unwrap();
    let configuration = sim.world().get::<DiffDriveConfiguration>(entity).unwrap();
    assert_eq!(
        (
            &*configuration.topic,
            &*configuration.odom_topic,
            &*configuration.frame_id,
            &*configuration.child_frame_id
        ),
        (
            "/m91/robot/cmd_vel",
            "/m91/robot/odom",
            "robot/odom",
            "robot/base"
        )
    );
    let initial = states(&sim);
    sim.set_paused(true);
    equivalent.set_paused(true);
    command(&mut sim, "robot", -0.4, 1.);
    command(&mut sim, "robot", 0.4, 0.);
    command(&mut equivalent, "robot", 0.4, 0.);
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            set_diff_drive_command(&mut sim, owner, invalid, 0.),
            Err(DiffDriveCommandError::InvalidCommand)
        ));
        assert!(matches!(
            set_diff_drive_command(&mut sim, owner, 0., invalid),
            Err(DiffDriveCommandError::InvalidCommand)
        ));
    }
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
        assert!(equivalent.tick().unwrap().dt.is_zero());
    }
    assert_eq!(states(&sim), initial);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert!(sample(&sim, "robot").is_none());
    equal_response(&sim, &equivalent, "robot");
    sim.set_paused(false);
    equivalent.set_paused(false);
    let mut reference = EncoderReference::new(&sim, "robot", 0.2);
    let mut stamps = Vec::new();
    for _ in 0..1000 {
        sim.step_exact(1).unwrap();
        equivalent.step_exact(1).unwrap();
        if let Some(time) = reference.observe_step(&sim, "robot") {
            stamps.push(time);
        }
        if sample(&sim, "robot").is_some_and(|row| row.0 == sim.elapsed()) {
            equal_response(&sim, &equivalent, "robot");
        }
    }
    assert_eq!(
        stamps,
        (1..=50)
            .map(|i| Duration::from_millis(i * 20))
            .collect::<Vec<_>>()
    );
    equal_response(&sim, &equivalent, "robot");
    let measured_sample = sample(&sim, "robot").unwrap();
    assert!(
        measured_sample.2[0] > 0.01,
        "sample={measured_sample:?}, angles={:?}",
        angles(&sim, "robot")
    );
    let free_body = native(&sim, "w::robot::base_link");
    assert!(
        free_body.position[0].abs() < 1e-7,
        "suspended body={free_body:?}"
    );
    // Same physics and initial state: out-of-range commands must be exactly
    // equivalent to their bounded valid command, without claiming motor speed.
    let before = states(&sim);
    command(&mut sim, "robot", 100., -100.);
    command(&mut equivalent, "robot", 1., -2.);
    assert_eq!(states(&sim), before);
    for _ in 0..1000 {
        sim.step_exact(1).unwrap();
        equivalent.step_exact(1).unwrap();
        if let Some(time) = reference.observe_step(&sim, "robot") {
            stamps.push(time);
        }
    }
    equal_response(&sim, &equivalent, "robot");
    // Independently recreate the same moving state for a continuing-command
    // countercontrol. Zero changes the target, not the instantaneous velocity.
    let mut continuing = load(&text);
    command(&mut continuing, "robot", 0.4, 0.);
    continuing.step_exact(1000).unwrap();
    command(&mut continuing, "robot", 1., -2.);
    continuing.step_exact(1000).unwrap();
    equal_response(&sim, &continuing, "robot");
    let before_angles = angles(&sim, "robot");
    let before = states(&sim);
    command(&mut sim, "robot", 0., 0.);
    command(&mut equivalent, "robot", 0., 0.);
    assert_eq!(states(&sim), before);
    for _ in 0..1000 {
        for model in [&mut sim, &mut equivalent, &mut continuing] {
            model.step_exact(1).unwrap();
        }
        if let Some(time) = reference.observe_step(&sim, "robot") {
            stamps.push(time);
        }
    }
    assert_eq!(
        stamps,
        (1..=150)
            .map(|i| Duration::from_millis(i * 20))
            .collect::<Vec<_>>()
    );
    equal_response(&sim, &equivalent, "robot");
    let zero = angles(&sim, "robot");
    let held = angles(&continuing, "robot");
    let measured_travel = |values: [f64; 2]| {
        values
            .into_iter()
            .zip(before_angles)
            .map(|(a, b)| (a - b).abs() * 0.1)
            .sum::<f64>()
    };
    let zero_travel = measured_travel(zero);
    let held_travel = measured_travel(held);
    assert!(
        held_travel - zero_travel > 0.01,
        "zero={zero:?} held={held:?} before={before_angles:?}; distances {zero_travel} vs {held_travel}"
    );
}

#[test]
fn compound_drive_force_caps_use_centered_explicit_inertia_not_member_density() {
    let source = robot("robot", 4, "0 0 10 0 0 0", 1., false)
        .replace("<effort>5</effort>", "<effort>0.002</effort>")
        .replace("<ixx>0.027083333333333334</ixx><iyy>0.06666666666666667</iyy><izz>0.08041666666666666</izz>", "<ixx>2</ixx><iyy>2</iyy><izz>2</izz>");
    let heavy = source.replace(
        "<ixx>2</ixx><iyy>2</iyy><izz>2</izz>",
        "<ixx>4</ixx><iyy>4</iyy><izz>4</izz>",
    );
    let mut a = load(&world(&source, "0 0 0", false));
    let mut b = load(&world(
        &source.replace("<density>1</density>", "<density>1000000</density>"),
        "0 0 0",
        false,
    ));
    let mut c = load(&world(&heavy, "0 0 0", false));
    for sim in [&mut a, &mut b, &mut c] {
        command(sim, "robot", 1., 0.);
        sim.step_exact(1).unwrap();
    }
    let light = native(&a, "w::robot::base_link").angular_velocity[1];
    let heavier = native(&c, "w::robot::base_link").angular_velocity[1];
    assert!(light < 0. && light.abs() <= 2. * 0.002 / 2. * 0.001 + 1e-12);
    assert!(heavier < 0. && heavier.abs() <= 2. * 0.002 / 4. * 0.001 + 1e-12);
    assert!(heavier.abs() < light.abs());
    assert_eq!(states(&a), states(&b));
    for side in ["left", "right"] {
        assert!(wheel(&a, "robot", side).velocity > 0.);
        near(
            wheel(&a, "robot", side).velocity,
            wheel(&b, "robot", side).velocity,
        );
    }
}

#[test]
fn compound_chassis_ground_traction_moves_and_turns_without_suspended_teleport() {
    let source = robot("robot", 2, "0 0 0.2 0 0 0", 1000., true);
    for (linear, angular) in [(0.4, 0.), (-0.4, 0.), (0., 1.)] {
        let mut grounded = load(&world(&source, "0 0 -9.81", true));
        let mut suspended = load(&world(&source, "0 0 0", false));
        let mut quiet = load(&world(&source, "0 0 -9.81", true));
        for sim in [&mut grounded, &mut suspended] {
            let before = states(sim);
            command(sim, "robot", linear, angular);
            assert_eq!(states(sim), before);
        }
        // Same retained one-second window; quiet ground contact may settle.
        for sim in [&mut grounded, &mut suspended, &mut quiet] {
            sim.step_exact(1000).unwrap();
        }
        let ground = native(&grounded, "w::robot::base_link");
        let free = native(&suspended, "w::robot::base_link");
        let still = native(&quiet, "w::robot::base_link");
        assert!(
            free.position[0].abs() < 1e-7 && free.position[1].abs() < 1e-7,
            "suspended={free:?}"
        );
        assert!(
            ground
                .position
                .into_iter()
                .chain(ground.orientation)
                .chain(still.position)
                .chain(still.orientation)
                .all(f64::is_finite)
        );
        let driven_odom = sample(&grounded, "robot").unwrap();
        let quiet_odom = sample(&quiet, "robot").unwrap();
        assert_eq!((driven_odom.0, driven_odom.1), (Duration::from_secs(1), 0));
        assert_eq!((quiet_odom.0, quiet_odom.1), (Duration::from_secs(1), 0));
        if linear != 0. {
            let displacement = (ground.position[0] - still.position[0]) * linear.signum();
            assert!(
                displacement > 0.01,
                "command={linear}; driven={ground:?}, quiet={still:?}, signed separation={displacement}"
            );
            assert!(
                (driven_odom.2[0] - quiet_odom.2[0]) * linear.signum() > 0.01,
                "driven={driven_odom:?}, quiet={quiet_odom:?}"
            );
        } else {
            let forward = DQuat::from_array(ground.orientation) * DVec3::X;
            let neutral = DQuat::from_array(still.orientation) * DVec3::X;
            assert!(forward.x.hypot(forward.y) > 0.5 && neutral.x.hypot(neutral.y) > 0.5);
            let delta = forward.y.atan2(forward.x) - neutral.y.atan2(neutral.x);
            let yaw = delta.sin().atan2(delta.cos());
            assert!(
                yaw > 0.01,
                "driven={ground:?}, quiet={still:?}, yaw separation={yaw}"
            );
            assert!(
                driven_odom.2[2] - quiet_odom.2[2] > 0.01,
                "driven={driven_odom:?}, quiet={quiet_odom:?}"
            );
        }
    }
}

#[test]
fn compound_chassis_bumper_contacts_move_a_target_while_quiet_controls_remain_clear() {
    let ball = |name: &str, x: f64| {
        format!(
            "<model name='{name}'><pose>{x} 0 10.406 0 0 0</pose><link name='body'><inertial><mass>0.2</mass><inertia><ixx>0.0032</ixx><iyy>0.0032</iyy><izz>0.0032</izz></inertia></inertial><collision name='ball'><geometry><sphere><radius>0.2</radius></sphere></geometry></collision></link></model>"
        )
    };
    let models =
        robot("robot", 2, "0 0 10 0 0 0", 1000., true) + &ball("target", 0.5) + &ball("clear", 20.);
    let text = world(&models, "0 0 0", false);
    let mut driven = load(&text);
    let mut quiet = load(&text);
    // Isolate motor reaction from unsupported two-wheel ground-settling paths.
    // Bumper center [.5,0,10.1], r=.1; target [.5,0,10.406], r=.2:
    // exact initial gap .006. Main box top10.1 and wheel centers x0/r.2
    // cannot touch the target before this bumper; the clear body is at x20.
    let target_before = native(&driven, "w::target::body");
    let bumper = driven
        .world()
        .get::<WorldPose>(named(&driven, "w::robot::base_link::c1"))
        .unwrap()
        .translation;
    vector(bumper, [0.5, 0., 10.1]);
    near(
        (DVec3::from_array(target_before.position) - DVec3::from_array(bumper)).length() - 0.3,
        0.006,
    );
    let chassis_before = native(&driven, "w::robot::base_link");
    command(&mut driven, "robot", 0.4, 0.);
    assert_eq!(native(&driven, "w::robot::base_link"), chassis_before);
    driven.step_exact(1).unwrap();
    quiet.step_exact(1).unwrap();
    assert!(
        native(&driven, "w::robot::base_link").angular_velocity[1] < 0.,
        "first-step chassis={:?}",
        native(&driven, "w::robot::base_link")
    );
    // Positive wheel torques produce actual negative-Y chassis reaction: its
    // +X bumper rises into the above target. Total horizon remains1500steps.
    driven.step_exact(1499).unwrap();
    quiet.step_exact(1499).unwrap();
    let moved = native(&driven, "w::target::body");
    let neutral = native(&quiet, "w::target::body");
    assert!(
        moved.position[2] - target_before.position[2] > 1e-7,
        "driven={moved:?}, initial={target_before:?}"
    );
    assert!(
        (neutral.position[2] - target_before.position[2]).abs() < 1e-7,
        "quiet={neutral:?}, initial={target_before:?}"
    );
    let chassis = native(&driven, "w::robot::base_link");
    let stationary = native(&quiet, "w::robot::base_link");
    let turn = (DQuat::from_array(chassis.orientation) * DVec3::X
        - DQuat::from_array(stationary.orientation) * DVec3::X)
        .length();
    assert!(
        turn > 1e-7,
        "driven chassis={chassis:?}, quiet={stationary:?}"
    );
    for sim in [&driven, &quiet] {
        let clear = native(sim, "w::clear::body");
        for (actual, expected) in clear.position.into_iter().zip([20., 0., 10.406]) {
            assert!((actual - expected).abs() < 1e-7, "clear={clear:?}");
        }
    }
}

#[test]
fn compound_drive_resets_clear_scene_targets_and_prepared_reset_restores_only_geometry() {
    let mut sim = load(&world(
        &robot("robot", 4, "0 0 10 0 0 0", 1000., false),
        "0 0 0",
        false,
    ));
    let keys = identities(&sim).keys().copied().collect::<Vec<_>>();
    let original = states(&sim);
    for loaded in [false, true] {
        command(&mut sim, "robot", 0.4, 0.5);
        sim.step_exact(1000).unwrap();
        sim.set_paused(true);
        command(&mut sim, "robot", -0.4, -1.);
        if loaded {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        } else {
            sim.reset().unwrap();
        }
        sim.set_paused(false);
        assert_eq!(identities(&sim).keys().copied().collect::<Vec<_>>(), keys);
        assert_eq!(states(&sim), original);
        assert!(
            sim.world()
                .get::<DiffDriveSample>(named(&sim, "w::robot"))
                .is_none()
        );
        assert!(diff_drive_binding_matches(&sim, id(&sim, "w::robot")));
        sim.step_exact(100).unwrap();
        for side in ["left", "right"] {
            near(wheel(&sim, "robot", side).position, 0.);
            near(wheel(&sim, "robot", side).velocity, 0.);
        }
        check_members(&sim, "robot", 4, true);
    }
    let initial = sim.world().resource::<PhysicsBackend>().initial_state();
    command(&mut sim, "robot", 0.4, 0.);
    sim.step_exact(100).unwrap();
    let prepared = sim
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial)
        .unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    // No Scene step or assertion that bare backend reconstruction clears Drive.desired.
    check_members(&sim, "robot", 4, false);
    near(wheel(&sim, "robot", "left").position, 0.);
}

#[test]
fn compound_drive_owners_retire_independently_and_rebuild_original_ids_above_high_water() {
    let models = (0..8)
        .map(|i| {
            robot(
                &format!("r{i}"),
                2,
                &format!("{} 0 10 0 0 0", i * 100),
                1000.,
                false,
            )
        })
        .collect::<String>();
    let text = world(&models, "0 0 0", false);
    let mut sim = load(&text);
    let original = identities(&sim).keys().copied().collect::<Vec<_>>();
    let owners = initial_diff_drive_configurations_with_identity(&sim)
        .unwrap()
        .map(|r| r.id)
        .collect::<Vec<_>>();
    assert_eq!(owners.len(), 8);
    for i in 0_usize..8 {
        command(
            &mut sim,
            &format!("r{i}"),
            if i.is_multiple_of(2) { 0.4 } else { -0.4 },
            0.,
        );
    }
    sim.step_exact(1000).unwrap();
    for i in 0_usize..8 {
        assert!(
            wheel(&sim, &format!("r{i}"), "left").position
                * if i.is_multiple_of(2) { 1. } else { -1. }
                > 0.01
        );
    }
    let retired = id(&sim, "w::r0");
    let old_body = handle(&sim, "w::r0::base_link");
    let removed = remove_model(&mut sim, retired).unwrap();
    assert!(removed.contains(&retired));
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(old_body)
            .is_none()
    );
    assert!(
        matches!(set_diff_drive_command(&mut sim,retired,0.,0.),Err(DiffDriveCommandError::MissingDrive(key)) if key==retired)
    );
    for i in 1..8 {
        assert!(diff_drive_binding_matches(
            &sim,
            id(&sim, &format!("w::r{i}"))
        ));
    }
    sim.step_exact(100).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        identities(&sim).keys().copied().collect::<Vec<_>>(),
        original
    );
    for i in 0_usize..8 {
        assert!(diff_drive_binding_matches(
            &sim,
            id(&sim, &format!("w::r{i}"))
        ));
    }
    let fresh = sim.world_mut().spawn_empty().id();
    assert!(gz_sim_core::assign_id(sim.world_mut(), fresh).unwrap() > *original.last().unwrap());
    assert!(matches!(
        load_world_with_sdf_step(
            &world(
                &(models + &robot("ninth", 2, "900 0 10 0 0 0", 1000., false)),
                "0 0 0",
                false
            ),
            "ninth",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "multiple differential-drive systems",
            ..
        })
    ));
}

#[test]
fn compound_drive_binding_drift_refuses_before_any_owner_changes_pending_physics() {
    let models = robot("good", 4, "0 0 10 0 0 0", 1000., false)
        + &robot("bad", 2, "100 0 10 0 0 0", 1000., false);
    for native_fault in [false, true] {
        let mut sim = load(&world(&models, "0 0 0", false));
        command(&mut sim, "good", 0.4, 0.);
        command(&mut sim, "bad", -0.4, 0.);
        let owner = id(&sim, "w::bad");
        let entity = entity_by_id(sim.world(), owner).unwrap();
        let configuration = sim
            .world()
            .get::<DiffDriveConfiguration>(entity)
            .unwrap()
            .clone();
        let native_joint = sim
            .world()
            .get::<JointHandle>(named(&sim, "w::bad::right_wheel_joint"))
            .unwrap()
            .id();
        if native_fault {
            assert!(
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove_joint(native_joint)
            );
        } else {
            let mut changed = configuration.clone();
            changed.topic = "/forged/drive".into();
            sim.world_mut().entity_mut(entity).insert(changed);
        }
        assert!(!diff_drive_binding_matches(&sim, owner));
        let before = states(&sim);
        let keys = identities(&sim);
        let clock = (sim.epoch(), sim.elapsed());
        if native_fault {
            assert!(matches!(
                set_diff_drive_command(&mut sim, owner, 0., 0.),
                Err(DiffDriveCommandError::ForeignPhysics)
            ));
        } else {
            assert!(
                matches!(set_diff_drive_command(&mut sim,owner,0.,0.),Err(DiffDriveCommandError::MissingDrive(key)) if key==owner)
            );
        }
        assert_eq!(states(&sim), before);
        assert_eq!(identities(&sim), keys);
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        assert!(sim.step_exact(1).is_err());
        assert_eq!(states(&sim), before);
        // The later bad owner prevented the good owner's motor from advancing.
        near(wheel(&sim, "good", "left").position, 0.);
        // StepFault is terminal: restoring metadata cannot make reset legal.
        assert!(sim.is_faulted());
        assert!(matches!(sim.reset(), Err(gz_sim_core::StepError::Faulted)));
        assert_eq!(states(&sim), before);
        assert_eq!(identities(&sim), keys);
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        if !native_fault {
            // Repair before any faulting step in an independent healthy world,
            // preserving both pending commands, then exercise ordinary reset.
            let mut healthy = load(&world(&models, "0 0 0", false));
            command(&mut healthy, "good", 0.4, 0.);
            command(&mut healthy, "bad", -0.4, 0.);
            let bad = entity_by_id(healthy.world(), id(&healthy, "w::bad")).unwrap();
            let original = healthy
                .world()
                .get::<DiffDriveConfiguration>(bad)
                .unwrap()
                .clone();
            let mut drift = original.clone();
            drift.topic = "/forged/drive".into();
            healthy.world_mut().entity_mut(bad).insert(drift);
            assert!(!diff_drive_binding_matches(
                &healthy,
                id(&healthy, "w::bad")
            ));
            healthy.world_mut().entity_mut(bad).insert(original);
            assert!(diff_drive_binding_matches(&healthy, id(&healthy, "w::bad")));
            healthy.step_exact(1).unwrap();
            let good = wheel(&healthy, "good", "left");
            let bad = wheel(&healthy, "bad", "left");
            assert!(good.position > 0., "repaired good encoder={good:?}");
            assert!(bad.position < 0., "repaired bad encoder={bad:?}");
            healthy.reset().unwrap();
            near(wheel(&healthy, "good", "left").position, 0.);
            command(&mut healthy, "good", 0.4, 0.);
            healthy.step_exact(1000).unwrap();
            let measured = wheel(&healthy, "good", "left");
            assert!(
                measured.position > 0.01,
                "healthy reset encoder={measured:?}"
            );
        }
    }
}

#[test]
fn compound_chassis_admission_preserves_singleton_wheel_parameters_and_closed_domains() {
    let base = robot("robot", 2, "0 0 10 0 0 0", 1000., false);
    for shape in [
        "<sphere><radius>0.2</radius></sphere>",
        "<box><size>0.4 0.4 0.4</size></box>",
        "<cylinder><radius>0.2</radius><length>0.4</length></cylinder>",
        "<capsule><radius>0.2</radius><length>0.4</length></capsule>",
    ] {
        let geometry = WHEEL.replace("<sphere><radius>0.2</radius></sphere>", shape);
        let text = world(
            &base.replace(WHEEL, &geometry).replace(
                "<wheel_radius>0.2</wheel_radius>",
                "<wheel_radius>0.3</wheel_radius>",
            ),
            "0 0 0",
            false,
        );
        let mut sim = load(&text);
        let mut quiet = load(&text);
        let mut radius_control = load(&text.replace(
            "<wheel_radius>0.3</wheel_radius>",
            "<wheel_radius>0.2</wheel_radius>",
        ));
        command(&mut sim, "robot", 0.6, 0.);
        command(&mut radius_control, "robot", 0.4, 0.);
        let mut reference = EncoderReference::new(&sim, "robot", 0.3);
        let mut stamps = Vec::new();
        for _ in 0..1000 {
            sim.step_exact(1).unwrap();
            quiet.step_exact(1).unwrap();
            radius_control.step_exact(1).unwrap();
            if let Some(time) = reference.observe_step(&sim, "robot") {
                stamps.push(time);
            }
        }
        assert_eq!(
            stamps,
            (1..=50)
                .map(|i| Duration::from_millis(i * 20))
                .collect::<Vec<_>>()
        );
        // Equal authored targets (.6/.3 == .4/.2 == 2) must produce identical
        // native bodies and encoders with identical collision geometry. This
        // detects a motor that substitutes collider size for declared radius;
        // the independent odometry reference separately checks the .3 radius.
        assert_eq!(states(&sim), states(&radius_control));
        for side in ["left", "right"] {
            assert_eq!(
                wheel(&sim, "robot", side),
                wheel(&radius_control, "robot", side)
            );
        }
        for side in ["left", "right"] {
            let measured = wheel(&sim, "robot", side);
            let neutral = wheel(&quiet, "robot", side);
            assert!(measured.position.is_finite() && measured.velocity.is_finite());
            let travel = (measured.position - neutral.position) * 0.3;
            assert!(
                travel > 0.01,
                "{side}: driven={measured:?}, quiet={neutral:?}, travel={travel}"
            );
        }
    }
    let extra = "<collision name='extra'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>";
    let vc = "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
    let joint_controller = "<plugin filename='gz-sim-joint-controller-system' name='gz::sim::systems::JointController'><joint_name>left_wheel_joint</joint_name></plugin>";
    for (changed, feature) in [
        (
            base.replacen(WHEEL, &format!("{WHEEL}{extra}"), 1),
            "compound joint actuators",
        ),
        (
            robot("robot", 0, "0 0 10 0 0 0", 1000., false),
            "collision count (expected one)",
        ),
        (
            robot("robot", 17, "0 0 10 0 0 0", 1000., false),
            "collision count (expected one)",
        ),
        (
            base.replace(
                "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
                "<plane><normal>0 0 1</normal><size>1 1</size></plane>",
            ),
            "compound collision geometry",
        ),
        (
            base.replace(
                "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
                "<mesh><uri>missing.obj</uri></mesh>",
            ),
            "compound collision geometry",
        ),
        (
            base.replacen("<inertial>", "<inertial><pose>0.25 0 0 0 0 0</pose>", 1),
            "offset center of mass",
        ),
        (
            base.replacen("<xyz>0 1 0</xyz>", "<xyz>1 0 0</xyz>", 1),
            "DiffDrive wheel limits",
        ),
        (
            base.replacen("<effort>5</effort>", "<effort>0</effort>", 1),
            "DiffDrive wheel limits",
        ),
        (
            base.replace(
                "<wheel_radius>0.2</wheel_radius>",
                "<wheel_radius>0</wheel_radius>",
            ),
            "DiffDrive geometry",
        ),
        (
            base.replace(
                "<topic>/m91/robot/cmd_vel</topic>",
                "<topic>bad topic</topic>",
            ),
            "DiffDrive topics",
        ),
        (
            base.replace(
                "<right_joint>right_wheel_joint</right_joint>",
                "<right_joint>left_wheel_joint</right_joint>",
            ),
            "DiffDrive wheel pair",
        ),
        (
            base.replace(
                "<pose>0 -0.25 0 0 0 0</pose>",
                "<pose>0 -0.3 0 0 0 0</pose>",
            ),
            "DiffDrive wheel pair",
        ),
        (
            base.replace("<pose>0 0 10 0 0 0</pose>", "<pose>0 0 10 0.2 0 0</pose>"),
            "non-planar DiffDrive initial frame",
        ),
        (
            base.replacen(
                "<link name=\"base_link\">",
                "<static>true</static><link name=\"base_link\">",
                1,
            ),
            "DiffDrive model domain",
        ),
        (
            base.replacen(DRIVE, &format!("{vc}{DRIVE}"), 1),
            "VelocityControl requires a top-level dynamic free single link",
        ),
        (
            base.replacen(DRIVE, &format!("{joint_controller}{DRIVE}"), 1),
            "joint controller chain",
        ),
    ] {
        assert_ne!(changed, base);
        assert!(
            matches!(load_world_with_sdf_step(&world(&changed,"0 0 0",false),"closed","w",&sdformat::NoResources,PhysicsEngine::Rapier),Err(LoadError::Unsupported{feature:actual,..}) if actual==feature),
            "{feature}"
        );
    }
    assert!(matches!(
        load_world_with_sdf_step(
            &world(&base, "0 0 0", false),
            "tpe",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "DiffDrive requires a top-level dynamic Rapier model",
            ..
        })
    ));
    let nested = format!("<model name='outer'>{base}</model>");
    assert!(matches!(
        load_world_with_sdf_step(
            &world(&nested, "0 0 0", false),
            "nested",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        ),
        Err(LoadError::Unsupported {
            feature: "DiffDrive requires a top-level dynamic Rapier model",
            ..
        })
    ));
    let text = world(&base, "0 0 0", false);
    let parsed = sdformat::load_str(&text, "closed-edit", &sdformat::NoResources).unwrap();
    let mut sim = load(&text);
    let owner = id(&sim, "w::robot");
    let before = states(&sim);
    let keys = identities(&sim);
    let next = next_entity_id(sim.world()).unwrap();
    assert!(matches!(
        model_state(&sim, owner),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        set_model_state(
            &mut sim,
            owner,
            ModelState {
                translation: [0.; 3],
                rotation: [0., 0., 0., 1.],
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3]
            }
        ),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(states(&sim), before);
    assert_eq!(identities(&sim), keys);
    assert_eq!(next_entity_id(sim.world()).unwrap(), next);
}
