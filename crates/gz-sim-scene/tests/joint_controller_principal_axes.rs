// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Principal axes reuse the existing scalar motors and native two-joint chain.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodySpec, BodyState, JointState, PhysicsBackend, RotationSpec, Shape};
use gz_sim_core::{EntityId, Simulation, StepError, assign_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, JointControllerCommandError, JointControllerConfiguration, JointHandle,
    JointSample, LoadError, LocalPose, ModelState, Name, PhysicsEngine, WorldEditError, WorldPose,
    initial_joint_controller_configurations_with_identity, joint_controller_binding_matches,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_joint_controller_command, set_model_state,
};
use std::{collections::BTreeMap, time::Duration};

const POSITION: &str = "<plugin name='gz::sim::systems::JointPositionController' filename='gz-sim-joint-position-controller-system'><joint_name>JOINT</joint_name><use_velocity_commands>true</use_velocity_commands><cmd_max>4</cmd_max><initial_position>0</initial_position></plugin>";
const VELOCITY: &str = "<plugin name='gz::sim::systems::JointController' filename='gz-sim-joint-controller-system'><joint_name>JOINT</joint_name><initial_velocity>0</initial_velocity></plugin>";
const AXES: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
const OFFSET: f64 = 100.;

fn inertia(moment: f64) -> String {
    format!(
        "<inertial><mass>2</mass><inertia><ixx>{moment}</ixx><iyy>{moment}</iyy><izz>{moment}</izz></inertia></inertial>"
    )
}
fn children(count: usize, density: f64) -> String {
    (0..count).map(|i| {
        let geometry = match i % 4 {
            0 => "<sphere><radius>0.5</radius></sphere>",
            1 => "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
            2 => "<box><size>1 0.5 1.5</size></box>",
            _ => "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
        };
        let yaw = if i % 4 == 0 { 0. } else { 0.3 };
        format!("<collision name='c{i}'><pose>2 0 {} 0 0 {yaw}</pose><density>{density}</density><geometry>{geometry}</geometry></collision>", 4. * i as f64)
    }).collect()
}
fn arm(
    name: &str,
    axes: [[f64; 3]; 2],
    counts: [usize; 2],
    swapped: bool,
    moment: f64,
    density: f64,
    pose: &str,
) -> String {
    let root = if swapped { VELOCITY } else { POSITION }.replace("JOINT", "root");
    let tip = if swapped { POSITION } else { VELOCITY }.replace("JOINT", "tip");
    let joint = |name: &str, parent: &str, child: &str, axis: [f64; 3]| {
        format!(
            "<joint name='{name}' type='revolute'><parent>{parent}</parent><child>{child}</child><axis><xyz>{} {} {}</xyz><limit><effort>1000</effort><velocity>4</velocity></limit></axis></joint>",
            axis[0], axis[1], axis[2]
        )
    };
    format!(
        "<model name='{name}' canonical_link='base'><pose>{pose}</pose><link name='base'>{}{}<visual name='v'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual></link><link name='child'><pose>0 0 {OFFSET} 0 0 0</pose>{}{}<visual name='v'><geometry><sphere><radius>0.1</radius></sphere></geometry></visual></link>{}{}{root}{tip}</model>",
        inertia(moment),
        children(counts[0], density),
        inertia(moment),
        children(counts[1], density),
        joint("root", "world", "base", axes[0]),
        joint("tip", "base", "child", axes[1])
    )
}
fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "principal-controller",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|v| v.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id()
}
fn state(sim: &Simulation, name: &str, joint: &str) -> JointState {
    let e = named(sim, &format!("w::{name}::{joint}"));
    sim.world()
        .resource::<PhysicsBackend>()
        .joint_state(sim.world().get::<JointHandle>(e).unwrap().id())
        .unwrap()
}
fn command(sim: &mut Simulation, name: &str, joint: &str, value: f64) {
    let key = id(sim, &format!("w::{name}::{joint}"));
    set_joint_controller_command(sim, key, value).unwrap();
}
fn bodies(sim: &Simulation) -> BTreeMap<u64, BodyState> {
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
fn ids(sim: &Simulation) -> BTreeMap<u64, Entity> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|v| (v.get(), e.id())))
        .collect()
}
fn near(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: DVec3, b: DVec3) {
    for (a, b) in a.to_array().into_iter().zip(b.to_array()) {
        near(a, b);
    }
}
fn query(
    sim: &Simulation,
    origin: DVec3,
    direction: DVec3,
    max: f64,
    excluded: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin.to_array(), direction.to_array(), max, excluded)
        .unwrap()
}
// Independent geometric reference: Rodrigues rotates a vector using its axial
// and transverse parts. Compose the two measured scalar coordinates about their
// actual pivots; neither a motor target nor the native body's own pose is input.
fn rotate(axis: [f64; 3], angle: f64, v: DVec3) -> DVec3 {
    let a = DVec3::from_array(axis);
    v * angle.cos() + a.cross(v) * angle.sin() + a * a.dot(v) * (1. - angle.cos())
}
fn frames(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    origin: DVec3,
    model_q: DQuat,
) -> [(DVec3, [DVec3; 3]); 2] {
    let first = state(sim, name, "root").position;
    let second = state(sim, name, "tip").position;
    let basis = [DVec3::X, DVec3::Y, DVec3::Z];
    let base = basis.map(|v| model_q * rotate(axes[0], first, v));
    let child = basis.map(|v| model_q * rotate(axes[0], first, rotate(axes[1], second, v)));
    [
        (origin, base),
        (
            origin + model_q * rotate(axes[0], first, DVec3::Z * OFFSET),
            child,
        ),
    ]
}
fn apply(basis: [DVec3; 3], v: DVec3) -> DVec3 {
    basis[0] * v.x + basis[1] * v.y + basis[2] * v.z
}
fn check_frames_and_queries(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    counts: [usize; 2],
    origin: DVec3,
    model_q: DQuat,
) {
    let expected = frames(sim, name, axes, origin, model_q);
    for (index, link) in ["base", "child"].into_iter().enumerate() {
        let link_name = format!("w::{name}::{link}");
        let (position, basis) = expected[index];
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body(sim, &link_name))
            .unwrap();
        vector(DVec3::from_array(native.position), position);
        let first = state(sim, name, "root");
        let root_omega = model_q * DVec3::from_array(axes[0]) * first.velocity;
        let expected_omega = if index == 0 {
            root_omega
        } else {
            root_omega
                + model_q
                    * rotate(axes[0], first.position, DVec3::from_array(axes[1]))
                    * state(sim, name, "tip").velocity
        };
        vector(DVec3::from_array(native.angular_velocity), expected_omega);
        vector(
            DVec3::from_array(native.velocity),
            if index == 0 {
                DVec3::ZERO
            } else {
                root_omega.cross(position - origin)
            },
        );
        let logical = sim
            .world()
            .get::<WorldPose>(named(sim, &link_name))
            .unwrap();
        vector(DVec3::from_array(logical.translation), position);
        for (v, expected_axis) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().zip(basis) {
            vector(DQuat::from_array(native.orientation) * v, expected_axis);
            vector(DQuat::from_array(logical.rotation) * v, expected_axis);
        }
        let joint = if index == 0 { "root" } else { "tip" };
        let key = id(sim, &format!("w::{name}::{joint}"));
        assert!(joint_controller_binding_matches(sim, key));
        let sample = sim
            .world()
            .get::<JointSample>(named(sim, &format!("w::{name}::{joint}")))
            .unwrap();
        assert_eq!(sample.state, state(sim, name, joint));
        assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
        for i in 0..counts[index] {
            let local = DVec3::new(2., 0., 4. * i as f64);
            let center = position + apply(basis, local);
            let yaw = if i % 4 == 0 { 0. } else { 0.3 };
            let shape =
                [DVec3::X, DVec3::Y, DVec3::Z].map(|v| apply(basis, rotate(AXES[2], yaw, v)));
            let e = named(sim, &format!("{link_name}::c{i}"));
            let actual = sim.world().get::<WorldPose>(e).unwrap();
            vector(DVec3::from_array(actual.translation), center);
            let local_pose = sim.world().get::<LocalPose>(e).unwrap().0;
            vector(DVec3::from_array(local_pose.translation), local);
            for (v, wanted) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().zip(shape) {
                vector(DQuat::from_array(actual.rotation) * v, wanted);
            }
            // Every child has its own four-unit axial lane. At the small measured
            // angles below, the other link starts 100 units away and extends
            // outward from its pivot; every query is capped at five units.
            assert!(state(sim, name, "tip").position.abs() < 1.);
            let ray_origin = center + shape[0] * 2.;
            let ray_direction = -shape[0];
            let hit = query(sim, ray_origin, ray_direction, 3., None).unwrap();
            near(hit, 1.5);
            assert_eq!(query(sim, ray_origin, ray_direction, hit, None), Some(hit));
            assert!(query(sim, ray_origin, ray_direction, hit.next_down(), None).is_none());
            near(query(sim, center, shape[0], 0.125, None).unwrap(), 0.);
            assert!(
                query(
                    sim,
                    ray_origin,
                    ray_direction,
                    3.,
                    Some(body(sim, &link_name))
                )
                .is_none()
            );
            if i % 4 == 1 {
                near(
                    query(
                        sim,
                        center + shape[0] * 4. + shape[1] * 0.3,
                        -shape[0],
                        5.,
                        None,
                    )
                    .unwrap(),
                    3.6,
                );
                near(
                    query(sim, center + shape[2] * 2., -shape[2], 3., None).unwrap(),
                    0.5,
                );
            } else if i % 4 == 2 {
                near(
                    query(sim, center + shape[2] * 2., -shape[2], 3., None).unwrap(),
                    1.25,
                );
            } else if i % 4 == 3 {
                near(
                    query(sim, center + shape[2] * 2., -shape[2], 3., None).unwrap(),
                    0.75,
                );
            }
        }
        if counts[index] >= 2 {
            let gap = position + apply(basis, DVec3::new(5., 0., 2.));
            assert!(query(sim, gap, -basis[0], 6., None).is_none());
        }
    }
    let model = sim
        .world()
        .get::<WorldPose>(named(sim, &format!("w::{name}")))
        .unwrap();
    vector(DVec3::from_array(model.translation), origin);
    for (v, wanted) in [DVec3::X, DVec3::Y, DVec3::Z]
        .into_iter()
        .zip(expected[0].1)
    {
        vector(DQuat::from_array(model.rotation) * v, wanted);
    }
}

#[test]
fn principal_axes_and_noncoaxial_chains_move_native_frames_and_every_child_query() {
    let q = DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2);
    for axes in [
        [AXES[0], AXES[0]],
        [AXES[1], AXES[1]],
        [AXES[2], AXES[2]],
        [AXES[0], AXES[1]],
        [AXES[1], AXES[0]],
    ] {
        for counts in [[1, 1], [2, 4], [4, 2], [16, 16]] {
            for swapped in [false, true] {
                let text = world(&arm(
                    "arm",
                    axes,
                    counts,
                    swapped,
                    2.,
                    1000.,
                    "3 -2 8 0.2 -0.3 0.4",
                ));
                let mut driven = load(&text);
                let mut quiet = load(&text);
                let saved = ids(&driven);
                let before = bodies(&driven);
                check_frames_and_queries(&driven, "arm", axes, counts, DVec3::new(3., -2., 8.), q);
                command(&mut driven, "arm", "root", 0.4);
                command(&mut driven, "arm", "tip", -0.3);
                assert_eq!(bodies(&driven), before);
                driven.step_exact(1000).unwrap();
                quiet.step_exact(1000).unwrap();
                let a = state(&driven, "arm", "root");
                let b = state(&driven, "arm", "tip");
                assert!(
                    a.position.is_finite()
                        && b.position.is_finite()
                        && a.position > state(&quiet, "arm", "root").position + 1e-7
                        && b.position < state(&quiet, "arm", "tip").position - 1e-7,
                    "axes={axes:?}, modes={swapped}, root={a:?}, tip={b:?}, quiet={:?}",
                    bodies(&quiet)
                );
                assert_ne!(bodies(&driven), bodies(&quiet));
                assert_eq!(ids(&driven), saved);
                assert_eq!(driven.elapsed(), Duration::from_secs(1));
                check_frames_and_queries(&driven, "arm", axes, counts, DVec3::new(3., -2., 8.), q);
            }
        }
    }
}

#[test]
fn principal_axis_binary_sphere_endpoints_are_distinct_from_advected_roundoff() {
    let mut sim = load(&world(&arm(
        "arm",
        [AXES[0], AXES[1]],
        [4, 4],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    )));
    let handle = body(&sim, "w::arm::base");
    let origin = DVec3::new(3.5, 0., 0.);
    assert_eq!(query(&sim, origin, -DVec3::X, 1., None), Some(1.));
    assert!(query(&sim, origin, -DVec3::X, 0.999, None).is_none());
    assert_eq!(
        query(&sim, DVec3::new(2., 0., 0.), DVec3::X, 0.125, None),
        Some(0.)
    );
    assert!(query(&sim, origin, -DVec3::X, 1., Some(handle)).is_none());
    command(&mut sim, "arm", "root", 0.4);
    command(&mut sim, "arm", "tip", -0.3);
    sim.step_exact(1000).unwrap();
    check_frames_and_queries(
        &sim,
        "arm",
        [AXES[0], AXES[1]],
        [4, 4],
        DVec3::ZERO,
        DQuat::IDENTITY,
    );
}

#[test]
fn principal_axis_commands_keep_clamp_latest_zero_pause_and_invalid_equivalence() {
    let text = world(&arm(
        "arm",
        [AXES[0], AXES[1]],
        [4, 4],
        true,
        2.,
        1000.,
        "0 0 0 0 0 0",
    ));
    let mut saturated = load(&text);
    let mut bounded = load(&text);
    let mut continuing = load(&text);
    command(&mut continuing, "arm", "root", 4.);
    command(&mut continuing, "arm", "tip", 0.3);
    continuing.step_exact(200).unwrap();
    for sim in [&mut saturated, &mut bounded] {
        sim.set_paused(true);
    }
    let initial = bodies(&saturated);
    command(&mut saturated, "arm", "root", 100.);
    command(&mut bounded, "arm", "root", 4.);
    for sim in [&mut saturated, &mut bounded] {
        command(sim, "arm", "tip", -0.2);
        command(sim, "arm", "tip", 0.3);
    }
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let key = id(&saturated, "w::arm::root");
        assert!(matches!(
            set_joint_controller_command(&mut saturated, key, value),
            Err(JointControllerCommandError::InvalidCommand)
        ));
    }
    for sim in [&mut saturated, &mut bounded] {
        for _ in 0..3 {
            assert!(sim.tick().unwrap().dt.is_zero());
        }
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(bodies(sim), initial);
        sim.set_paused(false);
        sim.step_exact(200).unwrap();
    }
    assert_eq!(bodies(&saturated), bodies(&bounded));
    assert_eq!(bodies(&saturated), bodies(&continuing));
    assert_eq!(
        state(&saturated, "arm", "root"),
        state(&bounded, "arm", "root")
    );
    for sim in [&mut saturated, &mut bounded] {
        command(sim, "arm", "root", -0.4);
        command(sim, "arm", "root", 0.);
        command(sim, "arm", "tip", 0.);
        sim.step_exact(1000).unwrap();
    }
    assert_eq!(bodies(&saturated), bodies(&bounded));
    assert_eq!(
        state(&saturated, "arm", "tip"),
        state(&bounded, "arm", "tip")
    );
    continuing.step_exact(1000).unwrap();
    assert!(
        state(&continuing, "arm", "root").position
            > state(&saturated, "arm", "root").position + 1e-7,
        "zero={:?}, retained={:?}",
        state(&saturated, "arm", "root"),
        state(&continuing, "arm", "root")
    );
    let mut latest = load(&text);
    let mut direct = load(&text);
    command(&mut latest, "arm", "root", -0.5);
    command(&mut latest, "arm", "root", 0.4);
    command(&mut direct, "arm", "root", 0.4);
    for sim in [&mut latest, &mut direct] {
        sim.step_exact(1000).unwrap();
    }
    assert_eq!(bodies(&latest), bodies(&direct));
    assert_eq!(state(&latest, "arm", "root"), state(&direct, "arm", "root"));
}

#[test]
fn principal_axis_motor_effort_uses_centered_explicit_inertia_and_not_density() {
    for axis in AXES {
        let complete = arm("arm", [axis, axis], [4, 4], true, 2., 1., "0 0 0 0 0 0");
        // Only the world-root joint is powered. The retained unactuated tip
        // permits no force/damping requirement and stays a free scalar DoF.
        let start = complete.rfind("<plugin").unwrap();
        let root_only = format!("{}</model>", &complete[..start]);
        let tip_start = root_only.find("<joint name='tip'").unwrap();
        let tip_end =
            tip_start + root_only[tip_start..].find("</joint>").unwrap() + "</joint>".len();
        let passive = root_only[tip_start..tip_end].replace(
            "<limit><effort>1000</effort><velocity>4</velocity></limit>",
            "",
        );
        assert_ne!(passive, &root_only[tip_start..tip_end]);
        let mut root_only = root_only;
        root_only.replace_range(tip_start..tip_end, &passive);
        let root_only = root_only.replace("<effort>1000</effort>", "<effort>0.2</effort>");
        let low = world(&root_only);
        let high_density = low.replace("<density>1</density>", "<density>1000000</density>");
        let heavy = low.replace(
            "<ixx>2</ixx><iyy>2</iyy><izz>2</izz>",
            "<ixx>4</ixx><iyy>4</iyy><izz>4</izz>",
        );
        let mut a = load(&low);
        let mut b = load(&high_density);
        let mut c = load(&heavy);
        for sim in [&mut a, &mut b, &mut c] {
            command(sim, "arm", "root", 100.);
            sim.step_exact(1).unwrap();
        }
        let light = state(&a, "arm", "root").velocity;
        let heavier = state(&c, "arm", "root").velocity;
        assert!(
            light > 0. && light <= 0.2 * 0.001 / 2. + 1e-12,
            "axis={axis:?}, light={light}, bodies={:?}",
            bodies(&a)
        );
        assert!(
            heavier > 0. && heavier <= 0.2 * 0.001 / 4. + 1e-12 && heavier < light,
            "axis={axis:?}, light={light}, heavier={heavier}"
        );
        assert_eq!(bodies(&a), bodies(&b));
        assert_eq!(state(&a, "arm", "root"), state(&b, "arm", "root"));
    }
}

#[test]
fn principal_axis_resets_preserve_geometry_targets_identities_and_survivors() {
    let axes = [AXES[0], AXES[1]];
    let text = world(
        &(arm("arm", axes, [4, 4], true, 2., 1., "0 0 0 0 0 0")
            + &arm(
                "peer",
                [AXES[1], AXES[2]],
                [2, 4],
                false,
                2.,
                1.,
                "1000 0 0 0 0 0",
            )),
    );
    let mut sim = load(&text);
    let original_keys = ids(&sim).keys().copied().collect::<Vec<_>>();
    command(&mut sim, "arm", "root", 0.4);
    command(&mut sim, "arm", "tip", -0.3);
    sim.step_exact(200).unwrap();
    sim.reset().unwrap();
    assert_eq!(sim.epoch(), 1);
    assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), original_keys);
    check_frames_and_queries(&sim, "arm", axes, [4, 4], DVec3::ZERO, DQuat::IDENTITY);
    sim.step_exact(100).unwrap();
    near(state(&sim, "arm", "root").position, 0.);
    near(state(&sim, "arm", "tip").velocity, 0.);
    command(&mut sim, "arm", "root", 0.5);
    sim.step_exact(200).unwrap();
    // Loaded reset owns pending targets as well as logical/native reconstruction.
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.epoch(), 2);
    check_frames_and_queries(&sim, "arm", axes, [4, 4], DVec3::ZERO, DQuat::IDENTITY);
    sim.step_exact(100).unwrap();
    near(state(&sim, "arm", "root").velocity, 0.);
    let arm_id = id(&sim, "w::arm");
    let old_controller = id(&sim, "w::arm::root");
    let peer_controller = id(&sim, "w::peer::root");
    command(&mut sim, "peer", "root", 0.4);
    remove_model(&mut sim, arm_id).unwrap();
    assert!(matches!(
        set_joint_controller_command(&mut sim, old_controller, 1.),
        Err(JointControllerCommandError::MissingController(_))
    ));
    assert!(joint_controller_binding_matches(&sim, peer_controller));
    assert!(
        initial_joint_controller_configurations_with_identity(&sim)
            .unwrap()
            .any(|row| row.id == old_controller)
    );
    let before = state(&sim, "peer", "root").position;
    sim.step_exact(200).unwrap();
    assert!(
        state(&sim, "peer", "root").position > before,
        "before={before}, after={:?}",
        state(&sim, "peer", "root")
    );
    let old_native = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<BodyHandle>().map(BodyHandle::id))
        .max()
        .unwrap();
    let extra = sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [9000., 0., 10.],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(extra > old_native);
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove(extra)
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), original_keys);
    assert!(joint_controller_binding_matches(&sim, old_controller));
    let fresh = sim.world_mut().spawn_empty().id();
    assert!(assign_id(sim.world_mut(), fresh).unwrap() > *original_keys.last().unwrap());
    let after = sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.1 },
                position: [9000., 0., 10.],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(after > extra);
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove(after)
    );
    // A separate bare backend reconstruction proves geometry/native motors only.
    // It neither consumes Scene pending input nor promises synchronized ECS poses.
    let mut bare = load(&text);
    let original_handles = [body(&bare, "w::arm::base"), body(&bare, "w::arm::child")];
    let initial_physics = bare.world().resource::<PhysicsBackend>().initial_state();
    command(&mut bare, "arm", "root", 0.5);
    command(&mut bare, "arm", "tip", -0.3);
    bare.step_exact(200).unwrap();
    let prepared = bare
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial_physics)
        .unwrap();
    bare.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    assert_eq!(
        [body(&bare, "w::arm::base"), body(&bare, "w::arm::child")],
        original_handles
    );
    near(state(&bare, "arm", "root").position, 0.);
    near(state(&bare, "arm", "tip").velocity, 0.);
    assert_eq!(
        query(&bare, DVec3::new(3.5, 0., 0.), -DVec3::X, 1., None),
        Some(1.)
    );
}

#[test]
fn principal_axis_late_owner_faults_are_atomic_and_terminal_faults_do_not_reset() {
    let models = (0..8)
        .map(|i| {
            arm(
                &format!("a{i}"),
                [AXES[i % 3], AXES[(i + 1) % 3]],
                [2, 2],
                true,
                2.,
                1.,
                &format!("{} 0 0 0 0 0", i * 1000),
            )
        })
        .collect::<String>();
    let text = world(&models);
    let mut repaired = load(&text);
    let mut control = load(&text);
    assert_eq!(
        initial_joint_controller_configurations_with_identity(&repaired)
            .unwrap()
            .count(),
        16
    );
    for sim in [&mut repaired, &mut control] {
        for i in 0..8 {
            command(sim, &format!("a{i}"), "root", 0.4);
            command(sim, &format!("a{i}"), "tip", -0.3);
        }
    }
    let later = named(&repaired, "w::a7::tip");
    let saved = repaired
        .world()
        .get::<JointControllerConfiguration>(later)
        .unwrap()
        .clone();
    let mut bad = saved.clone();
    bad.topic = "/forged/principal".into();
    repaired.world_mut().entity_mut(later).insert(bad);
    let before = bodies(&repaired);
    let clock = (repaired.elapsed(), repaired.epoch());
    let early = id(&repaired, "w::a0::root");
    assert!(matches!(
        set_joint_controller_command(&mut repaired, early, -0.5),
        Err(JointControllerCommandError::InvalidPhysics)
    ));
    assert_eq!(bodies(&repaired), before);
    assert_eq!((repaired.elapsed(), repaired.epoch()), clock);
    repaired.world_mut().entity_mut(later).insert(saved);
    for sim in [&mut repaired, &mut control] {
        sim.step_exact(100).unwrap();
    }
    assert_eq!(bodies(&repaired), bodies(&control));
    for fault in 0..4 {
        let mut sim = load(&world(&arm(
            "arm",
            [AXES[0], AXES[1]],
            [4, 4],
            true,
            2.,
            1.,
            "0 0 0 0 0 0",
        )));
        command(&mut sim, "arm", "root", 0.5);
        let joint = named(&sim, "w::arm::tip");
        match fault {
            0 => {
                sim.world_mut().entity_mut(joint).remove::<JointSample>();
            }
            1 => {
                sim.world_mut()
                    .entity_mut(joint)
                    .insert(Name("w::arm::forged".into()));
            }
            2 => {
                sim.world_mut()
                    .entity_mut(joint)
                    .insert(EntityKind::Collision);
            }
            _ => {
                let other = *sim
                    .world()
                    .get::<JointHandle>(named(&sim, "w::arm::root"))
                    .unwrap();
                sim.world_mut().entity_mut(joint).insert(other);
            }
        }
        let before = bodies(&sim);
        let clock = (sim.elapsed(), sim.epoch());
        assert!(sim.step_exact(1).is_err());
        assert_eq!(bodies(&sim), before);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
        assert!(matches!(sim.step_exact(1), Err(StepError::Faulted)));
        assert!(matches!(sim.reset(), Err(StepError::Faulted)));
        assert_eq!(bodies(&sim), before);
    }
}

#[test]
fn principal_axis_motor_contacts_have_a_positive_gap_and_quiet_clear_controls() {
    let axes = [AXES[1], AXES[0]];
    let model = arm("arm", axes, [2, 2], true, 2., 1., "0 0 0 0 0 0");
    let probe = |x: f64| {
        format!(
            "<model name='probe'><pose>{x} 0 -1.006 0 0 0</pose><link name='body'>{}<collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='v'><geometry><sphere><radius>0.5</radius></sphere></geometry></visual></link></model>",
            inertia(2.)
        )
    };
    // The base Sphere centered [2,0,0] and probe have a .006 positive gap.
    // Positive Y motion moves that sphere toward -Z. No gravity or initial overlap
    // can move the probe in the quiet world; the clear world separates X by 18.
    let mut driven = load(&world(&(model.clone() + &probe(2.))));
    let mut quiet = load(&world(&(model.clone() + &probe(2.))));
    let mut clear = load(&world(&(model + &probe(20.))));
    for sim in [&mut driven, &mut clear] {
        command(sim, "arm", "root", 0.5);
    }
    for sim in [&mut driven, &mut quiet, &mut clear] {
        sim.step_exact(1500).unwrap();
    }
    let observed = |sim: &Simulation| {
        sim.world()
            .resource::<PhysicsBackend>()
            .state(body(sim, "w::probe::body"))
            .unwrap()
    };
    let a = observed(&driven);
    let b = observed(&quiet);
    let c = observed(&clear);
    assert!(
        a.position[2] < b.position[2] - 1e-7,
        "driven={a:?}, quiet={b:?}, clear={c:?}, joint={:?}",
        state(&driven, "arm", "root")
    );
    for (value, initial) in [(b, [2., 0., -1.006]), (c, [20., 0., -1.006])] {
        assert!(
            (DVec3::from_array(value.position) - DVec3::from_array(initial))
                .abs()
                .max_element()
                < 1e-7,
            "control={value:?}"
        );
    }
    assert!(
        state(&driven, "arm", "root").position > 1e-7,
        "joint={:?}",
        state(&driven, "arm", "root")
    );
}

#[test]
fn principal_axis_admission_keeps_negative_nonprincipal_frames_and_owner_domains_closed() {
    let base = world(&arm(
        "arm",
        [AXES[0], AXES[1]],
        [4, 4],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    ));
    let refused = |text: &str, feature| {
        assert_ne!(text, base);
        assert!(
            matches!(load_world_with_sdf_step(text, "principal-refusal", "w", &sdformat::NoResources, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature: actual, .. }) if actual == feature),
            "expected={feature}, input={text}"
        );
    };
    // Preserve all three exact formerly refused XML inputs as positive native
    // controls; each uses the same second axis, geometry, commands and bounds.
    // Generic transformed curved queries use a wide cap and analytic distance;
    // the unchanged mature helper's binary endpoint premises are not reused.
    let check_signed = |sim: &Simulation, axes: [[f64; 3]; 2]| {
        let expected = frames(sim, "arm", axes, DVec3::ZERO, DQuat::IDENTITY);
        let first = state(sim, "arm", "root");
        let root_omega = DVec3::from_array(axes[0]) * first.velocity;
        for (index, link) in ["base", "child"].into_iter().enumerate() {
            let name = format!("w::arm::{link}");
            let (position, basis) = expected[index];
            let handle = body(sim, &name);
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(handle)
                .unwrap();
            let logical = sim.world().get::<WorldPose>(named(sim, &name)).unwrap();
            vector(DVec3::from_array(native.position), position);
            vector(DVec3::from_array(logical.translation), position);
            let omega = if index == 0 {
                root_omega
            } else {
                root_omega
                    + rotate(axes[0], first.position, DVec3::from_array(axes[1]))
                        * state(sim, "arm", "tip").velocity
            };
            vector(DVec3::from_array(native.angular_velocity), omega);
            vector(
                DVec3::from_array(native.velocity),
                if index == 0 {
                    DVec3::ZERO
                } else {
                    root_omega.cross(position)
                },
            );
            for (v, wanted) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().zip(basis) {
                vector(DQuat::from_array(native.orientation) * v, wanted);
                vector(DQuat::from_array(logical.rotation) * v, wanted);
            }
            let joint = if index == 0 { "root" } else { "tip" };
            let joint_name = format!("w::arm::{joint}");
            assert!(joint_controller_binding_matches(sim, id(sim, &joint_name)));
            let sample = sim
                .world()
                .get::<JointSample>(named(sim, &joint_name))
                .unwrap();
            assert_eq!(sample.state, state(sim, "arm", joint));
            assert_eq!((sample.time, sample.epoch), (sim.elapsed(), sim.epoch()));
            for i in 0..4 {
                let center = position + apply(basis, DVec3::new(2., 0., 4. * i as f64));
                let child = sim
                    .world()
                    .get::<WorldPose>(named(sim, &format!("{name}::c{i}")))
                    .unwrap();
                vector(DVec3::from_array(child.translation), center);
                let yaw = if i == 0 { 0. } else { 0.3 };
                for v in [DVec3::X, DVec3::Y, DVec3::Z] {
                    vector(
                        DQuat::from_array(child.rotation) * v,
                        apply(basis, rotate(AXES[2], yaw, v)),
                    );
                }
                if i == 0 {
                    // Radius .5 Sphere, offset .3: sqrt(.25-.09)=.4.
                    // Other members occupy separate four-unit Z lanes and
                    // the other link is at its hundred-unit pivot offset.
                    let origin = center + basis[0] * 4. + basis[1] * 0.3;
                    near(query(sim, origin, -basis[0], 5., None).unwrap(), 3.6);
                    assert!(query(sim, origin, -basis[0], 5., Some(handle)).is_none());
                }
            }
        }
    };
    for (xyz, axis) in [
        ("-1 0 0", [-1., 0., 0.]),
        ("0 -1 0", [0., -1., 0.]),
        ("0 0 -1", [0., 0., -1.]),
    ] {
        let text = base.replacen("<xyz>1 0 0</xyz>", &format!("<xyz>{xyz}</xyz>"), 1);
        let mut driven = load(&text);
        let mut quiet = load(&text);
        let initial = bodies(&driven);
        let original_ids = ids(&driven);
        let axes = [axis, AXES[1]];
        check_signed(&driven, axes);
        command(&mut driven, "arm", "root", 0.4);
        command(&mut driven, "arm", "tip", 0.3);
        assert_eq!(bodies(&driven), initial);
        driven.step_exact(100).unwrap();
        quiet.step_exact(100).unwrap();
        assert_eq!(ids(&driven), original_ids);
        for joint in ["root", "tip"] {
            let actual = state(&driven, "arm", joint);
            assert!(actual.position.is_finite() && actual.velocity.is_finite());
            assert!(actual.position > 0. && actual.position < 1.);
            assert!(state(&quiet, "arm", joint).position.abs() < 1e-11);
        }
        assert_ne!(bodies(&driven), bodies(&quiet));
        check_signed(&driven, axes);
    }
    for xyz in ["0.5 0.5 0", "1 0.000000000001 0"] {
        refused(
            &base.replacen("<xyz>1 0 0</xyz>", &format!("<xyz>{xyz}</xyz>"), 1),
            "joint controller frames",
        );
    }
    refused(
        &base.replacen("<xyz>", "<xyz expressed_in='base'>", 1),
        "joint controller frames",
    );
    refused(
        &base.replacen("</axis>", "</axis><axis2><xyz>0 0 1</xyz></axis2>", 1),
        "joint controller frames",
    );
    refused(
        &base.replacen("type='revolute'", "type='fixed'", 1),
        "joint controller frames",
    );
    refused(
        &base.replacen(
            "<pose>0 0 100 0 0 0</pose>",
            "<pose>0 0 100 0.1 0 0</pose>",
            1,
        ),
        "joint controller frames",
    );
    refused(
        &base.replacen(
            "canonical_link='base'>",
            "canonical_link='base'><static>true</static>",
            1,
        ),
        "joint controller model domain",
    );
    refused(
        &base.replacen(
            "<use_velocity_commands>true</use_velocity_commands>",
            "<use_velocity_commands>false</use_velocity_commands>",
            1,
        ),
        "joint controller mode",
    );
    let zero = base.replacen("<xyz>1 0 0</xyz>", "<xyz>0 0 0</xyz>", 1);
    assert!(
        load_world_with_sdf_step(
            &zero,
            "principal-zero",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    refused(
        &base.replacen("<parent>world</parent>", "<parent>child</parent>", 1),
        "joint controller chain",
    );
    refused(
        &base.replacen("<effort>1000</effort>", "<effort>0</effort>", 1),
        "joint controller limits",
    );
    refused(
        &base.replacen(
            "<joint_name>tip</joint_name>",
            "<joint_name>root</joint_name>",
            1,
        ),
        "overlapping joint controllers",
    );
    let eight = (0..8)
        .map(|i| {
            arm(
                &format!("a{i}"),
                [AXES[0], AXES[1]],
                [2, 2],
                true,
                2.,
                1.,
                &format!("{} 0 0 0 0 0", i * 1000),
            )
        })
        .collect::<String>();
    let nine = world(
        &(eight
            + &arm(
                "ninth",
                [AXES[0], AXES[1]],
                [2, 2],
                true,
                2.,
                1.,
                "9000 0 0 0 0 0",
            )),
    );
    refused(&nine, "multiple joint-controller chains");
    assert!(matches!(
        load_world_with_sdf_step(
            &base,
            "principal-tpe",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        ),
        Err(LoadError::Unsupported {
            feature: "joint controllers require a top-level dynamic Rapier model",
            ..
        })
    ));
    // SDF normalizes authored axis lengths before this exact DOM gate. Preserve
    // that existing policy, rather than adding a new lexical-magnitude rule.
    let scaled = base.replacen("<xyz>1 0 0</xyz>", "<xyz>2 0 0</xyz>", 1);
    let parsed =
        sdformat::load_str(&scaled, "principal-normalized", &sdformat::NoResources).unwrap();
    assert_eq!(
        parsed.worlds[0].models[0].joints[0]
            .axis
            .as_ref()
            .unwrap()
            .xyz,
        AXES[0]
    );
    let mut unit = load(&base);
    let mut normalized = load(&scaled);
    for twin in [&mut unit, &mut normalized] {
        command(twin, "arm", "root", 0.4);
        command(twin, "arm", "tip", 0.3);
        twin.step_exact(100).unwrap();
    }
    assert_eq!(bodies(&unit), bodies(&normalized));
    let mut sim = load(&base);
    let owner = id(&sim, "w::arm");
    let sdf = sdformat::load_str(&base, "principal-template", &sdformat::NoResources).unwrap();
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        model_state(&sim, owner),
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
    assert!(matches!(
        set_joint_controller_command(&mut sim, u64::MAX, 1.),
        Err(JointControllerCommandError::MissingController(_))
    ));
}
