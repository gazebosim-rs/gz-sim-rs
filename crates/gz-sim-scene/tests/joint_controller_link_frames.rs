// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rotated LINK frames reuse the existing native anchors and scalar controllers.
//! References use independently measured coordinates, not motor targets or body poses.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodySpec, BodyState, JointState, PhysicsBackend, RotationSpec, Shape};
use gz_sim_core::{EntityId, Simulation, StepError, assign_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, JointControllerCommandError, JointControllerConfiguration,
    JointControllerMode, JointHandle, JointSample, LoadError, LocalPose, ModelState, Name,
    PhysicsEngine, WorldEditError, WorldPose,
    initial_joint_controller_configurations_with_identity, joint_controller_binding_matches,
    load_world_with_sdf_step, model_state, prepare_model_template, remove_model,
    reset_loaded_world, set_joint_controller_command, set_model_state,
};
use std::{collections::BTreeMap, time::Duration};

const POSITION: &str = "<plugin name='gz::sim::systems::JointPositionController' filename='gz-sim-joint-position-controller-system'><joint_name>JOINT</joint_name><use_velocity_commands>true</use_velocity_commands><cmd_max>4</cmd_max><initial_position>0</initial_position></plugin>";
const VELOCITY: &str = "<plugin name='gz::sim::systems::JointController' filename='gz-sim-joint-controller-system'><joint_name>JOINT</joint_name><initial_velocity>0</initial_velocity></plugin>";
// Rational unit vectors keep the reference independent of a config normalizer.
// They span nonprincipal planes, negative directions and noncoaxial chains.
const AXES: [[f64; 3]; 6] = [
    [0., 0.6, 0.8],
    [0.6, 0.8, 0.],
    [0.8, 0., 0.6],
    [0., -0.6, -0.8],
    [-0.6, -0.8, 0.],
    [-0.8, 0., -0.6],
];
const OFFSET: f64 = 100.;
// Binary local translations and distinct noncommuting unit frame rotations.
// LINK and JOINT rotations are distinct, noncommuting authored transforms.
const PIVOTS: [[f64; 3]; 2] = [[0.25, -0.125, 0.5], [-0.25, 0.125, -0.5]];
const FRAME_POSES: [&str; 2] = [
    "0.25 -0.125 0.5 0.3 -0.2 0.1",
    "-0.25 0.125 -0.5 -0.2 0.1 -0.4",
];
// These local LINK poses are relative to the model, not to the parent link.
// Their rotations change each joint's initial world axis independently.
const LINK_POSES: [&str; 2] = [
    "0.125 -0.25 0.5 -0.15 0.25 -0.1",
    "0.5 0.25 100 0.2 -0.1 0.15",
];
fn link_origin(index: usize) -> DVec3 {
    if index == 0 {
        DVec3::new(0.125, -0.25, 0.5)
    } else {
        DVec3::new(0.5, 0.25, OFFSET)
    }
}
fn link_q(index: usize) -> DQuat {
    if index == 0 {
        DQuat::from_euler(glam::EulerRot::ZYX, -0.1, 0.25, -0.15)
    } else {
        DQuat::from_euler(glam::EulerRot::ZYX, 0.15, -0.1, 0.2)
    }
}
fn frame_q(index: usize) -> DQuat {
    if index == 0 {
        DQuat::from_euler(glam::EulerRot::ZYX, 0.1, -0.2, 0.3)
    } else {
        DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.1, -0.2)
    }
}
fn effective_axis(index: usize, axis: [f64; 3]) -> [f64; 3] {
    (link_q(index) * frame_q(index) * DVec3::from_array(axis)).to_array()
}

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
    let joint = |index: usize, name: &str, parent: &str, child: &str, axis: [f64; 3]| {
        format!(
            "<joint name='{name}' type='revolute'><parent>{parent}</parent><child>{child}</child><pose relative_to='{child}'>{}</pose><axis><xyz>{} {} {}</xyz><limit><effort>1000</effort><velocity>4</velocity></limit></axis></joint>",
            FRAME_POSES[index], axis[0], axis[1], axis[2]
        )
    };
    format!(
        "<model name='{name}' canonical_link='base'><pose>{pose}</pose><link name='base'><pose>{}</pose>{}{}<visual name='v'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual></link><link name='child'><pose>{}</pose>{}{}<visual name='v'><geometry><sphere><radius>0.1</radius></sphere></geometry></visual></link>{}{}{root}{tip}</model>",
        LINK_POSES[0],
        inertia(moment),
        children(counts[0], density),
        LINK_POSES[1],
        inertia(moment),
        children(counts[1], density),
        joint(0, "root", "world", "base", axes[0]),
        joint(1, "tip", "base", "child", axes[1])
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
        "joint-link-frame-controller",
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
// At angle zero link origins are L_i, orientations Q_i. The world-root
// pivot is L0+Q0*t0; the second pivot is L1+Q1*t1. Rodrigues about these
// initial model-frame axes/pivots composes root then child motion. Body poses
// are outputs of this reference, never its inputs.
fn frames_with_links(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    origin: DVec3,
    model_q: DQuat,
    links: [DQuat; 2],
) -> [(DVec3, [DVec3; 3]); 2] {
    let first = state(sim, name, "root").position;
    let second = state(sim, name, "tip").position;
    let u0 = (links[0] * frame_q(0) * DVec3::from_array(axes[0])).to_array();
    let u1 = (links[1] * frame_q(1) * DVec3::from_array(axes[1])).to_array();
    let l0 = link_origin(0);
    let l1 = link_origin(1);
    let p0 = l0 + links[0] * DVec3::from_array(PIVOTS[0]);
    let p1 = l1 + links[1] * DVec3::from_array(PIVOTS[1]);
    let basis = [DVec3::X, DVec3::Y, DVec3::Z];
    [
        (
            origin + model_q * (p0 + rotate(u0, first, l0 - p0)),
            basis.map(|v| model_q * rotate(u0, first, links[0] * v)),
        ),
        (
            origin + model_q * (p0 + rotate(u0, first, p1 - p0 + rotate(u1, second, l1 - p1))),
            basis.map(|v| model_q * rotate(u0, first, rotate(u1, second, links[1] * v))),
        ),
    ]
}
fn frames(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    origin: DVec3,
    model_q: DQuat,
) -> [(DVec3, [DVec3; 3]); 2] {
    frames_with_links(sim, name, axes, origin, model_q, [link_q(0), link_q(1)])
}
fn anchors_with_links(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    origin: DVec3,
    model_q: DQuat,
    links: [DQuat; 2],
) -> [DVec3; 2] {
    let p0 = link_origin(0) + links[0] * DVec3::from_array(PIVOTS[0]);
    let p1 = link_origin(1) + links[1] * DVec3::from_array(PIVOTS[1]);
    let u0 = (links[0] * frame_q(0) * DVec3::from_array(axes[0])).to_array();
    [
        origin + model_q * p0,
        origin + model_q * (p0 + rotate(u0, state(sim, name, "root").position, p1 - p0)),
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
    check_with_link_frames(
        sim,
        name,
        axes,
        counts,
        origin,
        model_q,
        [link_q(0), link_q(1)],
    );
}
fn check_with_link_frames(
    sim: &Simulation,
    name: &str,
    axes: [[f64; 3]; 2],
    counts: [usize; 2],
    origin: DVec3,
    model_q: DQuat,
    links: [DQuat; 2],
) {
    let expected = frames_with_links(sim, name, axes, origin, model_q, links);
    let initial_axes: [[f64; 3]; 2] =
        std::array::from_fn(|i| (links[i] * frame_q(i) * DVec3::from_array(axes[i])).to_array());
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
        let joint_anchors = anchors_with_links(sim, name, axes, origin, model_q, links);
        let root_omega = model_q * DVec3::from_array(initial_axes[0]) * first.velocity;
        let tip_omega = model_q
            * rotate(
                initial_axes[0],
                first.position,
                DVec3::from_array(initial_axes[1]),
            )
            * state(sim, name, "tip").velocity;
        let expected_omega = if index == 0 {
            root_omega
        } else {
            root_omega + tip_omega
        };
        vector(DVec3::from_array(native.angular_velocity), expected_omega);
        vector(
            DVec3::from_array(native.velocity),
            root_omega.cross(position - joint_anchors[0])
                + if index == 0 {
                    DVec3::ZERO
                } else {
                    tip_omega.cross(position - joint_anchors[1])
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
        // Joint publication remains attached to the child at the resolved local
        // frame. Its origin is the physical anchor, not the link origin.
        let joint_pose = sim
            .world()
            .get::<WorldPose>(named(sim, &format!("w::{name}::{joint}")))
            .unwrap();
        vector(
            DVec3::from_array(joint_pose.translation),
            joint_anchors[index],
        );
        for v in [DVec3::X, DVec3::Y, DVec3::Z] {
            vector(
                DQuat::from_array(joint_pose.rotation) * v,
                apply(basis, frame_q(index) * v),
            );
        }

        for i in 0..counts[index] {
            let local = DVec3::new(2., 0., 4. * i as f64);
            let center = position + apply(basis, local);
            let yaw = if i % 4 == 0 { 0. } else { 0.3 };
            let shape =
                [DVec3::X, DVec3::Y, DVec3::Z].map(|v| apply(basis, rotate([0., 0., 1.], yaw, v)));
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
            // Advected geometry uses a wide cap. Exact binary endpoints are
            // isolated below in an identity/unrotated Sphere fixture.
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
    // ModelAnchor removes the initial canonical LINK transform. A translated,
    // rotated base LINK therefore does not make the model origin equal the link.
    let p0 = link_origin(0) + links[0] * DVec3::from_array(PIVOTS[0]);
    let angle = state(sim, name, "root").position;
    vector(
        DVec3::from_array(model.translation),
        origin + model_q * (p0 + rotate(initial_axes[0], angle, -p0)),
    );
    for (v, wanted) in [DVec3::X, DVec3::Y, DVec3::Z]
        .into_iter()
        .zip([DVec3::X, DVec3::Y, DVec3::Z].map(|v| model_q * rotate(initial_axes[0], angle, v)))
    {
        vector(DQuat::from_array(model.rotation) * v, wanted);
    }
}

fn equivalent_physical_states(a: &Simulation, b: &Simulation) {
    for link in ["base", "child"] {
        let name = format!("w::arm::{link}");
        let one = a
            .world()
            .resource::<PhysicsBackend>()
            .state(body(a, &name))
            .unwrap();
        let two = b
            .world()
            .resource::<PhysicsBackend>()
            .state(body(b, &name))
            .unwrap();
        vector(
            DVec3::from_array(one.position),
            DVec3::from_array(two.position),
        );
        vector(
            DVec3::from_array(one.velocity),
            DVec3::from_array(two.velocity),
        );
        vector(
            DVec3::from_array(one.angular_velocity),
            DVec3::from_array(two.angular_velocity),
        );
        // Native joint frames can choose different transverse bases for opposite
        // axes. Compare physical body bases, not quaternion signs or raw frames.
        for basis in [DVec3::X, DVec3::Y, DVec3::Z] {
            vector(
                DQuat::from_array(one.orientation) * basis,
                DQuat::from_array(two.orientation) * basis,
            );
        }
    }
}

#[test]
fn joint_link_frame_opposite_axes_and_commands_keep_measured_physical_equivalence() {
    for positive_axes in [[AXES[0], AXES[1]], [AXES[1], AXES[2]]] {
        for flipped in [[true, false], [false, true], [true, true]] {
            let signed_axes = std::array::from_fn(|i| {
                positive_axes[i].map(|value| if flipped[i] { -value } else { value })
            });
            for counts in [[1, 1], [2, 4], [16, 2]] {
                for swapped in [false, true] {
                    let identity = positive_axes == [AXES[0], AXES[1]];
                    let (pose, q, origin) = if identity {
                        ("0 0 0 0 0 0", DQuat::IDENTITY, DVec3::ZERO)
                    } else {
                        (
                            "3 -2 8 0.2 -0.3 0.4",
                            DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2),
                            DVec3::new(3., -2., 8.),
                        )
                    };
                    let mut positive = load(&world(&arm(
                        "arm",
                        positive_axes,
                        counts,
                        swapped,
                        2.,
                        1.,
                        pose,
                    )));
                    let mut signed = load(&world(&arm(
                        "arm",
                        signed_axes,
                        counts,
                        swapped,
                        2.,
                        1.,
                        pose,
                    )));
                    let before = [bodies(&positive), bodies(&signed)];
                    for (i, joint, target) in [(0, "root", 0.4), (1, "tip", -0.3)] {
                        command(&mut positive, "arm", joint, target);
                        command(
                            &mut signed,
                            "arm",
                            joint,
                            if flipped[i] { -target } else { target },
                        );
                    }
                    assert_eq!(bodies(&positive), before[0]);
                    assert_eq!(bodies(&signed), before[1]);
                    for steps in [1, 99, 100, 300, 500] {
                        positive.step_exact(steps).unwrap();
                        signed.step_exact(steps).unwrap();
                        equivalent_physical_states(&positive, &signed);
                        for (i, joint) in ["root", "tip"].into_iter().enumerate() {
                            let a = state(&positive, "arm", joint);
                            let b = state(&signed, "arm", joint);
                            let sign = if flipped[i] { -1. } else { 1. };
                            near(a.position, sign * b.position);
                            near(a.velocity, sign * b.velocity);
                        }
                    }
                    assert!(
                        state(&positive, "arm", "root").position > 1e-7,
                        "axes={positive_axes:?}, flip={flipped:?}, positive={:?}, signed={:?}",
                        bodies(&positive),
                        bodies(&signed)
                    );
                    assert!(
                        state(&positive, "arm", "tip").position < -1e-7,
                        "root={:?}, tip={:?}",
                        state(&positive, "arm", "root"),
                        state(&positive, "arm", "tip")
                    );
                    check_frames_and_queries(&positive, "arm", positive_axes, counts, origin, q);
                    check_frames_and_queries(&signed, "arm", signed_axes, counts, origin, q);
                }
            }
        }
    }
}

#[test]
fn joint_link_frame_coordinates_and_samples_remain_continuous_through_reversals() {
    let positive_axes = [AXES[0], AXES[1]];
    let signed_axes = [AXES[3], AXES[4]];
    let mut positive = load(&world(&arm(
        "arm",
        positive_axes,
        [4, 4],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    )));
    let mut signed = load(&world(&arm(
        "arm",
        signed_axes,
        [4, 4],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    )));
    for (root, tip) in [(0.4, -0.3), (-0.4, 0.3), (0., 0.), (0.25, -0.2)] {
        let before = [
            state(&positive, "arm", "root"),
            state(&signed, "arm", "root"),
        ];
        let physical = [bodies(&positive), bodies(&signed)];
        for (sim, sign) in [(&mut positive, 1.), (&mut signed, -1.)] {
            command(sim, "arm", "root", sign * root);
            command(sim, "arm", "tip", sign * tip);
        }
        assert_eq!(state(&positive, "arm", "root"), before[0]);
        assert_eq!(state(&signed, "arm", "root"), before[1]);
        assert_eq!(bodies(&positive), physical[0]);
        assert_eq!(bodies(&signed), physical[1]);
        // The published coordinate is a signed continuous scalar, not an angle
        // reconstructed from the body's quaternion or reset by a new command.
        for _ in 0..50 {
            positive.step_exact(20).unwrap();
            signed.step_exact(20).unwrap();
            equivalent_physical_states(&positive, &signed);
            for joint in ["root", "tip"] {
                let one = state(&positive, "arm", joint);
                let two = state(&signed, "arm", joint);
                near(one.position, -two.position);
                near(one.velocity, -two.velocity);
                let sample = signed
                    .world()
                    .get::<JointSample>(named(&signed, &format!("w::arm::{joint}")))
                    .unwrap();
                assert_eq!(sample.state, two);
                assert_eq!(
                    (sample.time, sample.epoch),
                    (signed.elapsed(), signed.epoch())
                );
            }
        }
        // These longer reversals have no assumed small-angle or ray-lane bound.
        // The independent signed-coordinate frame reference remains valid at
        // every angle; isolated finite-cap queries are covered by the matrix.
        for (sim, axes) in [(&positive, positive_axes), (&signed, signed_axes)] {
            for (link, (position, basis)) in ["base", "child"].into_iter().zip(frames(
                sim,
                "arm",
                axes,
                DVec3::ZERO,
                DQuat::IDENTITY,
            )) {
                let actual = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .state(body(sim, &format!("w::arm::{link}")))
                    .unwrap();
                vector(DVec3::from_array(actual.position), position);
                for (unit, wanted) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().zip(basis) {
                    vector(DQuat::from_array(actual.orientation) * unit, wanted);
                }
            }
        }
    }
}

#[test]
fn joint_link_frame_noncoaxial_chains_move_native_bases_velocities_and_all_children() {
    let q = DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2);
    for axes in [[AXES[3], AXES[1]], [AXES[0], AXES[4]], [AXES[4], AXES[5]]] {
        for counts in [[1, 1], [1, 2], [2, 1], [2, 4], [4, 2], [16, 16]] {
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
                for (joint, wanted) in [
                    (
                        "root",
                        if swapped {
                            JointControllerMode::Velocity
                        } else {
                            JointControllerMode::Position
                        },
                    ),
                    (
                        "tip",
                        if swapped {
                            JointControllerMode::Position
                        } else {
                            JointControllerMode::Velocity
                        },
                    ),
                ] {
                    let cfg = driven
                        .world()
                        .get::<JointControllerConfiguration>(named(
                            &driven,
                            &format!("w::arm::{joint}"),
                        ))
                        .unwrap();
                    assert_eq!(cfg.mode, wanted);
                    assert_eq!(cfg.joint_name, joint);
                }
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
fn joint_link_frame_commands_keep_clamp_latest_zero_pause_and_invalid_equivalence() {
    let text = world(&arm(
        "arm",
        [AXES[3], AXES[4]],
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
fn joint_link_frame_motor_effort_respects_explicit_isotropic_inertia_and_density_twins() {
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
fn joint_link_frame_resets_preserve_geometry_targets_identities_and_survivors() {
    let axes = [AXES[3], AXES[4]];
    let text = world(
        &(arm("arm", axes, [4, 4], true, 2., 1., "0 0 0 0 0 0")
            + &arm(
                "peer",
                [AXES[4], AXES[5]],
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
    near(
        query(
            &bare,
            link_origin(0) + link_q(0) * DVec3::new(3.5, 0., 0.),
            -(link_q(0) * DVec3::X),
            2.,
            None,
        )
        .unwrap(),
        1.,
    );
}

#[test]
fn joint_link_frame_eight_owner_late_faults_keep_pending_atomicity_and_terminal_latch() {
    let models = (0..8)
        .map(|i| {
            arm(
                &format!("a{i}"),
                [AXES[3 + i % 3], AXES[3 + (i + 1) % 3]],
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
    bad.topic = "/forged/signed".into();
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
            [AXES[3], AXES[4]],
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
fn joint_link_frame_motor_contact_moves_an_independent_probe_with_quiet_clear_controls() {
    let axes = [AXES[0], AXES[1]];
    let center = link_origin(0) + link_q(0) * DVec3::new(2., 0., 0.);
    let pivot = link_origin(0) + link_q(0) * DVec3::from_array(PIVOTS[0]);
    let tangent = DVec3::from_array(effective_axis(0, axes[0]))
        .cross(center - pivot)
        .normalize();
    let initial = center + tangent * 1.006;
    let model = arm("arm", axes, [2, 2], true, 2., 1., "0 0 0 0 0 0");
    let probe = |position: DVec3| {
        format!(
            "<model name='probe'><pose>{} {} {} 0 0 0</pose><link name='body'>{}<collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='v'><geometry><sphere><radius>0.5</radius></sphere></geometry></visual></link></model>",
            position.x,
            position.y,
            position.z,
            inertia(2.)
        )
    };
    // Sphere radii .5+.5 and separation1.006 give a positive .006 gap.
    // The motor sweeps the original sphere toward that tangent; other children
    // occupy separate rotated local-Z lanes; the child link starts100 units away.
    let clear_initial = initial + DVec3::X * 20.;
    let mut driven = load(&world(&(model.clone() + &probe(initial))));
    let mut quiet = load(&world(&(model.clone() + &probe(initial))));
    let mut clear = load(&world(&(model + &probe(clear_initial))));
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
        (DVec3::from_array(a.position) - DVec3::from_array(b.position)).dot(tangent) > 1e-7,
        "driven={a:?}, quiet={b:?}, clear={c:?}, joint={:?}",
        state(&driven, "arm", "root")
    );
    for (value, start) in [(b, initial), (c, clear_initial)] {
        assert!(
            (DVec3::from_array(value.position) - start)
                .abs()
                .max_element()
                < 1e-7,
            "control={value:?}, initial={start:?}"
        );
    }
    assert!(
        state(&driven, "arm", "root").position > 1e-7,
        "joint={:?}",
        state(&driven, "arm", "root")
    );
}

#[test]
fn joint_link_frame_axis_admission_keeps_frames_topology_limits_and_other_domains_closed() {
    let base = world(&arm(
        "arm",
        [AXES[3], AXES[4]],
        [4, 4],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    ));
    let refused = |text: &str, feature| {
        assert_ne!(text, base);
        assert!(
            matches!(load_world_with_sdf_step(text, "joint-frame-refusal", "w", &sdformat::NoResources, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature: actual, .. }) if actual == feature),
            "expected={feature}, input={text}"
        );
    };
    refused(
        &base.replacen("<pose relative_to='base'>", "<pose relative_to='child'>", 1),
        "joint controller frames",
    );
    refused(
        &base.replacen(FRAME_POSES[0], "1e300 0 0 0 0 0", 1),
        "joint controller frames",
    );
    // A zero authored quaternion fails the SDF parser before config admission.
    let zero_q = base.replacen(
        &format!("<pose relative_to='base'>{}</pose>", FRAME_POSES[0]),
        "<pose relative_to='base' rotation_format='quat_xyzw'>0 0 0 0 0 0 0</pose>",
        1,
    );
    assert!(
        matches!(load_world_with_sdf_step(&zero_q,"joint-frame-parser","w",&sdformat::NoResources,PhysicsEngine::Rapier),
        Err(LoadError::Sdf(error)) if error.code==sdformat::ErrorCode::InvalidValue)
    );
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
            &format!("<pose>{}</pose>", LINK_POSES[1]),
            &format!("<pose relative_to='base'>{}</pose>", LINK_POSES[1]),
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
    for xyz in ["0 0 0", "nan 0 0", "-inf 0 0"] {
        let bad = base.replacen("<xyz>0 -0.6 -0.8</xyz>", &format!("<xyz>{xyz}</xyz>"), 1);
        assert!(
            matches!(
                load_world_with_sdf_step(&bad, "arbitrary-parser", "w", &sdformat::NoResources, PhysicsEngine::Rapier),
                Err(LoadError::Sdf(error)) if error.code == sdformat::ErrorCode::InvalidValue
            ),
            "invalid axis must retain its earlier SDF route: {xyz}"
        );
    }
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
            "<velocity>4</velocity>",
            "<lower>0</lower><velocity>4</velocity>",
            1,
        ),
        "joint limits",
    );
    refused(
        &base.replacen(
            "<initial_velocity>0</initial_velocity>",
            "<use_force_commands>true</use_force_commands><initial_velocity>0</initial_velocity>",
            1,
        ),
        "joint controller mode",
    );
    refused(
        &base.replacen(
            "<joint_name>tip</joint_name>",
            "<joint_name>root</joint_name>",
            1,
        ),
        "overlapping joint controllers",
    );
    let offset_com = world(&arm(
        "arm",
        [AXES[0], AXES[1]],
        [1, 1],
        true,
        2.,
        1.,
        "0 0 0 0 0 0",
    ))
    .replacen("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>", 1);
    let error = load_world_with_sdf_step(
        &offset_com,
        "joint-frame-offset-com",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .err()
    .unwrap();
    // The loaded-link Scene guard rejects this same .1 COM before native joint
    // construction; it does not exercise the later native articulation guard.
    assert!(
        matches!(
            &error,
            LoadError::Unsupported {
                feature: "offset center of mass",
                ..
            }
        ),
        "{error:?}"
    );
    let eight = (0..8)
        .map(|i| {
            arm(
                &format!("a{i}"),
                [AXES[3], AXES[4]],
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
                [AXES[3], AXES[4]],
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
            "arbitrary-tpe",
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
    let scaled = base.replacen("<xyz>0 -0.6 -0.8</xyz>", "<xyz>0 -3 -4</xyz>", 1);
    let parsed =
        sdformat::load_str(&scaled, "arbitrary-normalized", &sdformat::NoResources).unwrap();
    assert_eq!(
        parsed.worlds[0].models[0].joints[0]
            .axis
            .as_ref()
            .unwrap()
            .xyz,
        AXES[3]
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
    let sdf = sdformat::load_str(&base, "arbitrary-template", &sdformat::NoResources).unwrap();
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

#[test]
fn joint_link_frame_link_quaternion_signs_and_joint_defaults_share_physical_trajectories() {
    for counts in [[1, 1], [2, 4], [16, 16]] {
        for swapped in [false, true] {
            let axes = [AXES[0], AXES[4]];
            let text = world(&arm(
                "arm",
                axes,
                counts,
                swapped,
                2.,
                1.,
                "3 -2 8 0.2 -0.3 0.4",
            ));
            let implicit = text
                .replace(" relative_to='base'", "")
                .replace(" relative_to='child'", "");
            let mut quaternion = text.clone();
            let mut negative = text.clone();
            for (i, link_pose) in LINK_POSES.iter().enumerate() {
                let q = link_q(i).to_array();
                let t = link_origin(i).to_array();
                let old = format!("<pose>{link_pose}</pose>");
                let explicit = format!(
                    "<pose rotation_format='quat_xyzw'>{} {} {} {} {} {} {}</pose>",
                    t[0], t[1], t[2], q[0], q[1], q[2], q[3]
                );
                let signed = format!(
                    "<pose rotation_format='quat_xyzw'>{} {} {} {} {} {} {}</pose>",
                    t[0], t[1], t[2], -q[0], -q[1], -q[2], -q[3]
                );
                assert!(quaternion.contains(&old));
                quaternion = quaternion.replacen(&old, &explicit, 1);
                negative = negative.replacen(&old, &signed, 1);
            }
            let mut a = load(&text);
            let mut b = load(&implicit);
            let mut c = load(&quaternion);
            let mut d = load(&negative);
            for sim in [&mut a, &mut b, &mut c, &mut d] {
                command(sim, "arm", "root", 0.4);
                command(sim, "arm", "tip", -0.3);
            }
            for steps in [1, 99, 400, 500] {
                for sim in [&mut a, &mut b, &mut c, &mut d] {
                    sim.step_exact(steps).unwrap();
                }
                for twin in [&b, &c, &d] {
                    equivalent_physical_states(&a, twin);
                    for joint in ["root", "tip"] {
                        near(
                            state(&a, "arm", joint).position,
                            state(twin, "arm", joint).position,
                        );
                        near(
                            state(&a, "arm", joint).velocity,
                            state(twin, "arm", joint).velocity,
                        );
                    }
                }
            }
            check_frames_and_queries(
                &a,
                "arm",
                axes,
                counts,
                DVec3::new(3., -2., 8.),
                DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2),
            );
        }
    }
}

fn with_link_rotations(text: &str, links: [DQuat; 2]) -> String {
    let mut result = text.to_owned();
    for i in 0..2 {
        let t = link_origin(i).to_array();
        let q = links[i].to_array();
        let old = format!("<pose>{}</pose>", LINK_POSES[i]);
        let new = format!(
            "<pose rotation_format='quat_xyzw'>{} {} {} {} {} {} {}</pose>",
            t[0], t[1], t[2], q[0], q[1], q[2], q[3]
        );
        assert!(result.contains(&old));
        result = result.replacen(&old, &new, 1);
    }
    result
}

#[test]
fn joint_link_frame_each_rotated_link_and_noncommuting_pair_obey_independent_kinematics() {
    let axes = [AXES[0], AXES[4]];
    let origin = DVec3::new(3., -2., 8.);
    let model_q = DQuat::from_euler(glam::EulerRot::ZYX, 0.4, -0.3, 0.2);
    for links in [
        [link_q(0), DQuat::IDENTITY],
        [DQuat::IDENTITY, link_q(1)],
        [link_q(0), link_q(1)],
    ] {
        for counts in [[1, 1], [2, 4], [16, 16]] {
            for swapped in [false, true] {
                let text = with_link_rotations(
                    &world(&arm(
                        "arm",
                        axes,
                        counts,
                        swapped,
                        2.,
                        1.,
                        "3 -2 8 0.2 -0.3 0.4",
                    )),
                    links,
                );
                let mut driven = load(&text);
                let mut quiet = load(&text);
                check_with_link_frames(&driven, "arm", axes, counts, origin, model_q, links);
                command(&mut driven, "arm", "root", 0.4);
                command(&mut driven, "arm", "tip", -0.3);
                for steps in [1, 99, 400, 500] {
                    driven.step_exact(steps).unwrap();
                    quiet.step_exact(steps).unwrap();
                    check_with_link_frames(&driven, "arm", axes, counts, origin, model_q, links);
                    check_with_link_frames(&quiet, "arm", axes, counts, origin, model_q, links);
                }
                let root = state(&driven, "arm", "root");
                let tip = state(&driven, "arm", "tip");
                assert!(
                    root.position > state(&quiet, "arm", "root").position + 1e-7
                        && tip.position < state(&quiet, "arm", "tip").position - 1e-7,
                    "links={links:?}, root={root:?}, tip={tip:?}, quiet={:?}",
                    bodies(&quiet)
                );
                assert_ne!(bodies(&driven), bodies(&quiet));
            }
        }
    }
}

#[test]
fn joint_link_frame_binary_endpoint_witness_is_separate_from_rotated_global_queries() {
    // Only the second LINK is rotated. The first Sphere's world center, radius,
    // origin and direction are binary identity arithmetic, before any step.
    let text = with_link_rotations(
        &world(&arm(
            "arm",
            [AXES[0], AXES[1]],
            [4, 4],
            true,
            2.,
            1.,
            "0 0 0 0 0 0",
        )),
        [DQuat::IDENTITY, link_q(1)],
    );
    let mut sim = load(&text);
    let center = link_origin(0) + DVec3::new(2., 0., 0.);
    let origin = center + DVec3::X * 1.5;
    let owner = body(&sim, "w::arm::base");
    assert_eq!(query(&sim, origin, -DVec3::X, 1., None), Some(1.));
    assert_eq!(
        query(&sim, origin, -DVec3::X, 1_f64.next_down(), None),
        None
    );
    assert_eq!(query(&sim, center, DVec3::X, 0.125, None), Some(0.));
    assert_eq!(query(&sim, origin, -DVec3::X, 1., Some(owner)), None);
    let closer = sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.125 },
                position: (center + DVec3::X).to_array(),
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert_eq!(query(&sim, origin, -DVec3::X, 1., None), Some(0.375));
    assert_eq!(query(&sim, origin, -DVec3::X, 1., Some(owner)), Some(0.375));
    assert_eq!(query(&sim, origin, -DVec3::X, 1., Some(closer)), Some(1.));
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove(closer)
    );
}
