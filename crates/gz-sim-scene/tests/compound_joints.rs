// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Passive revolute constraints retain the link frame and every compound member.
use bevy_ecs::prelude::Entity;
use glam::{DQuat, DVec3, EulerRot};
use gz_physics::{BodyId, BodySpec, BodyState, PhysicsBackend, PhysicsError, RotationSpec, Shape};
use gz_sim_core::{EntityId, Simulation, assign_id, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, DiffDriveSample, InputError, JointControllerConfiguration, JointControllerMode,
    JointHandle, JointSample, LoadError, LocalPose, ModelState, Name, PhysicsEngine, RemoveError,
    WorldEditError, WorldPose, diff_drive_binding_matches, joint_controller_binding_matches,
    load_world_with_sdf_step, model_state, queue_joint_effort, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_diff_drive_command,
    set_joint_controller_command, set_model_state,
};
use sdformat::NoResources;
use std::collections::BTreeMap;
use std::time::Duration;

const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial>";
const MODEL_POSE: &str = "2 -3 5 0.2 -0.3 0.4";

fn world(models: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}

fn members(count: usize) -> String {
    (0..count).map(|i| {
        let geometry = match i % 4 {
            0 => "<box><size>0.5 0.5 0.5</size></box>",
            1 => "<sphere><radius>0.25</radius></sphere>",
            2 => "<cylinder><radius>0.25</radius><length>0.5</length></cylinder>",
            _ => "<capsule><radius>0.25</radius><length>0.5</length></capsule>",
        };
        format!("<collision name='c{i}'><pose>{} 0 0 0 1.5707963267948966 0</pose><geometry>{geometry}</geometry></collision>", i * 3)
    }).collect()
}

fn link(name: &str, count: usize) -> String {
    format!(
        "<link name='{name}'>{INERTIA}{}<visual name='visual'><geometry><sphere><radius>0.25</radius></sphere></geometry></visual></link>",
        members(count)
    )
}

fn joint(name: &str, parent: &str, child: &str, axis: &str) -> String {
    format!(
        "<joint name='{name}' type='revolute'><parent>{parent}</parent><child>{child}</child><pose relative_to='{child}'>0 0 0 0 0 0</pose><axis><xyz expressed_in='__model__'>{axis}</xyz></axis></joint>"
    )
}

fn rotor(count: usize, axis: &str) -> String {
    world(&format!(
        "<model name='rig'><pose>{MODEL_POSE}</pose>{}{}</model>",
        link("body", count),
        joint("hinge", "world", "body", axis)
    ))
}

fn pair(parent_count: usize, child_count: usize) -> String {
    world(&format!(
        "<model name='rig'><pose>{MODEL_POSE}</pose>{}{}{}</model>",
        link("parent", parent_count),
        link("child", child_count),
        joint("hinge", "parent", "child", "0 0 1")
    ))
}

fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(xml, "compound-joints.sdf", "default", &NoResources, engine)
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

fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(named(sim, name))
        .unwrap()
        .id()
}

fn state(sim: &Simulation, name: &str) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body(sim, name))
        .unwrap()
}

fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *sim.world().get::<WorldPose>(named(sim, name)).unwrap()
}

fn metadata(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.clone(), e.get::<EntityId>()?.get())))
        .collect()
}

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() < tolerance,
        "{actual} != {expected}"
    );
}

fn near(actual: [f64; 3], expected: [f64; 3], tolerance: f64) {
    for (a, b) in actual.into_iter().zip(expected) {
        close(a, b, tolerance);
    }
}

fn model_q() -> DQuat {
    DQuat::from_euler(EulerRot::ZYX, 0.4, -0.3, 0.2)
}

fn cast(
    sim: &Simulation,
    origin: DVec3,
    direction: DVec3,
    maximum: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin.to_array(), direction.to_array(), maximum, exclude)
        .unwrap()
}

fn check_members(sim: &Simulation, link_name: &str, count: usize, exclude: Option<BodyId>) {
    let frame = pose(sim, link_name);
    let q = DQuat::from_array(frame.rotation);
    let p = DVec3::from_array(frame.translation);
    let child_q = q * DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);
    for i in 0..count {
        let name = format!("{link_name}::c{i}");
        let expected_center = p + q * DVec3::new((i * 3) as f64, 0., 0.);
        let actual = pose(sim, &name);
        near(actual.translation, expected_center.to_array(), 1e-11);
        assert!(DQuat::from_array(actual.rotation).dot(child_q).abs() > 1. - 1e-12);
        let local = sim.world().get::<LocalPose>(named(sim, &name)).unwrap().0;
        near(local.translation, [(i * 3) as f64, 0., 0.], 1e-11);
        let axis = child_q * DVec3::Z;
        close(
            cast(sim, expected_center + axis, -axis, 1.5, exclude).unwrap(),
            if i % 4 == 3 { 0.5 } else { 0.75 },
            1e-11,
        );
    }
}

#[test]
fn compound_world_revolutes_keep_axis_angle_pivot_and_all_child_geometry() {
    for count in [2, 4, 16] {
        let mut xml = rotor(count, "0 0 5");
        if count == 4 {
            xml = xml.replace(
                "<cylinder><radius>0.25</radius>",
                "<cylinder><radius>0.5</radius>",
            );
        }
        let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
        let native = body(&sim, "default::rig::body");
        let axis = model_q() * DVec3::Z;
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(native, [0.; 3], (axis * 1.5).to_array())
            .unwrap();
        sim.step_exact(100).unwrap();
        let sample = sim
            .world()
            .get::<JointSample>(named(&sim, "default::rig::hinge"))
            .unwrap();
        close(sample.state.position, 0.15, 1e-7);
        close(sample.state.velocity, 1.5, 1e-10);
        assert_eq!(sample.time, Duration::from_millis(100));
        near(
            state(&sim, "default::rig::body").position,
            [2., -3., 5.],
            1e-10,
        );
        near(
            pose(&sim, "default::rig::hinge").translation,
            [2., -3., 5.],
            1e-10,
        );
        let expected = DQuat::from_axis_angle(axis, 0.15) * model_q();
        assert!(
            DQuat::from_array(pose(&sim, "default::rig::body").rotation)
                .dot(expected)
                .abs()
                > 1. - 1e-12
        );
        check_members(&sim, "default::rig::body", count, None);
        assert_eq!(
            cast(
                &sim,
                DVec3::from_array(state(&sim, "default::rig::body").position),
                axis,
                0.125,
                Some(native)
            ),
            None
        );
        let frame = pose(&sim, "default::rig::body");
        let q = DQuat::from_array(frame.rotation);
        let p = DVec3::from_array(frame.translation);
        let gap = p + q * DVec3::new(1.5, 0., 0.);
        assert_eq!(
            cast(&sim, gap + q * DVec3::Z, -(q * DVec3::Z), 2., None),
            None
        );
        if count == 4 {
            let center = p + q * DVec3::new(6., 0., 0.);
            let local_q = q * DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);
            // Independent 3-4-5 radial chord on the advected radius-.5 cylinder.
            close(
                cast(
                    &sim,
                    center + local_q * DVec3::new(0.3, 4., 0.),
                    -(local_q * DVec3::Y),
                    5.,
                    None,
                )
                .unwrap(),
                3.6,
                1e-11,
            );
        }
    }
    // Rotation about world Z leaves this sphere's axial ray binary-exact in Z.
    let mut sim = load(
        &rotor(4, "0 0 1").replace(MODEL_POSE, "0 0 5 0 0 0"),
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let native = body(&sim, "default::rig::body");
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_world_velocity(native, [0.; 3], [0., 0., 1.])
        .unwrap();
    sim.step_exact(100).unwrap();
    let frame = pose(&sim, "default::rig::body");
    let center = DVec3::from_array(frame.translation)
        + DQuat::from_array(frame.rotation) * DVec3::new(3., 0., 0.);
    close(
        cast(&sim, center + DVec3::Z, -DVec3::Z, 0.75, None).unwrap(),
        0.75,
        1e-11,
    );
    assert_eq!(
        cast(&sim, center + DVec3::Z, -DVec3::Z, 0.75 - 1e-8, None),
        None
    );
    assert_eq!(cast(&sim, center, DVec3::Z, 0.125, None), Some(0.));
    assert_eq!(cast(&sim, center, DVec3::Z, 0.125, Some(native)), None);
}

#[test]
fn mixed_and_compound_body_endpoints_measure_relative_motion_without_self_contacts() {
    for (parent_count, child_count) in [(4, 1), (1, 4), (4, 4)] {
        for anchored in [false, true] {
            let mut xml = pair(parent_count, child_count);
            if anchored {
                xml = xml.replace(
                    "</model>",
                    &format!("{}</model>", joint("root", "world", "parent", "0 0 1")),
                );
            }
            let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
            let parent = body(&sim, "default::rig::parent");
            let child = body(&sim, "default::rig::child");
            let joint_id = id(&sim, "default::rig::hinge");
            assert!(matches!(
                queue_joint_effort(&mut sim, joint_id, 1.),
                Err(InputError::Physics(PhysicsError::Unsupported {
                    capability: "moving-parent joint effort commands",
                    ..
                }))
            ));
            let axis = model_q() * DVec3::Z;
            for (native, speed) in [(parent, 1.), (child, 2.)] {
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .set_world_velocity(
                        native,
                        if anchored { [0.; 3] } else { [0.3, -0.2, 0.1] },
                        (axis * speed).to_array(),
                    )
                    .unwrap();
            }
            sim.step_exact(100).unwrap();
            let hinge = named(&sim, "default::rig::hinge");
            let sample = sim.world().get::<JointSample>(hinge).unwrap();
            close(sample.state.position, 0.1, 1e-7);
            close(sample.state.velocity, 1., 1e-10);
            let position = if anchored {
                [2., -3., 5.]
            } else {
                [2.03, -3.02, 5.01]
            };
            for (name, count, other, angle) in [
                ("parent", parent_count, child, 0.1),
                ("child", child_count, parent, 0.2),
            ] {
                let name = format!("default::rig::{name}");
                near(state(&sim, &name).position, position, 1e-10);
                let expected = DQuat::from_axis_angle(axis, angle) * model_q();
                assert!(
                    DQuat::from_array(pose(&sim, &name).rotation)
                        .dot(expected)
                        .abs()
                        > 1. - 1e-12
                );
                check_members(&sim, &name, count, Some(other));
                assert_eq!(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .is_jointed(body(&sim, &name)),
                    Some(true)
                );
            }
            near(
                pose(&sim, "default::rig::hinge").translation,
                position,
                1e-10,
            );
            if anchored {
                let root = sim
                    .world()
                    .get::<JointSample>(named(&sim, "default::rig::root"))
                    .unwrap();
                close(root.state.position, 0.1, 1e-7);
                close(root.state.velocity, 1., 1e-10);
                near(
                    pose(&sim, "default::rig::root").translation,
                    position,
                    1e-10,
                );
            }
        }
    }
}

#[test]
fn compound_joint_effort_uses_explicit_inertia_and_preserves_paused_and_invalid_inputs() {
    for axis in ["5 0 0", "0 5 0", "0 0 5"] {
        let mut sim = load(&rotor(4, axis), PhysicsEngine::Rapier).unwrap();
        let hinge = id(&sim, "default::rig::hinge");
        let link = id(&sim, "default::rig::body");
        queue_joint_effort(&mut sim, hinge, 2.).unwrap();
        queue_joint_effort(&mut sim, hinge, -0.5).unwrap();
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                queue_joint_effort(&mut sim, hinge, invalid),
                Err(InputError::InvalidValue)
            ));
        }
        assert!(matches!(
            queue_link_world_velocity(&mut sim, link, [1.; 3], [0.; 3]),
            Err(InputError::Unsupported("jointed-body input"))
        ));
        assert!(matches!(
            queue_link_world_wrench(&mut sim, link, [1.; 3], [0.; 3]),
            Err(InputError::Unsupported("jointed-body input"))
        ));
        sim.tick().unwrap();
        sim.tick().unwrap();
        close(
            sim.world()
                .get::<JointSample>(entity_by_id(sim.world(), hinge).unwrap())
                .unwrap()
                .state
                .velocity,
            0.,
            1e-12,
        );
        sim.step_exact(1).unwrap();
        // Net 1.5 Nm / explicit 2 kg m², integrated over one millisecond.
        close(
            sim.world()
                .get::<JointSample>(entity_by_id(sim.world(), hinge).unwrap())
                .unwrap()
                .state
                .velocity,
            0.00075,
            1e-12,
        );
        let angle = sim
            .world()
            .get::<JointSample>(entity_by_id(sim.world(), hinge).unwrap())
            .unwrap()
            .state
            .position;
        assert!(angle > 0.);
        sim.step_exact(1).unwrap();
        close(
            sim.world()
                .get::<JointSample>(entity_by_id(sim.world(), hinge).unwrap())
                .unwrap()
                .state
                .velocity,
            0.00075,
            1e-12,
        );
        queue_joint_effort(&mut sim, hinge, -3.).unwrap();
        sim.step_exact(1).unwrap();
        close(
            sim.world()
                .get::<JointSample>(entity_by_id(sim.world(), hinge).unwrap())
                .unwrap()
                .state
                .velocity,
            -0.00075,
            1e-12,
        );
    }
}

#[test]
fn compound_joint_resets_retirement_and_high_water_keep_geometry_and_survivors() {
    let xml = rotor(4, "0 0 1").replace(
        "</world>",
        &format!(
            "<model name='survivor'><pose>100 0 5 0 0 0</pose>{}</model></world>",
            link("body", 1)
        ),
    );
    let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
    let ids = metadata(&sim);
    let model = id(&sim, "default::rig");
    let hinge_id = id(&sim, "default::rig::hinge");
    let native = body(&sim, "default::rig::body");
    let joint = sim
        .world()
        .get::<JointHandle>(named(&sim, "default::rig::hinge"))
        .unwrap()
        .id();
    let survivor = state(&sim, "default::survivor::body");
    queue_joint_effort(&mut sim, hinge_id, 2.).unwrap();
    sim.step_exact(10).unwrap();
    assert!(
        sim.world()
            .get::<JointSample>(named(&sim, "default::rig::hinge"))
            .unwrap()
            .state
            .position
            > 0.
    );
    queue_joint_effort(&mut sim, hinge_id, 2.).unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    close(
        sim.world()
            .get::<JointSample>(named(&sim, "default::rig::hinge"))
            .unwrap()
            .state
            .velocity,
        0.,
        1e-12,
    );
    check_members(&sim, "default::rig::body", 4, None);
    let initial = sim.world().resource::<PhysicsBackend>().initial_state();
    let extra = {
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let extra = backend
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.25 },
                    position: [200., 0., 5.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        assert!(backend.remove(extra));
        extra
    };
    let prepared = sim
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial)
        .unwrap();
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    assert_eq!(body(&sim, "default::rig::body"), native);
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().joint_child(joint),
        Some(native)
    );
    check_members(&sim, "default::rig::body", 4, None);
    let removed = remove_model(&mut sim, model).unwrap();
    for (name, public) in &ids {
        if name.starts_with("default::rig") {
            assert!(removed.contains(public));
        }
    }
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(native)
            .is_none()
    );
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(joint)
            .is_none()
    );
    assert!(matches!(
        queue_joint_effort(&mut sim, hinge_id, 1.),
        Err(InputError::MissingEntity(_))
    ));
    assert_eq!(state(&sim, "default::survivor::body"), survivor);
    let temporary = sim.world_mut().spawn_empty().id();
    let high_public = assign_id(sim.world_mut(), temporary).unwrap();
    sim.world_mut().entity_mut(temporary).despawn();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(metadata(&sim), ids);
    assert!(next_entity_id(sim.world()).unwrap() > high_public);
    assert_eq!(body(&sim, "default::rig::body"), native);
    assert_eq!(
        sim.world().resource::<PhysicsBackend>().joint_child(joint),
        Some(native)
    );
    assert_eq!(
        sim.world()
            .get::<JointSample>(named(&sim, "default::rig::hinge"))
            .unwrap()
            .epoch,
        2
    );
    sim.step_exact(1).unwrap();
    close(
        sim.world()
            .get::<JointSample>(named(&sim, "default::rig::hinge"))
            .unwrap()
            .state
            .velocity,
        0.,
        1e-12,
    );
    check_members(&sim, "default::rig::body", 4, None);
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let fresh = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: [200., 0., 5.],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(fresh > extra);
    assert!(backend.remove(fresh));
}

#[test]
fn jointed_compound_children_contact_other_models_but_keep_model_self_exclusion() {
    let spheres = |first: usize| {
        format!(
            "<collision name='main'><pose>{first} 0 0 0 0 0</pose><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><collision name='remote'><pose>4 0 0 0 0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>"
        )
    };
    for (same_model, first, contact) in [(false, 1, true), (false, 3, false), (true, 1, false)] {
        let ball = format!(
            "<link name='ball'><pose>1.6 0 0 0 0 0</pose>{INERTIA}<collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>"
        );
        let inner = if same_model {
            ball.clone()
        } else {
            String::new()
        };
        let outer = if same_model {
            String::new()
        } else {
            format!("<model name='other'><pose>0 0 5 0 0 0</pose>{ball}</model>")
        };
        let xml = world(&format!(
            "<model name='rig'><pose>0 0 5 0 0 0</pose><link name='body'>{INERTIA}{}</link>{inner}{}</model>{outer}",
            spheres(first),
            joint("hinge", "world", "body", "0 0 1")
        ));
        let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
        let ball_name = if same_model {
            "default::rig::ball"
        } else {
            "default::other::ball"
        };
        for reset in [false, true] {
            if reset {
                sim.reset().unwrap();
            }
            // Match the retained compound-contact settling window; solver settings are unchanged.
            sim.step_exact(1500).unwrap();
            let ball = state(&sim, ball_name);
            if contact {
                assert!(ball.position[0] > 1.7, "{}", ball.position[0]);
            } else {
                near(ball.position, [1.6, 0., 5.], 1e-11);
            }
            near(
                state(&sim, "default::rig::body").position,
                [0., 0., 5.],
                1e-10,
            );
            near(
                pose(&sim, "default::rig::hinge").translation,
                [0., 0., 5.],
                1e-10,
            );
        }
    }
}

#[test]
fn compound_joint_graph_and_numeric_refusals_preserve_existing_world_and_commands() {
    let valid = rotor(4, "0 0 1");
    let mut survivor = load(&valid, PhysicsEngine::Rapier).unwrap();
    let old_ids = metadata(&survivor);
    let initial = state(&survivor, "default::rig::body");
    let hinge = id(&survivor, "default::rig::hinge");
    queue_joint_effort(&mut survivor, hinge, 2.).unwrap();
    let rows = [
        (valid.replace("type='revolute'", "type='prismatic'"), "joints (unsupported type)"),
        (valid.replace("</axis>", "<limit><lower>-1</lower><upper>1</upper></limit></axis>"), "joint limits"),
        (valid.replace("</axis>", "<dynamics><damping>0.1</damping></dynamics></axis>"), "joint dynamics"),
        (valid.replace("<model name='rig'>", "<model name='rig'><static>true</static>"), "joint topology (expected one dynamic world-anchored link)"),
        (valid.replace("</axis>", "</axis><axis2><xyz>1 0 0</xyz></axis2>"), "joints (unsupported type)"),
        (valid.replace("</joint>", "</joint><joint name='second' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint>"), "multiple parent joints"),
    ];
    for (xml, expected) in rows {
        assert!(
            matches!(load(&xml, PhysicsEngine::Rapier), Err(LoadError::Unsupported { feature, .. }) if feature == expected),
            "{expected}"
        );
        assert_eq!(metadata(&survivor), old_ids);
        assert_eq!(state(&survivor, "default::rig::body"), initial);
        assert_eq!(survivor.elapsed(), Duration::ZERO);
    }
    for xml in [
        pair(4, 4).replace(
            "</model>",
            &format!("{}</model>", joint("back", "child", "parent", "0 0 1")),
        ),
        valid.replace("<child>body</child>", "<child>missing</child>"),
        valid.replace("0 0 1</xyz>", "0 0 0</xyz>"),
        valid.replace("<ixx>2</ixx>", "<ixx>0</ixx>"),
        valid.replace("<inertial>", "<inertial><pose>0.1 0 0 0 0 0</pose>"),
        valid.replace(
            "<box><size>0.5 0.5 0.5</size></box>",
            "<plane><normal>0 0 1</normal><size>1 1</size></plane>",
        ),
        valid.replace(
            "<capsule><radius>0.25</radius><length>0.5</length></capsule>",
            "<mesh><uri>missing-m87-last-child.gltf</uri></mesh>",
        ),
        valid.replace(
            "<capsule><radius>0.25</radius><length>0.5</length></capsule>",
            "<sphere><radius>1e308</radius></sphere>",
        ),
        valid.replace("<inertial>", "<inertial auto='true'>"),
        rotor(0, "0 0 1"),
        rotor(17, "0 0 1"),
    ] {
        assert!(load(&xml, PhysicsEngine::Rapier).is_err());
        assert_eq!(metadata(&survivor), old_ids);
        assert_eq!(state(&survivor, "default::rig::body"), initial);
    }
    assert!(matches!(
        load(&valid, PhysicsEngine::Tpe),
        Err(LoadError::Unsupported {
            feature: "TPE joints",
            ..
        })
    ));
    survivor.step_exact(1).unwrap();
    close(
        survivor
            .world()
            .get::<JointSample>(named(&survivor, "default::rig::hinge"))
            .unwrap()
            .state
            .velocity,
        0.001,
        1e-12,
    );
}

#[test]
fn compound_joint_chains_do_not_expand_joint_controllers_diff_drive_or_velocity_control() {
    let extra = "<collision name='m87_extra'><pose>2 0 0 0 0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>";
    for (row, (xml, target, occurrences)) in [
        (
            include_str!("../../../examples/m9_joint_controllers.sdf"),
            "<collision name=\"bar\"><geometry><box><size>1 0.14 0.14</size></box></geometry></collision>",
            1,
        ),
        (
            include_str!("../../../examples/m9_joint_controllers.sdf"),
            "<collision name=\"bar\"><geometry><box><size>0.6 0.12 0.12</size></box></geometry></collision>",
            1,
        ),
        (
            include_str!("../../../examples/m8_diff_drive.sdf"),
            "<collision name=\"chassis\"><geometry><box><size>0.6 0.35 0.2</size></box></geometry></collision>",
            1,
        ),
        (
            include_str!("../../../examples/m8_diff_drive.sdf"),
            "<collision name=\"wheel\"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision>",
            2,
        ),
    ].into_iter().enumerate() {
        assert!(load(xml, PhysicsEngine::Rapier).is_ok());
        assert_eq!(xml.matches(target).count(), occurrences);
        let at = xml.rfind(target).unwrap();
        let compound = format!("{}{target}{extra}{}", &xml[..at], &xml[at + target.len()..]);
        assert_ne!(compound, xml);
        if row < 2 {
            // Keep this historical case identity; only the two M9 rows widen.
            let mut sim = load(&compound, PhysicsEngine::Rapier).unwrap();
            let joints = [id(&sim, "default::arm::base_hinge"), id(&sim, "default::arm::tip_hinge")];
            for (joint, mode) in joints.into_iter().zip([JointControllerMode::Position, JointControllerMode::Velocity]) {
                assert!(joint_controller_binding_matches(&sim, joint));
                assert_eq!(sim.world().get::<JointControllerConfiguration>(entity_by_id(sim.world(), joint).unwrap()).unwrap().mode, mode);
            }
            let joint_states = |sim: &Simulation| joints.map(|joint| {
                let handle = sim.world().get::<JointHandle>(entity_by_id(sim.world(), joint).unwrap()).unwrap();
                sim.world().resource::<PhysicsBackend>().joint_state(handle.id()).unwrap()
            });
            let before = joint_states(&sim);
            let bodies = [state(&sim, "default::arm::base"), state(&sim, "default::arm::tip")];
            set_joint_controller_command(&mut sim, joints[0], 0.3).unwrap();
            set_joint_controller_command(&mut sim, joints[1], 0.5).unwrap();
            assert_eq!(joint_states(&sim), before, "motor receipt must not teleport joints");
            assert_eq!([state(&sim, "default::arm::base"), state(&sim, "default::arm::tip")], bodies);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            // Retain the existing M9 thirty-second tracking horizon and tolerance.
            sim.step_exact(30_000).unwrap();
            let measured = joint_states(&sim);
            close(measured[0].position, 0.3, 0.01);
            close(measured[1].velocity, 0.5, 0.01);
            assert_ne!(measured, before);
            for (joint, actual) in joints.into_iter().zip(measured) {
                let sample = sim.world().get::<JointSample>(entity_by_id(sim.world(), joint).unwrap()).unwrap();
                assert_eq!(sample.state, actual);
                assert_eq!(sample.time, sim.elapsed());
                assert_eq!(sample.epoch, sim.epoch());
                assert!(joint_controller_binding_matches(&sim, joint));
            }
            let link = if row == 0 { "default::arm::base" } else { "default::arm::tip" };
            let frame = pose(&sim, link);
            let center = DVec3::from_array(frame.translation) + DQuat::from_array(frame.rotation) * DVec3::X * 2.;
            near(pose(&sim, &format!("{link}::m87_extra")).translation, center.to_array(), 1e-10);
            // The other bars stay below this sphere's top; ground is beyond 2.1.
            close(cast(&sim, center + DVec3::Z * 2., -DVec3::Z, 2.1, None).unwrap(), 1.75, 1e-11);
            assert!(cast(&sim, center + DVec3::Z * 2., -DVec3::Z, 2.1, Some(body(&sim, link))).is_none());
        } else if row == 2 {
            // Only the canonical M8 chassis row widens; both wheels stay singleton.
            let mut sim = load(&compound, PhysicsEngine::Rapier).unwrap();
            let mut quiet = load(&compound, PhysicsEngine::Rapier).unwrap();
            let robot = id(&sim, "default::robot");
            assert!(diff_drive_binding_matches(&sim, robot));
            let chassis = "default::robot::base_link";
            let frame = pose(&sim, chassis);
            let center = DVec3::from_array(frame.translation) + DQuat::from_array(frame.rotation) * DVec3::X * 2.;
            near(pose(&sim, "default::robot::base_link::m87_extra").translation, center.to_array(), 1e-11);
            // Initial chassis/wheels lie around x=0, more than 1.5m from the
            // ray. Ground top z=0 is 2.2m below its origin, beyond max2.1.
            close(cast(&sim, center + DVec3::Z * 2., -DVec3::Z, 2.1, None).unwrap(), 1.75, 1e-11);
            assert!(cast(&sim, center + DVec3::Z * 2., -DVec3::Z, 2.1, Some(body(&sim, chassis))).is_none());
            let joints = ["default::robot::left_wheel_joint", "default::robot::right_wheel_joint"];
            let measured = |sim: &Simulation| joints.map(|name| {
                let handle = sim.world().get::<JointHandle>(named(sim, name)).unwrap();
                sim.world().resource::<PhysicsBackend>().joint_state(handle.id()).unwrap()
            });
            let before = measured(&sim);
            let body_before = state(&sim, chassis);
            let quiet_before = measured(&quiet);
            assert_eq!(before, quiet_before);
            assert_eq!(body_before, state(&quiet, chassis));
            set_diff_drive_command(&mut sim, robot, 0.4, 0.).unwrap();
            assert_eq!(measured(&sim), before);
            assert_eq!(state(&sim, chassis), body_before);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            // The retained drive horizon is one second, measured by real joints.
            sim.step_exact(1000).unwrap();
            quiet.step_exact(1000).unwrap();
            assert_ne!(measured(&sim), before);
            let mut encoder_separation = 0.;
            for (((actual, initial), control), control_initial) in measured(&sim).into_iter().zip(before).zip(measured(&quiet)).zip(quiet_before) {
                assert!(actual.position.is_finite() && actual.velocity.is_finite());
                assert!(control.position.is_finite() && control.velocity.is_finite());
                // Soft finite-force motors do not promise target-speed convergence.
                // Compare actual encoder response against the same settling fixture.
                let separation = (actual.position - initial.position) - (control.position - control_initial.position);
                assert!(separation > 0., "commanded encoder separation: {separation}");
                encoder_separation += separation;
            }
            // Existing M8's positive .01m encoder-travel witness, not body distance.
            assert!(0.2 * encoder_separation / 2. > 0.01);
            let sample = sim.world().get::<DiffDriveSample>(named(&sim, "default::robot")).unwrap();
            assert_eq!((sample.time, sample.epoch), (Duration::from_secs(1), 0));
            assert!(sample.pose.into_iter().chain(sample.twist).all(f64::is_finite));
            assert!(sample.pose[0] > 0.01);
            let moved = state(&sim, chassis);
            let quiet_moved = state(&quiet, chassis);
            assert!(moved.position.into_iter().chain(moved.orientation).all(f64::is_finite));
            assert!(quiet_moved.position.into_iter().chain(quiet_moved.orientation).all(f64::is_finite));
            assert!((moved.position, moved.orientation) != (body_before.position, body_before.orientation));
            let translation_separation = (DVec3::from_array(moved.position) - DVec3::from_array(quiet_moved.position)).abs().max_element();
            let driven_q = DQuat::from_array(moved.orientation);
            let quiet_q = DQuat::from_array(quiet_moved.orientation);
            // Basis vectors make the comparison invariant to quaternion sign.
            let rotation_separation = [DVec3::X, DVec3::Y, DVec3::Z].into_iter().map(|axis| (driven_q * axis - quiet_q * axis).abs().max_element()).fold(0_f64, f64::max);
            assert!(translation_separation > 1e-7 || rotation_separation > 1e-7,
                "no driven/quiet chassis separation: translation={translation_separation}, rotation={rotation_separation}");
            let moved_center = DVec3::from_array(moved.position) + DQuat::from_array(moved.orientation) * DVec3::X * 2.;
            near(pose(&sim, "default::robot::base_link::m87_extra").translation, moved_center.to_array(), 1e-10);
            // This inherited extra sphere initially penetrates ground by .05;
            // body settling is not attributed solely to motor traction here.
            // Dedicated M91 cases compare ground, quiet and suspended controls.
            assert!(diff_drive_binding_matches(&sim, robot));
        } else {
            assert!(matches!(
                load(&compound, PhysicsEngine::Rapier),
                Err(LoadError::Unsupported {
                    feature: "compound joint actuators",
                    ..
                })
            ));
        }
    }
    let controlled = rotor(4, "0 0 1").replace("</model>", "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/></model>");
    assert!(load(&controlled, PhysicsEngine::Rapier).is_err());
    let mut passive = load(&rotor(4, "0 0 1"), PhysicsEngine::Rapier).unwrap();
    let model = id(&passive, "default::rig");
    assert!(matches!(
        model_state(&passive, model),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        set_model_state(
            &mut passive,
            model,
            ModelState {
                translation: [2., -3., 5.],
                rotation: model_q().to_array(),
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3]
            }
        ),
        Err(WorldEditError::UnsupportedModel)
    ));
}

#[test]
fn compound_joint_binding_corruption_faults_before_step_or_retirement() {
    for fault in 0..4 {
        let mut sim = load(&pair(4, 4), PhysicsEngine::Rapier).unwrap();
        let model = id(&sim, "default::rig");
        let parent = named(&sim, "default::rig::parent");
        let child = named(&sim, "default::rig::child");
        let hinge = named(&sim, "default::rig::hinge");
        match fault {
            0 => {
                sim.world_mut().entity_mut(hinge).remove::<JointSample>();
            }
            1 => {
                sim.world_mut().entity_mut(child).remove::<WorldPose>();
            }
            2 => {
                let a = sim
                    .world_mut()
                    .entity_mut(parent)
                    .take::<BodyHandle>()
                    .unwrap();
                let b = sim
                    .world_mut()
                    .entity_mut(child)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(parent).insert(b);
                sim.world_mut().entity_mut(child).insert(a);
            }
            3 => {
                sim.world_mut()
                    .entity_mut(child)
                    .remove::<gz_sim_scene::Velocity>();
            }
            _ => unreachable!(),
        }
        assert_eq!(
            remove_model(&mut sim, model),
            Err(RemoveError::InvalidPhysics),
            "fault {fault}"
        );
        assert!(sim.step_exact(1).is_err(), "fault {fault}");
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}
