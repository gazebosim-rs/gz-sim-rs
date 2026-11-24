// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointDefinition, JointSample, Name, PhysicsEngine, load_world_with_sdf_step,
    queue_joint_effort, queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;

const DT: f64 = 0.0005;
const LEFT: DVec3 = DVec3::new(0.02, -0.01, 0.04);
const RIGHT: DVec3 = DVec3::new(-0.01, 0.04, 0.02);
const WELD: &str = r#"<joint name="lock" type="fixed"><parent>first</parent><child>second</child><pose>0.01 0.02 0.03 0.2 -0.1 0.3</pose></joint>"#;
fn xml() -> String {
    format!(
        r#"<sdf version="1.12"><world name="w"><gravity>0 0 0</gravity><physics name="p" type="ode"><max_step_size>0.0005</max_step_size></physics>
    <model name="body"><pose>0.1 0.3 0.5 0.4 -0.2 0.3</pose><link name="first">
    <inertial><pose>0.02 -0.01 0.04 0 0 0</pose><mass>2</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
    <collision name="c"><geometry><box><size>0.2 0.3 0.4</size></box></geometry></collision></link>
    <link name="second"><pose>0.3 0.2 0.1 0.3 0.15 -0.2</pose><inertial><pose>-0.01 0.04 0.02 0 0 0</pose><mass>3</mass><inertia><ixx>0.16</ixx><iyy>0.16</iyy><izz>0.16</izz></inertia></inertial>
    <collision name="c"><geometry><box><size>0.2 0.12 0.1</size></box></geometry></collision></link>{WELD}</model></world></sdf>"#
    )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "fixed.sdf",
        "w",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn read(s: &Simulation, id: u64) -> BodyState {
    let body = s
        .world()
        .get::<BodyHandle>(entity_by_id(s.world(), id).unwrap())
        .unwrap()
        .id();
    s.world().resource::<PhysicsBackend>().state(body).unwrap()
}
fn com(b: BodyState, offset: DVec3) -> DVec3 {
    DVec3::from(b.position) + DQuat::from_array(b.orientation) * offset
}
fn near(a: DVec3, b: DVec3, e: f64) {
    assert!(
        a.distance(b) <= e,
        "{a:?} vs {b:?} (error {})",
        a.distance(b)
    )
}
#[test]
fn each_welded_link_supplies_force_and_shifted_moment_about_assembly_com() {
    let mut s = load(&xml());
    let first = id(&s, "w::body::first");
    let second = id(&s, "w::body::second");
    let lock = id(&s, "w::body::lock");
    let force = DVec3::new(6., -2., 4.);
    let torque = DVec3::new(0.23, 0.1, -0.35);
    for (target, offset) in [(first, LEFT), (second, RIGHT)] {
        let input = [read(&s, first), read(&s, second)];
        let centers = [com(input[0], LEFT), com(input[1], RIGHT)];
        let center = (2. * centers[0] + 3. * centers[1]) / 5.;
        let initial_delta = DQuat::from_array(input[0].orientation).inverse()
            * (DVec3::from(input[1].position) - DVec3::from(input[0].position));
        let initial_rot = DQuat::from_array(input[0].orientation).inverse()
            * DQuat::from_array(input[1].orientation);
        let arm = com(read(&s, target), offset) - center;
        let expected = DT * (torque + arm.cross(force));
        s.set_paused(true);
        queue_link_world_wrench(&mut s, target, force.into(), torque.into()).unwrap();
        assert!(queue_joint_effort(&mut s, lock, 0.1).is_err());
        s.tick().unwrap();
        assert_eq!(read(&s, first), input[0]);
        s.step_exact(1).unwrap();
        let a = read(&s, first);
        let b = read(&s, second);
        let v = DVec3::from(a.velocity) * 2. + DVec3::from(b.velocity) * 3.;
        near(v, force * DT, 2e-8);
        let angular = DVec3::from(a.angular_velocity) * 0.1
            + DVec3::from(b.angular_velocity) * 0.16
            + (com(a, LEFT) - center).cross(2. * DVec3::from(a.velocity))
            + (com(b, RIGHT) - center).cross(3. * DVec3::from(b.velocity));
        near(angular, expected, 2e-8);
        s.step_exact(40).unwrap();
        let a = read(&s, first);
        let b = read(&s, second);
        let q = DQuat::from_array(a.orientation);
        near(
            q.inverse() * (DVec3::from(b.position) - DVec3::from(a.position)),
            initial_delta,
            1e-13,
        );
        assert!((q.inverse() * DQuat::from_array(b.orientation)).angle_between(initial_rot) < 1e-7);
        near(a.angular_velocity.into(), b.angular_velocity.into(), 1e-12);
        let joint = entity_by_id(s.world(), lock).unwrap();
        assert_eq!(
            s.world().get::<JointDefinition>(joint).unwrap().0.kind,
            sdformat::JointKind::Fixed
        );
        let j = s.world().get::<JointSample>(joint).unwrap();
        assert_eq!([j.state.position, j.state.velocity], [0.; 2]);
        assert_eq!(j.time, s.elapsed());
        reset_loaded_world(&mut s, |_| ()).unwrap();
        assert_eq!([read(&s, first), read(&s, second)], input);
    }
}
#[test]
fn world_anchored_fixed_tree_holds_both_links_and_other_engines_keep_admission() {
    let input=xml().replace(WELD,&format!("{WELD}<joint name=\"anchor\" type=\"fixed\"><parent>world</parent><child>first</child></joint>")).replace("<gravity>0 0 0","<gravity>0 0 -9.81");
    let mut s = load(&input);
    let ids = [id(&s, "w::body::first"), id(&s, "w::body::second")];
    let before = ids.map(|id| read(&s, id));
    queue_link_world_wrench(&mut s, ids[1], [10., 0., 0.], [0., 1., 0.]).unwrap();
    s.step_exact(10).unwrap();
    for (id, b) in ids.into_iter().zip(before) {
        let end = read(&s, id);
        near(end.position.into(), b.position.into(), 1e-13);
        near(end.velocity.into(), DVec3::ZERO, 1e-15);
        near(end.angular_velocity.into(), DVec3::ZERO, 1e-15);
    }
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        assert!(load_world_with_sdf_step(&input, "fixed.sdf", "w", &NoResources, engine).is_err());
    }
}

#[test]
fn free_fixed_links_start_at_one_assembly_twist_and_replay_after_reset() {
    let input=xml().replace("<link name=\"first\">",r#"<plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>0.3 -0.2 0.7</linear_velocity><angular_velocity>0.13 0.6 -0.2</angular_velocity></plugin><link name="first">"#);
    let mut s = load(&input);
    let a = id(&s, "w::body::first");
    let b = id(&s, "w::body::second");
    let snapshot = |s: &Simulation| [read(s, a), read(s, b)];
    let start = snapshot(&s);
    let velocity = DVec3::new(0.3, -0.2, 0.7);
    let spin = DVec3::new(0.13, 0.6, -0.2);
    let centers = [com(start[0], LEFT), com(start[1], RIGHT)];
    let center = (2. * centers[0] + 3. * centers[1]) / 5.;
    for (b, r) in start.iter().zip(centers) {
        near(b.velocity.into(), velocity + spin.cross(r - center), 3e-15);
        near(b.angular_velocity.into(), spin, 1e-15);
    }
    s.step_exact(300).unwrap();
    let end = snapshot(&s);
    let end_com = (2. * com(end[0], LEFT) + 3. * com(end[1], RIGHT)) / 5.;
    // The native constrained solve integrates the assembly twists on its ordinary clock.
    near(end_com, center + velocity * DT * 300., 3e-5);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert_eq!(snapshot(&s), start);
    s.step_exact(300).unwrap();
    assert_eq!(snapshot(&s), end);
}

#[test]
fn welded_and_free_hinges_do_not_get_reclassified_into_a_rigid_assembly() {
    let guest = r#"<link name="free"><pose>0.7 0 0 0 0 0</pose><collision name="c"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision></link>"#;
    let plain = xml();
    assert!(
        load_world_with_sdf_step(
            &plain.replace(WELD, &format!("{guest}{WELD}")),
            "w.sdf",
            "w",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
    let mixed=plain.replace(WELD,&format!("{guest}{WELD}<joint name=\"hinge\" type=\"revolute\"><parent>second</parent><child>free</child><axis><xyz>1 0 0</xyz></axis></joint>"));
    assert!(
        load_world_with_sdf_step(
            &mixed,
            "w.sdf",
            "w",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
}

#[test]
fn aggregate_inertia_momenta_and_held_wrenches_agree_from_either_link() {
    use glam::EulerRot;
    use gz_physics_multifidelity::MultiFidelityBackend;
    let input = xml()
        .replace("0.02 -0.01 0.04 0 0 0", "0.02 -0.01 0.04 0.2 0.1 -0.3")
        .replace(
            "<ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>",
            "<ixx>0.08</ixx><iyy>0.12</iyy><izz>0.16</izz>",
        );
    let mut s = load(&input);
    let links = [id(&s, "w::body::first"), id(&s, "w::body::second")];
    let native = links.map(|id| {
        s.world()
            .get::<BodyHandle>(entity_by_id(s.world(), id).unwrap())
            .unwrap()
            .id()
    });
    let sample = |s: &Simulation, i: usize| {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .welded_assembly_state(native[i])
            .unwrap()
    };
    let starts = links.map(|id| read(&s, id));
    let centers = [com(starts[0], LEFT), com(starts[1], RIGHT)];
    let com = (2. * centers[0] + 3. * centers[1]) / 5.;
    let prop = sample(&s, 1);
    assert_eq!(prop, sample(&s, 0));
    assert_eq!(prop.root, native[0]);
    assert_eq!(prop.links, 2);
    assert_eq!(prop.mass, 5.);
    near(prop.center.into(), com, 1e-15);
    let axes =
        DQuat::from_array(starts[0].orientation) * DQuat::from_euler(EulerRot::ZYX, -0.3, 0.1, 0.2);
    for (i, axis) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().enumerate() {
        let expected = axes * (DVec3::new(0.08, 0.12, 0.16) * (axes.inverse() * axis))
            + 0.16 * axis
            + 2. * (centers[0] - com).cross(axis.cross(centers[0] - com))
            + 3. * (centers[1] - com).cross(axis.cross(centers[1] - com));
        near(prop.inertia_columns[i].into(), expected, 1e-14);
    }
    let f = [DVec3::new(3., -2., 1.), DVec3::new(2., 1., -3.)];
    let t = [DVec3::X * 0.3, -DVec3::Y * 0.2];
    for i in 0..2 {
        queue_link_world_wrench(&mut s, links[i], f[i].into(), t[i].into()).unwrap()
    }
    let held = sample(&s, 1);
    near(held.force.into(), f[0] + f[1], 1e-14);
    let total_torque =
        t[0] + t[1] + (centers[0] - com).cross(f[0]) + (centers[1] - com).cross(f[1]);
    near(held.torque.into(), total_torque, 1e-14);
    assert_eq!(held, sample(&s, 0));
    s.step_exact(1).unwrap();
    let out = sample(&s, 1);
    near(out.linear_momentum.into(), DT * (f[0] + f[1]), 1e-8);
    near(out.angular_momentum.into(), DT * total_torque, 1e-8);
    assert_eq!(out.force, [0.; 3]);
    assert_eq!(out.torque, [0.; 3]);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert_eq!(sample(&s, 0), prop);
}
