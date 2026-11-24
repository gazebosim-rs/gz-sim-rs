// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_force_at_point,
    reset_loaded_world,
};

const SDF: &str = r#"<sdf version="1.12"><world name="w"><gravity>0 0 0</gravity>
<physics name="p" type="ignored"><max_step_size>0.0005</max_step_size></physics>
<model name="tool"><pose>0 0 0 0 0 1.5707963267948966</pose><link name="link">
<inertial><pose>0.1 0 0 0 0 0</pose><mass>2</mass><inertia><ixx>0.028</ixx><iyy>0.031</iyy><izz>0.035</izz></inertia></inertial>
<collision name="c"><geometry><box><size>0.30 0.24 0.1</size></box></geometry></collision>
</link></model></world></sdf>"#;
#[allow(deprecated)]
fn fixture() -> (Simulation, u64) {
    let sim = load_world_with_sdf_step(
        SDF,
        "point-force.sdf",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let link = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::tool::link"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (sim, link)
}
fn state(sim: &Simulation, link: u64) -> BodyState {
    let e = entity_by_id(sim.world(), link).unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(sim.world().get::<BodyHandle>(e).unwrap().id())
        .unwrap()
}
#[test]
fn force_through_offset_center_translates_once_and_invalid_input_preserves_it() {
    let (mut sim, link) = fixture();
    sim.set_paused(true);
    queue_link_force_at_point(&mut sim, link, [0.1, 0., 0.], [10., 0., 0.]).unwrap();
    assert!(queue_link_force_at_point(&mut sim, link, [f64::NAN; 3], [1.; 3]).is_err());
    assert!(
        queue_link_force_at_point(&mut sim, link, [f64::MAX, 0., 0.], [f64::MAX, 0., 0.]).is_err()
    );
    sim.tick().unwrap();
    assert_eq!(state(&sim, link).velocity, [0.; 3]);
    sim.step_exact(1).unwrap();
    let after = state(&sim, link);
    assert!((after.velocity[0] - 0.0025).abs() < 1e-12);
    assert_eq!(after.angular_velocity, [0.; 3]);
    sim.step_exact(1).unwrap();
    assert_eq!(after.velocity, state(&sim, link).velocity);
}
#[test]
fn opposed_forces_on_rotated_link_spin_without_linear_momentum_and_reset_clears_loads() {
    let (mut sim, link) = fixture();
    let couple = |s: &mut Simulation| {
        queue_link_force_at_point(s, link, [0.20, 0., 0.], [10., 0., 0.]).unwrap();
        queue_link_force_at_point(s, link, [0., 0., 0.], [-10., 0., 0.]).unwrap();
    };
    couple(&mut sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(state(&sim, link).angular_velocity, [0.; 3]);
    couple(&mut sim);
    sim.step_exact(1).unwrap();
    let body = state(&sim, link);
    assert_eq!(body.velocity, [0.; 3]);
    assert!((body.angular_velocity[2] + 2. * 0.0005 / 0.035).abs() < 1e-10);
}

#[test]
fn load_readback_distinguishes_queued_zero_from_a_missing_owner() {
    use gz_sim_scene::{pending_link_world_wrench, queue_link_world_wrench};
    let (mut sim, link) = fixture();
    assert_eq!(pending_link_world_wrench(&sim, u64::MAX), None);
    assert_eq!(pending_link_world_wrench(&sim, 1), None);
    assert_eq!(pending_link_world_wrench(&sim, link), None);
    queue_link_world_wrench(&mut sim, link, [0.; 3], [0.; 3]).unwrap();
    assert_eq!(
        pending_link_world_wrench(&sim, link),
        Some(([0.; 3], [0.; 3]))
    );
    sim.step_exact(1).unwrap();
    assert_eq!(pending_link_world_wrench(&sim, link), None);
    sim.world_mut().remove_resource::<PhysicsBackend>();
    assert_eq!(pending_link_world_wrench(&sim, link), None);
}
