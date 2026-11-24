// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/fleet_observation.rs"]
mod observation;
#[test]
fn loaded_links_models_visuals_and_imu_share_the_committed_translation() {
    observation::run(4).unwrap();
}

#[test]
fn mixed_sdf_publishes_decks_payloads_contacts_and_imu() {
    observation::run_mixed(300, true).unwrap();
    observation::run_mixed(300, false).unwrap();
}

#[test]
fn moving_fleet_lidar_sees_the_next_deck_after_an_edit_and_original_range_after_reset() {
    use gz_sim_core::{EntityId, Simulation};
    use gz_sim_scene::{
        LidarSample, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_world_velocity,
        reset_loaded_world,
    };
    use std::fmt::Write;

    let mut sdf = String::from(
        "<sdf version='1.12'><world name='fleet'><physics name='fixed' type='ignored'><max_step_size>0.01</max_step_size></physics>",
    );
    for i in 0..256 {
        let x = f64::from(i) * 1.1;
        write!(sdf,"<model name='d{i}'><pose>{x} 0 0 0 0 0</pose><plugin name='gz::sim::systems::PrescribedMotion' filename='gz-sim-prescribed-motion-system'><linear_velocity>0.25 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name='body'><collision name='deck'><geometry><box><size>1 1 0.2</size></box></geometry></collision>").unwrap();
        if i == 0 {
            sdf.push_str("<sensor name='scan' type='lidar'><always_on>true</always_on><update_rate>100</update_rate><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.01</min_angle><max_angle>0.01</max_angle></horizontal></scan><range><min>0</min><max>4</max><resolution>0.001</resolution></range></lidar></sensor>");
        }
        sdf.push_str("</link></model>");
    }
    sdf.push_str("</world></sdf>");
    let mut sim = load_world_with_sdf_step(
        &sdf,
        "fleet-lidar.sdf",
        "fleet",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let find = |sim: &Simulation, name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let sensor = find(&sim, "fleet::d0::body::scan");
    let target = find(&sim, "fleet::d1::body");
    let check = |sim: &Simulation, epoch, range: f64| {
        let scan = sim
            .world()
            .get::<LidarSample>(gz_sim_core::entity_by_id(sim.world(), sensor).unwrap())
            .unwrap();
        assert_eq!(scan.time, sim.elapsed());
        assert_eq!(scan.epoch, epoch);
        assert!(
            (scan.world_pose.translation[0] - 0.25 * sim.elapsed().as_secs_f64()).abs() < 1e-12
        );
        for (&hit, angle) in scan.ranges.iter().zip([-0.01_f64, 0., 0.01]) {
            assert!((hit - range / angle.cos()).abs() < 1e-12);
        }
    };
    sim.step_exact(5).unwrap();
    check(&sim, 0, 0.6);
    queue_link_world_velocity(&mut sim, target, [0.25, 20., 0.], [0.; 3]).unwrap();
    sim.set_paused(true);
    sim.tick().unwrap();
    check(&sim, 0, 0.6);
    sim.step_exact(5).unwrap();
    check(&sim, 0, 1.7);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(find(&sim, "fleet::d1::body"), target);
    sim.step_exact(1).unwrap();
    check(&sim, 1, 0.6);
}
