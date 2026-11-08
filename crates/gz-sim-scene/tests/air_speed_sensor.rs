// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::air_speed::{AirSpeedInput, air_speed};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    AirSpeedSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    queue_link_world_velocity, remove_model, reset_loaded_world, set_air_speed_interest,
};
use sdformat::NoResources;
use std::time::Duration;
const XML: &str = include_str!("fixtures/m26_air_speed.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "air-speed-public",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn named(sim: &Simulation, name: &str) -> (bevy_ecs::entity::Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]
}
#[test]
fn due_point_frame_oracle_and_held_samples_survive_real_rotation_and_imu_coexistence() {
    let xml = XML.replace(
        "</sensor>",
        "</sensor><sensor name='imu' type='imu'><update_rate>1000</update_rate><imu/></sensor>",
    );
    let mut sim = load(&xml);
    let (e, _) = named(&sim, "default::rig::body::air_speed");
    let (link, id) = named(&sim, "default::rig::body");
    queue_link_world_velocity(&mut sim, id, [3.0, 0.0, 0.0], [0.0, 0.0, 2.0]).unwrap();
    let initial_rotation = sim.world().get::<WorldPose>(e).unwrap().rotation;
    let mut held = None;
    let mut due = Vec::new();
    for tick in 1..=30 {
        sim.step_exact(1).unwrap();
        let sample = sim.world().get::<AirSpeedSample>(e).unwrap();
        let pair = (sample.time, *sample.reading.as_ref().unwrap());
        assert_eq!(sample.epoch, 0);
        if held.is_none_or(|(time, _)| time != sample.time) {
            let p = sim.world().get::<WorldPose>(e).unwrap();
            let l = sim.world().get::<WorldPose>(link).unwrap();
            let v = sim.world().get::<Velocity>(link).unwrap();
            let r = rotate(l.rotation, [0.0, 0.5, 0.5]);
            let w = v.angular;
            let point = [
                v.linear[0] + w[1] * r[2] - w[2] * r[1],
                v.linear[1] + w[2] * r[0] - w[0] * r[2],
                v.linear[2] + w[0] * r[1] - w[1] * r[0],
            ];
            let q = p.rotation;
            let body = rotate([-q[0], -q[1], -q[2], q[3]], point);
            let expected = air_speed(AirSpeedInput {
                world_z: p.translation[2],
                velocity_sensor: body,
            })
            .unwrap();
            assert!((pair.1.diff_pressure - expected.diff_pressure).abs() < 1e-6);
            assert_eq!(pair.1.temperature, expected.temperature);
            assert_eq!(sample.time, Duration::from_millis(tick));
            due.push(pair);
        } else {
            assert_eq!(
                Some(pair),
                held,
                "non-due acquisitions retain complete sample"
            );
        }
        held = Some(pair);
    }
    assert_eq!(
        due.iter().map(|p| p.0.as_millis()).collect::<Vec<_>>(),
        [1, 10, 20, 30]
    );
    assert_ne!(
        sim.world().get::<WorldPose>(e).unwrap().rotation,
        initial_rotation
    );
    assert!(
        due.windows(2)
            .any(|p| p[0].1.diff_pressure != p[1].1.diff_pressure)
    );
}
#[test]
fn direct_and_loaded_reset_clear_samples_restore_original_identity_and_retire_removed_family() {
    let mut sim = load(XML);
    let (old, id) = named(&sim, "default::rig::body::air_speed");
    let model = named(&sim, "default::rig").1;
    sim.step_exact(12).unwrap();
    sim.reset().unwrap();
    assert!(sim.world().get::<AirSpeedSample>(old).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<AirSpeedSample>(old).unwrap().time,
        Duration::from_millis(1)
    );
    let counts = sim.scheduled_system_counts();
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), id).is_none());
    assert_eq!(sim.scheduled_system_counts(), (counts.0 - 1, counts.1 - 1));
    set_air_speed_interest(&mut sim, [old]);
    reset_loaded_world(&mut sim, |_| {}).unwrap();
    let fresh = entity_by_id(sim.world(), id).unwrap();
    assert_ne!(fresh, old);
    assert!(sim.world().get::<AirSpeedSample>(fresh).is_none());
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(1).unwrap();
    let s = sim.world().get::<AirSpeedSample>(fresh).unwrap();
    assert_eq!(s.time, Duration::from_millis(1));
    assert_eq!(s.epoch, sim.epoch());
    assert_eq!(s.reading.as_ref().unwrap().diff_pressure, 0.0);
}
