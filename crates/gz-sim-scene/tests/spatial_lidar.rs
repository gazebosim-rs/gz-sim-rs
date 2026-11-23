// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    LidarPublication, LidarSample, Name, PhysicsEngine, clear_lidar_interest,
    load_world_with_sdf_step, queue_link_world_velocity, remove_model, reset_loaded_world,
    set_lidar_interest,
};
use std::time::Duration;
fn source(vertical: &str, noise: &str, owners: usize) -> String {
    let mut xml = String::from(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>",
    );
    for i in 0..owners {
        xml += &format!(
            "<model name='rig{i}'><pose>0 {} 0 0 0 0</pose><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><pose>0 0 0.25 0 0 0</pose><update_rate>100</update_rate><topic>/scan{i}</topic><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal>{vertical}</scan><range><min>0</min><max>20</max></range>{noise}</lidar></sensor></link></model>",
            i as f64 * 0.5
        );
    }
    xml + "<model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
}
const V: &str = "<vertical><samples>3</samples><resolution>1</resolution><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical>";
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "spatial.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn check(sim: &Simulation, sensor: u64) {
    let e = entity_by_id(sim.world(), sensor).unwrap();
    let s = sim.world().get::<LidarSample>(e).unwrap();
    assert_eq!(s.ranges.len(), 9);
    let [x, y, z, w] = s.world_pose.rotation;
    for row in 0..3 {
        for column in 0..3 {
            let v = -0.2 + row as f64 * 0.2;
            let h = -0.3 + column as f64 * 0.3;
            let [a, b, c] = [v.cos() * h.cos(), v.cos() * h.sin(), v.sin()];
            let dx = (1. - 2. * (y * y + z * z)) * a
                + 2. * (x * y - z * w) * b
                + 2. * (x * z + y * w) * c;
            let expected = (4. - s.world_pose.translation[0]) / dx;
            assert!(
                (s.ranges[row * 3 + column] - expected).abs() < 1e-10,
                "{s:?} ray{row}/{column} expected{expected}"
            );
        }
    }
}
#[test]
fn spatial_loaded_grid_observes_native_walls_in_acquisition_frames_during_motion() {
    let mut sim = load(&source(V, "", 1));
    let sensor = id(&sim, "default::rig0::body::scan");
    let link = id(&sim, "default::rig0::body");
    queue_link_world_velocity(&mut sim, link, [0.2, 0.1, 0.0], [0.0, 0.0, 0.1]).unwrap();
    for _ in 0..5 {
        sim.step_exact(10).unwrap();
        check(&sim, sensor);
    }
    let e = entity_by_id(sim.world(), sensor).unwrap();
    let p = sim.world().get::<LidarPublication>(e).unwrap();
    assert_eq!(p.vertical.samples, 3);
    assert_eq!(p.config.samples, 3);
    let sample = sim.world().get::<LidarSample>(e).unwrap();
    assert!(sample.world_pose.translation[0] > 0.0);
    assert!(sample.world_pose.rotation[2] > 0.0);
}
#[test]
fn spatial_demand_pause_reset_and_retirement_keep_epochs_and_restored_identities() {
    let mut sim = load(&source(V, "", 1));
    let sensor = id(&sim, "default::rig0::body::scan");
    let model = id(&sim, "default::rig0");
    let e = entity_by_id(sim.world(), sensor).unwrap();
    set_lidar_interest(&mut sim, []);
    sim.step_exact(5).unwrap();
    assert!(sim.world().get::<LidarSample>(e).is_none());
    set_lidar_interest(&mut sim, [e]);
    sim.step_exact(1).unwrap();
    check(&sim, sensor);
    let time = sim.world().get::<LidarSample>(e).unwrap().time;
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(sim.world().get::<LidarSample>(e).unwrap().time, time);
    sim.set_paused(false);
    clear_lidar_interest(&mut sim);
    sim.step_exact(10).unwrap();
    assert!(sim.world().get::<LidarSample>(e).unwrap().time > time);
    remove_model(&mut sim, model).unwrap();
    assert!(entity_by_id(sim.world(), sensor).is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let e = entity_by_id(sim.world(), sensor).unwrap();
    assert!(sim.world().get::<LidarSample>(e).is_none());
    sim.step_exact(1).unwrap();
    let s = sim.world().get::<LidarSample>(e).unwrap();
    assert_eq!((s.epoch, s.time), (1, Duration::from_millis(1)));
    check(&sim, sensor);
}
#[test]
fn spatial_eight_owner_noise_streams_are_independent_and_reset_reproducible() {
    let noise = "<noise><type>gaussian</type><mean>0.01</mean><stddev>0.001</stddev></noise>";
    let mut sim = load(&source(V, noise, 8));
    let first = id(&sim, "default::rig0::body::scan");
    let last = id(&sim, "default::rig7::body::scan");
    let a = entity_by_id(sim.world(), first).unwrap();
    let b = entity_by_id(sim.world(), last).unwrap();
    set_lidar_interest(&mut sim, [a]);
    sim.step_exact(1).unwrap();
    let original = sim.world().get::<LidarSample>(a).unwrap().ranges.clone();
    assert!(sim.world().get::<LidarSample>(b).is_none());
    set_lidar_interest(&mut sim, [a, b]);
    sim.step_exact(10).unwrap();
    let other = sim.world().get::<LidarSample>(b).unwrap().ranges.clone();
    assert_ne!(original, other);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let a = entity_by_id(sim.world(), first).unwrap();
    set_lidar_interest(&mut sim, [a]);
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().get::<LidarSample>(a).unwrap().ranges, original);
}
#[test]
fn spatial_bad_grid_and_backend_domains_refuse_without_returning_partial_worlds() {
    for v in [
        V.replace("<samples>3</samples>", "<samples>0</samples>"),
        V.replace("<samples>3</samples>", "<samples>64</samples>"),
        V.replace("<resolution>1</resolution>", "<resolution>2</resolution>"),
        V.replace("<min_angle>-0.2</min_angle>", "<min_angle>0.3</min_angle>"),
        V.replace("<max_angle>0.2</max_angle>", "<max_angle>2</max_angle>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &source(&v, "", 1),
                "bad-grid",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            &source(V, "", 9),
            "ninth",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    assert!(
        load_world_with_sdf_step(
            &source(V, "", 1),
            "tpe",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
}
#[test]
fn explicit_single_row_is_identical_to_the_omitted_legacy_planar_branch() {
    let explicit = "<vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>";
    let mut a = load(&source("", "", 1));
    let mut b = load(&source(explicit, "", 1));
    a.step_exact(11).unwrap();
    b.step_exact(11).unwrap();
    let ea = entity_by_id(a.world(), id(&a, "default::rig0::body::scan")).unwrap();
    let eb = entity_by_id(b.world(), id(&b, "default::rig0::body::scan")).unwrap();
    let a = a.world().get::<LidarSample>(ea).unwrap();
    let b = b.world().get::<LidarSample>(eb).unwrap();
    assert_eq!(a.ranges, b.ranges);
    assert_eq!(a.world_pose.translation, b.world_pose.translation);
    assert_eq!(a.world_pose.rotation, b.world_pose.rotation);
    assert_eq!(a.time, b.time);
}

#[test]
fn spatial_distinct_elevated_planes_misses_and_minimum_clipping_keep_ray_order() {
    let planes = "<model name='floor'><static>true</static><pose>0 0 -0.5 0 0 0</pose><link name='body'><collision name='p'><geometry><box><size>100 100 0.5</size></box></geometry></collision></link></model><model name='ceiling'><static>true</static><pose>0 0 1.25 0 0 0</pose><link name='body'><collision name='p'><geometry><box><size>100 100 0.5</size></box></geometry></collision></link></model>";
    let xml = source(V, "", 1).replace("</world>", &format!("{planes}</world>"));
    for minimum in [0., 3.] {
        let source = xml.replace(
            "<range><min>0</min>",
            &format!("<range><min>{minimum}</min>"),
        );
        let mut sim = load(&source);
        sim.step_exact(1).unwrap();
        let e = entity_by_id(sim.world(), id(&sim, "default::rig0::body::scan")).unwrap();
        let s = sim.world().get::<LidarSample>(e).unwrap();
        for row in 0..3 {
            for col in 0..3 {
                let expected = match row {
                    0 => 0.5 / 0.2_f64.sin(),
                    1 => 4. / (-0.3 + col as f64 * 0.3).cos(),
                    _ => 0.75 / 0.2_f64.sin(),
                };
                assert!((s.ranges[row * 3 + col] - expected.max(minimum)).abs() < 1e-10);
            }
        }
    }
    let mut miss = load(&xml.replace("<max>20</max>", "<max>2</max>"));
    miss.step_exact(1).unwrap();
    let e = entity_by_id(miss.world(), id(&miss, "default::rig0::body::scan")).unwrap();
    assert!(
        miss.world()
            .get::<LidarSample>(e)
            .unwrap()
            .ranges
            .iter()
            .all(|x| *x == f64::INFINITY)
    );
}
