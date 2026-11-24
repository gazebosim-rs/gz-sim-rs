// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual Gazebo LaserScan packets from four worker processes vs an independent native world.
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, PhysicsBackend, RotationSpec};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Primitive, Tier},
    pool::PoolFrame,
};
use gz_physics_multifidelity::{MultiFidelityBackend, SolidRay};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_proto::{
    msgs::{LaserScan, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_scene::{
    BodyHandle, LidarSample, ModelState, Name, PhysicsEngine, WorldLoadOptions,
    load_world_configured,
};
use gz_sim_server::{InterestPolicy, Server, ServerOptions};
use sdformat::NoResources;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufWriter, Write},
    num::NonZeroUsize,
    path::Path,
    time::Duration,
};
use zenoh::Wait;

const XML: &str = include_str!("../../../examples/distributed_pool_lidar.sdf");
const DT: Duration = Duration::from_millis(10);
const TICKS: u64 = 130;

fn sensor_world(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn named<'a>(sim: &'a Simulation, path: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == path))
        .unwrap()
}
fn native_world(seed: &PoolFrame) -> (MultiFidelityBackend, BTreeMap<u64, BodyId>) {
    let mut reference = MultiFidelityBackend::new([0.; 3]).unwrap();
    reference.set_partition_step(DT).unwrap();
    let mut shapes: Vec<_> = seed
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    shapes.sort_by_key(|b| b.id);
    let handles = shapes
        .into_iter()
        .map(|b| {
            let id = reference
                .spawn_partition_primitive(
                    BodySpec {
                        shape: b.shape().unwrap(),
                        position: b.position,
                        velocity: b.linear,
                        mass: Some(b.mass),
                    },
                    RotationSpec {
                        orientation: b.orientation,
                        angular_velocity: b.angular,
                    },
                    b.tier == Tier::Prescribed,
                    b.friction,
                )
                .unwrap();
            (b.id, id)
        })
        .collect();
    (reference, handles)
}

// Only supplies hit labels. Every expected range comes from cast_ray on the
// complete independent native world including occluders and the scanner.
fn axial_label(
    seed: &PoolFrame,
    native: &MultiFidelityBackend,
    ids: &BTreeMap<u64, BodyId>,
    origin: [f64; 3],
    dir: [f64; 3],
    nearest: f64,
) -> &'static str {
    for b in seed.workers.iter().flat_map(|w| &w.snapshot.bodies) {
        let n = native.state(ids[&b.id]).unwrap();
        let mut ray = SolidRay::new(origin, dir, 6.).unwrap();
        let label = match b.primitive {
            Primitive::Cylinder => {
                ray.include_cylinder(b.size[0] / 2., b.size[2] / 2., n.position, n.orientation)
                    .unwrap();
                "cylinder"
            }
            Primitive::Capsule => {
                ray.include_capsule(b.size[0] / 2., b.size[2] / 2., n.position, n.orientation)
                    .unwrap();
                "capsule"
            }
            _ => continue,
        };
        if ray.distance().is_some_and(|d| (d - nearest).abs() < 1e-10) {
            return label;
        }
    }
    "box"
}

fn check_header(p: &LaserScan, expected_time: Duration, seq: u64) {
    let head = p.header.as_ref().unwrap();
    let stamp = head.stamp.as_ref().unwrap();
    assert_eq!(
        Duration::new(stamp.sec as u64, stamp.nsec as u32),
        expected_time
    );
    assert_eq!(p.frame, "laser");
    assert_eq!((p.count, p.vertical_count), (31, 1));
    assert_eq!(
        (p.angle_min, p.angle_max, p.angle_step),
        (-0.75, 0.75, 0.05)
    );
    assert_eq!((p.range_min, p.range_max), (0.05, 6.));
    assert_eq!(p.intensities, vec![0.; 31]);
    assert_eq!(
        head.data.iter().find(|f| f.key == "seq").unwrap().value,
        [seq.to_string()]
    );
    assert_eq!(
        head.data
            .iter()
            .find(|f| f.key == "frame_id")
            .unwrap()
            .value,
        ["laser"]
    );
}

#[test]
fn zenoh_laser_packets_match_native_curved_geometry_through_worker_loss_edit_and_reset() {
    let sim = load_world_configured(
        XML,
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/distributed_pool_lidar.sdf"
        ),
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        },
    )
    .unwrap();
    let seed = sensor_world(&sim).pool_frame().unwrap().clone();
    assert!(seed.workers.iter().all(|w| w.snapshot.bodies.len() == 1));
    let scanner = named(&sim, "default::scanner::body")
        .get::<BodyHandle>()
        .unwrap()
        .id()
        .raw();
    let target = named(&sim, "default::capsule::body")
        .get::<BodyHandle>()
        .unwrap()
        .id()
        .raw();
    let model = named(&sim, "default::capsule")
        .get::<EntityId>()
        .unwrap()
        .get();
    let sensor_name = "default::scanner::body::laser";
    let stable_sensor = named(&sim, sensor_name).get::<EntityId>().unwrap().get();

    // Binding the observer first avoids port reservation races and keeps the
    // canonical scan transport separate from all four private worker sessions.
    let listen = zenoh::Config::from_json5(
        r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},
        connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#,
    )
    .unwrap();
    let observer = zenoh::open(listen).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|l| l.to_string())
        .collect();
    let config = serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}});
    let options = ServerOptions {
        partition: format!("pool_lidar_{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let key = options.topic_key("/scan").unwrap();
    let subscriber = observer.declare_subscriber(key.clone()).wait().unwrap();
    let mut server = Server::with_zenoh_config(
        sim,
        "default",
        options,
        zenoh::Config::from_json5(&config.to_string()).unwrap(),
        NonZeroUsize::new(8).unwrap(),
    )
    .unwrap();
    server.set_interest_policy(InterestPolicy::Always);
    let mut csv =
        std::env::var_os("GZ_POOL_LIDAR_CSV").map(|p| BufWriter::new(File::create(p).unwrap()));
    if let Some(out) = &mut csv {
        writeln!(out, "epoch,tick,seq,beam,target,range_m,native_m").unwrap();
    }
    let mut trace = Vec::new();
    let mut hits = BTreeMap::<&str, usize>::new();
    let mut max_error = 0_f64;
    let mut bit_exact = 0;
    let mut first_contact = None;
    let mut first_route = None;
    let mut foreign_hits = BTreeMap::<&str, usize>::new();
    let mut edit_changes = 0;
    let cylinder_id = named(server.simulation(), "default::cylinder::body")
        .get::<BodyHandle>()
        .unwrap()
        .id()
        .raw();
    let mut unedited = native_world(&seed);
    let mut four_owners = 0;
    let mut replacement = None;
    for epoch in 0..=1 {
        if epoch == 1 {
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                reset: Some(WorldReset {
                    all: true,
                    ..Default::default()
                }),
                ..Default::default()
            }));
            server.update().unwrap();
            assert_eq!(server.simulation().elapsed(), Duration::ZERO);
            assert_eq!(sensor_world(server.simulation()).pool_frame(), Some(&seed));
            let s = named(server.simulation(), sensor_name);
            assert_eq!(s.get::<EntityId>().unwrap().get(), stable_sensor);
            assert!(s.get::<LidarSample>().is_none());
        }
        let (mut native, ids) = native_world(&seed);
        for tick in 1..=TICKS {
            if tick == 39 {
                let h = 0.48_f64;
                let edited = ModelState {
                    translation: [2.5, 1.78, 0.55],
                    rotation: [0., h.sin(), 0., h.cos()],
                    linear_velocity: [-0.12, 0.1, 0.],
                    angular_velocity: [0.1, 0.03, -0.12],
                };
                let before = sensor_world(server.simulation()).pool_frame().unwrap();
                let (station, _) = before.body(target).unwrap();
                let members = &before.workers[usize::from(station)].components;
                assert!(
                    members
                        .iter()
                        .any(|c| c.ids == [target] && !c.contact_history),
                    "historical target at {tick}: {members:?}"
                );
                server.set_model_state(model, edited).unwrap();
                native
                    .set_body_state(
                        ids[&target],
                        BodyState {
                            position: edited.translation,
                            orientation: edited.rotation,
                            velocity: edited.linear_velocity,
                            angular_velocity: edited.angular_velocity,
                        },
                    )
                    .unwrap();
            }
            #[cfg(unix)]
            if epoch == 0 && tick == 108 {
                let owner = usize::from(
                    sensor_world(server.simulation())
                        .pool_frame()
                        .unwrap()
                        .body(scanner)
                        .unwrap()
                        .0,
                );
                let pid = sensor_world(server.simulation())
                    .local_worker_pids()
                    .unwrap()[owner];
                assert!(
                    std::process::Command::new("kill")
                        .args(["-KILL", &pid.to_string()])
                        .status()
                        .unwrap()
                        .success()
                );
                replacement = Some((owner, pid));
            }
            assert!(server.apply_local_control(WorldControl {
                pause: true,
                multi_step: 1,
                ..Default::default()
            }));
            server.update().unwrap();
            native.step(DT).unwrap();
            if epoch == 0 && tick <= 39 {
                unedited.0.step(DT).unwrap();
            }
            let epoch_tick = Duration::from_millis(tick * 10);
            let f = sensor_world(server.simulation()).pool_frame().unwrap();
            assert_eq!(f.tick, tick);
            if f.workers.iter().all(|w| !w.snapshot.bodies.is_empty()) {
                four_owners += 1;
            }
            if f.workers
                .iter()
                .any(|w| w.generations.values().any(|g| *g > 0))
            {
                first_route.get_or_insert(tick);
            }
            if f.workers
                .iter()
                .any(|w| w.snapshot.impulses.iter().any(|i| i.normal_impulse > 0.))
            {
                first_contact.get_or_insert(tick);
            }
            if epoch == 0
                && tick == 108
                && let Some((owner, previous)) = replacement
            {
                assert_ne!(
                    sensor_world(server.simulation())
                        .local_worker_pids()
                        .unwrap()[owner],
                    previous
                );
            }
            let sample = subscriber
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .expect("canonical /scan delivery");
            assert_eq!(sample.key_expr().as_str(), key);
            assert_eq!(
                sample.attachment().unwrap().to_bytes().as_ref(),
                b"gz.msgs.LaserScan"
            );
            let p = LaserScan::decode(sample.payload().to_bytes().as_ref()).unwrap();
            let seq = tick - 1;
            check_header(&p, epoch_tick, seq);
            let ecs = named(server.simulation(), sensor_name);
            let scan = ecs.get::<LidarSample>().unwrap();
            assert_eq!((scan.time, scan.epoch), (epoch_tick, epoch));
            assert_eq!(p.ranges, scan.ranges);
            let n = native.state(ids[&scanner]).unwrap();
            let origin = [n.position[0] + 0.02, n.position[1], n.position[2] + 0.02];
            let packet_pose = p.world_pose.as_ref().unwrap();
            let xyz = packet_pose.position.as_ref().unwrap();
            assert_eq!([xyz.x, xyz.y, xyz.z], origin);
            let q = packet_pose.orientation.as_ref().unwrap();
            assert_eq!([q.x, q.y, q.z, q.w], n.orientation);
            assert_eq!(
                native.cast_ray(origin, [1., 0., 0.], 6., None).unwrap(),
                Some(0.)
            );
            let mut bits = Vec::new();
            for (beam, &r) in p.ranges.iter().enumerate() {
                let angle = p.angle_min + beam as f64 * p.angle_step;
                let dir = [angle.cos(), angle.sin(), 0.];
                let reference = native
                    .cast_ray(origin, dir, 6., Some(ids[&scanner]))
                    .unwrap();
                let label = if let Some(d) = reference {
                    assert!(r.is_finite() && r > 0.);
                    max_error = max_error.max((r - d).abs());
                    axial_label(&seed, &native, &ids, origin, dir, d)
                } else {
                    assert_eq!(r, f64::INFINITY);
                    "miss"
                };
                *hits.entry(label).or_default() += 1;
                let axial_id = match label {
                    "cylinder" => Some(cylinder_id),
                    "capsule" => Some(target),
                    _ => None,
                };
                if let Some(id) = axial_id
                    && f.body(id).unwrap().0 != f.body(scanner).unwrap().0
                {
                    *foreign_hits.entry(label).or_default() += 1;
                }
                if epoch == 0 && tick == 39 {
                    // Both controls have identical histories up to the external pose edit.
                    let unedited_range = unedited
                        .0
                        .cast_ray(origin, dir, 6., Some(unedited.1[&scanner]))
                        .unwrap();
                    edit_changes += usize::from(unedited_range != reference);
                }
                let d = reference.unwrap_or(f64::INFINITY);
                bit_exact += usize::from(r.to_bits() == d.to_bits());
                bits.push(r.to_bits());
                if let Some(out) = &mut csv {
                    writeln!(out, "{epoch},{tick},{seq},{beam},{label},{r},{d}").unwrap();
                }
            }
            if epoch == 0 {
                trace.push(bits);
            } else {
                assert_eq!(trace[tick as usize - 1], bits);
            }
        }
        let count = server.counters().lidars_queued;
        assert!(server.apply_local_control(WorldControl {
            pause: true,
            ..Default::default()
        }));
        server.update().unwrap();
        assert_eq!(server.counters().lidars_queued, count);
        assert!(
            subscriber
                .recv_timeout(Duration::from_millis(30))
                .unwrap()
                .is_none()
        );
    }
    assert!(four_owners >= 70, "only {four_owners} distributed packets");
    assert!(first_route.unwrap() < first_contact.unwrap());
    for kind in ["box", "cylinder", "capsule", "miss"] {
        assert!(hits[kind] > 50, "{kind}: {hits:?}");
    }
    assert_eq!(bit_exact, 2 * TICKS as usize * 31);
    assert!(max_error < 1e-11, "max ray error {max_error}");
    assert!(edit_changes > 0);
    assert!(foreign_hits["capsule"] > 50 && foreign_hits["cylinder"] > 50);
    println!(
        "Returns from foreign axial owners {foreign_hits:?}. Exactly {edit_changes} rays changed in the first packet after the external target edit vs an unedited native world."
    );
    assert_eq!(server.counters().lidars_queued, 2 * TICKS);
    println!(
        "260 real LaserScan packets, 8060 rays, hits {hits:?}, {bit_exact} native bit-exact, worst error {max_error:e} m, route {first_route:?}, contact {first_contact:?}, replacement {replacement:?}; {four_owners} packets with all four owners occupied; replay after reset matched all 4030 beams"
    );
}
