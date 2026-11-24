// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn loaded(maximum: f64) -> (Simulation, u64) {
    let xml = format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='rig'><static>true</static><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><update_rate>{maximum}</update_rate><topic>/private/rate</topic><frame_id>rate_frame</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal></scan><range><min>0</min><max>20</max></range><noise><type>gaussian</type><mean>0.01</mean><stddev>0.002</stddev></noise></lidar></sensor></link></model><model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
    );
    let sim = load_world_with_engine(
        &xml,
        "private-lidar-rate.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let owner = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<Lidar>())
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    (sim, owner)
}

#[derive(Debug, PartialEq)]
struct SampleBits {
    time: Duration,
    epoch: u64,
    ranges: Vec<u64>,
    translation: [u64; 3],
    rotation: [u64; 4],
}

#[derive(Debug, PartialEq)]
struct State {
    authored: u64,
    effective: u64,
    link: u64,
    directions: Vec<[u64; 3]>,
    metadata: LidarPublication,
    noise_continuation: Vec<u64>,
    sample: Option<SampleBits>,
    cadence: Vec<bool>,
}

fn state(sim: &Simulation, owner: u64) -> State {
    let entity = entity_by_id(sim.world(), owner).unwrap();
    let sensor = sim.world().get::<Lidar>(entity).unwrap();
    let mut schedule = sensor.schedule.clone();
    let cadence = [0, 1, 99, 100, 101, 299, 300, 301, 500, 1000]
        .into_iter()
        .map(|ms| {
            schedule
                .take_sample(Duration::from_millis(ms), false)
                .unwrap()
        })
        .collect();
    let noise_continuation = sensor.noise.as_ref().map_or_else(Vec::new, |noise| {
        let mut probe = (**noise).clone();
        (0..9)
            .map(|_| probe.range(Some(4.0), 20.0).unwrap().to_bits())
            .collect()
    });
    State {
        authored: sensor.sdf_rate_hz.to_bits(),
        effective: sensor.rate_hz.to_bits(),
        link: sensor.link,
        directions: sensor
            .model
            .directions()
            .iter()
            .map(|d| d.map(f64::to_bits))
            .collect(),
        metadata: sim.world().get::<LidarPublication>(entity).unwrap().clone(),
        noise_continuation,
        sample: sim.world().get::<LidarSample>(entity).map(|s| SampleBits {
            time: s.time,
            epoch: s.epoch,
            ranges: s.ranges.iter().map(|r| r.to_bits()).collect(),
            translation: s.world_pose.translation.map(f64::to_bits),
            rotation: s.world_pose.rotation.map(f64::to_bits),
        }),
        cadence,
    }
}

#[test]
fn ignored_and_invalid_lidar_rates_preserve_full_native_sensor_continuation() {
    let (mut limited, owner) = loaded(10.0);
    limited.step_exact(50).unwrap();
    let before = state(&limited, owner);
    for request in [0.0, -1.0, f64::NEG_INFINITY, f64::NAN, f64::INFINITY, 20.0] {
        assert_eq!(
            set_lidar_rate(&mut limited, owner, request).unwrap(),
            RateChange::Ignored
        );
        assert_eq!(state(&limited, owner), before);
    }
    let (mut unlimited, owner) = loaded(0.0);
    set_lidar_rate(&mut unlimited, owner, 5.0).unwrap();
    unlimited.step_exact(1).unwrap();
    let before = state(&unlimited, owner);
    for request in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        assert!(matches!(
            set_lidar_rate(&mut unlimited, owner, request),
            Err(LidarRateError::InvalidPeriod)
        ));
        assert_eq!(state(&unlimited, owner), before);
    }
}

#[test]
fn applied_lidar_rate_changes_only_runtime_period_and_keeps_pending_deadline() {
    let (mut sim, owner) = loaded(10.0);
    sim.step_exact(50).unwrap();
    let mut before = state(&sim, owner);
    assert_eq!(
        set_lidar_rate(&mut sim, owner, 5.0).unwrap(),
        RateChange::Applied
    );
    let after = state(&sim, owner);
    // Every other field includes the original future noise draws and already
    // acquired sample, not merely a rate getter or repeated implementation.
    assert_ne!(after.cadence, before.cadence);
    before.effective = 5.0_f64.to_bits();
    before.cadence = after.cadence.clone();
    assert_eq!(after, before);
    let sensor = sim
        .world()
        .get::<Lidar>(entity_by_id(sim.world(), owner).unwrap())
        .unwrap();
    assert!(!sensor.schedule.is_due(Duration::from_millis(99)));
    assert!(sensor.schedule.is_due(Duration::from_millis(100)));
    assert_eq!(sim.elapsed(), Duration::from_millis(50));
}

#[test]
fn lidar_native_epsilon_sign_and_unlimited_nan_infinity_have_closed_clock_meaning() {
    let (mut sim, owner) = loaded(10.0);
    let lower = 1e-6_f64;
    assert_eq!(
        set_lidar_rate(&mut sim, owner, f64::from_bits(lower.to_bits() - 1)).unwrap(),
        RateChange::Ignored
    );
    assert_eq!(
        set_lidar_rate(&mut sim, owner, lower).unwrap(),
        RateChange::Applied
    );
    let upper = 10.0_f64 + 1e-6;
    let inside = f64::from_bits(upper.to_bits() - 1);
    assert_eq!(
        set_lidar_rate(&mut sim, owner, inside).unwrap(),
        RateChange::Applied
    );
    for request in [upper, f64::from_bits(upper.to_bits() + 1)] {
        assert_eq!(
            set_lidar_rate(&mut sim, owner, request).unwrap(),
            RateChange::Ignored
        );
        assert_eq!(
            lidar_rate(&sim, owner).unwrap().rate_hz.to_bits(),
            inside.to_bits()
        );
    }
    // An authored rate strictly below the native epsilon has no service cap.
    let (mut unlimited, owner) = loaded(f64::from_bits(lower.to_bits() - 1));
    set_lidar_rate(&mut unlimited, owner, -0.0).unwrap();
    assert_eq!(
        lidar_rate(&unlimited, owner).unwrap().rate_hz.to_bits(),
        (-0.0_f64).to_bits()
    );
    set_lidar_rate(&mut unlimited, owner, f64::NEG_INFINITY).unwrap();
    assert_eq!(
        lidar_rate(&unlimited, owner).unwrap().rate_hz.to_bits(),
        0.0_f64.to_bits()
    );
    set_lidar_rate(&mut unlimited, owner, f64::INFINITY).unwrap();
    let e = entity_by_id(unlimited.world(), owner).unwrap();
    let mut schedule = unlimited.world().get::<Lidar>(e).unwrap().schedule.clone();
    assert!(schedule.take_sample(Duration::ZERO, false).unwrap());
    assert!(!schedule.take_sample(Duration::ZERO, false).unwrap());
    assert!(
        schedule
            .take_sample(Duration::from_nanos(1), false)
            .unwrap()
    );
    set_lidar_rate(&mut unlimited, owner, f64::NAN).unwrap();
    assert!(lidar_rate(&unlimited, owner).unwrap().rate_hz.is_nan());
    let mut schedule = unlimited.world().get::<Lidar>(e).unwrap().schedule.clone();
    assert!(schedule.take_sample(Duration::ZERO, false).unwrap());
    assert!(schedule.take_sample(Duration::ZERO, false).unwrap());
    assert!(unlimited.world().get::<LidarSample>(e).is_none());
}

#[test]
fn forged_lidar_publication_cannot_replace_private_sensor_or_consume_an_identity() {
    let (mut sim, owner) = loaded(10.0);
    sim.step_exact(1).unwrap();
    let real = entity_by_id(sim.world(), owner).unwrap();
    let metadata = sim.world().get::<LidarPublication>(real).unwrap().clone();
    let fake = sim.world_mut().spawn(metadata).id();
    let fake_id = gz_sim_core::assign_id(sim.world_mut(), fake).unwrap();
    let before = state(&sim, owner);
    let next = gz_sim_core::next_entity_id(sim.world()).unwrap();
    assert!(
        matches!(lidar_rate(&sim, fake_id), Err(LidarRateError::MissingLidar(id)) if id == fake_id)
    );
    assert!(
        matches!(set_lidar_rate(&mut sim, fake_id, 5.0), Err(LidarRateError::MissingLidar(id)) if id == fake_id)
    );
    assert_eq!(state(&sim, owner), before);
    assert_eq!(gz_sim_core::next_entity_id(sim.world()).unwrap(), next);
    assert!(sim.world().get::<Lidar>(fake).is_none());
}
