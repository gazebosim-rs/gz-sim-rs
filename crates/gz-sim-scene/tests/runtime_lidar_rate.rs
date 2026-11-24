// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, StepFault, entity_by_id};
use gz_sim_scene::{
    LidarPublication, LidarRateError, LidarSample, Name, PhysicsEngine, RateChange,
    clear_lidar_interest, lidar_rate, load_world_with_engine, remove_model, reset_loaded_world,
    set_lidar_interest, set_lidar_rate,
};
use std::time::Duration;

fn source(maximum: f64, owners: usize, noisy: bool) -> String {
    let mut xml =
        String::from("<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity>");
    for i in 0..owners {
        let noise = if noisy {
            "<noise><type>gaussian</type><mean>0.01</mean><stddev>0.002</stddev></noise>"
        } else {
            ""
        };
        xml += &format!(
            "<model name='rig{i}'><static>true</static><pose>0 {} 0 0 0 0</pose><link name='body'><collision name='self'><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision><sensor name='scan' type='lidar'><pose>0 0 0.25 0 0 0</pose><update_rate>{maximum}</update_rate><topic>/rates/scan{i}</topic><frame_id>frame{i}</frame_id><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.3</min_angle><max_angle>0.3</max_angle></horizontal><vertical><samples>3</samples><resolution>1</resolution><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan><range><min>0</min><max>20</max></range>{noise}</lidar></sensor></link></model>",
            i * 2
        );
    }
    xml + "<model name='wall'><static>true</static><pose>5 0 0 0 0 0</pose><link name='body'><collision name='wall'><geometry><box><size>2 100 100</size></box></geometry></collision></link></model></world></sdf>"
}

fn load(maximum: f64, owners: usize, noisy: bool, step: Duration) -> Simulation {
    load_world_with_engine(
        &source(maximum, owners, noisy),
        "lidar-rate.sdf",
        "default",
        step,
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

fn sensor(sim: &Simulation, owner: usize) -> u64 {
    id(sim, &format!("default::rig{owner}::body::scan"))
}

fn stamp(sim: &Simulation, owner: u64) -> Option<Duration> {
    entity_by_id(sim.world(), owner)
        .and_then(|e| sim.world().get::<LidarSample>(e))
        .map(|s| s.time)
}

#[derive(Debug, PartialEq)]
struct SampleBits {
    time: Duration,
    epoch: u64,
    ranges: Vec<u64>,
    translation: [u64; 3],
    rotation: [u64; 4],
}

fn sample_bits(sim: &Simulation, owner: u64) -> Option<SampleBits> {
    let s = sim
        .world()
        .get::<LidarSample>(entity_by_id(sim.world(), owner)?)?;
    Some(SampleBits {
        time: s.time,
        epoch: s.epoch,
        ranges: s.ranges.iter().map(|v| v.to_bits()).collect(),
        translation: s.world_pose.translation.map(f64::to_bits),
        rotation: s.world_pose.rotation.map(f64::to_bits),
    })
}

fn native_number(value: &serde_json::Value) -> f64 {
    match value.as_str() {
        Some("nan") => f64::NAN,
        Some("inf") => f64::INFINITY,
        Some("-inf") => f64::NEG_INFINITY,
        None => value.as_f64().unwrap(),
        _ => panic!("unknown retained native numeric tag"),
    }
}

#[test]
fn lidar_rate_service_policy_matches_retained_native_sensor_numeric_observations() {
    // These are retained Sensor::Implementation::SetRate observations, shared
    // by native sensor classes, not a claim to execute a native lidar here.
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/runtime_imu_rate.json")).unwrap();
    let groups = fixture["observations"]["bounds"].as_array().unwrap();
    assert_eq!(groups.len(), 3);
    for group in groups {
        let maximum = group["sdf_max"].as_f64().unwrap();
        let mut sim = load(maximum, 1, false, Duration::from_millis(1));
        let owner = sensor(&sim, 0);
        assert_eq!(
            lidar_rate(&sim, owner).unwrap().sdf_max_hz.to_bits(),
            maximum.to_bits()
        );
        let rows = group["rows"].as_array().unwrap();
        assert_eq!(rows.len(), 15);
        for row in rows {
            let mut requested = native_number(&row["request"]);
            if row["request_negative_zero"].as_bool().unwrap() {
                requested = -0.0;
            }
            let before = (sim.elapsed(), sim.epoch(), sample_bits(&sim, owner));
            let _ = set_lidar_rate(&mut sim, owner, requested).unwrap();
            let actual = lidar_rate(&sim, owner).unwrap();
            let expected = native_number(&row["rate"]);
            assert!(
                actual.rate_hz == expected || actual.rate_hz.is_nan() && expected.is_nan(),
                "{row}"
            );
            assert_eq!(
                actual.rate_hz == 0.0 && actual.rate_hz.is_sign_negative(),
                row["rate_negative_zero"].as_bool().unwrap()
            );
            assert_eq!(actual.sdf_max_hz.to_bits(), maximum.to_bits());
            assert_eq!(
                (sim.elapsed(), sim.epoch(), sample_bits(&sim, owner)),
                before
            );
        }
    }
}

#[test]
fn lidar_rate_phase_changes_preserve_pending_deadlines_and_measured_sensor_geometry() {
    let mut sim = load(10.0, 1, false, Duration::from_millis(1));
    let owner = sensor(&sim, 0);
    sim.step_exact(50).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(1)));
    let before = sample_bits(&sim, owner);
    assert_eq!(
        set_lidar_rate(&mut sim, owner, 5.0).unwrap(),
        RateChange::Applied
    );
    assert_eq!(sample_bits(&sim, owner), before);
    sim.step_exact(49).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(1)));
    sim.step_exact(1).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(100)));
    set_lidar_rate(&mut sim, owner, 10.0).unwrap();
    sim.step_exact(199).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(100)));
    sim.step_exact(1).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(300)));
    sim.step_exact(99).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(300)));
    sim.step_exact(1).unwrap();
    let s = sim
        .world()
        .get::<LidarSample>(entity_by_id(sim.world(), owner).unwrap())
        .unwrap();
    assert_eq!(s.time, Duration::from_millis(400));
    assert_eq!(s.ranges.len(), 9);
    for v in 0..3 {
        for h in 0..3 {
            let pitch = -0.2 + v as f64 * 0.2;
            let yaw = -0.3 + h as f64 * 0.3;
            assert!((s.ranges[v * 3 + h] - 4.0 / (pitch.cos() * yaw.cos())).abs() < 1e-10);
        }
    }
}

#[test]
fn lidar_rate_catchup_matches_independent_clock_oracle_at_three_step_durations() {
    for step_ms in [1_u64, 7, 80] {
        let mut sim = load(10.0, 1, false, Duration::from_millis(step_ms));
        let owner = sensor(&sim, 0);
        let (mut next, mut period, mut last) = (0_u64, 100_u64, 0_u64);
        for turn in 0..24 {
            if turn == 5 || turn == 12 {
                let rate = if turn == 5 { 5.0 } else { 10.0 };
                set_lidar_rate(&mut sim, owner, rate).unwrap();
                period = if turn == 5 { 200 } else { 100 };
            }
            sim.step_exact(1).unwrap();
            let now = (turn + 1) * step_ms;
            if now >= next {
                last = now;
                next += period;
                while next <= now {
                    next += period;
                }
            }
            assert_eq!(
                stamp(&sim, owner),
                Some(Duration::from_millis(last)),
                "dt={step_ms}, turn={turn}"
            );
        }
    }
}

#[test]
fn eight_lidar_rates_are_independent_and_inactive_owners_do_not_acquire() {
    let mut sim = load(200.0, 8, false, Duration::from_millis(1));
    let owners: Vec<_> = (0..8).map(|i| sensor(&sim, i)).collect();
    let rates = [5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 125.0, 200.0];
    let periods = [200_u64, 100, 50, 40, 20, 10, 8, 5];
    for (&owner, rate) in owners.iter().zip(rates) {
        assert_eq!(
            set_lidar_rate(&mut sim, owner, rate).unwrap(),
            RateChange::Applied
        );
    }
    let selected = [
        entity_by_id(sim.world(), owners[0]).unwrap(),
        entity_by_id(sim.world(), owners[7]).unwrap(),
    ];
    set_lidar_interest(&mut sim, selected);
    sim.step_exact(21).unwrap();
    assert_eq!(stamp(&sim, owners[0]), Some(Duration::from_millis(1)));
    assert_eq!(stamp(&sim, owners[7]), Some(Duration::from_millis(20)));
    for &owner in &owners[1..7] {
        assert!(stamp(&sim, owner).is_none());
    }
    let before = sample_bits(&sim, owners[7]);
    set_lidar_rate(&mut sim, owners[3], 10.0).unwrap();
    assert!(stamp(&sim, owners[3]).is_none());
    assert_eq!(sample_bits(&sim, owners[7]), before);
    set_lidar_rate(&mut sim, owners[3], rates[3]).unwrap();
    clear_lidar_interest(&mut sim);
    sim.step_exact(1).unwrap();
    let mut next = [0_u64; 8];
    let mut last = [22_u64; 8];
    next[0] = 200;
    last[0] = 1;
    next[7] = 25;
    last[7] = 20;
    for (deadline, period) in next[1..7].iter_mut().zip(&periods[1..7]) {
        while *deadline <= 22 {
            *deadline += *period;
        }
    }
    for now in 23_u64..=205 {
        sim.step_exact(1).unwrap();
        for (i, &owner) in owners.iter().enumerate() {
            if now >= next[i] {
                last[i] = now;
                next[i] += periods[i];
            }
            assert_eq!(
                stamp(&sim, owner),
                Some(Duration::from_millis(last[i])),
                "owner={i} now={now}"
            );
            assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, rates[i]);
        }
    }
}

#[test]
fn lidar_rate_changes_do_not_draw_noise_or_replace_an_existing_acquisition() {
    let mut controlled = load(100.0, 1, true, Duration::from_millis(1));
    let mut twin = load(100.0, 1, true, Duration::from_millis(1));
    let a = sensor(&controlled, 0);
    let b = sensor(&twin, 0);
    for _ in 0..6 {
        controlled.step_exact(10).unwrap();
        twin.step_exact(10).unwrap();
        assert_eq!(sample_bits(&controlled, a), sample_bits(&twin, b));
        let before = sample_bits(&controlled, a);
        assert_eq!(
            set_lidar_rate(&mut controlled, a, 100.0).unwrap(),
            RateChange::Applied
        );
        assert_eq!(
            set_lidar_rate(&mut controlled, a, 200.0).unwrap(),
            RateChange::Ignored
        );
        assert_eq!(sample_bits(&controlled, a), before);
    }
    set_lidar_interest(&mut controlled, []);
    let metadata = controlled
        .world()
        .get::<LidarPublication>(entity_by_id(controlled.world(), a).unwrap())
        .unwrap()
        .clone();
    controlled.set_paused(true);
    let before = (controlled.elapsed(), controlled.epoch());
    set_lidar_rate(&mut controlled, a, 50.0).unwrap();
    controlled.tick().unwrap();
    assert_eq!((controlled.elapsed(), controlled.epoch()), before);
    assert!(stamp(&controlled, a).is_none());
    assert_eq!(
        controlled
            .world()
            .get::<LidarPublication>(entity_by_id(controlled.world(), a).unwrap())
            .unwrap(),
        &metadata
    );
}

#[test]
fn lidar_rate_errors_preserve_configuration_and_clock_fault_has_first_precedence() {
    let mut sim = load(0.0, 1, false, Duration::from_millis(1));
    let owner = sensor(&sim, 0);
    set_lidar_rate(&mut sim, owner, 5.0).unwrap();
    for rate in [f64::MIN_POSITIVE, f64::from_bits(1)] {
        assert!(matches!(
            set_lidar_rate(&mut sim, owner, rate),
            Err(LidarRateError::InvalidPeriod)
        ));
        assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, 5.0);
    }
    let body = id(&sim, "default::rig0::body");
    for missing in [0, u64::MAX, body] {
        assert!(
            matches!(lidar_rate(&sim, missing), Err(LidarRateError::MissingLidar(id)) if id == missing)
        );
        assert!(
            matches!(set_lidar_rate(&mut sim, missing, 1.0), Err(LidarRateError::MissingLidar(id)) if id == missing)
        );
    }
    sim.world()
        .resource::<StepFault>()
        .report(std::io::Error::other("lidar rate fault"));
    assert!(matches!(
        set_lidar_rate(&mut sim, 0, 1.0),
        Err(LidarRateError::Clock(_))
    ));
    assert!(matches!(
        set_lidar_rate(&mut sim, owner, 1.0),
        Err(LidarRateError::Clock(_))
    ));
    assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, 5.0);
}

#[test]
fn lidar_time_reset_keeps_runtime_rate_but_loaded_reset_restores_authored_rate_and_owner() {
    let mut sim = load(100.0, 1, false, Duration::from_millis(1));
    let owner = sensor(&sim, 0);
    let model = id(&sim, "default::rig0");
    set_lidar_rate(&mut sim, owner, 50.0).unwrap();
    sim.step_exact(21).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(20)));
    sim.reset().unwrap();
    assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, 50.0);
    assert!(stamp(&sim, owner).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(1)));
    assert_eq!(
        sim.world()
            .get::<LidarSample>(entity_by_id(sim.world(), owner).unwrap())
            .unwrap()
            .epoch,
        1
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, 100.0);
    assert_eq!(lidar_rate(&sim, owner).unwrap().sdf_max_hz, 100.0);
    remove_model(&mut sim, model).unwrap();
    assert!(matches!(
        lidar_rate(&sim, owner),
        Err(LidarRateError::MissingLidar(_))
    ));
    assert!(matches!(
        set_lidar_rate(&mut sim, owner, 10.0),
        Err(LidarRateError::MissingLidar(_))
    ));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sensor(&sim, 0), owner);
    assert_eq!(lidar_rate(&sim, owner).unwrap().rate_hz, 100.0);
    sim.step_exact(1).unwrap();
    assert_eq!(stamp(&sim, owner), Some(Duration::from_millis(1)));
}
