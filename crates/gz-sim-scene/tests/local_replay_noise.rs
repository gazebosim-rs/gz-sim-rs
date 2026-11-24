// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{ImuSample, Name, load_local_replay_world};

fn sensor(name: &str, rate: u32, extra: &str) -> String {
    format!(
        r#"<sensor name="{name}" type="imu"><always_on>true</always_on><update_rate>{rate}</update_rate><imu><angular_velocity><z><noise type="gaussian"><stddev>0.1</stddev><bias_stddev>0.04</bias_stddev></noise></z></angular_velocity><linear_acceleration>{extra}</linear_acceleration></imu></sensor>"#
    )
}
fn world(sensors: &str) -> Simulation {
    let xml = format!(
        r#"<sdf version="1.12"><world name="bench"><model name="frame"><static>true</static><link name="base"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision>{sensors}</link></model></world></sdf>"#
    );
    load_local_replay_world(&xml, "imu.sdf", "bench", [23; 32]).unwrap()
}
#[allow(deprecated)]
fn reading(sim: &Simulation, name: &str) -> (u64, u128, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == format!("bench::frame::base::{name}"))
        })
        .unwrap();
    let sample = e.get::<ImuSample>().unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        sample.time.as_nanos(),
        sample.reading.as_ref().unwrap().angular_velocity[2].to_bits(),
    )
}

#[test]
fn white_channel_uses_scoped_name_step_and_draw_counter_independently_of_sensor_order() {
    let x = r#"<x><noise type="gaussian"><stddev>0.3</stddev></noise></x>"#;
    let mut fast = world(&format!(
        "{}{}",
        sensor("main", 1000, ""),
        sensor("spare", 1000, "")
    ));
    let mut sparse = world(&format!(
        "{}{}",
        sensor("spare", 1000, ""),
        sensor("main", 500, x)
    ));
    let mut comparisons = 0;
    for _ in 0..24 {
        fast.step_exact(1).unwrap();
        sparse.step_exact(1).unwrap();
        let (id, time, draw) = reading(&fast, "main");
        let (other_id, other_time, other_draw) = reading(&sparse, "main");
        assert_ne!(id, other_id);
        assert_ne!(draw, reading(&fast, "spare").2);
        if time == other_time {
            assert_eq!(draw, other_draw);
            comparisons += 1;
        }
    }
    assert!((10..=13).contains(&comparisons));
}
