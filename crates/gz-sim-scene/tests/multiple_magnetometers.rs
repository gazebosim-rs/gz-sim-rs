// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::entity::Entity;
use gz_sim_core::{EntityId, Simulation, StepError, StepFault, entity_by_id};
use gz_sim_scene::{
    LoadError, MagnetometerPublication, MagnetometerSample, Name, PhysicsEngine, SceneResetError,
    WorldPose, clear_magnetometer_interest, load_world_seeded, queue_link_world_wrench,
    remove_model, reset_loaded_world, set_magnetometer_interest,
};
use std::time::Duration;

const XML: &str = include_str!("fixtures/m17_magnetometer.sdf");
const FIELD: [f64; 3] = [2e-5, -3e-5, 4e-5];
const MEANS: [f64; 3] = [1e-5, -2e-5, 3e-5];
const MEAN_NOISE: &str = "<x><noise type='gaussian'><mean>0.00001</mean></noise></x><y><noise type='gaussian'><mean>-0.00002</mean></noise></y><z><noise type='gaussian_quantized'><mean>0.00003</mean></noise></z>";
const RANDOM_NOISE: &str = "<x><noise type='gaussian'><stddev>0.2</stddev><bias_stddev>0.1</bias_stddev><dynamic_bias_stddev>8</dynamic_bias_stddev><dynamic_bias_correlation_time>0.2</dynamic_bias_correlation_time></noise></x><y><noise type='gaussian'><stddev>0.3</stddev></noise></y><z><noise type='gaussian_quantized'><stddev>0.4</stddev></noise></z>";
fn sensor_xml(index: usize, rate: u32, noise: &str) -> String {
    let rotation = if index.is_multiple_of(2) {
        "0.3 0.4 -0.2"
    } else {
        "-0.2 0.1 0.5"
    };
    format!(
        "<sensor name='magnetometer{index}' type='magnetometer'><pose>1 0 0.5 {rotation}</pose><update_rate>{rate}</update_rate><topic>/m52/magnetometer{index}</topic><frame_id>rig/body/magnetometer{index}</frame_id><magnetometer>{noise}</magnetometer></sensor>"
    )
}
fn world(sensors: &str) -> String {
    let start = XML.find("    <sensor ").unwrap();
    let end = start + XML[start..].find("</sensor>").unwrap() + "</sensor>".len();
    format!("{}{}{}", &XML[..start], sensors, &XML[end..])
}
fn pair(a: &str, b: &str) -> String {
    world(&format!(
        "{}{}",
        sensor_xml(0, 100, a),
        sensor_xml(1, 50, b)
    ))
}
fn try_load(xml: &str, seed: u8) -> Result<Simulation, LoadError> {
    load_world_seeded(
        xml,
        "m52.sdf",
        "default",
        Some(Duration::from_millis(1)),
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
        [seed; 32],
    )
}
fn load(xml: &str, seed: u8) -> Simulation {
    try_load(xml, seed).unwrap()
}
fn named(sim: &Simulation, name: &str) -> (Entity, u64) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (e.id(), e.get::<EntityId>().unwrap().get())
}
fn sensor(sim: &Simulation, index: usize) -> Entity {
    named(sim, &format!("default::rig::body::magnetometer{index}")).0
}
fn reading_at(sim: &Simulation, e: Entity) -> (Duration, u64, [f64; 3]) {
    let s = sim.world().get::<MagnetometerSample>(e).unwrap();
    (s.time, s.epoch, s.reading.as_ref().unwrap().field_tesla)
}
fn sample(sim: &Simulation, index: usize) -> (Duration, u64, [f64; 3]) {
    reading_at(sim, sensor(sim, index))
}
fn pose(sim: &Simulation, name: &str) -> ([f64; 3], [f64; 4]) {
    let p = sim.world().get::<WorldPose>(named(sim, name).0).unwrap();
    (p.translation, p.rotation)
}
fn wrench(sim: &mut Simulation, name: &str, torque: [f64; 3]) {
    let id = named(sim, name).1;
    queue_link_world_wrench(sim, id, [8.0, -4.0, 2.0], torque).unwrap();
}
// Independent transpose of the actual sensor-to-WORLD rotation; no translation term.
fn rotated(q: [f64; 4], field: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = q;
    let [a, b, c] = field;
    [
        (1.0 - 2.0 * (y * y + z * z)) * a + 2.0 * (x * y + z * w) * b + 2.0 * (x * z - y * w) * c,
        2.0 * (x * y - z * w) * a + (1.0 - 2.0 * (x * x + z * z)) * b + 2.0 * (y * z + x * w) * c,
        2.0 * (x * z + y * w) * a + 2.0 * (y * z - x * w) * b + (1.0 - 2.0 * (x * x + y * y)) * c,
    ]
}
fn geometry(sim: &Simulation, index: usize, means: [f64; 3], field: [f64; 3]) {
    let current = sample(sim, index);
    assert_eq!(current.0, sim.elapsed());
    let q = sim
        .world()
        .get::<WorldPose>(sensor(sim, index))
        .unwrap()
        .rotation;
    for ((actual, ideal), mean) in current.2.into_iter().zip(rotated(q, field)).zip(means) {
        assert!((actual - ideal - mean).abs() < 1e-18);
    }
}

#[test]
fn distinct_rotations_rates_and_axis_means_follow_actual_acquisitions_and_loaded_reset() {
    let mut sim = load(&pair("", MEAN_NOISE), 52);
    let handles = [sensor(&sim, 0), sensor(&sim, 1)];
    let ids = handles.map(|e| sim.world().get::<EntityId>(e).unwrap().get());
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<MagnetometerPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    let initial = pose(&sim, "default::rig::body");
    let counts = sim.scheduled_system_counts();
    wrench(&mut sim, "default::rig::body", [2.0, -3.0, 4.0]);
    let mut previous = [None, None];
    let mut stamps = [Vec::new(), Vec::new()];
    for _ in 0..41 {
        sim.step_exact(1).unwrap();
        for (index, (last, times)) in previous.iter_mut().zip(&mut stamps).enumerate() {
            let value = sample(&sim, index);
            if value.0 == sim.elapsed() {
                geometry(
                    &sim,
                    index,
                    if index == 0 { [0.0; 3] } else { MEANS },
                    FIELD,
                );
                times.push(value.0);
            } else {
                assert_eq!(Some(value), *last);
            }
            *last = Some(value);
        }
    }
    assert_eq!(stamps[0], [1, 10, 20, 30, 40].map(Duration::from_millis));
    assert_eq!(stamps[1], [1, 20, 40].map(Duration::from_millis));
    assert_ne!(pose(&sim, "default::rig::body").0, initial.0);
    assert_ne!(pose(&sim, "default::rig::body").1, initial.1);
    assert_ne!(sample(&sim, 0).2, sample(&sim, 1).2);
    let final_samples = [sample(&sim, 0), sample(&sim, 1)];
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(pose(&sim, "default::rig::body"), initial);
    assert_eq!(sim.scheduled_system_counts(), counts);
    for (index, (id, meta)) in ids.into_iter().zip(&metadata).enumerate() {
        assert_eq!(entity_by_id(sim.world(), id), Some(sensor(&sim, index)));
        assert_ne!(sensor(&sim, index), handles[index]);
        assert!(
            sim.world()
                .get::<MagnetometerSample>(sensor(&sim, index))
                .is_none()
        );
        assert_eq!(
            sim.world()
                .get::<MagnetometerPublication>(sensor(&sim, index)),
            Some(meta)
        );
        assert_eq!(meta.entity_name, format!("rig::body::magnetometer{index}"));
        assert_eq!(meta.topic, format!("/m52/magnetometer{index}"));
        assert_eq!(meta.frame_id, format!("rig/body/magnetometer{index}"));
    }
    wrench(&mut sim, "default::rig::body", [2.0, -3.0, 4.0]);
    sim.step_exact(41).unwrap();
    for (index, old) in final_samples.into_iter().enumerate() {
        let current = sample(&sim, index);
        assert_eq!((current.0, current.2), (old.0, old.2));
        assert_eq!(current.1, 1);
    }
}

#[test]
fn pure_translation_changes_neither_ideal_sensor_vector_for_differently_rotated_owners() {
    let mut translated = load(&pair("", ""), 52);
    let mut stationary = load(&pair("", ""), 52);
    let initial = pose(&translated, "default::rig::body");
    wrench(&mut translated, "default::rig::body", [0.0; 3]);
    for _ in 0..21 {
        translated.step_exact(1).unwrap();
        stationary.step_exact(1).unwrap();
        assert_eq!(
            pose(&translated, "default::rig::body").1,
            pose(&stationary, "default::rig::body").1
        );
        for index in [0, 1] {
            assert_eq!(sample(&translated, index), sample(&stationary, index));
        }
    }
    assert_ne!(pose(&translated, "default::rig::body").0, initial.0);
}

#[test]
fn independent_demand_preserves_xyz_streams_deadlines_pause_and_both_resets() {
    // Identical stationary measurement inputs isolate only the per-owner noise streams.
    let xml = pair(RANDOM_NOISE, RANDOM_NOISE).replace("-0.2 0.1 0.5", "0.3 0.4 -0.2");
    let mut selected = load(&xml, 52);
    let mut all = load(&xml, 52);
    let mut first = load(&xml, 52);
    let mut different = load(&xml, 53);
    let a = sensor(&selected, 0);
    let b = sensor(&selected, 1);
    set_magnetometer_interest(&mut selected, [a]);
    first.step_exact(1).unwrap();
    assert_eq!(
        pose(&first, "default::rig::body::magnetometer0"),
        pose(&first, "default::rig::body::magnetometer1")
    );
    assert_ne!(sample(&first, 0).2, sample(&first, 1).2);
    for _ in 0..50 {
        selected.step_exact(1).unwrap();
        all.step_exact(1).unwrap();
        different.step_exact(1).unwrap();
        assert_eq!(sample(&selected, 0), sample(&all, 0));
        assert!(selected.world().get::<MagnetometerSample>(b).is_none());
    }
    assert_ne!(sample(&selected, 0).2, sample(&different, 0).2);
    set_magnetometer_interest(&mut selected, [a, b]);
    selected.step_exact(1).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(51));
    assert_eq!(sample(&selected, 1).2, sample(&first, 1).2);
    set_magnetometer_interest(&mut selected, [a]);
    assert!(selected.world().get::<MagnetometerSample>(b).is_none());
    let held = sample(&selected, 0);
    selected.set_paused(true);
    selected.tick().unwrap();
    assert_eq!(sample(&selected, 0), held);
    selected.set_paused(false);
    selected.step_exact(0).unwrap();
    assert_eq!(sample(&selected, 0), held);
    set_magnetometer_interest(&mut selected, []);
    selected.step_exact(9).unwrap();
    assert!(selected.world().get::<MagnetometerSample>(a).is_none());
    set_magnetometer_interest(&mut selected, [b]);
    selected.step_exact(1).unwrap();
    first.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(61));
    assert_eq!(sample(&selected, 1).2, sample(&first, 1).2);
    clear_magnetometer_interest(&mut selected);
    selected.step_exact(19).unwrap();
    assert_eq!(sample(&selected, 1).0, Duration::from_millis(80));
    let ids = [0, 1].map(|i| {
        selected
            .world()
            .get::<EntityId>(sensor(&selected, i))
            .unwrap()
            .get()
    });
    reset_loaded_world(&mut selected, |_| ()).unwrap();
    selected.step_exact(1).unwrap();
    let mut fresh = load(&xml, 52);
    fresh.step_exact(1).unwrap();
    for (index, id) in ids.into_iter().enumerate() {
        assert_eq!(
            entity_by_id(selected.world(), id),
            Some(sensor(&selected, index))
        );
        assert_eq!(sample(&selected, index).2, sample(&fresh, index).2);
        assert_eq!(sample(&selected, index).1, 1);
    }
    let old = sensor(&selected, 0);
    selected.reset().unwrap();
    assert_eq!(sensor(&selected, 0), old);
    assert!(selected.world().get::<MagnetometerSample>(old).is_none());
    selected.step_exact(1).unwrap();
    for index in [0, 1] {
        assert_eq!(sample(&selected, index).2, sample(&fresh, index).2);
        assert_eq!(sample(&selected, index).1, 2);
    }
}

#[test]
fn eight_consumers_share_callbacks_and_late_invalid_or_ninth_sensor_never_authorizes_field() {
    let sensors = (0..8).map(|i| sensor_xml(i, 100, "")).collect::<String>();
    let xml = world(&sensors);
    let mut eight = load(&xml, 52);
    let one = load(&world(&sensor_xml(0, 100, "")), 52);
    let absent_xml = world("").replace(
        "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
        "",
    );
    let absent = load(&absent_xml, 52);
    let counts = absent.scheduled_system_counts();
    assert_eq!(
        eight.scheduled_system_counts(),
        one.scheduled_system_counts()
    );
    assert_eq!(one.scheduled_system_counts(), (counts.0 + 1, counts.1 + 1));
    eight.step_exact(1).unwrap();
    for index in 0..8 {
        geometry(&eight, index, [0.0; 3], FIELD);
    }
    let last = sample(&eight, 7);
    let clock = (eight.elapsed(), eight.epoch());
    let ninth = world(&format!("{sensors}{}", sensor_xml(8, 100, "")));
    assert!(matches!(
        try_load(&ninth, 52),
        Err(LoadError::Unsupported {
            feature: "multiple magnetometers",
            ..
        })
    ));
    let last_sensor = sensor_xml(7, 100, "");
    for (replacement, feature) in [
        (
            last_sensor.replace(
                "<update_rate>100</update_rate>",
                "<update_rate>1e-300</update_rate>",
            ),
            "unrepresentable sensor rate",
        ),
        (
            last_sensor.replace(
                "<magnetometer></magnetometer>",
                "<enable_metrics>true</enable_metrics><magnetometer/>",
            ),
            "magnetometer metadata",
        ),
        (
            last_sensor.replace(
                "<magnetometer></magnetometer>",
                "<magnetometer><x><noise type='none'><mean>1</mean></noise></x></magnetometer>",
            ),
            "unrepresentable magnetometer noise",
        ),
    ] {
        assert!(
            matches!(try_load(&xml.replace(&last_sensor,&replacement),52),Err(LoadError::Unsupported{feature:actual,..}) if actual==feature)
        );
    }
    let invalid_field = xml.replace(
        "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
        "<magnetic_field>NaN 0 0</magnetic_field>",
    );
    assert!(try_load(&invalid_field, 52).is_err());
    let spherical=xml.replace("<gravity>","<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates><gravity>");
    assert!(matches!(
        try_load(&spherical, 52),
        Err(LoadError::Unsupported {
            feature: "magnetometer spherical coordinates",
            ..
        })
    ));
    assert!(matches!(
        try_load(&world(""), 52),
        Err(LoadError::Unsupported {
            feature: "world magnetic field",
            ..
        })
    ));
    assert_eq!(sample(&eight, 7), last);
    assert_eq!((eight.elapsed(), eight.epoch()), clock);
}

#[test]
fn appending_a_peer_preserves_first_public_id_default_field_and_seeded_values() {
    for field_xml in [
        "",
        "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
    ] {
        for noise in ["", RANDOM_NOISE] {
            let build = |sensors: &str| {
                world(sensors).replace(
                    "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
                    field_xml,
                )
            };
            let mut single = load(&build(&sensor_xml(0, 100, noise)), 52);
            let mut pair = load(
                &build(&format!(
                    "{}{}",
                    sensor_xml(0, 100, noise),
                    sensor_xml(1, 50, "")
                )),
                52,
            );
            assert_eq!(
                single
                    .world()
                    .get::<EntityId>(sensor(&single, 0))
                    .unwrap()
                    .get(),
                pair.world()
                    .get::<EntityId>(sensor(&pair, 0))
                    .unwrap()
                    .get()
            );
            wrench(&mut single, "default::rig::body", [2.0, -3.0, 4.0]);
            wrench(&mut pair, "default::rig::body", [2.0, -3.0, 4.0]);
            for _ in 0..31 {
                single.step_exact(1).unwrap();
                pair.step_exact(1).unwrap();
                assert_eq!(sample(&single, 0), sample(&pair, 0));
                assert_eq!(
                    pose(&single, "default::rig::body"),
                    pose(&pair, "default::rig::body")
                );
            }
        }
    }
}

fn two_models() -> String {
    let one = world(&sensor_xml(0, 100, ""));
    let end = one.rfind("</world>").unwrap();
    format!(
        "{}<model name='peer'><pose>2 -1 4 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='box'><geometry><box><size>1 1 1</size></box></geometry></collision>{}</link></model></world></sdf>",
        &one[..end],
        sensor_xml(1, 50, "")
    )
}
#[test]
fn independent_owner_removal_last_retirement_and_loaded_restore_preserve_identities() {
    let mut sim = load(&two_models(), 52);
    let a = sensor(&sim, 0);
    let b = named(&sim, "default::peer::body::magnetometer1").0;
    let ids = [a, b].map(|e| sim.world().get::<EntityId>(e).unwrap().get());
    let counts = sim.scheduled_system_counts();
    sim.step_exact(1).unwrap();
    let first = reading_at(&sim, b);
    let first_model = named(&sim, "default::rig").1;
    remove_model(&mut sim, first_model).unwrap();
    assert!(entity_by_id(sim.world(), ids[0]).is_none());
    assert_eq!(sim.scheduled_system_counts(), counts);
    sim.step_exact(19).unwrap();
    let next = reading_at(&sim, b);
    assert_eq!(next.0, Duration::from_millis(20));
    assert_eq!(next.2, first.2);
    let peer_model = named(&sim, "default::peer").1;
    remove_model(&mut sim, peer_model).unwrap();
    let retired = sim.scheduled_system_counts();
    assert_eq!(counts, (retired.0 + 1, retired.1 + 1));
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.scheduled_system_counts(), counts);
    for id in ids {
        let entity = entity_by_id(sim.world(), id).unwrap();
        assert!(sim.world().get::<MagnetometerSample>(entity).is_none());
    }
    sim.step_exact(1).unwrap();
    for id in ids {
        assert_eq!(
            reading_at(&sim, entity_by_id(sim.world(), id).unwrap()).0,
            Duration::from_millis(1)
        );
    }
}
fn fault_clock(fault: bevy_ecs::prelude::Res<StepFault>) {
    fault.report(StepError::Overflow);
}
#[test]
fn terminal_reset_refusal_keeps_all_metadata_samples_and_physical_poses_unchanged() {
    let mut sim = load(&pair("", MEAN_NOISE), 52);
    sim.step_exact(1).unwrap();
    let values = [sample(&sim, 0), sample(&sim, 1)];
    let metadata = [0, 1].map(|i| {
        sim.world()
            .get::<MagnetometerPublication>(sensor(&sim, i))
            .unwrap()
            .clone()
    });
    let body = pose(&sim, "default::rig::body");
    sim.add_systems(gz_sim_core::Phase::PostStep, fault_clock);
    assert_eq!(sim.step_exact(1).unwrap_err(), StepError::Faulted);
    let terminal_values = [sample(&sim, 0), sample(&sim, 1)];
    assert_eq!(values, terminal_values);
    let clock = (sim.elapsed(), sim.epoch());
    let mut called = false;
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| {
            called = true;
        }),
        Err(SceneResetError::Clock(StepError::Faulted))
    ));
    assert!(!called);
    assert_eq!([sample(&sim, 0), sample(&sim, 1)], terminal_values);
    assert_eq!((sim.elapsed(), sim.epoch()), clock);
    assert_eq!(pose(&sim, "default::rig::body"), body);
    for (index, meta) in metadata.iter().enumerate() {
        assert_eq!(
            sim.world()
                .get::<MagnetometerPublication>(sensor(&sim, index)),
            Some(meta)
        );
    }
}

#[test]
fn zero_two_and_eight_consumers_preserve_default_and_finite_zero_field_policy() {
    let default = world("").replace(
        "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
        "",
    );
    let mut absent = load(&default, 52);
    let counts = absent.scheduled_system_counts();
    set_magnetometer_interest(&mut absent, []);
    absent.step_exact(3).unwrap();
    absent.reset().unwrap();
    assert_eq!(absent.scheduled_system_counts(), counts);
    for count in [2, 8] {
        let sensors = (0..count)
            .map(|i| sensor_xml(i, 100, ""))
            .collect::<String>();
        let zero = world(&sensors).replace(
            "<magnetic_field>0.00002 -0.00003 0.00004</magnetic_field>",
            "<magnetic_field>0 0 0</magnetic_field>",
        );
        let mut sim = load(&zero, 52);
        sim.step_exact(1).unwrap();
        for index in 0..count {
            assert_eq!(sample(&sim, index).2, [0.0; 3]);
            geometry(&sim, index, [0.0; 3], [0.0; 3]);
        }
    }
}
