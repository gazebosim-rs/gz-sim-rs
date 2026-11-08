// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn magnetometer_system_metadata_is_closed_unique_and_inert_without_a_sensor() {
    let plugin =
        "<plugin name='gz::sim::systems::Magnetometer' filename='gz-sim-magnetometer-system'/>";
    let plain = "<sdf version='1.12'><world name='w'/></sdf>";
    let xml = format!("<sdf version='1.12'><world name='w'>{plugin}</world></sdf>");
    let parsed = sdformat::load_str(&xml, "magnetic-intent.sdf", &sdformat::NoResources).unwrap();
    assert!(!validate(&parsed.worlds[0]).unwrap());
    let mut sim = load_world_with_sdf_step(
        &xml,
        "magnetic-intent.sdf",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let baseline = load_world_with_sdf_step(
        plain,
        "plain.sdf",
        "w",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    assert!(
        !sim.world()
            .contains_resource::<crate::magnetometer::Installed>()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    for plugins in [
        format!("{plugin}{plugin}"),
        plugin.replace("/>", "><use_units_gauss>false</use_units_gauss></plugin>"),
        plugin.replace("plugin ", "plugin extra='x' "),
        plugin.replace("plugin ", "plugin xmlns:e='urn:e' e:flag='x' "),
        plugin.replace(
            "gz-sim-magnetometer-system",
            "libgz-sim-magnetometer-system.so",
        ),
        plugin.replace("gz::sim::systems::Magnetometer", "Magnetometer"),
    ] {
        let xml = format!("<sdf version='1.12'><world name='w'>{plugins}</world></sdf>");
        let parsed =
            sdformat::load_str(&xml, "bad-magnetic-intent.sdf", &sdformat::NoResources).unwrap();
        assert!(validate(&parsed.worlds[0]).is_err(), "{plugins}");
    }
}

#[test]
fn altimeter_metadata_is_exact_empty_unique_and_installs_no_absent_sensor_work() {
    let plugin = "<plugin name='gz::sim::systems::Altimeter' filename='gz-sim-altimeter-system'/>";
    let plain = "<sdf version='1.12'><world name='w'/></sdf>";
    let xml = format!("<sdf version='1.12'><world name='w'>{plugin}</world></sdf>");
    let sdf = sdformat::load_str(&xml, "altimeter-system.sdf", &sdformat::NoResources).unwrap();
    assert!(!validate(&sdf.worlds[0]).unwrap());
    let mut sim = load_world(
        &xml,
        "altimeter-system.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let baseline = load_world(
        plain,
        "plain.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    assert!(
        !sim.world()
            .contains_resource::<crate::altimeter::Installed>()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    for plugins in [
        format!("{plugin}{plugin}"),
        plugin.replace("/>", "><topic>other</topic></plugin>"),
        plugin.replace("plugin ", "plugin extra='x' "),
        plugin.replace("gz-sim-altimeter-system", "libgz-sim-altimeter-system.so"),
        plugin.replace("gz::sim::systems::Altimeter", "Altimeter"),
    ] {
        let xml = format!("<sdf version='1.12'><world name='w'>{plugins}</world></sdf>");
        let sdf = sdformat::load_str(&xml, "invalid-altimeter-system.sdf", &sdformat::NoResources)
            .unwrap();
        assert!(validate(&sdf.worlds[0]).is_err(), "{plugins}");
    }
    let model = format!(
        "<sdf version='1.12'><world name='w'><model name='m'>{plugin}<link name='l'/></model></world></sdf>"
    );
    assert!(
        load_world(
            &model,
            "model-altimeter.sdf",
            "w",
            std::time::Duration::from_millis(1),
            &sdformat::NoResources
        )
        .is_err()
    );
}

#[test]
fn air_pressure_metadata_is_exact_empty_unique_and_installs_no_absent_sensor_work() {
    let plugin =
        "<plugin name='gz::sim::systems::AirPressure' filename='gz-sim-air-pressure-system'/>";
    let plain = "<sdf version='1.12'><world name='w'/></sdf>";
    let xml = format!("<sdf version='1.12'><world name='w'>{plugin}</world></sdf>");
    let sdf = sdformat::load_str(&xml, "air_pressure-system.sdf", &sdformat::NoResources).unwrap();
    assert!(!validate(&sdf.worlds[0]).unwrap());
    let mut sim = load_world(
        &xml,
        "air_pressure-system.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let baseline = load_world(
        plain,
        "plain.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    assert!(
        !sim.world()
            .contains_resource::<crate::air_pressure::Installed>()
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    for plugins in [
        format!("{plugin}{plugin}"),
        plugin.replace("/>", "><topic>other</topic></plugin>"),
        plugin.replace("plugin ", "plugin extra='x' "),
        plugin.replace(
            "gz-sim-air-pressure-system",
            "libgz-sim-air-pressure-system.so",
        ),
        plugin.replace("gz::sim::systems::AirPressure", "AirPressure"),
    ] {
        let xml = format!("<sdf version='1.12'><world name='w'>{plugins}</world></sdf>");
        let sdf = sdformat::load_str(
            &xml,
            "invalid-air_pressure-system.sdf",
            &sdformat::NoResources,
        )
        .unwrap();
        assert!(validate(&sdf.worlds[0]).is_err(), "{plugins}");
    }
    let model = format!(
        "<sdf version='1.12'><world name='w'><model name='m'>{plugin}<link name='l'/></model></world></sdf>"
    );
    assert!(
        load_world(
            &model,
            "model-air_pressure.sdf",
            "w",
            std::time::Duration::from_millis(1),
            &sdformat::NoResources
        )
        .is_err()
    );
}

#[test]
fn raw_record_must_match_the_owned_metadata() {
    let record = "<plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>";
    let xml = format!("<sdf version='1.12'><world name='w'>{record}</world></sdf>");
    let mut sdf = sdformat::load_str(&xml, "raw-world.sdf", &sdformat::NoResources).unwrap();
    for raw in [
        "<".into(),
        record.replace("plugin ", "other "),
        record.replace("name='gz::sim::systems::Physics'", "name='other'"),
        record.replace("filename='gz-sim-physics-system'", "filename='other'"),
        record.replace("name='gz::sim::systems::Physics'", ""),
        record.replace("filename='gz-sim-physics-system'", ""),
        record.replace("<plugin ", "<plugin xmlns:e='urn:e' e:flag='x' "),
        record.replace("/>", "><?instruction value?></plugin>"),
    ] {
        sdf.worlds[0].plugins[0].xml = raw.clone();
        assert!(validate(&sdf.worlds[0]).is_err(), "{raw}");
    }
    sdf.worlds[0].plugins[0].xml = record.replace(
        "name='gz::sim::systems::Physics'",
        "name=' gz::sim::systems::Physics '",
    );
    validate(&sdf.worlds[0]).unwrap();
}

#[test]
fn sensors_intent_is_world_only_bare_unique_and_can_share_physics() {
    let sensors = "<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>";
    let physics = "<plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>";
    for (plugins, expected) in [
        ("".to_owned(), false),
        (physics.to_owned(), false),
        (sensors.to_owned(), true),
        (format!("{physics}{sensors}"), true),
        (format!("{sensors}{physics}"), true),
    ] {
        let xml = format!("<sdf version='1.12'><world name='w'>{plugins}</world></sdf>");
        let sdf = sdformat::load_str(&xml, "systems.sdf", &sdformat::NoResources).unwrap();
        assert_eq!(validate(&sdf.worlds[0]).unwrap(), expected);
    }
    for plugins in [
        format!("{sensors}{sensors}"),
        format!("{physics}{physics}{sensors}"),
        sensors.replace("/>", "><render_engine>ogre2</render_engine></plugin>"),
        sensors.replace("/>", "><unknown/></plugin>"),
        sensors.replace("plugin ", "plugin flag='x' "),
        sensors.replace("plugin ", "plugin xmlns:e='urn:e' e:flag='x' "),
        sensors.replace("gz-sim-sensors-system", "other"),
    ] {
        let xml = format!("<sdf version='1.12'><world name='w'>{plugins}</world></sdf>");
        let sdf = sdformat::load_str(&xml, "systems.sdf", &sdformat::NoResources).unwrap();
        assert!(validate(&sdf.worlds[0]).is_err(), "{plugins}");
    }
    let xml = format!("<sdf version='1.12'><world name='w'>{sensors}</world></sdf>");
    let mut sdf = sdformat::load_str(&xml, "raw-sensors.sdf", &sdformat::NoResources).unwrap();
    // The loader rejects PI XML itself. Mutating the owned raw record exercises
    // the additional plugin-validation boundary, as in the existing raw test.
    sdf.worlds[0].plugins[0].xml = sensors.replace("/>", "><?instruction value?></plugin>");
    assert!(matches!(
        validate(&sdf.worlds[0]),
        Err(LoadError::Unsupported {
            feature: "Sensors system configuration",
            ..
        })
    ));
    let xml = format!(
        "<sdf version='1.12'><world name='w'><model name='m'>{sensors}<link name='l'/></model></world></sdf>"
    );
    assert!(matches!(
        load_world(
            &xml,
            "model-system.sdf",
            "w",
            std::time::Duration::from_millis(1),
            &sdformat::NoResources
        ),
        Err(LoadError::Unsupported { .. })
    ));
}

#[test]
fn sensors_configuration_metadata_survives_loaded_reset_without_installing_work() {
    let plain = "<sdf version='1.12'><world name='w'/></sdf>";
    let declared = plain.replace("name='w'/>", "name='w'><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/></world>");
    let mut sim = load_world(
        &declared,
        "intent.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    let baseline = load_world(
        plain,
        "plain.sdf",
        "w",
        std::time::Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap();
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
    let world = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<EntityKind>() == Some(&EntityKind::World))
        .unwrap()
        .id();
    assert!(
        sim.world()
            .get::<RenderingSensorsConfiguration>(world)
            .is_some()
    );
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<CameraSensor>())
    );
    sim.world_mut()
        .entity_mut(world)
        .remove::<RenderingSensorsConfiguration>();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(
        sim.world()
            .iter_entities()
            .any(|e| e.get::<EntityKind>() == Some(&EntityKind::World)
                && e.contains::<RenderingSensorsConfiguration>())
    );
    assert_eq!(
        sim.scheduled_system_counts(),
        baseline.scheduled_system_counts()
    );
}
