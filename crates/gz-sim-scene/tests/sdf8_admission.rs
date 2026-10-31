// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{LoadError, Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
use sdformat::{NoResources, Resource, ResourceResolver};

fn world(
    extra: &str,
    model_attributes: &str,
    model_extra: &str,
    link_extra: &str,
    collision_extra: &str,
    sensor: &str,
) -> String {
    format!(
        "<sdf version='1.12'><world name='default'>{extra}<model name='robot' {model_attributes}><static>true</static>{model_extra}<link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry>{collision_extra}</collision>{link_extra}{sensor}</link></model></world></sdf>"
    )
}
fn rejected(xml: &str, feature: &'static str) {
    let result = load_world_with_sdf_step(xml, "sdf8", "default", &NoResources, PhysicsEngine::Tpe);
    assert!(
        matches!(result, Err(LoadError::Unsupported { feature: actual, .. }) if actual == feature)
    );
}
#[test]
fn newly_parsed_runtime_effects_are_rejected_before_scene_creation() {
    for (extra, expected) in [
        ("<light name='lamp' type='point'/>", "world lights"),
        (
            "<wind><linear_velocity>1 0 0</linear_velocity></wind>",
            "world wind",
        ),
        (
            "<magnetic_field>0 0 0</magnetic_field>",
            "world magnetic field",
        ),
        ("<atmosphere type='adiabatic'/>", "world atmosphere"),
        (
            "<vendor_extension>payload</vendor_extension>",
            "world extensions",
        ),
    ] {
        rejected(&world(extra, "", "", "", "", ""), expected);
    }
    rejected(
        &world("", "namespace='robot'", "", "", "", ""),
        "model namespace",
    );
    rejected(
        &world("", "", "<enable_wind>true</enable_wind>", "", "", ""),
        "model wind",
    );
    rejected(
        &world("", "", "<vendor_extension/>", "", "", ""),
        "model extensions",
    );
    rejected(
        &world("", "", "", "<light name='lamp' type='point'/>", "", ""),
        "link lights",
    );
    rejected(
        &world("", "", "", "", "<density>1000</density>", ""),
        "collision density",
    );
    rejected(
        &world(
            "",
            "",
            "",
            "",
            "",
            "<sensor name='imu' type='imu'><imu/><visualize>true</visualize></sensor>",
        ),
        "sensor visualization",
    );
    rejected(
        &world(
            "",
            "",
            "",
            "",
            "",
            "<sensor name='imu' type='imu'><imu/><plugin name='p' filename='p.so'/></sensor>",
        ),
        "sensor plugins",
    );
    rejected(
        &world(
            "",
            "",
            "",
            "",
            "",
            "<sensor name='camera' type='camera'><camera><image/><clip/></camera></sensor>",
        ),
        "non-IMU sensors",
    );
}
#[test]
fn native_default_world_values_remain_admitted_and_resettable() {
    let xml = world(
        "<wind><linear_velocity>0 0 0</linear_velocity></wind><magnetic_field>0.0000055645 0.0000228758 -0.0000423884</magnetic_field>",
        "",
        "<enable_wind>false</enable_wind>",
        "",
        "",
        "",
    );
    let inert_visualization = world(
        "",
        "",
        "",
        "",
        "",
        "<sensor name='imu' type='imu'><visualize>false</visualize><imu/></sensor>",
    );
    assert!(
        load_world_with_sdf_step(
            &inert_visualization,
            "inert-visualization",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        )
        .is_ok()
    );
    let empty_namespace = world("", "namespace=''", "", "", "", "");
    assert!(
        load_world_with_sdf_step(
            &empty_namespace,
            "empty-namespace",
            "default",
            &NoResources,
            PhysicsEngine::Tpe
        )
        .is_ok()
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "defaults",
        "default",
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    sim.step_exact(2).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
}
struct MergedResource {
    link_extra: &'static str,
}
impl ResourceResolver for MergedResource {
    fn resolve(&self, uri: &str, _: &str) -> Result<Resource, String> {
        assert_eq!(uri, "model://part");
        Ok(Resource {
            canonical_uri: "file:///part.sdf".into(),
            xml: format!(
                "<sdf version='1.12'><model name='part' namespace='ignored'><enable_wind>true</enable_wind><link name='imported'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>{}</link></model></sdf>",
                self.link_extra
            ),
        })
    }
}
#[test]
fn merged_children_are_checked_without_rejecting_provenance() {
    let xml = "<sdf version='1.12'><world name='default'><model name='robot'><static>true</static><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link><include merge='true'><uri>model://part</uri></include></model></world></sdf>";
    let mut sim = load_world_with_sdf_step(
        xml,
        "merged",
        "default",
        &MergedResource { link_extra: "" },
        PhysicsEngine::Rapier,
    )
    .unwrap();
    assert!(sim.world().iter_entities().any(|e| {
        e.get::<Name>()
            .is_some_and(|n| n.0 == "default::robot::imported")
    }));
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let result = load_world_with_sdf_step(
        xml,
        "merged",
        "default",
        &MergedResource {
            link_extra: "<light name='lamp' type='point'/>",
        },
        PhysicsEngine::Rapier,
    );
    assert!(matches!(
        result,
        Err(LoadError::Unsupported {
            feature: "link lights",
            ..
        })
    ));
    let result = load_world_with_sdf_step(
        xml,
        "merged",
        "default",
        &MergedResource {
            link_extra: "<enable_wind>true</enable_wind>",
        },
        PhysicsEngine::Rapier,
    );
    assert!(matches!(
        result,
        Err(LoadError::Unsupported {
            feature: "link wind",
            ..
        })
    ));
}

#[test]
fn newly_parsed_world_and_joint_constraints_fail_before_runtime() {
    let empty_collision = world("", "", "", "", "", "").replace(
        "<geometry><sphere><radius>0.5</radius></sphere></geometry>",
        "",
    );
    rejected(&empty_collision, "collision geometry");
    let world_joint = "<joint name='joint' type='revolute'><parent>world</parent><child>robot::body</child><axis><xyz>0 0 1</xyz></axis></joint>";
    rejected(&world(world_joint, "", "", "", "", ""), "world joints");
    let link = |name| {
        format!(
            "<link name='{name}'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link>"
        )
    };
    let base = format!("{}{}", link("a"), link("b"));
    let leader = "<joint name='leader' type='revolute'><parent>world</parent><child>a</child><axis><xyz>0 0 1</xyz></axis></joint>";
    let follower = "<joint name='follower' type='revolute'><parent>world</parent><child>b</child><axis><xyz>0 0 1</xyz><mimic joint='leader' axis='axis'/></axis></joint>";
    let xml = format!(
        "<sdf version='1.12'><world name='default'><model name='robot'>{base}{leader}{follower}</model></world></sdf>"
    );
    rejected(&xml, "joint mimic");
    let ball = "<joint name='hinge' type='ball'><parent>world</parent><child>a</child></joint>";
    let xml = format!(
        "<sdf version='1.12'><world name='default'><model name='robot'>{base}{ball}</model></world></sdf>"
    );
    rejected(&xml, "joints (unsupported type)");
    let physics = "<joint name='hinge' type='revolute'><parent>world</parent><child>a</child><axis><xyz>0 0 1</xyz></axis><physics><ode/></physics></joint>";
    let xml = format!(
        "<sdf version='1.12'><world name='default'><model name='robot'>{base}{physics}</model></world></sdf>"
    );
    rejected(&xml, "joint physics metadata");
}

#[test]
fn producer_camera_lens_presence_survives_default_valued_explicit_fields() {
    fn camera(lens: &str) -> sdformat::Camera {
        let sensor = format!(
            "<sensor name='camera' type='camera'><camera><image><width>64</width><height>64</height><format>R8G8B8</format></image><clip><near>0.1</near><far>10</far></clip>{lens}</camera></sensor>"
        );
        let source = world("", "", "", "", "", &sensor);
        let mut sdf = sdformat::load_str(&source, "camera-presence", &NoResources).unwrap();
        let sensor = sdf.worlds[0].models[0].links[0].sensors.remove(0);
        let sdformat::SensorConfig::Camera(camera) = sensor.config else {
            panic!("camera config")
        };
        *camera
    }
    let omitted = camera("");
    assert!(!omitted.lens.has_intrinsics);
    assert!(!omitted.lens.has_projection);
    let explicit = camera(
        "<lens><intrinsics><fx>277</fx><fy>277</fy><cx>160</cx><cy>120</cy><s>0</s></intrinsics><projection><p_fx>277</p_fx><p_fy>277</p_fy><p_cx>160</p_cx><p_cy>120</p_cy><tx>0</tx><ty>0</ty></projection></lens>",
    );
    assert!(explicit.lens.has_intrinsics);
    assert!(explicit.lens.has_projection);
    assert_eq!(omitted.lens.intrinsics, explicit.lens.intrinsics);
    assert_eq!(omitted.lens.projection, explicit.lens.projection);
}

#[test]
fn revolute_pitch_is_preserved_as_metadata_across_step_and_reset() {
    use gz_sim_scene::JointDefinition;
    let xml = include_str!("fixtures/pendulum.sdf").replace(
        "</joint>",
        "<screw_thread_pitch>0.4</screw_thread_pitch></joint>",
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "joint-pitch",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let value = |sim: &gz_sim_core::Simulation| {
        let joint = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::robot::hinge")
            })
            .unwrap();
        let definition = joint.get::<JointDefinition>().unwrap();
        assert_eq!(definition.0.kind, sdformat::JointKind::Revolute);
        assert_eq!(definition.0.screw_thread_pitch, 0.4);
        assert!((definition.0.legacy_thread_pitch() - (-15.707963267948966)).abs() < 1e-14);
    };
    value(&sim);
    sim.step_exact(2).unwrap();
    value(&sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    value(&sim);
}
