// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::snapshot::{encode, extract_with_assets};
use super::*;

fn visual(name: &str, radius: f64, length: f64, color: &str) -> String {
    format!(
        "<visual name='{name}'><geometry><capsule><radius>{radius}</radius><length>{length}</length></capsule></geometry><material><diffuse>{color}</diffuse></material></visual>"
    )
}
fn scene(visuals: &str) -> Simulation {
    let sdf = format!(
        "<sdf version='1.12'><world name='default'><model name='hand'><static>true</static><link name='link'><collision name='base'><geometry><box><size>1 1 1</size></box></geometry></collision>{visuals}</link></model></world></sdf>"
    );
    gz_sim_scene::load_world(
        &sdf,
        "file:///hand.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
    )
    .unwrap()
}

#[test]
fn capsule_instances_preserve_si_dimensions_and_colors_without_stretching_the_cap() {
    let amber = "0.75 0.5 0.25 1";
    let sim = scene(&[
        visual("one",0.03125,0.5,amber), visual("twin",0.03125,0.5,amber),
        visual("long",0.03125,1.,amber), visual("thick",0.0625,0.5,amber),
        visual("blue",0.03125,0.5,"0.25 0.375 0.75 1"),
        format!("<visual name='shaft'><geometry><cylinder><radius>0.03125</radius><length>0.5</length></cylinder></geometry><material><diffuse>{amber}</diffuse></material></visual>"),
    ].concat());
    assert!(extract_with_assets(&sim, "default", 0, None).is_err());
    let assets = Assets::new(&sim).unwrap().unwrap();
    let snapshot = extract_with_assets(&sim, "default", 0, Some(&assets)).unwrap();
    let captured = assets.snapshot();
    assert_eq!(captured.files().len(), 5);
    let scene: serde_json::Value = serde_json::from_str(&encode(&snapshot).unwrap()).unwrap();
    let mut routes = BTreeMap::new();
    for v in scene["visuals"].as_array().unwrap() {
        let name = v["name"].as_str().unwrap().rsplit("::").next().unwrap();
        let uri = v["shape"]["uri"].as_str().unwrap();
        let route = uri.strip_prefix(captured.prefix()).unwrap();
        validate_recorded_document(uri, captured.prefix(), captured.files()).unwrap();
        let (r, h, color) = match name {
            "shaft" => {
                assert!(route.ends_with("/cylinder.glb"));
                continue;
            }
            "thick" => (0.0625, 0.5, [0.75, 0.5, 0.25, 1.]),
            "long" => (0.03125, 1., [0.75, 0.5, 0.25, 1.]),
            "blue" => (0.03125, 0.5, [0.25, 0.375, 0.75, 1.]),
            _ => (0.03125, 0.5, [0.75, 0.5, 0.25, 1.]),
        };
        assert!(route.ends_with("/capsule.glb"));
        assert_eq!(assets.capsule_uri(r, h, color).unwrap(), uri);
        assert_eq!(v["shape"]["scale"], serde_json::json!([1., 1., 1.]));
        assert_eq!(v["color"], serde_json::json!([1., 1., 1., 1.]));
        assert_eq!(
            captured.files()[route].as_ref(),
            gz_assets::mesh::capsule_visual_glb(r, h, color).unwrap()
        );
        routes.insert(name, route);
    }
    assert_eq!(routes["one"], routes["twin"]);
    assert_ne!(routes["one"], routes["long"]);
    assert_ne!(routes["one"], routes["thick"]);
    assert_ne!(routes["one"], routes["blue"]);
    assert!(assets.capsule_uri(0.03125, 0.125, [1.; 4]).is_err());
}
#[test]
fn capsule_resource_limits_and_invalid_geometry_are_enforced_before_serving() {
    let palette = "0.2 0.3 0.4 1";
    let too_many: String = (0..=FILE_COUNT)
        .map(|i| visual(&format!("p{i}"), 0.01, 0.1 + 0.001 * i as f64, palette))
        .collect();
    assert!(
        Assets::new(&scene(&too_many))
            .err()
            .unwrap()
            .to_string()
            .contains("128")
    );
    assert!(Assets::new(&scene(&visual("invalid", 1e-20, 1., palette))).is_err());
    assert!(Assets::new(&scene(&visual("glass", 0.02, 0.1, "1 1 1 0.4"))).is_err());
}

#[test]
fn snapshot_rejects_a_capsule_that_changed_after_resource_capture() {
    let mut sim = scene(&visual("tip", 0.02, 0.10, "0.3 0.2 0.1 1"));
    let admitted = Assets::new(&sim).unwrap().unwrap();
    let visual = sim
        .world()
        .iter_entities()
        .find(|e| e.contains::<VisualGeometry>())
        .unwrap()
        .id();
    sim.world_mut()
        .entity_mut(visual)
        .insert(VisualGeometry(Arc::new(sdformat::Geometry::Capsule {
            radius: 0.03,
            length: 0.10,
        })));
    let error = extract_with_assets(&sim, "default", 0, Some(&admitted))
        .err()
        .unwrap();
    assert!(
        error
            .to_string()
            .contains("outside the admitted asset snapshot")
    );
    assert_eq!(admitted.snapshot().files().len(), 1);
    let updated = Assets::new(&sim).unwrap().unwrap();
    assert!(extract_with_assets(&sim, "default", 0, Some(&updated)).is_ok());
}
