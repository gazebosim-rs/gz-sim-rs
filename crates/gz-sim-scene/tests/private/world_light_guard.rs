// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn exact_world() -> sdformat::World {
    let xml = "<sdf version='1.12'><world name='default'><model name='robot'><link name='body'><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><sensor name='imu' type='imu'><imu/></sensor></link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model></world></sdf>";
    sdformat::load_str(xml, "guard", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
}

pub(super) fn assert_world_light_metadata(value: &sdformat::World) {
    // This is the exact validation-only DTO mutation; its XML has no light.
    let mut expected = exact_world();
    expected.lights.push(sdformat::Light::default());
    let before = format!("{value:?}");
    assert_eq!(before, format!("{expected:?}"));
    assert_eq!(value.source.uri, "guard");
    assert_eq!(value.lights.as_slice(), &[sdformat::Light::default()]);
    validate(value, false, false).unwrap();
    assert_eq!(format!("{value:?}"), before);
}
