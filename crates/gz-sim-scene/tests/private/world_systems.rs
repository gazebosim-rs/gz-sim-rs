// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

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
