// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/support/fleet_observation.rs"]
mod observation;
#[test]
fn loaded_links_models_visuals_and_imu_share_the_committed_translation() {
    observation::run(4).unwrap();
}

#[test]
fn mixed_sdf_publishes_decks_payloads_contacts_and_imu() {
    observation::run_mixed(300, true).unwrap();
    observation::run_mixed(300, false).unwrap();
}
