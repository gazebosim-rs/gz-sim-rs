// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Route an approaching cold box to a worker that is already solving another contact.
#[path = "support/handoff_options.rs"]
mod options;
#[path = "support/routed_contacts.rs"]
mod qualification;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    options::execute(qualification::run)
}
