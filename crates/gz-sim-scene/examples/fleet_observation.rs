// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "support/fleet_loaded_rest.rs"]
mod loaded_rest;
#[path = "support/fleet_observation.rs"]
mod observation;
#[path = "support/fleet_traffic.rs"]
mod traffic;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let count = args
        .next()
        .ok_or("usage: fleet_observation NUMBER_OF_MODELS [mixed-auto|mixed-native|traffic-auto|traffic-native|traffic-verify|loaded-rest|loaded-rest-pulse]")?
        .parse()?;
    let mode = args.next();
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    match mode.as_deref() {
        None => observation::run(count),
        Some("mixed-auto") => observation::run_mixed(count, true),
        Some("loaded-rest" | "loaded-rest-pulse") => {
            println!(
                "{}",
                loaded_rest::run(count, mode.as_deref() == Some("loaded-rest-pulse"))?
            );
            Ok(())
        }
        Some("mixed-native") => observation::run_mixed(count, false),
        Some("traffic-auto") => traffic::measure(count, true),
        Some("traffic-native") => traffic::measure(count, false),
        Some("traffic-verify") => traffic::verify(count),
        _ => Err("unknown fleet benchmark mode".into()),
    }
}
