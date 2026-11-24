// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../examples/rough_mobility_sdf.rs"]
mod scene;
use std::{error::Error, fs, path::Path};

#[test]
fn sdf_covariance_reports_actual_rough_contact_and_independent_momentum_in_each_lab()
-> Result<(), Box<dyn Error>> {
    let tmp = tempfile::tempdir()?;
    for ms in [1, 2] {
        let path = tmp.path().join(ms.to_string());
        let missing = scene::qualify(Path::new(env!("CARGO_BIN_EXE_physics_worker")), ms, &path)?;
        let gates: serde_json::Value =
            serde_json::from_slice(&fs::read(path.join("covariance-gates.json"))?)?;
        assert_eq!(
            gates["all_native_and_analytic_physical_checks_passed"],
            true
        );
        assert_eq!(
            gates["outside_declared_frame_covariance_limits"],
            serde_json::to_value(&missing)?
        );
        if ms == 1 {
            assert!(missing.is_empty());
        }
        for name in [
            "axis-rest",
            "axis-parallel",
            "axis-cross",
            "oblique-rest",
            "oblique-parallel",
            "oblique-cross",
        ] {
            let r: serde_json::Value =
                serde_json::from_slice(&fs::read(path.join(name).join("summary.json"))?)?;
            assert_eq!(r["physical"]["direct_default_exact_ticks"], 1200 / ms);
            assert_eq!(r["physical"]["direct_point_exact_ticks"], 1200 / ms);
            assert_eq!(r["physical"]["public_pose_twist_checks"], 2400 / ms);
            assert!(
                r["physical"]["active_receipts"]
                    .as_u64()
                    .is_some_and(|n| n > 200)
            );
            assert!(r["physical"]["first_rolling_tick"].is_number());
            if let Some(witness) =
                r["symmetry_vs_stationary_axis"]["first_outside_gate"].as_object()
            {
                assert!(missing.iter().any(|n| n == name));
                assert!(witness["tick"].as_u64().is_some_and(|n| n > 0));
            } else {
                assert!(!missing.iter().any(|n| n == name));
            }
        }
    }
    Ok(())
}
