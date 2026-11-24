// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn trial(args: &[&str]) -> Result<Vec<Sample>> {
    simulate(Args::try_parse_from(
        ["bilateral"].into_iter().chain(args.iter().copied()),
    )?)
}
#[test]
fn real_load_and_moments_close_with_both_fingers_and_during_free_fall() -> Result<()> {
    let centered = trial(&["--repeat", "--substeps", "64"])?;
    let biased = trial(&["--biased", "--substeps", "64"])?;
    assert_eq!(biased[..60], centered[..60]);
    for row in [&centered, &biased] {
        let m = summary::report(row);
        assert_eq!(
            (
                m.first_grip_ms,
                m.first_clear_ms,
                m.first_regrip_ms,
                m.first_tray_ms
            ),
            (Some(1), Some(141), Some(178), None)
        );
        assert!(m.peak_linear_residual_ns < 1e-10 && m.peak_angular_residual_nms < 1e-7);
        for t in &row[140..164] {
            assert_eq!(t.contacts, Contacts::default());
        }
        let dv = row[163].vel_m_s[2] - row[139].vel_m_s[2];
        assert!((dv + 9.81 * 0.024).abs() < 1e-10);
        for h in [&m.loaded_101_to_140_ms, &m.loaded_246_to_280_ms] {
            assert!((h.jaw_force_n[0][2] + h.jaw_force_n[1][2] - 3.962).abs() < 0.002);
        }
    }
    let symmetric = summary::report(&centered).loaded_101_to_140_ms;
    assert!((symmetric.jaw_normal_n[0] - symmetric.jaw_normal_n[1]).abs() < 0.01);
    let load = summary::report(&biased).loaded_101_to_140_ms;
    assert!(load.jaw_normal_n[1] - load.jaw_normal_n[0] > 2.);
    assert!(load.jaw_force_n[1][2] - load.jaw_force_n[0][2] > 0.7);
    assert!((load.jaw_torque_nm[0][1] + load.jaw_torque_nm[1][1] + 0.025).abs() < 0.001);
    Ok(())
}
#[test]
fn held_open_control_drops_the_same_incoming_parcel_onto_the_stand() -> Result<()> {
    let grip = trial(&[])?;
    let open = trial(&["--stay-open", "--repeat"])?;
    assert_eq!(&grip[..164], &open[..164]);
    let m = summary::report(&open);
    assert!(m.first_regrip_ms.is_none() && m.first_tray_ms.is_some());
    assert!(
        open[245..]
            .iter()
            .all(|s| s.contacts.left.normal_ns == 0. && s.contacts.right.normal_ns == 0.)
    );
    Ok(())
}
#[test]
fn both_normals_and_vertical_loads_agree_with_a_finer_clock_on_round_tips() -> Result<()> {
    let probe = summary::report(&trial(&["--biased", "--substeps", "64"])?);
    let fine = summary::report(&trial(&["--biased", "--substeps", "256"])?);
    for (a, b) in [
        (&probe.loaded_101_to_140_ms, &fine.loaded_101_to_140_ms),
        (&probe.loaded_246_to_280_ms, &fine.loaded_246_to_280_ms),
    ] {
        for jaw in 0..2 {
            assert!((a.jaw_normal_n[jaw] - b.jaw_normal_n[jaw]).abs() < 0.045);
            assert!((a.jaw_force_n[jaw][2] - b.jaw_force_n[jaw][2]).abs() < 0.025);
        }
    }
    Ok(())
}
