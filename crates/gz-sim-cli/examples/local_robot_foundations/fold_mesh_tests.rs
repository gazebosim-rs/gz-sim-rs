// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[test]
fn width_transfer_preserves_mass_and_momentum_of_a_nonuniform_velocity_field() {
    let p: Vec<_> = (0..12)
        .map(|i| Particle {
            position: arm_sheet::seed(i, 0.).to_array(),
            velocity: [(i * i) as f64, i as f64, -0.2],
            inverse_mass: 10.,
        })
        .collect();
    let fine = resample(&p, 5);
    let a = moments(&p);
    let b = moments(&fine);
    assert!((a.0 - b.0).abs() < 1e-14 && a.1.distance(b.1) < 1e-14 && a.2.distance(b.2) < 1e-13);
    for i in 0..12 {
        let j = (i / 3) * 5 + 2 * (i % 3);
        assert_eq!(p[i].position, fine[j].position);
        assert_eq!(p[i].velocity, fine[j].velocity);
    }
    assert_eq!(edge_law(0, 1, 5) * 2., STRETCH);
    assert_eq!(hinge_law([0, 1, 5, 6], 5), 2. * BEND);
}

#[test]
fn finite_panel_gap_and_flat_face_calibration_are_independent_of_node_count() -> Result<()> {
    let loaded: Vec<_> = (0..12)
        .map(|i| Particle {
            position: arm_sheet::seed(i, 0.).to_array(),
            velocity: [0.; 3],
            inverse_mass: 1. / 0.0007,
        })
        .collect();
    let anchor = Anchor {
        loaded,
        half: [DVec3::new(0.012, 0.0425, 0.07); 2],
    };
    let jaw = |x| BodyState {
        position: [x, 0., BENCH + 0.03],
        orientation: DQuat::IDENTITY.to_array(),
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let jaws = [
        jaw(0.32 - 2. * RADIUS - 0.012),
        jaw(0.32 + 2. * RADIUS + 0.012),
    ];
    let poses = [jaws, jaws];
    for (columns, count) in [(3, 2), (5, 6), (7, 10)] {
        let points = resample(&anchor.loaded, columns);
        assert!((panel_gap(&points, columns)? - 0.00002).abs() < 1e-12);
        let test = trial(&anchor, columns, &poses)?;
        assert_eq!(test.face_nodes, [count; 2]);
        // Same force for a given flat indentation at both jaws.
        assert!((count as f64 / test.pad_compliance_m_per_n[0] - 2. / PAD_COMPLIANCE).abs() < 1e-9);
        assert_eq!(test.completed_ticks, 1);
        assert!(test.mass_error_kg < 1e-15 && test.linear_momentum_error_ns < 1e-15);
        assert!(
            test.max_matched_node_position_error_m == 0.
                && test.max_matched_node_velocity_error_m_s == 0.
        );
        let sums = test.loads.unwrap();
        assert!(
            (sums.cross_panel_width_n.iter().sum::<f64>() - sums.mean_cross_panel_n).abs() < 1e-11
        );
    }
    let mut distant = jaws;
    distant[1].position[2] += 1.;
    assert!(trial(&anchor, 3, &[distant, distant]).is_err());
    let mut buried = jaws;
    buried[0].position[0] = arm_sheet::seed(1, 0.).x;
    let refused = trial(&anchor, 3, &[buried, buried])?;
    assert_eq!(refused.status, "refused");
    assert_eq!(refused.completed_ticks, 0);
    assert!(refused.min_held_panel_skin_gap_m.is_none() && refused.loads.is_none());
    let dir = tempfile::tempdir()?;
    assert!(run(anchor, &[], dir.path()).is_err());
    Ok(())
}
