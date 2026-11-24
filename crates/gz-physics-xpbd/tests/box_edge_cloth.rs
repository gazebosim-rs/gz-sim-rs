// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Boundary, Particle, SoftBody, SolverConfig};
use std::time::Duration;
const R: f64 = 0.006;
const NORMAL: DVec3 = DVec3::new(0.6, 0., 0.8);
const CROSS: DVec3 = DVec3::new(0.8, 0., -0.6);
const RIDGE: DVec3 = DVec3::new(0.025, 0., 0.02);
fn make(rotation: DQuat, y: f64, d: f64, velocity: DVec3, mu: f64) -> SoftBody {
    let axes = [CROSS * 0.12, NORMAL * 0.09];
    let points = (0..4)
        .map(|i| Particle {
            position: (rotation
                * (RIDGE
                    + NORMAL * d
                    + y * DVec3::Y
                    + axes[0] * (2. * (i % 2) as f64 - 1.)
                    + axes[1] * (i / 2) as f64))
                .to_array(),
            velocity: (rotation * velocity).to_array(),
            inverse_mass: 40.,
        })
        .collect();
    SoftBody::cloth(
        points,
        2,
        0.,
        0.,
        0.,
        SolverConfig {
            gravity: [0.; 3],
            substeps: 1,
            iterations: 12,
            particle_radius: R,
            friction: mu,
            self_contact: false,
        },
    )
    .unwrap()
    .with_angular_momentum_conservation()
}
fn bench(q: DQuat) -> Boundary {
    Boundary::StaticCuboid {
        center: [0.; 3],
        half_extents: [0.025, 0.16, 0.02],
        orientation: q.to_array(),
    }
}
fn totals(s: &SoftBody) -> [DVec3; 2] {
    let mut p = DVec3::ZERO;
    let mut l = p;
    for v in s.particles() {
        let mv = DVec3::from(v.velocity) / v.inverse_mass;
        p += mv;
        l += DVec3::from(v.position).cross(mv);
    }
    [p, l]
}
#[test]
fn grazing_hit_conserves_momentum_and_the_fifty_micron_near_miss_stays_clear() {
    let vy = 2.;
    let vn = 0.35;
    let d0 = 0.02;
    let gap_at_end = (R + 50e-6) * f64::hypot(vy, vn) / vy;
    let y_miss = 0.16 - vy * (d0 - gap_at_end) / vn;
    let v = vy * DVec3::Y - vn * NORMAL;
    for q in [
        DQuat::IDENTITY,
        DQuat::from_rotation_x(0.5) * DQuat::from_rotation_y(-0.4),
    ] {
        for (occupied, y) in [(true, 0.), (false, y_miss)] {
            let mut cloth = make(q, y, d0, v, 0.25);
            let mut first = None;
            let mut total = 0.;
            for tick in 1..=90 {
                let old = totals(&cloth);
                cloth.step(Duration::from_millis(1), &[bench(q)]).unwrap();
                let j = &cloth.last_boundary_impulses()[0];
                for (i, v) in [j.linear, j.angular].into_iter().enumerate() {
                    assert!((totals(&cloth)[i] - old[i] + DVec3::from(v)).length() < 5e-10);
                }
                let impulse = q.conjugate() * DVec3::from(j.linear);
                let normal = -impulse.dot(NORMAL);
                if impulse.length() > 1e-10 && first.is_none() {
                    first = Some(tick);
                    println!("first hit={tick}, impulse={impulse:?}, normal={normal}");
                    assert!(
                        normal > 0. && (impulse + normal * NORMAL).length() <= 0.25 * normal + 1e-9
                    );
                }
                total += impulse.length();
            }
            if occupied {
                assert_eq!(first, Some(40));
                assert!(total > 0.017);
            } else {
                assert_eq!(first, None);
                assert_eq!(total, 0.);
            }
        }
    }
}
#[test]
fn swept_edge_stops_a_crossing_whose_initial_and_final_node_spheres_clear_the_box() {
    let mut cloth = make(DQuat::IDENTITY, 0., 0.016, 2. * DVec3::Y - 40. * NORMAL, 0.);
    // Both hypothetical endpoint distances from the leading segment to the ridge
    // exceed the 6 mm skin while the complete crossing lies within this one tick.
    assert!((0.016_f64 - 40. * 0.001).abs() > R);
    cloth
        .step(Duration::from_millis(1), &[bench(DQuat::IDENTITY)])
        .unwrap();
    let j = DVec3::from(cloth.last_boundary_impulses()[0].linear);
    let n = -j.dot(NORMAL);
    println!("fast J={j:?}");
    assert!(n > 1.);
    assert!((j + n * NORMAL).length() < 1e-7);
}

#[test]
fn a_long_ridge_carries_a_flat_sheet_without_a_node_or_box_corner_on_the_sheet() {
    let p = (0..4)
        .map(|i| Particle {
            position: [
                (i % 2) as f64 * 0.4 - 0.2,
                (i / 2) as f64 * 0.4 - 0.2,
                0.027,
            ],
            velocity: [0., 0., 0.],
            inverse_mass: 40.,
        })
        .collect();
    let mut s = SoftBody::cloth(
        p,
        2,
        0.,
        0.,
        0.,
        SolverConfig {
            gravity: [0., 0., -9.81],
            substeps: 4,
            iterations: 16,
            particle_radius: R,
            friction: 0.3,
            self_contact: false,
        },
    )
    .unwrap();
    let b = Boundary::StaticCuboid {
        center: [0.; 3],
        half_extents: [0.025, 0.5, 0.02],
        orientation: DQuat::IDENTITY.to_array(),
    };
    let mut support = 0.;
    for tick in 1..=180 {
        s.step(Duration::from_millis(1), &[b]).unwrap();
        if tick > 140 {
            support -= s.last_boundary_impulses()[0].linear[2] / 0.040;
        }
    }
    assert!((support - 0.981).abs() < 0.02, "{support}");
    assert!(
        s.particles()
            .iter()
            .all(|p| p.position[0].abs() > 0.15 && (p.position[2] - 0.026).abs() < 0.003)
    );
}
