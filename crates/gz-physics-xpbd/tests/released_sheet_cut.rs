// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, XpbdError, step_coupled_damped_pads};
use std::time::Duration;

#[path = "fixtures/released_sheet_3345183.rs"]
mod cut;

fn material() -> SoftBody {
    let rest = cut::REST.map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 1. / 0.0007,
    });
    SoftBody::cloth_from_rest(
        cut::INCOMING.to_vec(),
        &rest,
        3,
        cut::CONFIG,
        |a, b| {
            if a / 3 != b / 3 && a % 3 != b % 3 {
                0.00002
            } else {
                0.00001
            }
        },
        |_| 0.02,
    )
    .unwrap()
    .with_bending_damping(0.001)
    .unwrap()
    .with_static_plane_friction(0.7)
    .unwrap()
    .with_pad_loads()
    .with_self_contact_friction(0.3)
    .unwrap()
    .with_self_contact_loads()
    .unwrap()
}

#[test]
fn independently_restarted_material_reproduces_the_native_step_at_release() {
    gz_physics_xpbd::CUT_ACTIVE.store(true, std::sync::atomic::Ordering::Relaxed);
    let mut b = material();
    let result = step_coupled_damped_pads(
        &mut [&mut b],
        Duration::from_nanos(1_000),
        &cut::SURFACES,
        &cut::MOBILITIES,
        &[(0, 4.0), (1, 4.0)],
    );
    assert!(matches!(result, Err(XpbdError::UnresolvedSurface)), "{result:?}");
    assert_eq!(b.particles(), cut::INCOMING);
}

#[test]
fn independent_triangle_oracle_checks_the_entire_committed_sheet() {
    use parry3d_f64::{
        math::{Pose, Vector},
        query::{distance, intersection_test},
        shape::Triangle,
    };
    let panels: Vec<_> = [0, 1, 3, 4, 6, 7]
        .into_iter()
        .flat_map(|i| [[i, i + 1, i + 4], [i, i + 4, i + 3]])
        .collect();
    let triangle = |indices: [usize; 3]| {
        let [a, b, c] = indices.map(|i| Vector::from_array(cut::INCOMING[i].position));
        Triangle::new(a, b, c)
    };
    let (mut pairs, mut nearest) = (0, f64::INFINITY);
    for (i, a) in panels.iter().enumerate() {
        for b in &panels[i + 1..] {
            if a.iter().any(|node| b.contains(node)) {
                continue;
            }
            let (ta, tb) = (triangle(*a), triangle(*b));
            let intersects = intersection_test(&Pose::IDENTITY, &ta, &Pose::IDENTITY, &tb).unwrap();
            assert!(!intersects.intersecting, "interlocked faces {a:?} and {b:?}");
            let gap = distance(&Pose::IDENTITY, &ta, &Pose::IDENTITY, &tb).unwrap();
            nearest = nearest.min(gap.distance);
            pairs += 1;
        }
    }
    assert!(pairs > 0 && nearest > 0.);
    println!("independent disjoint panel pairs={pairs}, smallest mid-surface distance={nearest:e} m");
}
