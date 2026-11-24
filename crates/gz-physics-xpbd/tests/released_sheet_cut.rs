// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, step_coupled_damped_pads};
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
fn independently_restarted_material_resolves_the_native_step_at_release() {
    use glam::{DVec3, Vec3Swizzles};
    use gz_physics_xpbd::Boundary;
    let dt = Duration::from_nanos(1_000);
    let (deck_origin, half) = match cut::SURFACES[2] {
        Boundary::TranslatingCuboid {
            center,
            half_extents,
            ..
        } => (DVec3::from(center), half_extents),
        _ => panic!("expected the measured receiving deck"),
    };
    let mut results = Vec::new();
    for resolution in [1, 2, 4, 8] {
        let mut b = material();
        b.set_resolution(resolution, 24).unwrap();
        let before = b.mechanical_energy().unwrap().total();
        step_coupled_damped_pads(
            &mut [&mut b],
            dt,
            &cut::SURFACES,
            &cut::MOBILITIES,
            &[(0, 4.), (1, 4.)],
        )
        .unwrap();
        let impulse = &b.last_boundary_impulses()[2];
        assert!(
            b.last_boundary_impulses()[..2]
                .iter()
                .all(|j| j.linear == [0.; 3] && j.angular == [0.; 3])
        );
        let table = -DVec3::from(impulse.linear);
        assert!(table.z > 0. && table.xy().length() <= 0.7 * table.z + 1e-13);
        let shell = b.last_self_contact_impulse();
        assert!(
            shell.normal > 0.
                && shell.tangent <= 0.3 * shell.normal + 1e-14
                && shell.friction_work >= 0.
        );
        let mut residual = DVec3::from(impulse.linear);
        let mut moment = DVec3::from(impulse.angular);
        for (old, new) in cut::INCOMING.iter().zip(b.particles()) {
            let gravity = DVec3::from(cut::CONFIG.gravity) * dt.as_secs_f64() / old.inverse_mass;
            let pi = DVec3::from(old.velocity) / old.inverse_mass;
            let pf = DVec3::from(new.velocity) / new.inverse_mass;
            residual += pf - pi - gravity;
            let arm = DVec3::from(old.position) - deck_origin;
            moment += (DVec3::from(new.position) - deck_origin).cross(pf) - arm.cross(pi + gravity);
            assert!(
                new.position[2] >= deck_origin.z + half[2] + cut::CONFIG.particle_radius - 2e-11
            );
        }
        let delta_energy = b.mechanical_energy().unwrap().total() - before;
        println!(
            "resolution={resolution} impulse={table:?} moment={:?} balance={} angular_balance={} dE={delta_energy:e}",
            impulse.angular,
            residual.length(),
            moment.length()
        );
        assert!(residual.length() < 1e-14 && moment.length() < 1e-10 && delta_energy <= 1e-12);
        results.push(b.particles().to_vec());
    }
    let fine = results.last().unwrap();
    for coarse in &results[..3] {
        for (p, f) in coarse.iter().zip(fine) {
            assert!(DVec3::from(p.position).distance(f.position.into()) < 1e-7);
        }
    }
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
            assert!(
                !intersects.intersecting,
                "interlocked faces {a:?} and {b:?}"
            );
            let gap = distance(&Pose::IDENTITY, &ta, &Pose::IDENTITY, &tb).unwrap();
            nearest = nearest.min(gap.distance);
            pairs += 1;
        }
    }
    assert!(pairs > 0 && nearest > 0.);
    println!(
        "independent disjoint panel pairs={pairs}, smallest mid-surface distance={nearest:e} m"
    );
}

#[test]
fn the_actual_edge_pair_stays_clear_to_geometry_tolerance_but_ends_closing() {
    use glam::DVec3;
    use parry3d_f64::{
        math::{Pose, Vector},
        query::details::distance_segment_segment,
        shape::Segment,
    };
    let from = cut::SWEEP_START.map(DVec3::from);
    let to = cut::SWEEP_END.map(DVec3::from);
    let velocity = cut::TERMINAL_VELOCITIES.map(DVec3::from);
    let distance = |p: [DVec3; 4]| {
        let [a, b, c, d] = p.map(|v| Vector::from_array(v.to_array()));
        distance_segment_segment(&Pose::IDENTITY, &Segment::new(a, b), &Segment::new(c, d))
    };
    let skin_diameter = 2. * cut::CONFIG.particle_radius;
    let displacement: [DVec3; 4] = std::array::from_fn(|i| to[i] - from[i]);
    let speed_bound = displacement[..2]
        .iter()
        .flat_map(|a| displacement[2..].iter().map(move |b| a.distance(*b)))
        .fold(0., f64::max);
    // Any witness belongs to two convex segments. The biggest relative
    // displacement of their endpoints bounds variation anywhere between samples.
    let samples = 65_536;
    let min = (0..=samples)
        .map(|i| {
            let t = f64::from(i) / f64::from(samples);
            distance(std::array::from_fn(|j| from[j].lerp(to[j], t)))
        })
        .fold(f64::INFINITY, f64::min);
    let lower = min - speed_bound / (2. * f64::from(samples)) - 1e-14;
    assert!(lower > skin_diameter - 2.2e-11, "{lower:e}");
    let d = distance(to);
    assert!((d - skin_diameter).abs() < 1e-13);
    let derivative = (distance(std::array::from_fn(|i| to[i] + 1e-8 * velocity[i])) - d) / 1e-8;
    assert!((-0.00214..-0.00213).contains(&derivative), "{derivative:e}");
    println!(
        "independent swept edge minimum >= {lower:.16} m; endpoint gap {} m; closing {} m/s",
        d - skin_diameter,
        -derivative
    );
}

#[test]
fn an_edge_under_the_sheet_or_an_extra_receiver_keeps_the_original_refusal() {
    use gz_physics_xpbd::{Boundary, XpbdError};
    for extra in [false, true] {
        let mut b = material();
        let mut boundaries = cut::SURFACES.to_vec();
        let mut mobility = cut::MOBILITIES.to_vec();
        let Boundary::TranslatingCuboid {
            center,
            half_extents,
            orientation,
            velocity,
        } = boundaries[2]
        else {
            unreachable!()
        };
        if extra {
            boundaries.push(Boundary::StaticCuboid {
                center: [2.; 3],
                half_extents,
                orientation,
            });
            mobility.push(None);
        } else {
            boundaries[2] = Boundary::TranslatingCuboid {
                center,
                half_extents: [half_extents[0], 0.058, half_extents[2]],
                orientation,
                velocity,
            };
        }
        let result = step_coupled_damped_pads(
            &mut [&mut b],
            Duration::from_nanos(1000),
            &boundaries,
            &mobility,
            &[(0, 4.), (1, 4.)],
        );
        assert!(
            matches!(result, Err(XpbdError::UnresolvedSurface)),
            "{result:?}"
        );
        assert_eq!(b.particles(), cut::INCOMING);
        assert!(b.last_boundary_impulses().is_empty());
    }
}
