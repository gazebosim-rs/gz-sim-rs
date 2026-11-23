// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::{Boundary, Particle, SoftBody, SolverConfig, SphereMobility, step_coupled};
use std::time::Duration;

const R: f64 = 0.08;
const THICK: f64 = 0.02;
fn cfg(n: u32, mu: f64) -> SolverConfig {
    SolverConfig {
        gravity: [0.; 3],
        substeps: n,
        iterations: 4,
        particle_radius: THICK,
        self_contact: true,
        friction: mu,
    }
}
fn p(x: f64, y: f64, w: f64) -> Particle {
    Particle {
        position: [x, y, 0.],
        velocity: [0.; 3],
        inverse_mass: w,
    }
}
fn sphere(center: [f64; 3], velocity: [f64; 3]) -> Boundary {
    Boundary::Sphere {
        center,
        radius: R,
        velocity,
        angular_velocity: [0.; 3],
    }
}
fn mob() -> SphereMobility {
    SphereMobility {
        inverse_mass: 1.,
        inverse_inertia: 1. / (0.4 * R * R),
    }
}
fn near(a: f64, b: f64, t: f64) {
    assert!((a - b).abs() <= t, "{a:.12} vs {b:.12}, tolerance {t:e}");
}

#[test]
fn coarse_and_refined_fixed_panels_stop_a_between_vertex_crossing() {
    for n in [1, 2, 4] {
        for cols in [2, 5, 13] {
            let mut particles = Vec::new();
            for row in 0..cols {
                for col in 0..cols {
                    particles.push(p(
                        -0.5 + col as f64 / (cols - 1) as f64,
                        -0.5 + row as f64 / (cols - 1) as f64,
                        0.,
                    ));
                }
            }
            let mut body = SoftBody::cloth(particles, cols, 0., 0., 0., cfg(n, 0.))
                .unwrap()
                .with_pad_surface_contact()
                .unwrap();
            let b = sphere([0.13, 0.043, 0.18], [0., 0., -3.]);
            let m = step_coupled(
                &mut [&mut body],
                Duration::from_millis(90),
                &[b],
                &[Some(mob())],
            )
            .unwrap()[0]
                .unwrap();
            near(0.18 - 0.27 + m.translation_correction[2], R + THICK, 2e-10);
            near(m.linear_velocity_correction[2], 3., 1e-9);
            near(body.last_boundary_impulses()[0].linear[2], 3., 1e-9);
            near(
                DVec3::from(m.angular_velocity_correction).length(),
                0.,
                1e-8,
            );
        }
    }
}

#[test]
fn sliding_finite_pad_sticks_on_physical_skin_with_the_expected_rotational_inertia() {
    for n in [1, 2, 4] {
        let mut b = SoftBody::new(
            vec![p(-1., -1., 0.), p(1., -1., 0.), p(0., 2., 0.)],
            vec![],
            cfg(n, 0.7),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let sphere = sphere([0.14, -0.1, 0.12], [0.3, 0., -2.]);
        let m = step_coupled(
            &mut [&mut b],
            Duration::from_millis(10),
            &[sphere],
            &[Some(mob())],
        )
        .unwrap()[0]
            .unwrap();
        let jt = 0.3 / (1. + R * R * mob().inverse_inertia);
        near(m.linear_velocity_correction[0], -jt, 2e-10);
        near(m.linear_velocity_correction[2], 2., 2e-10);
        near(
            m.angular_velocity_correction[1],
            R * jt * mob().inverse_inertia,
            2e-8,
        );
    }
}

#[test]
fn free_glancing_receiver_and_triangle_conserve_spatial_momentum() {
    for mu in [0., 0.45] {
        let mut b = SoftBody::new(
            vec![p(-1., -1., 0.5), p(1., -1., 1.), p(0., 2., 2.)],
            vec![],
            cfg(1, mu),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let c = DVec3::new(0.13, -0.14, 0.12);
        let v = DVec3::new(0.4, 0.13, -2.);
        let sphere = sphere(c.to_array(), v.to_array());
        let dt = 0.01;
        let m = step_coupled(
            &mut [&mut b],
            Duration::from_millis(10),
            &[sphere],
            &[Some(mob())],
        )
        .unwrap()[0]
            .unwrap();
        let vf = v + DVec3::from(m.linear_velocity_correction);
        let omega = DVec3::from(m.angular_velocity_correction);
        let cf = c + dt * v + DVec3::from(m.translation_correction);
        let mut total = vf;
        let mut l = cf.cross(vf) + omega / mob().inverse_inertia;
        let mut energy =
            0.5 * (vf.length_squared() + omega.length_squared() / mob().inverse_inertia);
        for p in b.particles() {
            let v = DVec3::from(p.velocity);
            let momentum = v / p.inverse_mass;
            total += momentum;
            l += DVec3::from(p.position).cross(momentum);
            energy += 0.5 * v.dot(momentum);
        }
        println!(
            "free off-center impact mu={mu}: |dP|={:.3e} kg m/s, |dL|={:.3e} kg m²/s, kinetic energy {:.6}→{energy:.6} J, sphere spin {:.6} rad/s",
            total.distance(v),
            l.distance(c.cross(v)),
            0.5 * v.length_squared(),
            omega.length()
        );
        near(total.distance(v), 0., 1e-10);
        near(l.distance(c.cross(v)), 0., 1e-10);
        assert!(energy < 0.5 * v.length_squared());
        if mu > 0. {
            assert!(omega.length() > 1.);
        }
    }
}

#[test]
fn finite_sphere_sweeps_across_material_edge_and_resolves_a_true_near_miss() {
    for (y, miss) in [(-0.12, false), (-0.101, true)] {
        let mut b = SoftBody::new(
            vec![p(-1., 0., 0.), p(1., 0., 0.), p(0., 1., 0.)],
            vec![],
            cfg(1, 0.),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let v = if miss { [0., 0., -3.] } else { [0., 0.6, -0.8] };
        let start = [0.21, y, 0.16];
        let span = Duration::from_millis(160);
        let m = step_coupled(&mut [&mut b], span, &[sphere(start, v)], &[Some(mob())]).unwrap()[0]
            .unwrap();
        let actual = DVec3::from(v) + DVec3::from(m.linear_velocity_correction);
        if miss {
            near(actual.distance(v.into()), 0., 1e-12);
        } else {
            near(actual.length(), 0., 1e-10);
        }
    }
}

#[test]
fn frictionless_midstep_glance_has_no_artificial_sphere_spin() {
    for n in [1, 2, 4] {
        let mut body = SoftBody::new(
            vec![p(-1., -1., 0.), p(1., -1., 0.), p(0., 2., 0.)],
            vec![],
            cfg(n, 0.),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let m = step_coupled(
            &mut [&mut body],
            Duration::from_millis(40),
            &[sphere([0.08, 0.2, 0.13], [0.4, -0.17, -2.])],
            &[Some(mob())],
        )
        .unwrap()[0]
            .unwrap();
        let actual = DVec3::new(0.4, -0.17, -2.) + DVec3::from(m.linear_velocity_correction);
        near(actual.distance(DVec3::new(0.4, -0.17, 0.)), 0., 1e-9);
        near(
            DVec3::from(m.angular_velocity_correction).length(),
            0.,
            1e-8,
        );
    }
}

#[test]
fn intersected_center_and_unsupported_receiver_sets_rollback() {
    use gz_physics_xpbd::XpbdError;
    let mk = || {
        SoftBody::new(
            vec![p(-1., -1., 1.), p(1., -1., 1.), p(0., 2., 1.)],
            vec![],
            cfg(1, 0.),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap()
    };
    let mut body = mk();
    let initial = body.particles().to_vec();
    let force = sphere([0., 0., 0.], [0., 0., 0.]);
    assert!(matches!(
        step_coupled(
            &mut [&mut body],
            Duration::from_millis(2),
            &[force],
            &[Some(mob())]
        ),
        Err(XpbdError::UnresolvedSurface)
    ));
    assert_eq!(body.particles(), initial);
    assert!(matches!(
        step_coupled(
            &mut [&mut body],
            Duration::from_millis(2),
            &[force],
            &[None]
        ),
        Err(XpbdError::InvalidBoundary)
    ));
    let mut other = mk();
    assert!(matches!(
        step_coupled(
            &mut [&mut body, &mut other],
            Duration::from_millis(2),
            &[force],
            &[Some(mob())]
        ),
        Err(XpbdError::InvalidConfig)
    ));
    assert!(matches!(
        step_coupled(
            &mut [&mut body],
            Duration::from_millis(2),
            &[force; 5],
            &[Some(mob()); 5]
        ),
        Err(XpbdError::SurfaceBudget)
    ));
    assert_eq!(body.particles(), initial);
    assert!(body.last_boundary_impulses().is_empty());
}

#[test]
fn either_winding_and_swapped_diagonal_give_the_same_shared_edge_contact() {
    for tris in [
        vec![[0, 1, 2], [0, 2, 3]],
        vec![[3, 2, 0], [2, 1, 0]],
        vec![[0, 1, 3], [1, 2, 3]],
    ] {
        let mut b = SoftBody::new(
            vec![
                p(-1., -1., 0.),
                p(1., -1., 0.),
                p(1., 1., 0.),
                p(-1., 1., 0.),
            ],
            vec![],
            cfg(1, 1.),
        )
        .unwrap()
        .with_self_contact_surface(tris)
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let m = step_coupled(
            &mut [&mut b],
            Duration::from_millis(10),
            &[sphere([-0.003, 0., 0.12], [0.3, 0., -2.])],
            &[Some(mob())],
        )
        .unwrap()[0]
            .unwrap();
        near(
            m.angular_velocity_correction[1],
            R * (0.3 / 3.5) * mob().inverse_inertia,
            1e-8,
        );
        near(m.linear_velocity_correction[2], 2., 1e-9);
    }
}

#[test]
fn opposite_pads_stop_symmetrically_on_a_free_membrane() {
    for n in [1, 2, 4] {
        let mut body = SoftBody::new(
            vec![p(-1., -1., 1.), p(1., -1., 1.), p(0., 2., 1.)],
            vec![],
            cfg(n, 0.),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let pads = [
            sphere([0., 0., 0.12], [0., 0., -1.]),
            sphere([0., 0., -0.12], [0., 0., 1.]),
        ];
        let motion = step_coupled(
            &mut [&mut body],
            Duration::from_millis(50),
            &pads,
            &[Some(mob()); 2],
        )
        .unwrap();
        for (i, sign) in [1., -1.].into_iter().enumerate() {
            let m = motion[i].unwrap();
            near(m.linear_velocity_correction[2], sign, 2e-9);
            near(m.translation_correction[2], sign * 0.03, 2e-10);
        }
        for p in body.particles() {
            near(DVec3::from(p.velocity).length(), 0., 1e-9);
        }
    }
}

#[test]
fn moving_finite_receiver_on_three_resolutions_of_the_same_free_sheet() {
    use parry3d_f64::{math::Vector, query::PointQuery, shape::Triangle};
    // Same 0.4 m square, 2 kg areal mass integral, collision shell, and
    // drive. The lumped nodal masses integrate each incident quarter cell.
    for columns in [3usize, 5, 9] {
        let cell = 0.4 / (columns - 1) as f64;
        let cell_mass = 2. / ((columns - 1) * (columns - 1)) as f64;
        let axis_mass = |i| if i == 0 || i + 1 == columns { 0.5 } else { 1. };
        let particles = (0..columns)
            .flat_map(|row| {
                (0..columns).map(move |col| {
                    p(
                        col as f64 * cell - 0.2,
                        row as f64 * cell - 0.2,
                        1. / (cell_mass * axis_mass(row) * axis_mass(col)),
                    )
                })
            })
            .collect();
        let mut sheet = SoftBody::cloth(particles, columns, 1e-5, 2e-5, 0.02, cfg(2, 0.))
            .unwrap()
            .with_pad_surface_contact()
            .unwrap();
        let next = Duration::from_micros(500);
        let mut center = DVec3::new(0.017, -0.012, 0.115);
        let initial_velocity = DVec3::new(0.1, 0.03, -1.);
        let mut v = initial_velocity;
        let mut spin = DVec3::ZERO;
        let triangles = (1..columns)
            .flat_map(|row| {
                (1..columns).flat_map(move |col| {
                    let i = row * columns + col;
                    [
                        [i - columns - 1, i - columns, i],
                        [i - columns - 1, i, i - 1],
                    ]
                })
            })
            .collect::<Vec<_>>();
        let mut min_clearance = f64::INFINITY;
        let mut max_deflection = 0.0f64;
        for _ in 0..80 {
            let boundary = Boundary::Sphere {
                center: center.to_array(),
                radius: R,
                velocity: v.to_array(),
                angular_velocity: spin.to_array(),
            };
            let result = step_coupled(&mut [&mut sheet], next, &[boundary], &[Some(mob())])
                .unwrap()[0]
                .unwrap();
            center += v * next.as_secs_f64() + DVec3::from(result.translation_correction);
            v += DVec3::from(result.linear_velocity_correction);
            spin += DVec3::from(result.angular_velocity_correction);
            for ids in &triangles {
                let [a, b, c] = ids.map(|i| Vector::from_array(sheet.particles()[i].position));
                let distance = Triangle::new(a, b, c)
                    .distance_to_local_point(Vector::from_array(center.to_array()), false);
                min_clearance = min_clearance.min(distance - R - THICK);
            }
            for p in sheet.particles() {
                max_deflection = max_deflection.max(-p.position[2]);
            }
        }
        let momentum: DVec3 = sheet
            .particles()
            .iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum();
        let error = (momentum + v - initial_velocity).length();
        let energy = 0.5
            * (v.length_squared()
                + spin.length_squared() / mob().inverse_inertia
                + sheet
                    .particles()
                    .iter()
                    .map(|p| DVec3::from(p.velocity).length_squared() / p.inverse_mass)
                    .sum::<f64>());
        println!(
            "{columns}x{columns} moving sheet: Jz={:.6} N s, peak deflection {max_deflection:.6} m, minimum independent skin clearance {min_clearance:e} m, momentum error {error:e}, final kinetic energy {energy:.6} J",
            v.z - initial_velocity.z
        );
        assert!(
            error < 1e-9
                && min_clearance >= -1.1e-8
                && energy <= 0.5 * initial_velocity.length_squared() + 1e-7
        );
        assert!(
            max_deflection > 0.005 && max_deflection < 0.04 && v.z > initial_velocity.z && v.z < 0.
        );
    }
}

#[test]
fn initially_touching_pad_hits_then_leaves_the_edge_in_one_interval() {
    use gz_physics_xpbd::XpbdError;
    // Throughout the free step the nearest feature is the x-directed edge.
    // Contact starts along (0,-.6,.8); the 0.5 m/s incident speed is arrested.
    for friction in [0., 0.3] {
        let mut sheet = SoftBody::new(
            vec![p(-1., 0., 0.), p(1., 0., 0.), p(0., 1., 0.)],
            vec![],
            cfg(1, friction),
        )
        .unwrap()
        .with_self_contact_surface(vec![[0, 1, 2]])
        .unwrap()
        .with_pad_surface_contact()
        .unwrap();
        let seed = sheet.particles().to_vec();
        let v = DVec3::new(0., -0.9, -1.3);
        let n = DVec3::new(0., -0.6, 0.8);
        let result = step_coupled(
            &mut [&mut sheet],
            Duration::from_millis(120),
            &[sphere([0.2, -0.06, 0.08], v.to_array())],
            &[Some(mob())],
        );
        if friction == 0. {
            let corrected = v + DVec3::from(result.unwrap()[0].unwrap().linear_velocity_correction);
            near(corrected.distance(v - n * v.dot(n)), 0., 1e-9);
        } else {
            assert!(matches!(result, Err(XpbdError::UnresolvedSurface)));
            assert_eq!(seed, sheet.particles());
        }
    }
}
