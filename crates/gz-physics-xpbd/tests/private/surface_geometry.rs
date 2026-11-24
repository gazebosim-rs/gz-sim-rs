// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use parry3d_f64::{
    math::{Pose, Vector},
    query::{PointQuery, details::distance_segment_segment},
    shape::{Segment, Triangle},
};

fn reference(p: [DVec3; 4], vf: bool) -> f64 {
    let [a, b, c, d] = p.map(|v| Vector::from_array(v.to_array()));
    if vf {
        Triangle::new(b, c, d).distance_to_local_point(a, false)
    } else {
        distance_segment_segment(&Pose::IDENTITY, &Segment::new(a, b), &Segment::new(c, d))
    }
}

#[test]
fn independent_native_vertex_face_and_edge_edge_distances_cover_voronoi_regions() {
    let mut seed = 0x98ec7561u64;
    let mut rand = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 11) as f64) * 2f64.powi(-52) - 1.
    };
    let mut worst = 0.0_f64;
    for _ in 0..2048 {
        let p = std::array::from_fn(|_| DVec3::new(rand(), rand(), rand()));
        for vf in [true, false] {
            let witness = if vf { vertex_face(&p) } else { edge_edge(&p) };
            worst = worst.max((witness.delta.length() - reference(p, vf)).abs());
            assert!(witness.weights.iter().sum::<f64>().abs() < 1e-14);
        }
    }
    assert!(worst < 1e-12, "max independent distance error {worst:e}");
    println!("4096 native feature distances, maximum error {worst:e} m");
}

#[test]
fn deforming_sweeps_match_independent_native_distance_root() {
    let mut worst = 0.0_f64;
    for vf in [true, false] {
        let raw = if vf {
            [
                [0.11, 0.32, 0.34],
                [-1., -1., 0.],
                [1., -1., 0.],
                [0., 2., 0.],
            ]
        } else {
            [[-1., 0., 0.], [1., 0., 0.], [0., -1., 0.34], [0., 1., 0.34]]
        };
        let start = raw.map(DVec3::from);
        for k in 0..64 {
            let f = (k as f64 - 32.) / 64.;
            let disp = if vf {
                [
                    [0.29, -0.19, -0.83],
                    [0.13, 0.34 * f, 0.26 * f],
                    [-0.18, 0.09, 0.17],
                    [0.31, -0.04, -0.24],
                ]
            } else {
                [
                    [0.18, 0.11, 0.23 * f],
                    [0.19 * f, 0.15, 0.20],
                    [-0.09, 0.03, -0.81],
                    [0.18, 0.32, -0.68],
                ]
            }
            .map(DVec3::from);
            let at = |t: f64| std::array::from_fn(|i| start[i] + disp[i] * t);
            let gap = 0.02;
            let index = (1..=1024)
                .find(|i| reference(at(*i as f64 / 1024.), vf) <= gap)
                .expect("independent crossing bracket");
            let mut lo = (index - 1) as f64 / 1024.;
            let mut hi = index as f64 / 1024.;
            for _ in 0..45 {
                let mid = (lo + hi) * 0.5;
                if reference(at(mid), vf) > gap {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let particles = std::array::from_fn::<_, 4, _>(|i| Particle {
                position: start[i].to_array(),
                velocity: disp[i].to_array(),
                inverse_mass: 1.,
            });
            let hit = Stencil {
                ids: [0, 1, 2, 3],
                vf,
            }
            .sweep(
                &at(1.),
                &particles,
                gap,
                1e-11,
                &mut 0,
                SweepMode::Geometric,
            )
            .unwrap()
            .expect("feature impact");
            worst = worst.max((hit.time - hi).abs());
        }
    }
    assert!(worst < 1e-9, "deforming CCD time error {worst:e}");
    println!("128 deforming feature impacts, max time error {worst:e} normalized substeps");
}

#[test]
fn triangle_admission_matches_native_including_interlocked_pairs() {
    let mut seed = 0x181a7ba7u64;
    let mut rand = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 11) as f64 * 2f64.powi(-52) - 1.
    };
    let mut count = [0, 0];
    for _ in 0..2048 {
        let mut triangle = || std::array::from_fn(|_| DVec3::new(rand(), rand(), rand()));
        let a = triangle();
        let b = triangle();
        let native = |q: [DVec3; 3]| {
            let [x, y, z] = q.map(|p| Vector::from_array(p.to_array()));
            Triangle::new(x, y, z)
        };
        let hit = parry3d_f64::query::intersection_test(
            &Pose::IDENTITY,
            &native(a),
            &Pose::IDENTITY,
            &native(b),
        )
        .unwrap();
        count[usize::from(hit.intersecting)] += 1;
        assert_eq!(geometry::intersect(a, b), hit.intersecting);
    }
    assert!(count.iter().all(|&n| n > 200));
    println!(
        "native SAT admission: {} disjoint, {} interlocked",
        count[0], count[1]
    );
}

#[test]
fn moving_triangle_clearance_certificates_agree_with_independent_distances() {
    let mut seed = 0x97c358du64;
    let mut random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 11) as f64 * 2f64.powi(-52) - 1.
    };
    let mut certified = 0;
    for _ in 0..1024 {
        let start: [DVec3; 4] = std::array::from_fn(|_| DVec3::new(random(), random(), random()));
        let end: [DVec3; 4] = start.map(|p| p + DVec3::new(random(), random(), random()) * 0.4);
        let gap = 0.2;
        if interval::clear(start, end, gap) {
            certified += 1;
            for k in 0..=64 {
                let t = k as f64 / 64.;
                let at = std::array::from_fn(|i| start[i].lerp(end[i], t));
                assert!(reference(at, true) > gap);
            }
        }
    }
    assert!(
        certified > 500 && certified < 1024,
        "{certified} certificates"
    );
    println!("{certified}/1024 certified-clear sweeps checked at 65 native Parry samples each");
}

#[test]
fn slow_near_grazing_face_entry_is_bracketed_in_the_original_pad_sweep() {
    let start = [
        [
            -0.02300072560488636,
            0.037003969757911356,
            0.040498519762837144,
        ],
        [
            -0.024996658122157234,
            5.069387721227858e-7,
            0.040500166820819326,
        ],
        [
            -0.01249670109365012,
            1.7053020329669318e-5,
            0.04050006062321841,
        ],
        [
            -0.012496644550896448,
            9.659278063746107e-5,
            0.05300017647345791,
        ],
    ]
    .map(DVec3::from);
    let end = [
        [
            -0.02300071812032942,
            0.03700351905140542,
            0.040498441242862115,
        ],
        [
            -0.024996790396309653,
            -9.572819954299346e-7,
            0.040499585276545985,
        ],
        [
            -0.012496824315733141,
            2.4272187821545996e-5,
            0.04049946181557727,
        ],
        [
            -0.012496817046988342,
            8.461355680878522e-5,
            0.05299971525939452,
        ],
    ]
    .map(DVec3::from);
    let gap = 0.037;
    let native = |t| reference(std::array::from_fn(|i| start[i].lerp(end[i], t)), true);
    assert!(native(0.) > gap && native(1.) < gap);
    let mut lo = 0.;
    let mut hi = 1.;
    for _ in 0..50 {
        let mid = (lo + hi) / 2.;
        if native(mid) > gap {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let hit = first_entry(start, end, true, gap, 3.7e-11, &mut 0)
        .unwrap()
        .unwrap();
    assert!((native(hit.time) - gap).abs() < 4e-11);
    assert!((hit.time - hi).abs() < 0.0005, "{} vs {hi}", hit.time);
}

#[test]
fn sliding_edge_and_face_interval_bounds_are_separating_in_native_geometry() {
    let mut state = 0x8f4c5511u64;
    let mut random = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (state >> 11) as f64 * 2f64.powi(-52) - 1.
    };
    let mut count = [0; 2];
    for k in 0..2048 {
        let mut a: [DVec3; 4] = std::array::from_fn(|_| DVec3::new(random(), random(), random()));
        let mut b = a.map(|p| p + DVec3::new(random(), random(), random()) * 0.3);
        if k % 4 == 0 {
            // Include exactly parallel moving segments.
            a[3] = a[2] + (a[1] - a[0]);
            b[3] = b[2] + (b[1] - b[0]);
        }
        for (i, clear) in [
            interval::clear_edges(a, b, 0.12),
            interval::clear_triangle(a, b, 0.12, true),
        ]
        .into_iter()
        .enumerate()
        {
            if !clear {
                continue;
            }
            count[i] += 1;
            for t in 0..=80 {
                let p = std::array::from_fn(|j| a[j].lerp(b[j], t as f64 / 80.));
                assert!(reference(p, i == 1) > 0.12);
            }
        }
    }
    assert!(count.into_iter().all(|n| n > 900 && n < 2048));
    println!(
        "2048 moving configurations: {} edge-edge and {} vertex-face certificates independently sampled 81 times",
        count[0], count[1]
    );
}

#[test]
fn near_parallel_folded_hems_separate_at_the_end_of_a_loaded_substep() {
    // Frozen contact at 611 ms in the 22 mm folded-shell grasp. Structural
    // and pad corrections make the endpoint velocities differ from the chord.
    let start = [
        [
            0.30895660512080986,
            -4.750021818417992e-8,
            0.1310012681176649,
        ],
        [0.30898127283281324, 0.0530000248167637, 0.131],
        [
            0.3310644288022167,
            5.642826278344346e-8,
            0.13100126812810606,
        ],
        [0.3310930095543092, 0.05300309676578527, 0.1310126131438334],
    ]
    .map(DVec3::from);
    let end = [
        [
            0.30901189622393016,
            -1.124012750539129e-6,
            0.13100238299470862,
        ],
        [
            0.30904970409640187,
            0.052997589993278145,
            0.13099158466088895,
        ],
        [
            0.3310119600371572,
            2.192299523782285e-6,
            0.13100043026594277,
        ],
        [
            0.33104969820383817,
            0.05300236658980045,
            0.13101715212111573,
        ],
    ]
    .map(DVec3::from);
    let velocities = [
        [
            -0.0020766626195484507,
            0.003127726932714058,
            0.0011543762347388665,
        ],
        [
            0.013810143301156609,
            -0.0014313303691684512,
            -0.0013831354862747043,
        ],
        [
            -0.0017010032429956438,
            -0.002967450975537347,
            -0.0019030273708278889,
        ],
        [
            0.013834391068041456,
            6.461373762071823e-5,
            -0.0007030591887868124,
        ],
    ]
    .map(DVec3::from);
    let stencil = Stencil {
        ids: [0, 1, 2, 3],
        vf: false,
    };
    let gap = 0.022;
    let eps = 2.200941093527743e-11;
    let mut queries = 0;
    assert!(
        stencil
            .sweep_points(
                start,
                end,
                velocities,
                gap,
                eps,
                &mut queries,
                SweepMode::MaterialShell
            )
            .unwrap()
            .is_none()
    );
    // The capsule sweep owns a separate trajectory envelope. It must still
    // request coupled subdivision when the original certificate is inconclusive.
    assert!(matches!(
        material_sweep(start, end, velocities, false, gap, eps, &mut 0),
        Err(XpbdError::UnresolvedSurface)
    ));
    for i in 0..=256 {
        let at = std::array::from_fn(|id| start[id].lerp(end[id], f64::from(i) / 256.));
        assert!(reference(at, false) >= gap - 4. * eps);
    }
    // Translate one layer a further micrometre toward the other. A real
    // overlap at the same almost-parallel pose must still produce a response.
    let mut penetrating = end;
    penetrating[2].x -= 1e-6;
    penetrating[3].x -= 1e-6;
    let hit = stencil
        .sweep_points(
            start,
            penetrating,
            velocities,
            gap,
            eps,
            &mut 0,
            SweepMode::MaterialShell,
        )
        .unwrap()
        .unwrap();
    assert!(hit.depth > 0.9e-6);
}

#[path = "landing_cut.rs"]
mod landing_cut;

#[test]
fn loaded_floor_cut_keeps_separate_material_features_free_and_balances_support() {
    let mut p = landing_cut::PARTICLES.map(|r| Particle {
        position: [r[0], r[1], r[2]],
        velocity: [r[3], r[4], r[5]],
        inverse_mass: r[6],
    });
    let mut x = landing_cut::PARTICLES.map(|r| DVec3::new(r[7], r[8], r[9]));
    let mut shell = Surface::new(landing_cut::FACES.to_vec(), &p).unwrap();
    shell.set_friction(0.3).unwrap();
    assert_eq!(
        shell.verify(&x, &p, 0.022),
        Err(XpbdError::UnresolvedSurface)
    );
    let before = x;
    let momentum = |p: &[Particle]| {
        p.iter()
            .map(|v| DVec3::from(v.velocity) / v.inverse_mass)
            .sum::<DVec3>()
    };
    let initial = momentum(&p);
    let mut impulse = DVec3::ZERO;
    shell
        .finish_supported(&mut x, &mut p, 0.022, |_, p| {
            let incoming = DVec3::from(p[0].velocity);
            if incoming.z < 0. {
                let side = DVec3::new(incoming.x, incoming.y, 0.);
                let delta = -incoming.z * DVec3::Z
                    - side * ((-incoming.z * 0.7) / side.length().max(f64::MIN_POSITIVE)).min(1.);
                p[0].velocity = (incoming + delta).to_array();
                impulse += delta / p[0].inverse_mass;
            }
            Ok(())
        })
        .unwrap();
    assert_eq!(x, before);
    assert!((momentum(&p) - initial - impulse).length() < 1e-12);
}

#[test]
fn a_rolling_edge_uses_the_incoming_material_normal_at_the_plane() {
    let data = landing_cut::GRAZE;
    let p = data.map(|r| Particle {
        position: [r[0], r[1], r[2]],
        velocity: [r[3], r[4], r[5]],
        inverse_mass: r[6],
    });
    let resolved = data.map(|r| DVec3::new(r[7], r[8], r[9]));
    let hit = Hit {
        stencil: Stencil {
            ids: [0, 1, 10, 15],
            vf: false,
        },
        weights: [
            0.9917476724030727,
            0.008252327596927332,
            -0.006446173275961642,
            -0.9935538267240384,
        ],
        normal: DVec3::from_array([
            -0.9990431000398635,
            0.03596606913537136,
            -0.02488626395602196,
        ]),
        time: 0.4510735807241938,
        depth: 1.250371889360835e-10,
        speed: 4.882558760632705e-07,
    };
    let rings = crate::material::patches(&landing_cut::FACES).unwrap();
    let supports = [
        rings[&[0, 1, usize::MAX]].as_slice(),
        rings[&[10, 15, usize::MAX]].as_slice(),
    ];
    let mut displacement = [DVec3::ZERO; 20];
    super::position::correct(&hit, supports, &mut displacement, &p).unwrap();
    let mut x = std::array::from_fn::<_, 20, _>(|i| resolved[i] - displacement[i]);
    let mut shell = Surface::new(landing_cut::FACES.to_vec(), &p).unwrap();
    shell.set_friction(0.3).unwrap();
    let incoming = p;
    let mut p = p;
    shell.resolve(&hit, &mut x, &mut p, 0.022, 9.76e-7).unwrap();
    assert_eq!(p, incoming);
    assert_eq!(shell.impulse.normal, 0.);
    for (actual, expected) in x.iter().zip(resolved) {
        assert!((*actual - expected).length() < 1e-15);
    }
    let eps = Surface::tolerance(&x, &p, 0.022).unwrap();
    assert!(
        hit.stencil
            .sweep(&x, &p, 0.022, eps, &mut 0, SweepMode::MaterialShell)
            .unwrap()
            .is_none()
    );

    // A support can stop the material after an earlier incoming event.
    // The old normal must not accelerate the resting patch.
    let mut hit = hit;
    hit.speed = 1e-3;
    for particle in &mut p {
        particle.velocity = [0.; 3];
    }
    let stopped = p;
    let mut dx = [DVec3::ZERO; 20];
    super::position::correct(&hit, supports, &mut dx, &p).unwrap();
    let mut x = std::array::from_fn::<_, 20, _>(|i| resolved[i] - dx[i]);
    shell.resolve(&hit, &mut x, &mut p, 0.022, 9.76e-7).unwrap();
    assert_eq!(p, stopped);
    assert_eq!(shell.impulse.normal, 0.);
    for (end, expected) in x.iter().zip(resolved) {
        assert!((*end - expected).length() < 1e-15);
    }
    // With no overlap left, a closing non-passive witness must be refused.
    hit.depth = 0.;
    assert_eq!(
        shell.resolve(&hit, &mut x, &mut p, 0.022, 9.76e-7),
        Err(XpbdError::UnresolvedSurface)
    );
}

#[test]
fn only_distance_exhaustion_requests_temporal_subdivision() {
    let p = landing_cut::PARTICLES.map(|r| Particle {
        position: [r[0], r[1], r[2]],
        velocity: [r[3], r[4], r[5]],
        inverse_mass: r[6],
    });
    let x = landing_cut::PARTICLES.map(|r| DVec3::new(r[7], r[8], r[9]));
    let gap = 0.022;
    let eps = Surface::tolerance(&x, &p, gap).unwrap();
    let mut shell = Surface::new(landing_cut::FACES.to_vec(), &p).unwrap();
    shell.set_friction(0.3).unwrap();
    let mut candidates = MAX_CANDIDATES;
    assert!(matches!(
        shell.next_hit(&x, &p, gap, eps, (&mut candidates, &mut 0), true),
        Err(XpbdError::SurfaceBudget)
    ));
    assert!(!shell.subdivide_on_budget);
    let mut queries = MAX_DISTANCE_QUERIES;
    assert!(matches!(
        shell.next_hit(&x, &p, gap, eps, (&mut 0, &mut queries), true),
        Err(XpbdError::SurfaceBudget)
    ));
    assert!(shell.subdivide_on_budget);
}

#[test]
fn a_rotating_separator_certifies_only_truly_clear_sweeps() {
    let mut seed = 0xa39eeed9u64;
    let mut next = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((seed >> 11) as f64) * 2f64.powi(-52) - 1.
    };
    for vf in [false, true] {
        let mut counts = [0; 2];
        for _ in 0..512 {
            let a = std::array::from_fn(|_| DVec3::new(next(), next(), next()));
            let b = a.map(|u| u + 0.25 * DVec3::new(next(), next(), next()));
            let witness = |p: &[DVec3; 4]| if vf { vertex_face(p) } else { edge_edge(p) };
            let normals = [witness(&a).delta.normalize(), witness(&b).delta.normalize()];
            let gap = 0.35;
            let safe = interval::moving_plane(&a, &b, normals, if vf { 1 } else { 2 }, gap);
            counts[usize::from(safe)] += 1;
            if safe {
                for i in 0..=64 {
                    let x = std::array::from_fn(|j| a[j].lerp(b[j], f64::from(i) / 64.));
                    assert!(reference(x, vf) > gap);
                }
            }
        }
        assert!(counts.into_iter().all(|n| n > 0));
    }
}
