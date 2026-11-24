// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use glam::DVec3;
use gz_assets::mesh::capsule_visual_glb;
use std::collections::BTreeMap;

#[test]
fn si_capsule_has_two_closed_hemispheres_unit_normals_and_a_straight_barrel() {
    let color = [0.22, 0.49, 0.84, 1.];
    for (radius, length) in [(0.03125, 0.5), (0.25, 0.0625)] {
        let bytes = capsule_visual_glb(radius, length, color).unwrap();
        assert_eq!(capsule_visual_glb(radius, length, color).unwrap(), bytes);
        assert!(bytes.len() < 16 * 1024 * 1024);
        let doc = gltf::Gltf::from_slice(&bytes).unwrap();
        let prim = doc.meshes().next().unwrap().primitives().next().unwrap();
        assert_eq!(
            prim.material().pbr_metallic_roughness().base_color_factor(),
            color
        );
        let reader = prim.reader(|_| doc.blob.as_deref());
        // Inverse of simulation Z-up to glTF Y-up.
        let z_up = |[x, y, z]: [f32; 3]| DVec3::from_array([x, -z, y].map(f64::from));
        let points: Vec<_> = reader.read_positions().unwrap().map(z_up).collect();
        let normals: Vec<_> = reader.read_normals().unwrap().map(z_up).collect();
        let indices: Vec<_> = reader.read_indices().unwrap().into_u32().collect();
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        let half = length / 2.;
        let mut edges = BTreeMap::new();
        let mut cap_bands = [
            std::collections::BTreeSet::new(),
            std::collections::BTreeSet::new(),
        ];
        for (&p, &normal) in points.iter().zip(&normals) {
            min = min.min(p);
            max = max.max(p);
            let offset = p - DVec3::Z * p.z.clamp(-half, half);
            assert!((offset.length() - radius).abs() < 1e-7);
            assert!((offset.normalize() - normal).length() < 1e-6);
            assert!((normal.length() - 1.).abs() < 1e-7);
            if p.z.abs() > half {
                cap_bands[usize::from(p.z > 0.)].insert(p.z.to_bits());
            }
        }
        assert_eq!(max, DVec3::new(radius, radius, half + radius));
        assert_eq!(min, -max);
        assert_eq!(cap_bands.map(|b| b.len()), [16, 16]);
        let mut barrel_faces = 0;
        for ids in indices.chunks_exact(3) {
            let [a, b, c] = [ids[0], ids[1], ids[2]].map(|i| points[i as usize]);
            let face = (b - a).cross(c - a).normalize();
            for i in 0..3 {
                assert!(face.dot(normals[ids[i] as usize]) > 0.99);
                let (u, v) = (ids[i], ids[(i + 1) % 3]);
                let count = edges.entry((u.min(v), u.max(v))).or_insert([0, 0]);
                count[usize::from(u < v)] += 1;
            }
            if [a, b, c].iter().all(|p| p.z.abs() == half) {
                assert_eq!(face.z, 0.);
                barrel_faces += 1;
            }
        }
        assert_eq!(barrel_faces, 128);
        assert!(edges.values().all(|counts| *counts == [1, 1]));
    }
}

#[test]
fn unrepresentable_capsule_dimensions_and_translucent_colors_are_rejected() {
    for (radius, length) in [
        (0., 1.),
        (1., 0.),
        (-1., 0.25),
        (1., -1.),
        (f64::NAN, 1.),
        (1., f64::INFINITY),
        (1., 1e60),
        (1e-60, 1.),
        (0.01, 1e12),
        (1e38, f64::MAX),
    ] {
        assert!(capsule_visual_glb(radius, length, [1.; 4]).is_err());
    }
    for color in [
        [0., 0., 0., 0.99],
        [-0.1, 0., 0., 1.],
        [1., f32::NAN, 0., 1.],
    ] {
        assert!(capsule_visual_glb(0.02, 0.14, color).is_err());
    }
}
