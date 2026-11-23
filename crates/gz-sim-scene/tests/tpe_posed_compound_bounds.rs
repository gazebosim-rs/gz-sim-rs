// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! TPE model envelopes remain kinematic AABB contact reports, not shape resolution.
use bevy_ecs::prelude::*;
use glam::{DQuat, DVec3};
use gz_physics::BodyState;
use gz_physics_tpe::{ContactLimit, Contacts, ModelBounds, Motion};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{Name, PhysicsEngine, load_world_with_engine, remove_model, reset_loaded_world};
use sdformat::NoResources;
use std::time::Duration;
fn near(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
fn rotation([x, y, z]: [f64; 3]) -> DQuat {
    DQuat::from_rotation_z(z) * DQuat::from_rotation_y(y) * DQuat::from_rotation_x(x)
}
fn envelope(points: impl IntoIterator<Item = DVec3>) -> (DVec3, DVec3) {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for p in points {
        lo = lo.min(p);
        hi = hi.max(p);
    }
    (lo, hi)
}
fn corners(lo: DVec3, hi: DVec3) -> Vec<DVec3> {
    let mut points = Vec::new();
    for x in [lo.x, hi.x] {
        for y in [lo.y, hi.y] {
            for z in [lo.z, hi.z] {
                points.push(DVec3::new(x, y, z));
            }
        }
    }
    points
}
fn primitive(kind: usize) -> (&'static str, [f64; 3], [f64; 3]) {
    match kind % 4 {
        0 => (
            "<box><size>2 1 3</size></box>",
            [1., 0.5, 1.5],
            [0.2, -0.3, 0.4],
        ),
        1 => (
            "<sphere><radius>0.5</radius></sphere>",
            [0.5; 3],
            [0.2, -0.3, 0.4],
        ),
        2 => (
            "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
            [0.5, 1., 0.5],
            [std::f64::consts::FRAC_PI_2, 0., 0.],
        ),
        _ => (
            "<capsule><radius>0.5</radius><length>2</length></capsule>",
            [1.5, 0.5, 0.5],
            [0., std::f64::consts::FRAC_PI_2, 0.],
        ),
    }
}
// This reference enumerates box corners and uses analytic round-shape 90-degree
// extrema. It then composes the retained LINK/model AABB envelopes explicitly.
fn bounds(count: usize, kind: usize, rotated: bool) -> (DVec3, DVec3) {
    let mut points = Vec::new();
    for i in 0..count {
        let (_, half, rpy) = primitive(kind + i);
        let center = DVec3::new(4. * i as f64, 0.75, -0.25);
        if (kind + i).is_multiple_of(4) {
            let q = rotation(rpy);
            points.extend(
                corners(-DVec3::from_array(half), DVec3::from_array(half))
                    .into_iter()
                    .map(|p| center + q * p),
            );
        } else {
            points.extend(corners(
                center - DVec3::from_array(half),
                center + DVec3::from_array(half),
            ));
        }
    }
    let (lo, hi) = envelope(points);
    let q = if rotated {
        rotation([0.3, 0.5, -0.2])
    } else {
        DQuat::IDENTITY
    };
    let link = DVec3::new(1., -2., 0.5);
    envelope(corners(lo, hi).into_iter().map(|p| link + q * p))
}
fn xml(count: usize, kind: usize, rotated: bool, fixed: bool, probe: [f64; 3]) -> String {
    let collision=(0..count).map(|i|{let(g,_,q)=primitive(kind+i);format!("<collision name='c{i}'><pose>{} 0.75 -0.25 {} {} {}</pose><geometry>{g}</geometry></collision>",4.*i as f64,q[0],q[1],q[2])}).collect::<String>();
    let link = if rotated {
        "1 -2 0.5 0.3 0.5 -0.2"
    } else {
        "1 -2 0.5 0 0 0"
    };
    format!(
        "<sdf version='1.12'><world name='w'><model name='subject'><static>{fixed}</static><link name='l'><pose>{link}</pose>{collision}<visual name='v'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model><model name='probe'><static>true</static><pose>{} {} {} 0 0 0</pose><link name='l'><collision name='c'><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision></link></model></world></sdf>",
        probe[0], probe[1], probe[2]
    )
}
fn load(text: &str) -> Simulation {
    let mut sim = load_world_with_engine(
        text,
        "tpe-posed",
        "w",
        Duration::from_millis(1),
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    sim.initialize_schedules().unwrap();
    sim
}
fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world().get::<EntityId>(named(sim, name)).unwrap().get()
}
fn points(sim: &Simulation) -> Vec<[f64; 3]> {
    sim.world()
        .resource::<Contacts>()
        .as_slice()
        .iter()
        .map(|c| c.point)
        .collect()
}
#[test]
fn tpe_all_primitives_posed_and_2_4_16_unions_have_independent_boundary_contacts() {
    for count in [1, 2, 4, 16] {
        for kind in 0..4 {
            for rotated in [false, true] {
                let (lo, hi) = bounds(count, kind, rotated);
                let middle = (lo + hi) * 0.5;
                for axis in 0..3 {
                    for gap in [-0.01, 0.01] {
                        let mut probe = middle;
                        probe[axis] = hi[axis] + 0.25 + gap;
                        let mut sim = load(&xml(count, kind, rotated, false, probe.to_array()));
                        sim.step_exact(1).unwrap();
                        let got = points(&sim);
                        assert_eq!(got.len(), usize::from(gap < 0.));
                        if gap < 0. {
                            let overlap_lo = lo.max(probe - DVec3::splat(0.25));
                            let overlap_hi = hi.min(probe + DVec3::splat(0.25));
                            vector(got[0], ((overlap_lo + overlap_hi) * 0.5).to_array());
                        }
                    }
                }
            }
        }
    }
}
#[test]
fn tpe_compound_bounds_keep_model_rotation_as_the_existing_outer_envelope() {
    let count = 4;
    let (lo, hi) = bounds(count, 0, true);
    let q = rotation([0.4, -0.1, 0.2]);
    let translation = DVec3::new(3., 2., 1.);
    let (world_lo, world_hi) = envelope(corners(lo, hi).into_iter().map(|p| translation + q * p));
    let mut probe = (world_lo + world_hi) * 0.5;
    probe.y = world_hi.y + 0.24;
    let text = xml(count, 0, true, false, probe.to_array()).replacen(
        "<model name='subject'>",
        "<model name='subject'><pose>3 2 1 0.4 -0.1 0.2</pose>",
        1,
    );
    let mut sim = load(&text);
    sim.step_exact(1).unwrap();
    assert_eq!(points(&sim).len(), 1);
    vector(
        points(&sim)[0],
        ((world_lo.max(probe - DVec3::splat(0.25)) + world_hi.min(probe + DVec3::splat(0.25)))
            * 0.5)
            .to_array(),
    );
}
#[test]
fn tpe_posed_bounds_motion_pause_contact_exit_and_both_resets_are_kinematic() {
    let (lo, hi) = bounds(2, 0, false);
    let mut probe = (lo + hi) * 0.5;
    probe.x = hi.x + 0.24;
    let mut sim = load(&xml(2, 0, false, false, probe.to_array()));
    let model = named(&sim, "w::subject");
    let public = id(&sim, "w::subject");
    sim.step_exact(1).unwrap();
    assert_eq!(points(&sim).len(), 1);
    sim.world_mut()
        .get_mut::<Motion>(model)
        .unwrap()
        .set_world_velocity([-10., 0., 0.], [0.; 3])
        .unwrap();
    sim.set_paused(true);
    let before = *sim.world().get::<Motion>(model).unwrap().state();
    sim.tick().unwrap();
    assert_eq!(*sim.world().get::<Motion>(model).unwrap().state(), before);
    sim.set_paused(false);
    sim.step_exact(100).unwrap();
    assert!(points(&sim).is_empty());
    near(
        sim.world().get::<Motion>(model).unwrap().state().position[0],
        -1.,
    );
    sim.reset().unwrap();
    vector(
        sim.world().get::<Motion>(model).unwrap().state().position,
        [0.; 3],
    );
    assert_eq!(points(&sim).len(), 1);
    remove_model(&mut sim, public).unwrap();
    sim.step_exact(1).unwrap();
    assert!(points(&sim).is_empty());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "w::subject"), public);
    assert_ne!(named(&sim, "w::subject"), model);
    assert_eq!(points(&sim).len(), 1);
}
#[test]
fn tpe_new_bounds_preserve_masks_static_filtering_and_contact_failure_atomicity() {
    let (lo, hi) = bounds(4, 0, false);
    let probe = ((lo + hi) * 0.5).to_array();
    let mut fixed = load(&xml(4, 0, false, true, probe));
    fixed.step_exact(1).unwrap();
    assert!(points(&fixed).is_empty());
    let mut masked = load(&xml(4, 0, false, false, probe));
    let model = named(&masked, "w::subject");
    masked
        .world_mut()
        .entity_mut(model)
        .insert(ModelBounds::new(lo.to_array(), hi.to_array(), false, 0).unwrap());
    masked.step_exact(1).unwrap();
    assert!(points(&masked).is_empty());
    let mut limited = load(&xml(4, 0, false, false, probe));
    limited.world_mut().resource_mut::<ContactLimit>().0 = 0;
    assert!(limited.step_exact(1).is_err());
    assert!(limited.is_faulted());
    assert!(points(&limited).is_empty());
    assert!(limited.step_exact(1).is_err());
}
#[test]
fn tpe_pose_and_union_invalid_domains_refuse_without_implicit_mesh_or_plane_fallback() {
    for count in [0, 17] {
        assert!(
            load_world_with_engine(
                &xml(count, 0, false, false, [1000.; 3]),
                "bad",
                "w",
                Duration::from_millis(1),
                &NoResources,
                PhysicsEngine::Tpe
            )
            .is_err()
        );
    }
    let valid = xml(2, 0, true, false, [1000.; 3]);
    for text in [
        valid.replace("4 0.75 -0.25", "1e308 1e308 -0.25"),
        valid.replace("<radius>0.5</radius>", "<radius>NaN</radius>"),
        valid.replace(
            "<geometry><box><size>2 1 3</size></box></geometry>",
            "<geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>",
        ),
        valid.replace(
            "<geometry><box><size>2 1 3</size></box></geometry>",
            "<geometry><mesh><uri>missing.obj</uri></mesh></geometry>",
        ),
    ] {
        assert!(
            load_world_with_engine(
                &text,
                "bad",
                "w",
                Duration::from_millis(1),
                &NoResources,
                PhysicsEngine::Tpe
            )
            .is_err()
        );
    }
    // The pinned SDF parser replaces any nonpositive Box size with its unit
    // default before Scene sees the DOM. Preserve the exact XML input as a
    // normalized positive, independently checking the resulting envelopes.
    let normalized = valid.replace("<size>2 1 3</size>", "<size>0 1 3</size>");
    assert_ne!(normalized, valid);
    let parsed = sdformat::load_str(&normalized, "normalized-box", &NoResources).unwrap();
    assert!(matches!(
        &parsed.worlds[0].models[0].links[0].collisions[0].geometry,
        sdformat::Geometry::Box { size } if *size == [1.; 3]
    ));
    let collision_q = rotation([0.2, -0.3, 0.4]);
    let mut points_local: Vec<_> = corners(DVec3::splat(-0.5), DVec3::splat(0.5))
        .into_iter()
        .map(|p| DVec3::new(0., 0.75, -0.25) + collision_q * p)
        .collect();
    points_local.extend(corners(
        DVec3::new(3.5, 0.25, -0.75),
        DVec3::new(4.5, 1.25, 0.25),
    ));
    let (local_lo, local_hi) = envelope(points_local);
    let link_q = rotation([0.3, 0.5, -0.2]);
    let link_origin = DVec3::new(1., -2., 0.5);
    let (lo, hi) = envelope(
        corners(local_lo, local_hi)
            .into_iter()
            .map(|p| link_origin + link_q * p),
    );
    let mut quiet = load(&normalized);
    let subject = named(&quiet, "w::subject");
    assert!(quiet.world().get::<Motion>(subject).is_some());
    assert!(quiet.world().get::<ModelBounds>(subject).is_some());
    assert!(
        quiet
            .world()
            .iter_entities()
            .all(|e| !e.contains::<gz_sim_scene::BodyHandle>())
    );
    quiet.step_exact(1).unwrap();
    assert!(points(&quiet).is_empty());
    let frame = quiet
        .world()
        .get::<gz_sim_scene::WorldPose>(named(&quiet, "w::subject::l"))
        .unwrap();
    vector(frame.translation, link_origin.to_array());
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        vector(
            (DQuat::from_array(frame.rotation) * axis).to_array(),
            (link_q * axis).to_array(),
        );
    }
    for (i, center) in [DVec3::new(0., 0.75, -0.25), DVec3::new(4., 0.75, -0.25)]
        .into_iter()
        .enumerate()
    {
        let frame = quiet
            .world()
            .get::<gz_sim_scene::WorldPose>(named(&quiet, &format!("w::subject::l::c{i}")))
            .unwrap();
        vector(
            frame.translation,
            (link_origin + link_q * center).to_array(),
        );
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            vector(
                (DQuat::from_array(frame.rotation) * axis).to_array(),
                (link_q * collision_q * axis).to_array(),
            );
        }
    }
    let middle = (lo + hi) * 0.5;
    for axis in 0..3 {
        for upper in [false, true] {
            for gap in [-0.01, 0.01] {
                let mut probe = middle;
                probe[axis] = if upper {
                    hi[axis] + 0.25 + gap
                } else {
                    lo[axis] - 0.25 - gap
                };
                // Only the independent static probe moves; subject XML stays
                // identical to the exact parser-normalized former refusal.
                let text = xml(2, 0, true, false, probe.to_array())
                    .replace("<size>2 1 3</size>", "<size>0 1 3</size>");
                let mut sim = load(&text);
                sim.step_exact(1).unwrap();
                let contacts = points(&sim);
                assert_eq!(contacts.len(), usize::from(gap < 0.));
                if gap < 0. {
                    vector(
                        contacts[0],
                        ((lo.max(probe - DVec3::splat(0.25)) + hi.min(probe + DVec3::splat(0.25)))
                            * 0.5)
                            .to_array(),
                    );
                }
            }
        }
    }
}
#[test]
fn tpe_legacy_single_centered_primitive_retains_exact_bounds_and_constant_velocity() {
    let text = "<sdf version='1.12'><world name='w'><model name='subject'><link name='l'><collision name='c'><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model><model name='probe'><static>true</static><pose>1.5 0 0 0 0 0</pose><link name='l'><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>";
    let mut sim = load(text);
    sim.step_exact(1).unwrap();
    assert_eq!(points(&sim), vec![[1., 0., 0.]]);
    let model = named(&sim, "w::subject");
    sim.world_mut()
        .get_mut::<Motion>(model)
        .unwrap()
        .set_world_velocity([-1., 0., 0.], [0.; 3])
        .unwrap();
    sim.step_exact(100).unwrap();
    vector(
        sim.world().get::<Motion>(model).unwrap().state().position,
        [-0.1, 0., 0.],
    );
    assert!(points(&sim).is_empty());
    let state = BodyState {
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    assert_eq!(*Motion::new(state).unwrap().state(), state);
}

#[test]
fn tpe_posed_bounds_angular_motion_matches_independent_outer_envelope() {
    let (lo, hi) = bounds(4, 0, true);
    let q = DQuat::from_rotation_z(0.1);
    let (world_lo, world_hi) = envelope(corners(lo, hi).into_iter().map(|p| q * p));
    let mut probe = (world_lo + world_hi) * 0.5;
    probe.x = world_hi.x + 0.24;
    let mut sim = load(&xml(4, 0, true, false, probe.to_array()));
    let model = named(&sim, "w::subject");
    sim.world_mut()
        .get_mut::<Motion>(model)
        .unwrap()
        .set_world_velocity([0.; 3], [0., 0., 1.])
        .unwrap();
    sim.step_exact(100).unwrap();
    let actual = glam::DQuat::from_array(
        sim.world()
            .get::<Motion>(model)
            .unwrap()
            .state()
            .orientation,
    );
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        vector((actual * axis).to_array(), (q * axis).to_array());
    }
    assert_eq!(points(&sim).len(), 1);
    vector(
        points(&sim)[0],
        ((world_lo.max(probe - DVec3::splat(0.25)) + world_hi.min(probe + DVec3::splat(0.25)))
            * 0.5)
            .to_array(),
    );
}
