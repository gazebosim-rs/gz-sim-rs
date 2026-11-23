// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::{LoadError, PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;

const FIRST: &str = "<collision name='first'><pose>0.5 0 0 0 0 0</pose><density>7</density><geometry><sphere><radius>0.1</radius></sphere></geometry></collision>";
const SECOND: &str = "<collision name='second'><pose>-0.5 0 0 0 0 1.5707963267948966</pose><density>11</density><geometry><box><size>0.2 0.4 0.6</size></box></geometry></collision>";

fn xml(collisions: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='body'><link name='link'><pose>0 0 10 0 0 0</pose><inertial><mass>2</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>{collisions}</link></model></world></sdf>"
    )
}
fn load(source: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(source, "compound.sdf", "default", &NoResources, engine)
}

fn tpe_near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && expected.is_finite() && (actual - expected).abs() < 1e-11,
        "{actual} != {expected}"
    );
}

fn tpe_frames(sim: &Simulation, compound: bool) {
    let mut frames = vec![
        ("default::body", [0., 0., 0.], [0., 0., 0., 1.]),
        ("default::body::link", [0., 0., 10.], [0., 0., 0., 1.]),
        (
            "default::body::link::first",
            [0.5, 0., 10.],
            [0., 0., 0., 1.],
        ),
    ];
    if compound {
        frames.push((
            "default::body::link::second",
            [-0.5, 0., 10.],
            [
                0.,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        ));
    }
    for (name, translation, rotation) in frames {
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<gz_sim_scene::Name>().is_some_and(|n| n.0 == name))
            .unwrap();
        let pose = entity.get::<gz_sim_scene::WorldPose>().unwrap();
        for (actual, expected) in pose.translation.into_iter().zip(translation) {
            tpe_near(actual, expected);
        }
        for (actual, expected) in pose.rotation.into_iter().zip(rotation) {
            tpe_near(actual, expected);
        }
        if name == "default::body::link" {
            let velocity = entity.get::<gz_sim_scene::Velocity>().unwrap();
            assert_eq!(velocity.linear, [0.; 3]);
            assert_eq!(velocity.angular, [0.; 3]);
        }
    }
    let model = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == "default::body")
        })
        .unwrap();
    let motion = model.get::<gz_physics_tpe::Motion>().unwrap().state();
    assert_eq!(motion.position, [0.; 3]);
    assert_eq!(motion.orientation, [0., 0., 0., 1.]);
    assert_eq!(motion.velocity, [0.; 3]);
    assert_eq!(motion.angular_velocity, [0.; 3]);
    assert!(
        !model
            .get::<gz_physics_tpe::ModelBounds>()
            .unwrap()
            .is_static()
    );
    assert!(
        !sim.world()
            .iter_entities()
            .any(|e| e.contains::<gz_sim_scene::BodyHandle>())
    );
}

fn tpe_selected_bounds(source: &str, compound: bool) {
    // These are the original exact input frames: sphere center .5X/radius .1;
    // the second box's Z90 rotation exchanges its .1/.2 planar half extents.
    let (lo, hi): ([f64; 3], [f64; 3]) = if compound {
        ([-0.7, -0.1, 9.7], [0.6, 0.1, 10.3])
    } else {
        ([0.4, -0.1, 9.9], [0.6, 0.1, 10.1])
    };
    let mut quiet = load(source, PhysicsEngine::Tpe).unwrap();
    tpe_frames(&quiet, compound);
    quiet.step_exact(1).unwrap();
    tpe_frames(&quiet, compound);
    assert!(
        quiet
            .world()
            .resource::<gz_physics_tpe::Contacts>()
            .as_slice()
            .is_empty()
    );
    assert_eq!(
        quiet
            .world()
            .iter_entities()
            .filter(|e| e.contains::<gz_physics_tpe::Motion>()
                && e.contains::<gz_physics_tpe::ModelBounds>())
            .count(),
        1
    );
    // TPE exposes model-envelope overlap, not shape rays or collision response.
    // Witness both sides of every face with an independent static .5-size box.
    for (axis, (&lower, &higher)) in lo.iter().zip(&hi).enumerate() {
        for upper in [false, true] {
            for gap in [-0.01, 0.01] {
                let mut probe: [f64; 3] = std::array::from_fn(|i| (lo[i] + hi[i]) * 0.5);
                probe[axis] = if upper {
                    higher + 0.25 + gap
                } else {
                    lower - 0.25 - gap
                };
                let probe_xml = format!(
                    "<model name='bounds_probe'><static>true</static><pose>{} {} {} 0 0 0</pose><link name='l'><collision name='c'><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision></link></model></world>",
                    probe[0], probe[1], probe[2]
                );
                let with_probe = source.replace("</world>", &probe_xml);
                let mut sim = load(&with_probe, PhysicsEngine::Tpe).unwrap();
                sim.step_exact(1).unwrap();
                tpe_frames(&sim, compound);
                let contacts = sim
                    .world()
                    .resource::<gz_physics_tpe::Contacts>()
                    .as_slice();
                assert_eq!(
                    contacts.len(),
                    usize::from(gap < 0.),
                    "axis {axis}, upper {upper}, gap {gap}"
                );
                if gap < 0. {
                    for (((actual, lower), higher), center) in
                        contacts[0].point.into_iter().zip(lo).zip(hi).zip(probe)
                    {
                        let overlap_lo = lower.max(center - 0.25);
                        let overlap_hi = higher.min(center + 0.25);
                        tpe_near(actual, (overlap_lo + overlap_hi) * 0.5);
                    }
                }
            }
        }
    }
}

#[test]
fn other_engines_keep_single_centered_collision_domain() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for (case, source) in [xml(FIRST), xml(&format!("{FIRST}{SECOND}"))]
            .into_iter()
            .enumerate()
        {
            if matches!(engine, PhysicsEngine::Rapier) && case == 0 {
                // M82 admits only this single posed sphere. Its local .5X
                // offset moves geometry, while the reported link stays at Z10.
                let sim = load(&source, engine).unwrap();
                let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
                for (x, expected) in [(0.5, 0.9), (0.56, 0.92)] {
                    // Radius .1 and transverse .06 leave the .08 chord root.
                    let actual = backend
                        .cast_ray([x, 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap()
                        .unwrap();
                    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
                }
                for (name, expected) in [
                    ("default::body::link", [0., 0., 10.]),
                    ("default::body::link::first", [0.5, 0., 10.]),
                ] {
                    let entity = sim
                        .world()
                        .iter_entities()
                        .find(|entity| {
                            entity
                                .get::<gz_sim_scene::Name>()
                                .is_some_and(|n| n.0 == name)
                        })
                        .unwrap();
                    let pose = entity.get::<gz_sim_scene::WorldPose>().unwrap();
                    for (actual, expected) in pose.translation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                    assert_eq!(pose.rotation, [0., 0., 0., 1.]);
                }
            } else if matches!(engine, PhysicsEngine::Rapier) && case == 1 {
                // M84 keeps the same nonidentity link frame and both primitives.
                let sim = load(&source, engine).unwrap();
                let backend = sim.world().resource::<gz_physics::PhysicsBackend>();
                for (x, expected) in [(0.5, 0.9), (0.56, 0.92), (-0.5, 0.7)] {
                    let actual = backend
                        .cast_ray([x, 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap()
                        .unwrap();
                    assert!((actual - expected).abs() < 1e-11, "{actual} != {expected}");
                }
                assert_eq!(
                    backend
                        .cast_ray([0., 0., 11.], [0., 0., -1.], 2., None)
                        .unwrap(),
                    None
                );
                for (name, expected) in [
                    ("default::body", [0., 0., 0.]),
                    ("default::body::link", [0., 0., 10.]),
                    ("default::body::link::first", [0.5, 0., 10.]),
                    ("default::body::link::second", [-0.5, 0., 10.]),
                ] {
                    let entity = sim
                        .world()
                        .iter_entities()
                        .find(|entity| {
                            entity
                                .get::<gz_sim_scene::Name>()
                                .is_some_and(|n| n.0 == name)
                        })
                        .unwrap();
                    let pose = entity.get::<gz_sim_scene::WorldPose>().unwrap();
                    for (actual, expected) in pose.translation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                    let expected = if name.ends_with("::second") {
                        [
                            0.,
                            0.,
                            std::f64::consts::FRAC_1_SQRT_2,
                            std::f64::consts::FRAC_1_SQRT_2,
                        ]
                    } else {
                        [0., 0., 0., 1.]
                    };
                    for (actual, expected) in pose.rotation.into_iter().zip(expected) {
                        assert!((actual - expected).abs() < 1e-11);
                    }
                }
                assert_eq!(
                    sim.world()
                        .iter_entities()
                        .filter(|e| e.contains::<gz_sim_scene::BodyHandle>())
                        .count(),
                    1
                );
            } else {
                // M107 admits these same posed singleton/compound primitives.
                tpe_selected_bounds(&source, case == 1);
            }
        }
    }
}
