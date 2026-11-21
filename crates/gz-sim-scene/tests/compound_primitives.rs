// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One physical link owns independent primitive child geometry and metadata.
use gz_physics::{BodyId, BodySpec, BodyState, PhysicsBackend, RotationSpec, Shape};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LoadError, LocalPose, ModelState, Name,
    PhysicsEngine, WorldEditError, WorldPose, load_world_with_sdf_step, model_state,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_state,
};
use sdformat::NoResources;
use std::collections::BTreeMap;
use std::time::Duration;

const FIXTURE: &str = include_str!("fixtures/m84_compound_primitives.sdf");
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
fn load(xml: &str, engine: PhysicsEngine) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "compound-primitives.sdf",
        "default",
        &NoResources,
        engine,
    )
}
fn world(models: &str, gravity: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='default'><gravity>{gravity}</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn collision(name: &str, pose: &str, geometry: &str) -> String {
    format!(
        "<collision name='{name}'><pose>{pose}</pose><density>7</density><geometry>{geometry}</geometry></collision>"
    )
}
fn children(count: usize) -> String {
    (0..count)
        .map(|i| {
            collision(
                &format!("c{i}"),
                &format!("{} 0 0 0 0 0", i * 3),
                "<sphere><radius>0.25</radius></sphere>",
            )
        })
        .collect()
}
fn simple(count: usize) -> String {
    world(
        &format!(
            "<model name='rig'><pose>0 0 5 0 0 0</pose><link name='body'><pose>0.5 0 0 0 0 0</pose>{INERTIA}{}<visual name='view'><geometry><box><size>0.1 0.1 0.1</size></box></geometry></visual></link></model>",
            children(count)
        ),
        "0 0 0",
    )
}
fn entity<'a>(sim: &'a Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    entity(sim, name).get::<EntityId>().unwrap().get()
}
fn body(sim: &Simulation, name: &str) -> BodyId {
    entity(sim, name).get::<BodyHandle>().unwrap().id()
}
fn pose(sim: &Simulation, name: &str) -> WorldPose {
    *entity(sim, name).get::<WorldPose>().unwrap()
}
fn state(sim: &Simulation, name: &str) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body(sim, name))
        .unwrap()
}
fn near(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        assert!(
            a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
            "{a} != {b}"
        );
    }
}
fn scalar(a: f64, b: f64) {
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn rotation(a: [f64; 4], b: [f64; 4]) {
    let dot: f64 = a.into_iter().zip(b).map(|(a, b)| a * b).sum();
    scalar(dot.abs(), 1.);
}
// Independent matrix/product arithmetic, separate from Scene pose composition.
fn rotate([x, y, z, w]: [f64; 4], [a, b, c]: [f64; 3]) -> [f64; 3] {
    [
        (1. - 2. * (y * y + z * z)) * a + 2. * (x * y - z * w) * b + 2. * (x * z + y * w) * c,
        2. * (x * y + z * w) * a + (1. - 2. * (x * x + z * z)) * b + 2. * (y * z - x * w) * c,
        2. * (x * z - y * w) * a + 2. * (y * z + x * w) * b + (1. - 2. * (x * x + y * y)) * c,
    ]
}
fn product([x, y, z, w]: [f64; 4], [a, b, c, d]: [f64; 4]) -> [f64; 4] {
    [
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}
fn ray(
    sim: &Simulation,
    p: [f64; 3],
    d: [f64; 3],
    max: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(p, d, max, exclude)
        .unwrap()
}
fn metadata(sim: &Simulation) -> BTreeMap<String, u64> {
    sim.world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.clone(), e.get::<EntityId>()?.get())))
        .collect()
}

#[test]
fn mixed_compounds_resolve_all_child_frames_and_independent_primitive_rays() {
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let model_q = [0., 0., h, h];
    let link_q = product(model_q, [0., h, 0., h]);
    let box_q = product(model_q, [h, 0., 0., h]);
    let cylinders_q = product(link_q, [h, 0., 0., h]);
    let capsule_q = product(link_q, [0., 0., h, h]);
    let sphere_q = product(link_q, [0., 0., (0.15f64).sin(), (0.15f64).cos()]);
    let nested = FIXTURE
        .replace(
            "<model name=\"rig\">",
            "<model name=\"outer\"><pose>2 0 0 0 0 0</pose><model name=\"rig\">",
        )
        .replace(
            "    <model name=\"survivor\">",
            "    </model><model name=\"survivor\">",
        );
    assert_ne!(nested, FIXTURE);
    for (xml, prefix, shift) in [
        (FIXTURE, "default::rig", 0.),
        (nested.as_str(), "default::outer::rig", 2.),
    ] {
        let sim = load(xml, PhysicsEngine::Rapier).unwrap();
        let link = format!("{prefix}::body");
        let link_position = [1. + shift, 2.5, 8.];
        near(pose(&sim, prefix).translation, [1. + shift, 2., 8.]);
        rotation(pose(&sim, prefix).rotation, model_q);
        near(pose(&sim, &link).translation, link_position);
        rotation(pose(&sim, &link).rotation, link_q);
        near(
            entity(&sim, &link)
                .get::<LocalPose>()
                .unwrap()
                .0
                .translation,
            [0.5, 0., 0.],
        );
        near(
            pose(&sim, &format!("{link}::view")).translation,
            add(link_position, rotate(link_q, [0., 0., 1.])),
        );
        near(
            pose(&sim, &format!("{link}::imu")).translation,
            link_position,
        );
        for (child, center, q, half, density) in [
            ("box", [shift, 4., 8.], box_q, [0.5, 0.25, 0.75], 7.),
            (
                "cylinder",
                add(link_position, rotate(link_q, [3., 0., 0.])),
                cylinders_q,
                [0.5, 0.5, 0.75],
                11.,
            ),
            (
                "capsule",
                add(link_position, rotate(link_q, [-3., 0., 0.])),
                capsule_q,
                [0.25, 0.25, 0.75],
                13.,
            ),
            (
                "sphere",
                add(link_position, rotate(link_q, [0., -3., 0.])),
                sphere_q,
                [0.5; 3],
                17.,
            ),
        ] {
            let name = format!("{link}::{child}");
            let node = entity(&sim, &name);
            near(pose(&sim, &name).translation, center);
            rotation(pose(&sim, &name).rotation, q);
            assert_eq!(node.get::<EntityKind>(), Some(&EntityKind::Collision));
            assert!(node.get::<BodyHandle>().is_none());
            assert_eq!(node.get::<CollisionDensity>().unwrap().0, density);
            for axis in 0..3 {
                let local: [f64; 3] =
                    std::array::from_fn(|i| if i == axis { half[i] + 1. } else { 0. });
                let direction =
                    rotate(q, std::array::from_fn(|i| if i == axis { -1. } else { 0. }));
                scalar(
                    ray(&sim, add(center, rotate(q, local)), direction, 1.5, None).unwrap(),
                    1.,
                );
                assert_eq!(
                    ray(
                        &sim,
                        add(center, rotate(q, local)),
                        direction,
                        1.5,
                        Some(body(&sim, &link))
                    ),
                    None
                );
            }
            assert_eq!(ray(&sim, center, [0., 0., 1.], 0.125, None), Some(0.));
        }
        // Curved off-axis cylinder/sphere roots use an independent .3/.4/.5 chord.
        for (center, q, local_origin, local_direction) in [
            (
                add(link_position, rotate(link_q, [3., 0., 0.])),
                cylinders_q,
                [0.3, 1., 0.],
                [0., -1., 0.],
            ),
            (
                add(link_position, rotate(link_q, [0., -3., 0.])),
                sphere_q,
                [0.3, 0., 1.],
                [0., 0., -1.],
            ),
        ] {
            let origin = add(center, rotate(q, local_origin));
            let direction = rotate(q, local_direction);
            scalar(ray(&sim, origin, direction, 1.5, None).unwrap(), 0.6);
        }
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<BodyHandle>())
                .count(),
            2
        );
        let ids = metadata(&sim);
        assert_eq!(
            ids.values()
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            ids.len()
        );
    }
}

#[test]
fn compound_count_bounds_and_resolved_defaults_keep_singleton_and_auto_domains_distinct() {
    for count in [1, 2, 16] {
        let sim = load(&simple(count), PhysicsEngine::Rapier).unwrap();
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Collision))
                .count(),
            count
        );
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<BodyHandle>())
                .count(),
            1
        );
        scalar(
            ray(&sim, [0.5, 0., 8.], [0., 0., -1.], 4., None).unwrap(),
            2.75,
        );
        assert_eq!(
            ray(&sim, [0.5, 0., 8.], [0., 0., -1.], 2.75, None),
            Some(2.75)
        );
        assert_eq!(
            ray(&sim, [0.5, 0., 8.], [0., 0., -1.], 2.75 - 1e-8, None),
            None
        );
    }
    for count in [0, 17] {
        assert!(load(&simple(count), PhysicsEngine::Rapier).is_err());
    }
    let defaulted = simple(2).replace(INERTIA, "");
    assert_ne!(defaulted, simple(2));
    let mut sim = load(&defaulted, PhysicsEngine::Rapier).unwrap();
    let link = id(&sim, "default::rig::body");
    queue_link_world_wrench(&mut sim, link, [1., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    near(state(&sim, "default::rig::body").velocity, [0.001, 0., 0.]);
    let automatic = simple(2).replace(INERTIA, "<inertial auto='true'/>");
    assert!(load(&automatic, PhysicsEngine::Rapier).is_err());
    // Pinned SDFormat normalizes nonpositive sphere radii to one, before Scene admission.
    let normalized = simple(2).replace("<radius>0.25</radius>", "<radius>0</radius>");
    let sim = load(&normalized, PhysicsEngine::Rapier).unwrap();
    scalar(
        ray(&sim, [0.5, 0., 8.], [0., 0., -1.], 4., None).unwrap(),
        2.,
    );
}

#[test]
fn compound_explicit_link_mass_and_inertia_ignore_child_density_under_real_inputs() {
    for xml in [
        simple(2),
        simple(2).replace("<density>7</density>", "<density>7000000</density>"),
    ] {
        let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
        let link = id(&sim, "default::rig::body");
        queue_link_world_wrench(&mut sim, link, [4., 0., 0.], [0., 0., 8.]).unwrap();
        sim.step_exact(1).unwrap();
        let actual = state(&sim, "default::rig::body");
        near(actual.velocity, [0.002, 0., 0.]);
        near(actual.angular_velocity, [0., 0., 0.002]);
        assert!(actual.position[0] > 0.5 && actual.position[0] < 0.50001);
        scalar(actual.position[1], 0.);
        scalar(actual.position[2], 5.);
        near(
            pose(&sim, "default::rig").translation,
            add(actual.position, rotate(actual.orientation, [-0.5, 0., 0.])),
        );
        for i in 0..2 {
            let child = format!("default::rig::body::c{i}");
            near(
                pose(&sim, &child).translation,
                add(
                    actual.position,
                    rotate(actual.orientation, [f64::from(i) * 3., 0., 0.]),
                ),
            );
        }
    }
}

#[test]
fn compound_ordinary_prepared_and_loaded_resets_preserve_geometry_ids_and_high_water() {
    let mut sim = load(FIXTURE, PhysicsEngine::Rapier).unwrap();
    let ids = metadata(&sim);
    let model = id(&sim, "default::rig");
    let link = id(&sim, "default::rig::body");
    let owned = body(&sim, "default::rig::body");
    let survivor = body(&sim, "default::survivor::body");
    queue_link_world_velocity(&mut sim, link, [1., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(4).unwrap();
    sim.reset().unwrap();
    near(pose(&sim, "default::rig::body").translation, [1., 2.5, 8.]);
    assert_eq!(sim.elapsed(), Duration::ZERO);
    let initial = sim.world().resource::<PhysicsBackend>().initial_state();
    let first_extra = {
        let mut b = sim.world_mut().resource_mut::<PhysicsBackend>();
        let extra = b
            .spawn_link(
                BodySpec {
                    shape: Shape::Sphere { radius: 0.25 },
                    position: [100., 0., 5.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
        assert!(b.remove(extra));
        extra
    };
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove(owned)
    );
    let prepared = sim
        .world()
        .resource::<PhysicsBackend>()
        .prepare_initial(&initial)
        .unwrap();
    assert_eq!(ray(&sim, [0., 4., 10.], [0., 0., -1.], 3., None), None);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .apply_initial(prepared)
        .unwrap();
    scalar(
        ray(&sim, [0., 4., 10.], [0., 0., -1.], 3., None).unwrap(),
        1.75,
    );
    assert_eq!(body(&sim, "default::rig::body"), owned);
    let removed = remove_model(&mut sim, model).unwrap();
    assert!(removed.contains(&model) && removed.contains(&link));
    for (name, public) in &ids {
        if name.starts_with("default::rig") {
            assert!(removed.contains(public));
        }
    }
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(owned)
            .is_none()
    );
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(survivor)
            .is_some()
    );
    scalar(
        ray(&sim, [20., 0., 3.], [0., 0., -1.], 4., None).unwrap(),
        2.5,
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(metadata(&sim), ids);
    assert_eq!(body(&sim, "default::rig::body"), owned);
    assert_eq!(body(&sim, "default::survivor::body"), survivor);
    scalar(
        ray(&sim, [0., 4., 10.], [0., 0., -1.], 3., None).unwrap(),
        1.75,
    );
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let next = backend
        .spawn_link(
            BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: [100., 0., 5.],
                velocity: [0.; 3],
                mass: Some(1.),
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(next.raw() > first_extra.raw());
    assert!(backend.remove(next));
}

#[test]
fn compound_cap_support_contacts_where_centered_countercontrol_misses() {
    let support = |x: f64| {
        format!(
            "<model name='ground'><static>true</static><link name='body'>{}{}</link></model>",
            collision(
                "support",
                &format!("{x} 0 0.25 0 0 0"),
                "<box><size>1 1 0.5</size></box>"
            ),
            collision(
                "remote",
                "-3 0 0.25 0 0 0",
                "<sphere><radius>0.25</radius></sphere>"
            )
        )
    };
    let ball = "<model name='ball'><pose>2 0 1 0 0 0</pose><link name='body'><collision name='shape'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>";
    for (x, supported) in [(2., true), (0., false)] {
        let mut sim = load(
            &world(&(support(x) + ball), "0 0 -9.81"),
            PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.step_exact(1200).unwrap();
        let z = state(&sim, "default::ball::body").position[2];
        if supported {
            assert!((z - 1.).abs() < 0.02, "{z}");
        } else {
            assert!(z < -1., "{z}");
        }
        near(pose(&sim, "default::ground::body").translation, [0.; 3]);
    }
}

#[test]
fn compound_links_keep_passive_model_self_exclusion_and_distinct_contacts() {
    let link = |name: &str, x: f64| {
        format!(
            "<link name='{name}'><pose>{x} 0 5 0 0 0</pose>{INERTIA}{}{}</link>",
            collision(
                "main",
                "0 0 0 0 0 0",
                "<sphere><radius>0.5</radius></sphere>"
            ),
            collision(
                "remote",
                "3 0 0 0 0 0",
                "<box><size>0.5 0.5 0.5</size></box>"
            )
        )
    };
    let a = link("a", 0.);
    let b = link("b", 0.6);
    for (xml, names, same) in [
        (
            world(&format!("<model name='same'>{a}{b}</model>"), "0 0 0"),
            ["default::same::a", "default::same::b"],
            true,
        ),
        (
            world(
                &format!("<model name='one'>{a}</model><model name='two'>{b}</model>"),
                "0 0 0",
            ),
            ["default::one::a", "default::two::b"],
            false,
        ),
    ] {
        let mut sim = load(&xml, PhysicsEngine::Rapier).unwrap();
        // Use the common 1.5 s compound-contact window before and after reset.
        sim.step_exact(1500).unwrap();
        let x = state(&sim, names[0]).position[0];
        let y = state(&sim, names[1]).position[0];
        if same {
            scalar(x, 0.);
            scalar(y, 0.6);
        } else {
            assert!(x < -0.1 && y > 0.7, "{x} {y}");
        }
        sim.reset().unwrap();
        scalar(state(&sim, names[0]).position[0], 0.);
        scalar(state(&sim, names[1]).position[0], 0.6);
        sim.step_exact(1500).unwrap();
        let x = state(&sim, names[0]).position[0];
        let y = state(&sim, names[1]).position[0];
        if same {
            scalar(x, 0.);
            scalar(y, 0.6);
        } else {
            assert!(x < -0.1 && y > 0.7);
        }
    }
}

#[test]
fn compound_late_invalid_children_preserve_other_simulations_and_reconstructed_ids() {
    let valid = simple(2);
    let existing = load(&valid, PhysicsEngine::Rapier).unwrap();
    let before = state(&existing, "default::rig::body");
    let ids = metadata(&existing);
    let last = "<collision name='c1'><pose>3 0 0 0 0 0</pose><density>7</density><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>";
    assert_eq!(valid.matches(last).count(), 1);
    for replacement in [
        last.replace("0.25", "1e308"),
        last.replace("3 0 0 0 0 0", "1e308 0 0 0 0 0"),
        last.replace("0.25", "1e-308"),
    ] {
        let invalid = valid.replace(last, &replacement);
        assert_ne!(invalid, valid);
        assert!(load(&invalid, PhysicsEngine::Rapier).is_err());
        let actual = state(&existing, "default::rig::body");
        near(actual.position, before.position);
        near(actual.velocity, before.velocity);
        near(actual.angular_velocity, before.angular_velocity);
        rotation(actual.orientation, before.orientation);
        assert_eq!(metadata(&existing), ids);
        assert_eq!(existing.elapsed(), Duration::ZERO);
        let rebuilt = load(&valid, PhysicsEngine::Rapier).unwrap();
        assert_eq!(metadata(&rebuilt), ids);
        assert_eq!(
            body(&rebuilt, "default::rig::body"),
            body(&existing, "default::rig::body")
        );
    }
}

#[test]
fn compound_refusals_keep_tpe_mesh_plane_com_surfaces_joints_and_owners_closed() {
    let valid = simple(2);
    assert!(load(&valid, PhysicsEngine::Tpe).is_err());
    for (source,feature) in [
        (valid.replacen("<sphere><radius>0.25</radius></sphere>","<plane><normal>0 0 1</normal><size>2 2</size></plane>",1),"compound collision geometry"),
        (valid.replacen("<sphere><radius>0.25</radius></sphere>","<mesh><uri>missing-compound-mesh.gltf</uri></mesh>",1),"compound collision geometry"),
        (valid.replace("<inertial>","<inertial><pose>0.1 0 0 0 0 0</pose>"),"offset center of mass"),
        (valid.replacen("</collision>","<surface><friction><ode><mu>0.3</mu></ode></friction></surface></collision>",1),"collision surface parameters"),
        (valid.replace("</model>","<joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model>"),"compound model joints"),
    ] {
        assert_ne!(source,valid);assert!(matches!(load(&source,PhysicsEngine::Rapier),Err(LoadError::Unsupported{feature:actual,..}) if actual==feature),"{feature}");
    }
    let owner_domain = valid.replace("<pose>0.5 0 0 0 0 0</pose>", "<pose>0 0 0 0 0 0</pose>");
    assert_ne!(owner_domain, valid);
    assert!(load(&owner_domain, PhysicsEngine::Rapier).is_ok());
    let controlled=owner_domain.replace("<model name='rig'>","<model name='rig'><plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>");
    assert_ne!(controlled, owner_domain);
    assert!(load(&controlled, PhysicsEngine::Rapier).is_err());
    let singleton = owner_domain.replace(&children(2), &children(1));
    assert_ne!(singleton, owner_domain);
    let control = load(&singleton, PhysicsEngine::Rapier).unwrap();
    assert!(model_state(&control, id(&control, "default::rig")).is_ok());
    let mut sim = load(&owner_domain, PhysicsEngine::Rapier).unwrap();
    let owner = id(&sim, "default::rig");
    let before = state(&sim, "default::rig::body");
    assert!(matches!(
        model_state(&sim, owner),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert!(matches!(
        set_model_state(
            &mut sim,
            owner,
            ModelState {
                translation: before.position,
                rotation: before.orientation,
                linear_velocity: before.velocity,
                angular_velocity: before.angular_velocity
            }
        ),
        Err(WorldEditError::UnsupportedModel)
    ));
    near(state(&sim, "default::rig::body").position, before.position);
}
