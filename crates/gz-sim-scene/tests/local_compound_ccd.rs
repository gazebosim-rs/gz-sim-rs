// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
use sdformat::NoResources;

const COURSE: &str = include_str!("../../../examples/local_collision_course.sdf");
fn load(source: &str) -> Simulation {
    load_world_with_sdf_step(
        source,
        "collision-course.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[allow(deprecated)]
fn id(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}
fn assert_close(value: f64, expected: f64, tolerance: f64) {
    assert!(
        (value - expected).abs() <= tolerance,
        "{value} vs {expected} (tolerance {tolerance})"
    )
}
#[test]
fn rotated_and_offset_parts_have_real_surfaces_and_a_single_link_identity() {
    let sim = load(COURSE);
    let hit = id(&sim, "default::hit::carriage");
    let physics = sim.world().resource::<PhysicsBackend>();
    let side = [-0.90, -0.50, 0.15];
    // The line passes through the rotated box's center, reaching its local -Y face first.
    let distance = 0.5 - 0.04 / (std::f64::consts::FRAC_PI_6).cos();
    assert_close(
        physics
            .cast_ray(side, [0., 1., 0.], 0.55, None)
            .unwrap()
            .unwrap(),
        distance,
        1e-12,
    );
    assert!(
        physics
            .cast_ray(side, [0., 1., 0.], 0.55, Some(hit))
            .unwrap()
            .is_none()
    );
    let tip = [-0.25, 0., 0.15];
    assert_close(
        physics
            .cast_ray(tip, [-1., 0., 0.], 0.5, None)
            .unwrap()
            .unwrap(),
        0.39,
        1e-12,
    );
    assert!(
        physics
            .cast_ray(tip, [-1., 0., 0.], 0.5, Some(hit))
            .unwrap()
            .is_none()
    );
    // Ray exits through the real gap between parts, inside the compound's bounding box.
    assert!(
        physics
            .cast_ray([-0.78, 0., 0.4], [0., 0., -1.], 0.5, None)
            .unwrap()
            .is_none()
    );
}

#[test]
fn sweep_hits_a_thin_fence_and_leaves_an_adjacent_compound_clear_across_reset() {
    let mut sim = load(COURSE);
    for pass in 0..2 {
        if pass > 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let hit = id(&sim, "default::hit::carriage");
        let clear = id(&sim, "default::clear::carriage");
        let wall = id(&sim, "default::fence::rail");
        sim.step_exact(1).unwrap();
        let physics = sim.world().resource::<PhysicsBackend>();
        let x = physics.state(hit).unwrap().position[0];
        // The sphere center is 0.15 m ahead of the link, with radius 0.06 m.
        assert_close(x, -0.006 - 0.15 - 0.06, 0.000101);
        assert_close(physics.state(clear).unwrap().position[0], 0.15, 1e-10);
        sim.step_exact(1).unwrap();
        let physics = sim.world().resource::<PhysicsBackend>();
        let patches = physics
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_patches();
        let first = patches
            .iter()
            .find(|p| p.points.iter().any(|x| x.normal_impulse > 0.))
            .unwrap();
        assert_eq!((first.a, first.b), (hit.min(wall), hit.max(wall)));
        let sign = if first.a == hit { -1. } else { 1. };
        let tip_witness = physics
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_surface_witnesses()
            .into_iter()
            .flatten()
            .find(|w| w.body == hit)
            .unwrap();
        assert_eq!(tip_witness.subshape, 1);
        assert_close(
            tip_witness.world_point[0],
            physics.state(hit).unwrap().position[0] + 0.21,
            1e-8,
        );
        assert_close(first.normal_on_a[0], sign, 1e-6);
        assert!(patches.iter().all(|p| p.a != clear && p.b != clear));
        assert_close(physics.state(clear).unwrap().position[0], 1.15, 1e-10);
        assert!(physics.state(hit).unwrap().velocity[0].abs() < 1.);
    }
}

fn without_sweep() -> String {
    let start = COURSE
        .find("    <plugin filename=\"gz-sim-continuous-collision-system")
        .unwrap();
    let end = start + COURSE[start..].find("</plugin>").unwrap() + "</plugin>".len();
    format!("{}{}", &COURSE[..start], &COURSE[end..])
}

#[test]
fn unchecked_step_crosses_the_fence_and_compound_density_preserves_authored_inertia() {
    let mut sim = load(&without_sweep());
    let hit = id(&sim, "default::hit::carriage");
    sim.step_exact(1).unwrap();
    assert_close(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(hit)
            .unwrap()
            .position[0],
        0.15,
        1e-10,
    );
    // Apply an arbitrary COM impulse to the other independently moving link before its first tick.
    let mut sim = load(COURSE);
    let clear = id(&sim, "default::clear::carriage");
    let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
    physics
        .apply_impulse(clear, [0., 0., 2.], [0., 0.019, 0.])
        .unwrap();
    let state = physics.state(clear).unwrap();
    assert_close(state.velocity[2], 1., 1e-12);
    assert_close(state.angular_velocity[1], 1., 1e-12);
}

#[test]
fn oblique_contact_normal_and_location_follow_the_authored_surface() {
    let theta = 0.37_f64;
    let (s, c) = theta.sin_cos();
    let xml = format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity>
        <physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>
        <model name="fence"><static>true</static><link name="rail">
          <collision name="slant"><pose>0.2 -0.1 0 0 0 {theta}</pose><geometry><box><size>0.02 0.6 0.4</size></box></geometry></collision>
          <collision name="post"><pose>0.4 0.5 0 0 0 0</pose><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
        </link></model>
        <model name="ball"><pose>{} {} 0 0 0 0</pose><link name="mass"><collision name="sphere"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision></link>
        <plugin filename="gz-sim-initial-velocity-system" name="gz::sim::systems::InitialVelocity"><linear_velocity>{c} {s} 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin></model>
        </world></sdf>"#,
        0.2 - 0.09 * c,
        -0.1 - 0.09 * s
    );
    let mut sim = load(&xml);
    let ball = id(&sim, "default::ball::mass");
    let wall = id(&sim, "default::fence::rail");
    let ray_origin = [0.2 - 0.2 * c, -0.1 - 0.2 * s, 0.];
    let distance = sim
        .world()
        .resource::<PhysicsBackend>()
        .cast_ray(ray_origin, [c, s, 0.], 0.3, Some(ball))
        .unwrap()
        .unwrap();
    assert_close(distance, 0.19, 1e-12);
    for _ in 0..45 {
        sim.step_exact(1).unwrap();
        let engine = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        for patch in engine.rigid_contact_patches() {
            if patch.points.iter().any(|p| p.normal_impulse > 0.) {
                assert_eq!((patch.a, patch.b), (ball.min(wall), ball.max(wall)));
                let sign = if patch.a == wall { 1. } else { -1. };
                assert_close(sign * patch.normal_on_a[0], c, 1e-10);
                assert_close(sign * patch.normal_on_a[1], s, 1e-10);
                let surface = engine
                    .rigid_surface_witnesses()
                    .into_iter()
                    .flatten()
                    .find(|w| w.body == wall)
                    .expect("wall witness");
                assert_eq!(surface.subshape, 0);
                assert_close(surface.world_point[0], 0.2 - 0.01 * c, 1e-10);
                assert_close(surface.world_point[1], -0.1 - 0.01 * s, 1e-10);
                return;
            }
        }
    }
    panic!("no loaded oblique impact");
}

#[test]
fn invalid_sweeps_and_incompatible_compound_materials_are_rejected() {
    for xml in [
        COURSE.replace("default::clear::carriage</link>", "default::hit::carriage</link>"),
        COURSE.replace("default::clear::carriage</link>", "default::unknown::rail</link>"),
        COURSE.replace("0.0001</linear_slop>", "NaN</linear_slop>"),
        COURSE.replace(
            "<collision name=\"tip\">",
            "<collision name=\"tip\"><surface><friction><ode><mu>0.2</mu><mu2>0.2</mu2></ode></friction></surface>",
        ),
    ] {
        assert!(load_world_with_sdf_step(&xml, "invalid.sdf", "default", &NoResources, PhysicsEngine::MultiFidelity).is_err());
    }
}

#[test]
fn swept_link_configuration_rejects_ambiguous_or_incomplete_declarations() {
    let source = without_sweep();
    let link = "<link>default::hit::carriage</link>";
    let plugin = |body: &str| {
        format!(
            r#"<plugin filename="gz-sim-continuous-collision-system" name="gz::sim::systems::ContinuousCollision">{body}</plugin>"#
        )
    };
    for (case, settings) in [
        ("duplicate plugin", plugin(link).repeat(2)),
        (
            "empty link selection",
            plugin("<linear_slop>0.01</linear_slop>"),
        ),
        (
            "misspelled link selector",
            plugin("<links>default::hit::carriage</links>"),
        ),
        (
            "attribute on link",
            plugin("<link enabled='true'>default::hit::carriage</link>"),
        ),
        (
            "nested link name",
            plugin("<link><name>default::hit::carriage</name></link>"),
        ),
        (
            "duplicate tolerance",
            plugin(&format!(
                "{link}<linear_slop>0.01</linear_slop><linear_slop>0.02</linear_slop>"
            )),
        ),
        (
            "unparseable tolerance",
            plugin(&format!("{link}<linear_slop>tight</linear_slop>")),
        ),
    ] {
        let xml = source.replace("</world>", &format!("{settings}</world>"));
        let result = load_world_with_sdf_step(
            &xml,
            "collision-course.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        );
        assert!(
            matches!(result, Err(gz_sim_scene::LoadError::Unsupported { ref entity, feature })
                if entity == "default" && feature.contains("ContinuousCollision")),
            "{case} should report the offending collision configuration"
        );
    }
}

#[test]
fn reordered_sweep_links_preserve_independent_hit_and_clear_trajectories() {
    let selectors = "<link>default::hit::carriage</link><link>default::clear::carriage</link>";
    let reordered = COURSE.replace(
        selectors,
        "<link>default::clear::carriage</link><!-- independent link --> <link> default::hit::carriage </link>",
    );
    assert_ne!(reordered, COURSE);
    let mut baseline = load(COURSE);
    let mut actual = load(&reordered);
    for _ in 0..4 {
        baseline.step_exact(1).unwrap();
        actual.step_exact(1).unwrap();
        for name in ["default::hit::carriage", "default::clear::carriage"] {
            let sample = |sim: &Simulation| {
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(id(sim, name))
                    .unwrap()
            };
            let left = sample(&baseline);
            let right = sample(&actual);
            assert_eq!(left.position, right.position, "{name}");
            assert_eq!(left.velocity, right.velocity, "{name}");
            assert_eq!(left.orientation, right.orientation, "{name}");
            assert_eq!(left.angular_velocity, right.angular_velocity, "{name}");
        }
    }
}

#[test]
fn swept_compound_transfers_momentum_to_a_free_weld_without_tunneling() {
    // Give the moving fence two welded links. The thin rail alone makes
    // contact; the offset ballast receives momentum through its weld.
    let suffix = r#"<link name="ballast"><pose>0 0 0.45 0 0 0</pose>
      <inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      <collision name="body"><geometry><sphere><radius>0.04</radius></sphere></geometry></collision></link>
      <joint name="bracket" type="fixed"><parent>rail</parent><child>ballast</child></joint>
      </model></world>"#;
    let scene = COURSE.replace("</model>\n  </world>", suffix);
    let mut sim = load(&scene);
    for trial in 0..2 {
        if trial > 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let carriage = id(&sim, "default::hit::carriage");
        let rail = id(&sim, "default::fence::rail");
        let ballast = id(&sim, "default::fence::ballast");
        sim.step_exact(1).unwrap();
        let physics = sim.world().resource::<PhysicsBackend>();
        assert_close(
            physics.state(carriage).unwrap().position[0],
            -0.216,
            0.000101,
        );
        sim.step_exact(1).unwrap();
        let physics = sim.world().resource::<PhysicsBackend>();
        let v = |id| glam::DVec3::from(physics.state(id).unwrap().velocity);
        let reaction = 2. * (v(carriage) - 100. * glam::DVec3::X);
        let fence_reaction = 500. * v(rail) + 100. * v(ballast);
        assert!(reaction.x < -100. && (reaction + fence_reaction).length() < 1e-9);
        let native = physics.downcast_ref::<MultiFidelityBackend>().unwrap();
        let witnesses = native.rigid_surface_witnesses();
        assert!(witnesses.iter().any(|pair| {
            pair.iter().any(|w| w.body == carriage && w.subshape == 1)
                && pair.iter().any(|w| w.body == rail)
        }));
        assert!(witnesses.iter().flatten().all(|w| w.body != ballast));
    }
}

#[test]
fn continuous_collision_refuses_impulse_joint_children_and_parents() {
    let robot = include_str!("../../../examples/local_robot_parcel.sdf");
    for link in ["default::robot::carriage", "default::robot::tool"] {
        let plugin = format!(
            r#"<plugin name="gz::sim::systems::ContinuousCollision" filename="gz-sim-continuous-collision-system"><link>{link}</link></plugin></world>"#
        );
        let scene = robot.replace("</world>", &plugin);
        let result = load_world_with_sdf_step(
            &scene,
            "ccd-robot.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        );
        assert!(result.is_err_and(|e| {
            e.to_string()
                .contains("continuous collision detection on an independent dynamic link")
        }));
    }
}

#[test]
fn cylinder_endcaps_and_rounded_caps_follow_their_local_gazebo_axes() {
    let xml = r#"<sdf version="1.12"><world name="default"><model name="shapes"><static>true</static><link name="body">
        <collision name="drum"><pose>0 -0.5 0 0 1.5707963267948966 0</pose><geometry><cylinder><radius>0.05</radius><length>0.4</length></cylinder></geometry></collision>
        <collision name="pin"><pose>0 0.5 0 0 1.5707963267948966 0</pose><geometry><capsule><radius>0.05</radius><length>0.4</length></capsule></geometry></collision>
        </link></model></world></sdf>"#;
    let sim = load(xml);
    let physics = sim.world().resource::<PhysicsBackend>();
    for (y, distance) in [(-0.5, 0.8), (0.5, 0.75)] {
        assert_close(
            physics
                .cast_ray([-1., y, 0.], [1., 0., 0.], 1., None)
                .unwrap()
                .unwrap(),
            distance,
            1e-9,
        );
        assert_close(
            physics
                .cast_ray([0., y, 0.4], [0., 0., -1.], 0.5, None)
                .unwrap()
                .unwrap(),
            0.35,
            1e-9,
        );
    }
}

#[test]
fn specialized_contact_boundaries_still_require_a_centered_collider() {
    for (xml, collision) in [
        (
            include_str!("../../../examples/multifidelity_local_box_features.sdf"),
            "<collision name=\"sole\">",
        ),
        (
            include_str!("../../../examples/cuboid_soft_impact.sdf"),
            "<collision name=\"shell\">",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                xml,
                "centered.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_ok()
        );
        let offset = xml.replacen(
            collision,
            &format!("{collision}<pose>0.01 0 0 0 0 0</pose>"),
            1,
        );
        assert_ne!(xml, offset);
        assert!(
            load_world_with_sdf_step(
                &offset,
                "offset.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn lone_offset_box_is_not_treated_as_a_centered_primitive() {
    let source = r#"<sdf version="1.12"><world name="default"><model name="body"><link name="link">
      <collision name="offset"><pose>4 0 0 0 0 0</pose><geometry><box><size>0.6 0.2 0.2</size></box></geometry></collision>
    </link></model></world></sdf>"#;
    let sim = load(source);
    let physics = sim.world().resource::<PhysicsBackend>();
    assert!(
        physics
            .cast_ray([0., 0., -0.4], [0., 0., 1.], 0.8, None)
            .unwrap()
            .is_none()
    );
    assert_close(
        physics
            .cast_ray([4., 0., -0.4], [0., 0., 1.], 0.8, None)
            .unwrap()
            .unwrap(),
        0.3,
        1e-12,
    );
    // The specialized centered-box broadphase cannot use this offset shape.
    let native = physics.downcast_ref::<MultiFidelityBackend>().unwrap();
    assert!(
        native
            .soft_cuboid_inertia(id(&sim, "default::body::link"))
            .is_err()
    );
}
