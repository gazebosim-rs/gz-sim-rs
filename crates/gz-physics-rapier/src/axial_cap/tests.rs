// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::Shape;

fn fixture() -> (RapierBackend, BodyId, BodyId) {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    b.world.integration_parameters.dt = 0.001;
    let floor = b
        .spawn(BodySpec {
            shape: Shape::Box {
                half_extents: [0.5, 0.5, 0.025],
            },
            position: [0., 0., -0.025],
            velocity: [0.; 3],
            mass: None,
        })
        .unwrap();
    let cap = b
        .spawn(BodySpec {
            shape: Shape::Cylinder {
                half_height: 0.105,
                radius: 0.045,
            },
            position: [0., 0., 0.105],
            velocity: [0., 0., -1.6],
            mass: Some(0.25),
        })
        .unwrap();
    b.set_initial_restitution(cap, Restitution::new(0.65).unwrap())
        .unwrap();
    b.set_axial_cap_impact(true);
    (b, cap, floor)
}
fn admitted(b: &mut RapierBackend) -> bool {
    let hits = b.begin_axial_cap_impacts();
    b.end_axial_cap_impacts(&hits);
    !hits.is_empty()
}
fn peer(x: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Sphere { radius: 0.02 },
        position: [x, 0., 0.11],
        velocity: [0.; 3],
        mass: Some(0.1),
    }
}

#[test]
fn rebuilt_fixed_geometry_and_filtering_follow_actual_lifecycle() {
    let (mut b, cap, floor) = fixture();
    assert!(admitted(&mut b));
    let slab_state = b.state(floor).unwrap();
    let mut displaced = slab_state;
    displaced.position[0] = 3.;
    b.set_body_state(floor, displaced).unwrap();
    assert!(!admitted(&mut b));
    b.set_body_state(floor, slab_state).unwrap();
    assert!(admitted(&mut b));
    let plane = b
        .spawn_horizontal_halfspace(gz_physics::PlaneSpec {
            position: [0.; 3],
            orientation: [0., 0., 0., 1.],
            size: [3.; 2],
        })
        .unwrap();
    assert!(!admitted(&mut b));
    assert!(b.remove(plane) && admitted(&mut b));
    let triangle = b
        .spawn_mesh_link(
            MeshBodySpec {
                vertices: &[[-0.2, -0.2, 0.], [0.2, -0.2, 0.], [0., 0.2, 0.]],
                triangles: &[[0, 1, 2]],
                collision_position: [0.; 3],
                collision_orientation: [0., 0., 0., 1.],
                position: [0.; 3],
                velocity: [0.; 3],
                mass: None,
            },
            RotationSpec::default(),
            None,
        )
        .unwrap();
    assert!(!admitted(&mut b));
    assert!(b.remove(triangle) && admitted(&mut b));
    let inner = b.spawn(peer(0.)).unwrap();
    assert!(!admitted(&mut b));
    b.set_self_collision_group(cap, Some(2)).unwrap();
    b.set_self_collision_group(inner, Some(2)).unwrap();
    assert!(admitted(&mut b));
    b.set_axial_cap_impact(false);
    assert!(!admitted(&mut b));
    b.set_axial_cap_impact(true);
    b.reset().unwrap();
    assert!(admitted(&mut b));
    assert!(b.remove(cap));
    assert!(!admitted(&mut b));
}

#[test]
fn unconstrained_energy_and_single_collision_pass_are_required() {
    let (mut b, cap, _) = fixture();
    let h = b.bodies[&cap].0;
    b.world.bodies[h].add_torque(Vector::X, true);
    assert!(!admitted(&mut b));
    b.world.bodies[h].reset_torques(true);
    b.world.bodies[h].add_force(Vector::X, true);
    assert!(!admitted(&mut b));
    b.world.bodies[h].reset_forces(true);
    let c = b.world.bodies[h].colliders()[0];
    b.world.colliders[c].set_contact_skin(0.001);
    assert!(!admitted(&mut b));
    b.world.colliders[c].set_contact_skin(0.);
    b.world.bodies[h].sleep();
    assert!(!admitted(&mut b));
    b.world.bodies[h].set_linvel(-1.6 * Vector::Z, true);
    assert!(admitted(&mut b));
    b.world.integration_parameters.max_ccd_substeps = 2;
    assert!(!admitted(&mut b));
    b.world.integration_parameters.max_ccd_substeps = 1;
    b.world.bodies[h].enable_ccd(true);
    assert!(!admitted(&mut b));
}

#[test]
fn prescribed_driven_and_ccd_peers_must_be_filtered_or_outside_the_actual_margin() {
    let (mut b, cap, _) = fixture();
    let mut spec = peer(10.);
    spec.mass = None;
    let driven = b
        .spawn_velocity_boundary(spec, RotationSpec::default(), None)
        .unwrap();
    assert!(!admitted(&mut b));
    b.set_self_collision_group(cap, Some(3)).unwrap();
    b.set_self_collision_group(driven, Some(3)).unwrap();
    assert!(admitted(&mut b));
    assert!(b.remove(driven));
    let other = b.spawn(peer(0.3)).unwrap();
    let h = b.bodies[&other].0;
    b.world.bodies[h].set_soft_ccd_prediction(0.5);
    assert!(!admitted(&mut b));
    b.world.bodies[h].set_soft_ccd_prediction(0.);
    assert!(admitted(&mut b));
    b.world.bodies[h].enable_ccd(true);
    assert!(!admitted(&mut b));
    b.world.bodies[h].enable_ccd(false);
    let neighbor = b.spawn(peer(0.6)).unwrap();
    b.world
        .insert_impulse_joint(h, b.bodies[&neighbor].0, FixedJointBuilder::new());
    assert!(!admitted(&mut b));
}

#[test]
fn a_remote_hit_can_redirect_a_stationary_peer_into_the_striking_cap() {
    let (mut b, _, _) = fixture();
    let middle = b.spawn(peer(0.2)).unwrap();
    assert!(admitted(&mut b));
    let mut striker_spec = peer(0.239);
    striker_spec.velocity[0] = -70.;
    let striker = b.spawn(striker_spec).unwrap();
    assert!(!admitted(&mut b));
    b.step(Duration::from_millis(1)).unwrap();
    let [a, c] = [middle, striker].map(|id| b.world.bodies[b.bodies[&id].0].colliders()[0]);
    assert!(
        b.world
            .narrow_phase
            .contact_pair(a, c)
            .unwrap()
            .total_impulse_magnitude()
            > 1.
    );
    assert!(b.state(middle).unwrap().velocity[0] < -20.);
}
