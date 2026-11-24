// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force};
use gz_physics_multifidelity::RigidPatchPoint;
use gz_sim_core::EntityId;
use std::time::Duration;

fn body(vz: f64, omega: [f64; 3]) -> BodyState {
    BodyState {
        // A world origin unrelated to the solver levers.
        position: [101., 42., -7.],
        velocity: [3., 2., vz],
        angular_velocity: omega,
        orientation: [0., 0., 0., 1.],
    }
}

fn point(a: [f64; 3], b: [f64; 3], impulse: f64) -> RigidPatchPoint {
    RigidPatchPoint {
        lever_a: a,
        lever_b: b,
        normal_impulse: impulse,
        tangent_impulse_on_a: None,
    }
}

fn patch(a: BodyId, b: BodyId, points: Vec<RigidPatchPoint>) -> RigidContactPatch {
    RigidContactPatch {
        a,
        b,
        normal_on_a: [1., 0., 0.],
        points,
    }
}

#[test]
fn weights_only_loaded_surface_points_and_accounts_for_rotation_on_each_side() {
    let [left, parcel, right, other] = [1, 2, 3, 4].map(BodyId::from_raw);
    let left_arm = [0.01, 0., 0.];
    let parcel_arm = [-0.02, 0., 0.];
    let patches = vec![
        patch(
            left,
            parcel,
            vec![
                point(left_arm, parcel_arm, 1.),
                point(left_arm, parcel_arm, 2.),
                point([f64::NAN; 3], parcel_arm, 0.),
            ],
        ),
        patch(
            parcel,
            right,
            vec![point([0.01, 0., 0.], [0., 0.01, 0.], 1.)],
        ),
        patch(left, other, vec![point(left_arm, parcel_arm, 100.)]),
        patch(parcel, other, vec![point(left_arm, parcel_arm, 100.)]),
    ];
    let measure = |ps, axis| {
        weighted_slip(
            ps,
            parcel,
            body(0.24, [0., 2., 0.]),
            [left, right],
            [body(0.2, [0., 1., 0.]), body(0.2, [1., 0., 0.])],
            axis,
        )
    };
    // Left pad sees +0.09 m/s; the right sees +0.01 m/s.
    assert!((measure(patches.clone(), DVec3::Z).unwrap() - 0.07).abs() < 1e-12);
    assert!((measure(patches.clone(), -DVec3::Z).unwrap() + 0.07).abs() < 1e-12);
    let mut one_side = patches;
    one_side[1].points[0].normal_impulse = 0.;
    assert_eq!(measure(one_side, DVec3::Z), None);
}

#[test]
fn rejects_unobservable_samples_and_removes_the_normal_velocity_component() {
    let [a, b, c] = [0, 1, 2].map(BodyId::from_raw);
    let both = [a, c].map(|pad| patch(b, pad, vec![point([0.; 3], [0.; 3], 1.)]));
    let measure = |ps| {
        weighted_slip(
            ps,
            b,
            body(2., [0.; 3]),
            [a, c],
            [body(0., [0.; 3]); 2],
            DVec3::Z,
        )
    };
    let mut vertical = both.to_vec();
    for p in &mut vertical {
        p.normal_on_a = [0., 0., -1.];
    }
    assert_eq!(measure(vertical), Some(0.));
    let mut tilted = both.to_vec();
    for p in &mut tilted {
        p.normal_on_a = DVec3::new(0., 1., 1.).normalize().to_array();
    }
    assert!((measure(tilted).unwrap() - 1.).abs() < 1e-12);
    let mut bad = both.to_vec();
    bad[1].points[0].normal_impulse = f64::NAN;
    assert_eq!(measure(bad), None);
    let mut bad = both.to_vec();
    bad[0].points[0].lever_b = [f64::NAN; 3];
    assert_eq!(measure(bad), None);
    assert_eq!(measure(vec![]), None);
}

#[test]
fn tactile_endpoint_speed_tracks_the_weakening_accelerating_grip() {
    let mut empty = Simulation::new(Duration::from_millis(1)).unwrap();
    let [a, b, c] = [0, 1, 2].map(BodyId::from_raw);
    let z = [0., 0., 2.];
    assert_eq!(bilateral_slip_velocity(&empty, [a, b], c, z), None);
    empty.world_mut().insert_resource(PhysicsBackend::from(
        gz_physics_rapier::RapierBackend::new([0.; 3]).unwrap(),
    ));
    assert_eq!(bilateral_slip_velocity(&empty, [a, b], c, z), None);
    let mut sim = load_world_with_sdf_step(
        include_str!("../../../../examples/worlds/local_offset_gripper_carry.sdf"),
        "carry.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let lookup = sim
        .world_mut()
        .query::<(&Name, &EntityId, Option<&BodyHandle>)>()
        .iter(sim.world())
        .map(|(n, id, b)| (n.0.clone(), (id.get(), b.map(BodyHandle::id))))
        .collect::<std::collections::BTreeMap<_, _>>();
    let pads = ["default::gripper::left", "default::gripper::right"].map(|n| lookup[n].1.unwrap());
    let parcel = lookup["default::parcel::body"].1.unwrap();
    let slides = [
        "default::gripper::left_slide",
        "default::gripper::right_slide",
    ]
    .map(|n| lookup[n].0);
    let read = |sim: &Simulation| bilateral_slip_velocity(sim, pads, parcel, z);
    let absent = BodyId::from_raw(u64::MAX);
    for missing in [[absent, pads[1]], [pads[0], absent]] {
        assert_eq!(bilateral_slip_velocity(&sim, missing, parcel, z), None);
    }
    assert_eq!(read(&sim), None);
    assert_eq!(bilateral_slip_velocity(&sim, pads, parcel, [0.; 3]), None);
    assert_eq!(bilateral_slip_velocity(&sim, [pads[0]; 2], parcel, z), None);
    assert_eq!(bilateral_slip_velocity(&sim, pads, pads[1], z), None);
    assert_eq!(
        bilateral_slip_velocity(&sim, pads, BodyId::from_raw(u64::MAX), z),
        None
    );
    for j in slides {
        set_joint_force(&mut sim, j, 4.2).unwrap();
    }
    sim.step_exact(750).unwrap();
    set_joint_force(&mut sim, lookup["default::gripper::lift"].0, 10.4956).unwrap();
    sim.step_exact(300).unwrap();
    assert!(read(&sim).unwrap().abs() < 1e-5);
    for j in slides {
        set_joint_force(&mut sim, j, 2.3).unwrap();
    }
    sim.step_exact(400).unwrap();
    let slip = read(&sim).unwrap();
    assert!((-0.26..-0.24).contains(&slip), "{slip}");
    assert!(
        (slip + bilateral_slip_velocity(&sim, pads, parcel, [0., 0., -1.]).unwrap()).abs() < 1e-12
    );

    // Inspect all 200 native solves in the next public tick to distinguish the
    // final material speed from the mean during continued downward acceleration.
    use bevy_ecs::prelude::{ResMut, Resource};
    use gz_sim_core::Phase;
    #[derive(Default, Resource)]
    struct NativeSpeeds(Vec<f64>);
    sim.world_mut().init_resource::<NativeSpeeds>();
    sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)
        .unwrap();
    sim.add_systems(
        Phase::PhysicsStep,
        move |mut engine: ResMut<PhysicsBackend>, mut speeds: ResMut<NativeSpeeds>| {
            engine
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .observe_contact_interval(Duration::from_micros(200), 200, 4, |sample| {
                    speeds.0.push(
                        weighted_slip(
                            sample.contact_patches(),
                            parcel,
                            sample.state(parcel).unwrap(),
                            pads,
                            pads.map(|p| sample.state(p).unwrap()),
                            DVec3::Z,
                        )
                        .unwrap(),
                    );
                })
                .unwrap();
        },
    );
    sim.step_exact(1).unwrap();
    let samples = &sim.world().resource::<NativeSpeeds>().0;
    assert_eq!(samples.len(), 200);
    let endpoint = read(&sim).unwrap();
    assert!(
        (samples.last().unwrap() - endpoint).abs() < 1e-10,
        "{endpoint} vs {:?}",
        samples.last()
    );
    assert!(samples.iter().sum::<f64>() / 200. - endpoint > 1e-4);
}
