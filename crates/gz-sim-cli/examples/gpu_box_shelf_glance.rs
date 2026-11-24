// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate the second glancing strike from a frozen, matched incoming cuboid state.
#[path = "gpu_box_shelf_glance/corners.rs"]
mod corners;
#[path = "gpu_box_shelf_glance/reload.rs"]
mod reload;
#[path = "gpu_box_shelf_glance/sliding.rs"]
mod sliding;
use rapier3d_f64::prelude::*;
use serde::Deserialize;
use serde_json::{Value, json};
use std::collections::BTreeMap;

const INPUT: &str =
    include_str!("../../../docs/measurements/local-manipulation/gpu-shelf-glance-state.json");
const MASS: f64 = 0.08;
const DT: f64 = 0.001;
const OFFSET: Vector = Vector::new(0.005, 0., -0.002);
const MOMENTS: Vector = Vector::new(0.00001450666666666667, 0.00002602666666666667, 0.0000192);

#[derive(Deserialize)]
struct State {
    position: [f64; 3],
    orientation: [f64; 4],
    center_velocity: [f64; 3],
    omega: [f64; 3],
}
#[derive(Deserialize)]
struct Pair {
    incoming: State,
    continuous_outgoing: State,
}
#[derive(Deserialize)]
struct Inputs {
    states: BTreeMap<String, Pair>,
}
struct Native {
    world: PhysicsWorld,
    body: RigidBodyHandle,
    support: ColliderHandle,
    shape: ColliderHandle,
}
fn native(state: &State, steps: usize, reverse: bool) -> Native {
    let mut world = PhysicsWorld::new();
    world.gravity = -9.81 * Vector::Z;
    let p = &mut world.integration_parameters;
    p.dt = DT / steps as f64;
    p.normalized_allowed_linear_error = 0.;
    p.normalized_prediction_distance = 0.;
    p.normalized_contact_recycle_distance = 0.;
    p.num_solver_iterations = 32;
    p.num_internal_pgs_iterations = 8;
    p.num_internal_stabilization_iterations = 0;
    p.friction_model = FrictionModel::Coulomb;
    p.static_contact_softness.natural_frequency = 1e6;
    p.static_contact_softness.damping_ratio = 1.;
    p.normalized_max_corrective_velocity = 0.;
    let material = |c: ColliderBuilder, mu| {
        c.friction(mu)
            .friction_combine_rule(CoefficientCombineRule::Min)
            .restitution(0.)
    };
    let mut slab = material(ColliderBuilder::cuboid(0.06, 0.105, 0.007), 0.45).build();
    let orientation = if reverse {
        Rotation::from_xyzw(0., 0., 1., 0.)
    } else {
        Rotation::IDENTITY
    };
    slab.set_position(Pose::from_parts(Vector::new(0.18, 0., 0.007), orientation));
    let support = world.insert_collider(slab, None);
    let q = Rotation::from_array(state.orientation);
    let body = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(state.position.into())
            .rotation(q.to_scaled_axis())
            .can_sleep(false),
    );
    let mut box_shape = material(
        ColliderBuilder::cuboid(0.024, 0.012, 0.020)
            .mass_properties(MassProperties::new(OFFSET, MASS, MOMENTS)),
        1.,
    )
    .build();
    box_shape.set_position(Pose::from_parts(Vector::ZERO, Rotation::IDENTITY));
    let shape = world.insert_collider(box_shape, Some(body));
    let initial_velocity = Vector::from_array(state.center_velocity);
    world.bodies[body].set_linvel(initial_velocity, true);
    world.bodies[body].set_angvel(state.omega.into(), true);
    Native {
        world,
        body,
        support,
        shape,
    }
}
impl Native {
    /// Evaluate each solver row on the box, with the impulse oriented from shelf to box.
    fn contacts(
        &self,
        mut visit: impl FnMut(Vector, Vector, Vector, f64, &rapier3d_f64::geometry::ContactData),
    ) {
        if let Some(pair) = self
            .world
            .narrow_phase
            .contact_pair(self.support, self.shape)
        {
            let second = pair.collider2 == self.shape;
            let sign = if second { 1. } else { -1. };
            for m in pair.solver_manifolds() {
                for row in &m.data.solver_contacts {
                    let point = &m.points[row.contact_indices()[0] as usize];
                    let (local, arm) = if second {
                        (point.local_p2, point.data.solver_dp2)
                    } else {
                        (point.local_p1, point.data.solver_dp1)
                    };
                    visit(local, arm, m.data.normal, sign, &point.data);
                }
            }
        }
    }
}
fn solve(
    state: &State,
    steps: usize,
    reverse: bool,
    mut audit: Option<&mut corners::Witnesses>,
) -> Value {
    let mut native = native(state, steps, reverse);
    let initial_velocity = Vector::from_array(state.center_velocity);
    let q = Rotation::from_array(state.orientation);
    let mut first = None;
    let mut upward = 0.;
    for substep in 0..steps {
        if let Some(a) = audit.as_deref_mut() {
            let b = &native.world.bodies[native.body];
            a.incoming = (*b.rotation(), b.linvel(), b.angvel());
        }
        native.world.step();
        native.contacts(|local, arm, normal, sign, data| {
            if let Some(a) = audit.as_deref_mut() {
                a.record(substep, local, arm, normal, sign, data);
            }
            assert!(data.impulse >= 0.);
            if data.impulse > 1e-9 {
                first.get_or_insert(substep);
            }
            upward += sign * normal.z * data.impulse;
        });
    }
    let b = &native.world.bodies[native.body];
    let momentum = |q: Rotation, w: Vector| q * (MOMENTS * (q.inverse() * w));
    let linear = MASS * (b.linvel() - initial_velocity + DT * 9.81 * Vector::Z);
    let angular = momentum(*b.rotation(), b.angvel()) - momentum(q, state.omega.into());
    assert!(upward > 0.03 && (upward - linear.z).abs() < 1e-12);
    json!({"substeps_per_ms":steps,"half_turn":reverse,"first_loaded_substep_zero_based":first.unwrap(),"outgoing_omega_rad_s":b.angvel().to_array(),"outgoing_com_velocity_m_s":b.linvel().to_array(),"linear_impulse_ns":linear.to_array(),"angular_impulse_nms":angular.to_array(),"identified_upward_ns":upward})
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args().nth(1).as_deref() == Some("--reload") {
        return reload::run();
    }
    if std::env::args().nth(1).as_deref() == Some("--corners") {
        return corners::run();
    }
    let inputs: Inputs = serde_json::from_str(INPUT)?;
    let mut cases = BTreeMap::new();
    for (name, pair) in inputs.states {
        let mut results: Vec<_> = [8, 32, 64, 128, 256, 512, 1024]
            .map(|n| solve(&pair.incoming, n, false, None))
            .into();
        results.push(solve(&pair.incoming, 32, true, None));
        cases.insert(name,json!({"continuous_outgoing_yaw_speed_rad_s":pair.continuous_outgoing.omega[2],"cold_matched_interval":results}));
    }
    println!("{}", serde_json::to_string_pretty(&cases)?);
    Ok(())
}
