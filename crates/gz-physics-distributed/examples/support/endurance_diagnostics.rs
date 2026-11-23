// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::Error,
    physics::{
        Body, BodyWrench, ContactImpulse, GroundSupport, NativeSolverProfile, Snapshot,
        potential_contact_groups_in_field,
    },
    pool::PoolFrame,
};
use serde_json::{Value, json};
use std::{fs, path::Path};

fn norm(v: [f64; 3]) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

pub fn comparison(a: &[&Body], b: &[&Body]) -> Value {
    let mut max_pos: f64 = 0.;
    let mut max_vel: f64 = 0.;
    let mut momentum_delta = [0.; 3];
    let mut potential_delta = 0.;
    let mut translational_ke_delta = 0.;
    for (a, b) in a.iter().zip(b) {
        let position = [0, 1, 2].map(|i| a.position[i] - b.position[i]);
        let velocity = [0, 1, 2].map(|i| a.linear[i] - b.linear[i]);
        max_pos = max_pos.max(norm(position));
        max_vel = max_vel.max(norm(velocity));
        for (p, v) in momentum_delta.iter_mut().zip(velocity) {
            *p += a.mass * v;
        }
        potential_delta += a.mass * 9.81 * position[2];
        translational_ke_delta += 0.5
            * a.mass
            * (0..3)
                .map(|i| (a.linear[i] - b.linear[i]) * (a.linear[i] + b.linear[i]))
                .sum::<f64>();
    }
    json!({"max_position_error_m":max_pos,"max_linear_velocity_error_m_s":max_vel,
        "total_linear_momentum_difference_kg_m_s":momentum_delta,
        "gravitational_potential_energy_difference_j":potential_delta,
        "translational_kinetic_energy_difference_j":translational_ke_delta})
}

pub fn admission(
    out: &Path,
    frame: &PoolFrame,
    wrenches: &[BodyWrench],
    operation: &str,
    profile: NativeSolverProfile,
    error: &Error,
) -> super::Result<()> {
    let mut s = Snapshot {
        tick: frame.tick,
        step: frame.step(),
        ground: frame.ground(),
        ..Default::default()
    };
    let mut components = vec![];
    for w in &frame.workers {
        s.bodies.extend(w.snapshot.bodies.iter().cloned());
        components.extend(w.components.iter().cloned());
    }
    s.bodies.sort_by_key(|b| b.id);
    let groups: Vec<Vec<u64>> =
        potential_contact_groups_in_field(&s, &components, &[], super::scene::GRAVITY, wrenches)?
            .iter()
            .map(|g| {
                let mut ids: Vec<_> = g
                    .iter()
                    .flat_map(|i| components[*i].ids.iter().copied())
                    .collect();
                ids.sort_unstable();
                ids
            })
            .collect();
    fs::write(
        out.join("admission.json"),
        serde_json::to_vec_pretty(&json!({
            "last_committed_tick":frame.tick,"operation":operation,"error":error.to_string(),
            "native_component_limit":profile.max_bodies(),"prospective_group_ids":groups,"wrenches":wrenches,"workers":frame.workers,
        }))?,
    )?;
    Ok(())
}

/// Replay a detached body's exact previous state at one and ten native solves.
/// Its contacts are included so a free-flight diagnosis cannot hide an impact.
pub fn isolated_solve_check(
    before: &[Body],
    frame: &PoolFrame,
    expected: &[&Body],
    wrenches: &[BodyWrench],
) -> super::Result<Vec<Value>> {
    let mut report = Vec::new();
    for b in before {
        let actual = frame.body(b.id).ok_or("missing body")?.1;
        let reference = expected
            .binary_search_by_key(&b.id, |b| b.id)
            .map(|i| expected[i])
            .map_err(|_| "missing oracle body")?;
        if actual == reference {
            continue;
        }
        let loads: Vec<_> = wrenches.iter().filter(|w| w.id == b.id).cloned().collect();
        let mut solves = Vec::new();
        for steps in [1, 10] {
            let mut isolated = super::native::Control::new(
                std::slice::from_ref(b),
                frame.ground(),
                super::scene::GRAVITY,
            )?;
            let (pairs, floors) = isolated.advance(steps, &loads, &[])?;
            let reconstructed = &isolated.bodies[0];
            solves.push(json!({"substeps":steps,"pairs":pairs,"floors":floors,
                "exact_distributed_body":reconstructed==actual,
                "exact_whole_scene_control":reconstructed==reference,
                "distributed_error":comparison(&[reconstructed],&[actual]),
                "whole_scene_error":comparison(&[reconstructed],&[reference])}));
        }
        report.push(json!({"id":b.id,"before":b,"isolated":solves}));
    }
    Ok(report)
}

/// Diagnostic-only localization limits for differences caused by floating point solver order.
/// The usual continuous and short-epoch witnesses always require byte equality.
#[derive(Default, serde::Serialize)]
pub struct Roundoff {
    pub first_tick: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    failure: Option<&'static str>,
    differing_ticks: u64,
    max_translation_m: f64,
    max_rotation_quaternion_chord: f64,
    max_linear_velocity_m_s: f64,
    max_angular_velocity_rad_s: f64,
    max_impulse_n_s: f64,
}
fn same_impulse_sign(a: f64, b: f64) -> bool {
    a.is_finite() && b.is_finite() && a >= 0. && b >= 0. && (a > 0.) == (b > 0.)
}
impl Roundoff {
    fn refuse(&mut self, reason: &'static str) -> bool {
        self.failure = Some(reason);
        false
    }
    #[allow(clippy::too_many_arguments)]
    pub fn observe(
        &mut self,
        tick: u64,
        actual: &[&Body],
        native: &[&Body],
        pairs: &[&ContactImpulse],
        npairs: &[ContactImpulse],
        floors: &[&GroundSupport],
        nfloors: &[GroundSupport],
    ) -> bool {
        self.first_tick.get_or_insert(tick);
        self.differing_ticks += 1;
        if actual.len() != native.len()
            || pairs.len() != npairs.len()
            || floors.len() != nfloors.len()
        {
            return self.refuse("body, pair or ground receipt count");
        }
        for (a, b) in actual.iter().zip(native) {
            if a.id != b.id
                || a.primitive != b.primitive
                || a.tier != b.tier
                || a.friction.to_bits() != b.friction.to_bits()
                || a.mass.to_bits() != b.mass.to_bits()
                || a.size.map(f64::to_bits) != b.size.map(f64::to_bits)
            {
                return self.refuse("immutable body definition or identity");
            }
            for body in [a, b] {
                if body
                    .position
                    .iter()
                    .chain(&body.orientation)
                    .chain(&body.linear)
                    .chain(&body.angular)
                    .any(|v| !v.is_finite())
                {
                    return self.refuse("nonfinite body state");
                }
            }
            let distance = |a: [f64; 3], b: [f64; 3]| norm([0, 1, 2].map(|i| a[i] - b[i]));
            self.max_translation_m = self.max_translation_m.max(distance(a.position, b.position));
            self.max_linear_velocity_m_s = self
                .max_linear_velocity_m_s
                .max(distance(a.linear, b.linear));
            self.max_angular_velocity_rad_s = self
                .max_angular_velocity_rad_s
                .max(distance(a.angular, b.angular));
            let chord = [1., -1.].map(|sign| {
                (0..4)
                    .map(|i| (a.orientation[i] - sign * b.orientation[i]).powi(2))
                    .sum::<f64>()
                    .sqrt()
            });
            self.max_rotation_quaternion_chord = self
                .max_rotation_quaternion_chord
                .max(chord[0].min(chord[1]));
        }
        for (a, b) in pairs.iter().zip(npairs) {
            if (a.a, a.b) != (b.a, b.b) || !same_impulse_sign(a.normal_impulse, b.normal_impulse) {
                return self.refuse("finite pair identity, impulse sign or nonfinite value");
            }
            self.max_impulse_n_s = self
                .max_impulse_n_s
                .max((a.normal_impulse - b.normal_impulse).abs());
        }
        for (a, b) in floors.iter().zip(nfloors) {
            if a.body != b.body || !same_impulse_sign(a.normal_impulse, b.normal_impulse) {
                return self.refuse("floor identity, impulse sign or nonfinite value");
            }
            self.max_impulse_n_s = self
                .max_impulse_n_s
                .max((a.normal_impulse - b.normal_impulse).abs());
        }
        self.failure = [
            (self.max_translation_m, 1e-8, "translation"),
            (self.max_rotation_quaternion_chord, 1e-8, "quaternion chord"),
            (self.max_linear_velocity_m_s, 1e-7, "linear velocity"),
            (self.max_angular_velocity_rad_s, 1e-7, "angular velocity"),
            (self.max_impulse_n_s, 1e-7, "normal impulse"),
        ]
        .into_iter()
        .find(|(value, limit, _)| value > limit)
        .map(|(_, _, name)| name);
        self.failure.is_none()
    }
}

/// Keep the prior compared geometry only when a long-run witness asks for
/// neighboring frames. Buffers are reused and written solely at a divergence.
#[derive(Default, serde::Serialize)]
pub struct Previous {
    tick: u64,
    held_wrenches: Vec<BodyWrench>,
    actual_bodies: Vec<Body>,
    reference_bodies: Vec<Body>,
    actual_pairs: Vec<ContactImpulse>,
    reference_pairs: Vec<ContactImpulse>,
    actual_floors: Vec<GroundSupport>,
    reference_floors: Vec<GroundSupport>,
}
impl Previous {
    pub fn remember(
        &mut self,
        tick: u64,
        frame: &PoolFrame,
        loads: &[BodyWrench],
        reference: &[&Body],
        pairs: &[ContactImpulse],
        floors: &[GroundSupport],
    ) {
        self.tick = tick;
        self.held_wrenches.clear();
        self.held_wrenches.extend_from_slice(loads);
        self.actual_bodies.clear();
        self.actual_pairs.clear();
        self.actual_floors.clear();
        for owner in &frame.workers {
            self.actual_bodies.extend_from_slice(&owner.snapshot.bodies);
            self.actual_pairs
                .extend_from_slice(&owner.snapshot.impulses);
            self.actual_floors
                .extend_from_slice(&owner.snapshot.supports);
        }
        self.actual_bodies.sort_by_key(|b| b.id);
        self.actual_pairs.sort_by_key(|p| (p.a, p.b));
        self.actual_floors.sort_by_key(|p| p.body);
        self.reference_bodies.clear();
        self.reference_bodies
            .extend(reference.iter().map(|b| (*b).clone()));
        self.reference_pairs.clear();
        self.reference_pairs.extend_from_slice(pairs);
        self.reference_floors.clear();
        self.reference_floors.extend_from_slice(floors);
    }
}
