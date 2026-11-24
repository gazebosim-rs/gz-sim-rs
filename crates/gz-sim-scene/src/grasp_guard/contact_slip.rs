// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{MultiFidelityBackend, RigidContactPatch};
use gz_sim_core::Simulation;

/// Signed tangential speed of the parcel relative to both jaws along `up`
/// (m/s, positive upward). Read once after an advancing public physics step.
/// Each solved point contributes in proportion to its *positive* normal impulse.
/// A missing load on either pad gives no observation.
///
/// Samples the last native solve, including on refined ticks. Velocities are
/// evaluated at its frozen solver levers using endpoint COM velocities and
/// endpoint angular velocities. This endpoint reading cannot reconstruct
/// within-tick strikes, nor separate translation from local rocking. Intended
/// for responding, independently represented rigid links with COM-relative levers.
pub fn bilateral_slip_velocity(
    sim: &Simulation,
    pads: [BodyId; 2],
    parcel: BodyId,
    up: [f64; 3],
) -> Option<f64> {
    let up = DVec3::from(up).try_normalize()?;
    if pads[0] == pads[1] || pads.contains(&parcel) {
        return None;
    }
    let backend = sim.world().get_resource::<PhysicsBackend>()?;
    let native = backend.downcast_ref::<MultiFidelityBackend>()?;
    let state = |id| {
        // Static/prescribed sides can carry absolute solver anchors. A welded
        // member can carry a lever about the assembly COM instead of its own.
        if backend.is_dynamic(id) == Some(true) && !native.is_welded_link(id) {
            backend.state(id)
        } else {
            None
        }
    };
    let parcel_state = state(parcel)?;
    let pad_states = [state(pads[0])?, state(pads[1])?];
    weighted_slip(
        native.rigid_contact_patches(),
        parcel,
        parcel_state,
        pads,
        pad_states,
        up,
    )
}

fn point_speed(body: BodyState, arm: [f64; 3]) -> DVec3 {
    DVec3::from(body.velocity) + DVec3::from(body.angular_velocity).cross(arm.into())
}

fn weighted_slip(
    patches: Vec<RigidContactPatch>,
    parcel: BodyId,
    parcel_state: BodyState,
    pads: [BodyId; 2],
    pad_states: [BodyState; 2],
    up: DVec3,
) -> Option<f64> {
    let mut loads = [0.; 2];
    let mut weighted = 0.;
    for patch in patches {
        let (pad, parcel_first) = if patch.a == parcel {
            (patch.b, true)
        } else if patch.b == parcel {
            (patch.a, false)
        } else {
            continue;
        };
        let Some(i) = pads.iter().position(|&id| id == pad) else {
            continue;
        };
        let normal = DVec3::from(patch.normal_on_a);
        let tangent_up = up - up.dot(normal) * normal;
        for p in patch.points {
            if p.normal_impulse <= 0. {
                continue;
            }
            let (parcel_arm, pad_arm) = if parcel_first {
                (p.lever_a, p.lever_b)
            } else {
                (p.lever_b, p.lever_a)
            };
            let relative =
                point_speed(parcel_state, parcel_arm) - point_speed(pad_states[i], pad_arm);
            weighted += p.normal_impulse * relative.dot(tangent_up);
            loads[i] += p.normal_impulse;
        }
    }
    let total = loads[0] + loads[1];
    if !total.is_finite() || loads.contains(&0.) {
        return None;
    }
    let speed = weighted / total;
    speed.is_finite().then_some(speed)
}

#[cfg(test)]
#[path = "../../tests/private/contact_slip.rs"]
mod tests;
