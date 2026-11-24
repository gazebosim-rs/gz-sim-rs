// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::Result;
use glam::DVec3;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::grasp_guard::PadReading;
pub(crate) struct Tactile {
    pub(crate) pads: [PadReading; 2],
    pub(crate) station_up_ns: [f64; 3],
    pub(crate) station_ns: [f64; 3],
}

fn velocity_at(s: BodyState, lever: [f64; 3]) -> DVec3 {
    DVec3::from(s.velocity) + DVec3::from(s.angular_velocity).cross(lever.into())
}

pub(crate) fn tactile(
    sim: &Simulation,
    load: BodyId,
    jaws: [BodyId; 2],
    supports: [BodyId; 3],
) -> Result<Tactile> {
    let physics = sim.world().resource::<PhysicsBackend>();
    let solver = physics
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing local solver")?;
    let mut pads = [PadReading::default(); 2];
    let mut station_up_ns = [0.; 3];
    let mut station_ns = [0.; 3];
    for patch in solver.rigid_contact_patches() {
        let load_is_a = patch.a == load;
        let peer = if load_is_a {
            patch.b
        } else if patch.b == load {
            patch.a
        } else {
            continue;
        };
        if let Some(i) = supports.iter().position(|s| *s == peer) {
            let impulse = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
            let up = if load_is_a {
                patch.normal_on_a[2]
            } else {
                -patch.normal_on_a[2]
            };
            station_up_ns[i] += impulse * up;
            station_ns[i] += impulse;
            continue;
        }
        let Some(i) = jaws.iter().position(|p| *p == peer) else {
            continue;
        };
        let load_state = physics.state(load).ok_or("missing parcel state")?;
        let pad_state = physics.state(peer).ok_or("missing jaw state")?;
        let normal = DVec3::from(patch.normal_on_a);
        for point in &patch.points {
            let (load_r, pad_r) = if load_is_a {
                (point.lever_a, point.lever_b)
            } else {
                (point.lever_b, point.lever_a)
            };
            let relative = velocity_at(load_state, load_r) - velocity_at(pad_state, pad_r);
            let shear = (relative - normal * relative.dot(normal)).length();
            let force = point.normal_impulse / sim.step_duration().as_secs_f64();
            pads[i].slip_speed_m_s += shear * force;
            pads[i].normal_force_n += force;
        }
    }
    for pad in &mut pads {
        if pad.normal_force_n > 0. {
            pad.slip_speed_m_s /= pad.normal_force_n;
        }
    }
    Ok(Tactile {
        pads,
        station_ns,
        station_up_ns,
    })
}
