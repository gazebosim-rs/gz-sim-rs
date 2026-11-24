// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cold, same-build primitive construction for launching an independent curve owner.
use super::*;
use gz_physics::trajectory::{PathEnd, Waypoint};
use serde::{Deserialize, Serialize};
const COLD: &[u8; 8] = b"GZCURC01";
const MAX_KNOTS: usize = 65536;

#[derive(Serialize, Deserialize)]
struct Row {
    id: u64,
    position: [f64; 3],
    velocity: [f64; 3],
    mass: Option<f64>,
    kinematic: bool,
    can_sleep: bool,
    orientation: [f64; 4],
    spin: [f64; 3],
    inertia: Option<([f64; 3], [f64; 4], [f64; 6])>,
    group: Option<u64>,
    collider: Collider,
}
impl RapierBackend {
    /// Immutable same-build seed containing rigid geometry, materials, inertias and authored paths.
    pub fn trajectory_construction(&self) -> Result<Vec<u8>, PhysicsError> {
        let seed = self.curve_recipe()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(COLD);
        bytes.extend_from_slice(&[0; 32]);
        bytes.extend_from_slice(&seed);
        let parameters = IntegrationParameters {
            dt: IntegrationParameters::default().dt,
            ..self.world.integration_parameters
        };
        append(
            &mut bytes,
            &(
                self.world.gravity,
                parameters,
                self.next_id,
                self.bodies.len() as u32,
            ),
        )?;
        for (&id, (_, s, r, m, g)) in &self.bodies {
            append(
                &mut bytes,
                &Row {
                    id: id.raw(),
                    position: s.position,
                    velocity: s.velocity,
                    mass: s.mass,
                    kinematic: s.kinematic,
                    can_sleep: s.can_sleep,
                    orientation: r.orientation,
                    spin: r.angular_velocity,
                    inertia: m.map(|v| (v.center, v.orientation, v.tensor)),
                    group: *g,
                    collider: s.collider.clone(),
                },
            )?;
            let path = self.curves.tracks.get(&id).map(|t| &t.path);
            let count = path.map_or(0, |p| p.waypoints().len());
            if count > MAX_KNOTS {
                return Err(invalid());
            }
            append(
                &mut bytes,
                &(
                    count as u32,
                    path.is_some_and(|p| p.end_policy() == PathEnd::Repeat),
                ),
            )?;
            if let Some(p) = path {
                for k in p.waypoints() {
                    append(&mut bytes, &(k.time, k.displacement, k.velocity))?;
                }
            }
        }
        let digest = Sha256::digest(&bytes[40..]);
        bytes[8..40].copy_from_slice(&digest);
        Ok(bytes)
    }
    /// Create a new owner from an integrity-checked construction published by a trusted peer.
    pub fn from_trajectory_construction(bytes: &[u8]) -> Result<Self, PhysicsError> {
        if !(72..=TRAJECTORY_IMAGE_LIMIT).contains(&bytes.len())
            || &bytes[..8] != COLD
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let mut input = &bytes[72..];
        let (gravity, parameters, next, count): (Vector, IntegrationParameters, u64, u32) =
            read(&mut input)?;
        if !(1..=BODIES as u32).contains(&count) || !gravity.is_finite() {
            return Err(invalid());
        }
        let mut b = Self::new_multifidelity(gravity.to_array())?;
        b.world.integration_parameters = parameters;
        b.fleet_clock = FleetClock::Coupled;
        b.fleet_dispatch = fleet_isolation::Dispatch::Disabled;
        let mut last = None;
        for _ in 0..count {
            let r: Row = read(&mut input)?;
            if last.is_some_and(|id| r.id <= id)
                || r.id >= next
                || !finite(r.position)
                || !finite(r.velocity)
                || !finite(r.spin)
                || r.mass.is_some_and(|m| !m.is_finite() || m <= 0.)
                || rotation(r.orientation).is_err()
            {
                return Err(invalid());
            }
            last = Some(r.id);
            let id = BodyId::from_raw(r.id);
            let (count, repeat): (u32, bool) = read(&mut input)?;
            if count as usize > MAX_KNOTS
                || (count == 0 && repeat)
                || count as usize > input.len() / 60
            {
                return Err(invalid());
            }
            let mut knots = Vec::with_capacity(count as usize);
            for _ in 0..count {
                let (time, displacement, velocity) = read(&mut input)?;
                knots.push(Waypoint {
                    time,
                    displacement,
                    velocity,
                });
            }
            let c = Construction {
                position: r.position,
                velocity: r.velocity,
                mass: r.mass,
                kinematic: r.kinematic,
                can_sleep: r.can_sleep,
                continuous_collision: false,
                articulation: None,
                collider: r.collider,
            };
            let rot = RotationSpec {
                orientation: r.orientation,
                angular_velocity: r.spin,
            };
            let inertia = r.inertia.map(|(center, orientation, tensor)| Inertia {
                center,
                orientation,
                tensor,
            });
            if let Some(m) = inertia {
                inertia_properties(m, r.mass.unwrap_or(1.))?;
            }
            let h = initial::insert_initial(&mut b.world, &c, rot, inertia);
            collision::assign(&mut b.world, h, r.group);
            b.bodies.insert(id, (h, c, rot, inertia, r.group));
            if count != 0 {
                let p = TranslationSpline::new(
                    &knots,
                    if repeat {
                        PathEnd::Repeat
                    } else {
                        PathEnd::Hold
                    },
                )
                .map_err(|_| invalid())?;
                b.set_initial_translation_path(id, p)?;
            }
        }
        b.next_id = next;
        b.refresh_max_boundary_spin();
        if !input.is_empty() || b.curve_recipe()?[..] != bytes[40..72] {
            return Err(invalid());
        }
        Ok(b)
    }
}

#[cfg(test)]
#[path = "../../../tests/private/trajectory_image.rs"]
mod tests;
