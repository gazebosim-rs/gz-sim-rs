// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Refine normal compression at a free material's first approach to an enrolled finite deck.
use super::*;
use gz_physics_xpbd::{ReceiverMotion, XpbdError};

/// Last advancing local material step. A trigger starts one millisecond of
/// compression refinement. A loaded articulated pad vetoes the request.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FiniteLandingStep {
    pub triggered: bool,
    pub refined: bool,
    pub material_substeps: u32,
}

pub fn finite_landing_step(sim: &Simulation) -> Option<FiniteLandingStep> {
    sim.world()
        .get_resource::<Deformables>()?
        .landing_refinement
        .as_ref()
        .map(|p| p.last)
}

pub(super) struct Policy {
    receiver: usize,
    surfaces: usize,
    target_ns: u64,
    remaining: Duration,
    last: FiniteLandingStep,
}
impl Policy {
    pub(super) fn admit(
        root: roxmltree::Node<'_, '_>,
        bodies: &BTreeMap<String, SoftBody>,
        surfaces: &[Surface],
        dt: Duration,
        exchanges: u32,
    ) -> Result<Option<Self>, String> {
        let Some(attr) = root.attribute("landing_step_ns") else {
            return Ok(None);
        };
        let target_ns: u64 = attr
            .parse()
            .map_err(|_| "landing_step_ns must be a positive integer")?;
        if target_ns == 0 || dt.as_nanos().div_ceil(u128::from(target_ns)) > 1024 {
            return Err(
                "landing_step_ns must resolve the material tick in 1 to 1024 substeps".into(),
            );
        }
        let deck = |s: &Surface| {
            matches!(
                s,
                Surface::Rigid {
                    one_way_box: Some(BoxMotion::Fixed(_) | BoxMotion::Translating(_)),
                    ..
                }
            )
        };
        let pad = |s: &Surface| {
            matches!(
                s,
                Surface::Rigid {
                    mobility: Some(ReceiverMass::ArticulatedWrench { .. }),
                    ..
                }
            )
        };
        let decks = surfaces
            .iter()
            .enumerate()
            .filter(|(_, s)| deck(s))
            .map(|(i, _)| i)
            .collect::<Vec<_>>();
        if bodies.len() != 1
            || exchanges != 1
            || root.attributes().any(|a| a.name().starts_with("worker_"))
            || decks.len() != 1
            || !surfaces.iter().any(pad)
            || !surfaces.iter().all(|s| deck(s) || pad(s))
        {
            return Err("landing refinement requires one local material, one finite fixed or translating deck, articulated pads and one rigid exchange".into());
        }
        Ok(Some(Self {
            receiver: decks[0],
            surfaces: surfaces.len(),
            target_ns,
            remaining: Duration::ZERO,
            last: Default::default(),
        }))
    }
    pub(super) fn reset(&mut self) {
        self.remaining = Duration::ZERO;
        self.last = Default::default();
    }

    pub(super) fn solve(
        &mut self,
        body: &mut SoftBody,
        dt: Duration,
        slots: &[usize],
        boundaries: &[Boundary],
        mobility: &[Option<RigidMobility>],
        damping: &[(usize, f64)],
    ) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
        let cfg = body.config();
        // If a receiver or pad has been retired, its compact reaction index may
        // have changed. Let the ordinary solver handle the remaining geometry.
        let enrolled = slots.len() == self.surfaces;
        let previous = body.last_boundary_impulses();
        // Old compact receipts across retirement are never used to qualify a new contact.
        let held = previous
            .iter()
            .enumerate()
            .any(|(i, j)| i != self.receiver && DVec3::from(j.linear).length_squared() > 1e-24);
        let ready = enrolled && !held;
        let unloaded = previous
            .get(self.receiver)
            .is_none_or(|j| DVec3::from(j.linear).length_squared() <= 1e-24);
        let triggered = ready
            && self.remaining.is_zero()
            && unloaded
            && approaching(body, boundaries[self.receiver], dt.as_secs_f64());
        let window = if triggered {
            Duration::from_millis(1)
        } else {
            self.remaining
        };
        let refined = ready && !window.is_zero();
        let count = if refined {
            (dt.as_nanos().div_ceil(u128::from(self.target_ns)) as u32).max(cfg.substeps)
        } else {
            cfg.substeps
        };
        if count != cfg.substeps {
            body.set_resolution(count, cfg.iterations)?;
        }
        let result = step_coupled_damped_pads(&mut [body], dt, boundaries, mobility, damping);
        if count != cfg.substeps {
            body.set_resolution(cfg.substeps, cfg.iterations)?;
        }
        let result = result?;
        self.remaining = if ready {
            window.saturating_sub(dt)
        } else {
            Duration::ZERO
        };
        self.last = FiniteLandingStep {
            triggered,
            refined,
            material_substeps: count,
        };
        Ok(result)
    }
}

// Predict only top-face entry. Bounds include the particle skin, so an
// off-edge graze does not activate a face-impact policy.
fn approaching(body: &SoftBody, deck: Boundary, dt: f64) -> bool {
    let (center, half, orientation, velocity) = match deck {
        Boundary::TranslatingCuboid {
            center,
            half_extents,
            orientation,
            velocity,
        } => (center, half_extents, orientation, velocity),
        Boundary::StaticCuboid {
            center,
            half_extents,
            orientation,
        } => (center, half_extents, orientation, [0.; 3]),
        _ => return false,
    };
    let local = DQuat::from_array(orientation).conjugate();
    let cfg = body.config();
    let g = local * DVec3::from(cfg.gravity);
    let radius = cfg.particle_radius;
    body.particles().iter().any(|p| {
        let x = local * (DVec3::from(p.position) - DVec3::from(center));
        let v = local * (DVec3::from(p.velocity) - DVec3::from(velocity));
        let end = x + dt * (v + dt * g);
        let surface = half[2] + radius;
        p.inverse_mass > 0.
            && v.z < -0.1
            && x.z >= surface - 1e-9
            && end.z <= surface + 1e-9
            && x.x.abs().max(end.x.abs()) + radius < half[0]
            && x.y.abs().max(end.y.abs()) + radius < half[1]
    })
}
