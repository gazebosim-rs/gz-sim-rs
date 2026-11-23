// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One measured native mobility and a single generalized commit per cold edge.
use super::*;
use gz_physics::{
    MixedEdgeImpact, MixedEdgeResponse, NormalSiteState, OpeningEdgeImpact, OpeningEdgeResponse,
    SlidingEdgeImpact, SlidingEdgeResponse, TwoNormalContact, TwoNormalResponse,
};

/// One site supplied the entire compressive reaction. The second site
/// shares the initial tangency and has positive relative speed after impact.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpeningEdgeImpactReceipt {
    pub loaded_site: usize,
    pub loaded: CornerImpactReceipt,
    pub unloaded_point: [f64; 3],
    pub unloaded_gap: f64,
    pub before_unloaded_relative_velocity: [f64; 3],
    pub after_unloaded_relative_velocity: [f64; 3],
    pub separating_margin: [f64; 2],
}
/// Frictionless plastic impact at a cold two-site return. The previous bearing
/// was stationary along the normal. Its instantaneous load may vanish while
/// the arriving site is compressed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrictionlessRecontactReceipt {
    pub sites: [CornerImpactReceipt; 2],
    pub stationary_site: usize,
    pub state: [NormalSiteState; 2],
    pub kinetic_energy_change: f64,
    pub dissipated_energy: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EdgeImpactReceipt {
    BothSlide(SlidingEdgeImpactReceipt),
    Sticking(MixedEdgeImpactReceipt),
    OneLoaded(OpeningEdgeImpactReceipt),
    Recontact(FrictionlessRecontactReceipt),
}
#[derive(Clone, Copy, PartialEq, Eq)]
pub(in super::super) enum EdgeMode {
    Any,
    Sliding,
    Mixed,
}

enum Law {
    Sliding(SlidingEdgeResponse),
    Mixed(MixedEdgeResponse),
    Opening(OpeningEdgeResponse),
    NormalReturn(usize, TwoNormalResponse),
}
struct Prepared {
    law: Law,
    basis: [Vector; 3],
    before: [Vector; 2],
}
fn checked<T>(result: Result<T, PhysicsError>) -> Result<Option<T>, PhysicsError> {
    match result {
        Ok(s) => Ok(Some(s)),
        Err(PhysicsError::Unsupported { .. } | PhysicsError::InvalidValue) => Ok(None),
        Err(e) => Err(e),
    }
}
impl EdgeSites {
    fn solve(
        &self,
        sample: &ArticulatedContactSample,
        dv: Vector,
        mode: EdgeMode,
    ) -> Result<Option<Prepared>, PhysicsError> {
        let before = self.incoming(sample, dv);
        let n = self.basis[2];
        let basis = if mode == EdgeMode::Sliding {
            if before
                .iter()
                .any(|u| u.dot(n) >= -128. * f64::EPSILON * (1. + u.length()))
            {
                return Ok(None);
            }
            self.basis
        } else {
            let d = self.world[1] - self.world[0];
            let Some(e) = (d - n.dot(d) * n).try_normalize() else {
                return Ok(None);
            };
            [e, n.cross(e), n]
        };
        let inverse_mass = Self::mobility(sample, basis)?;
        let relative_velocity = before.map(|u| basis.map(|b| b.dot(u)));
        let friction = [self.boxes.friction; 2];
        let prepare = |law| Some(Prepared { law, basis, before });
        if mode == EdgeMode::Any && self.boxes.friction == 0. {
            let u = before.map(|v| v.dot(n));
            let eps = before.map(|v| 512. * f64::EPSILON * (1. + v.length()));
            if let Some(old) = (0..2).find(|&i| u[i].abs() <= eps[i] && u[1 - i] < -eps[1 - i]) {
                let response = TwoNormalContact {
                    inverse_mass: inverse_mass.map(|row| row.map(|block| block[2][2])),
                    free_normal: u,
                }
                .solve();
                return Ok(checked(response)?.and_then(|s| {
                    (s.state[1 - old] == NormalSiteState::Loaded
                        && !s.state.contains(&NormalSiteState::Neutral))
                    .then_some(Prepared {
                        law: Law::NormalReturn(old, s),
                        basis,
                        before,
                    })
                }));
            }
            if (0..2).all(|i| u[i] >= -eps[i]) {
                return Ok(None);
            }
        }
        if mode == EdgeMode::Any
            && let Some(s) = checked(
                OpeningEdgeImpact {
                    inverse_mass,
                    relative_velocity,
                    friction,
                }
                .solve(),
            )?
        {
            return Ok(prepare(Law::Opening(s)));
        }
        if mode != EdgeMode::Sliding
            && let Some(s) = checked(
                MixedEdgeImpact {
                    inverse_mass,
                    relative_velocity,
                    friction,
                }
                .solve(),
            )?
        {
            return Ok(prepare(Law::Mixed(s)));
        }
        if mode != EdgeMode::Mixed
            && let Some(s) = checked(
                SlidingEdgeImpact {
                    inverse_mass,
                    relative_velocity,
                    friction,
                }
                .solve(),
            )?
        {
            return Ok(prepare(Law::Sliding(s)));
        }
        Ok(None)
    }
}
impl Prepared {
    fn apply(
        self,
        e: EdgeSites,
        tx: ArticulatedContactTransaction<'_>,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
    ) -> Result<EdgeImpactReceipt, PhysicsError> {
        let Self { law, basis, before } = self;
        let local = match &law {
            Law::NormalReturn(_, s) => s.load.map(|j| [0., 0., j]),
            Law::Sliding(s) => s.impulses,
            Law::Opening(s) => {
                let mut j = [[0.; 3]; 2];
                j[s.loaded_site] = s.impulse;
                j
            }
            Law::Mixed(s) => {
                // All feasible splits have identical generalized impulse.
                let [lo, hi] = s.impulses.first_longitudinal_range;
                s.impulses
                    .split(lo * 0.5 + hi * 0.5)
                    .ok_or(PhysicsError::InvalidValue)?
            }
        };
        let j = local.map(|v| basis[0] * v[0] + basis[1] * v[1] + basis[2] * v[2]);
        let applied = tx.commit(&j.map(|u| u.to_array()))?;
        let after = std::array::from_fn::<_, 2, _>(|i| {
            Vector::from_array(applied.after_velocities[i]) - e.deck_velocity[i]
        });
        let site = |i: usize, sticking: bool| {
            let work = j[i].dot(0.5 * (before[i] + after[i]));
            CornerImpactReceipt {
                elapsed,
                gap: e.gaps[i],
                foot,
                support,
                point: e.world[i].to_array(),
                normal: basis[2].to_array(),
                impulse_on_foot: j[i].to_array(),
                before_relative_velocity: before[i].to_array(),
                after_relative_velocity: after[i].to_array(),
                friction: e.boxes.friction,
                sticking,
                kinetic_energy_change: work + j[i].dot(e.deck_velocity[i]),
                dissipated_energy: -work,
            }
        };
        Ok(match law {
            Law::NormalReturn(stationary_site, s) => {
                let sites = [site(0, false), site(1, false)];
                EdgeImpactReceipt::Recontact(FrictionlessRecontactReceipt {
                    sites,
                    stationary_site,
                    state: s.state,
                    kinetic_energy_change: applied.kinetic_energy_change,
                    dissipated_energy: sites.iter().map(|site| site.dissipated_energy).sum(),
                })
            }
            Law::Sliding(s) => EdgeImpactReceipt::BothSlide(SlidingEdgeImpactReceipt {
                sites: [site(0, false), site(1, false)],
                kinetic_energy_change: applied.kinetic_energy_change,
                dissipated_energy: s.dissipated_energy,
                contraction_bound: s.contraction,
                iterate_error_bound: s.iterate_error_bound,
            }),
            Law::Mixed(s) => EdgeImpactReceipt::Sticking(MixedEdgeImpactReceipt {
                elapsed,
                foot,
                support,
                points: e.world.map(|p| p.to_array()),
                gaps: e.gaps,
                normal: basis[2].to_array(),
                edge_direction: basis[0].to_array(),
                mode: s.mode,
                impulses: s.impulses,
                friction: e.boxes.friction,
                before_relative_velocity: before.map(|u| u.to_array()),
                after_relative_velocity: after.map(|u| u.to_array()),
                kinetic_energy_change: applied.kinetic_energy_change,
                dissipated_energy: s.dissipated_energy,
                uniqueness_bound: s.uniqueness_bound,
            }),
            Law::Opening(s) => {
                let k = 1 - s.loaded_site;
                EdgeImpactReceipt::OneLoaded(OpeningEdgeImpactReceipt {
                    loaded_site: s.loaded_site,
                    loaded: site(s.loaded_site, s.sticking),
                    unloaded_point: e.world[k].to_array(),
                    unloaded_gap: e.gaps[k],
                    before_unloaded_relative_velocity: before[k].to_array(),
                    after_unloaded_relative_velocity: after[k].to_array(),
                    separating_margin: s.separating_margin,
                })
            }
        })
    }
}
impl RapierBackend {
    pub(in super::super) fn qualified_edge(
        &self,
        foot: BodyId,
        support: BodyId,
        displacement: Vector,
        velocity_increment: Vector,
        allowance: f64,
        mode: EdgeMode,
    ) -> Result<Option<(ColliderHandle, ColliderHandle)>, PhysicsError> {
        let Some(e) = self.cold_edge_sites(foot, support, displacement, allowance)? else {
            return Ok(None);
        };
        let op = match self.frozen_contact_operator(
            &e.sites,
            Some(e.boxes.colliders),
            displacement != Vector::ZERO,
        ) {
            Ok(op) => op,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(err) => return Err(err),
        };
        Ok(e.solve(&op.sample, velocity_increment, mode)?
            .map(|_| e.boxes.colliders))
    }
    fn edge_now(
        &self,
        foot: BodyId,
        support: BodyId,
        mode: EdgeMode,
    ) -> Result<bool, PhysicsError> {
        Ok(self
            .qualified_edge(foot, support, Vector::ZERO, Vector::ZERO, 0., mode)?
            .is_some())
    }
    /// Both sites are strictly sliding at the qualified cold boundary.
    pub fn sliding_edge_at_support_face(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.edge_now(foot, support, EdgeMode::Sliding)
    }
    /// At least one of the two loaded sites sticks at the qualified boundary.
    pub fn mixed_edge_at_support_face(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.edge_now(foot, support, EdgeMode::Mixed)
    }
    /// A cold cuboid edge has a certified active set and friction response.
    pub fn edge_at_support_face(
        &self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<bool, PhysicsError> {
        self.edge_now(foot, support, EdgeMode::Any)
    }
    /// Select the qualified two-vertex law without assuming the active set.
    pub fn project_edge_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<EdgeImpactReceipt>, PhysicsError> {
        self.apply_edge_at(foot, support, Duration::ZERO, 0.)
    }
    /// Select only a simultaneous strict two-site sliding contact.
    pub fn project_sliding_edge_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<SlidingEdgeImpactReceipt>, PhysicsError> {
        self.apply_sliding_edge_at(foot, support, Duration::ZERO, 0.)
    }
    /// Select only two loaded contact sites with at least one sticking.
    pub fn project_mixed_edge_impact(
        &mut self,
        foot: BodyId,
        support: BodyId,
    ) -> Result<Option<MixedEdgeImpactReceipt>, PhysicsError> {
        self.apply_mixed_edge_at(foot, support, Duration::ZERO, 0.)
    }
    pub(crate) fn apply_edge_at(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        allowance: f64,
    ) -> Result<Option<EdgeImpactReceipt>, PhysicsError> {
        self.apply_edge_mode(foot, support, elapsed, allowance, EdgeMode::Any)
    }
    pub(crate) fn apply_sliding_edge_at(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        allowance: f64,
    ) -> Result<Option<SlidingEdgeImpactReceipt>, PhysicsError> {
        Ok(self
            .apply_edge_mode(foot, support, elapsed, allowance, EdgeMode::Sliding)?
            .map(|r| match r {
                EdgeImpactReceipt::BothSlide(s) => s,
                _ => unreachable!("explicit sliding law"),
            }))
    }
    pub(crate) fn apply_mixed_edge_at(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        allowance: f64,
    ) -> Result<Option<MixedEdgeImpactReceipt>, PhysicsError> {
        Ok(self
            .apply_edge_mode(foot, support, elapsed, allowance, EdgeMode::Mixed)?
            .map(|r| match r {
                EdgeImpactReceipt::Sticking(s) => s,
                _ => unreachable!("explicit sticking law"),
            }))
    }
    fn apply_edge_mode(
        &mut self,
        foot: BodyId,
        support: BodyId,
        elapsed: Duration,
        allowance: f64,
        mode: EdgeMode,
    ) -> Result<Option<EdgeImpactReceipt>, PhysicsError> {
        if foot == support {
            return Err(PhysicsError::InvalidValue);
        }
        let Some(e) = self.cold_edge_sites(foot, support, Vector::ZERO, allowance)? else {
            return Ok(None);
        };
        let tx = match self.prepare_contact_sites(&e.sites, Some(e.boxes.colliders)) {
            Ok(t) => t,
            Err(PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(err) => return Err(err),
        };
        let Some(prepared) = e.solve(tx.sample(), Vector::ZERO, mode)? else {
            return Ok(None);
        };
        prepared.apply(e, tx, foot, support, elapsed).map(Some)
    }
}
