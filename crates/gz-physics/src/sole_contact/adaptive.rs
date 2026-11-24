// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local truncation estimates comparing neighboring temporal and spatial grids.
use super::*;

#[derive(Debug, Clone, Copy)]
pub struct SoleAccuracy {
    /// The coarsest temporal comparison starts at this many midpoint substeps.
    pub min_substeps: u32,
    pub max_substeps: u32,
    /// The coarsest spatial comparison starts at this quadrature depth.
    pub min_depth: u8,
    pub max_depth: u8,
    /// Absolute tolerances on the difference between adjacent grids in one tick.
    pub position_m: f64,
    pub rotation_rad: f64,
    pub impulse_ns: f64,
    pub angular_impulse_nms: f64,
    pub heat_j: f64,
}

impl SoleAccuracy {
    pub fn is_valid(&self) -> bool {
        self.min_substeps > 0
            && self.min_substeps <= self.max_substeps / 2
            && self.min_depth < self.max_depth
            && self.max_depth <= 5
            && [
                self.position_m,
                self.rotation_rad,
                self.impulse_ns,
                self.angular_impulse_nms,
                self.heat_j,
            ]
            .into_iter()
            .all(|x| x.is_finite() && x >= 0.)
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SoleDifference {
    pub position_m: f64,
    pub rotation_rad: f64,
    pub impulse_ns: f64,
    pub angular_impulse_nms: f64,
    pub heat_j: f64,
}
impl SoleDifference {
    pub(crate) fn between_states(a: &SoleState, b: &SoleState) -> Self {
        let q = a.orientation.conjugate() * b.orientation;
        Self {
            position_m: a.center.distance(b.center),
            rotation_rad: 2. * q.xyz().length().atan2(q.w.abs()),
            ..Default::default()
        }
    }
    pub(crate) fn within(&self, p: &SoleAccuracy) -> bool {
        self.position_m <= p.position_m
            && self.rotation_rad <= p.rotation_rad
            && self.impulse_ns <= p.impulse_ns
            && self.angular_impulse_nms <= p.angular_impulse_nms
            && self.heat_j <= p.heat_j
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SoleAdaptiveStep {
    pub step: SoleStep,
    pub substeps: u32,
    pub depth: u8,
    pub trials: u32,
    pub temporal: SoleDifference,
    pub spatial: SoleDifference,
}

pub(crate) struct GridResult<T, D> {
    pub value: T,
    pub substeps: u32,
    pub depth: u8,
    pub trials: u32,
    pub temporal: D,
    pub spatial: D,
}

// The single-support and coupled paths use the same time/area acceptance
// and retry logic, while comparing different sets of physical observables.
pub(crate) fn select_grid<T, D, E: From<SoleError>>(
    policy: &SoleAccuracy,
    mut run: impl FnMut(u32, u8) -> Result<T, E>,
    compare: impl Fn(&T, &T) -> D,
    within: impl Fn(&D) -> bool,
    unresolved: impl Fn(&T) -> bool,
) -> Result<GridResult<T, D>, E> {
    if !policy.is_valid() {
        return Err(SoleError::InvalidInput.into());
    }
    let mut cache = Vec::<(u32, u8, T)>::new();
    let mut n = 2 * policy.min_substeps;
    let mut d = policy.min_depth + 1;
    loop {
        let mut sample = |n, d| -> Result<usize, E> {
            if let Some(i) = cache.iter().position(|(t, q, _)| (*t, *q) == (n, d)) {
                return Ok(i);
            }
            let v = run(n, d)?;
            cache.push((n, d, v));
            Ok(cache.len() - 1)
        };
        let current = sample(n, d)?;
        let time = sample(n / 2, d)?;
        let area = sample(n, d - 1)?;
        let t = &cache[current].2;
        let temporal = compare(t, &cache[time].2);
        let spatial = compare(t, &cache[area].2);
        let refine_time = !within(&temporal) || unresolved(t);
        let refine_area = !within(&spatial);
        if !refine_time && !refine_area {
            return Ok(GridResult {
                trials: cache.len() as u32,
                value: cache.swap_remove(current).2,
                substeps: n,
                depth: d,
                temporal,
                spatial,
            });
        }
        if (refine_time && n == policy.max_substeps) || (refine_area && d == policy.max_depth) {
            return Err(SoleError::AccuracyBudget.into());
        }
        if refine_time {
            n = n.saturating_mul(2).min(policy.max_substeps);
        }
        if refine_area {
            d += 1;
        }
    }
}

impl SoleBody {
    /// Compare (n,d) against (n/2,d) and (n,d-1), refining only the axes
    /// exceeding tolerance. Each trial starts from exactly the same incoming
    /// state; rejected trials contribute no work or impulse to the result.
    /// Differences are empirical local error estimates, not rigorous bounds.
    #[allow(clippy::too_many_arguments)]
    pub fn step_adaptive(
        &self,
        state: &mut SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        gravity: DVec3,
        dt: f64,
        policy: &SoleAccuracy,
        force: DVec3,
        torque: DVec3,
    ) -> Result<SoleAdaptiveStep, SoleError> {
        let initial = *state;
        let grid = select_grid(
            policy,
            |n, d| -> Result<_, SoleError> {
                let mut next = initial;
                let step = self.step_with_wrench(
                    &mut next, deck, material, gravity, dt, n, d, force, torque,
                )?;
                Ok((next, step))
            },
            |(a, x), (b, y)| SoleDifference {
                impulse_ns: x.impulse.distance(y.impulse),
                angular_impulse_nms: x.angular_impulse.distance(y.angular_impulse),
                heat_j: (x.dissipated_energy - y.dissipated_energy).abs(),
                ..SoleDifference::between_states(a, b)
            },
            |e| e.within(policy),
            |(_, step)| step.unresolved(),
        )?;
        *state = grid.value.0;
        Ok(SoleAdaptiveStep {
            step: grid.value.1,
            substeps: grid.substeps,
            depth: grid.depth,
            trials: grid.trials,
            temporal: grid.temporal,
            spatial: grid.spatial,
        })
    }
}
