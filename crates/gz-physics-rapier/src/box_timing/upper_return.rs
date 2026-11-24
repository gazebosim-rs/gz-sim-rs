// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Certify two temporally ordered measured reactions within one actuator tick.
use super::*;
/// A lower material corner arrived after an explicitly measured upper strike.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpposingReturnReceipt {
    pub lower: OpposingCornerReceipt,
    pub previous_upper_corner: u8,
    /// Fractional seconds from the beginning of this public interval.
    pub previous_upper_s: f64,
    /// Foot state at the start of the lower-return branch. For a compound
    /// interval this is the independently advanced state after the upper strike.
    pub branch_start: BodyState,
    /// Starting position and twist of the native call enclosing the lower impact.
    pub native_start: BodyState,
    pub native_start_elapsed: Duration,
    /// Signed distances of the preceding upper material point to its own plane.
    pub previous_upper_gaps_m: [f64; 3],
}
#[derive(Clone, Copy)]
pub(crate) struct PriorUpper {
    pub corner: u8,
    pub time: f64,
}
impl From<OpposingCornerReceipt> for PriorUpper {
    fn from(h: OpposingCornerReceipt) -> Self {
        Self {
            corner: h.point.corner,
            time: h.point.pulse_start.as_secs_f64() + h.point.after_cut_s,
        }
    }
}
impl UpperPair {
    pub(super) fn state_at(&self, e: &RapierBackend, x: Cut) -> BodyState {
        let center = e.world.bodies[self.c.foot]
            .mass_properties()
            .local_mprops
            .local_com;
        BodyState {
            position: (x.com - x.q * center).into(),
            orientation: x.q.into(),
            velocity: x.v.into(),
            angular_velocity: x.spin.into(),
        }
    }
    /// This estimate only selects a candidate split. Each accepted part is
    /// separately tested over its full interval from the measured native cut.
    pub(super) fn first_upper_hint(&self, i: Incoming, h: f64) -> Option<f64> {
        let s = i.state;
        let d = i.decks[1];
        let end = i.rotor.bounded(s, h, 5)?;
        let mut first = f64::INFINITY;
        for &arm in &i.arms[4..] {
            let f = Surface {
                arm,
                top: d.top,
                n: d.n,
                velocity: d.v,
            };
            let a = f.gap(s, 0.);
            let b = f.gap(end, h);
            if a < -i.tol || b > i.tol || f.incoming(s).dot(d.n) >= -0.05 {
                continue;
            }
            let mut t = (h * a / (a - b)).clamp(0., h);
            for _ in 0..8 {
                let x = i.rotor.bounded(s, t, 5)?;
                let gap = f.gap(x, t);
                if gap.abs() < i.tol {
                    break;
                }
                let speed = f.incoming(x).dot(d.n);
                if speed >= -0.05 {
                    break;
                }
                t = (t - gap / speed).clamp(0., h);
            }
            first = first.min(t);
        }
        first.is_finite().then_some(first)
    }
    pub(super) fn for_lower(
        &self,
        e: &RapierBackend,
        mut i: Incoming,
        x: Cut,
        t: f64,
    ) -> Option<Incoming> {
        let a = &e.world.colliders[self.c.sole];
        let b = &e.world.colliders[self.c.deck];
        if CoefficientCombineRule::combine(
            a.restitution(),
            b.restitution(),
            a.restitution_combine_rule(),
            b.restitution_combine_rule(),
        ) != 0.
        {
            return None;
        }
        i.friction = CoefficientCombineRule::combine(
            a.friction(),
            b.friction(),
            a.friction_combine_rule(),
            b.friction_combine_rule(),
        );
        i.state = x;
        i.decks = [i.decks[1].at(t), i.decks[0].at(t)];
        i.arms.rotate_left(4);
        Some(i)
    }
    fn lower_branch(
        &self,
        e: &RapierBackend,
        i: Incoming,
        branch: Branch,
        prior: PriorUpper,
        native: (Cut, Duration),
    ) -> Option<UpperStep> {
        let entry = self.state_at(e, i.state);
        let dt = branch.dt.as_secs_f64();
        let mut lower = self.branch(e, i, branch)?;
        let hit = lower.receipt.take()?;
        let at = hit.point.pulse_start.as_secs_f64() + hit.point.after_cut_s;
        if at - prior.time <= 1e-9 {
            return None;
        }
        let d = i.decks[0];
        let arm = i.arms[usize::from(prior.corner)];
        let measured = |x: BodyState, t: f64| {
            (Vector::from(x.position)
                + Rotation::from_array(x.orientation)
                    * (arm
                        + e.world.bodies[self.c.foot]
                            .mass_properties()
                            .local_mprops
                            .local_com)
                - d.at(t).top)
                .dot(d.n)
        };
        let gaps = [
            measured(entry, 0.),
            measured(hit.incoming, hit.point.after_cut_s),
            measured(self.state_at(e, lower.end), dt),
        ];
        if gaps[1] <= i.tol || gaps[2] < -i.tol {
            return None;
        }
        lower.returned = Some(Box::new(OpposingReturnReceipt {
            lower: hit,
            previous_upper_corner: prior.corner,
            previous_upper_s: prior.time,
            branch_start: entry,
            native_start: self.state_at(e, native.0),
            native_start_elapsed: native.1,
            previous_upper_gaps_m: gaps,
        }));
        Some(lower)
    }
    pub(crate) fn ordered(
        &self,
        e: &RapierBackend,
        elapsed: Duration,
        dt: Duration,
        separated: bool,
        prior: Option<PriorUpper>,
    ) -> Option<UpperStep> {
        if dt.is_zero() || dt > Duration::from_millis(1) {
            return None;
        }
        let i = self.incoming(e)?;
        if let Some(p) = prior
            && let Some(lower) = self.for_lower(e, i, i.state, 0.)
            && let Some(step) = self.lower_branch(
                e,
                lower,
                Branch {
                    elapsed,
                    dt,
                    separated: false,
                    lower_after: Some(p.corner),
                },
                p,
                (i.state, elapsed),
            )
        {
            return Some(step);
        }
        if let Some(one) = self.branch(
            e,
            i,
            Branch {
                elapsed,
                dt,
                separated: separated || prior.is_some(),
                lower_after: None,
            },
        ) {
            return Some(one);
        }
        let upper_time = self.first_upper_hint(i, dt.as_secs_f64())?;
        // Leave the native integrator unmodified until *both* independent
        // sweeps and momentum changes have succeeded. A failed sequence commits nothing.
        let split = Duration::from_nanos((upper_time * 1e9).ceil() as u64 + 2);
        if split >= dt {
            return None;
        }
        let first = self.branch(
            e,
            i,
            Branch {
                elapsed,
                dt: split,
                separated: false,
                lower_after: None,
            },
        )?;
        let hit = first.receipt?;
        let p = PriorUpper::from(hit);
        let lower = self.for_lower(e, i, first.end, split.as_secs_f64())?;
        let mut second = self.lower_branch(
            e,
            lower,
            Branch {
                elapsed: elapsed + split,
                dt: dt - split,
                separated: false,
                lower_after: Some(p.corner),
            },
            p,
            (i.state, elapsed),
        )?;
        second.receipt = Some(hit);
        Some(second)
    }
    /// Spend a remaining native call at an isolated return before a third
    /// contact can make the original grid endpoint ambiguous. Every proposed
    /// duration is verified by the same full branch admission used in ordered().
    pub(crate) fn shorten(
        &self,
        e: &RapierBackend,
        elapsed: Duration,
        h: Duration,
        prior: Option<PriorUpper>,
    ) -> Option<(Duration, UpperStep)> {
        let i = self.incoming(e)?;
        let short = |t: f64| Duration::from_nanos((t * 1e9).ceil() as u64 + 1);
        let native = (i.state, elapsed);
        if let Some(p) = prior {
            let lower = self.for_lower(e, i, i.state, 0.)?;
            let dt = short(self.first_upper_hint(lower, h.as_secs_f64())?);
            if dt >= h {
                return None;
            }
            let step = self.lower_branch(
                e,
                lower,
                Branch {
                    elapsed,
                    dt,
                    separated: false,
                    lower_after: Some(p.corner),
                },
                p,
                native,
            )?;
            return Some((dt, step));
        }
        let split = short(self.first_upper_hint(i, h.as_secs_f64())?);
        if split >= h {
            return None;
        }
        let first = self.branch(
            e,
            i,
            Branch {
                elapsed,
                dt: split,
                separated: false,
                lower_after: None,
            },
        )?;
        let hit = first.receipt?;
        let p = PriorUpper::from(hit);
        if let Some(lower) = self.for_lower(e, i, first.end, split.as_secs_f64())
            && let Some(next) = self.first_upper_hint(lower, (h - split).as_secs_f64())
        {
            let df = short(next);
            if split + df < h
                && let Some(mut second) = self.lower_branch(
                    e,
                    lower,
                    Branch {
                        elapsed: elapsed + split,
                        dt: df,
                        separated: false,
                        lower_after: Some(p.corner),
                    },
                    p,
                    native,
                )
            {
                second.receipt = Some(hit);
                return Some((split + df, second));
            }
        }
        Some((split, first))
    }
}

#[cfg(test)]
#[path = "../../tests/private/shortened_impact.rs"]
mod tests;
