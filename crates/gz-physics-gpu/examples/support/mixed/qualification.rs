// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{pair, plane, support};
use gz_physics_gpu::{Environment as Env, EnvironmentState as State};
use pair::reference::V;
pub const N: usize = 2048;
pub const TICKS: usize = 600;

/// Ascending stable IDs deliberately interleave profiles. Actual registration is
/// a second permutation. A dense device offset cannot identify an environment.
pub fn fixture() -> Vec<Env> {
    let mut planes = plane::reference::fixture(N.div_ceil(3)).into_iter();
    let mut pairs = pair::reference::fixture((N - 1).div_ceil(3)).into_iter();
    let mut supported = support::fixture((N - 2).div_ceil(3)).into_iter();
    (0..N)
        .map(|i| Env {
            id: 91 + 17 * i as u64,
            state: match i % 3 {
                0 => State::SpherePlane(planes.next().unwrap()),
                1 => State::SpherePair(pairs.next().unwrap()),
                _ => State::SupportedSpherePair(supported.next().unwrap()),
            },
        })
        .collect()
}
pub fn registered(v: &[Env]) -> Vec<Env> {
    (0..N).map(|i| v[(i * 733 + 59) % N]).collect()
}
pub fn cpu_step(s: &mut Env) {
    match &mut s.state {
        State::SpherePlane(v) => plane::reference::analytical(v),
        State::SpherePair(v) => pair::reference::analytical(v),
        State::SupportedSpherePair(v) => support::cpu_step(v),
    }
}
pub fn signature(lanes: &[Env]) -> u64 {
    lanes.iter().fold(7_u64, |acc, env| {
        env.state
            .values()
            .iter()
            .fold(acc ^ env.id, |acc, x| acc.rotate_left(7) ^ x.to_bits())
    })
}
#[derive(Default, Debug)]
pub struct Dense {
    plane: plane::check::Errors,
    pair: pair::check::Errors,
    support: pair::check::Errors,
    load: f64,
    contacts: [usize; 3],
}
impl Dense {
    pub fn add(&mut self, measured: &Env, expected: &Env) {
        assert_eq!(measured.id, expected.id);
        match (measured.state, expected.state) {
            (State::SpherePlane(a), State::SpherePlane(b)) => {
                self.plane.add(&a, &b);
                self.contacts[0] += usize::from(a.normal_impulse > 1e-8);
            }
            (State::SpherePair(a), State::SpherePair(b)) => {
                self.pair.dense(&a, &b);
                self.contacts[1] += usize::from(a.normal_impulse > 1e-8);
            }
            (State::SupportedSpherePair(a), State::SupportedSpherePair(b)) => {
                self.support.dense(&a.pair, &b.pair);
                self.contacts[2] += usize::from(a.pair.normal_impulse > 1e-8);
                for (measured, reference) in a
                    .support_normal_impulses
                    .into_iter()
                    .zip(b.support_normal_impulses)
                {
                    assert!(measured > 0. && reference > 0.);
                    self.load = self.load.max((measured - reference).abs());
                }
            }
            _ => panic!("profile changed at observation barrier"),
        }
    }
    pub fn verify(&self) {
        println!("CPU analytical maxima by profile, SI units and orientation chord {self:?}");
        let s = &self.plane;
        assert!(s.p < 2e-9 && s.v < 5e-9 && s.w < 2e-7 && s.q < 2e-8 && s.jn < 5e-9 && s.jt < 5e-9);
        for s in [&self.pair, &self.support] {
            assert!(
                s.motion.p < 2e-8
                    && s.motion.v < 2e-8
                    && s.motion.w < 2e-7
                    && s.motion.q < 2e-8
                    && s.impulse < 2e-8
                    && s.normal < 2e-8
                    && s.toi < 2e-10
            );
        }
        assert!(self.load < 2e-10 && self.contacts[0] > 1000);
        for (count, physical) in [(683, self.contacts[1]), (682, self.contacts[2])] {
            assert_eq!(physical, count - count / 8);
        }
    }
}
pub struct Native {
    plane: Vec<(usize, plane::reference::Native)>,
    pairs: Vec<(usize, pair::reference::Native)>,
    supported: Vec<(usize, pair::reference::Native, [f64; 2])>,
    plane_errors: plane::check::NativeAgreement,
    pair_errors: pair::check::NativeAgreement,
    support_errors: pair::check::NativeAgreement,
    max_step_load: f64,
    max_integrated_load: f64,
}
impl Native {
    pub fn new(initial: &[Env]) -> Self {
        let mut s = Self {
            plane: vec![],
            pairs: vec![],
            supported: vec![],
            plane_errors: Default::default(),
            pair_errors: Default::default(),
            support_errors: Default::default(),
            max_step_load: 0.,
            max_integrated_load: 0.,
        };
        for k in 0..32 {
            let i = 3 * (k * 19);
            let State::SpherePlane(p) = initial[i].state else {
                unreachable!()
            };
            let State::SpherePair(a) = initial[i + 1].state else {
                unreachable!()
            };
            let State::SupportedSpherePair(b) = initial[i + 2].state else {
                unreachable!()
            };
            s.plane.push((i, plane::reference::Native::new(p)));
            s.pairs.push((i + 1, pair::reference::Native::new(a)));
            s.supported.push((
                i + 2,
                pair::reference::Native::with_support(
                    b.pair,
                    V::from(b.plane_normal),
                    b.plane_offset,
                    16,
                ),
                [0.; 2],
            ));
        }
        s
    }
    pub fn step(&mut self, k: usize, states: &[Env]) {
        for (i, n) in &mut self.plane {
            n.step();
            let State::SpherePlane(s) = states[*i].state else {
                unreachable!()
            };
            self.plane_errors.add(k, &s, n);
        }
        for (i, n) in &mut self.pairs {
            n.step();
            let State::SpherePair(s) = states[*i].state else {
                unreachable!()
            };
            self.pair_errors.add(k, &s, n);
        }
        for (i, n, total) in &mut self.supported {
            n.step();
            let State::SupportedSpherePair(s) = states[*i].state else {
                unreachable!()
            };
            self.support_errors.add(k, &s.pair, n);
            for ((sum, native), gpu) in total
                .iter_mut()
                .zip(n.support_impulses)
                .zip(s.support_normal_impulses)
            {
                let d = native - gpu;
                *sum += d;
                self.max_step_load = self.max_step_load.max(d.abs());
                self.max_integrated_load = self.max_integrated_load.max(sum.abs());
            }
        }
    }
    pub fn verify(&self) {
        let plane = &self.plane_errors;
        println!("32 native sphere/plane worlds, 8 substeps and 1 ms observations {plane:?}");
        assert!(
            self.plane
                .iter()
                .all(|(_, n)| n.gpu_first.is_some() && n.native_first.is_some())
        );
        assert!(
            plane.raw.p < 0.012
                && plane.max_impact_offset_ticks <= 2
                && plane.outside_impact.v < 0.065
                && plane.synchronized_surface_omega < 0.065
                && plane.outside_impact.q < 0.06
                && plane.postimpact_impulse_speed_m_s < 0.011
        );
        for (kind, s) in [
            ("free pair", &self.pair_errors),
            ("supported pair", &self.support_errors),
        ] {
            println!("32 native {kind} worlds, 16 substeps and 1 ms observations {s:?}");
            assert!(
                s.max_offset <= 1
                    && s.capped_velocity_residual < 0.025
                    && s.capped_rim_residual < 0.040
                    && s.capped_position_residual < 0.009
                    && s.matched_impact == 28
                    && s.mixed_sliding > 0
            );
        }
        println!(
            "native supporting impulse step max {} N s, integrated max {} N s",
            self.max_step_load, self.max_integrated_load
        );
        assert!(self.max_integrated_load < 0.003);
    }
}
