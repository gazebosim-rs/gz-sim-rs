// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    pair, plane,
    task::{Edits, Episode, index, sphere},
};
use gz_physics_gpu::{Environment, EnvironmentState as State, SolidSphere};
use pair::reference::{Q, V};
use std::collections::BTreeMap;
enum Probe {
    Plane(Box<plane::reference::Native>),
    Pair(Box<pair::reference::Native>),
}
impl Probe {
    fn new(env: &Environment) -> Self {
        match env.state {
            State::SpherePlane(p) => Self::Plane(Box::new(plane::reference::Native::new(p))),
            State::SpherePair(p) => Self::Pair(Box::new(pair::reference::Native::new(p))),
            State::SupportedSpherePair(s) => {
                Self::Pair(Box::new(pair::reference::Native::with_support(
                    s.pair,
                    V::from(s.plane_normal),
                    s.plane_offset,
                    16,
                )))
            }
        }
    }
    fn action(&mut self, j: [f64; 3]) {
        match self {
            Self::Plane(p) => p.impulse_a(j),
            Self::Pair(p) => p.impulse_a(j),
        }
    }
    fn step(&mut self) -> SolidSphere {
        match self {
            Self::Plane(p) => {
                p.step();
                sphere(&State::SpherePlane(p.state))
            }
            Self::Pair(p) => {
                p.step();
                p.state.a
            }
        }
    }
}
#[derive(Default, Debug, Clone, Copy)]
struct Error {
    p: f64,
    v: f64,
    w: f64,
    q: f64,
    task_delta: f64,
    native_cost: f64,
    gpu_cost: f64,
    total: u64,
}
impl Error {
    fn add(&mut self, meas: SolidSphere, native: SolidSphere, target: f64) {
        self.p = self
            .p
            .max(V::from(meas.position).distance(V::from(native.position)));
        self.v = self
            .v
            .max(V::from(meas.velocity).distance(V::from(native.velocity)));
        self.w = self
            .w
            .max(V::from(meas.angular_velocity).distance(V::from(native.angular_velocity)));
        let (a, b) = (
            Q::from_array(meas.orientation),
            Q::from_array(native.orientation),
        );
        self.q = self.q.max((a - b).length().min((a + b).length()));
        let g = (meas.velocity[0] - target).abs();
        let n = (native.velocity[0] - target).abs();
        self.gpu_cost += g;
        self.native_cost += n;
        self.task_delta += (n - g).abs();
        self.total += 1;
    }
}
pub struct NativeRun {
    probes: BTreeMap<usize, Probe>,
    errors: [Error; 3],
}
impl NativeRun {
    pub fn new(items: &[Environment]) -> Self {
        let mut probes = BTreeMap::new();
        for j in 0..32 {
            for k in 0..3 {
                let i = j * 19 * 3 + k;
                probes.insert(i, Probe::new(&items[i]));
            }
        }
        Self {
            probes,
            errors: Default::default(),
        }
    }
    pub fn edit(&mut self, edits: &Edits) {
        for s in &edits.resets {
            if let Some(p) = self.probes.get_mut(&index(s.id)) {
                *p = Probe::new(s);
            }
        }
        for a in &edits.actions {
            if let Some(p) = self.probes.get_mut(&index(a.id)) {
                p.action(a.impulse);
            }
        }
    }
    pub fn step(&mut self, states: &[Environment], episodes: &[Episode]) {
        for (&i, p) in &mut self.probes {
            self.errors[i % 3].add(sphere(&states[i].state), p.step(), episodes[i].target);
        }
    }
    pub fn report(&self, name: &str) -> [f64; 3] {
        std::array::from_fn(|i| {
            let e = self.errors[i];
            let count = e.total as f64;
            println!(
                "{name} refined native profile {i}: 32 trajectories, max position {} m, velocity {} m/s, spin {} rad/s, orientation chord {}; mean task error native {} GPU {} m/s, mean absolute per-tick discrepancy {} m/s",
                e.p,
                e.v,
                e.w,
                e.q,
                e.native_cost / count,
                e.gpu_cost / count,
                e.task_delta / count
            );
            // Less than one fifth of a 0.28 m/s action quantum in average
            // per-tick task discrepancy, including impacts and switching times.
            assert!(e.task_delta / count < 0.056);
            e.native_cost / count
        })
    }
}
