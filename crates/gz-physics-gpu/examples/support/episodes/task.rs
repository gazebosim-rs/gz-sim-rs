// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    BodySlot, CentralImpulse, Environment as Env, EnvironmentState as State, MixedBatch,
    SolidSphere, SpherePairLane, SpherePlaneLane, SupportedSpherePairLane,
    sampling::{counter, uniform},
};
use serde::{Deserialize, Serialize};
pub const COUNT: usize = 2048;
pub const TRAIN: u64 = 1152;
pub const EVALUATE: u64 = 640;
const ACTION: [f64; 3] = [-0.28, 0., 0.28];
const CUTS: [f64; 8] = [-0.85, -0.46, -0.22, -0.10, 0.10, 0.22, 0.46, 0.85];
pub fn id(i: usize) -> u64 {
    97 + 13 * i as u64
}
pub fn index(id: u64) -> usize {
    ((id - 97) / 13) as usize
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
struct Transition {
    state: usize,
    action: usize,
    reward: f64,
    samples: u32,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Episode {
    pub start: u64,
    pub end: u64,
    pub target: f64,
    pub initial_x: f64,
    period: u64,
    last: Option<Transition>,
}
fn spawn(seed: u64, i: usize, t: u64) -> (Env, Episode) {
    let id = id(i);
    let u = |stream, draw| uniform(seed, id, t, stream, draw);
    // Geometry stays fixed for this identity across episodes. The physical
    // state, mass, material, target, decision period and horizon are resampled.
    let r = 0.085 + 0.025 * uniform(seed, id, 0, 0, 0);
    let m = 0.55 + 1.6 * u(1, 0);
    let vx = -0.45 + 0.9 * u(1, 1);
    let x = -0.25 + 0.12 * u(1, 2);
    let spin = [0., 0., -3. + 6. * u(1, 3)];
    let q = [0., 0., 0., 1.];
    let mut a = SolidSphere {
        position: [x, 0., r],
        velocity: [vx, 0., 0.],
        angular_velocity: spin,
        orientation: q,
        radius: r,
        inverse_mass: 1. / m,
    };
    let friction = 0.07 + 0.38 * u(1, 4);
    let n = [0., 0., 1.];
    let g = [0., 0., -9.81];
    let profile = i % 3;
    let state = if profile == 0 {
        State::SpherePlane(SpherePlaneLane {
            position: [x, 0., r + 0.04 * u(1, 5)],
            velocity: [vx, 0., -0.14 * u(1, 6)],
            angular_velocity: spin,
            orientation: q,
            gravity: g,
            plane_normal: n,
            plane_offset: 0.,
            radius: r,
            inverse_mass: a.inverse_mass,
            friction,
            normal_impulse: 0.,
            tangential_impulse: [0.; 3],
        })
    } else {
        let b = SolidSphere {
            position: [x + 2. * r + 0.025 + 0.12 * u(1, 7), 0., r],
            velocity: [-0.55 + 0.80 * u(1, 8), 0., 0.],
            angular_velocity: [0., 0., -2. + 4. * u(1, 9)],
            inverse_mass: 1. / (0.65 + 1.7 * u(1, 10)),
            ..a
        };
        if profile == 1 {
            a.velocity[1] = 0.08 * (u(1, 11) - 0.5);
        }
        let p = SpherePairLane {
            a,
            b,
            gravity: if profile == 1 { [0., 0., -0.12] } else { g },
            friction,
            restitution: 0.12 + 0.58 * u(1, 12),
            normal_impulse: 0.,
            tangential_impulse: [0.; 3],
            contact_normal: [0.; 3],
            impact_time: -1.,
        };
        if profile == 1 {
            State::SpherePair(p)
        } else {
            State::SupportedSpherePair(SupportedSpherePairLane {
                pair: p,
                plane_normal: n,
                plane_offset: 0.,
                support_normal_impulses: [0.; 2],
            })
        }
    };
    let target = (if u(2, 0) < 0.5 { -1. } else { 1. }) * (0.72 + 0.52 * u(2, 1));
    let end = t + 129 + counter(seed, id, t, 2, 2) % 129;
    let period = 12 + counter(seed, id, t, 2, 3) % 9;
    (
        Env { id, state },
        Episode {
            start: t,
            end,
            target,
            initial_x: x,
            period,
            last: None,
        },
    )
}
pub fn sphere(s: &State) -> SolidSphere {
    match *s {
        State::SpherePlane(s) => SolidSphere {
            position: s.position,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
            orientation: s.orientation,
            radius: s.radius,
            inverse_mass: s.inverse_mass,
        },
        State::SpherePair(p) => p.a,
        State::SupportedSpherePair(s) => s.pair.a,
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    Train,
    Greedy,
    Coast,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Policy {
    pub q: [[f64; 3]; 27],
}
impl Default for Policy {
    fn default() -> Self {
        Self { q: [[0.; 3]; 27] }
    }
}
fn best(q: [f64; 3]) -> usize {
    // Equal estimates prefer zero effort, then negative, then positive action.
    [0, 2]
        .into_iter()
        .fold(1, |b, i| if q[i] > q[b] { i } else { b })
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Agent {
    seed: u64,
    mode: Mode,
    pub policy: Policy,
    pub episodes: Vec<Episode>,
    pub resets: u64,
    pub decisions: u64,
    pub impulse_count: u64,
    pub min_horizon: u64,
    pub max_horizon: u64,
    pub cost: [f64; 3],
    pub steps: [u64; 3],
    pub raw_returns: f64,
}
#[derive(Default, Debug, PartialEq)]
pub struct Edits {
    pub resets: Vec<Env>,
    pub actions: Vec<CentralImpulse>,
}
impl Agent {
    pub fn new(seed: u64, mode: Mode, policy: Policy) -> (Self, Vec<Env>) {
        let (states, episodes) = (0..COUNT).map(|i| spawn(seed, i, 0)).unzip();
        (
            Self {
                seed,
                mode,
                policy,
                episodes,
                resets: 0,
                decisions: 0,
                impulse_count: 0,
                min_horizon: u64::MAX,
                max_horizon: 0,
                cost: [0.; 3],
                steps: [0; 3],
                raw_returns: 0.,
            },
            states,
        )
    }
    pub fn plan(
        &mut self,
        k: u64,
        batch: &MixedBatch,
        chunk: usize,
    ) -> Result<Edits, gz_physics_gpu::GpuError> {
        let mut edits = Edits::default();
        // Freeze decision values for this epoch to make chunking independent
        // of policy writes. Reduce Q updates in canonical logical-ID order.
        let frozen = self.policy.clone();
        if chunk >= COUNT {
            for (i, env) in batch.environments().iter().enumerate() {
                self.decide(k, i, *env, &frozen, &mut edits);
            }
        } else {
            let ids: Vec<_> = (0..COUNT).map(id).collect();
            for keys in ids.chunks(chunk) {
                for env in batch.get_many(keys)? {
                    self.decide(k, index(env.id), *env, &frozen, &mut edits);
                }
            }
        }
        Ok(edits)
    }
    fn decide(&mut self, k: u64, i: usize, mut env: Env, frozen: &Policy, edits: &mut Edits) {
        let e = &mut self.episodes[i];
        let terminal = e.end == k;
        if !terminal && !(k - e.start).is_multiple_of(e.period) {
            return;
        }
        let prev = e.last.take();
        if terminal {
            let (replacement, next) = spawn(self.seed, i, k);
            self.min_horizon = self.min_horizon.min(k - e.start);
            self.max_horizon = self.max_horizon.max(k - e.start);
            *e = next;
            env = replacement;
            edits.resets.push(env);
            self.resets += 1;
        }
        let a = sphere(&env.state);
        let err = e.target - a.velocity[0];
        let state = (i % 3) * 9 + CUTS.partition_point(|&cut| err > cut);
        if let Some(old) = prev
            && self.mode == Mode::Train
        {
            let reward = old.reward / f64::from(old.samples) - 0.015 * ACTION[old.action].abs();
            let next = if terminal {
                0.
            } else {
                frozen.q[state][best(frozen.q[state])]
            };
            let q = &mut self.policy.q[old.state][old.action];
            *q += 0.035 * (reward + 0.83 * next - *q);
        }
        let action = match self.mode {
            Mode::Coast => 1,
            Mode::Train if uniform(self.seed, env.id, k, 3, 0) < 0.24 => {
                (counter(self.seed, env.id, k, 3, 1) % 3) as usize
            }
            _ => best(frozen.q[state]),
        };
        if action != 1 {
            edits.actions.push(CentralImpulse {
                id: env.id,
                body: BodySlot::A,
                impulse: [ACTION[action] / a.inverse_mass, 0., 0.],
            });
            self.impulse_count += 1;
        }
        e.last = Some(Transition {
            state,
            action,
            reward: 0.,
            samples: 0,
        });
        self.decisions += 1;
    }
    pub fn observe(&mut self, states: &[Env]) {
        for (i, (env, e)) in states.iter().zip(&mut self.episodes).enumerate() {
            let err = (sphere(&env.state).velocity[0] - e.target).abs();
            self.cost[i % 3] += err;
            self.steps[i % 3] += 1;
            let sample = e.last.as_mut().expect("episode has a policy decision");
            sample.reward -= err;
            sample.samples += 1;
            self.raw_returns -= err;
        }
    }
    pub fn error(&self) -> [f64; 3] {
        std::array::from_fn(|i| self.cost[i] / self.steps[i] as f64)
    }
}
