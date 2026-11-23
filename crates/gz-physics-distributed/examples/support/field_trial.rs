// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::{SceneBackend, physics::STEP};
use gz_physics_multifidelity::MultiFidelityBackend;
use serde::Serialize;
use std::{error::Error, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
struct Rng(u64);
impl Rng {
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        let unit = ((z ^ (z >> 31)) >> 11) as f64 * (1. / ((1u64 << 53) as f64));
        lo + (hi - lo) * unit
    }
}
#[derive(Default, Serialize)]
pub struct Trial {
    pub seed: u64,
    pub ticks: u64,
    pub bitwise: bool,
    pub exact_contact_sets: bool,
    pub transfers: u64,
    pub impacts: [Option<u64>; 2],
    pub replacement: bool,
    pub position_m: f64,
    pub velocity_m_s: f64,
    pub orientation_rad: f64,
    pub spin_rad_s: f64,
    pub impulse_n_s: f64,
    pub first_body_difference: Option<u64>,
    pub error: Option<String>,
}
fn norm<const N: usize>(x: [f64; N], y: [f64; N]) -> f64 {
    x.into_iter().zip(y).fold(0_f64, |s, (a, b)| s.hypot(a - b))
}
pub fn trial(t: &mut Trial, worker: &Path) -> Result<()> {
    let mut rng = Rng(t.seed);
    let gravity = [
        rng.range(-0.045, 0.045),
        rng.range(-0.035, 0.035),
        -rng.range(7., 15.),
    ];
    let mut actual = SceneBackend::new(worker, gravity, STEP)?;
    let mut native = MultiFidelityBackend::new(gravity)?;
    let mut ids = Vec::<BodyId>::new();
    for i in 0..3 {
        let (center, half, mass) = match i {
            0 => ([0., 0., 0.], [2.6, 1., 0.14], 3.),
            _ => (
                [
                    if i == 1 { -0.84 } else { 0.84 },
                    rng.range(-0.1, 0.1),
                    if i == 1 {
                        rng.range(1.3, 2.2)
                    } else {
                        rng.range(9., 11.)
                    },
                ],
                [
                    rng.range(0.21, 0.34),
                    rng.range(0.22, 0.30),
                    rng.range(0.2, 0.33),
                ],
                rng.range(0.55, 2.7),
            ),
        };
        let spec = BodySpec {
            position: center,
            shape: Shape::Box { half_extents: half },
            mass: Some(mass),
            velocity: if i == 0 {
                [0.; 3]
            } else {
                [rng.range(-0.09, 0.09), 0., 0.]
            },
        };
        let angle = if i == 0 { 0. } else { rng.range(-0.7, 0.7) };
        let rotation = RotationSpec {
            orientation: [0., 0., angle.sin(), angle.cos()],
            angular_velocity: if i == 0 {
                [0.; 3]
            } else {
                [0., 0., rng.range(-0.4, 0.4)]
            },
        };
        let id = if i == 0 {
            actual.spawn_kinematic_link(spec, rotation, None)?
        } else {
            actual.spawn_link(spec, rotation, None)?
        };
        assert_eq!(
            id,
            native.spawn_awake_frictionless_box(spec, rotation, i == 0)?
        );
        ids.push(id);
    }
    actual.initialize()?;
    let mut owner = actual.frame().unwrap().body(ids[2].raw()).unwrap().0;
    let fault_tick = rng.range(40., 180.) as u64;
    for tick in 1..=216 {
        if [13, 57, 86, 123, 171].contains(&tick) {
            let id = ids[1 + usize::from(rng.range(0., 1.) > 0.5)];
            let j = [
                rng.range(-0.02, 0.02),
                rng.range(-0.02, 0.02),
                rng.range(0., 0.035),
            ];
            let angular = [0., 0., rng.range(-0.014, 0.014)];
            let force = [
                rng.range(-0.6, 0.6),
                rng.range(-0.2, 0.2),
                rng.range(-1.2, 1.2),
            ];
            let torque = [0., 0., rng.range(-0.21, 0.21)];
            actual
                .apply_impulse(id, [0.; 3], angular)
                .map_err(|e| format!("step {tick}, immediate spatial impulse: {e}"))?;
            native.apply_impulse(id, [0.; 3], angular)?;
            actual
                .queue_linear_impulse(id, j)
                .map_err(|e| format!("step {tick}, queued impulse: {e}"))?;
            native.apply_impulse(id, j, [0.; 3])?;
            actual
                .add_world_wrench(id, force, torque)
                .map_err(|e| format!("step {tick}, held wrench: {e}"))?;
            native.add_world_wrench(id, force, torque)?;
        }
        let pids = actual.worker_pids().unwrap();
        let failed = usize::from(t.seed % 2 == 1);
        if tick == fault_tick
            && !Command::new("kill")
                .args(["-KILL", &pids[failed].to_string()])
                .status()?
                .success()
        {
            return Err("failed to stop the selected worker".into());
        }
        actual
            .step(STEP)
            .map_err(|e| format!("step {tick}, advance: {e}"))?;
        native.step(STEP)?;
        t.ticks = tick;
        if tick == fault_tick {
            let now = actual.worker_pids().unwrap();
            t.replacement = now[failed] != pids[failed] && now[1 - failed] == pids[1 - failed];
        }
        let f = actual.frame().unwrap();
        let now = f.body(ids[2].raw()).unwrap().0;
        t.transfers += u64::from(now != owner);
        owner = now;
        for &id in &ids {
            let a = actual.state(id).unwrap();
            let b = native.state(id).unwrap();
            if a.position.map(f64::to_bits) != b.position.map(f64::to_bits)
                || a.velocity.map(f64::to_bits) != b.velocity.map(f64::to_bits)
                || a.orientation.map(f64::to_bits) != b.orientation.map(f64::to_bits)
                || a.angular_velocity.map(f64::to_bits) != b.angular_velocity.map(f64::to_bits)
            {
                t.bitwise = false;
                t.first_body_difference.get_or_insert(tick);
            }
            t.position_m = t.position_m.max(norm(a.position, b.position));
            t.velocity_m_s = t.velocity_m_s.max(norm(a.velocity, b.velocity));
            t.spin_rad_s = t
                .spin_rad_s
                .max(norm(a.angular_velocity, b.angular_velocity));
            let chord = norm(a.orientation, b.orientation)
                .min(norm(a.orientation, b.orientation.map(|x| -x)));
            t.orientation_rad = t.orientation_rad.max(4. * (0.5 * chord).min(1.).asin());
        }
        let mut reported: Vec<_> = f
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .map(|p| ((p.a, p.b), p.normal_impulse))
            .collect();
        reported.sort_by_key(|p| p.0);
        let reference: Vec<_> = native
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
            .map(|p| ((p.a.raw(), p.b.raw()), p.normal_impulse))
            .collect();
        if !reported
            .iter()
            .map(|p| p.0)
            .eq(reference.iter().map(|p| p.0))
        {
            t.exact_contact_sets = false;
            t.bitwise = false;
        }
        for (pair, impulse) in reported {
            if pair.0 == ids[0].raw()
                && impulse > 0.
                && let Some(i) = ids[1..].iter().position(|id| id.raw() == pair.1)
            {
                t.impacts[i].get_or_insert(tick);
            }
            if let Some(r) = reference.iter().find(|r| r.0 == pair) {
                t.bitwise &= impulse.to_bits() == r.1.to_bits();
                t.impulse_n_s = t.impulse_n_s.max((impulse - r.1).abs());
            }
        }
    }
    Ok(())
}
