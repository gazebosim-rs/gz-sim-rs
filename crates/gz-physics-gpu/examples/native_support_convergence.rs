// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Vary the impact phase of an inclined sphere against a closed-form Coulomb solution.
#[allow(dead_code)]
#[path = "support/sphere_plane/reference.rs"]
mod reference;
use gz_physics_gpu::{
    SpherePlaneBatch, SpherePlaneLane as Sphere, simulation::sphere_plane_simulation,
};
use reference::V;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    time::Instant,
};
const DT: f64 = 0.001;
const TICKS: usize = 300;
const LANES: usize = 96;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
struct Scenario {
    s: Sphere,
    impact: f64,
}
fn fixtures() -> Vec<Scenario> {
    (0..LANES)
        .map(|i| {
            let phase = (i as f64 + 0.5) / (LANES as f64);
            let n = V::new(0.16 * (phase + 0.2), -0.21, 1.).normalize();
            let tan = (V::X - n * n.x).normalize();
            let mass = 0.7 + phase;
            let radius = 0.09 + 0.035 * phase;
            let impact = 0.044 + phase * DT;
            let speed = 1.7 + phase * 0.4;
            let gravity = -9.3 * n;
            let spin = (0.45 + 0.2 * phase) * n - 2. * n.cross(tan);
            let gap = speed * impact + 0.5 * 9.3 * impact * impact;
            Scenario {
                s: Sphere {
                    position: (n * (radius + gap) - 0.4 * tan).to_array(),
                    velocity: (1.1 * tan - speed * n).to_array(),
                    angular_velocity: spin.to_array(),
                    orientation: [0., 0., 0., 1.],
                    gravity: gravity.to_array(),
                    plane_normal: n.to_array(),
                    plane_offset: 0.,
                    radius,
                    inverse_mass: mass.recip(),
                    friction: [0., 0.035, 0.14, 0.75][i % 4],
                    normal_impulse: 0.,
                    tangential_impulse: [0.; 3],
                },
                impact,
            }
        })
        .collect()
}
impl Scenario {
    fn analytical(&self, t: f64) -> (f64, V, V, V) {
        let s = self.s;
        let n = V::from(s.plane_normal);
        let m = s.inverse_mass.recip();
        let v0 = V::from(s.velocity);
        let w0 = V::from(s.angular_velocity);
        let g = V::from(s.gravity);
        let arm = -s.radius * n;
        let free_velocity = v0 + g * t;
        if t < self.impact {
            return (
                n.dot(V::from(s.position) + v0 * t + 0.5 * g * t * t) - s.radius,
                free_velocity,
                w0,
                V::ZERO,
            );
        }
        let jn = -m * n.dot(free_velocity);
        let surface = v0 + w0.cross(arm);
        let slip = surface - n * n.dot(surface);
        let tangent = -slip.normalize() * (s.friction * jn).min(m * slip.length() / 3.5);
        let j = jn * n + tangent;
        (
            0.,
            free_velocity + j / m,
            w0 + arm.cross(j) / (0.4 * m * s.radius * s.radius),
            j,
        )
    }
}
#[derive(Default)]
struct Track {
    first: Option<usize>,
    penetration: f64,
    predicted_motion: f64,
    worst_continuous_gap: f64,
    worst_speed: f64,
    impulse: V,
    final_velocity: f64,
    final_spin: f64,
    final_impulse: f64,
    final_gap: f64,
}
impl Track {
    fn sample(&mut self, f: &Scenario, p: Sphere, tick: usize, substeps: usize) {
        let n = V::from(p.plane_normal);
        let gap = n.dot(V::from(p.position)) - p.radius;
        let (want_gap, v, w, j) = f.analytical(tick as f64 * DT);
        self.penetration = self.penetration.max(-gap);
        self.worst_continuous_gap = self.worst_continuous_gap.max((gap - want_gap).abs());
        self.worst_speed = self.worst_speed.max(V::from(p.velocity).distance(v));
        if p.normal_impulse > 1e-6 {
            self.first.get_or_insert(tick);
        }
        self.impulse += p.normal_impulse * n + V::from(p.tangential_impulse);
        let h = DT / substeps.max(1) as f64;
        let bound = if substeps == 0 {
            1e-10
        } else {
            let speed = n
                .dot(V::from(f.s.velocity) + f.impact * V::from(f.s.gravity))
                .abs();
            speed * h + 2. * 9.3 * h * h + 1.1e-6
        };
        // Incoming motion at detection estimates one discrete substep. Rapier's
        // compliant iterative contact can retain or add penetration afterwards.
        // Record every exceedance instead of imposing a hard geometry guarantee.
        self.predicted_motion = bound;
        if substeps == 0 {
            assert!(self.penetration < 1e-10);
        }
        if tick == TICKS {
            self.final_velocity = V::from(p.velocity).distance(v);
            self.final_spin = V::from(p.angular_velocity).distance(w);
            self.final_impulse = self.impulse.distance(j);
            self.final_gap = gap.abs();
            let event_error = (self.first.unwrap() as f64 * DT - f.impact).abs();
            assert!(
                event_error <= DT + 1.5 * h,
                "event observation latency {event_error}"
            );
            // The exact normal-plane solution bounds velocity response. The native friction
            // impulse radius can differ by the measured geometric penetration divided by r.
            let lever = self.penetration / f.s.radius;
            let base = if substeps == 0 {
                1e-8
            } else {
                1e-8 + 8. * lever * j.length() * f.s.inverse_mass
            };
            assert!(
                self.final_velocity <= base
                    && self.final_impulse <= base / f.s.inverse_mass
                    && self.final_spin <= base * 3. / f.s.radius,
                "final response outside contact geometry bound, substeps={substeps} v={} w={} J={} bound={base}",
                self.final_velocity,
                self.final_spin,
                self.final_impulse
            );
        }
    }
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER NEW.csv")?;
    let file = args.next().ok_or("provide new CSV")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    let mut csv = BufWriter::new(File::create_new(file)?);
    writeln!(
        csv,
        "solver,native_substeps,id,mu,impact_s,first_report_s,peak_penetration_m,peak_continuous_gap_error_m,max_velocity_error_including_event_m_s,final_velocity_error_m_s,final_spin_error_rad_s,final_cumulative_impulse_error_Ns,final_abs_gap_m,one_substep_motion_estimate_m"
    )?;
    let seeds = fixtures();
    for substeps in [0, 4, 8, 16, 32, 64] {
        let mut tracks = (0..LANES).map(|_| Track::default()).collect::<Vec<_>>();
        let start = Instant::now();
        if substeps == 0 {
            let mut sim = sphere_plane_simulation(&adapter, seeds.iter().map(|s| s.s).collect())?;
            for t in 1..=TICKS {
                sim.step_exact(1)?;
                for ((tr, seed), &p) in tracks
                    .iter_mut()
                    .zip(&seeds)
                    .zip(sim.world().resource::<SpherePlaneBatch>().lanes())
                {
                    tr.sample(seed, p, t, 0);
                }
            }
        } else {
            let mut worlds = seeds
                .iter()
                .map(|s| reference::Native::with_substeps(s.s, substeps))
                .collect::<Vec<_>>();
            for t in 1..=TICKS {
                for ((tr, seed), w) in tracks.iter_mut().zip(&seeds).zip(&mut worlds) {
                    w.step();
                    tr.sample(seed, w.state, t, substeps);
                }
            }
        }
        let elapsed = start.elapsed();
        let mode = if substeps == 0 { "GPU" } else { "Rapier" };
        for (id, (f, tr)) in seeds.iter().zip(&tracks).enumerate() {
            writeln!(
                csv,
                "{mode},{substeps},{id},{},{},{},{},{},{},{},{},{},{},{}",
                f.s.friction,
                f.impact,
                tr.first.unwrap() as f64 * DT,
                tr.penetration,
                tr.worst_continuous_gap,
                tr.worst_speed,
                tr.final_velocity,
                tr.final_spin,
                tr.final_impulse,
                tr.final_gap,
                tr.predicted_motion
            )?;
        }
        let exceedances = tracks
            .iter()
            .filter(|t| t.penetration > t.predicted_motion)
            .count();
        let maximum_ratio = tracks
            .iter()
            .map(|t| t.penetration / t.predicted_motion)
            .fold(0., f64::max);
        let maxima = |get: fn(&Track) -> f64| tracks.iter().map(get).fold(0., f64::max);
        println!(
            "solver={mode} substeps={substeps} seeds={LANES} ticks={TICKS} one_substep_prediction_exceeded={exceedances} max_ratio={maximum_ratio:.4} penetration_um={:.4} peak_gap_error_um={:.4} final_speed_m_s={:.6e} final_spin_rad_s={:.6e} final_impulse_Ns={:.6e} worst_sampled_speed_error_including_impact_m_s={:.5} total_wall_ms={:.3}",
            1e6 * maxima(|t| t.penetration),
            1e6 * maxima(|t| t.worst_continuous_gap),
            maxima(|t| t.final_velocity),
            maxima(|t| t.final_spin),
            maxima(|t| t.final_impulse),
            maxima(|t| t.worst_speed),
            elapsed.as_secs_f64() * 1000.
        );
    }
    Ok(())
}
