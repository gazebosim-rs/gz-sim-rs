// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    fixture::{fixture, physical, scaled, tangent_gravity},
    reference::{self, Native, Q, V},
};
use gz_physics_gpu::{
    SolidSphere as Body, SupportedSpherePairBatch, SupportedSpherePairLane as Lane,
};
use std::{error::Error, io::Write};
const BASE: u32 = 600;
#[derive(Default)]
struct Stats {
    q2: f64,
    n: usize,
    max_p: f64,
    max_v: f64,
    max_spin: f64,
    max_j: f64,
    max_support: f64,
}
impl Stats {
    fn state(&mut self, a: Body, b: Body) {
        let (a_q, b_q) = (Q::from_array(a.orientation), Q::from_array(b.orientation));
        self.q2 += (a_q - b_q)
            .length_squared()
            .min((a_q + b_q).length_squared());
        self.n += 1;
        self.max_p = self
            .max_p
            .max(V::from(a.position).distance(V::from(b.position)));
        self.max_v = self
            .max_v
            .max(V::from(a.velocity).distance(V::from(b.velocity)));
        self.max_spin = self
            .max_spin
            .max(V::from(a.angular_velocity).distance(V::from(b.angular_velocity)));
    }
    fn rms(&self) -> f64 {
        (self.q2 / self.n as f64).sqrt()
    }
}
fn exact(s: &Lane) -> gz_physics_gpu::SpherePairLane {
    let mut p = s.pair;
    p.gravity = tangent_gravity(s).to_array();
    reference::exact_interval(&mut p, BASE as f64 * 0.001);
    p
}
fn integrated_pair(p: &gz_physics_gpu::SpherePairLane) -> V {
    p.normal_impulse * V::from(p.contact_normal) + V::from(p.tangential_impulse)
}
fn weight(s: &Lane, k: usize) -> f64 {
    let mass = 1. / [s.pair.a, s.pair.b][k].inverse_mass;
    -mass * V::from(s.plane_normal).dot(V::from(s.pair.gravity)) * f64::from(BASE) * 0.001
}
fn report(file: &mut impl Write, kind: &str, f: u32, s: &Stats) -> std::io::Result<()> {
    println!(
        "convergence {kind} dt=1/{f} ms, {} bodies, q-rms {:.5e}, max p {:.4e} m, v {:.4e} m/s, angular {:.4e} rad/s, summed pair J {:.4e} N s, summed support {:.4e} N s",
        s.n,
        s.rms(),
        s.max_p,
        s.max_v,
        s.max_spin,
        s.max_j,
        s.max_support
    );
    writeln!(
        file,
        "{kind},{f},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
        s.n,
        s.rms(),
        s.max_p,
        s.max_v,
        s.max_spin,
        s.max_j,
        s.max_support
    )
}
pub fn run(adapter: &str, file: &mut impl Write) -> Result<(), Box<dyn Error>> {
    writeln!(
        file,
        "method,refinement,bodies,quaternion_chord_rms,max_position_m,max_velocity_m_s,max_angular_rad_s,max_integrated_pair_impulse_N_s,max_integrated_support_N_s"
    )?;
    let set = fixture(128);
    let controls: Vec<_> = set.iter().map(exact).collect();
    let mut coarse = 0.;
    let mut previous = f64::INFINITY;
    for f in [1u32, 2, 4, 8] {
        // The public kernel clock stays 1 ms. Scaling v and spin by 1/f and g
        // by 1/f², then taking f steps, represents physical dt=1/f ms with the
        // same mass, length, friction and restitution. Multiply impulses by f.
        let mut b = SupportedSpherePairBatch::new(
            adapter,
            set.iter().copied().map(|x| scaled(x, f)).collect(),
        )?;
        let mut sums = vec![(V::ZERO, [0.; 2]); set.len()];
        for _ in 0..BASE * f {
            b.step()?;
            for (s, delta) in b.lanes().iter().zip(&mut sums) {
                delta.0 += f64::from(f) * integrated_pair(&s.pair);
                for (k, r) in delta.1.iter_mut().enumerate() {
                    *r += f64::from(f) * s.support_normal_impulses[k];
                }
            }
        }
        let mut m = Stats::default();
        for (i, s) in b.lanes().iter().enumerate() {
            let e = controls[i];
            m.state(physical(s.pair.a, f), e.a);
            m.state(physical(s.pair.b, f), e.b);
            m.max_j = m.max_j.max(sums[i].0.distance(integrated_pair(&e)));
            for k in 0..2 {
                m.max_support = m.max_support.max((sums[i].1[k] - weight(&set[i], k)).abs());
            }
        }
        report(file, "GPU exact time scaling", f, &m)?;
        assert!(m.max_p < 2e-8 && m.max_v < 2e-8 && m.max_j < 2e-8 && m.max_support < 2e-8);
        if f == 1 {
            coarse = m.rms();
        } else {
            // Normalized quaternion Euler has local phase error O((h*ω)^3)
            // and global phase error O(h²) for each constant-spin arc.
            assert!(
                m.rms() < previous / 3.5 && m.rms() > previous / 4.6,
                "orientation refinement outside its second-order band"
            );
        }
        previous = m.rms();
    }
    println!(
        "measured GPU quaternion temporal order {:.4}",
        (coarse / previous).log2() / 3.
    );
    for f in [1u32, 4, 16, 64] {
        let (mut all, mut common) = (Stats::default(), Stats::default());
        for (i, s) in set.iter().enumerate().step_by(5) {
            let normal = V::from(s.plane_normal);
            let mut n = Native::with_support(s.pair, normal, s.plane_offset, f as usize);
            let mut sum = V::ZERO;
            let mut supports = [0.; 2];
            for _ in 0..BASE {
                n.step();
                sum += n.impulse_b;
                for (x, j) in supports.iter_mut().zip(n.support_impulses) {
                    *x += j;
                }
            }
            let e = controls[i];
            let cap = V::from(e.tangential_impulse).length()
                - s.pair.friction * e.normal_impulse / (1. + s.pair.restitution);
            let record = |m: &mut Stats| {
                m.state(n.state.a, e.a);
                m.state(n.state.b, e.b);
                m.max_j = m.max_j.max(sum.distance(integrated_pair(&e)));
                for (j, observed) in supports.iter().enumerate() {
                    m.max_support = m.max_support.max((observed - weight(s, j)).abs());
                }
            };
            record(&mut all);
            if cap < 1e-8 {
                record(&mut common);
            }
        }
        report(file, "Rapier all friction laws", f, &all)?;
        report(file, "Rapier shared friction law", f, &common)?;
        if f == 64 {
            assert!(common.max_p < 0.012 && common.max_v < 0.025 && common.max_support < 0.003);
        }
    }
    Ok(())
}

/// Plot the measured orientation convergence and the persistent mixed-contact
/// law difference. Tick refinement is plotted as log2(1ms / physical dt).
pub fn plot(dir: &std::path::Path) -> Result<(), Box<dyn Error>> {
    use plotters::prelude::*;
    let data = std::fs::read_to_string(dir.join("convergence.csv"))?;
    let mut rows = Vec::new();
    for line in data.lines().skip(1) {
        let cols: Vec<_> = line.split(',').collect();
        rows.push((
            cols[0],
            cols[1].parse::<f64>()?.log2(),
            cols[3].parse::<f64>()?,
            cols[4].parse::<f64>()?,
        ));
    }
    let path = dir.join("convergence.svg");
    let root = SVGBackend::new(&path, (1140, 550)).into_drawing_area();
    root.fill(&WHITE)?;
    let panel = root.titled(
        "Two spinning spheres on one frictionless plane",
        ("sans-serif", 29),
    )?;
    let panels = panel.split_evenly((1, 2));
    for (i, p) in panels.iter().enumerate() {
        let (title, unit, bottom) = if i == 0 {
            (
                "Orientation against exact event solution",
                "RMS quaternion chord",
                -8.,
            )
        } else {
            (
                "Center against exact event solution",
                "Maximum position error (m)",
                -15.,
            )
        };
        let mut chart = ChartBuilder::on(p)
            .caption(title, ("sans-serif", 18))
            .margin(14)
            .x_label_area_size(50)
            .y_label_area_size(73)
            .build_cartesian_2d(-0.12f64..6.13, bottom..0.)?;
        chart
            .configure_mesh()
            .x_desc("Physical step (milliseconds)")
            .x_labels(7)
            .y_desc(unit)
            .y_labels(8)
            .x_label_formatter(&|v| {
                let k = v.round() as u32;
                if (*v - v.round()).abs() > 0.001 {
                    String::new()
                } else if k == 0 {
                    "1".into()
                } else {
                    format!("1/{}", 1u32 << k)
                }
            })
            .y_label_formatter(&|v| format!("10^{v:.0}"))
            .draw()?;
        for (method, label, color) in [
            (
                "GPU exact time scaling",
                "GPU • time-scaled",
                RGBColor(11, 122, 132),
            ),
            (
                "Rapier shared friction law",
                "Rapier • common law",
                RGBColor(199, 99, 26),
            ),
            (
                "Rapier all friction laws",
                "Rapier • all lanes",
                RGBColor(113, 69, 155),
            ),
        ] {
            let points: Vec<_> = rows
                .iter()
                .filter(|r| r.0 == method)
                .map(|r| (r.1, if i == 0 { r.2 } else { r.3 }.log10()))
                .collect();
            chart
                .draw_series(LineSeries::new(points.clone(), color.stroke_width(3)))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(3))
                });
            chart.draw_series(
                points
                    .into_iter()
                    .map(|p| Circle::new(p, 4, color.filled())),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::LowerRight)
            .background_style(WHITE.mix(0.9))
            .border_style(RGBColor(220, 224, 228))
            .label_font(("sans-serif", 12))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
