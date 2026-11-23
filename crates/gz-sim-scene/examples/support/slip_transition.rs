// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched physical sources and independent controls for finite-skin slip transitions.
use super::sheet::{self, Incline};
use glam::{DQuat, DVec3};
use std::error::Error;
#[derive(Clone, Copy)]
pub struct Run {
    pub mu: f64,
    pub speed: f64,
    pub width: u64,
    pub transition: Option<u64>,
    pub yaw: f64,
    pub extra_mass: f64,
}
impl Run {
    pub fn sdf(self) -> String {
        let mut src = sheet::sdf(self.mu, self.speed, self.width);
        if let Some(ns) = self.transition {
            src = src.replace(
                "max_contact_step_ns=",
                &format!("max_slip_transition_step_ns=\"{ns}\" max_contact_step_ns="),
            );
        }
        if self.extra_mass != 0. {
            src = src.replacen(
                "mass=\"0.13250000000000001\"",
                &format!("mass=\"{}\"", 0.1325 + self.extra_mass),
                1,
            );
        }
        if self.yaw == 0. {
            return src;
        }
        reexpress(&src, [0., 0., self.yaw], DVec3::ZERO)
    }
    pub fn down(self) -> DVec3 {
        DQuat::from_rotation_z(self.yaw) * sheet::downhill()
    }
}
/// Express the entire physical world in a new rigid frame, including gravity,
/// the world-joint model pose, and the coordinates and velocities of every node.
pub fn reexpress(xml: &str, [roll, pitch, yaw]: [f64; 3], origin: DVec3) -> String {
    let q = DQuat::from_euler(glam::EulerRot::ZYX, yaw, pitch, roll);
    let gravity = q * DVec3::new(0., 0., -sheet::G);
    let src = xml
        .replace(
            "<gravity>0 0 -9.81</gravity>",
            &format!(
                "<gravity>{} {} {}</gravity>",
                gravity.x, gravity.y, gravity.z
            ),
        )
        .replace(
            "<model name=\"gripper\">",
            &format!(
                "<model name=\"gripper\"><pose>{} {} {} {roll} {pitch} {yaw}</pose>",
                origin.x, origin.y, origin.z
            ),
        );
    let mut output = String::new();
    for line in src.lines() {
        if !line.contains("<particle ") {
            output.push_str(line);
        } else {
            let mut l = line.to_owned();
            for key in ["position", "velocity"] {
                let (_, tail) = l.split_once(&format!("{key}=\"")).unwrap();
                let (old, _) = tail.split_once('"').unwrap();
                let v = old
                    .split_whitespace()
                    .map(|s| s.parse::<f64>().unwrap())
                    .collect::<Vec<_>>();
                let v = q * DVec3::new(v[0], v[1], v[2])
                    + if key == "position" {
                        origin
                    } else {
                        DVec3::ZERO
                    };
                l = l.replacen(
                    &format!("{key}=\"{old}\""),
                    &format!("{key}=\"{} {} {}\"", v.x, v.y, v.z),
                    1,
                );
            }
            output.push_str(&l);
        }
        output.push('\n');
    }
    output
}
#[derive(Default, Debug)]
pub struct ResultStats {
    pub x: f64,
    pub v: f64,
    pub worst_x: f64,
    pub worst_v: f64,
    pub p: f64,
    pub l: f64,
    pub spin: f64,
    pub cuts: u64,
    pub trials: u64,
    pub slip_refinements: u64,
    pub crossing: Option<(u64, u64)>,
    pub time_ns: u128,
    pub transition_tick_ns: u128,
    pub peak_step_ns: u128,
    pub wall_p50: u128,
    pub wall_p95: u128,
}
pub fn angular(g: &Incline) -> f64 {
    let rigid = g
        .states()
        .into_iter()
        .map(|b| {
            DVec3::from(b.position)
                .cross(0.75 * DVec3::from(b.velocity))
                .z
                + 0.025 * b.angular_velocity[2]
        })
        .sum::<f64>();
    rigid
        + g.material()
            .particles()
            .iter()
            .map(|n| {
                DVec3::from(n.position)
                    .cross(DVec3::from(n.velocity) / n.inverse_mass)
                    .z
            })
            .sum::<f64>()
}
pub fn run(cfg: Run, ms: u64) -> Result<ResultStats, Box<dyn Error>> {
    let mut g = Incline::new(&cfg.sdf())?;
    let mut result = ResultStats::default();
    let (start, v0, m) = g.centroid();
    let mut last_p = m * v0;
    let l0 = angular(&g);
    let along = cfg.down();
    let mut ticks = vec![];
    for tick in 1..=ms {
        let t = std::time::Instant::now();
        g.step()?;
        let elapsed_ns = t.elapsed().as_nanos();
        ticks.push(elapsed_ns);
        let tip = -cfg.speed / sheet::acceleration(cfg.mu, cfg.speed.signum());
        if tip.is_finite() && (tip * 1000.).ceil() == tick as f64 {
            result.transition_tick_ns = elapsed_ns;
        }
        let (x, v, _) = g.centroid();
        result.x = (x - start).dot(along);
        result.v = v.dot(along);
        let expect = sheet::planar_motion(tick as f64 * 0.001, cfg.mu, cfg.speed);
        result.worst_x = result.worst_x.max((result.x - expect.0).abs());
        result.worst_v = result.worst_v.max((result.v - expect.1).abs());
        let mut on_link = DVec3::ZERO;
        for r in g.material().support_this_step() {
            result.cuts += 1;
            result.slip_refinements += u64::from(r.slip_transition_refinements);
            result.trials += u64::from(r.trajectory_trials);
            for a in r.beginning.iter().chain(&r.end) {
                for l in &a.material {
                    on_link += DVec3::from(l.on_rigid);
                }
            }
            if let Some((a, b)) = r.beginning.as_ref().zip(r.end.as_ref()) {
                for old in &a.material {
                    let Some(new) = b.material.iter().find(|q| {
                        q.site.sphere == old.site.sphere && q.site.feature == old.site.feature
                    }) else {
                        continue;
                    };
                    let (l, rhs) = (old.friction.unwrap(), new.friction.unwrap());
                    if DVec3::from(l.initial_slip).dot(along) < -1e-12
                        && DVec3::from(rhs.final_slip).dot(along) >= -1e-12
                        && result.crossing.is_none()
                    {
                        result.crossing = Some((
                            r.start.as_nanos() as u64,
                            (r.start + r.duration).as_nanos() as u64,
                        ));
                    }
                }
            }
        }
        result.p = result
            .p
            .max((m * v - last_p + on_link + DVec3::Z * m * 9.81 * 0.001).length());
        last_p = m * v;
        result.l = result.l.max((angular(&g) - l0).abs());
        result.spin = result.spin.max(
            g.states()
                .into_iter()
                .map(|s| DVec3::from(s.angular_velocity).length())
                .fold(0., f64::max),
        );
    }
    ticks.sort_unstable();
    result.time_ns = ticks.iter().sum();
    result.peak_step_ns = *ticks.last().unwrap();
    result.wall_p50 = ticks[(ticks.len() * 50).div_ceil(100) - 1];
    result.wall_p95 = ticks[(ticks.len() * 95).div_ceil(100) - 1];
    Ok(result)
}
