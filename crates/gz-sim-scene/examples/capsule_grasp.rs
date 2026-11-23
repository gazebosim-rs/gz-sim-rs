// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Ordinary Bevy force-driven capsule pinch, lift, lateral carry and release.
//! With `--sweep`, vary real closing force and the soft clock. CSV rows report
//! consecutive held ticks, wrench closure and whole-system momentum balance.
#[path = "support/capsule_grasp.rs"]
mod grasp;
use glam::{DQuat, DVec3};
use grasp::*;
use std::error::Error;
fn momentum(r: &Grasp) -> DVec3 {
    r.jaws()
        .iter()
        .map(|b| JAW_MASS * DVec3::from(b.velocity))
        .sum::<DVec3>()
        + r.cloth()
            .particles()
            .iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum::<DVec3>()
}
fn measure(mode: Mode, force: f64, n: u32, step: f64) -> Result<(), Box<dyn Error>> {
    let xml = WORLD
        .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
        .replace("<max_step_size>0.001", &format!("<max_step_size>{step}"));
    let mut run = Grasp::new(mode, xml, force)?;
    let dt = run.sim.step_duration().as_secs_f64();
    let initial = run.center();
    let mut predicted = momentum(&run);
    let mut max_dp = 0f64;
    let mut hold = DVec3::ZERO;
    let mut ticks = 0;
    let mut support = 0.;
    let mut tangential = 0.;
    let mut normal = 0.;
    let mut com_lag = 0.;
    let mut hold_slip = 0f64;
    let (mut weighted_slip, mut hold_jn, mut accumulated_slip) = (0., 0., 0.);
    let mut both = 0;
    let mut min_gap = f64::INFINITY;
    let mut min_vertex = f64::INFINITY;
    let mut peak_friction_ratio = 0f64;
    let mut bad = 0f64;
    let mut completed = 0;
    let mut retries = 0;
    let mut refine = 0;
    while run.sim.elapsed().as_secs_f64() + 0.1 * dt < END {
        let commands = run.step()?;
        let t = run.sim.elapsed().as_secs_f64();
        let c = run.center();
        let s = run.cloth();
        let j = s.last_boundary_impulses();
        let jaws = run.jaws();
        let points = jaws.map(|b| {
            (
                DVec3::from(b.position),
                DQuat::from_array(b.orientation) * DVec3::Z,
            )
        });
        for (center, axis) in points {
            for p in s.particles() {
                let r = DVec3::from(p.position) - center;
                let gap = (r - axis * r.dot(axis).clamp(-HALF_LENGTH, HALF_LENGTH)).length()
                    - RADIUS
                    - SKIN;
                min_vertex = min_vertex.min(gap);
            }
        }
        // Distances between the two facing caps are a conservative-grip
        // diagnostic below. Independent segment distances in the test also
        // include every point on both barrel centerlines.
        let a = points[0].0 + points[0].1 * HALF_LENGTH;
        let b = points[1].0 - points[1].1 * HALF_LENGTH;
        min_gap = min_gap.min(a.distance(b) - 2. * RADIUS);
        retries += s.last_sweep_refinements();
        refine = refine.max(s.last_sweep_refinements());
        let mut sums = [(DVec3::ZERO, DVec3::ZERO); 2];
        for l in s.last_capsule_material_loads() {
            sums[l.boundary].0 += DVec3::from(l.linear);
            sums[l.boundary].1 += DVec3::from(l.angular);
        }
        for l in s.last_capsule_friction_loads() {
            sums[l.boundary].0 += DVec3::from(l.linear);
            sums[l.boundary].1 += DVec3::from(l.angular);
            peak_friction_ratio = peak_friction_ratio
                .max(l.tangent_impulse_sum / l.normal_impulse_sum.max(f64::MIN_POSITIVE));
            bad = bad.max(
                (DVec3::from(l.linear)
                    + l.nodal.iter().map(|(_, j)| DVec3::from(*j)).sum::<DVec3>())
                .length(),
            );
        }
        if mode != Mode::Particles {
            for (i, (l, a)) in sums.into_iter().enumerate() {
                bad = bad
                    .max(l.distance(j[i].linear.into()))
                    .max(a.distance(j[i].angular.into()));
            }
        }
        predicted += dt
            * (commands[0].0 + commands[1].0 - DVec3::Z * (GRAVITY * (2. * JAW_MASS + CLOTH_MASS)))
            - DVec3::from(j[2].linear);
        max_dp = max_dp.max(predicted.distance(momentum(&run)));
        completed += 1;
        if t >= HOLD.0 && t <= HOLD.1 {
            hold += c;
            ticks += 1;
            support -= j[0].linear[2] + j[1].linear[2];
            tangential -= s
                .last_capsule_friction_loads()
                .iter()
                .map(|l| l.linear[2])
                .sum::<f64>();
            normal += 0.5 * (j[0].linear[1] - j[1].linear[1]);
            for load in s.last_capsule_friction_loads() {
                weighted_slip += load.normal_weighted_slip;
                hold_jn += load.normal_impulse_sum;
                accumulated_slip += load.slip_distance;
            }
            hold_slip = s
                .last_capsule_friction_loads()
                .iter()
                .map(|l| DVec3::from(l.last_slip_velocity).length())
                .fold(hold_slip, f64::max);
            com_lag += (0.5 * (points[0].0.z + points[1].0.z) - 0.081) - (c.z - initial.z);
            both += usize::from(j[0].linear[1] > 1e-9 && j[1].linear[1] < -1e-9);
        }
    }
    hold /= ticks as f64;
    let duration = ticks as f64 * dt;
    let end = run.center();
    println!(
        "{mode:?},{force:.6},{dt:.7},{n},{completed},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{both},{ticks},{:.9},{:.9},{:.9},{:.9},{peak_friction_ratio:.10},{max_dp:e},{bad:e},{retries},{refine},{hold_slip:e},{:.9e},{:.9e}",
        hold.x - initial.x,
        hold.z - initial.z,
        com_lag / ticks as f64,
        support / duration,
        tangential / duration,
        normal / duration,
        CLOTH_MASS * GRAVITY,
        end.x,
        end.z,
        min_gap,
        min_vertex,
        weighted_slip / f64::max(hold_jn, f64::MIN_POSITIVE),
        accumulated_slip
    );
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "mode,squeeze_n,public_s,substeps,ticks,hold_dx_m,hold_dz_m,hold_jaw_com_lift_difference_m,hold_support_n,hold_friction_z_n,hold_jaw_normal_n,weight_n,bilateral_ticks,hold_ticks,final_x_m,final_z_m,min_facing_gap_m,min_vertex_to_jaw_clearance_m,max_coulomb_ratio,max_p_residual_n_s,max_attribution_residual,total_clock_doublings,max_clock_doublings,max_hold_skin_slip_m_s,hold_normal_weighted_slip_m_s,hold_summed_slip_distance_m"
    );
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if let [s, force, sub, dt] = args.as_slice()
        && s == "--case"
    {
        return measure(Mode::Closed, force.parse()?, sub.parse()?, dt.parse()?);
    }
    if args == ["--sweep"] {
        for f in [
            0.003, 0.008, 0.012, 0.015, 0.018, 0.025, 0.040, 0.055, 0.085,
        ] {
            measure(Mode::Closed, f, 4, 0.001)?;
        }
        for n in [1, 2, 8, 16] {
            measure(Mode::Closed, SQUEEZE, n, 0.001)?;
        }
        measure(Mode::Closed, SQUEEZE, 4, 0.0005)?;
        return Ok(());
    }
    if !args.is_empty() {
        return Err("expected --sweep or --case FORCE_N SUBSTEPS DT_S".into());
    }
    for mode in [Mode::Closed, Mode::Open, Mode::NoFriction, Mode::Particles] {
        measure(mode, SQUEEZE, 4, 0.001)?;
    }
    Ok(())
}
