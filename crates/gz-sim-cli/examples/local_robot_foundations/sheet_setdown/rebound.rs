// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate a late drop from the exact material committed half a millisecond after the deck stops.
use super::*;
use gz_physics_xpbd::{Boundary, Particle, SoftBody, step_coupled_rigid};
use std::time::Duration;

#[path = "rebound_view.rs"]
mod view;

const CUT_NS: u64 = 500_000;
const MS: u64 = 1_000_000;
const HORIZON_MS: u64 = 80;
#[derive(Debug, Serialize)]
struct Sample {
    time_ns: u64,
    gap_m: f64,
    pz_ns: f64,
    normal_kinetic_j: f64,
    com_normal_kinetic_j: f64,
    table_ns: f64,
    positions: Vec<[f64; 3]>,
}
fn sample(p: &[Particle], z: f64, t: u64, impulse: f64, capture: bool) -> Sample {
    let mass: f64 = p.iter().map(|p| 1. / p.inverse_mass).sum();
    let pz: f64 = p.iter().map(|p| p.velocity[2] / p.inverse_mass).sum();
    Sample {
        time_ns: t,
        gap_m: p
            .iter()
            .map(|p| p.position[2] - arm_sheet::RADIUS - z)
            .fold(f64::INFINITY, f64::min),
        pz_ns: pz,
        normal_kinetic_j: p
            .iter()
            .map(|p| 0.5 * p.velocity[2].powi(2) / p.inverse_mass)
            .sum(),
        com_normal_kinetic_j: 0.5 * pz * pz / mass,
        table_ns: impulse,
        positions: if capture {
            p.iter().map(|p| p.position).collect()
        } else {
            Vec::new()
        },
    }
}

struct Seed {
    material: SoftBody,
    bench: Boundary,
    top: f64,
    time_ns: u128,
}
fn seed(r: &Recording) -> Result<Seed> {
    let id = entity_id(&r.sim, "default::source::body")?;
    let deck = state(&r.sim, id)?;
    let half = arm_cable::half_size(&r.sdf, "source", "body")?;
    if deck.velocity != [0.; 3]
        || deck.angular_velocity != [0.; 3]
        || deck.orientation != [0., 0., 0., 1.]
    {
        return Err("expected a stopped, upright receiving bench".into());
    }
    let material = gz_sim_scene::deformable::material_snapshot(&r.sim, "sheet")
        .ok_or("missing committed material")?;
    let bench = Boundary::TranslatingCuboid {
        center: deck.position,
        half_extents: half.to_array(),
        orientation: deck.orientation,
        velocity: deck.velocity,
    };
    Ok(Seed {
        material,
        bench,
        top: deck.position[2] + half.z,
        time_ns: r.sim.committed_update_info()?.sim_time.as_nanos(),
    })
}
#[derive(Clone, Copy, Serialize)]
struct Clock {
    initial: u64,
    after: u64,
    switch_ms: u64,
}
impl Clock {
    fn uniform(rate: u64) -> Self {
        Self {
            initial: rate,
            after: rate,
            switch_ms: HORIZON_MS,
        }
    }
    fn label(self) -> String {
        if self.initial == self.after {
            self.initial.to_string()
        } else {
            format!("{}-{}-at-{}ms", self.initial, self.after, self.switch_ms)
        }
    }
}

fn trial(s: &Seed, clock: Clock) -> Result<(Value, Vec<Sample>)> {
    let mut b = s.material.clone();
    let mut rows = vec![sample(b.particles(), s.top, 0, 0., true)];
    let energy0 = b.mechanical_energy()?.total();
    let (mut first, mut first_ms, mut budget) = (None, None, 0u64);
    let mut peak = (0., 0);
    let (mut departed, mut returned) = (false, false);
    let (mut impulse, mut residual, mut gain) = (0., 0_f64, 0_f64);
    let mut before_energy = energy0;
    for ms in 0..HORIZON_MS {
        let rate = if ms < clock.switch_ms {
            clock.initial
        } else {
            clock.after
        };
        let h = MS / rate;
        let mut public_impulse = 0.;
        for step in 1..=rate {
            let t = ms * MS + step * h;
            step_coupled_rigid(&mut [&mut b], Duration::from_nanos(h), &[s.bench], &[None])?;
            budget += 1;
            let j = -b.last_boundary_impulses()[0].linear[2];
            public_impulse += j;
            impulse += j;
            let row = sample(b.particles(), s.top, t, public_impulse, step == rate);
            residual = residual
                .max((row.pz_ns - rows[0].pz_ns - (impulse - WEIGHT * t as f64 * 1e-9)).abs());
            let energy = b.mechanical_energy()?;
            let total = energy.total();
            gain = gain.max(total - before_energy);
            if j > 1e-12 && first.is_none() {
                first = Some(
                    json!({"time_ns":t,"first_impulse_ns":j,"before_j":before_energy,"after_j":total,
                    "normal_kinetic_j":row.normal_kinetic_j,"com_normal_kinetic_j":row.com_normal_kinetic_j}),
                );
            }
            if first.is_some() {
                departed |= row.gap_m > 0.00005;
                returned |= departed && row.gap_m < 1e-6;
                if !returned && row.gap_m > peak.0 {
                    peak = (row.gap_m, t);
                }
            }
            if step == rate {
                if ms == 0 {
                    first_ms = Some(json!({"normal_kinetic_j":row.normal_kinetic_j,
                    "com_normal_kinetic_j":row.com_normal_kinetic_j,"momentum_ns":row.pz_ns,
                    "upward_table_impulse_ns":public_impulse,"mechanical_j":total,"distance_j":energy.distance,"bending_j":energy.bending}));
                }
                rows.push(row);
            }
            before_energy = total;
        }
    }
    if !departed || residual > 1e-10 || gain > 1e-8 {
        return Err(format!("unqualified outgoing sheet: departed={departed}, momentum={residual}, energy gain={gain}").into());
    }
    Ok((
        json!({"clock":clock,"native_steps":budget,"initial_energy_j":energy0,"first":first,"at_one_ms":first_ms,
        "first_rebound_m":peak.0,"peak_time_ns":peak.1,"table_impulse_ns":impulse,"max_vertical_momentum_residual_ns":residual,"max_step_energy_gain_j":gain}),
        rows,
    ))
}
/// A single particle with the same total mass and lowest incoming skin, against
/// the unchanged receiver. This isolates the normal impact law from sheet elasticity.
fn point_control(seed: &Seed, rate: u64) -> Result<Value> {
    let particle = *seed
        .material
        .particles()
        .iter()
        .min_by(|a, b| a.position[2].total_cmp(&b.position[2]))
        .ok_or("empty material")?;
    let mass: f64 = seed
        .material
        .particles()
        .iter()
        .map(|p| 1. / p.inverse_mass)
        .sum();
    let mut point = SoftBody::new(
        vec![Particle {
            inverse_mass: 1. / mass,
            ..particle
        }],
        vec![],
        gz_physics_xpbd::SolverConfig {
            self_contact: false,
            substeps: 1,
            ..seed.material.config()
        },
    )?;
    let dt = Duration::from_nanos(MS / rate);
    let (mut impulse, mut first, mut rebound) = (0., None, 0_f64);
    for i in 1..=3 * rate {
        step_coupled_rigid(&mut [&mut point], dt, &[seed.bench], &[None])?;
        let j = -point.last_boundary_impulses()[0].linear[2];
        impulse += j;
        if j > 1e-12 && first.is_none() {
            first = Some(i * MS / rate);
        }
        if first.is_some() {
            rebound = rebound.max(point.particles()[0].position[2] - arm_sheet::RADIUS - seed.top);
        }
    }
    let velocity = point.particles()[0].velocity[2];
    let expected = mass * (velocity - particle.velocity[2] + 0.003 * 9.81);
    if first.is_none()
        || rebound > 1e-10
        || velocity.abs() > 1e-9
        || (impulse - expected).abs() > 1e-11
    {
        return Err("single-particle inelastic impact failed its momentum or rest check".into());
    }
    Ok(
        json!({"rate":rate,"first_impulse_time_ns":first,"rebound_m":rebound,"final_vz_m_s":velocity,"total_table_impulse_ns":impulse,"momentum_residual_ns":impulse-expected}),
    )
}

pub(crate) fn run(out: &Path, incoming: usize) -> Result<()> {
    if ![16, 64].contains(&incoming) {
        return Err("incoming rate must be 16 or 64".into());
    }
    let xml = world(incoming, true, false)?;
    let mut r = Recording::new(&xml, "rebound.sdf", "default")?;
    let mut cut = None;
    let mut observed = Vec::new();
    let mut js = 0.;
    episode_observing(&mut r, Mode::DropLate, &mut None, |r, t| {
        if t == CUT_NS {
            cut = Some(seed(r)?);
        }
        let Some(s) = &cut else { return Ok(()) };
        if t <= CUT_NS + HORIZON_MS * MS {
            let cloth = arm_sheet::cloth(&r.sim)?;
            let parts = cloth.last_boundary_impulses();
            if parts[..2]
                .iter()
                .any(|p| DVec3::from(p.linear).length() > 1e-14)
            {
                return Err("pads loaded inside the isolated landing interval".into());
            }
            if t > CUT_NS {
                js -= parts[2].linear[2];
            }
            if (t - CUT_NS).is_multiple_of(MS) {
                observed.push(sample(cloth.particles(), s.top, t - CUT_NS, js, true));
                js = 0.;
            }
        }
        Ok(())
    })?;
    let s = cut.ok_or("receiver did not reach the committed falling state")?;
    let mut results = Vec::new();
    let mut lanes = Vec::new();
    let clocks = [16, 64, 125, 250, 500, 1000, 2000, 4000]
        .into_iter()
        .map(Clock::uniform)
        .chain([
            Clock {
                initial: 16,
                after: 1000,
                switch_ms: 1,
            },
            Clock {
                initial: 1000,
                after: 16,
                switch_ms: 1,
            },
            Clock {
                initial: 1000,
                after: 16,
                switch_ms: 4,
            },
        ]);
    for clock in clocks {
        let (mut result, rows) = trial(&s, clock)?;
        let error = rows
            .iter()
            .zip(&observed)
            .flat_map(|(a, b)| a.positions.iter().zip(&b.positions))
            .map(|(a, b)| DVec3::from(*a).distance((*b).into()))
            .fold(0., f64::max);
        let impulse_error = rows
            .iter()
            .zip(&observed)
            .map(|(a, b)| (a.table_ns - b.table_ns).abs())
            .fold(0., f64::max);
        if clock.initial == incoming as u64
            && clock.initial == clock.after
            && (rows.len() != observed.len() || error > 1e-12 || impulse_error > 1e-13)
        {
            return Err("matched isolated branch disagrees with full Bevy robot and bench".into());
        }
        result["max_displacement_vs_direct_m"] = json!(error);
        result["max_ms_impulse_error_vs_direct_ns"] = json!(impulse_error);
        write_json(
            &out.join(format!("trace-{}.json", clock.label())),
            &serde_json::to_value(&rows)?,
        )?;
        if clock.initial == clock.after && [16, 64, 4000].contains(&clock.initial) {
            lanes.push(rows);
        }
        results.push(result);
    }
    view::save(out, &s, &lanes)?;
    write_json(&out.join("direct.json"), &serde_json::to_value(observed)?)?;
    let point = [16, 64, 4000]
        .map(|rate| point_control(&s, rate))
        .into_iter()
        .collect::<Result<Vec<_>>>()?;
    let Boundary::TranslatingCuboid {
        center,
        half_extents,
        orientation,
        velocity,
    } = s.bench
    else {
        unreachable!("bench constructed from finite receiver")
    };
    write_json(
        &out.join("comparison.json"),
        &json!({"incoming":incoming,"rates":results,"cut_ns_after_deck_stopped":CUT_NS,"bench":{"center":center,"half_extents":half_extents,"orientation":orientation,"velocity":velocity},"physical_seed_time_ns":s.time_ns,"point_control":point,"bench_top_m":s.top,"incoming_particles":s.material.particles().iter().map(|p|json!([p.position,p.velocity,p.inverse_mass])).collect::<Vec<_>>(),"distance_constraints":s.material.constraints().iter().map(|d|json!([d.a,d.b,d.rest_length,d.compliance])).collect::<Vec<_>>(),"dihedrals":s.material.bending_constraints().iter().map(|d|json!([d.vertices,d.rest_angle,d.compliance])).collect::<Vec<_>>()}),
    )
}

#[cfg(test)]
#[path = "rebound_tests.rs"]
mod tests;
