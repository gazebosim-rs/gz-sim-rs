// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare three incident robot links at matched physical times and time their ordinary Bevy steps.
use glam::DVec3;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_engine};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::{Duration, Instant},
};

const STOCK: &str = include_str!("../../../examples/worlds/local_fast_obstacle.sdf");
const LANES: [&str; 3] = ["hit", "graze", "clear"];
const END_US: u64 = 20_000;
const POSITION_TOL_M: f64 = 0.002;
const PENETRATION_TOL_M: f64 = 0.0005;
const IMPULSE_REL_TOL: f64 = 0.05;
const FIRST_DIFFERENCE_M: f64 = 0.000001;
const SPIN_TOL_RAD_S: f64 = 1e-7;
const CLEAR_GAP_M: f64 = 0.005;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Default, Clone, Copy)]
struct Sample {
    position: DVec3,
    velocity: DVec3,
    angular_speed: f64,
    step_impulse: f64,
    total_impulse: f64,
    gap: f64,
}
struct Run {
    step_us: u64,
    rows: Vec<[Sample; 3]>,
    times_ns: Vec<u64>,
}

#[allow(deprecated)]
fn id(sim: &Simulation, name: &str) -> Result<BodyId> {
    Ok(sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<BodyHandle>())
        .ok_or_else(|| format!("missing {name}"))?
        .id())
}

fn run(step_us: u64) -> Result<Run> {
    let mut sim = load_world_with_engine(
        STOCK,
        "fast-obstacle.sdf",
        "default",
        Duration::from_micros(step_us),
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut bodies = Vec::new();
    for lane in LANES {
        bodies.push((
            id(&sim, &format!("default::{lane}::tip"))?,
            id(&sim, &format!("default::{lane}_guard::rail"))?,
        ));
    }
    let mut rows = Vec::new();
    let mut times_ns = Vec::new();
    let mut totals = [0.; 3];
    for t in 0..=END_US / step_us {
        if t > 0 {
            let timer = Instant::now();
            sim.step_exact(1)?;
            times_ns.push(timer.elapsed().as_nanos() as u64);
        }
        let backend = sim.world().resource::<PhysicsBackend>();
        let native = backend
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("expected local solver")?;
        let mut impulses = [0.; 3];
        for patch in native.rigid_contact_patches() {
            let j: f64 = patch.points.iter().map(|p| p.normal_impulse).sum();
            if j > 0. {
                let lane = bodies
                    .iter()
                    .position(|&(tip, rail)| (patch.a, patch.b) == (tip.min(rail), tip.max(rail)))
                    .ok_or("positive impulse outside the three intended contact pairs")?;
                impulses[lane] += j;
            }
        }
        let mut samples = [Sample::default(); 3];
        for (i, &(tip, rail)) in bodies.iter().enumerate() {
            let state = backend.state(tip).ok_or("missing probe")?;
            let position = DVec3::from(state.position);
            let center = DVec3::from(backend.state(rail).ok_or("missing rail")?.position);
            let q = (position - center).abs() - DVec3::new(0.006, 0.35, 0.15);
            let gap = q.max(DVec3::ZERO).length() + q.max_element().min(0.) - 0.06;
            totals[i] += impulses[i];
            samples[i] = Sample {
                position,
                velocity: state.velocity.into(),
                angular_speed: DVec3::from(state.angular_velocity).length(),
                step_impulse: impulses[i],
                total_impulse: totals[i],
                gap,
            };
        }
        rows.push(samples);
    }
    Ok(Run {
        step_us,
        rows,
        times_ns,
    })
}

fn onset(run: &Run, lane: usize) -> Option<u64> {
    run.rows
        .iter()
        .position(|r| r[lane].step_impulse > 0.)
        .map(|i| i as u64 * run.step_us)
}

fn compare(run: &Run, reference: &Run) -> Result<Value> {
    let mut lanes = BTreeMap::new();
    let mut passed = true;
    for (i, lane) in LANES.iter().enumerate() {
        let lateral: f64 = [0.0, 0.055, 0.065][i];
        let incoming = 100.0 * DVec3::X;
        let impact = (lateral <= 0.06).then(|| {
            let axial = (0.06_f64.powi(2) - lateral * lateral).sqrt();
            let normal = DVec3::new(-axial, lateral, 0.0) / 0.06;
            let seconds = (0.85 - 0.006 - axial) / incoming.x;
            let outgoing = incoming - incoming.dot(normal) * normal;
            (seconds, outgoing, -2.0 * incoming.dot(normal))
        });
        let mut analytic_error = 0_f64;
        let mut max_angular_speed = 0_f64;
        let mut min_gap = f64::INFINITY;
        let mut peak_error = 0_f64;
        let mut first = None;
        let mut penetration = 0_f64;
        for (k, row) in run.rows.iter().enumerate() {
            let us = k as u64 * run.step_us;
            let expected = reference.rows[(us / reference.step_us) as usize][i];
            if k == 0
                && (row[i].position != expected.position || row[i].velocity != expected.velocity)
            {
                return Err("unmatched initial states".into());
            }
            let seconds = us as f64 * 1e-6;
            let mut predicted = run.rows[0][i].position + incoming * seconds;
            if let Some((touch, outgoing, _)) = impact {
                predicted += (outgoing - incoming) * (seconds - touch).max(0.0);
            }
            analytic_error = analytic_error.max(row[i].position.distance(predicted));
            max_angular_speed = max_angular_speed.max(row[i].angular_speed);
            min_gap = min_gap.min(row[i].gap);
            let e = row[i].position.distance(expected.position);
            if first.is_none() && e > FIRST_DIFFERENCE_M {
                first = Some(us)
            }
            peak_error = peak_error.max(e);
            penetration = penetration.max(-row[i].gap);
        }
        let actual = run.rows.last().unwrap()[i];
        let expected = reference.rows.last().unwrap()[i];
        let err = (actual.total_impulse - expected.total_impulse).abs();
        let measured = onset(run, i);
        let fine = onset(reference, i);
        let classification = measured.is_some() == (i < 2) && measured.is_some() == fine.is_some();
        let onset_pass = match (measured, fine) {
            (Some(a), Some(b)) => a.abs_diff(b) <= run.step_us,
            (None, None) => true,
            _ => false,
        };
        let lane_pass = classification
            && onset_pass
            && peak_error <= POSITION_TOL_M
            && analytic_error <= POSITION_TOL_M
            && impact
                .is_none_or(|(_, _, j)| (actual.total_impulse - j).abs() <= IMPULSE_REL_TOL * j)
            && max_angular_speed <= SPIN_TOL_RAD_S
            && (i != 2 || min_gap >= CLEAR_GAP_M - 1e-12)
            && penetration <= PENETRATION_TOL_M
            && err <= IMPULSE_REL_TOL * expected.total_impulse
            && (i != 2
                || actual
                    .position
                    .distance(run.rows[0][i].position + DVec3::X * 2.)
                    <= FIRST_DIFFERENCE_M);
        passed &= lane_pass;
        let analytic = impact.map(|(touch, outgoing, impulse)| {
            json!({
                "touch_us": touch * 1e6,
                "normal_impulse_ns": impulse,
                "outgoing_velocity_m_s": outgoing.to_array()
            })
        });
        lanes.insert(
            *lane,
            json!({
                "pass": lane_pass,
                "analytic": analytic,
                "impulse_ns": actual.total_impulse,
                "reference_impulse_ns": expected.total_impulse,
                "absolute_impulse_error_ns": err,
                "first_positive_end_us": measured,
                "reference_first_positive_end_us": fine,
                "first_over_1um_us": first,
                "peak_center_error_m": peak_error,
                "peak_analytic_center_error_m": analytic_error,
                "peak_angular_speed_rad_s": max_angular_speed,
                "min_sampled_separation_m": min_gap,
                "peak_sampled_penetration_m": penetration,
                "final_velocity_m_s": actual.velocity.to_array()
            }),
        );
    }
    Ok(
        json!({"step_us":run.step_us,"reference_step_us":reference.step_us,"pass":passed,"lanes":lanes}),
    )
}

fn main() -> Result<()> {
    let out = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: fast_obstacle_course OUTPUT_DIR")?,
    );
    fs::create_dir_all(&out)?;
    let steps = [10_000, 100, 10, 5];
    let mut runs = Vec::new();
    let mut times: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for step in steps {
        runs.push(run(step)?);
    }
    for rep in 0..21 {
        for j in 0..steps.len() {
            let dt = steps[(j + rep) % steps.len()];
            times.entry(dt).or_default().extend(run(dt)?.times_ns);
        }
    }
    let reference = runs.last().unwrap();
    let mut comparisons = Vec::new();
    for r in &runs {
        let mut csv = BufWriter::new(File::create(
            out.join(format!("trace-{}us.csv", r.step_us)),
        )?);
        writeln!(
            csv,
            "time_us,lane,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,step_normal_ns,total_normal_ns,gap_m,angular_speed_rad_s"
        )?;
        for (k, row) in r.rows.iter().enumerate() {
            for (lane, s) in LANES.into_iter().zip(row) {
                writeln!(
                    csv,
                    "{},{lane},{},{},{},{},{},{},{},{},{},{}",
                    k as u64 * r.step_us,
                    s.position.x,
                    s.position.y,
                    s.position.z,
                    s.velocity.x,
                    s.velocity.y,
                    s.velocity.z,
                    s.step_impulse,
                    s.total_impulse,
                    s.gap,
                    s.angular_speed
                )?;
            }
        }
        csv.flush()?;
        let costs = times.get_mut(&r.step_us).unwrap();
        costs.sort_unstable();
        let mut result = compare(r, reference)?;
        result["p95_public_step_us"] =
            json!(costs[(costs.len() * 95).div_ceil(100) - 1] as f64 / 1000.);
        result["timed_steps"] = json!(costs.len());
        comparisons.push(result);
    }
    let qualified = comparisons[1..].iter().all(|entry| entry["pass"] == true);
    let report = json!({
        "pass": qualified,
        "runs": comparisons,
        "thresholds": {
            "peak_center_error_m": POSITION_TOL_M,
            "peak_sampled_penetration_m": PENETRATION_TOL_M,
            "relative_impulse_error": IMPULSE_REL_TOL,
            "first_positive_end_difference": "one candidate tick",
            "clear_displacement_error_m": FIRST_DIFFERENCE_M,
            "clear_min_separation_m": CLEAR_GAP_M,
            "peak_angular_speed_rad_s": SPIN_TOL_RAD_S
        }
    });
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    if !qualified {
        return Err("collision reference envelope missed; inspect comparison.json".into());
    }
    Ok(())
}
