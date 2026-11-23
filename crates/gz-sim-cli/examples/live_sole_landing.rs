// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the measured landing through an SDF-loaded Bevy simulation, then compare at identical public clocks.
#[path = "live_sole_landing/fixture.rs"]
mod fixture;
#[path = "live_sole_landing/metrics.rs"]
mod metrics;

use clap::Parser;
use fixture::{Donor, Material};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, damped_sole::last_step, load_world_with_sdf_step,
};
use metrics::{Cost, Errors};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const CLOCK: Duration = Duration::from_micros(500);

#[derive(Parser)]
struct Args {
    #[arg(
        long,
        default_value = "artifacts/multifidelity-free-landing/donor.json"
    )]
    donor: PathBuf,
    #[arg(
        long,
        default_value = "artifacts/multifidelity-damped-landing/reference.json"
    )]
    reference: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long, default_value_t = 128)]
    substeps: u32,
    #[arg(long, default_value_t = 1)]
    depth: u8,
    /// Refine from the requested minimum grid until the per-tick comparison passes.
    #[arg(long)]
    adaptive: bool,
    /// Fail if shared-clock errors exceed this fixture’s documented qualification bounds.
    #[arg(long)]
    qualify: bool,
    /// Repeated complete landings per material, following one untimed warm-up.
    #[arg(long, default_value_t = 9)]
    repetitions: usize,
}

#[derive(Deserialize)]
struct Reference {
    case: u32,
    stroke: u32,
    horizon_s: f64,
    material: Material,
    profiles: Vec<History>,
}
#[derive(Deserialize)]
struct History {
    name: String,
    damping_s_m: f64,
    frames: Vec<Frame>,
    checks: ReferenceChecks,
}
#[derive(Deserialize)]
struct ReferenceChecks {
    peak_normal_n: f64,
    lift_off_time_s: f64,
}
#[derive(Serialize, Deserialize, Default)]
struct Frame {
    original_time_s: f64,
    foot_m: [f64; 3],
    com_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    v_m_s: [f64; 3],
    spin_rad_s: [f64; 3],
    support_frame_j_ns: [f64; 3],
    normal_force_n: f64,
    normal_dissipation_j: f64,
    loaded_area_m2: f64,
    center_of_pressure_m: Option<[f64; 3]>,
    loaded_polygon_m: Vec<[f64; 3]>,
    sliding_work_j: f64,
    boundary_energy_flux_j: f64,
    strain_energy_j: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    refinement: Option<Refinement>,
}

#[derive(Serialize, Deserialize)]
struct Refinement {
    substeps: u32,
    depth: u8,
    trials: u32,
    temporal_impulse_delta_ns: f64,
    spatial_impulse_delta_ns: f64,
}

fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "live-sole.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}

#[allow(deprecated)]
fn body_id(sim: &Simulation, name: &str) -> Result<BodyId> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<BodyHandle>())
        .map(BodyHandle::id)
        .ok_or_else(|| format!("missing physics body for {name}").into())
}

fn body_state(sim: &Simulation, id: BodyId) -> Result<BodyState> {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(id)
        .ok_or_else(|| "missing body state".into())
}

fn snapshot(body: BodyState, deck: BodyState, donor: &Donor, t: f64) -> Frame {
    let relative_shift = DVec3::from(deck.position) - DVec3::from(donor.deck.center_m);
    let foot = DVec3::from(body.position) - relative_shift;
    Frame {
        original_time_s: t,
        foot_m: foot.to_array(),
        com_m: (foot + DQuat::from_array(body.orientation) * donor.com_offset()).to_array(),
        orientation_xyzw: body.orientation,
        v_m_s: (DVec3::from(body.velocity) - DVec3::from(deck.velocity)).to_array(),
        spin_rad_s: body.angular_velocity,
        ..Frame::default()
    }
}

fn trajectory(donor: &Donor, xml: &str, ticks: u32) -> Result<(Vec<Frame>, f64)> {
    let mut sim = load(xml)?;
    let foot_id = body_id(&sim, "default::foot::body")?;
    let deck_id = body_id(&sim, "default::deck::body")?;
    let mut frames = vec![snapshot(
        body_state(&sim, foot_id)?,
        body_state(&sim, deck_id)?,
        donor,
        0.,
    )];
    let (mut impulse, mut heat, mut work, mut flux, mut peak) = (DVec3::ZERO, 0., 0., 0., 0_f64);
    for tick in 1..=ticks {
        if let Err(e) = sim.step_exact(1) {
            return Err(format!(
                "public tick {tick}: {}",
                sim.fault()
                    .map_or_else(|| e.to_string(), |fault| fault.to_string())
            )
            .into());
        }
        let elapsed = sim.elapsed();
        if elapsed != tick * CLOCK {
            return Err("public clock changed".into());
        }
        let read = last_step(&sim).ok_or("sole system produced no step report")?;
        let s = &read.step;
        let contact = &s.contact;
        let body = body_state(&sim, foot_id)?;
        let deck = body_state(&sim, deck_id)?;
        let shift = DVec3::from(deck.position) - DVec3::from(donor.deck.center_m);
        let t = elapsed.as_secs_f64();
        let mut f = snapshot(body, deck, donor, t);
        if (DVec3::from(f.com_m) + shift).distance(read.state.center) > 1e-10
            || DVec3::from(body.velocity).distance(read.state.velocity) > 1e-10
            || DVec3::from(body.angular_velocity).distance(read.state.spin) > 1e-10
            || shift.distance(t * DVec3::from(donor.drive)) > 1e-10
        {
            return Err("published body or deck disagrees with sole integration".into());
        }
        impulse += s.impulse;
        heat += s.dissipated_energy;
        work += s.sliding_work;
        flux += s.boundary_work;
        peak = peak.max(s.peak_normal_force);
        f.normal_force_n = contact.normal_force;
        f.normal_dissipation_j = heat;
        f.sliding_work_j = work;
        f.boundary_energy_flux_j = flux;
        f.strain_energy_j = contact.strain_energy;
        f.loaded_area_m2 = contact.area;
        f.support_frame_j_ns = impulse.to_array();
        f.center_of_pressure_m = contact.center_of_pressure.map(|p| (p - shift).to_array());
        f.loaded_polygon_m = contact
            .polygon()
            .iter()
            .map(|p| (*p - shift).to_array())
            .collect();
        f.refinement = read.adaptive.map(|r| Refinement {
            substeps: r.substeps,
            depth: r.depth,
            trials: r.trials,
            temporal_impulse_delta_ns: r.temporal.impulse_ns,
            spatial_impulse_delta_ns: r.spatial.impulse_ns,
        });
        frames.push(f);
    }
    Ok((frames, peak))
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.repetitions == 0 {
        return Err("choose at least one timed repetition".into());
    }
    let donor: Donor = serde_json::from_slice(&fs::read(args.donor)?)?;
    let reference: Reference = serde_json::from_slice(&fs::read(args.reference)?)?;
    let ticks = (donor.horizon_s / CLOCK.as_secs_f64()).round() as u32;
    if (donor.case, donor.stroke) != (80, 2)
        || (reference.case, reference.stroke) != (donor.case, donor.stroke)
        || reference.profiles.len() != 3
        || reference.horizon_s != donor.horizon_s
        || (ticks * CLOCK).as_secs_f64() != donor.horizon_s
    {
        return Err("reference and donor do not share fixture or clock".into());
    }
    let parent = args
        .output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    if args.output.exists() {
        return Err("choose a new output directory".into());
    }
    let stage = tempfile::Builder::new()
        .prefix(".live-sole-")
        .tempdir_in(parent)?;
    let mut profiles = Vec::new();
    let mut results = Vec::new();
    for (i, history) in reference.profiles.iter().enumerate() {
        let xml = donor.xml(
            &reference.material,
            history.damping_s_m,
            args.substeps,
            args.depth,
            args.adaptive,
        );
        fs::write(stage.path().join(format!("material-{i}.sdf")), &xml)?;
        let (frames, peak) = trajectory(&donor, &xml, ticks)?;
        let errors = Errors::compare(&frames, history, peak)?;
        if args.qualify {
            errors.qualify()?;
        }
        let timings = Cost::measure(&xml, ticks, args.repetitions)?;
        eprintln!(
            "{}: peak {:.2} N, maximum COM error {:.3} µm",
            history.name,
            peak,
            1e6 * errors.max_com_m
        );
        profiles.push(json!({"name":history.name,"damping_s_m":history.damping_s_m,"start_s":0.,"frames":frames}));
        results.push(json!({"name":history.name,"damping_s_m":history.damping_s_m,"accuracy":errors,"step_cost":timings}));
    }
    let common = json!({"case":donor.case,"stroke":donor.stroke,"horizon_s":donor.horizon_s,
        "base_substeps_per_tick":args.substeps,"base_quadrature_depth":args.depth,"adaptive":args.adaptive,"qualification_requested":args.qualify,"source":"SDF-loaded local Bevy multifidelity simulation"});
    let save = |name: &str, key: &str, data: Vec<Value>| -> Result<()> {
        let mut doc = common.clone();
        doc[key] = json!(data);
        Ok(fs::write(
            stage.path().join(name),
            serde_json::to_vec_pretty(&doc)?,
        )?)
    };
    save("live.json", "profiles", profiles)?;
    save("comparison.json", "results", results)?;
    fs::rename(stage.path(), args.output)?;
    Ok(())
}
