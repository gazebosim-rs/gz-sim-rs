// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay two independently measured local CPU trajectories on the same Gazebo clock.
mod soft_replay;
use bevy_ecs::hierarchy::ChildOf;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_scene::{
    LocalPose, Name, PhysicsEngine, VisualGeometry, VisualMaterial, load_world_with_sdf_step,
};
use serde::Deserialize;
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{error::Error, fs::File, io::BufReader, path::PathBuf, time::Duration};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = include_str!("../../../examples/multifidelity_free_payload_impact.sdf");
const BODY_NAMES: [&str; 3] = ["arm::base", "arm::tip", "payload::body"];
// Turn the measured XY plane upright. World X maps to display −Y and world Y maps to display Z.
const VIEW: DQuat = DQuat::from_xyzw(-0.5, 0.5, 0.5, -0.5);

#[derive(Parser)]
struct Args {
    /// Directory containing trajectory.json and summary.json from free_payload_promotion.
    measurements: PathBuf,
    /// New native replay directory.
    output: PathBuf,
}

#[derive(Deserialize)]
struct Run {
    dt_s: f64,
    samples: Vec<Sample>,
}
#[derive(Deserialize)]
struct Events {
    first_promotion_tick: u64,
    first_contact_tick: u64,
}
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    automatic: [Body; 3],
    reference: [Body; 3],
}
#[derive(Deserialize)]
struct Body {
    name: String,
    position: [f64; 3],
    orientation: [f64; 4],
}

struct Visual {
    body: usize,
    name: String,
    position: DVec3,
    rotation: DQuat,
    shape: Value,
    color: [f64; 4],
}

fn authored_visuals() -> Result<Vec<Visual>> {
    let sim = load_world_with_sdf_step(
        SOURCE,
        "free_payload.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut visuals = Vec::new();
    for entity in sim.world().iter_entities() {
        let Some(geometry) = entity.get::<VisualGeometry>() else {
            continue;
        };
        let owner = entity
            .get::<ChildOf>()
            .ok_or("visual needs an owning link")?
            .parent();
        let parent = &sim.world().get::<Name>(owner).ok_or("missing link name")?.0;
        let body = BODY_NAMES
            .iter()
            .position(|name| parent.strip_prefix("default::") == Some(*name))
            .ok_or("unexpected body in impact fixture")?;
        let name = entity
            .get::<Name>()
            .ok_or("missing visual name")?
            .0
            .strip_prefix("default::")
            .ok_or("unexpected world")?
            .replace("::", "_");
        let pose = entity
            .get::<LocalPose>()
            .ok_or("missing visual local pose")?
            .0;
        let shape = match geometry.0.as_ref() {
            sdformat::Geometry::Sphere { radius } => json!({"kind":"sphere","radius":radius}),
            sdformat::Geometry::Box { size } => json!({"kind":"box","size":size}),
            _ => return Err("unexpected geometry in measured arm fixture".into()),
        };
        let color = entity
            .get::<VisualMaterial>()
            .ok_or("missing authored material")?
            .0
            .diffuse
            .map(f64::from);
        visuals.push(Visual {
            body,
            name,
            position: pose.translation.into(),
            rotation: DQuat::from_array(pose.rotation),
            shape,
            color,
        });
    }
    visuals.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(visuals)
}

fn panel_separation(samples: &[Sample]) -> f64 {
    let mut lower = f64::INFINITY;
    let mut upper = f64::NEG_INFINITY;
    for sample in samples {
        for body in sample.automatic.iter().chain(&sample.reference) {
            lower = lower.min(body.position[0]);
            upper = upper.max(body.position[0]);
        }
    }
    upper - lower + 0.75
}

fn main() -> Result<()> {
    let args = Args::parse();
    let run: Run = serde_json::from_reader(BufReader::new(File::open(
        args.measurements.join("trajectory.json"),
    )?))?;
    let events: Events =
        serde_json::from_reader(File::open(args.measurements.join("summary.json"))?)?;
    if run.dt_s != 0.001
        || run.samples.is_empty()
        || run.samples[0].tick != 0
        || events.first_contact_tick < events.first_promotion_tick
        || events.first_contact_tick as usize >= run.samples.len()
    {
        return Err(
            "expected a 1 ms impact series with committed promotion and contact events".into(),
        );
    }

    let visuals = authored_visuals()?;
    let spacing = panel_separation(&run.samples);
    let mut frames = Vec::new();
    let mut retained = Vec::new();
    for (i, sample) in run.samples.iter().enumerate() {
        if sample.tick != i as u64 {
            return Err("missing measured interval".into());
        }
        // Retain 50 Hz away from impact and every committed tick around the two events.
        if sample.tick % 20 != 0
            && i + 1 < run.samples.len()
            && !(events.first_promotion_tick.saturating_sub(2)
                ..=events.first_contact_tick.saturating_add(3))
                .contains(&sample.tick)
        {
            continue;
        }
        let mut frame = Frame::default();
        for (lane, bodies, offset) in [
            ("automatic", &sample.automatic, spacing * 0.5),
            ("reference", &sample.reference, -spacing * 0.5),
        ] {
            let shift = DVec3::Y * offset;
            for visual in &visuals {
                let body = &bodies[visual.body];
                if body.name != BODY_NAMES[visual.body] {
                    return Err("recorded body order disagrees with the scene".into());
                }
                let q = DQuat::from_array(body.orientation);
                let position = VIEW * (DVec3::from(body.position) + q * visual.position) + shift;
                frame.add(
                    &format!("{lane}_{}", visual.name),
                    position,
                    VIEW * q * visual.rotation,
                    visual.shape.clone(),
                    visual.color,
                );
            }
        }
        // The green pin marks the switch in the automatic arm. It follows the measured
        // base-to-joint attachment. The gold pin passes through the payload center at first impulse.
        if sample.tick >= events.first_promotion_tick {
            let base = &sample.automatic[0];
            let world =
                DVec3::from(base.position) + DQuat::from_array(base.orientation) * DVec3::Y * 0.325;
            let hinge = VIEW * world + DVec3::Y * spacing * 0.5;
            frame.edge(
                &format!("promoted_at_{}ms", events.first_promotion_tick),
                hinge - DVec3::X * 0.04,
                hinge + DVec3::X * 0.04,
                0.08,
                0.065,
                [0.25, 0.95, 0.23, 1.],
            );
        }
        if sample.tick >= events.first_contact_tick {
            let body = &run.samples[events.first_contact_tick as usize].automatic[2];
            frame.add(
                &format!(
                    "payload_center_at_first_impulse_{}ms",
                    events.first_contact_tick
                ),
                VIEW * DVec3::from(body.position) + DVec3::Y * spacing * 0.5,
                DQuat::IDENTITY,
                // Draw through the motion plane so the payload cannot occlude its own marker.
                json!({"kind":"box","size":[0.36,0.05,0.05]}),
                [1., 0.79, 0.16, 1.],
            );
        }
        retained.push(sample.tick);
        frames.push(frame.finish(Duration::from_millis(sample.tick), frames.len() + 1));
    }
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&json!({"milliseconds":retained,
        "panel_separation_m":spacing,"display_rotation_xyzw":VIEW.to_array(),
        "first_promotion_tick":events.first_promotion_tick,"first_contact_tick":events.first_contact_tick,
        "authored_visuals_per_world":visuals.len()}))?
    );
    Ok(())
}
