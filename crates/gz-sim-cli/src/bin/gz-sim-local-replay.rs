// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic, ordered local CPU physics input replay.
#![allow(deprecated)]
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{self, BufRead, BufReader, BufWriter, Write},
    path::PathBuf,
};

use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::Parser;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{Name, WorldPose};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[path = "local_replay/observation.rs"]
mod observation;

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
#[command(about = "Replay an ordered local CPU input journal with the SDF's fixed step")]
struct Args {
    world: PathBuf,
    journal: PathBuf,
    /// Create an exact profile/physics trace; the destination must be new.
    #[arg(long, conflicts_with = "verify")]
    trace: Option<PathBuf>,
    /// Stop at the first different observation or execution profile.
    #[arg(long)]
    verify: Option<PathBuf>,
    /// Write ordinary gz-web recordings into separate epoch directories.
    #[arg(long)]
    record: Option<PathBuf>,
    /// Use the viewer's lighting in the recording and omit authored SDF lights.
    #[arg(long, requires = "record")]
    viewer_lighting: bool,
    /// Save every Nth advancing step (default 10). Lifecycle and final frames are always saved.
    #[arg(long, requires = "record", value_parser = clap::value_parser!(u64).range(1..))]
    record_every: Option<u64>,
    /// Use one Coulomb friction constraint per rigid contact point.
    #[arg(long)]
    rapier_per_point_friction: bool,
    /// Resolve qualified axial cylinder impacts with a centered normal reaction.
    #[arg(long)]
    rapier_axial_cap_impact: bool,
    /// Use a 120 Hz fixed-body contact response (default 60 Hz).
    #[arg(long)]
    stiff_static_contacts: bool,
    /// Use SDFormat friction for omitted collision surfaces in the local CPU engine.
    #[arg(long)]
    sdf_default_friction: bool,
    /// Host compute-pool size; the authoritative local CPU solve is serial.
    #[arg(long, default_value_t=1, value_parser=clap::value_parser!(u8).range(1..=64))]
    threads: u8,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Journal {
    seed: String,
    /// Scoped links and scalar joints to identify in the trace. Links include ground-truth poses.
    #[serde(default)]
    observe: Vec<String>,
    commands: Vec<Command>,
}

#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum ReplayTier {
    Prescribed,
    Dynamic,
}

/// Startup links use their scoped SDF name. A live alias identifies the single
/// link belonging to that particular spawned instance.
#[derive(Deserialize, Serialize)]
#[serde(untagged, deny_unknown_fields)]
enum LoadLink {
    Authored(String),
    Spawned { alias: String },
}

#[derive(Deserialize, Serialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Command {
    Step {
        count: u64,
    },
    PausedTick,
    DiffDrive {
        model: String,
        linear_m_s: f64,
        angular_rad_s: f64,
    },
    Target {
        joint: String,
        value: f64,
    },
    SetJointForce {
        joint: String,
        force: f64,
    },
    JointEffort {
        joint: String,
        effort: f64,
    },
    UniversalTorque {
        joint: String,
        torque_nm: [f64; 2],
    },
    TrackBall {
        count: u64,
        joint: String,
        orientation_xyzw: [f64; 4],
        #[serde(default)]
        angular_velocity_rad_s: [f64; 3],
        #[serde(default)]
        feedforward_nm: [f64; 3],
        stiffness_nm_rad: f64,
        damping_nm_s_rad: f64,
        max_torque_nm: f64,
    },
    BallTorque {
        joint: String,
        torque_nm: [f64; 3],
    },
    SetContactRefinement {
        enabled: bool,
    },
    SetTier {
        model: String,
        tier: ReplayTier,
    },
    PointForce {
        link: LoadLink,
        point_in_link_m: [f64; 3],
        world_force_n: [f64; 3],
    },
    Wrench {
        link: LoadLink,
        force: [f64; 3],
        torque: [f64; 3],
    },
    Spawn {
        template: String,
        alias: String,
        position: [f64; 3],
        /// World orientation of the new model. Existing journals spawn it upright.
        rotation_xyzw: Option<[f64; 4]>,
    },
    Remove {
        alias: String,
    },
    /// Remove a model from the loaded SDF at the current simulation instant.
    RemoveModel {
        model: String,
    },
    Reset,
}

fn digest(data: impl AsRef<[u8]>) -> String {
    format!("{:x}", Sha256::digest(data))
}

fn build_id() -> Result<String> {
    let mut hasher = Sha256::new();
    io::copy(&mut File::open(std::env::current_exe()?)?, &mut hasher)?;
    Ok(format!("{:x}", hasher.finalize()))
}

fn cpu_id() -> Result<String> {
    if std::env::consts::ARCH != "x86_64" || std::env::consts::OS != "linux" {
        return Err("local-cpu-v1 requires Linux x86-64".into());
    }
    let text = std::fs::read_to_string("/proc/cpuinfo")?;
    cpu_signature(&text)
}

fn cpu_signature(text: &str) -> Result<String> {
    let mut cpus = std::collections::BTreeSet::new();
    for block in text.trim().split("\n\n") {
        let fields: BTreeMap<_, _> = block
            .lines()
            .filter_map(|line| line.split_once(':'))
            .map(|(key, value)| (key.trim(), value.trim()))
            .collect();
        let names = [
            "vendor_id",
            "cpu family",
            "model",
            "stepping",
            "model name",
            "microcode",
            "flags",
        ];
        let cpu = names
            .map(|key| fields.get(key).copied())
            .into_iter()
            .collect::<Option<Vec<_>>>()
            .ok_or("incomplete x86 CPU identity")?;
        cpus.insert(cpu.join("|"));
    }
    if cpus.len() != 1 {
        return Err("local-cpu-v1 requires one homogeneous CPU signature".into());
    }
    Ok(cpus.pop_first().unwrap())
}

fn entity_id(sim: &Simulation, name: &str) -> Result<u64> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .and_then(|e| e.get::<EntityId>().map(EntityId::get))
        .ok_or_else(|| format!("unknown scoped entity {name}").into())
}

fn observed_entities(sim: &Simulation, names: &[String]) -> Result<BTreeMap<String, u64>> {
    let mut bindings = BTreeMap::new();
    for name in names {
        let entity = sim
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|n| &n.0 == name))
            .ok_or_else(|| format!("unknown observed entity: {name}"))?;
        if !(entity.contains::<gz_sim_scene::BodyHandle>()
            || entity.contains::<gz_sim_scene::JointSample>())
        {
            return Err(format!("expected observed link or scalar joint: {name}").into());
        }
        let id = entity
            .get::<EntityId>()
            .ok_or("missing stable observed identity")?
            .get();
        if bindings.insert(name.clone(), id).is_some() {
            return Err(format!("duplicate observed entity: {name}").into());
        }
    }
    Ok(bindings)
}

struct Sink {
    output: Box<dyn Write>,
    expected: Option<io::Lines<BufReader<File>>>,
    line: usize,
}

impl Sink {
    fn emit(&mut self, value: &impl Serialize) -> Result<()> {
        let row = serde_json::to_string(value)?;
        self.line += 1;
        if let Some(expected) = &mut self.expected
            && expected.next().transpose()?.as_deref() != Some(&row)
        {
            return Err(format!("physics replay differs at trace line {}", self.line).into());
        }
        writeln!(self.output, "{row}")?;
        Ok(())
    }
}

struct Capture {
    visuals: gz_sim_cli::ArchiveVisuals,
    path: PathBuf,
    frames: Vec<(String, String)>,
    last_time: Option<std::time::Duration>,
}
impl Capture {
    fn sample(&mut self, sim: &Simulation, sdf: &sdformat::Sdf, world: &str) -> Result<()> {
        if self.frames.len() >= 4096 {
            return Err("one epoch exceeds 4096 display frames".into());
        }
        self.last_time = Some(sim.elapsed());
        self.frames.push(
            self.visuals
                .frame(sim, sdf, world, self.frames.len() as u64 + 1)?,
        );
        Ok(())
    }
    fn finish(&mut self, sim: &Simulation, sdf: &sdformat::Sdf, world: &str) -> Result<()> {
        if self.last_time != Some(sim.elapsed()) {
            self.sample(sim, sdf, world)?;
        }
        let epoch = sim.epoch();
        gz_sim_cli::export_archive_at(
            &self.path.join(format!("epoch-{epoch}")),
            self.visuals.files(),
            epoch,
            std::time::Duration::ZERO,
            std::mem::take(&mut self.frames),
        )?;
        Ok(())
    }
}

fn wrench_link(
    sim: &Simulation,
    world: &sdformat::World,
    aliases: &BTreeMap<String, u64>,
    link: &LoadLink,
) -> Result<u64> {
    let link = match link {
        LoadLink::Authored(name) => name,
        LoadLink::Spawned { alias } => {
            let id = aliases
                .get(alias)
                .ok_or_else(|| format!("unknown live alias {alias}"))?;
            let world = sim.world();
            let model = gz_sim_core::entity_by_id(world, *id).ok_or("spawned model missing")?;
            let children = world.get::<bevy_ecs::hierarchy::Children>(model);
            let Some([body]) = children.map(|nodes| &nodes[..]) else {
                return Err("spawned template must own exactly one link".into());
            };
            return Ok(world
                .entity(*body)
                .get::<EntityId>()
                .ok_or("spawned link missing")?
                .get());
        }
    };
    if !world
        .models
        .iter()
        .any(|model| !model.is_static && model.links.iter().any(|body| body.scoped_name == *link))
    {
        return Err("one-step loads require a link of an original dynamic model".into());
    }
    entity_id(sim, link)
}

fn run(args: Args) -> Result<()> {
    let pool = ComputeTaskPool::get_or_init(|| {
        TaskPoolBuilder::new()
            .num_threads(usize::from(args.threads))
            .build()
    });
    eprintln!(
        "local-cpu-v1 Bevy compute pool: {} threads, one native solver",
        pool.thread_num()
    );
    let xml = std::fs::read_to_string(&args.world)?;
    let bytes = std::fs::read(&args.journal)?;
    let journal: Journal = serde_json::from_slice(&bytes)?;
    let source = args.world.to_string_lossy();
    let sdf = sdformat::load_str(&xml, &source, &sdformat::NoResources)?;
    let [world] = sdf.worlds.as_slice() else {
        return Err("exactly one inline SDF world is required".into());
    };
    let seed = Sha256::digest(journal.seed.as_bytes()).into();
    let runtime = gz_sim_scene::WorldRuntimeOptions {
        rapier_per_point_friction: args.rapier_per_point_friction,
        rapier_axial_cap_impact: args.rapier_axial_cap_impact,
        stiff_static_contacts: args.stiff_static_contacts,
        sdf_default_friction: args.sdf_default_friction,
        ..Default::default()
    };
    let mut sim = gz_sim_scene::load_local_replay_world_with_runtime(
        &xml,
        &source,
        &world.name,
        seed,
        runtime,
    )?;
    sim.set_paused(true);
    let mut sink = Sink {
        output: match &args.trace {
            Some(p) => Box::new(BufWriter::new(File::create_new(p)?)),
            None if args.verify.is_some() => Box::new(io::sink()),
            None => Box::new(BufWriter::new(io::stdout())),
        },
        expected: args
            .verify
            .as_deref()
            .map(File::open)
            .transpose()?
            .map(|f| BufReader::new(f).lines()),
        line: 0,
    };
    let mut profile = serde_json::json!({
        "profile": "local-cpu-v1", "binary_sha256": build_id()?, "platform": "linux-x86_64", "cpu": cpu_id()?,
        "cargo_lock_sha256": digest(include_bytes!("../../../../Cargo.lock")), "package": env!("CARGO_PKG_VERSION"),
        "numerics": "native serial Rapier f64/enhanced-determinism", "compiled_gpu_batch": cfg!(feature="gpu-batch"),
        "compiled_distributed": cfg!(feature="distributed-physics"), "world_sha256": digest(&xml), "journal_sha256": digest(&bytes),
        "seed": journal.seed, "step_ns": sim.step_duration().as_nanos()
    });
    if args.rapier_per_point_friction {
        profile["rapier_per_point_friction"] = true.into();
    }
    if args.rapier_axial_cap_impact {
        profile["rapier_axial_cap_impact"] = true.into();
    }
    if args.sdf_default_friction {
        profile["sdf_default_friction"] = true.into();
    }
    if args.stiff_static_contacts {
        profile["static_contact_frequency_hz"] = 120.into();
    }
    let bindings = observed_entities(&sim, &journal.observe)?;
    let observed_ids = bindings.values().copied().collect();
    if !bindings.is_empty() {
        profile["observed_entities"] = serde_json::to_value(bindings)?;
    }
    sink.emit(&profile)?;
    let record_every = args.record_every.unwrap_or(10);
    let mut capture = if let Some(path) = args.record {
        Some(Capture {
            path,
            visuals: gz_sim_cli::ArchiveVisuals::new(&sim)?
                .with_viewer_lighting(args.viewer_lighting),
            frames: Vec::new(),
            last_time: None,
        })
    } else {
        None
    };
    if let Some(c) = &mut capture {
        std::fs::create_dir(&c.path)?;
        c.sample(&sim, &sdf, &world.name)?;
    }
    let mut aliases = BTreeMap::new();
    let mut templates = BTreeMap::new();
    let mut original_models = BTreeMap::new();
    for op in &journal.commands {
        if let Command::RemoveModel { model } = op {
            // Bind before any edits. A later clone cannot take over a retired name.
            original_models.insert(model, entity_id(&sim, model)?);
        }
        if let Command::Spawn { template, .. } = op
            && !templates.contains_key(template)
        {
            let model = world
                .models
                .iter()
                .find(|m| m.scoped_name == *template)
                .ok_or_else(|| format!("unknown startup template {template}"))?;
            templates.insert(
                template.clone(),
                gz_sim_scene::prepare_model_template(&sim, &sdf, model)?,
            );
        }
    }
    sink.emit(&observation::state(&sim, 0, &observed_ids)?)?;
    for (index, command) in journal.commands.iter().enumerate() {
        let mut keyframe = false;
        match command {
            Command::Step { count } | Command::TrackBall { count, .. } => {
                sim.validate_steps(*count)?;
                let drive = if let Command::TrackBall {
                    joint,
                    orientation_xyzw,
                    angular_velocity_rad_s,
                    feedforward_nm,
                    stiffness_nm_rad,
                    damping_nm_s_rad,
                    max_torque_nm,
                    ..
                } = command
                {
                    if *count == 0 {
                        return Err(
                            "a spherical drive must request at least one advancing tick".into()
                        );
                    }
                    Some((
                        entity_id(&sim, joint)?,
                        gz_physics::joint_drive::SphericalDrive::new(
                            *stiffness_nm_rad,
                            *damping_nm_s_rad,
                            *max_torque_nm,
                        )?,
                        gz_physics_multifidelity::BallJointState {
                            orientation: *orientation_xyzw,
                            angular_velocity: *angular_velocity_rad_s,
                        },
                        *feedforward_nm,
                    ))
                } else {
                    None
                };
                for _ in 0..*count {
                    if let Some((id, motor, target, ff)) = &drive {
                        gz_sim_scene::queue_ball_joint_drive(&mut sim, *id, motor, *target, *ff)?;
                    }
                    sim.step_exact(1)
                        .map_err(|e| gz_sim_cli::with_fault(&sim, e))?;
                    sink.emit(&observation::state(&sim, index + 1, &observed_ids)?)?;
                    if let Some(c) = &mut capture
                        && sim
                            .committed_update_info()?
                            .iteration
                            .is_multiple_of(record_every)
                    {
                        c.sample(&sim, &sdf, &world.name)?;
                    }
                }
                continue;
            }
            Command::PausedTick => {
                sim.tick()?;
            }
            Command::DiffDrive {
                model,
                linear_m_s,
                angular_rad_s,
            } => {
                let id = entity_id(&sim, model)?;
                gz_sim_scene::set_diff_drive_command(&mut sim, id, *linear_m_s, *angular_rad_s)?;
            }
            Command::Target { joint, value } => {
                let id = entity_id(&sim, joint)?;
                gz_sim_scene::set_joint_controller_command(&mut sim, id, *value)?;
            }
            Command::SetJointForce { joint, force } => {
                let id = entity_id(&sim, joint)?;
                gz_sim_scene::set_joint_force(&mut sim, id, *force)?;
            }
            Command::JointEffort { joint, effort } => {
                let id = entity_id(&sim, joint)?;
                gz_sim_scene::queue_joint_effort(&mut sim, id, *effort)?;
            }
            Command::UniversalTorque { joint, torque_nm } => {
                let id = entity_id(&sim, joint)?;
                gz_sim_scene::queue_universal_joint_effort(&mut sim, id, *torque_nm)?;
            }
            Command::BallTorque { joint, torque_nm } => {
                let id = entity_id(&sim, joint)?;
                gz_sim_scene::queue_ball_joint_effort(&mut sim, id, *torque_nm)?;
            }
            Command::SetContactRefinement { enabled } => {
                gz_sim_scene::primary_contact::set_enabled(&mut sim, *enabled)?;
            }
            Command::SetTier { model, tier } => {
                use gz_sim_scene::ArticulationTier;
                let id = entity_id(&sim, model)?;
                let tier = match tier {
                    ReplayTier::Prescribed => ArticulationTier::Prescribed,
                    ReplayTier::Dynamic => ArticulationTier::Dynamic,
                };
                gz_sim_scene::set_model_fidelity(&mut sim, id, tier)?;
            }
            Command::PointForce {
                link,
                point_in_link_m,
                world_force_n,
            } => {
                let id = wrench_link(&sim, world, &aliases, link)?;
                gz_sim_scene::queue_link_force_at_point(
                    &mut sim,
                    id,
                    *point_in_link_m,
                    *world_force_n,
                )?;
            }
            Command::Wrench {
                link,
                force,
                torque,
            } => {
                let id = wrench_link(&sim, world, &aliases, link)?;
                gz_sim_scene::queue_link_world_wrench(&mut sim, id, *force, *torque)?;
            }
            Command::Spawn {
                template,
                alias,
                position,
                rotation_xyzw,
            } => {
                if aliases.contains_key(alias) {
                    return Err(format!("duplicate live alias {alias}").into());
                }
                let pose = WorldPose {
                    translation: *position,
                    rotation: rotation_xyzw.unwrap_or([0., 0., 0., 1.]),
                };
                aliases.insert(
                    alias.clone(),
                    gz_sim_scene::spawn_model_template(&mut sim, &templates[template], pose)?,
                );
                keyframe = true;
            }
            Command::RemoveModel { model } => {
                gz_sim_scene::remove_model(&mut sim, original_models[model])
                    .map_err(|e| format!("cannot remove model {model}: {e}"))?;
                keyframe = true;
            }
            Command::Remove { alias } => {
                let id = aliases
                    .remove(alias)
                    .ok_or_else(|| format!("unknown live alias {alias}"))?;
                gz_sim_scene::remove_model(&mut sim, id)?;
                keyframe = true;
            }
            Command::Reset => {
                if let Some(c) = &mut capture {
                    c.finish(&sim, &sdf, &world.name)?;
                }
                gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
                aliases.clear();
                keyframe = true;
            }
        }
        sink.emit(&observation::state(&sim, index + 1, &observed_ids)?)?;
        if keyframe && let Some(c) = &mut capture {
            c.sample(&sim, &sdf, &world.name)?;
        }
    }
    if let Some(c) = &mut capture {
        c.finish(&sim, &sdf, &world.name)?;
    }
    if sink
        .expected
        .as_mut()
        .is_some_and(|rows| rows.next().is_some())
    {
        return Err("extra data after final replay observation".into());
    }
    sink.output.flush()?;
    eprintln!("{} profile and state records", sink.line);
    Ok(())
}
fn main() -> Result<()> {
    run(Args::parse())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn heterogeneous_or_incomplete_cpu_records_cannot_acquire_a_profile_identity() {
        let cpu = "vendor_id: test\ncpu family: 6\nmodel: 1\nstepping: 1\nmodel name: test\nmicrocode: 1\nflags: sse";
        assert!(cpu_signature("vendor_id: test").is_err());
        assert!(
            cpu_signature(&format!(
                "{cpu}\n\n{}",
                cpu.replace("stepping: 1", "stepping: 2")
            ))
            .is_err()
        );
        assert_eq!(
            cpu_signature(cpu).unwrap(),
            cpu_signature(&format!("{cpu}\n\n{cpu}")).unwrap()
        );
    }
    #[test]
    fn stale_and_non_template_alias_bindings_cannot_redirect_an_input() {
        let xml = include_str!("../../../../examples/worlds/local_robot_replay.sdf");
        let sim =
            gz_sim_scene::load_local_replay_world(xml, "robot.sdf", "default", [0; 32]).unwrap();
        let sdf = sdformat::load_str(xml, "robot.sdf", &sdformat::NoResources).unwrap();
        let alias = LoadLink::Spawned {
            alias: "parcel".into(),
        };
        for (id, message) in [
            (u64::MAX, "spawned model missing"),
            (
                entity_id(&sim, "default::gripper").unwrap(),
                "spawned template must own exactly one link",
            ),
        ] {
            let aliases = BTreeMap::from([("parcel".into(), id)]);
            assert_eq!(
                wrench_link(&sim, &sdf.worlds[0], &aliases, &alias)
                    .unwrap_err()
                    .to_string(),
                message
            );
        }
    }

    #[test]
    fn trace_bindings_identify_scoped_links_and_wheels_and_reject_ambiguous_requests() {
        let xml = include_str!("../../../../examples/worlds/upstream_diff_drive.sdf");
        let mut sim =
            gz_sim_scene::load_local_replay_world(xml, "drive.sdf", "diff_drive", [0; 32]).unwrap();
        let wheel = String::from("diff_drive::vehicle::left_wheel_joint");
        let body = String::from("diff_drive::vehicle::chassis");
        let names = [wheel.clone(), body.clone()];
        let bindings = observed_entities(&sim, &names).unwrap();
        assert_ne!(bindings[&wheel], bindings[&body]);
        for (names, message) in [
            (vec![wheel.clone(), wheel], "duplicate observed entity"),
            (
                vec![String::from("vehicle::chassis")],
                "unknown observed entity",
            ),
            (
                vec![String::from("diff_drive::vehicle")],
                "expected observed link or scalar joint",
            ),
        ] {
            assert!(
                observed_entities(&sim, &names)
                    .unwrap_err()
                    .to_string()
                    .contains(message)
            );
        }
        let entity = gz_sim_core::entity_by_id(sim.world(), bindings[&body]).unwrap();
        sim.world_mut().entity_mut(entity).remove::<WorldPose>();
        let ids = bindings.values().copied().collect();
        assert!(
            observation::state(&sim, 0, &ids)
                .err()
                .unwrap()
                .to_string()
                .contains("observed link has no committed world pose")
        );
        sim.world_mut().entity_mut(entity).remove::<EntityId>();
        assert!(
            observed_entities(&sim, &[body])
                .unwrap_err()
                .to_string()
                .contains("missing stable observed identity")
        );
    }

    #[test]
    fn viewer_lighting_omits_only_the_authored_light() {
        let xml = include_str!("../../../../examples/worlds/upstream_double_pendulum.sdf");
        let sdf = sdformat::load_str(xml, "pendulum.sdf", &sdformat::NoResources).unwrap();
        let mut sim =
            gz_sim_scene::load_local_replay_world(xml, "pendulum.sdf", "default", [0; 32]).unwrap();
        let archive = gz_sim_cli::ArchiveVisuals::new(&sim).unwrap();
        assert!(
            archive
                .frame(&sim, &sdf, "default", 1)
                .unwrap_err()
                .to_string()
                .contains("does not lower SDF lights")
        );
        let archive = archive.with_viewer_lighting(true);
        let (scene, _) = archive.frame(&sim, &sdf, "default", 1).unwrap();
        let scene: serde_json::Value = serde_json::from_str(&scene).unwrap();
        assert_eq!(scene["visuals"].as_array().unwrap().len(), 15);

        let mut properties = sim
            .world_mut()
            .query::<&mut gz_sim_scene::VisualProperties>();
        properties
            .iter_mut(sim.world_mut())
            .next()
            .unwrap()
            .0
            .transparency = 0.5;
        assert!(
            archive
                .frame(&sim, &sdf, "default", 2)
                .unwrap_err()
                .to_string()
                .contains("opaque diffuse primitives")
        );
    }

    #[test]
    fn full_display_epoch_refuses_an_additional_frame() {
        let xml = include_str!("../../../../examples/worlds/local_robot_replay.sdf");
        let sim =
            gz_sim_scene::load_local_replay_world(xml, "robot.sdf", "default", [0; 32]).unwrap();
        let sdf = sdformat::load_str(xml, "robot.sdf", &sdformat::NoResources).unwrap();
        let mut capture = Capture {
            visuals: gz_sim_cli::ArchiveVisuals::default(),
            path: PathBuf::new(),
            frames: vec![(String::new(), String::new()); 4096],
            last_time: None,
        };
        assert!(capture.sample(&sim, &sdf, "default").is_err());
        assert_eq!(capture.frames.len(), 4096);
    }
}
