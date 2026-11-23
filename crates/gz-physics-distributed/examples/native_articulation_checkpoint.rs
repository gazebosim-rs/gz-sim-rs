// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two independent processes capture or restore the moving native tree from an ordinary SDF load.
use clap::{Parser, Subcommand};
use gz_physics::{BodyId, JointId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, queue_joint_effort, queue_link_world_wrench,
};
use rapier3d_f64::math::{Rotation, Vector};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{fs, io::Read, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_articulation.sdf");
const CUT: u64 = 120;
const PENDING: f64 = -0.8125;
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    mode: Mode,
}
#[derive(Subcommand)]
enum Mode {
    Source {
        image: PathBuf,
        trace: PathBuf,
    },
    Restored {
        image: PathBuf,
        trace: PathBuf,
    },
    /// Compare exact native observation bits from two independent processes.
    Compare {
        source: PathBuf,
        restored: PathBuf,
    },
}
fn node<'s>(sim: &'s Simulation, name: &str) -> Result<bevy_ecs::world::EntityRef<'s>> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .ok_or_else(|| format!("missing {name}").into())
}
#[derive(Serialize, Deserialize)]
struct Link {
    public_link: u64,
    public_visual: u64,
    native_body: u64,
    position: [f64; 3],
    orientation_xyzw: [f64; 4],
    com: [f64; 3],
    com_velocity: [f64; 3],
    angular_velocity: [f64; 3],
    anchor: [f64; 3],
}
#[derive(Serialize, Deserialize)]
struct Sample {
    offset: u64,
    ecs_clock_ns: u64,
    links: [Link; 2],
    axis: [f64; 3],
    q: f64,
    qdot: f64,
    linear_momentum: [f64; 3],
    angular_momentum: [f64; 3],
}
#[derive(Serialize, Deserialize)]
struct Trace {
    mode: String,
    native_step_ns: u64,
    native_source_cut_ns: u64,
    model: u64,
    public_joint: u64,
    native_joint: u64,
    pending_effort_nm: f64,
    later_effort_tick: u64,
    later_effort_nm: f64,
    frames: Vec<Sample>,
}
struct Binding {
    public: u64,
    visual: u64,
    native: BodyId,
    com: Vector,
    anchor: Vector,
    mass: f64,
    inertia: f64,
}
fn sample(s: &Simulation, links: &[Binding; 2], joint: JointId, offset: u64) -> Sample {
    let backend = s.world().resource::<PhysicsBackend>();
    let mut p = Vector::ZERO;
    let mut l = Vector::ZERO;
    let captured = links.each_ref().map(|link| {
        let b = backend.state(link.native).unwrap();
        let r = Rotation::from_array(b.orientation);
        let pos = Vector::from(b.position);
        let com = pos + r * link.com;
        let linear = Vector::from(b.velocity) * link.mass;
        p += linear;
        l += com.cross(linear) + Vector::from(b.angular_velocity) * link.inertia;
        Link {
            public_link: link.public,
            public_visual: link.visual,
            native_body: link.native.raw(),
            position: b.position,
            orientation_xyzw: b.orientation,
            com: com.to_array(),
            com_velocity: b.velocity,
            angular_velocity: b.angular_velocity,
            anchor: (pos + r * link.anchor).to_array(),
        }
    });
    let hinge = backend.joint_state(joint).unwrap();
    Sample {
        offset,
        ecs_clock_ns: s.elapsed().as_nanos() as u64,
        axis: (Rotation::from_array(captured[0].orientation_xyzw) * Vector::Z).to_array(),
        links: captured,
        q: hinge.position,
        qdot: hinge.velocity,
        linear_momentum: p.to_array(),
        angular_momentum: l.to_array(),
    }
}
fn verify_bevy_publication(
    s: &Simulation,
    links: &[Binding; 2],
    joint: u64,
    frame: &Sample,
) -> Result<()> {
    for (actual, binding) in frame.links.iter().zip(links) {
        let link = entity_by_id(s.world(), binding.public).ok_or("missing public link")?;
        let visual = entity_by_id(s.world(), binding.visual).ok_or("missing public visual")?;
        let pose = s
            .world()
            .get::<WorldPose>(link)
            .ok_or("missing public link pose")?;
        let visual_pose = s
            .world()
            .get::<WorldPose>(visual)
            .ok_or("missing public visual pose")?;
        let velocity = s
            .world()
            .get::<Velocity>(link)
            .ok_or("missing public velocity")?;
        let rotation = Rotation::from_array(actual.orientation_xyzw);
        let omega = Vector::from_array(actual.angular_velocity);
        let origin_velocity =
            Vector::from_array(actual.com_velocity) - omega.cross(rotation * binding.com);
        let sign = visual_pose
            .rotation
            .iter()
            .zip(actual.orientation_xyzw)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .signum();
        let orientation_error = visual_pose
            .rotation
            .iter()
            .zip(actual.orientation_xyzw)
            .map(|(a, b)| (a - sign * b).abs())
            .fold(0., f64::max);
        let link_bits = pose.translation.map(f64::to_bits) == actual.position.map(f64::to_bits)
            && pose.rotation.map(f64::to_bits) == actual.orientation_xyzw.map(f64::to_bits)
            && velocity.angular.map(f64::to_bits) == actual.angular_velocity.map(f64::to_bits);
        let velocity_error = (Vector::from_array(velocity.linear) - origin_velocity).length();
        let visual_error = (Vector::from_array(visual_pose.translation)
            - Vector::from_array(actual.position))
        .length();
        if !link_bits || velocity_error > 1e-12 || visual_error > 1e-12 || orientation_error > 1e-12
        {
            return Err(format!("Bevy publication disagrees at offset {} link {}: link bits {link_bits}, origin velocity error {velocity_error:e}, visual position error {visual_error:e}, visual quaternion error {orientation_error:e}", frame.offset,binding.public).into());
        }
    }
    let id = entity_by_id(s.world(), joint).ok_or("missing public hinge")?;
    let sample = s
        .world()
        .get::<JointSample>(id)
        .ok_or("missing public joint state")?;
    if sample.time != s.elapsed()
        || sample.epoch != s.epoch()
        || sample.state.position.to_bits() != frame.q.to_bits()
        || sample.state.velocity.to_bits() != frame.qdot.to_bits()
    {
        return Err(format!(
            "Bevy hinge publication disagrees at offset {}",
            frame.offset
        )
        .into());
    }
    Ok(())
}

fn compare(source: PathBuf, restored: PathBuf) -> Result<()> {
    let read = |path| -> Result<Trace> { Ok(serde_json::from_slice(&fs::read(path)?)?) };
    let a = read(source)?;
    let b = read(restored)?;
    let contract = |t: &Trace| {
        (
            t.model,
            t.public_joint,
            t.native_joint,
            t.native_step_ns,
            t.native_source_cut_ns,
            t.pending_effort_nm.to_bits(),
            t.later_effort_tick,
            t.later_effort_nm.to_bits(),
        )
    };
    if a.mode != "source"
        || b.mode != "restored"
        || a.native_step_ns != 1_000_000
        || a.native_source_cut_ns != CUT * a.native_step_ns
        || contract(&a) != contract(&b)
        || a.frames.len() != 257
        || b.frames.len() != a.frames.len()
    {
        return Err("mismatched articulation trace contracts".into());
    }
    let mut digest = Sha256::new();
    let mut anchor: f64 = 0.;
    let mut linear_drift: f64 = 0.;
    let mut angular_drift: f64 = 0.;
    let initial = &a.frames[0];
    for (i, (x, y)) in a.frames.iter().zip(&b.frames).enumerate() {
        // Use f64's binary representation, including signed zero, for every observation.
        // The independently constructed ECS clocks have distinct origins.
        let physical = |s: &Sample| {
            bincode::serialize(&(
                s.offset,
                &s.links,
                s.axis,
                s.q,
                s.qdot,
                s.linear_momentum,
                s.angular_momentum,
            ))
        };
        let bits = physical(x)?;
        let elapsed = i as u64 * a.native_step_ns;
        if x.offset != i as u64
            || y.offset != x.offset
            || x.ecs_clock_ns != a.native_source_cut_ns + elapsed
            || y.ecs_clock_ns != elapsed
            || bits != physical(y)?
        {
            return Err(format!("native observation or clock mismatch at offset {i}").into());
        }
        digest.update(bits);
        let distance =
            |first, second| (Vector::from_array(first) - Vector::from_array(second)).length();
        anchor = anchor.max(distance(x.links[0].anchor, x.links[1].anchor));
        linear_drift = linear_drift.max(distance(x.linear_momentum, initial.linear_momentum));
        angular_drift = angular_drift.max(distance(x.angular_momentum, initial.angular_momentum));
    }
    let last = a.frames.last().unwrap();
    let report = serde_json::json!({
        "exact_native_frames": a.frames.len(),
        "physical_observations_sha256": format!("{:x}", digest.finalize()),
        "native_step_ns": a.native_step_ns, "source_cut_ns": a.native_source_cut_ns,
        "restored_ecs_start_ns": b.frames[0].ecs_clock_ns, "pending_effort_nm": a.pending_effort_nm,
        "later_effort_offset": a.later_effort_tick, "later_effort_nm": a.later_effort_nm,
        "q_rad": [initial.q,last.q], "qdot_rad_per_s": [initial.qdot,last.qdot],
        "max_anchor_error_m": anchor, "max_linear_momentum_drift_kg_m_per_s": linear_drift,
        "max_angular_momentum_drift_kg_m2_per_s": angular_drift
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
fn main() -> Result<()> {
    let a = Args::parse();
    let (source, image, trace) = match a.mode {
        Mode::Source { image, trace } => (true, image, trace),
        Mode::Restored { image, trace } => (false, image, trace),
        Mode::Compare { source, restored } => return compare(source, restored),
    };
    let load = || {
        load_world_with_sdf_step(
            SDF,
            "articulation.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    let mut sim = load()?;
    let sdf = sdformat::load_str(SDF, "articulation.sdf", &sdformat::NoResources)?;
    let robot = &sdf.worlds[0].models[0];
    let model = node(&sim, &robot.scoped_name)?
        .get::<EntityId>()
        .unwrap()
        .get();
    let hinge = &robot.joints[0];
    let hinge_name = format!("{}::{}", robot.scoped_name, hinge.name);
    let e = node(&sim, &hinge_name)?;
    let public_joint = e.get::<EntityId>().unwrap().get();
    let native_joint = e.get::<JointHandle>().unwrap().id();
    let mut bindings = Vec::new();
    for link in &robot.links {
        let e = node(&sim, &link.scoped_name)?;
        let public = e.get::<EntityId>().unwrap().get();
        let native = e.get::<BodyHandle>().unwrap().id();
        let visual = node(
            &sim,
            &format!("{}::{}", link.scoped_name, link.visuals[0].name),
        )?
        .get::<EntityId>()
        .unwrap()
        .get();
        let origin = sdf.pose_of(&hinge_name, &link.scoped_name)?.translation;
        bindings.push(Binding {
            public,
            visual,
            native,
            com: Vector::from(link.inertial.pose.transform.translation),
            anchor: Vector::from(origin),
            mass: link.inertial.mass,
            inertia: link.inertial.inertia[0],
        });
    }
    let links: [Binding; 2] = bindings.try_into().map_err(|_| "expected two links")?;
    if source {
        for link in &links {
            queue_link_world_wrench(&mut sim, link.public, [link.mass * 250., 0., 0.], [0.; 3])?;
        }
        for i in 0..CUT {
            if i < 43 {
                queue_joint_effort(&mut sim, public_joint, 4.)?;
            }
            sim.step_exact(1)?;
        }
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        backend.add_joint_effort(native_joint, PENDING)?;
        let bytes = backend
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_floating_revolute()?;
        use std::io::Write;
        fs::OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&image)?
            .write_all(&bytes)?;
    } else {
        // One extra byte lets the adapter reject oversized input without reading an
        // arbitrary file into memory. This profile has exactly 369 or 377 bytes.
        let mut bytes = Vec::new();
        fs::File::open(image)?.take(378).read_to_end(&mut bytes)?;
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .unwrap()
            .restore_floating_revolute(&bytes)?;
    }
    // Publish the just-restored native cut through the ordinary Bevy schedule
    // without advancing either clock or consuming the pending joint torque.
    let paused = sim.is_paused();
    sim.set_paused(true);
    let publication = sim.tick()?;
    if !publication.dt.is_zero() {
        return Err("paused publication advanced native time".into());
    }
    sim.set_paused(paused);
    let mut frames = Vec::with_capacity(257);
    for n in 0..=256 {
        let observed = sample(&sim, &links, native_joint, n);
        verify_bevy_publication(&sim, &links, public_joint, &observed)?;
        frames.push(observed);
        if n == 256 {
            break;
        }
        if n == 99 {
            queue_joint_effort(&mut sim, public_joint, 0.375)?;
        }
        sim.step_exact(1)?;
    }
    let data = Trace {
        mode: if source { "source" } else { "restored" }.to_owned(),
        model,
        public_joint,
        native_joint: native_joint.raw(),
        native_step_ns: 1_000_000,
        native_source_cut_ns: CUT * 1_000_000,
        pending_effort_nm: PENDING,
        later_effort_tick: 99,
        later_effort_nm: 0.375,
        frames,
    };
    // The Bevy loader owns a separate immutable initial/reset resource. Restoring
    // the moving native adapter must leave that original ownership token valid.
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    let mut fresh = load()?;
    for tick in 0..=24 {
        if bincode::serialize(&sample(&sim, &links, native_joint, tick))?
            != bincode::serialize(&sample(&fresh, &links, native_joint, tick))?
        {
            return Err(format!("loaded reset disagrees with fresh SDF at tick {tick}").into());
        }
        if tick < 24 {
            sim.step_exact(1)?;
            fresh.step_exact(1)?;
        }
    }
    eprintln!(
        "257 Bevy link, visual and joint publications agree with the native state, including the paused cut."
    );
    eprintln!("Loaded Bevy reset and 24 successors match a separately loaded SDF.");
    fs::write(trace, serde_json::to_vec(&data)?)?;
    Ok(())
}
