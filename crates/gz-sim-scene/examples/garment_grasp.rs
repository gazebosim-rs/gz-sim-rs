// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Force-controlled pickup of an unpinned cloth swatch in terrestrial gravity.
//! Two free spherical pads apply opposing world-frame forces to the same swatch.
//! Both pads feel the cloth reaction. A vertical position controller lifts them,
//! holds, then opens. An otherwise identical open-jaw run controls for the lift.
//! The particle collision radius is 3 mm; every particle remains mobile.
//! Run `cargo run --release --locked -p gz-sim-scene --example garment_grasp`.
//! Append `-- --capture grasp.json` to save the initial pose and every subsequent
//! committed millisecond of the force-controlled pinch in world coordinates.
//! The capture path must be new. Add `--folded-transport` to load
//! `examples/folded_garment_transport.sdf`, move the opposing pads laterally,
//! and compare with both open-pad and zero-contact-friction controls. All
//! cloth particles remain free. The optional capture is the loaded pinch run.
use glam::DVec3;
#[path = "support/folded_carry.rs"]
mod folded_carry;
use folded_carry::Carry;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, WorldPose, deformable, load_world_with_sdf_step,
    queue_link_world_wrench,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{
    error::Error,
    fmt::Write,
    io::{BufWriter, Write as IoWrite},
    path::{Path, PathBuf},
};
use tempfile::NamedTempFile;

const STEPS: usize = 1400;
const TIP_NAMES: [&str; 2] = ["default::left::tip", "default::right::tip"];
const G: f64 = 9.81;
const FINGER_RADIUS: f64 = 0.034;
const FINGER_MASS: f64 = 0.003;
const PARTICLE_RADIUS: f64 = 0.003;
const HALF_OPEN: f64 = 0.038;
const START_Z: f64 = 0.0405;
const PINCH_FORCE: f64 = 0.15;
const PARTICLE_MASS: f64 = 0.00005;
const COLUMNS: usize = 7;
const ROWS: usize = 5;
const SPACING: f64 = 0.0125;
const CARRY: f64 = 0.055;
const FOLDED: &str = include_str!("../../../examples/folded_garment_transport.sdf");
const LIFT: f64 = 0.095;

fn world_sdf() -> String {
    let mut xml = String::from(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
<physics name="one_ms" type="ignored"><max_step_size>0.001</max_step_size></physics>"#,
    );
    for (name, sign) in [("left", 1.), ("right", -1.)] {
        let y = HALF_OPEN * sign;
        let i = 0.4 * FINGER_MASS * FINGER_RADIUS.powi(2);
        writeln!(xml,r#"<model name="{name}"><pose>0 {y} {START_Z} 0 0 0</pose><link name="tip">
<inertial><mass>{FINGER_MASS}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>
<collision name="surface"><geometry><sphere><radius>{FINGER_RADIUS}</radius></sphere></geometry></collision>
<visual name="pad"><geometry><sphere><radius>{FINGER_RADIUS}</radius></sphere></geometry></visual></link></model>"#).unwrap();
    }
    write!(xml,r#"<plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">
<sphere link="default::left::tip" coupling="two_way"/><sphere link="default::right::tip" coupling="two_way"/>
<plane point="0 0 0" normal="0 0 1"/>
<cloth name="swatch" columns="{COLUMNS}" substeps="4" iterations="16" stretch="0.00001" shear="0.00002" bend="0.02" radius="{PARTICLE_RADIUS}" self_contact="true" friction="0.65">"#).unwrap();
    for row in 0..ROWS {
        for column in 0..COLUMNS {
            let x = (column as f64 - (COLUMNS / 2) as f64) * SPACING;
            // Slight lean removes the perfectly balanced edge-on equilibrium
            // of the swatch on the table in the open-jaw control.
            let y = (row as f64 - 2.) * 0.00010;
            let z = PARTICLE_RADIUS + row as f64 * SPACING;
            writeln!(
                xml,
                r#"<particle position="{x} {y} {z}" mass="{PARTICLE_MASS}"/>"#
            )
            .unwrap();
        }
    }
    xml.push_str("</cloth></plugin></world></sdf>");
    xml
}

fn trajectory(t: f64, start: f64, duration: f64, origin: f64, length: f64) -> (f64, f64) {
    let u = ((t - start) / duration).clamp(0., 1.);
    let p = u * u * u * (10. + u * (-15. + 6. * u));
    let v = 30. * u * u * (1. - u).powi(2) / duration;
    (origin + length * p, length * v)
}

struct Capture {
    frames: Vec<Value>,
}

impl Capture {
    fn sample(&mut self, sim: &Simulation, tips: [[f64; 3]; 2]) {
        let (_, swatch) = deformable::bodies(sim)
            .find(|(name, _)| *name == "swatch")
            .unwrap();
        assert_eq!(swatch.sim_time(), sim.elapsed());
        let time_ns = u64::try_from(swatch.sim_time().as_nanos()).unwrap();
        let particles: Vec<_> = swatch.particles().iter().map(|p| p.position).collect();
        assert_eq!(particles.len(), ROWS * COLUMNS);
        assert!(
            tips.iter()
                .chain(&particles)
                .flatten()
                .all(|v| v.is_finite())
        );
        self.frames
            .push(json!({"time_ns":time_ns,"tips":tips,"particles":particles}));
    }

    fn save(self, path: &Path) -> Result<(), Box<dyn Error>> {
        assert_eq!(self.frames.len(), STEPS + 1);
        // Use the same upper-left to lower-right diagonal as SoftBody::cloth.
        let mut triangles = Vec::new();
        for row in 0..ROWS - 1 {
            for col in 0..COLUMNS - 1 {
                let a = row * COLUMNS + col;
                triangles.extend([
                    [a, a + 1, a + COLUMNS + 1],
                    [a, a + COLUMNS + 1, a + COLUMNS],
                ]);
            }
        }
        let tips = TIP_NAMES
            .into_iter()
            .zip(["#e88749", "#4179ac"])
            .map(|(name, color)| json!({"name":name,"color":color,"radius":FINGER_RADIUS}))
            .collect::<Vec<_>>();
        let mut document = json!({
            "version":1,"units":"m","up":"z","ground_z":0,"tips":tips,
            "cloth":{"rows":ROWS,"columns":COLUMNS,"particle_radius":PARTICLE_RADIUS,
                    "color":"#269c88","triangles":triangles},
        });
        document["frames"] = Value::Array(self.frames);
        // A failed physics run or a short write must leave an existing capture intact.
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        let file = NamedTempFile::new_in(parent)?;
        let mut writer = BufWriter::new(file);
        serde_json::to_writer(&mut writer, &document)?;
        writer.flush()?;
        writer.into_inner()?.persist_noclobber(path)?;
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Grip {
    Closed,
    Open,
    Frictionless,
    ParticleControl,
}
#[derive(Clone, Copy, PartialEq, Eq)]
enum Task {
    Flat,
    Folded,
    SurfacePads,
}
fn run(mode: Grip, task: Task, mut capture: Option<&mut Capture>) -> Result<Carry, Box<dyn Error>> {
    let grasp = mode != Grip::Open;
    let folded = task != Task::Flat;
    let (xml, name) = match task {
        Task::Flat => (world_sdf(), "garment_grasp.sdf"),
        Task::Folded => (FOLDED.to_owned(), "folded_garment_transport.sdf"),
        Task::SurfacePads => (
            include_str!("../../../examples/folded_surface_pad_transport.sdf").to_owned(),
            "folded_surface_pad_transport.sdf",
        ),
    };
    let xml = if mode == Grip::ParticleControl {
        let active = "pad_contact=\"surface\"";
        if xml.matches(active).count() != 1 {
            return Err("expected one surface-contact garment".into());
        }
        xml.replacen(active, "pad_contact=\"particles\"", 1)
    } else {
        xml
    };
    let xml = if mode == Grip::Frictionless {
        let active = "friction=\"0.65\"";
        if xml.matches(active).count() != 1 {
            return Err("expected one explicit cloth contact coefficient for the control".into());
        }
        xml.replacen(active, "friction=\"0\"", 1)
    } else {
        xml
    };
    let mut sim = load_world_with_sdf_step(
        &xml,
        name,
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let tips = TIP_NAMES.map(|name| {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|s| s.0 == name))
            .unwrap();
        (e.id(), e.get::<EntityId>().unwrap().get())
    });
    let center = |sim: &Simulation| {
        let (_, body) = deformable::bodies(sim).next().unwrap();
        body.particles()
            .iter()
            .map(|p| DVec3::from_array(p.position))
            .sum::<DVec3>()
            / (ROWS * COLUMNS) as f64
    };
    let tip_positions =
        |sim: &Simulation| tips.map(|(e, _)| sim.world().get::<WorldPose>(e).unwrap().translation);
    if let Some(capture) = capture.as_deref_mut() {
        capture.sample(&sim, tip_positions(&sim));
    }
    let initial = center(&sim);
    let z0 = initial.z;
    let initial_particles = deformable::bodies(&sim)
        .next()
        .unwrap()
        .1
        .particles()
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    let seed_tips = tip_positions(&sim);
    let check_seed = |sim: &Simulation| {
        assert_eq!(tip_positions(sim), seed_tips);
        let (_, sheet) = deformable::bodies(sim).next().unwrap();
        for (particle, q) in sheet.particles().iter().zip(&initial_particles) {
            assert!(particle.inverse_mass > 0.);
            assert_eq!(DVec3::from(particle.position), *q);
            assert_eq!(particle.velocity, [0.; 3]);
        }
    };
    if folded {
        sim.set_paused(true);
        sim.tick()?;
        check_seed(&sim);
        sim.set_paused(false);
    }
    let grip_x = seed_tips[0][0];
    let open_gap = tip_positions(&sim)[0][1];
    let mut travel = Carry::from_seed(deformable::bodies(&sim).next().unwrap().1.particles());
    let mut bilateral_ticks = 0;
    let mut applied_squeeze_impulse = [0.; 2];
    let mut supported_impulse = 0.;
    let mut squeeze_impulses = [0.; 2];
    let mut min_gap = f64::INFINITY;
    let mut max_overlap = 0_f64;
    let mut min_hold_height = f64::INFINITY;
    let mut max_hold_height = f64::NEG_INFINITY;
    let mut min_hold_clearance = f64::INFINITY;
    let mut max_edge_strain = 0_f64;
    // 150 ms to close, 550 ms to lift, 300 ms at height, 400 ms to open and release.
    for tick in 0..STEPS {
        let t = sim.elapsed().as_secs_f64();
        let (z, vz) = trajectory(t, 0.15, 0.55, START_Z, LIFT);
        let (x_target, vx_target) =
            trajectory(t, 0.25, 0.45, grip_x, if folded { CARRY } else { 0. });
        for (&(entity, id), sign) in tips.iter().zip([1., -1.]) {
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            let v = sim.world().get::<Velocity>(entity).unwrap();
            let [x, y, h] = pose.translation;
            let [vx, vy, vh] = v.linear;
            let fy = if grasp && tick < 1000 {
                // The cloth determines the pad spacing through two-way contact.
                -sign * PINCH_FORCE - 1.5 * vy
            } else {
                let half_gap = if tick < 1000 { open_gap } else { 0.055 };
                400. * (sign * half_gap - y) - 1.7 * vy
            };
            if (750..1000).contains(&tick) {
                let i = usize::from(sign < 0.);
                applied_squeeze_impulse[i] += -sign * fy * sim.step_duration().as_secs_f64();
            }
            let force = [
                400. * (x_target - x) + 1.7 * (vx_target - vx),
                fy,
                FINGER_MASS * G + 400. * (z - h) + 1.7 * (vz - vh),
            ];
            let damping = if task == Task::SurfacePads {
                0.0010
            } else {
                0.0003
            };
            let torque = v.angular.map(|w| -damping * w);
            queue_link_world_wrench(&mut sim, id, force, torque)?;
        }
        if let Err(error) = sim.step_exact(1) {
            return Err(format!("step {tick} failed ({error}), cause {:?}", sim.fault()).into());
        }
        let (_, body) = deformable::bodies(&sim).next().unwrap();
        assert_eq!(body.sim_time(), sim.elapsed());
        let positions = tip_positions(&sim);
        if let Some(capture) = capture.as_deref_mut() {
            capture.sample(&sim, positions);
        }
        let centers = positions.map(DVec3::from_array);
        assert!(centers.iter().all(|p| p.is_finite() && p.z > FINGER_RADIUS));
        min_gap = min_gap.min(centers[0].distance(centers[1]) - 2. * FINGER_RADIUS);
        let particles = body.particles();
        if folded {
            let twists = tips.map(|(entity, _)| *sim.world().get::<Velocity>(entity).unwrap());
            travel.observe(
                tick,
                centers,
                twists,
                particles,
                body.last_boundary_impulses(),
                sim.step_duration().as_secs_f64(),
            );
        }
        for (i, p) in particles.iter().enumerate() {
            let position = DVec3::from_array(p.position);
            assert!(position.is_finite() && p.inverse_mass == PARTICLE_MASS.recip());
            for tip in centers {
                max_overlap =
                    max_overlap.max(FINGER_RADIUS + PARTICLE_RADIUS - tip.distance(position));
            }
            let row_edge = (i % COLUMNS > 0).then(|| i - 1);
            let diagonal = i.checked_sub(COLUMNS + 1).filter(|_| i % COLUMNS > 0);
            for neighbor in [row_edge, i.checked_sub(COLUMNS), diagonal]
                .into_iter()
                .flatten()
            {
                let length = position.distance(DVec3::from_array(particles[neighbor].position));
                max_edge_strain = max_edge_strain.max(
                    (length / initial_particles[i].distance(initial_particles[neighbor]) - 1.)
                        .abs(),
                );
            }
        }
        if (750..1000).contains(&tick) {
            let reactions = body.last_boundary_impulses();
            if reactions[0].linear[1] > 1e-5 && reactions[1].linear[1] < -1e-5 {
                bilateral_ticks += 1;
            }
            for (i, sign) in [1., -1.].into_iter().enumerate() {
                squeeze_impulses[i] += sign * reactions[i].linear[1];
                supported_impulse -= reactions[i].linear[2];
            }
            let height = center(&sim).z;
            min_hold_height = min_hold_height.min(height);
            max_hold_height = max_hold_height.max(height);
            for p in particles {
                min_hold_clearance = min_hold_clearance.min(p.position[2] - PARTICLE_RADIUS);
            }
        }
    }
    let weight = (ROWS * COLUMNS) as f64 * PARTICLE_MASS * G;
    let supported = supported_impulse / 0.25;
    let squeeze = squeeze_impulses.map(|j| j / 0.25);
    let applied = applied_squeeze_impulse.map(|j| j / 0.25);
    let lift = min_hold_height - z0;
    let drop = min_hold_height - center(&sim).z;
    let label = match mode {
        Grip::Closed => "force-controlled pinch",
        Grip::Open => "open control",
        Grip::Frictionless => "frictionless closed control",
        Grip::ParticleControl => "matched particle-contact control",
    };
    println!(
        "{label}: minimum held lift {lift:.5} m, post-release drop {drop:.5} m, bilateral contact {bilateral_ticks}/250 ticks"
    );
    println!(
        "  mean supporting force {supported:.6} N, weight {weight:.6} N, mean opposing pad forces {squeeze:.4?} N, applied {applied:.4?} N"
    );
    println!(
        "  minimum pad gap {min_gap:.6} m, peak pad penetration {max_overlap:.6} m, peak edge strain {max_edge_strain:.4}"
    );
    if folded {
        travel.report();
        println!(
            "  held COM height range {:.6} m; minimum clearance above table {:.6} m",
            max_hold_height - min_hold_height,
            min_hold_clearance
        );
    }
    assert!(min_gap > 0.002 && max_overlap < 0.00015 && max_edge_strain < 0.03);
    if mode != Grip::Frictionless && task != Task::SurfacePads {
        assert!(max_hold_height - min_hold_height < 0.001);
    }
    if grasp {
        if task == Task::SurfacePads {
            assert!(lift > 0.060 && min_hold_clearance > 0.055 && drop > 0.065);
        } else {
            assert!(lift > 0.080 && min_hold_clearance > 0.060 && drop > 0.080);
        }
        assert!(bilateral_ticks >= 245 && (supported / weight - 1.).abs() < 0.03);
        for (reaction, applied) in squeeze.into_iter().zip(applied) {
            assert!(applied > 0.10 && (reaction / applied - 1.).abs() < 0.02);
        }
    } else {
        assert!(lift < 0.001 && bilateral_ticks == 0 && supported.abs() < weight * 0.01);
    }
    if folded {
        sim.reset()?;
        check_seed(&sim);
    }
    Ok(travel)
}
fn qualify_carry(grip: &Carry, open: &Carry, dry: &Carry) {
    assert!(grip.held_translation > 0.035 && open.held_translation.abs() < 0.001);
    assert!(dry.held_translation + 0.010 < grip.held_translation);
    assert!((grip.initial_fold - 90.).abs() < 0.01 && grip.hold_fold > 80.);
    assert!(grip.max_mid_overlap < 0.00001 && grip.max_shell_overlap < 0.0015);
    assert!(grip.relative_com_drift < 0.015 && dry.relative_com_drift > 0.030);
    assert_eq!(grip.tangent_samples, [450, 450]);
    for i in 0..2 {
        assert!(grip.tangent_distance[i] < 0.003 && dry.tangent_distance[i] > 0.025);
    }
    assert_eq!(open.last_pad_impulse, None);
    assert_eq!(grip.last_pad_impulse, Some(1000));
    assert!(grip.first_clear.is_some_and(|t| (1001..=1010).contains(&t)));
    assert!(grip.touchdown.is_some_and(|t| (1090..=1170).contains(&t)));
    assert!(grip.final_clearance > 0.06);
}
fn qualify_surface(grip: &Carry, open: &Carry, particle: &Carry) {
    assert!(grip.max_shell_overlap < 1e-7 && particle.max_shell_overlap > 2e-4);
    assert!(grip.held_translation > 0.030 && open.held_translation.abs() < 0.001);
    assert!(
        grip.last_pad_impulse
            .is_some_and(|t| (995..=1010).contains(&t))
    );
    assert!(grip.first_clear.is_some_and(|t| (1001..=1015).contains(&t)));
    assert!(grip.final_clearance > 0.05);
}
const USAGE: &str =
    "usage: garment_grasp [--folded-transport | --surface-pads] [--capture FILE.json]";

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1).collect::<Vec<_>>();
    let task = args
        .iter()
        .position(|a| a == "--surface-pads" || a == "--folded-transport")
        .map_or(Task::Flat, |i| {
            if args.remove(i) == "--surface-pads" {
                Task::SurfacePads
            } else {
                Task::Folded
            }
        });
    let path = match args.as_slice() {
        [] => None,
        [flag, file] if flag == "--capture" && !file.is_empty() => Some(PathBuf::from(file)),
        [flag] if flag == "--help" || flag == "-h" => {
            println!("{USAGE}");
            return Ok(());
        }
        _ => return Err(USAGE.into()),
    };
    if let Some(path) = &path
        && path.try_exists()?
    {
        return Err(format!("capture file already exists: {}", path.display()).into());
    }
    let mut capture = path.as_ref().map(|_| Capture {
        frames: Vec::with_capacity(STEPS + 1),
    });
    let grip = run(Grip::Closed, task, capture.as_mut())?;
    let open = run(Grip::Open, task, None)?;
    match task {
        Task::Flat => {}
        Task::Folded => qualify_carry(&grip, &open, &run(Grip::Frictionless, task, None)?),
        Task::SurfacePads => {
            qualify_surface(&grip, &open, &run(Grip::ParticleControl, task, None)?)
        }
    }
    if let (Some(capture), Some(path)) = (capture, path) {
        capture.save(&path)?;
        println!(
            "Wrote {} committed samples to {}",
            STEPS + 1,
            path.display()
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_pad_grip_matches_continuous_mesh_and_opens_cleanly() -> Result<(), Box<dyn Error>> {
        qualify_surface(
            &run(Grip::Closed, Task::SurfacePads, None)?,
            &run(Grip::Open, Task::SurfacePads, None)?,
            &run(Grip::ParticleControl, Task::SurfacePads, None)?,
        );
        Ok(())
    }

    #[test]
    fn finite_pads_transport_and_release_a_folded_free_sheet() -> Result<(), Box<dyn Error>> {
        qualify_carry(
            &run(Grip::Closed, Task::Folded, None)?,
            &run(Grip::Open, Task::Folded, None)?,
            &run(Grip::Frictionless, Task::Folded, None)?,
        );
        Ok(())
    }

    #[test]
    fn capture_preserves_world_scale_topology_and_every_committed_pose()
    -> Result<(), Box<dyn Error>> {
        let mut capture = Capture { frames: Vec::new() };
        run(Grip::Closed, Task::Flat, Some(&mut capture))?;
        let dir = tempfile::tempdir()?;
        let path = dir.path().join("grasp.json");
        capture.save(&path)?;
        let data: Value = serde_json::from_reader(std::fs::File::open(path)?)?;
        assert_eq!(data["units"], "m");
        assert_eq!(data["up"], "z");
        assert_eq!(data["tips"][0]["name"], TIP_NAMES[0]);
        assert_eq!(data["tips"][1]["name"], TIP_NAMES[1]);
        assert_ne!(data["tips"][0]["color"], data["tips"][1]["color"]);
        assert_eq!(data["tips"][0]["radius"], FINGER_RADIUS);
        let tris = data["cloth"]["triangles"].as_array().unwrap();
        assert_eq!(tris.len(), 48);
        assert_eq!(tris[0], json!([0, 1, 8]));
        assert_eq!(tris[47], json!([26, 34, 33]));
        let frames = data["frames"].as_array().unwrap();
        assert_eq!(frames.len(), 1401);
        for (i, frame) in frames.iter().enumerate() {
            assert_eq!(frame["time_ns"], i as u64 * 1_000_000);
            let tips = frame["tips"].as_array().unwrap();
            let particles = frame["particles"].as_array().unwrap();
            assert_eq!((tips.len(), particles.len()), (2, 35));
            for point in tips.iter().chain(particles) {
                let xyz: [f64; 3] = serde_json::from_value(point.clone())?;
                assert!(xyz.iter().all(|v| v.is_finite()));
            }
        }
        // Initial geometry and recorded COM distinguish measured motion from a
        // static rest mesh, a reordered grid, or prescribed pad trajectories.
        let f0 = &frames[0];
        assert_eq!(f0["tips"][0], json!([0., HALF_OPEN, START_Z]));
        assert_eq!(f0["tips"][1], json!([0., -HALF_OPEN, START_Z]));
        let first: [f64; 3] = serde_json::from_value(f0["particles"][0].clone())?;
        let last: [f64; 3] = serde_json::from_value(f0["particles"][34].clone())?;
        assert!((first[0] + 3. * SPACING).abs() < 1e-12 && (last[0] - 3. * SPACING).abs() < 1e-12);
        assert!((last[2] - first[2] - 4. * SPACING).abs() < 1e-12);
        let mean_z = |index: usize| {
            frames[index]["particles"]
                .as_array()
                .unwrap()
                .iter()
                .map(|p| p[2].as_f64().unwrap())
                .sum::<f64>()
                / 35.
        };
        assert!(mean_z(850) - mean_z(0) > 0.08 && mean_z(850) - mean_z(1400) > 0.1);
        assert!(frames[1400]["tips"][0][1].as_f64().unwrap() > 0.05);
        Ok(())
    }
}
