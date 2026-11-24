// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public SDF/Bevy finite rough contact in rotated and translating lab frames.
#[path = "support/rough_contact_native.rs"]
mod native;
#[path = "support/rough_covariance_plot.rs"]
mod plot;
use bevy_ecs::entity::Entity;
use clap::Parser;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Body, ContactImpulse, GroundSupport, Primitive, Tier},
    pool::PoolFrame,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose, load_world_configured,
    queue_link_world_velocity,
};
use rapier3d_f64::prelude::{Rotation, Vector};
use sdformat::NoResources;
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Duration,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const G: f64 = 9.81;
const RADIUS: f64 = 0.3;
const MASS: f64 = 2.;
const MU: f64 = 0.35;
const KICK: f64 = 7.;
const SDF: &str = include_str!("../../../examples/distributed_rough_mobility.sdf");
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, default_value_t = 2)]
    step_ms: u64,
    out: PathBuf,
}
#[derive(Clone, Copy)]
struct FrameChange {
    yaw: f64,
    boost_local: Vector,
}
impl FrameChange {
    fn rotation(self) -> Rotation {
        Rotation::from_rotation_z(self.yaw)
    }
    fn boost(self) -> Vector {
        self.rotation() * self.boost_local
    }
    fn drive(self) -> Vector {
        self.rotation() * Vector::X
    }
    fn document(self) -> String {
        let v = self.boost();
        SDF.replace(
            "<pose>0 0 -0.2 0 0 0</pose>",
            &format!("<pose>0 0 -0.2 0 0 {:.17}</pose>", self.yaw),
        )
        .replace(
            "<pose>0 0 0.35 0 0 0</pose>",
            &format!("<pose>0 0 0.35 0 0 {:.17}</pose>", self.yaw),
        )
        .replace(
            "<linear_velocity>0 0 0</linear_velocity>",
            &format!(
                "<linear_velocity>{:.17} {:.17} 0</linear_velocity>",
                v.x, v.y
            ),
        )
    }
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn pool(sim: &Simulation) -> &PoolFrame {
    backend(sim).pool_frame().unwrap()
}
#[derive(Clone, Copy)]
struct Link {
    id: BodyId,
    public: u64,
    entity: Entity,
}
fn link(sim: &Simulation, name: &str) -> Result<Link> {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .ok_or("SDF link missing")?;
    Ok(Link {
        id: e.get::<BodyHandle>().ok_or("no native body")?.id(),
        public: e.get::<EntityId>().ok_or("missing ECS link ID")?.get(),
        entity: e.id(),
    })
}
fn state(sim: &Simulation, links: &[Link; 2]) -> Result<[Body; 2]> {
    let f = pool(sim);
    let mut bodies = Vec::new();
    for (index, link) in links.iter().enumerate() {
        let mut b = f
            .body(link.id.raw())
            .ok_or("missing link in worker")?
            .1
            .clone();
        let pose = sim
            .world()
            .get::<WorldPose>(link.entity)
            .ok_or("missing ECS pose")?;
        let twist = sim
            .world()
            .get::<Velocity>(link.entity)
            .ok_or("missing ECS twist")?;
        if pose.translation.map(f64::to_bits) != b.position.map(f64::to_bits)
            || pose.rotation.map(f64::to_bits) != b.orientation.map(f64::to_bits)
            || twist.linear.map(f64::to_bits) != b.linear.map(f64::to_bits)
            || twist.angular.map(f64::to_bits) != b.angular.map(f64::to_bits)
        {
            return Err(format!("SDF link {index} differs from worker at tick {}", f.tick).into());
        }
        b.id = index as u64 + 1;
        bodies.push(b);
    }
    Ok(bodies.try_into().unwrap())
}
fn physical_pairs(sim: &Simulation, links: &[Link; 2]) -> Result<Vec<ContactImpulse>> {
    let mut pairs = vec![];
    for w in &pool(sim).workers {
        if !w.snapshot.supports.is_empty() {
            return Err("unexpected infinite floor".into());
        }
        for p in &w.snapshot.impulses {
            if (p.a, p.b) != (links[0].id.raw(), links[1].id.raw()) {
                return Err("unexpected finite contact IDs".into());
            }
            pairs.push(ContactImpulse {
                a: 1,
                b: 2,
                normal_impulse: p.normal_impulse,
            });
        }
    }
    Ok(pairs)
}
#[derive(Clone, Debug, Serialize)]
struct Sample {
    positions: [[f64; 3]; 2],
    rotations: [[f64; 4]; 2],
    linear: [[f64; 3]; 2],
    angular: [[f64; 3]; 2],
    contact: bool,
    positive_contact: bool,
    normal: [f64; 3],
    tangent: [f64; 3],
}
fn quat(q: [f64; 4]) -> Rotation {
    Rotation::from_xyzw(q[0], q[1], q[2], q[3])
}
impl Sample {
    fn reference_frame(x: FrameChange, t: f64, b: &[Body; 2], r: &native::Reaction) -> Self {
        let inv = x.rotation().conjugate();
        let u = x.boost();
        Self {
            positions: b
                .each_ref()
                .map(|a| (inv * (Vector::from(a.position) - u * t)).to_array()),
            rotations: b.each_ref().map(|a| (inv * quat(a.orientation)).to_array()),
            linear: b
                .each_ref()
                .map(|a| (inv * (Vector::from(a.linear) - u)).to_array()),
            angular: b
                .each_ref()
                .map(|a| (inv * Vector::from(a.angular)).to_array()),
            contact: r.active,
            positive_contact: r.normal_n_s > 0.,
            normal: (inv * Vector::from(r.normal_on_sphere)).to_array(),
            tangent: (inv * Vector::from(r.tangent_on_sphere)).to_array(),
        }
    }
}
fn maximum_vector(a: [f64; 3], b: [f64; 3]) -> f64 {
    (Vector::from(a) - Vector::from(b)).length()
}
fn chord(a: [f64; 4], b: [f64; 4]) -> f64 {
    [1., -1.]
        .into_iter()
        .map(|s| {
            a.into_iter()
                .zip(b)
                .map(|(x, y)| (x - s * y).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}
#[derive(Default, Serialize)]
struct Symmetry {
    ticks: u64,
    exact_serialized_samples: u64,
    max_position_m: f64,
    max_orientation_chord: f64,
    max_linear_m_s: f64,
    max_angular_rad_s: f64,
    max_normal_n_s: f64,
    max_tangent_n_s: f64,
    different_active_contact_frames: u64,
    different_positive_contact_frames: u64,
    first_outside_gate: Option<Value>,
}
impl Symmetry {
    fn observe(&mut self, tick: u64, a: &Sample, b: &Sample) -> Result<()> {
        self.ticks += 1;
        self.exact_serialized_samples +=
            u64::from(bincode::serialize(a)? == bincode::serialize(b)?);
        for i in 0..2 {
            self.max_position_m = self
                .max_position_m
                .max(maximum_vector(a.positions[i], b.positions[i]));
            self.max_orientation_chord = self
                .max_orientation_chord
                .max(chord(a.rotations[i], b.rotations[i]));
            self.max_linear_m_s = self
                .max_linear_m_s
                .max(maximum_vector(a.linear[i], b.linear[i]));
            self.max_angular_rad_s = self
                .max_angular_rad_s
                .max(maximum_vector(a.angular[i], b.angular[i]));
        }
        self.max_normal_n_s = self.max_normal_n_s.max(maximum_vector(a.normal, b.normal));
        self.max_tangent_n_s = self
            .max_tangent_n_s
            .max(maximum_vector(a.tangent, b.tangent));
        self.different_active_contact_frames += u64::from(a.contact != b.contact);
        self.different_positive_contact_frames +=
            u64::from(a.positive_contact != b.positive_contact);
        if self.first_outside_gate.is_none()
            && (self.max_position_m > 1e-6
                || self.max_orientation_chord > 1e-7
                || self.max_linear_m_s > 1e-6
                || self.max_angular_rad_s > 1e-5
                || self.max_normal_n_s > 1e-7
                || self.max_tangent_n_s > 1e-7
                || self.different_active_contact_frames != 0
                || self.different_positive_contact_frames != 0)
        {
            self.first_outside_gate =
                Some(json!({"tick":tick,"stationary_reference":a,"transformed_observation":b}));
        }
        Ok(())
    }
}
#[derive(Default, Serialize)]
struct PhysicalAudit {
    direct_default_exact_ticks: usize,
    direct_point_exact_ticks: usize,
    public_pose_twist_checks: usize,
    active_receipts: usize,
    positive_receipts: usize,
    saturated_ticks: usize,
    max_momentum_residual_n_s: f64,
    max_cone_excess_n_s: f64,
    max_saturation_error_n_s: f64,
    max_angular_balance_n_m_s: f64,
    accumulated_tangent_on_sphere_n_s: [f64; 3],
    final_slip_m_s: f64,
    final_relative_speed_m_s: f64,
    final_spin_rad_s: f64,
    relative_dissipation_j: f64,
    table_work_j: f64,
    lab_energy_work_residual_j: f64,
    first_rolling_tick: Option<u64>,
}
fn relative_energy(body: &Body, u: Vector) -> f64 {
    let v = Vector::from(body.linear) - u;
    let w = Vector::from(body.angular);
    0.5 * MASS * (v.x * v.x + v.y * v.y) + 0.5 * (0.4 * MASS * RADIUS * RADIUS) * w.length_squared()
}
fn slip(body: &Body, u: Vector) -> Vector {
    let v = Vector::from(body.linear) - u + Vector::from(body.angular).cross(-RADIUS * Vector::Z);
    Vector::new(v.x, v.y, 0.)
}
fn checked(name: &str, v: f64, tolerance: f64, tick: u64) -> Result<()> {
    if !v.is_finite() || v.abs() > tolerance {
        return Err(format!("{name} exceeded at {tick}: {v:e}, allowed {tolerance:e}").into());
    }
    Ok(())
}
fn owner(sim: &Simulation, id: BodyId) -> u8 {
    pool(sim).body(id.raw()).unwrap().0
}
fn queue_impulse(sim: &mut Simulation, id: BodyId, j: Vector) -> Result<()> {
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<SceneBackend>()
        .unwrap()
        .queue_linear_impulse(id, j.to_array())?;
    Ok(())
}
fn run_case(
    worker: &Path,
    h: u64,
    change: FrameChange,
    reference: Option<&[Sample]>,
    out: &Path,
) -> Result<(Vec<Sample>, bool)> {
    fs::create_dir(out)?;
    let dt = h as f64 / 1000.;
    let document = change.document();
    fs::write(out.join("scene.sdf"), &document)?;
    let mut sim = load_world_configured(
        &document,
        "distributed_rough_mobility.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            step: Some(Duration::from_millis(h)),
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let links = [
        link(&sim, "default::m1_table::link")?,
        link(&sim, "default::m2_sphere::link")?,
    ];
    let mut defaults = state(&sim, &links)?;
    if (
        defaults[0].tier,
        defaults[0].primitive,
        defaults[0].friction,
        defaults[0].size,
        defaults[0].mass,
    ) != (Tier::Prescribed, Primitive::Box, 0.65, [10., 2., 0.4], 10.)
        || (
            defaults[1].tier,
            defaults[1].primitive,
            defaults[1].friction,
            defaults[1].size,
            defaults[1].mass,
        ) != (Tier::Dynamic, Primitive::Sphere, MU, [2. * RADIUS; 3], MASS)
    {
        return Err("loaded SDF changed the finite primitive or its isotropic material".into());
    }
    let initial_owners = links.map(|l| owner(&sim, l.id));
    let u = change.boost();
    let direction = change.drive();
    let drive = KICK * direction;
    // The prescribed slab's speed is in SDF. A normal deferred link velocity
    // establishes the matching sphere speed before the first physics step.
    queue_link_world_velocity(&mut sim, links[1].public, u.to_array(), [0.; 3])?;
    defaults[1].linear = u.to_array();
    let admitted_seeds = defaults.clone();
    let mut points = defaults.clone();
    let mut control = native::Native::new(&defaults, dt, false);
    let mut point_control = native::Native::new(&points, dt, true);
    let mut before = defaults[1].clone();
    let mut drive_state = before.clone();
    let mut samples = Vec::new();
    let mut events = vec![];
    let mut audit = PhysicalAudit::default();
    let mut symmetry = Symmetry::default();
    let mut state_hash = Sha256::new();
    let mut consecutive = 0;
    let mut total = Vector::ZERO;
    let mut previous_owners = initial_owners;
    for tick in 1..=1200 / h {
        let ms = tick * h;
        let impulse = if ms == 200 + h { drive } else { Vector::ZERO };
        if impulse != Vector::ZERO {
            drive_state = before.clone();
            drive_state.linear = (Vector::from(drive_state.linear) + impulse / MASS).to_array();
            queue_impulse(&mut sim, links[1].id, impulse)?;
            control.kick(impulse.to_array());
            point_control.kick(impulse.to_array());
            events.push(json!({"tick":tick,"operation":"public loaded link center impulse","impulse_n_s":impulse.to_array()}));
        }
        let active_owner = owner(&sim, links[1].id);
        let old_pids = backend(&sim).local_worker_pids().unwrap();
        if matches!(ms, 260 | 660) {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<SceneBackend>()
                .unwrap()
                .drop_next_acknowledgement(active_owner)?;
        }
        #[cfg(target_os = "linux")]
        if matches!(ms, 300 | 700) {
            use rustix::process::{Pid, Signal, kill_process};
            let pid = Pid::from_raw(old_pids[active_owner as usize] as i32)
                .ok_or("invalid worker PID")?;
            kill_process(pid, Signal::KILL)?;
        }
        sim.step_exact(1)?;
        if pool(&sim).tick != tick || sim.elapsed() != Duration::from_millis(ms) {
            return Err("public SDF clock disagrees with native barrier".into());
        }
        let pids = backend(&sim).local_worker_pids().unwrap();
        if matches!(ms, 260 | 660) {
            if pids != old_pids {
                return Err("a healthy worker was replaced after cached retry".into());
            }
            events.push(json!({"tick":tick,"operation":"cached ACK retry","owner":active_owner,"pids":pids}));
        }
        #[cfg(target_os = "linux")]
        if matches!(ms, 300 | 700) {
            if pids[active_owner as usize] == old_pids[active_owner as usize] {
                return Err("active worker PID did not change".into());
            }
            events.push(json!({"tick":tick,"operation":"active owner killed and recovered","owner":active_owner,"old_pids":old_pids,"new_pids":pids}));
        }
        let current_owners = links.map(|l| owner(&sim, l.id));
        if previous_owners != current_owners {
            events.push(json!({"tick":tick,"operation":"automatic public ownership change","before":previous_owners,"after":current_owners}));
            previous_owners = current_owners;
        }
        let bodies = state(&sim, &links)?;
        audit.public_pose_twist_checks += 2;
        let pairs = physical_pairs(&sim, &links)?;
        let default = control.step(&mut defaults);
        let point = point_control.step(&mut points);
        let expected = default.active.then_some(ContactImpulse {
            a: 1,
            b: 2,
            normal_impulse: default.normal_n_s,
        });
        if defaults != bodies || pairs.as_slice() != expected.as_slice() {
            fs::write(
                out.join("first-control-difference.json"),
                serde_json::to_vec_pretty(
                    &json!({"tick":tick,"distributed_bodies":bodies,"native_default_bodies":defaults,"native_reaction":default,"public_pairs":pairs}),
                )?,
            )?;
            return Err(
                format!("SDF state or normal impulse diverged from native at {tick}").into(),
            );
        }
        audit.direct_default_exact_ticks += 1;
        if points != bodies
            || point.active != default.active
            || point.normal_n_s.to_bits() != default.normal_n_s.to_bits()
        {
            fs::write(
                out.join("first-point-model-difference.json"),
                serde_json::to_vec_pretty(&json!({
                    "tick":tick,"public_bodies":bodies,"native_point_bodies":points,
                    "public_pairs":pairs,"native_point_reaction":point
                }))?,
            )?;
            return Err(format!(
                "point-Coulomb native control differs from the SDF state at {tick}"
            )
            .into());
        }
        audit.direct_point_exact_ticks += 1;
        if point.points.len() > 1 || (ms > 200 && (!point.active || point.normal_n_s <= 0.)) {
            return Err("single loaded rough point precondition violated".into());
        }
        audit.active_receipts += usize::from(default.active);
        audit.positive_receipts += usize::from(default.normal_n_s > 0.);
        let floors = Vec::<GroundSupport>::new();
        state_hash.update(bincode::serialize(&(tick, &bodies, &pairs, &floors))?);
        let sphere = &bodies[1];
        let jt = Vector::from(point.tangent_on_sphere);
        let jn = Vector::from(point.normal_on_sphere);
        let residual = MASS
            * (Vector::from(sphere.linear) - Vector::from(before.linear) + Vector::Z * (G * dt))
            - impulse
            - jt
            - jn;
        let cone = (jt.length() - MU * point.normal_n_s).max(0.);
        audit.max_momentum_residual_n_s = audit.max_momentum_residual_n_s.max(residual.length());
        audit.max_cone_excess_n_s = audit.max_cone_excess_n_s.max(cone);
        checked("full vector impulse balance", residual.length(), 1e-9, tick)?;
        checked("Coulomb cone excess", cone, 1e-8, tick)?;
        let new_slip = slip(sphere, u);
        if ms > 200 {
            total += jt;
            let old_slip = slip(&before, u) + impulse / MASS;
            if old_slip.length() > 0.08 && new_slip.length() > 0.04 {
                let saturated = (jt + old_slip.normalize() * (MU * point.normal_n_s)).length();
                audit.saturated_ticks += 1;
                audit.max_saturation_error_n_s = audit.max_saturation_error_n_s.max(saturated);
                checked("vector Coulomb saturation", saturated, 1e-6, tick)?;
            }
            let dw = Vector::from(sphere.angular) - Vector::from(drive_state.angular);
            let angular = (0.4 * MASS * RADIUS * RADIUS) * dw - (-RADIUS * Vector::Z).cross(total);
            audit.max_angular_balance_n_m_s = audit.max_angular_balance_n_m_s.max(angular.length());
            checked("angular balance", angular.length(), 0.003, tick)?;
            if new_slip.length() <= 0.01 {
                consecutive += h;
                if consecutive >= 20 && audit.first_rolling_tick.is_none() {
                    audit.first_rolling_tick = Some((ms + h - consecutive) / h);
                }
            } else {
                if audit.first_rolling_tick.is_some() {
                    return Err("slip increased after qualified rolling".into());
                }
                consecutive = 0;
            }
        }
        let sample = Sample::reference_frame(change, ms as f64 / 1000., &bodies, &point);
        if let Some(r) = reference {
            symmetry.observe(tick, &r[tick as usize - 1], &sample)?;
        }
        samples.push(sample);
        before = sphere.clone();
    }
    let tick = 1200 / h;
    audit.accumulated_tangent_on_sphere_n_s = total.to_array();
    audit.final_slip_m_s = slip(&before, u).length();
    audit.final_relative_speed_m_s = (Vector::from(before.linear) - u).dot(direction);
    audit.final_spin_rad_s = Vector::from(before.angular).dot(Vector::Z.cross(direction));
    audit.relative_dissipation_j = relative_energy(&drive_state, u) - relative_energy(&before, u);
    audit.table_work_j = u.dot(total);
    audit.lab_energy_work_residual_j = relative_energy(&before, Vector::ZERO)
        - relative_energy(&drive_state, Vector::ZERO)
        + audit.relative_dissipation_j
        - audit.table_work_j;
    let transition = audit.first_rolling_tick.ok_or("rolling never qualified")?;
    checked(
        "analytic transition time",
        (transition * h - 200) as f64 / 1000. - 1. / (MU * G),
        2. * dt + 0.003,
        tick,
    )?;
    checked(
        "analytic tangent",
        (total + 2. * direction).length(),
        0.02,
        tick,
    )?;
    checked(
        "analytic relative speed",
        audit.final_relative_speed_m_s - 2.5,
        0.01,
        tick,
    )?;
    checked(
        "analytic spin",
        audit.final_spin_rad_s - 2.5 / RADIUS,
        0.05,
        tick,
    )?;
    checked(
        "relative energy loss",
        audit.relative_dissipation_j - 3.5,
        0.05,
        tick,
    )?;
    checked(
        "lab table work",
        audit.lab_energy_work_residual_j,
        1e-8,
        tick,
    )?;
    if change.yaw != 0.
        && change.boost_local == Vector::ZERO
        && let Some(reference) = reference
    {
        plot::write(out, h, reference, &samples)?;
    }
    let passed = symmetry.first_outside_gate.is_none();
    fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
            "admitted_native_seeds":admitted_seeds,"source":"examples/distributed_rough_mobility.sdf","step_ms":h,"ticks":tick,"world_yaw_rad":change.yaw,
            "table_velocity_m_s":u.to_array(),"kick_impulse_n_s":drive.to_array(),"initial_owners":initial_owners,
            "final_owners":previous_owners,"physical":audit,"events":events,"symmetry_vs_stationary_axis":reference.map(|_|&symmetry),
            "trajectory_sha256":format!("{:x}",state_hash.finalize())
        }))?,
    )?;
    Ok((samples, passed))
}
fn compare_series(a: &[Sample], b: &[Sample]) -> Result<Symmetry> {
    let mut stats = Symmetry::default();
    for (i, (reference, changed)) in a.iter().zip(b).enumerate() {
        stats.observe(i as u64 + 1, reference, changed)?;
    }
    Ok(stats)
}
pub(crate) fn qualify(worker: &Path, h: u64, out: &Path) -> Result<Vec<String>> {
    if !matches!(h, 1 | 2 | 5 | 10) {
        return Err("choose a supported 1, 2, 5 or 10 ms public interval".into());
    }
    fs::create_dir(out)?;
    let mut axis: Vec<Vec<Sample>> = vec![];
    let mut failed = vec![];
    let mut isolated = vec![];
    for (heading, yaw) in [("axis", 0.), ("oblique", 3f64.atan2(4.))] {
        let mut rest: Option<Vec<Sample>> = None;
        for (index, (name, boost_local)) in [
            ("rest", Vector::ZERO),
            ("parallel", Vector::X),
            ("cross", Vector::new(0.6, -0.7, 0.)),
        ]
        .into_iter()
        .enumerate()
        {
            let label = format!("{heading}-{name}");
            let dir = out.join(&label);
            let (samples, passed) = run_case(
                worker,
                h,
                FrameChange { yaw, boost_local },
                axis.first().map(Vec::as_slice),
                &dir,
            )?;
            if !passed {
                failed.push(label.clone());
            }
            if let Some(reference) = &rest {
                let comparison = compare_series(reference, &samples)?;
                if comparison.first_outside_gate.is_some() && !failed.contains(&label) {
                    failed.push(label.clone());
                }
                isolated.push(json!({"comparison":"Galilean boost with yaw held fixed","case":label,"passed":comparison.first_outside_gate.is_none()}));
                fs::write(
                    dir.join("galilean-vs-same-yaw.json"),
                    serde_json::to_vec_pretty(&comparison)?,
                )?;
            }
            if heading == "oblique" {
                let comparison = compare_series(&axis[index], &samples)?;
                if comparison.first_outside_gate.is_some() && !failed.contains(&label) {
                    failed.push(label.clone());
                }
                isolated.push(json!({"comparison":"rotation with boost held fixed","case":label,"passed":comparison.first_outside_gate.is_none()}));
                fs::write(
                    dir.join("rotation-vs-same-boost.json"),
                    serde_json::to_vec_pretty(&comparison)?,
                )?;
            }
            if name == "rest" {
                rest = Some(samples.clone());
            }
            if heading == "axis" {
                axis.push(samples);
            }
        }
    }
    fs::write(
        out.join("covariance-gates.json"),
        serde_json::to_vec_pretty(&json!({
            "step_ms":h,"all_native_and_analytic_physical_checks_passed":true,
            "outside_declared_frame_covariance_limits":failed,"isolated_covariance":isolated
        }))?,
    )?;
    Ok(failed)
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    let args = Args::parse();
    let failed = qualify(&args.worker, args.step_ms, &args.out)?;
    if !failed.is_empty() {
        return Err(format!("declared numerical covariance exceeded in {failed:?}").into());
    }
    Ok(())
}
