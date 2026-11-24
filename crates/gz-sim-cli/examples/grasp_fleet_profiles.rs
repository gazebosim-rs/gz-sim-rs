// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify real fleet placements against one native ordinary Gazebo world.
#[allow(dead_code)]
#[path = "support/floating_tree_drive.rs"]
mod floating_tree_drive;
#[allow(dead_code)]
#[path = "support/grasp_drive.rs"]
mod grasp_drive;
#[path = "support/grasp_fleet_drive.rs"]
mod grasp_fleet_drive;
#[path = "support/tree_visuals.rs"]
mod tree_visuals;
use clap::Parser;
use gz_physics::{DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, PhysicsEngine, Velocity, WorldPose,
    distributed_hinge::{
        DistributedGraspFleetBackend, GraspFleetLayout, load_distributed_grasp_fleet_world,
    },
    load_world_with_sdf_step,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    sdf: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    storage: PathBuf,
    #[arg(long,value_parser=gz_sim_cli::parse_grasp_fleet_placement)]
    fleet: GraspFleetLayout,
    #[arg(long,value_enum,default_value_t=grasp_fleet_drive::FleetSchedule::Bilateral)]
    schedule: grasp_fleet_drive::FleetSchedule,
    /// Quantify differences against ordinary for a deliberately difficult solver-order control.
    #[arg(long)]
    measure_divergence: bool,
    /// Save source-native full graphs at this tick and its preceding interval for solver-order diagnosis.
    #[arg(long)]
    probe_at: Option<u32>,
    #[arg(long,default_value_t=1650,value_parser=clap::value_parser!(u32).range(1..))]
    ticks: u32,
    /// Migrate and replace the worker for a gripper and save the durable viewer cut here.
    #[arg(long)]
    record_at: Option<u32>,
    output: PathBuf,
}
fn backend(s: &Simulation) -> &DistributedGraspFleetBackend {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
#[derive(Default, serde::Serialize)]
struct Divergence {
    first_body: Option<Value>,
    first_contact: Option<Value>,
    max_position_m: f64,
    max_orientation_component: f64,
    max_linear_velocity_m_s: f64,
    max_angular_velocity_rad_s: f64,
    max_joint_angle_rad: f64,
    max_joint_rate_rad_s: f64,
    max_normal_impulse_ns: f64,
    max_warm_impulse_ns: f64,
    max_tangent_impulse_ns: f64,
    changed_active_link_ticks: u64,
}
fn delta<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt()
}
fn compare(
    s: &Simulation,
    normal: &Simulation,
    tick: u32,
    visuals: bool,
    measure: bool,
    deviations: &mut Divergence,
) -> Result<Option<Value>> {
    let p = backend(s).pool();
    let ordinary = normal
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    if p.tick() != u64::from(tick)
        || s.elapsed() != normal.elapsed()
        || s.elapsed().as_millis() != u128::from(tick)
    {
        return Err("fleet clock mismatch".into());
    }
    let body = |public| {
        normal
            .world()
            .get::<BodyHandle>(entity_by_id(normal.world(), public)?)
            .map(BodyHandle::id)
    };
    let mut rows = Vec::new();
    for (a, owner) in p.spec().assemblies.iter().zip(p.owners()) {
        let model = a.payload.tree.model_id;
        let state = p.observation(model).unwrap();
        let part = p.payload_state(model).unwrap();
        for l in state.links.iter().chain(std::iter::once(part)) {
            let actual = ordinary
                .state(body(l.id).ok_or("missing body")?)
                .ok_or("missing ordinary state")?;
            let e = entity_by_id(s.world(), l.id).ok_or("missing fleet ECS link")?;
            let re = entity_by_id(normal.world(), l.id).ok_or("missing ordinary ECS link")?;
            let pose = s.world().get::<WorldPose>(e).unwrap();
            let rp = normal.world().get::<WorldPose>(re).unwrap();
            let vel = s.world().get::<Velocity>(e).unwrap();
            let rv = normal.world().get::<Velocity>(re).unwrap();
            if (pose.translation, pose.rotation, vel.linear, vel.angular)
                != (rp.translation, rp.rotation, rv.linear, rv.angular)
                || (l.position, l.orientation, l.velocity, l.angular_velocity)
                    != (
                        actual.position,
                        actual.orientation,
                        actual.velocity,
                        actual.angular_velocity,
                    )
            {
                if !measure {
                    return Err(format!("fleet link {} differs from ordinary at tick {tick}: position {:?}/{:?} velocity {:?}/{:?}",l.id,l.position,actual.position,l.velocity,actual.velocity).into());
                }
                if deviations.first_body.is_none() {
                    deviations.first_body = Some(
                        json!({"tick":tick,"body":l.id,"fleet":l,"ordinary":{"position":actual.position,"orientation":actual.orientation,"velocity":actual.velocity,"angular_velocity":actual.angular_velocity}}),
                    );
                }
                deviations.max_position_m = deviations
                    .max_position_m
                    .max(delta(l.position, actual.position));
                let rot = l
                    .orientation
                    .iter()
                    .zip(actual.orientation)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0f64, f64::max);
                deviations.max_orientation_component =
                    deviations.max_orientation_component.max(rot);
                deviations.max_linear_velocity_m_s = deviations
                    .max_linear_velocity_m_s
                    .max(delta(l.velocity, actual.velocity));
                deviations.max_angular_velocity_rad_s = deviations
                    .max_angular_velocity_rad_s
                    .max(delta(l.angular_velocity, actual.angular_velocity));
            }
        }
        let mut motors = Vec::new();
        for j in &state.joints {
            let e = entity_by_id(normal.world(), j.joint_id).unwrap();
            let id = normal.world().get::<JointHandle>(e).unwrap().id();
            let sample = normal.world().get::<JointSample>(e).unwrap();
            let actual = s
                .world()
                .get::<JointSample>(entity_by_id(s.world(), j.joint_id).unwrap())
                .unwrap();
            let motor = p.velocity_motor(j.joint_id);
            if actual.time != sample.time
                || actual.epoch != sample.epoch
                || motor != ordinary.motor_revolute_velocity(id)?
                || j.pending_effort != ordinary.pending_joint_effort(id)
            {
                return Err(format!(
                    "fleet clock or held command on joint {} differs at {tick}",
                    j.joint_id
                )
                .into());
            }
            if (j.q, j.qdot) != (sample.state.position, sample.state.velocity)
                || actual.state != sample.state
            {
                if !measure {
                    return Err(format!("fleet joint {} differs at tick {tick}", j.joint_id).into());
                }
                deviations.max_joint_angle_rad = deviations
                    .max_joint_angle_rad
                    .max((j.q - sample.state.position).abs());
                deviations.max_joint_rate_rad_s = deviations
                    .max_joint_rate_rad_s
                    .max((j.qdot - sample.state.velocity).abs());
            }
            if visuals {
                motors.push(json!({"joint_id":j.joint_id,"target_velocity":motor.map(|m|m.target_velocity),"max_force":motor.map(|m|m.max_force)}));
            }
        }
        let mut contacts = Vec::new();
        for (l, r) in state
            .links
            .iter()
            .zip(p.payload_reactions(model).ok_or("missing fleet contacts")?)
        {
            let physical =
                ordinary.payload_tree_reaction(body(l.id).unwrap(), body(part.id).unwrap())?;
            if physical != *r {
                if !measure {
                    return Err(format!(
                        "fleet contact {} with {} differs at tick {tick}: {physical:?}/{r:?}",
                        l.id, part.id
                    )
                    .into());
                }
                if deviations.first_contact.is_none() {
                    deviations.first_contact = Some(
                        json!({"tick":tick,"link":l.id,"payload":part.id,"ordinary":physical,"fleet":r}),
                    );
                }
                deviations.max_normal_impulse_ns = deviations
                    .max_normal_impulse_ns
                    .max((r.normal_impulse - physical.normal_impulse).abs());
                deviations.max_warm_impulse_ns = deviations
                    .max_warm_impulse_ns
                    .max((r.warmstart_normal_impulse - physical.warmstart_normal_impulse).abs());
                deviations.max_tangent_impulse_ns = deviations
                    .max_tangent_impulse_ns
                    .max(delta(r.tangent_impulse, physical.tangent_impulse));
                deviations.changed_active_link_ticks += u64::from(r.active != physical.active);
            }
            if visuals {
                contacts.push(json!({"link_id":l.id,"reaction":r}));
            }
        }
        if visuals {
            rows.push(json!({"model_id":model,"owner":owner,"state":state,"payload":part,"velocity_motors":motors,"payload_reactions":contacts}));
        }
    }
    if p.is_coupled() {
        let mut groups = Vec::new();
        for a in &p.spec().assemblies {
            let tree = &a.payload.tree;
            let root = tree
                .links
                .iter()
                .find(|l| tree.joints.iter().all(|j| j.child_id != l.id))
                .ok_or("missing tree root")?;
            groups.push((body(root.id).unwrap(), body(a.payload.payload.id).unwrap()));
        }
        let groups = groups
            .try_into()
            .map_err(|_| "expected two coupled grippers")?;
        if tick > 0
            && (p.layout() == GraspFleetLayout::Coupled
                || matches!(p.layout(), GraspFleetLayout::Replay { .. })
                || p.joined_at().is_none())
            && p.coupled_image() != Some(ordinary.checkpoint_grasp_fleet(&groups)?.as_slice())
        {
            return Err(format!("whole native fleet image differs at tick {tick}").into());
        }
        for pair in p.cross_reactions() {
            let physical = ordinary.sphere_pair_reaction(
                body(pair.first_body).unwrap(),
                body(pair.second_body).unwrap(),
            )?;
            if physical != pair.reaction {
                return Err(format!(
                    "coupled cross contact {} {} differs at tick {tick}",
                    pair.first_body, pair.second_body
                )
                .into());
            }
        }
    }
    if !visuals {
        return Ok(None);
    }
    let poses = tree_visuals::poses(s)?;
    if !measure && poses != tree_visuals::poses(normal)? {
        return Err(format!("fleet rendered poses differ at tick {tick}").into());
    }
    Ok(Some(
        json!({"tick":tick,"epoch":p.epoch(),"generation":p.generation(),"owners":p.owners(),"coupled":p.is_coupled(),"joined_at":p.joined_at(),"assemblies":rows,"cross_reactions":p.cross_reactions(),"visuals":poses.into_values().collect::<Vec<_>>()}),
    ))
}
fn main() -> Result<()> {
    let a = Args::parse();
    if a.record_at.is_some_and(|t| t == 0 || t >= a.ticks) {
        return Err("record a positive cut with a following interval".into());
    }
    let xml = fs::read_to_string(&a.sdf)?;
    let source = a.sdf.to_string_lossy();
    let temp = tempfile::tempdir_in(&a.storage)?;
    let open = || {
        load_distributed_grasp_fleet_world(
            &xml,
            &source,
            "default",
            &a.worker,
            temp.path(),
            Some(4),
            a.fleet,
        )
    };
    let mut local = load_world_with_sdf_step(
        &xml,
        &source,
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut host = open()?;
    let drive = grasp_fleet_drive::FleetDrive::resolve(
        &local,
        &backend(&host).pool().spec().assemblies,
        a.schedule,
    )?;
    grasp_fleet_drive::FleetDrive::install(&mut host, a.schedule)?;
    fs::create_dir(&a.output)?;
    let mut frames = a
        .record_at
        .map(|_| File::create(a.output.join("states.jsonl")).map(BufWriter::new))
        .transpose()?;
    let mut times = Vec::new();
    let mut deviations = Divergence::default();
    for tick in 0..=a.ticks {
        if tick > 0 {
            drive.apply(
                &mut local.world_mut().resource_mut::<PhysicsBackend>(),
                u64::from(tick - 1),
            )?;
            local.step_exact(1)?;
            let start = Instant::now();
            host.step_exact(1)
                .map_err(|e| gz_sim_cli::with_fault(&host, e))?;
            times.push(u64::try_from(start.elapsed().as_nanos())?);
        }
        if tick > 0
            && a.probe_at
                .is_some_and(|t| t == tick || t.checked_sub(1) == Some(tick))
        {
            let native = local
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let p = backend(&host).pool();
            let mut roots = Vec::new();
            let mut labels = Vec::new();
            for assembly in &p.spec().assemblies {
                let a = &assembly.payload;
                let root = a
                    .tree
                    .links
                    .iter()
                    .find(|l| a.tree.joints.iter().all(|j| j.child_id != l.id))
                    .unwrap()
                    .id;
                let id = |public| {
                    local
                        .world()
                        .get::<BodyHandle>(entity_by_id(local.world(), public).unwrap())
                        .unwrap()
                        .id()
                };
                roots.push((id(root), id(a.payload.id)));
                for b in a.tree.links.iter().chain(std::iter::once(&a.payload)) {
                    labels.push(
                        json!({"public":b.id,"native":id(b.id).raw(),"model":a.tree.model_id}),
                    );
                }
            }
            let dir = a.output.join("native-probe");
            fs::create_dir_all(&dir)?;
            fs::write(
                dir.join(format!("{tick}-labels.json")),
                serde_json::to_vec(&labels)?,
            )?;
            fs::write(
                dir.join(format!("{tick}-whole.gzfc")),
                native.checkpoint_grasp_fleet(
                    &roots.try_into().map_err(|_| "expected two native groups")?,
                )?,
            )?;
            if let gz_sim_scene::distributed_hinge::ManagedGraspFleet::Independent(session) = p {
                for group in &p.spec().assemblies {
                    let model = group.payload.tree.model_id;
                    fs::write(
                        dir.join(format!("{tick}-{model}.gzfr")),
                        session.image(model).ok_or("missing member image")?,
                    )?;
                }
            }
        }
        if a.record_at == Some(tick) {
            compare(
                &host,
                &local,
                tick,
                false,
                a.measure_divergence,
                &mut Divergence::default(),
            )?;
            let mut be = host.world_mut().resource_mut::<PhysicsBackend>();
            let fleet = be.downcast_mut::<DistributedGraspFleetBackend>().unwrap();
            let model = fleet.pool().spec().assemblies[0].payload.tree.model_id;
            let original = fleet.pool().owners()[0];
            let dest = (0..fleet.pool().width())
                .find(|id| !fleet.pool().owners().contains(id))
                .ok_or("missing spare worker")?;
            if a.fleet == GraspFleetLayout::Coupled {
                fleet.transfer_coupled(dest)?;
            } else {
                fleet.transfer(model, dest)?;
            }
            fleet.replace_worker(dest)?;
            if a.fleet != GraspFleetLayout::Coupled {
                fleet.transfer(model, original)?;
                fleet.replace_worker(original)?;
            }

            let save = a.output.join("cut");
            fs::create_dir(&save)?;
            fs::copy(
                temp.path().join(a.fleet.archive()),
                save.join(a.fleet.archive()),
            )?;
            drop(host);
            host = open()?;
            grasp_fleet_drive::FleetDrive::install(&mut host, a.schedule)?;
        }
        if let Some(row) = compare(
            &host,
            &local,
            tick,
            frames.is_some(),
            a.measure_divergence,
            &mut deviations,
        )? {
            let f = frames.as_mut().unwrap();
            serde_json::to_writer(&mut *f, &row)?;
            writeln!(f)?;
        }
    }
    if let Some(mut f) = frames {
        f.flush()?;
    }
    fs::write(
        a.output.join("solver-difference.json"),
        serde_json::to_vec_pretty(&deviations)?,
    )?;
    fs::write(a.output.join("samples.json"), serde_json::to_vec(&times)?)?;
    times.sort_unstable();
    let p = backend(&host).pool();
    let summary = json!({"ticks":p.tick(),"groups":p.owners().len(),"owners":p.owners(),"placement":p.layout(),"cut_tick":a.record_at,"p50_ns":times[(times.len()-1)/2],"p95_ns":times[(times.len()*95).div_ceil(100)-1],"total_ns":times.iter().sum::<u64>()});
    fs::write(
        a.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
