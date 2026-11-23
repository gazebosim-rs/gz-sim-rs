// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Repeatedly release two authored feet. Check native reactions, actuator inputs and edit epochs.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step,
    model_fidelity, prepare_model_template, primary_contact::last_refinement,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
    set_model_fidelity, spawn_model_template,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_primary_endurance.sdf");
const SETTLE: u64 = 1400;
const CYCLE: u64 = 720;
const STROKES: u64 = 8;
const DT: f64 = 0.0005;
const STROKE: [(u64, u64, u64, f64); 3] = [
    (48, 140, 120, 0.30),
    (72, 110, 180, 0.45),
    (96, 80, 240, 0.60),
];
const PHASE: [u64; 4] = [0, 64, 32, 96];
fn speed(t: u64, stroke: u64) -> f64 {
    let (down, plateau, brake, up) = STROKE[(stroke % 3) as usize];
    if t < down {
        -1.25
    } else if t < down + 24 {
        0.
    } else if t < down + 24 + plateau {
        up
    } else if t < down + 24 + plateau + brake {
        up * (1. - (t - down - 24 - plateau) as f64 / brake as f64 - 0.5 / brake as f64)
    } else {
        0.
    }
}
fn id(s: &Simulation, name: &str) -> Result<u64> {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == format!("default::{name}"))
        })
        .and_then(|e| e.get::<EntityId>())
        .map(EntityId::get)
        .ok_or_else(|| format!("missing {name}").into())
}
fn body(s: &Simulation, id: u64) -> Result<BodyId> {
    Ok(s.world()
        .get::<BodyHandle>(entity_by_id(s.world(), id).ok_or("missing entity")?)
        .ok_or("missing body")?
        .id())
}
fn native(s: &Simulation) -> &MultiFidelityBackend {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn as_value(b: BodyState) -> Value {
    json!({"position":b.position,"orientation":b.orientation,"velocity":b.velocity,"angular_velocity":b.angular_velocity})
}
#[derive(Clone, Copy)]
struct Rig {
    model: u64,
    lift: u64,
    pad_link: u64,
    foot: BodyId,
    upper: BodyId,
    pad: BodyId,
}
impl Rig {
    fn new(s: &Simulation, m: &str, l: &str) -> Result<Self> {
        let foot = body(s, id(s, &format!("{m}::foot"))?)?;
        let upper = body(s, id(s, &format!("{m}::upper"))?)?;
        let pad_link = id(s, &format!("{l}::pad"))?;
        Ok(Self {
            model: id(s, m)?,
            lift: id(s, l)?,
            pad_link,
            foot,
            upper,
            pad: body(s, pad_link)?,
        })
    }
    fn states(&self, s: &Simulation) -> Option<[BodyState; 3]> {
        let n = native(s);
        Some([
            n.state(self.foot)?,
            n.state(self.upper)?,
            n.state(self.pad)?,
        ])
    }
    fn reaction(&self, s: &Simulation, tick: u64) -> (f64, bool, u32) {
        let key = (self.foot.min(self.pad), self.foot.max(self.pad));
        if let Some(e) = last_refinement(s).filter(|e| e.iteration == tick) {
            let hit = e
                .requested_pairs
                .iter()
                .any(|p| p.foot == self.foot && p.support == self.pad);
            let impulse = e
                .interval
                .pairs
                .iter()
                .find(|c| (c.a, c.b) == key)
                .map_or(0., |p| p.normal_impulse);
            (impulse, hit, e.interval.native_steps)
        } else {
            let impulse = native(s)
                .rigid_contact_observations()
                .iter()
                .find(|p| (p.a, p.b) == key)
                .map_or(0., |p| p.normal_impulse);
            (impulse, false, 1)
        }
    }
}
/// Accumulate real applied point vectors on the foot for one measured public tick.
fn applied(rig: Rig, s: &Simulation, tick: u64) -> (DVec3, Option<DVec3>) {
    let key = (rig.foot.min(rig.pad), rig.foot.max(rig.pad));
    let sign = if key.0 == rig.foot { 1. } else { -1. };
    if let Some(e) = last_refinement(s).filter(|e| e.iteration == tick) {
        let p = e
            .interval
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == key)
            .expect("positive native contact");
        return (
            DVec3::from(p.normal_impulse_on_a) * sign,
            p.tangent_impulse_on_a.map(|v| DVec3::from(v) * sign),
        );
    }
    let mut normal = DVec3::ZERO;
    let mut tangent = Some(DVec3::ZERO);
    for patch in native(s)
        .rigid_contact_patches()
        .into_iter()
        .filter(|p| (p.a, p.b) == key)
    {
        for p in patch.points {
            normal += DVec3::from(patch.normal_on_a) * (p.normal_impulse * sign);
            tangent = tangent
                .zip(p.tangent_impulse_on_a)
                .map(|(sum, t)| sum + DVec3::from(t) * sign);
        }
    }
    (normal, tangent)
}
fn gap([f, _, p]: [BodyState; 3]) -> f64 {
    let vertical = DQuat::from_array(f.orientation).conjugate() * DVec3::Z;
    f.position[2] - p.position[2] - 0.06 - vertical.abs().dot(DVec3::new(0.18, 0.10, 0.06))
}
#[derive(Default)]
struct Detector {
    armed: bool,
    peak_gap: f64,
}
impl Detector {
    fn observe(&mut self, gap: f64, impulse: f64) -> Option<f64> {
        if impulse == 0. && gap > 0.001 {
            self.armed = true;
            self.peak_gap = self.peak_gap.max(gap);
        }
        if impulse > 0. && self.armed {
            self.armed = false;
            return Some(std::mem::take(&mut self.peak_gap));
        }
        None
    }
}
fn mix(hash: &mut u64, b: BodyState) {
    for v in b
        .position
        .into_iter()
        .chain(b.orientation)
        .chain(b.velocity)
        .chain(b.angular_velocity)
    {
        *hash ^= v.to_bits();
        *hash = hash.wrapping_mul(0x100000001b3);
    }
}
fn trial(label: &str, insert: &str) -> Result<(Value, Vec<Value>)> {
    let start = SDF
        .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
        .unwrap();
    let end = start + SDF[start..].find("</plugin>").unwrap() + "</plugin>".len();
    let xml = if label == "ordinary" {
        format!("{}{}", &SDF[..start], &SDF[end..])
    } else {
        SDF.replace("<relaxation_iterations>8", insert)
    };
    let source = sdformat::load_str(&xml, "endurance.sdf", &NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "endurance.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let spare = prepare_model_template(
        &sim,
        &source,
        source.worlds[0]
            .models
            .iter()
            .find(|m| m.name == "spare")
            .unwrap(),
    )?;
    let rigs = [
        Rig::new(&sim, "rocker", "lift")?,
        Rig::new(&sim, "edge_rocker", "edge_lift")?,
    ];
    let witness = id(&sim, "witness::body")?;
    let witness_body = body(&sim, witness)?;
    let mut rows = vec![];
    let mut visual = vec![];
    for epoch in 0..4 {
        if epoch > 0 {
            queue_link_world_wrench(&mut sim, witness, [999., 0., 0.], [0.; 3])?;
            reset_loaded_world(&mut sim, |_| ())?;
            if last_refinement(&sim).is_some()
                || native(&sim).state(witness_body).unwrap().velocity[0] != 0.
                || rigs.iter().any(|r| r.states(&sim).is_none())
            {
                return Err("epoch restore failed".into());
            }
        }
        let mut hash = 0xcbf29ce484222325_u64;
        let mut events = vec![];
        let mut selections = vec![];
        let mut edits = vec![];
        let mut holds = vec![];
        let mut active = [true; 2];
        let mut previous_speed = [0.; 2];
        let mut detectors: [Detector; 2] = Default::default();
        let mut clone_model = None;
        let mut expected_vx = 0.;
        let mut worst_wrench_error = 0.0_f64;
        for boundary in 0..SETTLE + CYCLE * STROKES {
            let tick = boundary + 1;
            let drive = boundary.checked_sub(SETTLE);
            let mut physical_was = [None; 2];
            // Applying the same isolated force every public tick checks single consumption during
            // both shared refinements and ordinary advances, even with a pause between queue and solve.
            let force = if tick % 4 < 2 { 24. } else { -16. };
            queue_link_world_wrench(&mut sim, witness, [force, 0., 0.], [0.; 3])?;
            if drive.is_some_and(|d| d % CYCLE == 0) {
                let d = drive.unwrap();
                sim.set_paused(true);
                let before = sim.elapsed();
                let witness_before = native(&sim).state(witness_body).unwrap();
                if !sim.tick()?.dt.is_zero()
                    || sim.elapsed() != before
                    || native(&sim).state(witness_body) != Some(witness_before)
                {
                    return Err("paused tick consumed input or time".into());
                }
                if epoch == 1 && d / CYCLE == 3 || epoch == 2 && d / CYCLE == 5 {
                    let i = usize::from(epoch == 2);
                    let rig = rigs[i];
                    let pad_state = rig.states(&sim).unwrap()[2];
                    let removed = if i == 0 { rig.lift } else { rig.model };
                    remove_model(&mut sim, removed)?;
                    active[i] = false;
                    if i == 0 {
                        if queue_link_world_velocity(&mut sim, rig.pad_link, [0.; 3], [0.; 3])
                            .is_ok()
                        {
                            return Err("retired identity accepted a new command".into());
                        }
                        let born = spawn_model_template(
                            &mut sim,
                            &spare,
                            WorldPose {
                                translation: pad_state.position,
                                rotation: pad_state.orientation,
                            },
                        )?;
                        clone_model = Some(born);
                        edits.push(json!({"tick":tick,"retired_model":removed,"replacement_model":born,"retired_link":rig.pad_link}));
                    } else {
                        edits.push(json!({"tick":tick,"retired_model":removed}));
                    }
                }
                if epoch == 1
                    && d / CYCLE == 6
                    && let Some(born) = clone_model.take()
                {
                    remove_model(&mut sim, born)?;
                    edits.push(json!({"tick":tick,"retired_clone":born}));
                }
            }
            for (i, rig) in rigs.iter().enumerate() {
                if !active[i] {
                    continue;
                }
                if let Some(d) = drive {
                    let stroke = d / CYCLE;
                    let age = d % CYCLE;
                    let delay = if i == 0 {
                        0
                    } else {
                        PHASE[(stroke % 4) as usize]
                    };
                    // Park each settled assembly between selected strokes where admissible.
                    if age == delay && stroke % 3 == 1 {
                        let held =
                            set_model_fidelity(&mut sim, rig.model, ArticulationTier::Prescribed);
                        holds.push(json!({"tick":tick,"rig":i,"admitted":held.is_ok(),"reason":held.err().map(|e|e.to_string())}));
                    }
                    let cmd = age.checked_sub(delay).map_or(0., |t| speed(t, stroke));
                    if cmd != previous_speed[i] {
                        queue_link_world_velocity(&mut sim, rig.pad_link, [0., 0., cmd], [0.; 3])?;
                        previous_speed[i] = cmd;
                    }
                }
                physical_was[i] = rig.states(&sim);
            }
            sim.step_exact(1)?;
            expected_vx += force * 0.5 * DT;
            let witness_end = native(&sim).state(witness_body).unwrap();
            worst_wrench_error =
                worst_wrench_error.max((witness_end.velocity[0] - expected_vx).abs());
            mix(&mut hash, witness_end);
            if let Some(e) = last_refinement(&sim).filter(|e| e.iteration == tick) {
                if e.epoch != epoch
                    || e.sim_time - e.start_time != sim.step_duration()
                    || e.interval.native_steps != 8
                {
                    return Err("incorrect public receipt clock".into());
                }
                for p in &e.requested_pairs {
                    let i = rigs
                        .iter()
                        .position(|r| r.foot == p.foot && r.pad == p.support)
                        .ok_or("unknown selected pair")?;
                    if !active[i]
                        || model_fidelity(&sim, rigs[i].model)? != ArticulationTier::Dynamic
                    {
                        return Err("retired or prescribed pair requested refinement".into());
                    }
                    selections.push(json!({"tick":tick,"rig":i,"normal_impulse_ns":p.normal_impulse,"first_positive_end_ns":p.first_positive_end.map(|t|t.as_nanos() as u64)}));
                }
            }
            for (i, rig) in rigs.iter().enumerate() {
                if !active[i] {
                    continue;
                }
                let states = rig.states(&sim).unwrap();
                for b in states {
                    mix(&mut hash, b);
                }
                let (impulse, selected, native_steps) = rig.reaction(&sim, tick);
                let clearance = gap(states);
                if let Some(peak) = detectors[i].observe(clearance, impulse) {
                    let [f0, u0, _] = physical_was[i].unwrap();
                    let momentum = DVec3::from(states[0].velocity)
                        + DVec3::from(states[1].velocity)
                        - DVec3::from(f0.velocity)
                        - DVec3::from(u0.velocity)
                        + DVec3::Z * (2. * 9.81 * DT);
                    let (normal, tangent) = applied(*rig, &sim, tick);
                    events.push(json!({"tick":tick,"rig":i,"drive_stroke":drive.map(|d|d/CYCLE),"selected":selected,"native_steps":native_steps,
                        "normal_impulse_ns":impulse,"momentum_balance_ns":momentum.z,"peak_free_gap_m":peak,
                        "normal_on_foot_ns":normal.to_array(),"tangent_on_foot_ns":tangent.map(|t|t.to_array()),"momentum_on_foot_ns":momentum.to_array(),
                        "vector_residual_ns":tangent.map(|t|(normal+t-momentum).to_array())}));
                }
                if i == 0 && epoch == 0 && drive.is_some_and(|d| d < 300) {
                    visual.push(json!({"tick":tick,"time_us":tick*500,"native_steps":native_steps,"solver_iterations":4,
                        "selected":selected,"weak_start":false,"secondary_start":false,"foot":as_value(states[0]),"upper":as_value(states[1]),
                        "pad":as_value(states[2]),"normal_impulse_ns":impulse,"gap_m":clearance}));
                }
            }
        }
        if worst_wrench_error > 2e-10 {
            return Err(format!("one-shot force impulse error {worst_wrench_error}").into());
        }
        rows.push(json!({"epoch":epoch,"public_ticks":SETTLE+CYCLE*STROKES,"native_state_digest":format!("{hash:016x}"),"max_witness_velocity_error_m_s":worst_wrench_error,
            "physical_landings":events,"pair_requests":selections,"edits":edits,"tier_attempts":holds}));
    }
    for key in [
        "native_state_digest",
        "physical_landings",
        "pair_requests",
        "tier_attempts",
    ] {
        if rows[0][key] != rows[3][key] {
            return Err(format!("{label}: clean epoch replay differs in {key}").into());
        }
    }
    Ok((json!({"policy":label,"epochs":rows}), visual))
}
fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: local_primary_endurance OUT")?,
    );
    fs::create_dir_all(dir.join("traces"))?;
    let mut outcomes = vec![];
    let mut traces = vec![];
    for (label, budget) in [
        ("ordinary", ""),
        ("relax8", "<relaxation_iterations>8"),
        ("relax32", "<relaxation_iterations>32"),
    ] {
        let (row, trace) = trial(label, budget)?;
        outcomes.push(row);
        traces.push((label, trace));
    }
    let first = outcomes[0]["epochs"][0]["physical_landings"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["rig"] == 0 && e["drive_stroke"] == 0)
        .ok_or("no measured first stroke")?["tick"]
        .as_u64()
        .unwrap();
    let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]});
    for (label, trace) in traces {
        let frames: Vec<_> = trace
            .into_iter()
            .filter(|p| (first - 8..=first + 40).contains(&p["tick"].as_u64().unwrap()))
            .collect();
        fs::write(
            dir.join(format!("traces/{label}.json")),
            serde_json::to_vec(&json!({"geometry":geometry,"frames":frames}))?,
        )?;
    }
    let lanes = [
        ("ordinary", "Ordinary solve"),
        ("relax8", "8 relaxation passes"),
        ("relax32", "32 relaxation passes"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        dir.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Repeated lift release · first return","pair_totals_only":true,"independent_histories":true,"lanes":lanes}),
        )?,
    )?;
    fs::write(
        dir.join("epochs.json"),
        serde_json::to_vec_pretty(&outcomes)?,
    )?;
    Ok(())
}
