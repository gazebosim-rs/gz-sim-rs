// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use gz_physics::{JointId, PhysicsBackend};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::distributed_hinge::{DistributedGroundedTreeBackend, GroundedTreeSpec};
use gz_sim_scene::distributed_hinge::{DistributedPayloadTreeBackend, PayloadTreeSpec};
use gz_sim_scene::{
    JointHandle, Name,
    distributed_hinge::{
        DistributedStoppedTreeBackend, DistributedTreeBackend, FloatingTreeSpec, StoppedTreeSpec,
        TreeConstruction,
    },
};
use std::{collections::BTreeMap, error::Error, io, num::NonZeroU64};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Clone, Resource)]
pub struct Drive(pub Vec<(u64, JointId, f64)>, pub Option<NonZeroU64>);
impl Drive {
    /// Signed input for the interval that begins at this committed tick.
    pub fn torque(&self, tick: u64, value: f64) -> f64 {
        if self.1.is_some_and(|period| (tick / period.get()) % 2 == 1) {
            -value
        } else {
            value
        }
    }
}

pub fn parse(text: &str) -> std::result::Result<(String, f64), String> {
    let (name, v) = text
        .split_once('=')
        .ok_or("expected joint_name=torque_nm")?;
    let torque = v.parse::<f64>().map_err(|e| e.to_string())?;
    if name.is_empty() || !torque.is_finite() {
        return Err("expected a joint and finite torque".into());
    }
    Ok((name.into(), torque))
}

pub fn resolve(
    sim: &Simulation,
    entries: &[(String, f64)],
    reverse_every: Option<NonZeroU64>,
) -> Result<Drive> {
    let mut available: BTreeMap<u64, (JointId, &str)> = BTreeMap::new();
    for e in sim.world().iter_entities() {
        if let (Some(j), Some(id), Some(name)) =
            (e.get::<JointHandle>(), e.get::<EntityId>(), e.get::<Name>())
        {
            available.insert(id.get(), (j.id(), &name.0));
        }
    }
    let mut result = Vec::new();
    if entries.is_empty() {
        for (i, (public, (native, _))) in available.into_iter().enumerate() {
            let strength = 0.80 + 0.11 * (i % 5) as f64;
            result.push((
                public,
                native,
                if i % 2 == 0 { strength } else { -strength },
            ));
        }
    } else {
        for (name, torque) in entries {
            let mut matches = available
                .iter()
                .filter(|(_, (_, full))| *full == name || full.rsplit("::").next() == Some(name));
            let (&id, &(native, _)) = matches
                .next()
                .ok_or_else(|| format!("unknown joint {name}"))?;
            if matches.next().is_some()
                || result.iter().any(|&(prior, _, _)| prior == id)
                || !torque.is_finite()
            {
                return Err(format!("ambiguous, repeated, or nonfinite drive for {name}").into());
            }
            result.push((id, native, *torque));
        }
        result.sort_by_key(|row| row.0);
    }
    Ok(Drive(result, reverse_every))
}

pub fn install(
    sim: &mut Simulation,
    entries: &[(String, f64)],
    reverse_every: Option<NonZeroU64>,
) -> Result<Drive> {
    let drive = resolve(sim, entries, reverse_every)?;
    if sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedPayloadTreeBackend>()
        .is_some()
    {
        return install_typed::<PayloadTreeSpec>(sim, drive);
    }
    if sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedGroundedTreeBackend>()
        .is_some()
    {
        return install_typed::<GroundedTreeSpec>(sim, drive);
    }
    if sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedStoppedTreeBackend>()
        .is_some()
    {
        install_typed::<StoppedTreeSpec>(sim, drive)
    } else {
        install_typed::<FloatingTreeSpec>(sim, drive)
    }
}
fn install_typed<S: TreeConstruction + Send + Sync + 'static>(
    sim: &mut Simulation,
    drive: Drive,
) -> Result<Drive> {
    let host = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<DistributedTreeBackend<S>>()
        .ok_or("expected floating tree host")?;
    for &(id, _, torque) in &drive.0 {
        let torque = drive.torque(host.pool().observation().tick, torque);
        if host
            .pool()
            .pending_joint_effort(id)
            .is_some_and(|e| e != torque)
        {
            return Err(
                format!("saved torque on joint {id} differs from its selected drive").into(),
            );
        }
    }
    sim.world_mut().insert_resource(drive.clone());
    sim.add_systems(Phase::PreStep, command::<S>);
    Ok(drive)
}

fn command<S: TreeConstruction + Send + Sync + 'static>(
    time: Res<UpdateInfo>,
    drive: Res<Drive>,
    mut backend: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
    mut required: Local<Vec<(JointId, f64)>>,
) {
    if time.dt.is_zero() {
        return;
    }
    let Some(host) = backend.downcast_mut::<DistributedTreeBackend<S>>() else {
        fault.report(io::Error::other("floating tree backend replaced"));
        return;
    };
    required.clear();
    for &(id, native, torque) in &drive.0 {
        let torque = drive.torque(host.pool().observation().tick, torque);
        match host.pool().pending_joint_effort(id) {
            Some(e) if e == torque => (),
            Some(_) => {
                fault.report(io::Error::other(format!(
                    "saved torque differs for joint {id}"
                )));
                return;
            }
            None => required.push((native, torque)),
        }
    }
    // The saved subset already belongs to this interval. All missing contributions
    // are validated and published together, even when they span both branches.
    let pool = host.pool();
    if let Err(error) = host.stage_step_efforts_at(pool.epoch(), pool.observation().tick, &required)
    {
        fault.report(error);
    }
}
