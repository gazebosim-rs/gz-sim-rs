// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the two already-warm native worlds across one selected interval.
use super::{Result, native::Control};
use gz_physics::BodyId;
use gz_physics_distributed::{
    physics::{Body, BodyWrench, ContactImpulse, GroundSupport, NativeSolverProfile},
    pool::{PoolNativeRecovery, WorkerPool},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

pub struct Probe {
    tick: u64,
    directory: PathBuf,
    worker: Control,
    reference: Control,
    before: Value,
}
fn decode(
    bodies: &[Body],
    handles: BTreeMap<u64, BodyId>,
    bytes: &[u8],
    profile: NativeSolverProfile,
) -> Result<Control> {
    let mut solver = MultiFidelityBackend::restore_solver_with_profile(bytes, profile)?;
    solver.set_partition_gravity(super::scene::GRAVITY)?;
    // The independent control maps local native indices to public IDs through its
    // body array. Verify that the selected complete native world has dense indices.
    let mut rows = bodies.to_vec();
    rows.sort_by_key(|b| handles[&b.id]);
    if rows
        .iter()
        .enumerate()
        .any(|(i, b)| handles[&b.id].raw() != i as u64)
    {
        return Err("selected native order probe requires contiguous body handles".into());
    }
    Ok(Control {
        solver,
        bodies: rows,
        handles,
    })
}
fn local_contacts(c: &Control) -> Vec<Value> {
    c.solver.rigid_contact_observations().into_iter().map(|p|{
        let ia=p.a.raw();
        let ib=p.b.raw();
        json!({"local":[ia,ib],"public_in_local_order":[c.bodies[ia as usize].id,c.bodies[ib as usize].id],"active":p.active,"normal_impulse":p.normal_impulse})
    }).collect()
}
impl Probe {
    pub fn capture(
        directory: &Path,
        pool: &mut WorkerPool,
        reference: &mut Control,
    ) -> Result<Self> {
        let tick = pool.frame().tick;
        let path = directory.join("order-before.gzpool");
        pool.save_native_recovery(&path)?;
        // Verify the ordinary complete worker checksums, epoch, authority and native images
        // before extracting the exact already-authorized native byte vector for direct replay.
        let plan = PoolNativeRecovery::open(&path)?;
        if plan.frame() != pool.frame() {
            return Err("order capture disagrees with the public barrier".into());
        }
        let owner = plan
            .frame()
            .workers
            .iter()
            .position(|w| {
                w.snapshot.bodies.len() == reference.bodies.len() && w.components.len() == 1
            })
            .ok_or("order capture requires one complete native component on one worker")?;
        let raw = fs::read(path)?;
        let encoded: Value = serde_json::from_slice(&raw[40..])?;
        let component = &encoded["images"][owner]["partition"]["solvers"][0];
        let bindings: Vec<u64> = serde_json::from_value(component["bindings"].clone())?;
        let worker_bytes: Vec<u8> = serde_json::from_value(component["native"].clone())?;
        let bodies = &plan.frame().workers[owner].snapshot.bodies;
        let handles = bodies
            .iter()
            .zip(bindings)
            .map(|(b, h)| (b.id, BodyId::from_raw(h)))
            .collect();
        let worker = decode(bodies, handles, &worker_bytes, pool.native_solver_profile())?;
        // Checkpoints seal the unforced boundary. The external gravity component
        // is restored before either world advances and does not alter a native cache.
        reference.solver.set_partition_gravity([0.; 3])?;
        let capture = reference
            .solver
            .checkpoint_solver_with_profile(NativeSolverProfile::Extended64);
        reference
            .solver
            .set_partition_gravity(super::scene::GRAVITY)?;
        let reference_bytes = capture?;
        fs::write(
            directory.join("order-before-reference.native"),
            &reference_bytes,
        )?;
        let reference = decode(
            &reference.bodies,
            reference.handles.clone(),
            &reference_bytes,
            NativeSolverProfile::Extended64,
        )?;
        let before = json!({"tick":tick,"worker_owner":owner,
            "worker_native_sha256":format!("{:x}",Sha256::digest(&worker_bytes)),
            "reference_native_sha256":format!("{:x}",Sha256::digest(&reference_bytes)),
            "worker_handles":worker.handles.iter().map(|(id,h)|(*id,h.raw())).collect::<Vec<_>>(),
            "reference_handles":reference.handles.iter().map(|(id,h)|(*id,h.raw())).collect::<Vec<_>>(),
            "worker_native_pairs":local_contacts(&worker),"reference_native_pairs":local_contacts(&reference)});
        Ok(Self {
            tick: tick + 1,
            directory: directory.into(),
            worker,
            reference,
            before,
        })
    }
    pub fn tick(&self) -> u64 {
        self.tick
    }
    pub fn compare(
        mut self,
        loads: &[BodyWrench],
        worker_rate: u8,
        actual: &[u8],
        expected: &[u8],
    ) -> Result<()> {
        let mut results = vec![];
        for (name, c, rate, live) in [
            ("worker", &mut self.worker, worker_rate, actual),
            ("reference", &mut self.reference, 10, expected),
        ] {
            let (mut pairs, floors): (Vec<ContactImpulse>, Vec<GroundSupport>) =
                c.advance(rate, loads, &[])?;
            for p in &mut pairs {
                if p.a > p.b {
                    std::mem::swap(&mut p.a, &mut p.b);
                }
            }
            pairs.sort_by_key(|p| (p.a, p.b));
            let mut bodies = c.bodies.iter().collect::<Vec<_>>();
            bodies.sort_by_key(|b| b.id);
            let replay = bincode::serialize(&(self.tick, bodies, &pairs, &floors))?;
            if replay != live {
                return Err(
                    format!("{name} native-only replay differs from the live trajectory").into(),
                );
            }
            results.push(json!({"world":name,"substeps":rate,"exact_live_physical_bytes":true,
                "sha256":format!("{:x}",Sha256::digest(&replay)),"poststep_native_pairs":local_contacts(c)}));
        }
        fs::write(
            self.directory.join("order-replay.json"),
            serde_json::to_vec_pretty(&json!({
                "before":self.before,"tick":self.tick,"loads":loads,"direct_native_replays":results,
                "same_poststep_physical_bytes":actual==expected
            }))?,
        )?;
        Ok(())
    }
}
