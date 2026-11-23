// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One durable clock for a native articulated worker and a Bevy-hosted XPBD material.
use super::{pool::Cut, *};
use crate::articulation::ledger::{Ledger, MATERIAL_TREE_FILE as FILE};
use bevy_ecs::prelude::{Res, ResMut, Resource};
use bincode::Options;
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Particle, SoftBody, articulated_impact::ImpactSphere};
use gz_sim_core::{Phase, Simulation, StepFault, UpdateInfo};
use std::{
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
    sync::Arc,
    time::Duration,
};
const DT: Duration = Duration::from_millis(1);
const MAGIC: [u8; 8] = *b"GZMTCU01";
const LIMIT: usize = 2 * 1024 * 1024;
const MAX_PARTICLES: usize = 256;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialImpactBarrier {
    /// Accepted endpoint of the advancing 1 ms interval.
    pub tick: u64,
    pub links: Vec<u64>,
    pub restitution: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArticulatedMaterialSpec {
    pub tree: ContactTreeSpec,
    pub material_name: String,
    pub barriers: Vec<MaterialImpactBarrier>,
}
impl ArticulatedMaterialSpec {
    fn check(&self, body: &SoftBody) -> Result<()> {
        use profile::sealed::Recipe;
        self.tree.validate_configuration()?;
        self.tree.tree.root()?;
        if self.material_name.is_empty()
            || self.material_name.len() > 256
            || self.barriers.is_empty()
            || self.barriers.len() > 256
            || !self.barriers.windows(2).all(|v| v[0].tick < v[1].tick)
            || body.particles().len() > MAX_PARTICLES
            || !body.supports_articulated_impact()
        {
            return Err(invalid(
                "invalid material name, barrier sequence, particle count or contact profile",
            ));
        }
        for b in &self.barriers {
            if b.tick == 0
                || !(0. ..=1.).contains(&b.restitution)
                || b.links.is_empty()
                || b.links.len() > contact::MAX_SITES
                || b.links.iter().collect::<BTreeSet<_>>().len() != b.links.len()
                || b.links.iter().any(|id| {
                    self.tree
                        .tree
                        .links
                        .binary_search_by_key(id, |l| l.id)
                        .is_err()
                })
            {
                return Err(invalid(
                    "invalid authored material impact time, target or restitution",
                ));
            }
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialContactLoad {
    pub link: u64,
    pub point: [f64; 3],
    pub local_point: [f64; 3],
    pub normal: [f64; 3],
    pub feature: Vec<usize>,
    pub weights: [f64; 3],
    pub clearance: f64,
    pub on_link: [f64; 3],
    pub on_particles: Vec<(usize, [f64; 3])>,
    pub initial_normal_velocity: f64,
    pub final_normal_velocity: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MaterialContactReceipt {
    pub epoch: u64,
    pub tick: u64,
    pub restitution: f64,
    pub loads: Vec<MaterialContactLoad>,
    pub basis: ContactPointBasis,
    pub native: ContactImpulseReceipt,
    pub kinetic_energy_change: f64,
    pub prescribed_support_work: f64,
    pub dissipated_energy: f64,
}
#[derive(Clone, Serialize, Deserialize)]
struct Saved {
    tree: Cut,
    #[serde(with = "serde_bytes")]
    material: Vec<u8>,
    last: Option<MaterialContactReceipt>,
}
#[derive(Resource)]
struct MaterialBody(SoftBody);
fn soft_step(mut m: ResMut<MaterialBody>, info: Res<UpdateInfo>, fault: Res<StepFault>) {
    if !info.dt.is_zero()
        && let Err(e) = m.0.step(info.dt, &[])
    {
        fault.report(e);
    }
}
fn material_world(body: SoftBody, epoch: u64, tick: u64) -> Result<Simulation> {
    let time = Duration::from_millis(tick);
    let mut w = Simulation::from_committed_clock(
        DT,
        UpdateInfo {
            start_time: time.saturating_sub(DT),
            sim_time: time,
            dt: if tick == 0 { Duration::ZERO } else { DT },
            iteration: tick,
            epoch,
            paused: tick == 0,
        },
        true,
    )
    .map_err(NativeError::from)?;
    w.world_mut().insert_resource(MaterialBody(body));
    w.add_systems(Phase::PhysicsStep, soft_step);
    Ok(w)
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_limit(LIMIT as u64)
        .reject_trailing_bytes()
}
fn boundary(_point: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_point);
}
/// The XPBD and native halves become visible together after one durable publication.
pub struct ArticulatedMaterialPool {
    exe: PathBuf,
    ledger: Arc<Ledger>,
    spec: ArticulatedMaterialSpec,
    cold: SoftBody,
    cold_digest: [u8; 32],
    saved: Saved,
    rigid: Option<ContactTreePool>,
    soft: Simulation,
}
impl ArticulatedMaterialPool {
    fn ledger(dir: &Path, create: bool) -> Result<Arc<Ledger>> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(FILE)? {
            return Err(invalid("directory belongs to a different physics profile"));
        }
        Ok(Arc::new(l))
    }
    fn read(l: &Ledger) -> Result<(ArticulatedMaterialSpec, [u8; 32], Saved)> {
        let mut v = Vec::new();
        File::open(l.archive_path(FILE))?
            .take((LIMIT + 41) as u64)
            .read_to_end(&mut v)?;
        if !(40..=LIMIT + 40).contains(&v.len())
            || v[..8] != MAGIC
            || v[8..40] != Sha256::digest(&v[40..])[..]
        {
            return Err(invalid(
                "incorrect articulated material archive size, version or digest",
            ));
        }
        Ok(codec().deserialize(&v[40..]).map_err(io::Error::other)?)
    }
    fn store(&self, c: &Saved) -> io::Result<()> {
        let bytes = codec()
            .serialize(&(&self.spec, self.cold_digest, c))
            .map_err(io::Error::other)?;
        self.ledger
            .publish_bytes(&self.ledger.archive_path(FILE), &MAGIC, &bytes, boundary)
    }
    pub fn create(
        exe: &Path,
        dir: &Path,
        width: u8,
        spec: ArticulatedMaterialSpec,
        cold: SoftBody,
    ) -> Result<Self> {
        spec.check(&cold)?;
        let packet = cold.checkpoint_articulated_material()?;
        let l = Self::ledger(dir, true)?;
        if l.archive_path(FILE).try_exists()? {
            return Err(invalid("articulated material session already exists"));
        }
        let exe = exe.canonicalize()?;
        let tree = ContactTreePool::contained_cold(&exe, l.clone(), width, spec.tree.clone())?;
        let saved = Saved {
            tree: tree.native_cut().clone(),
            material: packet.clone(),
            last: None,
        };
        let p = Self {
            exe,
            ledger: l,
            spec,
            soft: material_world(cold.clone(), 0, 0)?,
            cold,
            cold_digest: Sha256::digest(packet).into(),
            saved,
            rigid: Some(tree),
        };
        p.store(&p.saved)?;
        Ok(p)
    }
    /// Cold material is the original SDF construction. Its full physics recipe and
    /// initial particle pose/twist must hash identically before any worker starts.
    pub fn resume(exe: &Path, dir: &Path, cold: SoftBody) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let (spec, cold_digest, saved) = Self::read(&l)?;
        spec.check(&cold)?;
        if Sha256::digest(cold.checkpoint_articulated_material()?)[..] != cold_digest {
            return Err(invalid("saved and supplied cold materials differ"));
        }
        if let Some(r) = &saved.last {
            if r.epoch != saved.tree.epoch
                || r.tick > saved.tree.tick()
                || spec
                    .barriers
                    .binary_search_by_key(&r.tick, |b| b.tick)
                    .is_err()
            {
                return Err(invalid("invalid articulated material receipt time"));
            }
            r.validate(&spec, &cold)?;
        }
        let expected = spec
            .barriers
            .iter()
            .rev()
            .find(|b| b.tick <= saved.tree.tick())
            .map(|b| b.tick);
        if saved.last.as_ref().map(|r| r.tick) != expected {
            return Err(invalid("missing or stale authored impact receipt"));
        }
        let mut current = cold.clone();
        current.restore_articulated_material(&saved.material)?;
        let soft = material_world(current, saved.tree.epoch, saved.tree.tick())?;
        let exe = exe.canonicalize()?;
        let rigid = ContactTreePool::contained_restore(
            &exe,
            l.clone(),
            spec.tree.clone(),
            saved.tree.clone(),
        )?;
        Ok(Self {
            exe,
            ledger: l,
            spec,
            cold,
            cold_digest,
            saved,
            rigid: Some(rigid),
            soft,
        })
    }
    pub fn spec(&self) -> &ArticulatedMaterialSpec {
        &self.spec
    }
    pub fn tick(&self) -> u64 {
        self.saved.tree.tick()
    }
    pub fn epoch(&self) -> u64 {
        self.saved.tree.epoch
    }
    pub fn owner(&self) -> u8 {
        self.saved.tree.owner
    }
    pub fn generation(&self) -> u64 {
        self.saved.tree.generation
    }
    pub fn width(&self) -> u8 {
        self.saved.tree.width
    }
    pub fn observation(&self) -> &FloatingTreeObservation {
        &self.saved.tree.snapshot.state
    }
    pub fn native_image(&self) -> Option<&[u8]> {
        self.saved.tree.snapshot.image.native()
    }
    pub fn particles(&self) -> &[Particle] {
        self.soft.world().resource::<MaterialBody>().0.particles()
    }
    pub fn material_image(&self) -> &[u8] {
        &self.saved.material
    }
    pub fn last_impact(&self) -> Option<&MaterialContactReceipt> {
        self.saved.last.as_ref()
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.rigid.as_ref()?.worker_pid(slot)
    }
    pub fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        crate::articulation::check_interval(
            self.rigid.is_some(),
            epoch,
            tick,
            self.epoch(),
            self.tick(),
        )
    }
    fn rollback(&mut self, reopen: bool) -> Result<()> {
        self.rigid.take();
        let mut body = self.cold.clone();
        body.restore_articulated_material(&self.saved.material)?;
        self.soft = material_world(body, self.epoch(), self.tick())?;
        if reopen {
            self.rigid = Some(ContactTreePool::contained_restore(
                &self.exe,
                self.ledger.clone(),
                self.spec.tree.clone(),
                self.saved.tree.clone(),
            )?);
        }
        Ok(())
    }
    fn change(
        &mut self,
        mut op: impl FnMut(
            &mut ContactTreePool,
            &mut Simulation,
            &mut Option<MaterialContactReceipt>,
        ) -> Result<()>,
    ) -> Result<()> {
        let mut retry = true;
        let next = loop {
            let tree = self
                .rigid
                .as_mut()
                .ok_or(invalid("material session requires recovery"))?;
            let mut last = self.saved.last.clone();
            let next = (|| {
                op(tree, &mut self.soft, &mut last)?;
                if self
                    .soft
                    .committed_update_info()
                    .map_err(NativeError::from)?
                    .iteration
                    != tree.native_cut().tick()
                {
                    return Err(invalid("native and XPBD clocks diverged"));
                }
                Ok(Saved {
                    tree: tree.native_cut().clone(),
                    material: self
                        .soft
                        .world()
                        .resource::<MaterialBody>()
                        .0
                        .checkpoint_articulated_material()?,
                    last,
                })
            })();
            match next {
                Ok(s) => break s,
                Err(e) => {
                    self.rollback(true)?;
                    if retry && matches!(e, ArticulationError::Io(_)) {
                        retry = false;
                        continue;
                    }
                    return Err(e);
                }
            }
        };
        boundary("material-both-prepared");
        if let Err(e) = self.store(&next) {
            self.rollback(false)?;
            return Err(ArticulationError::Publication(e));
        }
        boundary("after-sync");
        self.saved = next;
        Ok(())
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_with_efforts_at(self.epoch(), self.tick(), &[])
    }
    pub fn advance_with_efforts_at(
        &mut self,
        epoch: u64,
        tick: u64,
        efforts: &[(u64, f64)],
    ) -> Result<()> {
        self.check_interval(epoch, tick)?;
        let next = tick
            .checked_add(1)
            .ok_or(invalid("material clock exhausted"))?;
        let event = self
            .spec
            .barriers
            .binary_search_by_key(&next, |b| b.tick)
            .ok()
            .map(|i| self.spec.barriers[i].clone());
        self.change(|tree, soft, last| {
            tree.advance_with_efforts_at(epoch, tick, efforts)?;
            boundary("material-rigid-advanced");
            soft.step_exact(1).map_err(NativeError::from)?;
            boundary("material-xpbd-advanced");
            if let Some(b) = &event {
                *last = Some(resolve(
                    tree,
                    &mut soft.world_mut().resource_mut::<MaterialBody>().0,
                    epoch,
                    b,
                )?);
            }
            Ok(())
        })
    }
    pub fn transfer(&mut self, dest: u8) -> Result<()> {
        self.change(|r, _, _| {
            r.transfer(dest)?;
            boundary("material-native-transferred");
            Ok(())
        })
    }
    pub fn reset(&mut self) -> Result<()> {
        let cold = self.cold.clone();
        self.change(|r, s, last| {
            r.reset()?;
            *s = material_world(cold.clone(), r.epoch(), 0)?;
            *last = None;
            Ok(())
        })
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.rigid
            .as_mut()
            .ok_or(invalid("material session requires recovery"))?
            .drop_next_acknowledgement(slot)
    }
}

fn resolve(
    tree: &mut ContactTreePool,
    material: &mut SoftBody,
    epoch: u64,
    event: &MaterialImpactBarrier,
) -> Result<MaterialContactReceipt> {
    #[cfg(test)]
    super::crash_tests::kill_owner_at_marker(
        "GZ_MATERIAL_GEOMETRY_KILL_ONCE",
        tree.worker_pid(tree.owner())
            .expect("resident material native worker"),
    );
    let spheres = tree
        .geometry(&event.links)?
        .into_iter()
        .map(|(center, radius)| ImpactSphere { center, radius })
        .collect::<Vec<_>>();
    let material = material.sphere_impact_at_barrier(&spheres)?;
    if material.sites().is_empty() {
        return Err(invalid(
            "authored material barrier has no incoming contact geometry",
        ));
    }
    let mut sites = Vec::new();
    let obs = tree.observation();
    for s in material.sites() {
        let link = event.links[s.sphere];
        let i = obs
            .links
            .binary_search_by_key(&link, |r| r.id)
            .map_err(|_| invalid("unknown material link"))?;
        let r = &obs.links[i];
        let pt = DVec3::from(s.point) - DVec3::from(r.position);
        sites.push(ArticulatedPointSite {
            link,
            local_point: (DQuat::from_array(r.orientation).conjugate() * pt).to_array(),
        });
    }
    let cert = tree.sample_contact_points_at(epoch, event.tick, &sites)?;
    let basis = cert.basis();
    let plan = material.prepare(
        &basis.velocities,
        |a, b| basis.mobility(a, b).expect("square verified basis"),
        event.restitution,
    )?;
    let js = plan
        .report()
        .loads
        .iter()
        .map(|l| l.on_rigid)
        .collect::<Vec<_>>();
    if js.iter().all(|j| *j == [0.; 3]) {
        return Err(invalid(
            "authored material barrier has no incoming normal velocity",
        ));
    }
    boundary("material-signed-impulses-prepared");
    #[cfg(test)]
    super::crash_tests::kill_owner_at_marker(
        "GZ_MATERIAL_CONTACT_KILL_ONCE",
        tree.worker_pid(tree.owner())
            .expect("resident material native worker"),
    );
    let native = tree.accept_points(&cert, &js, false)?;
    boundary("material-native-impulses-applied");
    let report = plan.commit();
    boundary("material-particle-impulses-applied");
    Ok(MaterialContactReceipt::from_report(
        epoch,
        event.tick,
        event.restitution,
        &sites,
        cert.basis().clone(),
        native,
        report,
    ))
}
impl MaterialContactReceipt {
    pub(super) fn from_report(
        epoch: u64,
        tick: u64,
        restitution: f64,
        sites: &[ArticulatedPointSite],
        basis: ContactPointBasis,
        native: ContactImpulseReceipt,
        report: gz_physics_xpbd::articulated_impact::MaterialImpactReport,
    ) -> Self {
        let loads = report
            .loads
            .iter()
            .zip(sites)
            .map(|(l, s)| MaterialContactLoad {
                link: s.link,
                point: l.site.point,
                local_point: s.local_point,
                normal: l.site.normal,
                feature: l.site.feature.vertices().to_vec(),
                weights: l.site.weights,
                clearance: l.site.clearance,
                on_link: l.on_rigid,
                on_particles: l.on_particles.clone(),
                initial_normal_velocity: l.initial_normal_velocity,
                final_normal_velocity: l.final_normal_velocity,
            })
            .collect();
        Self {
            epoch,
            tick,
            restitution,
            loads,
            basis,
            native,
            kinetic_energy_change: report.kinetic_energy_change,
            prescribed_support_work: report.prescribed_support_work,
            dissipated_energy: report.dissipated_energy,
        }
    }
    fn validate(&self, s: &ArticulatedMaterialSpec, body: &SoftBody) -> Result<()> {
        let b = s
            .barriers
            .iter()
            .find(|b| b.tick == self.tick)
            .ok_or(invalid("no material barrier at archived contact"))?;
        self.validate_at(s.tree.tree.links.len(), &b.links, b.restitution, body)
    }
    pub(super) fn validate_at(
        &self,
        links: usize,
        receivers: &[u64],
        restitution: f64,
        body: &SoftBody,
    ) -> Result<()> {
        self.validate_dimensions((links, links + 5), receivers, restitution, body)
    }
    pub(super) fn validate_dimensions(
        &self,
        dimensions: (usize, usize),
        receivers: &[u64],
        restitution: f64,
        body: &SoftBody,
    ) -> Result<()> {
        let count = self.loads.len();
        self.basis
            .check_dimensions(count, dimensions.0, dimensions.1)?;
        if self.restitution.to_bits() != restitution.to_bits()
            || self.native.after_velocities.len() != count
            || !self
                .native
                .after_velocities
                .iter()
                .flatten()
                .all(|v| v.is_finite())
            || ![
                self.native.kinetic_energy_change,
                self.kinetic_energy_change,
                self.prescribed_support_work,
                self.dissipated_energy,
            ]
            .iter()
            .all(|v| v.is_finite())
            || self.dissipated_energy < 0.
        {
            return Err(invalid(
                "invalid archived material impulse energy or restitution",
            ));
        }
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-10 * (1. + a.abs() + b.abs());
        if count == 0
            || self.loads.iter().all(|l| l.on_link == [0.; 3])
            || !close(
                self.kinetic_energy_change + self.dissipated_energy,
                self.prescribed_support_work,
            )
        {
            return Err(invalid(
                "archived material receipt has no impulse or energy balance",
            ));
        }
        for (site, row) in self.loads.iter().enumerate() {
            if !receivers.contains(&row.link)
                || row.feature.is_empty()
                || row.feature.len() > 3
                || !row.feature.windows(2).all(|w| w[0] < w[1])
                || row.feature.iter().any(|&i| i >= body.particles().len())
                || row.on_particles.len() != row.feature.len()
                || !row
                    .on_particles
                    .iter()
                    .zip(&row.feature)
                    .all(|((id, j), i)| id == i && j.iter().all(|v| v.is_finite()))
                || ![
                    row.point,
                    row.local_point,
                    row.normal,
                    row.on_link,
                    row.weights,
                ]
                .iter()
                .flatten()
                .all(|v| v.is_finite())
                || ![
                    row.initial_normal_velocity,
                    row.final_normal_velocity,
                    row.clearance,
                ]
                .iter()
                .all(|v| v.is_finite())
            {
                return Err(invalid("invalid archived material contact load"));
            }
            let n = row.feature.len();
            if row.weights[..n]
                .iter()
                .any(|&w| !(-1e-12..=1. + 1e-12).contains(&w))
                || row.weights[n..].iter().any(|&w| w != 0.)
                || !close(row.weights.iter().sum(), 1.)
                || !close(DVec3::from(row.normal).length_squared(), 1.)
                || !row
                    .point
                    .iter()
                    .zip(self.basis.world_points[site])
                    .all(|(&a, b)| close(a, b))
                || row
                    .on_particles
                    .iter()
                    .zip(&row.weights)
                    .any(|((_, j), w)| {
                        j.iter()
                            .zip(row.on_link)
                            .any(|(&particle, rigid)| particle != -rigid * w)
                    })
            {
                return Err(invalid(
                    "archived point, barycentric weight or equal-and-opposite impulse differs",
                ));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod crash_tests;
