// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::ledger::Ledger;
use crate::moving::worker::Peer;
use crate::paged_native::{self, Answer, Op, Req};
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{Particle, articulated_impact::ImpactSphere};
use soft::MaterialWorld;
use std::{fs::File, io::Read, path::PathBuf};
#[derive(Clone, Serialize, Deserialize)]
struct Cut {
    width: u8,
    owner: u8,
    epoch: u64,
    generation: u64,
    native: TimedCut,
    #[serde(with = "serde_bytes")]
    material: Vec<u8>,
    last: Option<AutomaticImpactReceipt>,
}
impl Cut {
    fn check(&self, s: &impl AutomaticConstruction, cold: &SoftBody) -> Result<()> {
        self.native.check(s)?;
        let tick = self.native.state.tick;
        if !(2..=8).contains(&self.width)
            || self.owner >= self.width
            || self.epoch > self.generation
            || self.native.consumed != self.last.is_some()
        {
            return Err(invalid(
                "invalid automatic material native authority or event flag",
            ));
        }
        let mut expected = if tick == 0 { 0 } else { s.interval_ns() };
        if let Some(r) = &self.last {
            let end = r
                .input_tick
                .checked_add(1)
                .ok_or(invalid("automatic material event tick overflows"))?;
            if end > tick
                || r.contact.tick != end
                || r.contact.epoch != self.epoch
                || r.head_ns.checked_add(r.tail_ns) != Some(s.interval_ns())
                || !r.continuous_seconds.is_finite()
                || r.continuous_seconds < 0.
                || r.continuous_seconds > Duration::from_nanos(s.interval_ns()).as_secs_f64()
                || !r.rounding_distance.is_finite()
                || r.rounding_distance < 0.
                || (r.continuous_seconds * 1e9 - r.head_ns as f64).abs() > 0.50001
                || r.features.is_empty()
                || !r.features.iter().all(|(i, ids)| {
                    s.receivers().get(*i).is_some_and(|id| {
                        r.contact
                            .loads
                            .iter()
                            .any(|l| l.link == *id && l.feature == *ids)
                    })
                })
            {
                return Err(invalid(
                    "invalid material time bracket or contact witnesses",
                ));
            }
            if !r
                .head_links
                .iter()
                .map(|l| l.id)
                .eq(s.link_specs().iter().map(|l| l.id))
                || r.head_links.iter().any(|l| {
                    l.position
                        .iter()
                        .chain(&l.orientation)
                        .chain(&l.velocity)
                        .chain(&l.angular_velocity)
                        .any(|x| !x.is_finite())
                        || (DQuat::from_array(l.orientation).length_squared() - 1.).abs() > 1e-8
                })
            {
                return Err(invalid("invalid native material event head"));
            }
            let absolute_ns = r
                .input_tick
                .checked_mul(s.interval_ns())
                .and_then(|t| t.checked_add(r.head_ns))
                .ok_or(invalid("material event time overflows"))?;
            let mut flight = cold.clone();
            if absolute_ns != 0 {
                flight.advance_articulated_flight_from(
                    cold.particles(),
                    Duration::ZERO,
                    Duration::from_nanos(absolute_ns),
                )?;
            }
            if !flight
                .particles()
                .iter()
                .map(|p| (p.position, p.velocity))
                .eq(r.head_particles.iter().copied())
            {
                return Err(invalid(
                    "material event head differs from the authored flight",
                ));
            }
            if end == tick && r.tail_ns > 0 {
                expected = r.tail_ns;
            }
            r.contact
                .validate_dimensions(s.dimensions(), s.receivers(), s.coefficient(), cold)?;
            r.check_response(s, cold)?;
        }
        if self.native.last_ns != expected {
            return Err(invalid(
                "last native interval differs from material collision time",
            ));
        }
        Ok(())
    }
}
impl AutomaticImpactReceipt {
    fn check_response(&self, s: &impl AutomaticConstruction, body: &SoftBody) -> Result<()> {
        let receipt = &self.contact;
        let mut nodal = vec![DVec3::ZERO; self.head_particles.len()];
        for row in &receipt.loads {
            for &(node, impulse) in &row.on_particles {
                nodal[node] += DVec3::from(impulse);
            }
        }
        let mut native_energy = 0.;
        for (k, row) in receipt.loads.iter().enumerate() {
            let i = s.link_index(row.link)?;
            let link = &self.head_links[i];
            let shape = &s.link_specs()[i];
            let center = DVec3::from(link.position);
            let q = DQuat::from_array(link.orientation);
            let point = center + q * DVec3::from(row.local_point);
            let normal = DVec3::from(row.normal);
            let velocity = DVec3::from(link.velocity)
                + DVec3::from(link.angular_velocity)
                    .cross(point - center - q * DVec3::from(shape.inertia_center));
            let native_delta =
                receipt
                    .loads
                    .iter()
                    .enumerate()
                    .fold(DVec3::ZERO, |dv, (source, other)| {
                        let block = receipt
                            .basis
                            .mobility(k, source)
                            .expect("validated square basis");
                        let inc =
                            block.map(|axis| DVec3::from(axis).dot(DVec3::from(other.on_link)));
                        dv + DVec3::from(inc)
                    });
            let geometry_tolerance = 1e-9 * (1. + center.length() + shape.radius);
            let velocity_tolerance = 1e-9 * (1. + velocity.length() + native_delta.length());
            let mut material_position = DVec3::ZERO;
            let mut material_velocity = DVec3::ZERO;
            let mut material_delta = DVec3::ZERO;
            for (&node, &weight) in row.feature.iter().zip(&row.weights) {
                material_position += DVec3::from(self.head_particles[node].0) * weight;
                material_velocity += DVec3::from(self.head_particles[node].1) * weight;
                material_delta += nodal[node] * (body.particles()[node].inverse_mass * weight);
            }
            let separation = body.config().particle_radius + row.clearance;
            if point.distance(DVec3::from(row.point)) > geometry_tolerance
                || (point - center - normal * shape.radius).length() > geometry_tolerance
                || (material_position - point - normal * separation).length() > geometry_tolerance
                || velocity.distance(DVec3::from(receipt.basis.velocities[k])) > velocity_tolerance
                || (velocity + native_delta)
                    .distance(DVec3::from(receipt.native.after_velocities[k]))
                    > velocity_tolerance
                || ((material_velocity - velocity).dot(normal) - row.initial_normal_velocity).abs()
                    > velocity_tolerance
                || ((material_velocity + material_delta - velocity - native_delta).dot(normal)
                    - row.final_normal_velocity)
                    .abs()
                    > velocity_tolerance
            {
                return Err(invalid(
                    "archived event geometry, point mobility or signed response differs",
                ));
            }
            native_energy += DVec3::from(row.on_link).dot(velocity + 0.5 * native_delta);
        }
        if (native_energy - receipt.native.kinetic_energy_change).abs()
            > 1e-9 * (1. + native_energy.abs())
        {
            return Err(invalid("archived native contact work differs"));
        }
        Ok(())
    }
}
fn boundary(_s: &str) {
    #[cfg(test)]
    crate::moving::crash_tests::gate(_s);
}
struct Owners {
    slots: Vec<Peer>,
}
impl Owners {
    fn new<S: AutomaticConstruction>(exe: &Path, l: &Ledger, s: &S, width: u8) -> Result<Self> {
        let mut slots = Vec::new();
        for i in 0..width {
            slots.push(Peer::new_profile(exe, i, &l.slot(i), s, S::FLAG, S::WIRE)?);
        }
        Ok(Self { slots })
    }
    fn call(&mut self, slot: u8, tick: u64, generation: u64, op: Op) -> Result<Answer> {
        self.slots
            .get_mut(slot as usize)
            .ok_or(invalid("missing automatic material owner"))?
            .call_typed(tick, Req { generation, op })
    }
    fn ack(&mut self, i: u8, t: u64, g: u64, op: Op) -> Result<()> {
        paged_native::acknowledge(self.call(i, t, g, op)?)
    }
    fn stage(&mut self, slot: u8, g: u64, cut: &TimedCut) -> Result<[u8; 32]> {
        paged_native::stage(
            encode(cut, NATIVE_LIMIT)?,
            NATIVE_LIMIT,
            |op| self.call(slot, cut.state.tick, g, op),
            || {},
        )
    }
    fn install(&mut self, c: &Cut) -> Result<()> {
        let hash = self.stage(c.owner, c.generation, &c.native)?;
        self.ack(
            c.owner,
            c.native.state.tick,
            c.generation,
            Op::Activate(hash),
        )
    }
    fn query(&mut self, c: &Cut, request: Subrequest) -> Result<Subreply> {
        let response = self.call(
            c.owner,
            c.native.state.tick,
            c.generation,
            Op::Private(encode(&request, NATIVE_LIMIT)?),
        )?;
        let Answer::Private(bytes) = response else {
            return Err(invalid("missing private automatic material response"));
        };
        decode(&bytes, NATIVE_LIMIT)
    }
    fn finish(&mut self, c: &Cut, proof: Option<[u8; 32]>) -> Result<TimedCut> {
        let previous = c.native.state.tick;
        let t = previous
            .checked_add(1)
            .ok_or(invalid("automatic material clock exhausted"))?;
        let a = self.call(
            c.owner,
            previous,
            c.generation,
            Op::FinishPrivate(encode(&proof, NATIVE_LIMIT)?),
        )?;
        let bytes = paged_native::receive(
            a,
            NATIVE_LIMIT,
            |op| self.call(c.owner, t, c.generation, op),
            || {},
        )?;
        let snap: TimedCut = decode(&bytes, NATIVE_LIMIT)?;
        if snap.state.tick != t {
            return Err(invalid(
                "automatic material worker did not close one public interval",
            ));
        }
        Ok(snap)
    }
    fn transfer(&mut self, c: &mut Cut, dest: u8, next: TimedCut) -> Result<()> {
        let next_generation = c
            .generation
            .checked_add(1)
            .ok_or(invalid("automatic material generation exhausted"))?;
        let hash = self.stage(dest, next_generation, &next)?;
        boundary("automatic-destination-staged");
        let old = Sha256::digest(encode(&c.native, NATIVE_LIMIT)?).into();
        self.ack(c.owner, c.native.state.tick, c.generation, Op::Fence(old))?;
        boundary("automatic-source-fenced");
        self.ack(dest, next.state.tick, next_generation, Op::Activate(hash))?;
        boundary("automatic-destination-active");
        c.native = next;
        c.owner = dest;
        c.generation = next_generation;
        Ok(())
    }
}
/// First swept, frictionless material contact against a translating passive articulated tree.
pub type AutomaticMaterialPool = AutomaticPool<AutomaticMaterialSpec>;
pub type RotorMaterialPool = AutomaticPool<AutomaticRotorMaterialSpec>;
/// Durable owner of a two-way material interval in one certified native construction.
pub struct AutomaticPool<S: AutomaticConstruction> {
    ledger: Ledger,
    exe: PathBuf,
    spec: S,
    cold: SoftBody,
    cold_digest: [u8; 32],
    cut: Cut,
    workers: Option<Owners>,
    soft: MaterialWorld,
    temporary_directory: Option<tempfile::TempDir>,
}
impl<S: AutomaticConstruction> Drop for AutomaticPool<S> {
    fn drop(&mut self) {
        self.workers.take();
    }
}
impl<S: AutomaticConstruction> AutomaticPool<S> {
    fn ledger(dir: &Path, create: bool) -> Result<Ledger> {
        let l = Ledger::lock(dir, create)?;
        if l.other_archive_exists(S::FILE)? {
            return Err(invalid("session belongs to another physics profile"));
        }
        Ok(l)
    }
    /// An application-owned temporary session, automatically removed when the managed world drops.
    pub fn transient(exe: &Path, width: u8, s: S, cold: SoftBody) -> Result<Self> {
        let dir = tempfile::Builder::new()
            .prefix("gz-automatic-material-")
            .tempdir()?;
        let mut p = Self::create(exe, dir.path(), width, s, cold)?;
        p.temporary_directory = Some(dir);
        Ok(p)
    }
    pub fn create(exe: &Path, dir: &Path, width: u8, s: S, cold: SoftBody) -> Result<Self> {
        construction::validate(&s, &cold)?;
        if !(2..=8).contains(&width) {
            return Err(invalid(
                "automatic material requires two to eight worker slots",
            ));
        }
        let ledger = Self::ledger(dir, true)?;
        if ledger.archive_path(S::FILE).try_exists()? {
            return Err(invalid("automatic material archive already exists"));
        }
        let m = cold.checkpoint_articulated_material()?;
        let digest = Sha256::digest(&m).into();
        let cut = Cut {
            width,
            owner: 0,
            epoch: 0,
            generation: 0,
            native: native::TimedWorld::new(&s)?.cut()?,
            material: m,
            last: None,
        };
        let p = Self::open(exe, ledger, s, cold, digest, cut)?;
        p.publish(&p.cut)?;
        Ok(p)
    }
    pub fn resume(exe: &Path, dir: &Path, cold: SoftBody) -> Result<Self> {
        let l = Self::ledger(dir, false)?;
        let mut b = Vec::new();
        File::open(l.archive_path(S::FILE))?
            .take((FILE_LIMIT + 41) as u64)
            .read_to_end(&mut b)?;
        if !(40..=FILE_LIMIT + 40).contains(&b.len())
            || b[..8] != S::MAGIC
            || b[8..40] != Sha256::digest(&b[40..])[..]
        {
            return Err(invalid(
                "invalid automatic material archive size, version or digest",
            ));
        }
        let (s, hash, c): (_, [u8; 32], Cut) = decode(&b[40..], FILE_LIMIT)?;
        Self::open(exe, l, s, cold, hash, c)
    }
    fn open(
        exe: &Path,
        ledger: Ledger,
        spec: S,
        cold: SoftBody,
        cold_digest: [u8; 32],
        cut: Cut,
    ) -> Result<Self> {
        construction::validate(&spec, &cold)?;
        cut.check(&spec, &cold)?;
        if Sha256::digest(cold.checkpoint_articulated_material()?)[..] != cold_digest {
            return Err(invalid("automatic material initial flight source differs"));
        }
        let soft = MaterialWorld::at(&cold, Some(&cut.material))?;
        let exe = exe.canonicalize()?;
        let mut workers = Owners::new(&exe, &ledger, &spec, cut.width)?;
        workers.install(&cut)?;
        Ok(Self {
            ledger,
            exe,
            spec,
            cold,
            cold_digest,
            cut,
            workers: Some(workers),
            soft,
            temporary_directory: None,
        })
    }
    fn publish(&self, c: &Cut) -> io::Result<()> {
        let bytes =
            encode(&(&self.spec, self.cold_digest, c), FILE_LIMIT).map_err(io::Error::other)?;
        self.ledger.publish_bytes(
            &self.ledger.archive_path(S::FILE),
            &S::MAGIC,
            &bytes,
            boundary,
        )
    }
    fn recover(&mut self) -> Result<()> {
        self.workers.take();
        self.soft = MaterialWorld::at(&self.cold, Some(&self.cut.material))?;
        let mut w = Owners::new(&self.exe, &self.ledger, &self.spec, self.cut.width)?;
        w.install(&self.cut)?;
        self.workers = Some(w);
        Ok(())
    }
    fn transact(
        &mut self,
        mut trial: impl FnMut(&S, &SoftBody, &mut Owners, &mut MaterialWorld, &mut Cut) -> Result<()>,
    ) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        let mut retry = true;
        loop {
            let mut next = self.cut.clone();
            let proposed = (|| {
                trial(
                    &self.spec,
                    &self.cold,
                    self.workers.as_mut().expect("checked worker"),
                    &mut self.soft,
                    &mut next,
                )?;
                next.material = self.soft.body().checkpoint_articulated_material()?;
                next.check(&self.spec, &self.cold)
            })();
            if let Err(e) = proposed {
                self.recover()?;
                if retry && matches!(e, ArticulationError::Io(_)) {
                    retry = false;
                    continue;
                }
                return Err(e);
            }
            boundary("automatic-complete-proposal");
            if let Err(e) = self.publish(&next) {
                self.workers.take();
                return Err(ArticulationError::Publication(e));
            }
            boundary("after-sync");
            self.cut = next;
            return Ok(());
        }
    }
    pub fn advance_at(&mut self, epoch: u64, tick: u64) -> Result<()> {
        self.check_interval(epoch, tick)?;
        self.transact(|s, _, w, soft, c| solve_interval(s, w, soft, c))
    }
    pub fn advance(&mut self) -> Result<()> {
        self.advance_at(self.epoch(), self.tick())
    }
    pub fn transfer(&mut self, slot: u8) -> Result<()> {
        if slot >= self.width() || slot == self.owner() {
            return Err(invalid("invalid automatic material destination"));
        }
        self.transact(|_, _, w, _, c| w.transfer(c, slot, c.native.clone()))
    }
    pub fn reset(&mut self) -> Result<()> {
        self.transact(|s, cold, w, soft, c| {
            let fresh = native::TimedWorld::new(s)?.cut()?;
            w.transfer(c, 0, fresh)?;
            c.last = None;
            *soft = MaterialWorld::at(cold, None)?;
            c.epoch = c
                .epoch
                .checked_add(1)
                .ok_or(invalid("automatic material epoch exhausted"))?;
            Ok(())
        })
    }
    pub fn check_interval(&self, e: u64, t: u64) -> Result<()> {
        crate::articulation::check_interval(self.workers.is_some(), e, t, self.epoch(), self.tick())
    }
    pub fn tick(&self) -> u64 {
        self.cut.native.state.tick
    }
    pub fn epoch(&self) -> u64 {
        self.cut.epoch
    }
    pub fn width(&self) -> u8 {
        self.cut.width
    }
    pub fn owner(&self) -> u8 {
        self.cut.owner
    }
    pub fn generation(&self) -> u64 {
        self.cut.generation
    }
    pub fn spec(&self) -> &S {
        &self.spec
    }
    pub fn observation(&self) -> &FloatingTreeObservation {
        &self.cut.native.state
    }
    pub fn particles(&self) -> &[Particle] {
        self.soft.body().particles()
    }
    pub fn last_impact(&self) -> Option<&AutomaticImpactReceipt> {
        self.cut.last.as_ref()
    }
    pub fn native_image(&self) -> Option<&[u8]> {
        self.cut.native.image.as_deref()
    }
    pub fn material_image(&self) -> &[u8] {
        &self.cut.material
    }
    pub fn last_native_duration(&self) -> Duration {
        Duration::from_nanos(self.cut.native.last_ns)
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        self.workers
            .as_ref()?
            .slots
            .get(slot as usize)
            .map(Peer::pid)
    }
    pub fn drop_next_acknowledgement(&mut self, slot: u8) -> Result<()> {
        self.check_interval(self.epoch(), self.tick())?;
        self.workers
            .as_mut()
            .unwrap()
            .slots
            .get_mut(slot as usize)
            .ok_or(invalid("invalid timed worker"))?
            .withhold = true;
        Ok(())
    }
}
fn solve_interval(
    s: &impl AutomaticConstruction,
    w: &mut Owners,
    soft: &mut MaterialWorld,
    c: &mut Cut,
) -> Result<()> {
    let tick = c.native.state.tick;
    let dt = Duration::from_nanos(s.interval_ns());
    let start = Duration::from_nanos(
        tick.checked_mul(s.interval_ns())
            .ok_or(invalid("automatic material time exhausted"))?,
    );
    let event = if c.last.is_none() {
        let shapes = w
            .query(c, Subrequest::Motion)?
            .into_shapes(s.rotor().is_some())?;
        soft.body().first_swept_sphere_impact(&shapes, dt)?
    } else {
        None
    };
    let Some(event) = event else {
        c.native = w.finish(c, None)?;
        soft.advance(dt, c.last.is_none().then_some(start))?;
        return Ok(());
    };
    let head = event.head();
    let nanos = u64::try_from(head.as_nanos())
        .map_err(|_| invalid("material event outside integer clock"))?;
    let Subreply::Head {
        geometry,
        state,
        stamp,
    } = w.query(c, Subrequest::Head { nanos })?
    else {
        return Err(invalid("missing private native material head"));
    };
    boundary("automatic-native-head");
    #[cfg(test)]
    crate::floating_tree::crash_tests::kill_owner_at_marker(
        "GZ_AUTOMATIC_HEAD_KILL_ONCE",
        w.slots[c.owner as usize].pid(),
    );
    soft.advance(head, Some(start))?;
    boundary("automatic-material-head");
    let head_particles = soft
        .body()
        .particles()
        .iter()
        .map(|p| (p.position, p.velocity))
        .collect();
    let spheres = geometry
        .into_iter()
        .map(|(center, radius)| ImpactSphere { center, radius })
        .collect::<Vec<_>>();
    let impact = soft
        .body_mut()
        .sphere_impact_at_detected_barrier(&spheres, &event)?;
    let sites = impact
        .sites()
        .iter()
        .map(|x| {
            let id = *s
                .receivers()
                .get(x.sphere)
                .ok_or(invalid("unknown automatic impact receiver"))?;
            let i = state
                .links
                .binary_search_by_key(&id, |l| l.id)
                .map_err(|_| invalid("missing automatic tree link"))?;
            let link = &state.links[i];
            Ok(ArticulatedPointSite {
                link: id,
                local_point: (DQuat::from_array(link.orientation).conjugate()
                    * (DVec3::from(x.point) - DVec3::from(link.position)))
                .to_array(),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let Subreply::Sample { basis, seal } = w.query(
        c,
        Subrequest::Sample {
            stamp,
            sites: sites.clone(),
        },
    )?
    else {
        return Err(invalid("missing private native point-mobility proof"));
    };
    basis.check_dimensions(sites.len(), s.dimensions().0, s.dimensions().1)?;
    if basis.seal(stamp, c.generation, tick, &sites) != seal {
        return Err(invalid("automatic native material basis signature differs"));
    }
    let plan = impact.prepare(
        &basis.velocities,
        |a, b| basis.mobility(a, b).expect("checked square basis"),
        s.coefficient(),
    )?;
    let impulses = plan.report().loads.iter().map(|r| r.on_rigid).collect();
    boundary("automatic-impulses-prepared");
    let Subreply::Applied(native) = w.query(
        c,
        Subrequest::Apply {
            stamp,
            sites: sites.clone(),
            seal,
            impulses,
        },
    )?
    else {
        return Err(invalid("missing native automatic contact response"));
    };
    boundary("automatic-native-impulses");
    #[cfg(test)]
    crate::floating_tree::crash_tests::kill_owner_at_marker(
        "GZ_AUTOMATIC_IMPULSE_KILL_ONCE",
        w.slots[c.owner as usize].pid(),
    );
    let report = plan.commit();
    boundary("automatic-material-impulses");
    let contact = MaterialContactReceipt::from_report(
        c.epoch,
        tick + 1,
        s.coefficient(),
        &sites,
        basis,
        native,
        report,
    );
    let tail = dt
        .checked_sub(head)
        .ok_or(invalid("automatic impact outside interval"))?;
    c.native = w.finish(c, Some(stamp))?;
    boundary("automatic-native-tail");
    soft.advance(tail, None)?;
    boundary("automatic-material-tail");
    c.last = Some(AutomaticImpactReceipt {
        input_tick: tick,
        head_ns: nanos,
        tail_ns: s.interval_ns() - nanos,
        continuous_seconds: event.seconds(),
        rounding_distance: event.rounding_distance(),
        features: event
            .features()
            .iter()
            .map(|(i, f)| (*i, f.vertices().to_vec()))
            .collect(),
        head_links: state.links,
        head_particles,
        contact,
    });
    Ok(())
}

#[cfg(test)]
mod admission_tests {
    use super::*;
    use std::{error::Error, fs};
    #[test]
    #[ignore = "requires both ordinary-validated warm material witnesses"]
    fn rehashed_but_physically_inconsistent_event_heads_cannot_resume()
    -> std::result::Result<(), Box<dyn Error>> {
        let root = std::env::var("GZ_AUTOMATIC_WITNESS")?;
        let worker = std::env::var("GZ_FLOATING_TREE_TEST_WORKER")?;
        let exe = Path::new(&worker);
        for label in ["cable", "cloth"] {
            let original = fs::read(Path::new(&root).join(label).join("accepted.gzautmaterial"))?;
            let cold = super::super::crash_tests::cold(label)?;
            let dir = tempfile::tempdir()?;
            let file = dir.path().join(FILE);
            for change in 0..10 {
                let (s, hash, mut cut): (AutomaticMaterialSpec, [u8; 32], Cut) =
                    decode(&original[40..], FILE_LIMIT)?;
                let event = cut.last.as_mut().unwrap();
                match change {
                    0 => event.head_particles[0].0[1] += 0.03,
                    1 => event.head_particles[0].1[0] += 0.01,
                    2 => event.head_links[1].position[0] += 0.002,
                    3 => event.contact.native.after_velocities[0][2] += 0.03,
                    4 => event.contact.native.kinetic_energy_change += 0.04,
                    5 => event.contact.loads[0].final_normal_velocity += 0.03,
                    6 => event.contact.loads[0].clearance += 0.002,
                    7 => event.features[0].1 = vec![usize::MAX],
                    8 => {
                        event.head_ns += 1;
                        event.tail_ns -= 1;
                    }
                    _ => cut.native.last_ns += 1,
                }
                let data = encode(&(s, hash, cut), FILE_LIMIT)?;
                let b = [MAGIC.as_slice(), Sha256::digest(&data).as_slice(), &data].concat();
                fs::write(&file, &b)?;
                assert!(
                    AutomaticMaterialPool::resume(exe, dir.path(), cold.clone()).is_err(),
                    "accepted altered event {label}/{change}"
                );
                assert_eq!(fs::read(&file)?, b);
            }
            fs::write(&file, &original)?;
            assert!(
                AutomaticMaterialPool::resume(exe, dir.path(), cold)?
                    .last_impact()
                    .is_some()
            );
        }
        Ok(())
    }
}
