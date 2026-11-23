// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
impl TreePool<ContactTreeSpec> {
    pub(in crate::floating_tree) fn contained_cold(
        exe: &Path,
        l: std::sync::Arc<Ledger>,
        width: u8,
        s: ContactTreeSpec,
    ) -> Result<Self> {
        Self::fresh(
            exe,
            crate::floating_tree::fleet_session::FleetSession::Contained(l),
            width,
            s,
        )
    }
    pub(in crate::floating_tree) fn contained_restore(
        exe: &Path,
        l: std::sync::Arc<Ledger>,
        s: ContactTreeSpec,
        c: Cut,
    ) -> Result<Self> {
        Self::open(
            exe,
            crate::floating_tree::fleet_session::FleetSession::Contained(l),
            s,
            c,
        )
    }
    pub(in crate::floating_tree) fn native_cut(&self) -> &Cut {
        &self.cut
    }
    pub(in crate::floating_tree) fn geometry(
        &mut self,
        ids: &[u64],
    ) -> Result<Vec<([f64; 3], f64)>> {
        let tick = self.cut.tick();
        let generation = self.cut.generation;
        let owner = self.cut.owner;
        let v: Answer = self.peer(owner)?.call_typed(
            tick,
            Request {
                generation,
                action: Action::SphereGeometry(ids.to_vec()),
            },
        )?;
        let Answer::Geometry(s) = v else {
            return Err(invalid("missing native contact sphere bounds"));
        };
        if s.len() != ids.len()
            || s.iter()
                .any(|(c, r)| !c.iter().all(|x| x.is_finite()) || !r.is_finite() || *r <= 0.)
        {
            return Err(invalid("invalid native contact sphere bounds"));
        }
        Ok(s)
    }
    /// Recompute all cross-link mobility blocks at this accepted physical interval.
    pub fn sample_contact_points_at(
        &mut self,
        epoch: u64,
        tick: u64,
        sites: &[ArticulatedPointSite],
    ) -> Result<ContactPointCertificate> {
        self.check_interval(epoch, tick)?;
        let topo = &self.spec.tree;
        if sites.is_empty()
            || sites.len() > crate::floating_tree::contact::MAX_SITES
            || sites.iter().any(|s| {
                !s.local_point.iter().all(|x| x.is_finite())
                    || topo.links.binary_search_by_key(&s.link, |l| l.id).is_err()
            })
        {
            return Err(invalid(
                "unknown contact point link, nonfinite offset or invalid site count",
            ));
        }
        let (generation, owner, source) = (
            self.cut.generation,
            self.cut.owner,
            self.cut.snapshot.digest,
        );
        let request = || Request {
            generation,
            action: Action::SamplePoints(sites.to_vec()),
        };
        let answer = match self.peer(owner)?.call_typed::<_, Answer>(tick, request()) {
            Ok(v) => v,
            Err(e) => {
                self.recover()?;
                if !matches!(e, ArticulationError::Io(_)) {
                    return Err(e);
                }
                let v = self.peer(owner)?.call_typed(tick, request());
                if v.is_err() {
                    self.recover()?;
                }
                v?
            }
        };
        let Answer::Basis(basis) = answer else {
            return Err(invalid("missing native mobility sample"));
        };
        basis.check(sites.len(), self.spec.tree.links.len())?;
        let seal = basis.seal(source, generation, tick, sites);
        Ok(ContactPointCertificate {
            epoch,
            tick,
            generation,
            source,
            session: self.contact_session.expect("contact session"),
            seal,
            sites: sites.to_vec(),
            basis,
        })
    }
    /// Apply all signed world point impulses in one native operation and advance one interval.
    /// A stale solver cut, changed pending effort, ownership transfer or new controller requires a fresh sample.
    pub fn advance_with_contact_impulses(
        &mut self,
        cert: &ContactPointCertificate,
        impulses: &[[f64; 3]],
    ) -> Result<ContactImpulseReceipt> {
        self.accept_points(cert, impulses, true)
    }
    pub(in crate::floating_tree) fn accept_points(
        &mut self,
        cert: &ContactPointCertificate,
        impulses: &[[f64; 3]],
        advance: bool,
    ) -> Result<ContactImpulseReceipt> {
        self.check_interval(cert.epoch, cert.tick)?;
        if Some(cert.session) != self.contact_session
            || self.cut.generation != cert.generation
            || self.cut.snapshot.digest != cert.source
        {
            return Err(invalid(
                "stale contact sample session, generation or complete native image",
            ));
        }
        if impulses.len() != cert.sites.len() || !impulses.iter().flatten().all(|v| v.is_finite()) {
            return Err(invalid("invalid signed contact impulse vector"));
        }
        let mut accepted = None;
        self.transact(|p, c| {
            let next = c
                .tick()
                .checked_add(u64::from(advance))
                .ok_or(invalid("contact tree clock exhausted"))?;
            let answer: Answer = p.peer(c.owner)?.call_typed(
                c.tick(),
                Request {
                    generation: c.generation,
                    action: Action::CommitPoints {
                        sites: cert.sites.clone(),
                        certificate: cert.seal,
                        impulses: impulses.to_vec(),
                        advance,
                    },
                },
            )?;
            let Answer::Applied(snapshot, receipt) = answer else {
                return Err(invalid("missing native impulse receipt"));
            };
            if snapshot.state.tick != next
                || !receipt.kinetic_energy_change.is_finite()
                || receipt.after_velocities.len() != impulses.len()
                || !receipt
                    .after_velocities
                    .iter()
                    .flatten()
                    .all(|x| x.is_finite())
            {
                return Err(invalid(
                    "invalid native impulse energy, velocities or clock",
                ));
            }
            c.snapshot = *snapshot;
            accepted = Some(receipt);
            boundary("contact-tree-native-prepared");
            Ok(())
        })?;
        accepted.ok_or(invalid("native contact interval has no receipt"))
    }
}
