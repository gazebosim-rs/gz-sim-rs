// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve successive isolated impacts on a deforming material inside one public tick.
use super::*;
use gz_physics_multifidelity::ArticulatedNativeInterval;
use gz_physics_xpbd::articulated_impact::MaterialSeparations;
use std::ops::ControlFlow;

struct Prediction {
    material: SoftBody,
    spheres: Vec<ImpactSphere>,
    separation: MaterialSeparations,
}
fn forecast(
    native: &ArticulatedNativeInterval<'_>,
    base: &SoftBody,
    delta: u64,
    sustained: bool,
) -> Result<Prediction, MaterialImpactError> {
    let dt = Duration::from_nanos(delta);
    let geometry = native.geometry_after(dt)?;
    predict_material(base, dt, &geometry, sustained)
}
fn predict_material(
    base: &SoftBody,
    dt: Duration,
    geometry: &[([f64; 3], f64)],
    sustained: bool,
) -> Result<Prediction, MaterialImpactError> {
    let mut material = base.clone();
    if !dt.is_zero() {
        if sustained {
            material.advance_supported_free_interval(dt)?;
        } else {
            material.step(dt, &[])?;
        }
    }
    let spheres: Vec<_> = geometry
        .iter()
        .copied()
        .map(|(center, radius)| ImpactSphere { center, radius })
        .collect();
    let separation = material.articulated_separations(&spheres)?;
    Ok(Prediction {
        material,
        spheres,
        separation,
    })
}
pub(super) struct Interval<'a, 'b> {
    pub(super) native: ArticulatedNativeInterval<'a>,
    pub(super) material: SoftBody,
    pub(super) supports: Vec<ArticulatedSupportInterval>,
    pub(super) previous_public: Option<&'b ArticulatedSupportInterval>,
    receipts: Vec<Arc<ArticulatedMaterialImpact>>,
    pub(super) bodies: &'b [gz_physics::BodyId],
    pub(super) event: &'b ScheduledImpact,
    pub(super) info: UpdateInfo,
    pub(super) nanos: u64,
}
impl Interval<'_, '_> {
    fn resolve(&mut self, cut: &MaterialImpactTime) -> Result<(), MaterialImpactError> {
        if self.receipts.len() >= 32 {
            return Err(MaterialImpactError::UnresolvedReturn);
        }
        let context = ImpactContext {
            time: self.info.start_time + Duration::from_nanos(self.nanos),
            epoch: self.info.epoch,
            bodies: self.bodies,
            links: &self.event.links,
            restitution: self.event.restitution,
            detected: Some((cut, self.info.start_time)),
        };
        let receipt = apply_at_cut(&mut self.native, &mut self.material, &context)?
            .ok_or(MaterialImpactError::MissingIncoming)?;
        self.receipts.push(receipt);
        Ok(())
    }
    fn accept(&mut self, delta: u64, material: SoftBody) -> Result<(), MaterialImpactError> {
        if delta != 0 {
            self.native.step(Duration::from_nanos(delta))?;
        }
        self.material = material;
        self.nanos += delta;
        Ok(())
    }
    fn find_entry(
        &mut self,
        mut upper: u64,
        mut after: Prediction,
    ) -> Result<bool, MaterialImpactError> {
        let minimum = u64::from(self.material.config().substeps);
        let mut lower = 0;
        let mut before = forecast(&self.native, &self.material, 0, self.event.sustained)?;
        while upper - lower > 1 {
            let mid = lower + (upper - lower) / 2;
            if mid < minimum {
                return Err(MaterialImpactError::UnresolvedReturn);
            }
            let trial = forecast(&self.native, &self.material, mid, self.event.sustained)?;
            // The lower half can contain a short-lived entry even when its
            // endpoints are clear. Never skip it merely because the final
            // state of the full bracket penetrates a different primitive.
            let earlier = if trial.separation.minimum() <= 0. {
                true
            } else {
                match before.material.deforming_chord_candidate(
                    &trial.material,
                    &before.spheres,
                    &trial.spheres,
                ) {
                    Ok(c) => c.is_some(),
                    Err(gz_physics_xpbd::XpbdError::SurfaceBudget) => return Ok(false),
                    Err(e) => return Err(e.into()),
                }
            };
            if earlier {
                upper = mid;
                after = trial;
            } else {
                lower = mid;
                before = trial;
            }
        }
        if self.event.sustained
            && lower > 0
            && after.separation.minimum() > 0.
            && before.separation.minimum() >= 0.
            && before.separation.minimum() <= super::support::BAND
        {
            // The geometric CCD margin can precede a genuine later crossing
            // by a few femtometres. Move to the real clear cut and let the
            // support solve enforce the reported spatial acquisition shell.
            self.accept(lower, before.material)?;
            return Ok(true);
        }
        let cut = before.material.bracketed_articulated_return(
            &before.spheres,
            &after.material,
            &after.spheres,
            Duration::from_nanos(upper - lower),
            Duration::from_nanos(self.nanos + lower),
        )?;
        self.accept(lower, before.material)?;
        self.resolve(&cut)?;
        Ok(true)
    }
    fn run_after(&mut self, end: u64, width: u64) -> Result<(), MaterialImpactError> {
        let minimum = u64::from(self.material.config().substeps);
        let mut budget = 16384;
        let mut slip_horizon = 0u64;
        while self.nanos < end {
            let remaining = end - self.nanos;
            let mut h = remaining.min(width);
            if self.event.slip_transition_step.is_some() {
                // A published observation does not advance the contact lattice.
                // After a local cut, rejoin the same epoch-relative nominal grid.
                let absolute = self.info.start_time.as_nanos() + u128::from(self.nanos);
                let next = width - (absolute % u128::from(width)) as u64;
                let grid = if next < minimum {
                    next + width / 2
                } else {
                    next
                };
                h = h.min(grid);
            }
            // A rejected longer slip trial already supplied a local time bound.
            // Use its remaining span as a proposal after the first half is
            // accepted. Every shorter trajectory is integrated and checked again.
            if let Some(unresolved) = slip_horizon.checked_sub(self.nanos)
                && unresolved >= minimum
            {
                h = h.min(unresolved);
            }
            if remaining - h > 0 && remaining - h < minimum {
                h = remaining;
            }
            let start: Vec<_> = self
                .native
                .geometry()?
                .into_iter()
                .map(|(center, radius)| ImpactSphere { center, radius })
                .collect();
            if self.material.articulated_separations(&start)?.minimum() < -1e-12 {
                return Err(MaterialImpactError::UnresolvedReturn);
            }
            let mut slip_refinements = 0;
            loop {
                if h < minimum || budget == 0 {
                    return Err(MaterialImpactError::UnresolvedReturn);
                }
                budget -= 1;
                if self.event.sustained {
                    match super::support::step(self, h, slip_refinements)? {
                        super::support::Outcome::Accepted => break,
                        super::support::Outcome::RefineSlip { next, horizon } => {
                            slip_refinements += 1;
                            slip_horizon = self.nanos + horizon;
                            h = next;
                            continue;
                        }
                        super::support::Outcome::Refine => {
                            h /= 2;
                            continue;
                        }
                        super::support::Outcome::Clear => {}
                    }
                }
                let dt = Duration::from_nanos(h);
                let choice =
                    self.native
                        .inspect_step(dt, |geometry| -> Result<_, MaterialImpactError> {
                            let after = predict_material(
                                &self.material,
                                dt,
                                geometry,
                                self.event.sustained,
                            )?;
                            if after.separation.minimum() <= 0. {
                                return Ok(ControlFlow::Break(Some(after)));
                            }
                            match self.material.deforming_chord_candidate(
                                &after.material,
                                &start,
                                &after.spheres,
                            ) {
                                Ok(None) => Ok(ControlFlow::Continue(after.material)),
                                Ok(Some(_)) | Err(gz_physics_xpbd::XpbdError::SurfaceBudget) => {
                                    Ok(ControlFlow::Break(None))
                                }
                                Err(e) => Err(e.into()),
                            }
                        })?;
                match choice {
                    ControlFlow::Continue(material) => {
                        self.material = material;
                        self.nanos += h;
                        break;
                    }
                    ControlFlow::Break(Some(after)) => {
                        if self.find_entry(h, after)? {
                            break;
                        }
                        h /= 2;
                    }
                    ControlFlow::Break(None) => h /= 2,
                }
            }
        }
        Ok(())
    }
}
pub(super) fn advance(
    world: &mut World,
    info: UpdateInfo,
    event: &ScheduledImpact,
    first: Option<MaterialImpactTime>,
    consumed: bool,
    width: Duration,
) -> Result<(), MaterialImpactError> {
    let bodies = event
        .links
        .iter()
        .map(|&id| loaded_link(world, id))
        .collect::<Result<Vec<_>, _>>()?;
    let end = u64::try_from(info.dt.as_nanos()).map_err(|_| InputError::InvalidValue)?;
    let width = u64::try_from(width.as_nanos()).map_err(|_| InputError::InvalidValue)?;
    world.resource_scope(|world, mut soft: Mut<Deformables>| {
        let material = soft
            .current
            .get(&event.material)
            .ok_or(MaterialImpactError::Scene)?
            .clone();
        let mut physics = world
            .get_resource_mut::<PhysicsBackend>()
            .ok_or(InputError::InvalidPhysics)?;
        let native = physics
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or(InputError::InvalidPhysics)?
            .begin_articulated_material_interval(&bodies)?;
        let mut stage = Interval {
            native,
            material,
            receipts: vec![],
            previous_public: soft
                .samples
                .get(&event.material)
                .filter(|p| {
                    p.time == info.start_time
                        && p.epoch == info.epoch
                        && event.slip_transition_step.is_some()
                })
                .and_then(|s| s.supports.last())
                .filter(|p| p.start + p.duration == info.start_time && p.links == event.links),
            supports: vec![],
            bodies: &bodies,
            event,
            info,
            nanos: 0,
        };
        if !consumed {
            let head = first.as_ref().map_or(info.dt, MaterialImpactTime::head);
            if !head.is_zero() {
                stage.native.step(head)?;
                stage.material.advance_articulated_flight_from(
                    soft.initial[&event.material].particles(),
                    info.start_time,
                    info.start_time + head,
                )?;
            }
            stage.nanos = head.as_nanos() as u64;
            if let Some(cut) = first {
                stage.resolve(&cut)?;
            }
        }
        stage.run_after(end, width)?;
        let Interval {
            native,
            material,
            supports,
            receipts,
            ..
        } = stage;
        native.commit();
        soft.current.insert(event.material.clone(), material);
        soft.publish(info.sim_time, info.epoch);
        if event.sustained && consumed {
            soft.automatic_consumed = true;
        }
        if !supports.is_empty() {
            soft.automatic_consumed = true;
            soft.samples
                .get_mut(&event.material)
                .expect("registered material sample")
                .supports = supports;
        }
        if !receipts.is_empty() {
            soft.automatic_consumed = true;
            let sample = soft
                .samples
                .get_mut(&event.material)
                .expect("registered material sample");
            for receipt in receipts {
                sample.record_impact(receipt);
            }
        }
        Ok(())
    })
}
