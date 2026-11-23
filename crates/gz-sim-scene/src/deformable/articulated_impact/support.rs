// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite-step unilateral support on an articulated sphere and movable material.
mod slip;
use super::{recurrent::Interval, *};
use gz_physics_multifidelity::ArticulatedNativeInterval;
use gz_physics_xpbd::MaterialFeature;
use gz_physics_xpbd::articulated_impact::{
    MaterialImpactSite, MaterialSupportLoad, MaterialSupportResponse,
};

pub(super) const BAND: f64 = 2e-9;
const TARGET: f64 = 4e-12;
const RESIDUAL: f64 = 1e-12;
#[derive(Clone, Debug, PartialEq)]
pub struct SupportApplication {
    pub material: Vec<MaterialSupportLoad>,
    pub native: ArticulatedImpulseReport,
}
#[derive(Clone, Debug, PartialEq)]
/// A finite interval that transmitted load. Beginning and end impulses are in
/// N s on their respective physical cuts. Divide their vector sum by the
/// duration to obtain mean contact force in N.
pub struct ArticulatedSupportInterval {
    pub start: Duration,
    pub duration: Duration,
    pub links: Vec<u64>,
    pub beginning: Option<SupportApplication>,
    pub end: Option<SupportApplication>,
    /// Relative tangent travel of the same material skin and native point in
    /// world metres, using the starting finite-skin Jacobian. Indexed by
    /// starting row, in the same order as `source_features`. Opening rows
    /// can carry zero force.
    pub tangential_travel: Vec<(usize, [f64; 3])>,
    /// One row per actual starting material feature, including separate
    /// physical contacts on the same native sphere.
    pub source_features: Vec<(usize, MaterialFeature)>,
    pub region_clearances: Vec<f64>,
    pub endpoint_clearances: Vec<f64>,
    pub trajectory_trials: u32,
    /// Rejected longer trials preceding this accepted loaded interval whose
    /// measured tangent velocities indicated a possible kinetic transition.
    pub slip_transition_refinements: u32,
}
impl ArticulatedSupportInterval {
    /// Relative world-tangent travel in metres for one specific starting material
    /// feature. Frictionless intervals carry only normal information.
    pub fn tangent_travel(&self, sphere: usize, feature: MaterialFeature) -> Option<[f64; 3]> {
        let index = self
            .source_features
            .iter()
            .position(|r| *r == (sphere, feature))?;
        self.tangential_travel.get(index).map(|r| r.1)
    }
}
pub(super) enum Outcome {
    Clear,
    Accepted,
    Refine,
    RefineSlip { next: u64, horizon: u64 },
}
fn geometry(n: &ArticulatedNativeInterval<'_>) -> Result<Vec<ImpactSphere>, MaterialImpactError> {
    Ok(n.geometry()?
        .into_iter()
        .map(|(center, radius)| ImpactSphere { center, radius })
        .collect())
}
fn local_points(
    n: &ArticulatedNativeInterval<'_>,
    bodies: &[gz_physics::BodyId],
    sites: &[MaterialImpactSite],
) -> Result<Vec<ArticulatedContactSite>, MaterialImpactError> {
    sites
        .iter()
        .map(|s| {
            let body = bodies[s.sphere];
            let pose = n.state(body).ok_or(PhysicsError::MissingBody)?;
            let r = DVec3::from(s.point) - DVec3::from(pose.position);
            Ok(ArticulatedContactSite {
                body,
                local_point: (DQuat::from_array(pose.orientation).conjugate() * r).to_array(),
            })
        })
        .collect()
}
struct Basis {
    sources: Vec<usize>,
    features: Vec<(usize, MaterialFeature)>,
    response: MaterialSupportResponse,
    local: Vec<ArticulatedContactSite>,
    points: Vec<[f64; 3]>,
}
fn basis(
    n: &mut ArticulatedNativeInterval<'_>,
    m: &mut SoftBody,
    bodies: &[gz_physics::BodyId],
    skin: f64,
) -> Result<Option<Basis>, MaterialImpactError> {
    let geometry = geometry(n)?;
    let patch = m.sphere_support_witnesses(&geometry, skin)?;
    let rows = patch.sites();
    if rows.is_empty() {
        return Ok(None);
    }
    let sources: Vec<_> = rows.iter().map(|r| r.sphere).collect();
    let features = rows.iter().map(|r| (r.sphere, r.feature)).collect();
    if rows.iter().any(|r| r.clearance.abs() > skin + 1.2e-11) {
        return Err(MaterialImpactError::UnresolvedSupport);
    }
    if sources.len() > 4 {
        return Err(MaterialImpactError::UnresolvedSupport);
    }
    let local = local_points(n, bodies, rows)?;
    let tx = n.prepare(&local)?;
    let k = tx.sample();
    let response = patch.support_response(&k.velocities, |i, j| {
        k.mobility(i, j).expect("same support rows")
    })?;
    Ok(Some(Basis {
        sources,
        features,
        response,
        points: rows.iter().map(|s| s.point).collect(),
        local,
    }))
}
fn apply(
    n: &mut ArticulatedNativeInterval<'_>,
    m: &mut SoftBody,
    bodies: &[gz_physics::BodyId],
    source: &Basis,
    j: &[f64],
    skin: f64,
) -> Result<Option<SupportApplication>, MaterialImpactError> {
    if j.iter().all(|v| *v == 0.) {
        return Ok(None);
    }
    let geo = geometry(n)?;
    let patch = m.sphere_support_witnesses(&geo, skin)?;
    if patch
        .sites()
        .iter()
        .map(|s| (s.sphere, s.feature))
        .ne(source.features.iter().copied())
    {
        return Err(MaterialImpactError::UnresolvedSupport);
    }
    let local = local_points(n, bodies, patch.sites())?;
    let tx = n.prepare(&local)?;
    let k = tx.sample();
    let material = patch.apply_support(&source.response, j, &k.velocities, |i, j| {
        k.mobility(i, j).expect("same support rows")
    })?;
    let j: Vec<_> = material.iter().map(|l| l.on_rigid).collect();
    let native = tx.commit(&j)?;
    Ok(Some(SupportApplication { material, native }))
}
pub(super) fn step(
    active: &mut Interval<'_, '_>,
    ns: u64,
    slip_transition_refinements: u32,
) -> Result<Outcome, MaterialImpactError> {
    let dt = Duration::from_nanos(ns);
    let h = dt.as_secs_f64();
    let Some(initial) = basis(
        &mut active.native,
        &mut active.material,
        active.bodies,
        BAND,
    )?
    else {
        return Ok(Outcome::Clear);
    };
    // Once the first event has fired, the finite-step Signorini problem
    // admits a new normal only within the measured spatial shell. Opening
    // features receive zero force. A disjoint approaching pad is refined
    // chronologically until it reaches this shell.
    let stride = initial.response.stride();
    let mut j = vec![0.; initial.sources.len() * stride];
    for iter in 1..=8 {
        let mut native = active.native.branch()?;
        let mut material = active.material.clone();
        let begin = apply(
            &mut native,
            &mut material,
            active.bodies,
            &initial,
            &j,
            BAND,
        )?;
        let start_spheres = geometry(&native)?;
        let start_material = material.clone();
        let midpoint_minimum = if ns >= 2 * u64::from(material.config().substeps) {
            let half = Duration::from_nanos(ns / 2);
            let mut mid = material.clone();
            mid.advance_supported_free_interval(half)?;
            let p = native
                .geometry_after(half)?
                .into_iter()
                .map(|(center, radius)| ImpactSphere { center, radius })
                .collect::<Vec<_>>();
            mid.articulated_separations(&p)?.minimum()
        } else {
            f64::INFINITY
        };
        native.step(dt)?;
        material.advance_supported_free_interval(dt)?;
        let finish = geometry(&native)?;
        let separation = material.articulated_separations(&finish)?;
        let gap = separation.sphere_minima();
        let (region, outside) = separation.support_clearances(&initial.features)?;
        // A new disjoint contact must be timed at its own cut, never
        // silently projected at the end of a loaded interval.
        if gap
            .iter()
            .enumerate()
            .any(|(i, d)| *d < 0. && !initial.sources.contains(&i))
        {
            return Ok(Outcome::Refine);
        }
        let deficit: Vec<_> = region.iter().map(|g| g - TARGET).collect();
        let satisfied = deficit
            .iter()
            .zip(j.iter().step_by(stride))
            .all(|(&g, &j)| g >= -RESIDUAL && (j == 0. || g.abs() <= RESIDUAL));
        let mut tangential_travel = Vec::new();
        if !satisfied || stride == 3 {
            let moved = initial.response.induced(&j)?;
            let rigid = if stride == 3 {
                initial
                    .local
                    .iter()
                    .zip(&initial.points)
                    .map(|(site, x)| {
                        let p = native.state(site.body).ok_or(PhysicsError::MissingBody)?;
                        Ok((DVec3::from(p.position)
                            + DQuat::from_array(p.orientation) * DVec3::from(site.local_point)
                            - DVec3::from(*x))
                        .to_array())
                    })
                    .collect::<Result<Vec<_>, PhysicsError>>()?
            } else {
                Vec::new()
            };
            let travel = initial.response.row_travel(
                start_material.particles(),
                material.particles(),
                &rigid,
                &deficit,
            )?;
            if stride == 3 {
                tangential_travel = initial
                    .sources
                    .iter()
                    .copied()
                    .zip(initial.response.tangent_vectors(&travel)?)
                    .collect();
            }
            let free: Vec<_> = travel.iter().zip(moved).map(|(&g, v)| g / h - v).collect();
            let next = initial.response.solve(&free)?;
            // For Coulomb contact, check the actual nonlinear trial against
            // the cone fixed point. It admits either zero relative travel or
            // boundary traction directly opposite travel, with no attraction.
            let delta = next.iter().zip(&j).map(|(a, b)| a - b).collect::<Vec<_>>();
            let error = initial
                .response
                .induced(&delta)?
                .into_iter()
                .map(|v| v.abs() * h)
                .fold(0., f64::max);
            if !satisfied || error > RESIDUAL {
                j = next;
                continue;
            }
        }
        // Only the converged loaded trajectory can establish a separate
        // collision on another fold of this same sphere. The unloaded trial
        // may cross a coplanar neighbour before its tangent impulse is known.
        if outside < -RESIDUAL {
            return Ok(Outcome::Refine);
        }
        match start_material.deforming_chord_outside_support(
            &material,
            &start_spheres,
            &finish,
            &initial.features,
        ) {
            Ok(None) => {}
            Ok(Some(_)) | Err(gz_physics_xpbd::XpbdError::SurfaceBudget) => {
                return Ok(Outcome::Refine);
            }
            Err(e) => return Err(e.into()),
        }
        // Check a real intermediate trajectory from the impulse-updated source
        // for hidden penetration even if all final distances look valid.
        if midpoint_minimum < -RESIDUAL {
            return Ok(Outcome::Refine);
        }
        // The position solve enforces clearance on the integrated trajectory.
        // A second, nonnegative impulse enforces the end velocity constraint.
        let mut tail = None;
        if let Some(end) = basis(
            &mut native,
            &mut material,
            active.bodies,
            TARGET + 2. * RESIDUAL,
        )? {
            if end.sources.iter().any(|s| {
                end.sources.iter().filter(|r| *r == s).count()
                    > initial.sources.iter().filter(|r| *r == s).count()
            }) {
                return Ok(Outcome::Refine);
            }
            let corr = end.response.solve(end.response.speeds())?;
            tail = apply(
                &mut native,
                &mut material,
                active.bodies,
                &end,
                &corr,
                TARGET + 2. * RESIDUAL,
            )?;
        }
        if let Some(cap) = active.event.slip_transition_step
            && dt > cap
            && let Some((begin, end)) = begin.as_ref().zip(tail.as_ref())
        {
            let start = active.info.start_time + Duration::from_nanos(active.nanos);
            let previous = active
                .supports
                .last()
                .or(active.previous_public)
                .filter(|p| p.start + p.duration == start);
            if let Some(p) = slip::proposal(
                begin,
                end,
                previous,
                dt,
                cap,
                u64::from(active.material.config().substeps),
            ) {
                return Ok(Outcome::RefineSlip {
                    next: p.next,
                    horizon: p.horizon,
                });
            }
        }
        native.commit();
        let start = active.info.start_time + Duration::from_nanos(active.nanos);
        active.nanos += ns;
        active.material = material;
        if begin.is_some() || tail.is_some() {
            active.supports.push(ArticulatedSupportInterval {
                start,
                duration: dt,
                links: active.event.links.clone(),
                beginning: begin,
                end: tail,
                endpoint_clearances: gap,
                tangential_travel,
                source_features: initial.features,
                region_clearances: region,
                trajectory_trials: iter,
                slip_transition_refinements,
            });
        }
        return Ok(Outcome::Accepted);
    }

    Ok(Outcome::Refine)
}
