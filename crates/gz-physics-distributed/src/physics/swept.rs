// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Conservative one-tick separation for centered primitives, external fields and shared terrain.
use super::*;
use gz_physics_multifidelity::{partition_linear_speed_limit, partition_prediction_distance};

mod candidates;
use candidates::Candidates;

struct Motion<'a> {
    body: &'a Body,
    radius: f64,
    free_axes: [f64; 3],
    hull: FreeHull,
    displacement: [f64; 3],
    acceleration_travel: [f64; 3],
    deflects: bool,
    /// The only potential deflection so far is a frictionless horizontal plane.
    plane_only: bool,
    max_travel: f64,
    component: usize,
}

// Constant-attitude capsules and cylinders can translate in neighboring lanes even when
// their circumspheres intersect. A prescribed twist also bounds rotation
// throughout the tick. Forced-rotation and deflected dynamics retain the
// whole circumsphere, independent of their orientation at the barrier.
enum FreeHull {
    Ball(f64),
    Axial {
        primitive: Primitive,
        outer_radius: f64,
        axis: [f64; 3],
        half_length: f64,
        radius: f64,
        axis_travel: f64,
    },
}
impl FreeHull {
    fn new(b: &Body, radius: f64, dt: f64, torque: bool) -> Self {
        if !matches!(b.primitive, Primitive::Capsule | Primitive::Cylinder)
            || (b.tier == Tier::Dynamic && (b.angular != [0.; 3] || torque))
        {
            return Self::Ball(radius);
        }
        let [x, y, z, w] = b.orientation;
        let axis = [
            2. * (x * z + w * y),
            2. * (y * z - w * x),
            1. - 2. * (x * x + y * y),
        ];
        let travel = b.angular[0].hypot(b.angular[1]).hypot(b.angular[2]) * dt;
        // Unit quaternions are admitted within 1e-12 of unit squared norm.
        // Include that input tolerance and roundoff before projecting a cap.
        Self::Axial {
            primitive: b.primitive,
            outer_radius: radius,
            axis,
            half_length: b.size[2] * 0.5,
            radius: b.size[0] * 0.5,
            axis_travel: travel + 8e-12,
        }
    }
    fn radius(&self, n: [f64; 3]) -> f64 {
        match self {
            Self::Ball(r) => *r,
            Self::Axial {
                primitive,
                outer_radius,
                axis,
                half_length,
                radius,
                axis_travel,
            } => {
                let c = axis.iter().zip(n).map(|(a, b)| a * b).sum::<f64>().abs();
                if *primitive == Primitive::Cylinder {
                    // A cylinder's projection is the sum of the centerline and
                    // radial disk projections. Any point on the solid moves by
                    // at most R*angle under a rotation of that angle; clip to the
                    // all-orientations bound when the prescribed twist is large.
                    let cross = std::array::from_fn::<_, 3, _>(|i| {
                        axis[(i + 1) % 3] * n[(i + 2) % 3] - axis[(i + 2) % 3] * n[(i + 1) % 3]
                    });
                    let radial = cross[0].hypot(cross[1]).hypot(cross[2]);
                    return (half_length * c + radius * radial + outer_radius * axis_travel)
                        .min(*outer_radius);
                }
                radius + half_length * (c + axis_travel).min(1. + 8e-12)
            }
        }
    }
}

fn motions<'a>(
    snapshot: &'a Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
) -> Option<(Vec<Motion<'a>>, Candidates)> {
    motions_with::<false>(snapshot, components, pending, forced, Candidates::new)
}
fn motions_with<'a, const PLANAR: bool>(
    snapshot: &'a Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
    discover: impl FnOnce(&[Motion<'_>]) -> Candidates,
) -> Option<(Vec<Motion<'a>>, Candidates)> {
    let mut owners = BTreeMap::new();
    for (i, component) in components.iter().enumerate() {
        if component.ids.is_empty() || component.ids.windows(2).any(|pair| pair[0] >= pair[1]) {
            return None;
        }
        for &id in &component.ids {
            if owners.insert(id, i).is_some() {
                return None;
            }
        }
    }
    if owners.len() != snapshot.bodies.len() {
        return None;
    }
    let dt = snapshot.step.duration().as_secs_f64();
    // The coordinator can present one sorted run per shard. Index by body ID
    // once so a gravity field affecting every body does not cause N² lookups.
    let mut indexed = std::borrow::Cow::Borrowed(forced);
    if !forced.is_sorted_by_key(|m| m.id) {
        indexed.to_mut().sort_by_key(|m| m.id);
    }
    let mut entries = Vec::with_capacity(snapshot.bodies.len());
    for b in &snapshot.bodies {
        let component = owners.remove(&b.id)?;
        let mut velocity = b.linear;
        for command in pending.iter().filter(|c| c.id == b.id) {
            for (v, delta) in velocity.iter_mut().zip(command.impulse) {
                *v += delta * b.mass.recip();
            }
        }
        let speed = velocity[0].hypot(velocity[1]).hypot(velocity[2]);
        let at = indexed.partition_point(|m| m.id < b.id);
        let force = indexed.get(at).filter(|m| m.id == b.id);
        let change = force.map_or([0.; 3], |m| m.delta_velocity);
        let end = std::array::from_fn::<_, 3, _>(|i| velocity[i] + change[i]);
        let end_speed = end[0].hypot(end[1]).hypot(end[2]);
        let near_clamp = change != [0.; 3]
            && speed.max(end_speed) >= partition_linear_speed_limit() * (1. - 32. * f64::EPSILON);
        let radius = match b.primitive {
            Primitive::Box => 0.5 * b.size[0].hypot(b.size[1]).hypot(b.size[2]) * (1. + 8e-12),
            Primitive::Sphere => 0.5 * b.size[0],
            Primitive::Cylinder => 0.5 * b.size[0].hypot(b.size[2]) * (1. + 8e-12),
            Primitive::Capsule => 0.5 * b.size[0] + (0.5 * b.size[2]) * (1. + 8e-12),
        };
        let hull = FreeHull::new(b, radius, dt, force.is_some_and(|f| f.has_torque));
        let free_axes = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].map(|n| hull.radius(n));
        let can_hit_ground = snapshot.ground.is_some_and(|ground| {
            if ground.horizontal() {
                let travel = (velocity[2].min(0.) + change[2].min(0.)) * dt;
                let slack = 64.
                    * f64::EPSILON
                    * (1.
                        + b.position[2].abs()
                        + ground.height().abs()
                        + travel.abs()
                        + free_axes[2]);
                let lower = b.position[2] - ground.height() + travel - free_axes[2] - slack;
                return !lower.is_finite() || lower <= partition_prediction_distance();
            }
            let n = ground.normal();
            let radius = hull.radius(n);
            let p = ground.point();
            let project = |v: [f64; 3]| v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
            let signed = project(std::array::from_fn(|i| b.position[i] - p[i]));
            let travel = dt * (project(velocity).min(0.) + project(change).min(0.));
            let roundoff = 64.
                * f64::EPSILON
                * (1.
                    + radius
                    + (0..3)
                        .map(|i| {
                            n[i].abs()
                                * (b.position[i].abs()
                                    + p[i].abs()
                                    + dt * (velocity[i].abs() + change[i].abs()))
                        })
                        .sum::<f64>());
            let lower = signed + travel - radius - roundoff;
            !lower.is_finite() || lower <= partition_prediction_distance()
        });
        let plane_only = PLANAR
            && can_hit_ground
            && b.tier == Tier::Dynamic
            && b.primitive == Primitive::Sphere
            && b.friction == 0.
            && components[component].ids.len() == 1
            && snapshot
                .ground
                .is_some_and(|p| p.horizontal() && p.friction() == 0.)
            && !near_clamp
            && speed.max(end_speed) < partition_linear_speed_limit();
        entries.push(Motion {
            body: b,
            plane_only,
            component,
            radius,
            free_axes,
            hull,
            displacement: velocity.map(|v| v * dt),
            acceleration_travel: change.map(|dv| dv * dt),
            max_travel: partition_linear_speed_limit() * dt,
            deflects: b.tier == Tier::Dynamic
                && (can_hit_ground
                    || near_clamp
                    // A ground-clear singleton cannot receive a cached finite-pair
                    // impulse. Prospective finite contacts enter the closure below.
                    || (components[component].contact_history && components[component].ids.len() > 1)
                    || speed > partition_linear_speed_limit()),
        });
    }
    let candidates = discover(&entries);
    // A prospective first contact can change either dynamic participant's direction.
    // Extend the reachable set to a fixed point, including possible secondary hits.
    loop {
        let mut changed = false;
        candidates.visit(|a, b| {
            let subject = |m: &Motion<'_>| {
                (!m.deflects || (PLANAR && m.plane_only)) && m.body.tier == Tier::Dynamic
            };
            if (subject(&entries[a]) || subject(&entries[b]))
                && overlaps_bound::<PLANAR>(&entries[a], &entries[b])
            {
                for i in [a, b] {
                    if subject(&entries[i]) {
                        entries[i].deflects = true;
                        entries[i].plane_only = false;
                        changed = true;
                    }
                }
            }
        });
        if !changed {
            return Some((entries, candidates));
        }
    }
}

fn overlaps(a: &Motion<'_>, b: &Motion<'_>) -> bool {
    overlaps_bound::<false>(a, b)
}

// Until its first possible finite-body contact, a centered sphere on a smooth
// horizontal halfspace receives only a vertical contact impulse. The native
// isotropic speed clamp cannot increase its horizontal speed. Rotation does not
// change the sphere's support. Acceleration and queued COM impulses are included
// in the field and command-adjusted displacements. Any possible finite contact
// promotes the sphere to the ordinary all-directions bound in the closure.
fn horizontal_reach(m: &Motion<'_>) -> f64 {
    if m.deflects && !m.plane_only {
        m.radius + m.max_travel
    } else {
        m.radius
            + m.displacement[0].hypot(m.displacement[1])
            + m.acceleration_travel[0].hypot(m.acceleration_travel[1])
    }
}
fn overlaps_bound<const PLANAR: bool>(a: &Motion<'_>, b: &Motion<'_>) -> bool {
    if PLANAR && (a.plane_only || b.plane_only) {
        let separation = (a.body.position[0] - b.body.position[0])
            .hypot(a.body.position[1] - b.body.position[1]);
        let reach = horizontal_reach(a) + horizontal_reach(b) + partition_prediction_distance();
        let scale = a.body.position[0].abs()
            + a.body.position[1].abs()
            + b.body.position[0].abs()
            + b.body.position[1].abs();
        let slack =
            128. * f64::EPSILON * (1. + scale + reach) + 8e-12 * (1. + a.max_travel + b.max_travel);
        if separation.is_finite() && reach.is_finite() && separation > reach + slack {
            return false;
        }
    }
    let boxes_overlap = (0..3).all(|axis| {
        let extent = |m: &Motion<'_>| {
            if m.deflects {
                m.radius + m.max_travel
            } else {
                m.free_axes[axis]
            }
        };
        let padding = extent(a) + extent(b) + partition_prediction_distance();
        // A deflectable center may travel in *any* direction during the step.
        // Freely moving centers follow a segment at the command-adjusted twist.
        let da = if a.deflects { 0. } else { a.displacement[axis] };
        let db = if b.deflects { 0. } else { b.displacement[axis] };
        // For a centered free body, each velocity kick from constant force
        // has the same sign on a given axis. Its integrated deviation from
        // the ballistic segment is between zero and (F/m)*dt^2. A contact
        // or possible clamp switches to the direction-independent bound.
        let ea = if a.deflects {
            0.
        } else {
            a.acceleration_travel[axis]
        };
        let eb = if b.deflects {
            0.
        } else {
            b.acceleration_travel[axis]
        };
        let start = a.body.position[axis] - b.body.position[axis];
        let end = start + da - db;
        // Include roundoff from native substep COM updates and the relative subtraction.
        let slack = 32.
            * f64::EPSILON
            * (1.
                + a.body.position[axis].abs()
                + b.body.position[axis].abs()
                + da.abs()
                + db.abs()
                + ea.abs()
                + eb.abs()
                + padding);
        let low = start.min(end) + ea.min(0.) - eb.max(0.);
        let high = start.max(end) + ea.max(0.) - eb.min(0.);
        !(low > padding + slack || high < -padding - slack)
    });
    if !boxes_overlap {
        return false;
    }
    if a.deflects || b.deflects {
        // A deflectable COM is inside a ball, including after secondary impacts.
        // Find a promising separating direction from the closest point on the
        // relative ballistic segment. Only the full projection below decides a
        // miss, including acceleration, prediction skin and rounding.
        let delta: [f64; 3] = std::array::from_fn(|i| a.body.position[i] - b.body.position[i]);
        let travel: [f64; 3] = std::array::from_fn(|i| {
            (if a.deflects { 0. } else { a.displacement[i] })
                - (if b.deflects { 0. } else { b.displacement[i] })
        });
        let length_sq = dot(travel, travel);
        let t = if length_sq > 0. {
            (-dot(delta, travel) / length_sq).clamp(0., 1.)
        } else {
            0.
        };
        let closest: [f64; 3] = std::array::from_fn(|i| delta[i] + t * travel[i]);
        let length_sq = dot(closest, closest);
        return !(length_sq.is_normal()
            && separates_on_axis(a, b, closest.map(|v| v / length_sq.sqrt())));
    }
    // Thin tilted disks can overlap on every world axis while their flat faces
    // remain separated. A single fixed world projection suffices to prove a
    // miss; the support bound already encloses all permitted rotation.
    for hull in [&a.hull, &b.hull] {
        if let FreeHull::Axial {
            primitive: Primitive::Cylinder,
            axis,
            ..
        } = hull
        {
            let length = axis[0].hypot(axis[1]).hypot(axis[2]);
            let n = axis.map(|v| v / length);
            if separates_on_axis(a, b, n) {
                return false;
            }
        }
    }
    true
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn separates_on_axis(a: &Motion<'_>, b: &Motion<'_>, n: [f64; 3]) -> bool {
    let project = |m: &Motion<'_>| {
        if m.deflects {
            (0., 0., m.radius + m.max_travel)
        } else {
            (
                dot(m.displacement, n),
                dot(m.acceleration_travel, n),
                m.hull.radius(n),
            )
        }
    };
    let start = dot(
        std::array::from_fn(|i| a.body.position[i] - b.body.position[i]),
        n,
    );
    let (da, ea, ra) = project(a);
    let (db, eb, rb) = project(b);
    let end = start + da - db;
    let padding = ra + rb + partition_prediction_distance();
    let scale = (0..3)
        .map(|i| {
            n[i].abs()
                * (a.body.position[i].abs()
                    + b.body.position[i].abs()
                    + a.displacement[i].abs()
                    + b.displacement[i].abs()
                    + a.acceleration_travel[i].abs()
                    + b.acceleration_travel[i].abs())
        })
        .sum::<f64>();
    let slack = 64. * f64::EPSILON * (1. + padding + scale);
    let low = start.min(end) + ea.min(0.) - eb.max(0.);
    let high = start.max(end) + ea.max(0.) - eb.min(0.);
    low.is_finite() && high.is_finite() && (low > padding + slack || high < -padding - slack)
}

/// True if a possible cross-worker interaction requires common native ownership.
pub fn partitions_may_interact(
    left: &Snapshot,
    left_components: &[ComponentInfo],
    right: &Snapshot,
    right_components: &[ComponentInfo],
    pending: &[PendingImpulse],
) -> bool {
    if left.ground != right.ground || left.step != right.step {
        return true;
    }
    let Some((mut a, _)) = motions(left, left_components, pending, &[]) else {
        return true;
    };
    let Some((b, _)) = motions(right, right_components, pending, &[]) else {
        return true;
    };
    if a.is_empty() || b.is_empty() {
        return false;
    }
    if a.len().saturating_mul(b.len()) <= 1024 {
        return a.iter().any(|a| b.iter().any(|b| overlaps(a, b)));
    }
    let boundary = a.len();
    a.extend(b);
    Candidates::new(&a).any(|i, j| i < boundary && j >= boundary && overlaps(&a[i], &a[j]))
}

/// Geometry-only prospective native unions. Members are indices of the supplied
/// components in increasing order; each union is ordered by its least member.
pub fn potential_contact_groups(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
) -> Result<Vec<Vec<usize>>, WorldError> {
    forced_contact_groups(snapshot, components, pending, &[])
}

/// Prospective solver unions before admission, including a uniform gravity field
/// and held COM wrenches. Indices refer to the supplied component list. This
/// geometry query can describe a union larger than the native checkpoint limit.
pub fn potential_contact_groups_in_field(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    wrenches: &[BodyWrench],
) -> Result<Vec<Vec<usize>>, WorldError> {
    let forced = field_motion(snapshot.step, &snapshot.bodies, gravity, wrenches)?;
    forced_contact_groups(snapshot, components, pending, &forced)
}

/// Inspect a narrower candidate bound for isolated spheres on a smooth horizontal plane.
/// Once a finite-body encounter is possible, the ordinary all-direction bound applies.
#[cfg(feature = "smooth-plane-diagnostics")]
pub fn potential_contact_groups_on_smooth_plane(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    gravity: [f64; 3],
    wrenches: &[BodyWrench],
) -> Result<Vec<Vec<usize>>, WorldError> {
    let forced = field_motion(snapshot.step, &snapshot.bodies, gravity, wrenches)?;
    contact_groups_with::<true>(snapshot, components, pending, &forced)
}

pub(crate) fn forced_contact_groups(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
) -> Result<Vec<Vec<usize>>, WorldError> {
    contact_groups_with::<false>(snapshot, components, pending, forced)
}

fn contact_groups_with<const PLANAR: bool>(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
) -> Result<Vec<Vec<usize>>, WorldError> {
    let (entries, candidates) =
        motions_with::<PLANAR>(snapshot, components, pending, forced, Candidates::new)
            .ok_or(WorldError::CrossIsland)?;
    let mut parents: Vec<usize> = (0..components.len()).collect();
    fn root(parents: &mut [usize], mut i: usize) -> usize {
        while parents[i] != i {
            parents[i] = parents[parents[i]];
            i = parents[i];
        }
        i
    }
    candidates.visit(|i, j| {
        let a = &entries[i];
        let b = &entries[j];
        let first = root(&mut parents, a.component);
        let second = root(&mut parents, b.component);
        if first != second && overlaps_bound::<PLANAR>(a, b) {
            parents[first.max(second)] = first.min(second);
        }
    });
    let mut groups = BTreeMap::<usize, Vec<usize>>::new();
    for i in 0..components.len() {
        groups.entry(root(&mut parents, i)).or_default().push(i);
    }
    Ok(groups.into_values().collect())
}

fn validate_warm_join(
    snapshot: &Snapshot,
    warm: &ComponentInfo,
    cold: &[&ComponentInfo],
) -> Result<(), WorldError> {
    let count = warm.ids.len() + cold.iter().map(|c| c.ids.len()).sum::<usize>();

    let mut resident = Vec::with_capacity(count);
    for member in std::iter::once(warm).chain(cold.iter().copied()) {
        let before = resident.len();
        for id in &member.ids {
            let b = snapshot
                .bodies
                .iter()
                .find(|b| b.id == *id)
                .ok_or(WorldError::CrossIsland)?;
            let bounds = b.bounds()?;
            // Preserve contacts internal to a native warm donor. Different
            // solvers must be brought together before their bounds intersect.
            let others = if member.contact_history {
                &resident[..before]
            } else {
                &resident[..]
            };
            if others.iter().any(|other| bounds.predicts_overlap(other)) {
                return Err(WorldError::CrossIsland);
            }
            resident.push(bounds);
        }
    }
    Ok(())
}

pub(super) fn groups(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
) -> Result<Vec<Vec<usize>>, WorldError> {
    forced_groups(snapshot, components, pending, &[])
}

pub(crate) fn forced_groups(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
) -> Result<Vec<Vec<usize>>, WorldError> {
    forced_groups_with_profile(
        snapshot,
        components,
        pending,
        forced,
        NativeSolverProfile::Standard32,
    )
}

pub(crate) fn forced_groups_with_profile(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
    forced: &[ForceMotion],
    profile: NativeSolverProfile,
) -> Result<Vec<Vec<usize>>, WorldError> {
    let groups = forced_contact_groups(snapshot, components, pending, forced)?;
    for g in &groups {
        if g.iter().map(|&i| components[i].ids.len()).sum::<usize>() > profile.max_bodies() {
            return Err(WorldError::CrossIsland);
        }
        if g.len() <= 1 {
            continue;
        }

        if let Some(&warm) = g.iter().find(|&&i| components[i].contact_history) {
            let cold: Vec<_> = g
                .iter()
                .filter(|&&i| i != warm)
                .map(|&i| &components[i])
                .collect();
            validate_warm_join(snapshot, &components[warm], &cold)?;
        }
    }
    Ok(groups)
}

/// Physical admission for advancing a worker whose observed native component
/// memberships and ordered command ledger match the supplied barrier.
pub fn partition_can_step(
    snapshot: &Snapshot,
    components: &[ComponentInfo],
    pending: &[PendingImpulse],
) -> bool {
    groups(snapshot, components, pending).is_ok()
}

#[cfg(test)]
#[path = "../../tests/private/swept_discovery.rs"]
mod discovery_tests;
