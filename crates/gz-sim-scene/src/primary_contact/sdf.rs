// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Strict authored pairs for local primary impact refinement.
use super::*;
use std::collections::BTreeSet;

pub(crate) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::PrimaryContactRefinement"
        && p.filename == "gz-sim-primary-contact-refinement-system"
}
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Projection {
    Corner,
    Planar,
    SlidingEdge,
    MixedEdge,
    Edge,
    Finite,
    FiniteSegment,
    FrictionlessSole,
}
pub(super) struct NamedPair {
    pub incoming_face: Option<DVec3>,
    pub follow_planar_edge: bool,
    pub follow_sliding_vertex: bool,
    pub follow_sticking_point: bool,
    pub compact_point_tail: bool,
    pub refine_active_point: bool,
    pub detect_initial_point: bool,
    pub subdivide_point_bounds: bool,
    pub target_point_bounds: bool,
    pub resolve_point_transfers: bool,
    pub follow_sliding_skew_edge: bool,
    pub follow_skew_slip_launch: bool,
    pub follow_frictionless_edge: bool,
    pub resolve_rocking_limit: bool,
    pub follow_frictionless_face: bool,
    pub follow_frictionless_deck_edge: bool,
    pub share_coplanar_face: bool,
    pub share_uniform_sliding: bool,
    pub settle_comoving_supports: bool,
    pub retime_finite_transfer: bool,
    pub certify_face_vertex: bool,
    pub certify_skew_edge: bool,
    pub resolve_fractional_points: bool,
    pub certify_segment_friction_work: bool,
    pub foot: String,
    pub support: String,
    pub foot_half: DVec3,
    pub pad_half: DVec3,
    pub refresh_box_features: bool,
    pub refresh_returning_box_features: bool,
    pub prospective_timing: bool,
    pub guard_separated_flight: bool,
    pub integrate_free_rotation: bool,
    pub ballistic_translation: bool,
    pub integrate_released_rotation: bool,
    pub refine_rotation_timing: bool,
    pub detect_separated_crossings: bool,
    pub detect_finite_arrival: bool,
    pub compact_budget: bool,
    pub rigid_corner: bool,
    pub rigid_corner_neighbors: bool,
    pub opponent: Option<(String, DVec3)>,
    pub project_opponent_corner: bool,
    pub project_opponent_return: bool,
    pub project_opposing_normals: bool,
    pub rigid_departure: bool,
    pub rotation_pulse: Option<gz_physics_multifidelity::RotationContactPulse>,
    pub projection: Option<Projection>,
}
pub(crate) struct Plan {
    pub(super) world: String,
    pub(super) calls: u32,
    pub(super) iterations: ContactSolverIterations,
    pub(super) pairs: Vec<NamedPair>,
}
fn children<'a, 'i>(n: roxmltree::Node<'a, 'i>) -> impl Iterator<Item = roxmltree::Node<'a, 'i>> {
    n.children().filter(|n| {
        !(n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty())
    })
}
fn field<'a, 'i>(n: roxmltree::Node<'a, 'i>) -> Option<&'a str> {
    let mut xs = children(n);
    let text = xs.next()?;
    if !text.is_text() || xs.next().is_some() {
        return None;
    }
    text.text().map(str::trim).filter(|s| !s.is_empty())
}
fn find_box<'a>(
    sdf: &sdformat::Sdf,
    world: &'a sdformat::World,
    name: &str,
) -> Option<(DVec3, &'a str)> {
    fn within<'a>(
        sdf: &sdformat::Sdf,
        models: &'a [sdformat::Model],
        name: &str,
    ) -> Option<(DVec3, &'a str)> {
        for model in models {
            for link in &model.links {
                if link.scoped_name != name {
                    continue;
                }
                if let [c] = link.collisions.as_slice()
                    && let sdformat::Geometry::Box { size } = c.geometry
                    && sdf
                        .resolve_pose(&c.pose, &model.scoped_name, name, name)
                        .is_ok_and(crate::identity)
                {
                    return Some((DVec3::from(size) * 0.5, &model.scoped_name));
                }
                return None;
            }
            if let Some(v) = within(sdf, &model.models, name) {
                return Some(v);
            }
        }
        None
    }
    within(sdf, &world.models, name)
}
pub(crate) fn parse(
    sdf: &sdformat::Sdf,
    world: &sdformat::World,
    engine: crate::PhysicsEngine,
) -> Result<Option<Plan>, LoadError> {
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(p) = plugins.next() else {
        return Ok(None);
    };
    let err = |s: &str| invalid(&world.name, s);
    if plugins.next().is_some() {
        return Err(err("use one world plugin containing one or more pairs"));
    }
    if engine != crate::PhysicsEngine::MultiFidelity {
        return Err(err("requires --physics-engine multifidelity"));
    }
    let xml = roxmltree::Document::parse(&p.xml).map_err(|e| err(&e.to_string()))?;
    let root = xml.root_element();
    if !root.has_tag_name("plugin")
        || root.namespaces().next().is_some()
        || root.attributes().len() != 2
        || root.attribute("name") != Some(&p.name)
        || root.attribute("filename") != Some(&p.filename)
    {
        return Err(err("invalid plugin attributes"));
    }
    let mut calls = None;
    let mut iters = None;
    let mut pgs = None;
    let mut relaxation = None;
    let mut pairs = vec![];
    let mut seen = BTreeSet::new();
    for n in children(root) {
        if !n.is_element() || n.namespaces().next().is_some() || n.attributes().len() != 0 {
            return Err(err("invalid policy child or attributes"));
        }
        match n.tag_name().name() {
            "native_steps" if calls.is_none() => {
                calls = Some(
                    field(n)
                        .and_then(|s| s.parse::<u32>().ok())
                        .filter(|n| *n > 0)
                        .ok_or_else(|| err("native_steps must be a positive integer"))?,
                );
            }
            "solver_iterations" if iters.is_none() => {
                iters = Some(
                    field(n)
                        .and_then(|s| s.parse::<usize>().ok())
                        .filter(|n| (1..=32).contains(n))
                        .ok_or_else(|| err("solver_iterations must be in 1..=32"))?,
                );
            }
            "pgs_iterations" if pgs.is_none() => {
                pgs = Some(
                    field(n)
                        .and_then(|s| s.parse::<usize>().ok())
                        .filter(|n| (1..=32).contains(n))
                        .ok_or_else(|| err("pgs_iterations must be in 1..=32"))?,
                );
            }
            "relaxation_iterations" if relaxation.is_none() => {
                relaxation = Some(
                    field(n)
                        .and_then(|s| s.parse::<usize>().ok())
                        .filter(|n| *n <= 32)
                        .ok_or_else(|| err("relaxation_iterations must be in 0..=32"))?,
                );
            }
            "pair" => {
                let mut incoming_face = None;
                let mut foot = None;
                let mut support = None;
                let mut refresh = None;
                let mut returning_refresh = None;
                let mut timing = None;
                let mut guard = None;
                let mut rotate = None;
                let mut ballistic = None;
                let mut release = None;
                let mut refine = None;
                let mut detect = None;
                let mut compact = None;
                let mut rigid_corner = None;
                let mut rigid_corner_neighbors = false;
                let mut opposing_name = None;
                let mut project_opponent_corner = None;
                let mut project_opponent_return = None;
                let mut project_opposing_normals = None;
                let mut rigid_departure = None;
                let mut pulse = None;
                let mut follow = None;
                let mut vertex = None;
                let mut sticking_point = None;
                let mut compact_point = None;
                let mut refine_active_point = None;
                let mut detect_initial_point = None;
                let mut subdivide = None;
                let mut directed = None;
                let mut point_transfers = None;
                let mut skew_slide = None;
                let mut finite_arrival = None;
                let mut slip_launch = None;
                let mut face_follow = None;
                let mut deck_follow = None;
                let mut share_face = None;
                let mut kinetic_share = None;
                let mut settle_comoving = None;
                let mut rocking_limit = None;
                let mut frictionless_follow = None;
                let mut project = None;
                let mut planar = None;
                let mut edge = None;
                let mut mixed = None;
                let mut any_edge = None;
                let mut frictionless_sole = None;
                let mut finite = None;
                let mut certify_vertex = None;
                let mut certify_skew = None;
                let mut fractional_points = None;
                let mut segment_work = None;
                let mut segment = None;
                let mut transfer = None;
                for a in children(n) {
                    if !a.is_element()
                        || a.attributes().len() != 0
                        || a.namespaces().next().is_some()
                    {
                        return Err(err("invalid pair child"));
                    }
                    let v = field(a).ok_or_else(|| err("expected a plain scoped link name"))?;
                    match a.tag_name().name() {
                        "project_opposing_normals" if project_opposing_normals.is_none() => {
                            project_opposing_normals = Some(v.parse::<bool>().map_err(|_| {
                                err("project_opposing_normals must be true or false")
                            })?);
                        }
                        "project_opponent_return" if project_opponent_return.is_none() => {
                            project_opponent_return = Some(v.parse::<bool>().map_err(|_| {
                                err("project_opponent_return must be true or false")
                            })?);
                        }
                        "project_opponent_corner" if project_opponent_corner.is_none() => {
                            project_opponent_corner = Some(v.parse::<bool>().map_err(|_| {
                                err("project_opponent_corner must be true or false")
                            })?);
                        }
                        "guard_opponent" if opposing_name.is_none() => opposing_name = Some(v),
                        "incoming_face" if incoming_face.is_none() => {
                            incoming_face = Some(match v {
                                "+x" => DVec3::X,
                                "-x" => DVec3::NEG_X,
                                "+y" => DVec3::Y,
                                "-y" => DVec3::NEG_Y,
                                "+z" => DVec3::Z,
                                "-z" => DVec3::NEG_Z,
                                _ => return Err(err("incoming_face requires a signed local axis")),
                            });
                        }
                        "foot" if foot.is_none() => foot = Some(v),
                        "support" if support.is_none() => support = Some(v),
                        "resolve_fractional_points" if fractional_points.is_none() => {
                            fractional_points = Some(v.parse::<bool>().map_err(|_| {
                                err("resolve_fractional_points must be true or false")
                            })?);
                        }
                        "certify_skew_edge" if certify_skew.is_none() => {
                            certify_skew = Some(
                                v.parse::<bool>()
                                    .map_err(|_| err("certify_skew_edge must be true or false"))?,
                            );
                        }
                        "certify_face_vertex" if certify_vertex.is_none() => {
                            certify_vertex =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("certify_face_vertex must be true or false")
                                })?);
                        }
                        "retime_finite_transfer" if transfer.is_none() => {
                            transfer = Some(v.parse::<bool>().map_err(|_| {
                                err("retime_finite_transfer must be true or false")
                            })?);
                        }
                        "certify_segment_friction_work" if segment_work.is_none() => {
                            segment_work = Some(v.parse::<bool>().map_err(|_| {
                                err("certify_segment_friction_work must be true or false")
                            })?);
                        }
                        "project_segment_endpoint" if segment.is_none() => {
                            segment = Some(v.parse::<bool>().map_err(|_| {
                                err("project_segment_endpoint must be true or false")
                            })?);
                        }
                        "project_finite_point" if finite.is_none() => {
                            finite =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("project_finite_point must be true or false")
                                })?);
                        }
                        "project_frictionless_sole" if frictionless_sole.is_none() => {
                            frictionless_sole = Some(v.parse::<bool>().map_err(|_| {
                                err("project_frictionless_sole must be true or false")
                            })?);
                        }
                        "project_edge" if any_edge.is_none() => {
                            any_edge = Some(
                                v.parse::<bool>()
                                    .map_err(|_| err("project_edge must be true or false"))?,
                            );
                        }
                        "project_mixed_edge" if mixed.is_none() => {
                            mixed =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("project_mixed_edge must be true or false")
                                })?);
                        }
                        "project_sliding_edge" if edge.is_none() => {
                            edge =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("project_sliding_edge must be true or false")
                                })?);
                        }
                        "project_planar" if planar.is_none() => {
                            planar = Some(
                                v.parse::<bool>()
                                    .map_err(|_| err("project_planar must be true or false"))?,
                            );
                        }
                        "settle_comoving_supports" if settle_comoving.is_none() => {
                            settle_comoving = Some(v.parse::<bool>().map_err(|_| {
                                err("settle_comoving_supports must be true or false")
                            })?);
                        }
                        "share_uniform_sliding" if kinetic_share.is_none() => {
                            kinetic_share =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("share_uniform_sliding must be true or false")
                                })?);
                        }
                        "share_coplanar_face" if share_face.is_none() => {
                            share_face =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("share_coplanar_face must be true or false")
                                })?);
                        }
                        "follow_frictionless_deck_edge" if deck_follow.is_none() => {
                            deck_follow = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_frictionless_deck_edge must be true or false")
                            })?);
                        }
                        "follow_frictionless_face" if face_follow.is_none() => {
                            face_follow = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_frictionless_face must be true or false")
                            })?);
                        }
                        "resolve_rocking_limit" if rocking_limit.is_none() => {
                            rocking_limit =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("resolve_rocking_limit must be true or false")
                                })?);
                        }
                        "follow_frictionless_edge" if frictionless_follow.is_none() => {
                            frictionless_follow = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_frictionless_edge must be true or false")
                            })?);
                        }
                        "follow_skew_slip_launch" if slip_launch.is_none() => {
                            slip_launch = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_skew_slip_launch must be true or false")
                            })?);
                        }
                        "follow_sliding_skew_edge" if skew_slide.is_none() => {
                            skew_slide = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_sliding_skew_edge must be true or false")
                            })?);
                        }
                        "target_point_bounds" if directed.is_none() => {
                            directed =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("target_point_bounds must be true or false")
                                })?);
                        }
                        "detect_initial_point" if detect_initial_point.is_none() => {
                            detect_initial_point =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("detect_initial_point must be true or false")
                                })?);
                        }
                        "refine_active_point" if refine_active_point.is_none() => {
                            refine_active_point =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("refine_active_point must be true or false")
                                })?);
                        }
                        "compact_point_tail" if compact_point.is_none() => {
                            compact_point =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("compact_point_tail must be true or false")
                                })?);
                        }
                        "subdivide_point_bounds" if subdivide.is_none() => {
                            subdivide = Some(v.parse::<bool>().map_err(|_| {
                                err("subdivide_point_bounds must be true or false")
                            })?);
                        }
                        "resolve_point_transfers" if point_transfers.is_none() => {
                            point_transfers = Some(v.parse::<bool>().map_err(|_| {
                                err("resolve_point_transfers must be true or false")
                            })?);
                        }
                        "follow_sticking_point" if sticking_point.is_none() => {
                            sticking_point =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("follow_sticking_point must be true or false")
                                })?);
                        }
                        "follow_sliding_vertex" if vertex.is_none() => {
                            vertex =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("follow_sliding_vertex must be true or false")
                                })?);
                        }
                        "follow_planar_edge" if follow.is_none() => {
                            follow =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("follow_planar_edge must be true or false")
                                })?);
                        }
                        "project_corner" if project.is_none() => {
                            project = Some(
                                v.parse::<bool>()
                                    .map_err(|_| err("project_corner must be true or false"))?,
                            );
                        }
                        "rotation_contact_pulse" if pulse.is_none() => {
                            use gz_physics_multifidelity::RotationContactPulse;
                            pulse = Some(if v == "measured" {
                                RotationContactPulse::Measured
                            } else {
                                let ns=v.strip_suffix("ns").and_then(|v|v.parse::<u16>().ok())
                                    .filter(|n|(1..=4000).contains(n)).ok_or_else(||err("rotation_contact_pulse requires measured or 1ns through 4000ns"))?;
                                RotationContactPulse::Fixed(ns)
                            });
                        }
                        "follow_rigid_departure" if rigid_departure.is_none() => {
                            rigid_departure = Some(v.parse::<bool>().map_err(|_| {
                                err("follow_rigid_departure must be true or false")
                            })?);
                        }
                        "project_rigid_corner" if rigid_corner.is_none() => {
                            rigid_corner_neighbors = v == "clear_neighbors";
                            rigid_corner = Some(match v {
                                "true" | "clear_neighbors" => true,
                                "false" => false,
                                _ => {
                                    return Err(err(
                                        "project_rigid_corner must be true, false or clear_neighbors",
                                    ));
                                }
                            });
                        }
                        "compact_impact_budget" if compact.is_none() => {
                            compact =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("compact_impact_budget must be true or false")
                                })?);
                        }
                        "detect_finite_arrival" if finite_arrival.is_none() => {
                            finite_arrival =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("detect_finite_arrival must be true or false")
                                })?);
                        }
                        "detect_separated_crossings" if detect.is_none() => {
                            detect = Some(v.parse::<bool>().map_err(|_| {
                                err("detect_separated_crossings must be true or false")
                            })?);
                        }
                        "refine_rotation_timing" if refine.is_none() => {
                            refine = Some(v.parse::<bool>().map_err(|_| {
                                err("refine_rotation_timing must be true or false")
                            })?);
                        }
                        "integrate_released_rotation" if release.is_none() => {
                            release = Some(v.parse::<bool>().map_err(|_| {
                                err("integrate_released_rotation must be true or false")
                            })?);
                        }
                        "integrate_ballistic_translation" if ballistic.is_none() => {
                            ballistic = Some(v.parse::<bool>().map_err(|_| {
                                err("integrate_ballistic_translation must be true or false")
                            })?);
                        }
                        "integrate_free_rotation" if rotate.is_none() => {
                            rotate = Some(v.parse::<bool>().map_err(|_| {
                                err("integrate_free_rotation must be true or false")
                            })?);
                        }
                        "guard_separated_flight" if guard.is_none() => {
                            guard = Some(v.parse::<bool>().map_err(|_| {
                                err("guard_separated_flight must be true or false")
                            })?);
                        }
                        "prospective_timing" if timing.is_none() => {
                            timing =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("prospective_timing must be true or false")
                                })?);
                        }
                        "refresh_returning_box_features" if returning_refresh.is_none() => {
                            returning_refresh = Some(v.parse::<bool>().map_err(|_| {
                                err("refresh_returning_box_features must be true or false")
                            })?);
                        }
                        "refresh_box_features" if refresh.is_none() => {
                            refresh =
                                Some(v.parse::<bool>().map_err(|_| {
                                    err("refresh_box_features must be true or false")
                                })?);
                        }
                        _ => return Err(err("each pair requires one foot and one support")),
                    }
                }
                if incoming_face.is_some() && children(n).count() != 3 {
                    return Err(err(
                        "incoming_face accepts only the foot and support fields",
                    ));
                }
                if returning_refresh == Some(true) && refresh == Some(true) {
                    return Err(err(
                        "choose either continuous or returning box feature refresh for this pair",
                    ));
                }
                let (foot, support) = foot
                    .zip(support)
                    .ok_or_else(|| err("each pair requires one foot and one support"))?;
                let (foot_half, model) = find_box(sdf, world, foot).ok_or_else(|| {
                    err(&format!(
                        "foot requires a scoped link with one box collision: {foot}"
                    ))
                })?;
                let (pad_half, pad_model) = find_box(sdf, world, support).ok_or_else(|| {
                    err(&format!(
                        "support requires a scoped link with one box collision: {support}"
                    ))
                })?;
                if project_opposing_normals == Some(true) && project_opponent_corner != Some(true) {
                    return Err(err(
                        "project_opposing_normals requires project_opponent_corner",
                    ));
                }
                if project_opponent_return == Some(true) && project_opponent_corner != Some(true) {
                    return Err(err(
                        "project_opponent_return requires project_opponent_corner",
                    ));
                }
                if project_opponent_corner == Some(true) && opposing_name.is_none() {
                    return Err(err("project_opponent_corner requires guard_opponent"));
                }
                let opponent = opposing_name.map(|name| {
                    if !rigid_corner_neighbors || name == foot || name == support {
                        return Err(err("guard_opponent requires clear_neighbors and a distinct scoped link"));
                    }
                    let (half, _) = find_box(sdf, world, name).ok_or_else(|| err("guard_opponent requires a scoped link with one box collision"))?;
                    Ok((name.to_owned(), half))
                }).transpose()?;
                if model == pad_model || !seen.insert((foot.to_owned(), support.to_owned())) {
                    return Err(err("pairs must be unique and join distinct models"));
                }
                if pulse.is_some() && refine != Some(true) {
                    return Err(err(
                        "rotation_contact_pulse requires refine_rotation_timing",
                    ));
                }
                if rigid_departure == Some(true) && rigid_corner != Some(true) {
                    return Err(err("follow_rigid_departure requires project_rigid_corner"));
                }
                if rigid_corner == Some(true)
                    && (detect != Some(true)
                        || pulse != Some(gz_physics_multifidelity::RotationContactPulse::Measured))
                {
                    return Err(err(
                        "project_rigid_corner requires detect_separated_crossings and measured rotation_contact_pulse",
                    ));
                }
                if compact == Some(true)
                    && (detect != Some(true)
                        || pulse != Some(gz_physics_multifidelity::RotationContactPulse::Measured))
                {
                    return Err(err(
                        "compact_impact_budget requires detect_separated_crossings and measured rotation_contact_pulse",
                    ));
                }
                if detect == Some(true) && refine != Some(true) {
                    return Err(err(
                        "detect_separated_crossings requires refine_rotation_timing",
                    ));
                }
                if refine == Some(true) && rotate != Some(true) {
                    return Err(err(
                        "refine_rotation_timing requires integrate_free_rotation",
                    ));
                }
                if release == Some(true) && rotate != Some(true) {
                    return Err(err(
                        "integrate_released_rotation requires integrate_free_rotation",
                    ));
                }
                if ballistic == Some(true) && rotate != Some(true) {
                    return Err(err(
                        "integrate_ballistic_translation requires integrate_free_rotation",
                    ));
                }
                if rotate == Some(true) && guard != Some(true) {
                    return Err(err(
                        "integrate_free_rotation requires guard_separated_flight",
                    ));
                }
                if guard == Some(true) && timing != Some(true) {
                    return Err(err("guard_separated_flight requires prospective_timing"));
                }
                if fractional_points == Some(true) && finite != Some(true) {
                    return Err(err(
                        "resolve_fractional_points requires project_finite_point",
                    ));
                }
                if certify_skew == Some(true) && finite != Some(true) {
                    return Err(err("certify_skew_edge requires project_finite_point"));
                }
                if certify_vertex == Some(true) && finite != Some(true) {
                    return Err(err("certify_face_vertex requires project_finite_point"));
                }
                if slip_launch == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "follow_skew_slip_launch requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if skew_slide == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "follow_sliding_skew_edge requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if directed == Some(true) && subdivide != Some(true) {
                    return Err(err("target_point_bounds requires subdivide_point_bounds"));
                }
                if detect_initial_point == Some(true) && point_transfers != Some(true) {
                    return Err(err("detect_initial_point requires resolve_point_transfers"));
                }
                if refine_active_point == Some(true) && point_transfers != Some(true) {
                    return Err(err("refine_active_point requires resolve_point_transfers"));
                }
                if compact_point == Some(true) && point_transfers != Some(true) {
                    return Err(err("compact_point_tail requires resolve_point_transfers"));
                }
                if subdivide == Some(true) && point_transfers != Some(true) {
                    return Err(err(
                        "subdivide_point_bounds requires resolve_point_transfers",
                    ));
                }
                if point_transfers == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "resolve_point_transfers requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if sticking_point == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "follow_sticking_point requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if vertex == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "follow_sliding_vertex requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if transfer == Some(true) && finite != Some(true) && segment != Some(true) {
                    return Err(err(
                        "retime_finite_transfer requires project_finite_point or project_segment_endpoint",
                    ));
                }
                if segment_work == Some(true) && segment != Some(true) {
                    return Err(err(
                        "certify_segment_friction_work requires project_segment_endpoint",
                    ));
                }
                let enabled = [
                    (project, Projection::Corner),
                    (planar, Projection::Planar),
                    (edge, Projection::SlidingEdge),
                    (mixed, Projection::MixedEdge),
                    (any_edge, Projection::Edge),
                    (frictionless_sole, Projection::FrictionlessSole),
                    (finite, Projection::Finite),
                    (segment, Projection::FiniteSegment),
                ];
                let mut enabled = enabled
                    .into_iter()
                    .filter_map(|(flag, geometry)| (flag == Some(true)).then_some(geometry));
                let projection = enabled.next();
                if enabled.next().is_some() {
                    return Err(err("choose one projected contact geometry per pair"));
                }
                if settle_comoving == Some(true) && kinetic_share != Some(true) {
                    return Err(err(
                        "settle_comoving_supports requires share_uniform_sliding",
                    ));
                }
                if kinetic_share == Some(true) && share_face != Some(true) {
                    return Err(err("share_uniform_sliding requires share_coplanar_face"));
                }
                if share_face == Some(true)
                    && (face_follow != Some(true)
                        || projection.is_some()
                        || deck_follow == Some(true)
                        || timing == Some(true)
                        || guard == Some(true))
                {
                    return Err(err(
                        "share_coplanar_face requires follow_frictionless_face, with no other projection, timing or deck-edge continuation",
                    ));
                }
                if deck_follow == Some(true) && face_follow != Some(true) {
                    return Err(err(
                        "follow_frictionless_deck_edge requires follow_frictionless_face",
                    ));
                }
                if rocking_limit == Some(true)
                    && (projection != Some(Projection::FrictionlessSole)
                        || frictionless_follow != Some(true))
                {
                    return Err(err(
                        "resolve_rocking_limit requires project_frictionless_sole and follow_frictionless_edge",
                    ));
                }
                if frictionless_follow == Some(true)
                    && !matches!(
                        projection,
                        Some(Projection::Edge | Projection::FrictionlessSole)
                    )
                {
                    return Err(err(
                        "follow_frictionless_edge requires project_edge or project_frictionless_sole",
                    ));
                }
                if follow == Some(true) && projection != Some(Projection::Planar) {
                    return Err(err("follow_planar_edge requires project_planar"));
                }
                pairs.push(NamedPair {
                    incoming_face,
                    follow_planar_edge: follow.unwrap_or(false),
                    follow_sliding_vertex: vertex.unwrap_or(false),
                    follow_sticking_point: sticking_point.unwrap_or(false),
                    resolve_point_transfers: point_transfers.unwrap_or(false),
                    subdivide_point_bounds: subdivide.unwrap_or(false),
                    compact_point_tail: compact_point.unwrap_or(false),
                    refine_active_point: refine_active_point.unwrap_or(false),
                    detect_initial_point: detect_initial_point.unwrap_or(false),
                    target_point_bounds: directed.unwrap_or(false),
                    follow_sliding_skew_edge: skew_slide.unwrap_or(false),
                    follow_skew_slip_launch: slip_launch.unwrap_or(false),
                    follow_frictionless_edge: frictionless_follow.unwrap_or(false),
                    resolve_rocking_limit: rocking_limit.unwrap_or(false),
                    follow_frictionless_face: face_follow.unwrap_or(false),
                    follow_frictionless_deck_edge: deck_follow.unwrap_or(false),
                    share_coplanar_face: share_face.unwrap_or(false),
                    share_uniform_sliding: kinetic_share.unwrap_or(false),
                    settle_comoving_supports: settle_comoving.unwrap_or(false),
                    retime_finite_transfer: transfer.unwrap_or(false),
                    certify_face_vertex: certify_vertex.unwrap_or(false),
                    certify_skew_edge: certify_skew.unwrap_or(false),
                    resolve_fractional_points: fractional_points.unwrap_or(false),
                    certify_segment_friction_work: segment_work.unwrap_or(false),
                    foot: foot.into(),
                    support: support.into(),
                    foot_half,
                    pad_half,
                    refresh_box_features: refresh.unwrap_or(false),
                    refresh_returning_box_features: returning_refresh.unwrap_or(false),
                    prospective_timing: timing.unwrap_or(false),
                    guard_separated_flight: guard.unwrap_or(false),
                    integrate_free_rotation: rotate.unwrap_or(false),
                    ballistic_translation: ballistic.unwrap_or(false),
                    integrate_released_rotation: release.unwrap_or(false),
                    refine_rotation_timing: refine.unwrap_or(false),
                    detect_separated_crossings: detect.unwrap_or(false),
                    detect_finite_arrival: finite_arrival.unwrap_or(false),
                    compact_budget: compact.unwrap_or(false),
                    rigid_corner: rigid_corner.unwrap_or(false),
                    rigid_corner_neighbors,
                    opponent,
                    project_opponent_corner: project_opponent_corner.unwrap_or(false),
                    project_opponent_return: project_opponent_return.unwrap_or(false),
                    project_opposing_normals: project_opposing_normals.unwrap_or(false),
                    rigid_departure: rigid_departure.unwrap_or(false),
                    rotation_pulse: pulse,
                    projection,
                });
            }
            _ => return Err(err("unknown or duplicate policy field")),
        }
    }
    if pairs.is_empty() {
        return Err(err("requires at least one foot/support pair"));
    }
    for p in pairs.iter().filter(|p| p.share_coplanar_face) {
        if pairs.iter().any(|v| {
            v.foot == p.foot
                && v.share_coplanar_face
                && (v.share_uniform_sliding != p.share_uniform_sliding
                    || v.settle_comoving_supports != p.settle_comoving_supports)
        }) {
            return Err(err(
                "both shared supports must agree on share_uniform_sliding and settle_comoving_supports",
            ));
        }
        if pairs
            .iter()
            .filter(|v| v.foot == p.foot && v.share_coplanar_face)
            .count()
            != 2
        {
            return Err(err(
                "share_coplanar_face requires two distinct enrolled supports on the same foot",
            ));
        }
    }
    Ok(Some(Plan {
        world: world.name.clone(),
        calls: calls.unwrap_or(8),
        iterations: ContactSolverIterations {
            temporal: iters.unwrap_or(8),
            pgs,
            relaxation,
        },
        pairs,
    }))
}
