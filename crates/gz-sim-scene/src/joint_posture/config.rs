// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::{Model, Sdf};
use std::collections::BTreeSet;

pub(super) struct AxisConfig {
    pub name: String,
    pub goal: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub max_effort: f64,
    pub feedback: Feedback,
    pub limits: [f64; 2],
}
pub(crate) struct Plan {
    pub(super) model: String,
    pub(super) links: Vec<(String, f64, DVec3)>,
    pub(super) joints: Vec<AxisConfig>,
    pub(super) gains: [f64; 2],
    pub(super) footsteps: Option<footsteps::FootstepPlan>,
    pub(super) max_correction: [f64; 2],
}
pub(crate) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::JointPosture" && p.filename == "gz-sim-joint-posture-system"
}
fn error(entity: &str, why: impl Into<String>) -> LoadError {
    LoadError::JointPosture {
        entity: entity.into(),
        reason: why.into(),
    }
}
fn num(n: roxmltree::Node, attr: &str, entity: &str) -> Result<f64, LoadError> {
    n.attribute(attr)
        .and_then(|v| v.parse::<f64>().ok())
        .filter(|v| v.is_finite())
        .ok_or_else(|| error(entity, format!("missing or nonfinite {attr}")))
}
fn attrs(n: roxmltree::Node, names: &[&str]) -> bool {
    n.is_element()
        && n.tag_name().namespace().is_none()
        && n.attributes()
            .all(|a| a.namespace().is_none() && names.contains(&a.name()))
}
fn empty(n: roxmltree::Node) -> bool {
    n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty()
}
fn source_axis(model: &Model, n: roxmltree::Node) -> Result<AxisConfig, LoadError> {
    let entity = &model.scoped_name;
    let fault = |s: &str| error(entity, s);
    if !attrs(
        n,
        &[
            "joint",
            "reference",
            "stiffness",
            "damping",
            "max_effort",
            "feedback",
        ],
    ) || !n.children().all(empty)
    {
        return Err(fault("invalid actuator attributes or nested configuration"));
    }
    let name = n.attribute("joint").unwrap_or_default();
    let joint = model
        .joints
        .iter()
        .find(|j| j.name == name && j.kind == sdformat::JointKind::Revolute)
        .ok_or_else(|| fault("actuator needs a named revolute joint in the selected model"))?;
    let axis = joint
        .axis
        .as_ref()
        .ok_or_else(|| fault("missing revolute axis"))?;
    let goal = num(n, "reference", entity)?;
    let stiffness = num(n, "stiffness", entity)?;
    let damping = num(n, "damping", entity)?;
    let max_effort = num(n, "max_effort", entity)?;
    if stiffness <= 0. || damping < 0. || max_effort <= 0. || goal < axis.lower || goal > axis.upper
    {
        return Err(fault(
            "invalid actuator gains, torque cap, or reference outside joint limits",
        ));
    }
    let feedback = match n.attribute("feedback") {
        None => Feedback::None,
        Some("x") => Feedback::X,
        Some("-y") => Feedback::NegativeY,
        _ => return Err(fault("feedback must be x or -y")),
    };
    Ok(AxisConfig {
        name: format!("{entity}::{name}"),
        goal,
        stiffness,
        damping,
        max_effort,
        feedback,
        limits: [axis.lower, axis.upper],
    })
}
pub(crate) fn parse(
    sdf: &Sdf,
    w: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<Vec<Plan>, LoadError> {
    let mut plans = vec![];
    let mut owners = BTreeSet::new();
    for p in w.plugins.iter().filter(|p| is_plugin(p)) {
        let fault = |s: &str| error(&w.name, s);
        if engine != PhysicsEngine::MultiFidelity {
            return Err(fault(
                "requires the multifidelity reduced-coordinate solver",
            ));
        }
        let xml = roxmltree::Document::parse(&p.xml).map_err(|e| error(&w.name, e.to_string()))?;
        let root = xml.root_element();
        if root.tag_name().name() != "plugin" || !attrs(root, &["name", "filename"]) {
            return Err(fault("invalid plugin attributes"));
        }
        let children = root.children().filter(|n| !empty(*n)).collect::<Vec<_>>();
        let mut robot = children
            .iter()
            .filter(|n| n.is_element() && n.tag_name().name() == "model");
        let field = robot
            .next()
            .filter(|n| attrs(**n, &[]) && n.children().all(|c| c.is_text()))
            .ok_or_else(|| fault("missing model name"))?;
        let scope = field.text().unwrap_or_default().trim();
        let model = w
            .models
            .iter()
            .find(|m| m.scoped_name == scope)
            .ok_or_else(|| fault("unknown top-level model"))?;
        if robot.next().is_some()
            || !owners.insert(scope.to_owned())
            || model.is_static
            || !model.models.is_empty()
            || model.plugins.len() != 1
            || !crate::articulation::is_plugin(&model.plugins[0])
        {
            return Err(fault(
                "expected a single top-level dynamic switchable tree for each posture controller",
            ));
        }
        let mut balance = None;
        let mut footsteps = None;
        let mut axes = vec![];
        let mut seen = BTreeSet::new();
        for n in children {
            match n.tag_name().name() {
                "model" => {}
                "balance"
                    if balance.is_none()
                        && attrs(n, &["position_gain", "velocity_gain", "max_x", "max_y"])
                        && n.children().all(empty) =>
                {
                    let gains = [
                        num(n, "position_gain", scope)?,
                        num(n, "velocity_gain", scope)?,
                    ];
                    let max = [num(n, "max_x", scope)?, num(n, "max_y", scope)?];
                    if gains.into_iter().chain(max).any(|v| v < 0.) {
                        return Err(fault("balance gains and limits must be nonnegative"));
                    }
                    balance = Some((gains, max));
                }
                "footsteps"
                    if footsteps.is_none()
                        && attrs(
                            n,
                            &[
                                "motion",
                                "lateral_m",
                                "length_m",
                                "count",
                                "placement_feedback",
                                "recovery_landing",
                                "support_relative",
                                "lateral_correction_limit_m",
                                "heading_gain",
                            ],
                        )
                        && n.children().all(empty) =>
                {
                    use footsteps::{FootstepMotion, FootstepPlan};
                    let motion = match n.attribute("motion") {
                        Some("weight-shift") => FootstepMotion::WeightShift,
                        Some("single-support") => FootstepMotion::SingleSupport,
                        Some("step") => FootstepMotion::Step,
                        Some("walk") => FootstepMotion::Walk,
                        _ => return Err(fault("invalid footstep motion")),
                    };
                    let placement_feedback = n
                        .attribute("placement_feedback")
                        .map(str::parse::<bool>)
                        .transpose()
                        .map_err(|_| fault("invalid foot placement feedback switch"))?
                        .unwrap_or(true);
                    let recovery_landing = n
                        .attribute("recovery_landing")
                        .map(str::parse::<bool>)
                        .transpose()
                        .map_err(|_| fault("invalid recovery landing switch"))?
                        .unwrap_or(false);
                    let support_relative = n
                        .attribute("support_relative")
                        .map(str::parse::<bool>)
                        .transpose()
                        .map_err(|_| fault("invalid support-relative placement switch"))?
                        .unwrap_or(false);
                    let lateral_correction_limit_m = n
                        .attribute("lateral_correction_limit_m")
                        .map(|_| num(n, "lateral_correction_limit_m", scope))
                        .transpose()?;
                    let plan = FootstepPlan {
                        heading_gain: n
                            .attribute("heading_gain")
                            .map(|_| num(n, "heading_gain", scope))
                            .transpose()?,
                        lateral_correction_limit_m,
                        support_relative,
                        recovery_landing,
                        placement_feedback,
                        motion,
                        lateral_m: num(n, "lateral_m", scope)?,
                        length_m: num(n, "length_m", scope)?,
                        count: n
                            .attribute("count")
                            .and_then(|v| v.parse().ok())
                            .ok_or_else(|| fault("invalid footstep count"))?,
                    };
                    if !plan.valid() || motion != FootstepMotion::Walk && plan.count != 1 {
                        return Err(fault("invalid footstep count or displacement"));
                    }
                    footsteps = Some(plan);
                }
                "actuator" => {
                    let a = source_axis(model, n)?;
                    if !seen.insert(a.name.clone()) {
                        return Err(fault("joint has two posture actuators"));
                    }
                    axes.push(a);
                }
                _ => return Err(fault("unknown or duplicate posture field")),
            }
        }
        if axes.is_empty() {
            return Err(fault("posture needs at least one actuator"));
        }
        if balance.is_none() && axes.iter().any(|a| !matches!(a.feedback, Feedback::None)) {
            return Err(fault("actuator feedback needs balance gains"));
        }
        let (gains, max_correction) = balance.unwrap_or(([0.; 2], [0.; 2]));
        let mut links = vec![];
        for link in &model.links {
            let name = &link.scoped_name;
            let inertia = sdf.resolve_pose(&link.inertial.pose, scope, name, name)?;
            if link.properties.kinematic
                || !link.inertial.mass.is_finite()
                || link.inertial.mass <= 0.
            {
                return Err(fault("posture requires massive dynamic links"));
            }
            links.push((
                name.clone(),
                link.inertial.mass,
                DVec3::from_array(inertia.translation),
            ));
        }
        plans.push(Plan {
            model: scope.into(),
            links,
            joints: axes,
            gains,
            footsteps,
            max_correction,
        });
    }
    Ok(plans)
}
