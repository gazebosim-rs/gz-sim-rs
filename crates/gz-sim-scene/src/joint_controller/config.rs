// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed native velocity/ABS-position plugin records for one revolute chain.
//! Field meanings and default topics follow gz-sim 71c60db3 JointController.cc
//! and JointPositionController.cc. PID, actuator and multi-DOF modes are deferred.
use crate::{LoadError, require};
use sdformat::{JointKind, Model, Plugin};
use std::collections::{BTreeMap, BTreeSet};

use super::{JointControllerConfiguration, JointControllerMode};

#[derive(Clone)]
pub(super) struct Parameters {
    pub joint: String,
    pub max_velocity: f64,
    pub max_force: f64,
    pub initial_target: f64,
    pub publication: JointControllerConfiguration,
}

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    (plugin.name == "gz::sim::systems::JointController"
        && plugin.filename == "gz-sim-joint-controller-system")
        || (plugin.name == "gz::sim::systems::JointPositionController"
            && plugin.filename == "gz-sim-joint-position-controller-system")
}

fn error(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}

fn fields(plugin: &Plugin, mode: JointControllerMode) -> Result<BTreeMap<&str, String>, ()> {
    let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| ())?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root.attributes().any(|attribute| {
            attribute.namespace().is_some() || !matches!(attribute.name(), "name" | "filename")
        })
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(());
    }
    let mut fields = BTreeMap::new();
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element() || child.namespaces().next().is_some() || child.attributes().len() != 0
            || child.children().any(|node| !node.is_text())
            // TinyXML selects the first lexical text node; roxmltree merges CDATA.
            // Keep the initial subset to ordinary, unsplit scalar text.
            || plugin.xml[child.range()].contains("<![CDATA[")
        {
            return Err(());
        }
        let name = child.tag_name().name();
        let allowed = matches!(name, "joint_name" | "topic")
            || match mode {
                JointControllerMode::Velocity => {
                    matches!(name, "use_force_commands" | "initial_velocity")
                }
                JointControllerMode::Position => matches!(
                    name,
                    "use_velocity_commands" | "cmd_max" | "initial_position" | "joint_index"
                ),
            };
        if !allowed {
            return Err(());
        }
        let value = child
            .text()
            .unwrap_or_default()
            .split_ascii_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        if fields.insert(name, value).is_some() {
            return Err(());
        }
    }
    Ok(fields)
}

fn number(fields: &BTreeMap<&str, String>, name: &str, default: Option<f64>) -> Result<f64, ()> {
    let Some(text) = fields.get(name) else {
        return default.ok_or(());
    };
    let value: f64 = text.parse().map_err(|_| ())?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(())
    }
}

fn valid_topic(topic: &str) -> bool {
    topic.starts_with('/')
        && topic != "/"
        && topic.len() <= 65535
        && !["~", " ", "//", "@", ":="]
            .into_iter()
            .any(|part| topic.contains(part))
}

fn chain(model: &Model) -> Result<(), LoadError> {
    require(
        !model.is_static && model.models.is_empty() && model.raw_namespace.is_none(),
        &model.scoped_name,
        "joint controller model domain",
    )?;
    require(
        model.links.len() == 2
            && model.joints.len() == 2
            && model
                .links
                .iter()
                .map(|link| &link.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 2
            && model
                .joints
                .iter()
                .map(|joint| &joint.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 2,
        &model.scoped_name,
        "joint controller chain",
    )?;
    let mut roots = model.joints.iter().filter(|joint| joint.parent == "world");
    let root = roots
        .next()
        .ok_or_else(|| error(model, "joint controller chain"))?;
    require(
        roots.next().is_none(),
        &model.scoped_name,
        "joint controller chain",
    )?;
    let (canonical, canonical_name) = model.canonical_link_and_relative_name();
    require(
        canonical.is_some() && canonical_name == root.child,
        &model.scoped_name,
        "joint controller chain",
    )?;
    let child = model
        .joints
        .iter()
        .find(|joint| joint.name != root.name)
        .expect("two distinct joint names include exactly one world root");
    require(
        child.parent == root.child && child.child != root.child,
        &model.scoped_name,
        "joint controller chain",
    )?;
    for link in &model.links {
        require(
            !link.name.is_empty()
                && !link.name.contains("::")
                && link.pose.relative_to.is_empty()
                // Preserve the legacy identity-LINK finite-translation domain.
                // Newly rotated LINK frames use the existing representable pose
                // boundary without normalizing or repairing the caller's DOM.
                && (link.pose.transform.rotation == [0.0, 0.0, 0.0, 1.0]
                    || crate::world_edit::representable(crate::WorldPose::from(link.pose.transform)))
                && link
                    .pose
                    .transform
                    .translation
                    .into_iter()
                    .all(f64::is_finite),
            &model.scoped_name,
            "joint controller frames",
        )?;
    }
    for joint in &model.joints {
        let axis = joint
            .axis
            .as_ref()
            .ok_or_else(|| error(model, "joint controller frames"))?;
        // Parsed XML and Axis::try_set_xyz already normalize upstream. Direct
        // public DOM assignment does not: admit the selected finite unit band
        // without repairing or mutating the caller's axis.
        let norm_squared = axis.xyz.into_iter().map(|value| value * value).sum::<f64>();
        require(
            !joint.name.is_empty()
                && !joint.name.contains("::")
                && joint.kind == JointKind::Revolute
                && joint.axis2.is_none()
                && model.links.iter().any(|link| link.name == joint.child)
                // Local joint frames reuse the same finite f64/f32 and unit-Q
                // rule as the existing scene state/pose boundary. Never repair
                // the public caller DOM or normalize its quaternion here.
                && crate::world_edit::representable(crate::WorldPose::from(joint.pose.transform))
                && (joint.pose.relative_to.is_empty() || joint.pose.relative_to == joint.child)
                && axis.xyz.into_iter().all(f64::is_finite)
                && norm_squared.is_finite()
                && (norm_squared - 1.0).abs() <= 1e-12
                && axis.expressed_in.is_empty(),
            &model.scoped_name,
            "joint controller frames",
        )?;
    }
    Ok(())
}

pub(super) fn parse(model: &Model) -> Result<Option<Vec<Parameters>>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(first) = plugins.next() else {
        return Ok(None);
    };
    let second = plugins.next();
    require(
        plugins.next().is_none(),
        &model.scoped_name,
        "joint controller count",
    )?;
    chain(model)?;
    let mut names = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut result = Vec::with_capacity(1 + usize::from(second.is_some()));
    for plugin in std::iter::once(first).chain(second) {
        let mode = if plugin.name == "gz::sim::systems::JointController" {
            JointControllerMode::Velocity
        } else {
            JointControllerMode::Position
        };
        let fields =
            fields(plugin, mode).map_err(|()| error(model, "joint controller configuration"))?;
        let name = fields
            .get("joint_name")
            .ok_or_else(|| error(model, "joint controller joint name"))?;
        let joint = model
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| error(model, "joint controller joint name"))?;
        require(
            names.insert(name.clone()),
            &model.scoped_name,
            "overlapping joint controllers",
        )?;
        let axis = joint
            .axis
            .as_ref()
            .expect("chain admission requires a revolute axis");
        require(
            [axis.effort, axis.velocity]
                .into_iter()
                .all(|value| value.is_finite() && value > 0.0),
            &model.scoped_name,
            "joint controller limits",
        )?;
        let (initial_target, max_velocity, suffix) = match mode {
            JointControllerMode::Velocity => {
                require(
                    fields
                        .get("use_force_commands")
                        .is_none_or(|value| matches!(value.as_str(), "false" | "0")),
                    &model.scoped_name,
                    "joint controller mode",
                )?;
                let initial = number(&fields, "initial_velocity", Some(0.0))
                    .map_err(|()| error(model, "initial joint controller target"))?;
                require(
                    initial.abs() <= axis.velocity,
                    &model.scoped_name,
                    "initial joint controller target",
                )?;
                (initial, axis.velocity, "cmd_vel")
            }
            JointControllerMode::Position => {
                require(
                    fields
                        .get("use_velocity_commands")
                        .is_some_and(|value| matches!(value.as_str(), "true" | "1"))
                        && fields
                            .get("joint_index")
                            .is_none_or(|value| value.parse::<u32>() == Ok(0)),
                    &model.scoped_name,
                    "joint controller mode",
                )?;
                let cap = number(&fields, "cmd_max", None)
                    .map_err(|()| error(model, "joint controller limits"))?;
                require(cap > 0.0, &model.scoped_name, "joint controller limits")?;
                let initial = number(&fields, "initial_position", Some(0.0))
                    .map_err(|()| error(model, "initial joint controller target"))?;
                (initial, cap.min(axis.velocity), "0/cmd_pos")
            }
        };
        let topic = fields
            .get("topic")
            .cloned()
            .unwrap_or_else(|| format!("/model/{}/joint/{name}/{suffix}", model.name));
        require(
            valid_topic(&topic) && topics.insert(topic.clone()),
            &model.scoped_name,
            "joint controller topics",
        )?;
        result.push(Parameters {
            joint: name.clone(),
            max_velocity,
            max_force: axis.effort,
            initial_target,
            publication: JointControllerConfiguration {
                model_name: model.name.clone(),
                joint_name: name.clone(),
                topic,
                mode,
            },
        });
    }
    Ok(Some(result))
}

#[cfg(test)]
#[path = "../../tests/private/joint_controller_config.rs"]
mod tests;

#[cfg(test)]
mod arbitrary_axis_dom_tests {
    use super::*;

    fn model() -> Model {
        sdformat::load_str(
            include_str!("../../../../examples/m9_joint_controllers.sdf"),
            "arbitrary-axis-dom.sdf",
            &sdformat::NoResources,
        )
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|m| m.name == "arm")
        .unwrap()
    }

    #[test]
    fn arbitrary_direct_dom_unit_directions_preserve_the_entire_caller_model() {
        for direction in [
            [0., 0.6, 0.8],
            [-0.6, 0.8, 0.],
            [1. / 3., -2. / 3., 2. / 3.],
            [1., 1e-12, 0.],
        ] {
            for joint in 0..2 {
                let mut caller = model();
                caller.joints[joint].axis.as_mut().unwrap().xyz = direction;
                let before = format!("{caller:?}");
                let bits = direction.map(f64::to_bits);
                let plans = parse(&caller).unwrap().unwrap();
                assert_eq!(plans.len(), 2);
                assert_eq!(format!("{caller:?}"), before);
                assert_eq!(
                    caller.joints[joint]
                        .axis
                        .as_ref()
                        .unwrap()
                        .xyz
                        .map(f64::to_bits),
                    bits
                );
            }
        }
    }

    #[test]
    fn arbitrary_direct_dom_nonunit_nonfinite_and_extreme_norms_are_not_repaired() {
        for direction in [
            [0., 3., 4.],
            [2., 0., 0.],
            [0.; 3],
            [f64::MIN_POSITIVE, 0., 0.],
            [1e308, 1e308, 0.],
            [f64::NAN, 0., 1.],
            [0., f64::INFINITY, 0.],
            [0., 0., f64::NEG_INFINITY],
        ] {
            for joint in 0..2 {
                let mut caller = model();
                caller.joints[joint].axis.as_mut().unwrap().xyz = direction;
                let before = format!("{caller:?}");
                let bits = direction.map(f64::to_bits);
                assert!(
                    matches!(
                        parse(&caller),
                        Err(LoadError::Unsupported {
                            feature: "joint controller frames",
                            ..
                        })
                    ),
                    "axis={direction:?}, joint={joint}"
                );
                assert_eq!(format!("{caller:?}"), before);
                assert_eq!(
                    caller.joints[joint]
                        .axis
                        .as_ref()
                        .unwrap()
                        .xyz
                        .map(f64::to_bits),
                    bits
                );
            }
        }
    }

    #[test]
    fn arbitrary_direct_dom_near_unit_band_checks_both_sides_without_normalizing() {
        for sign in [-1., 1.] {
            for (delta, admitted) in [
                (4e-13, true),
                (-4e-13, true),
                (6e-13, false),
                (-6e-13, false),
            ] {
                let direction = [sign * (1. + delta), 0., 0.];
                let norm_squared = direction.into_iter().map(|v| v * v).sum::<f64>();
                assert_eq!((norm_squared - 1.).abs() <= 1e-12, admitted);
                for joint in 0..2 {
                    let mut caller = model();
                    caller.joints[joint].axis.as_mut().unwrap().xyz = direction;
                    let before = format!("{caller:?}");
                    if admitted {
                        assert_eq!(parse(&caller).unwrap().unwrap().len(), 2);
                    } else {
                        assert!(matches!(
                            parse(&caller),
                            Err(LoadError::Unsupported {
                                feature: "joint controller frames",
                                ..
                            })
                        ));
                    }
                    assert_eq!(format!("{caller:?}"), before);
                    assert_eq!(
                        caller.joints[joint]
                            .axis
                            .as_ref()
                            .unwrap()
                            .xyz
                            .map(f64::to_bits),
                        direction.map(f64::to_bits)
                    );
                }
            }
        }
    }

    #[test]
    fn arbitrary_setter_normalization_is_explicit_and_failures_preserve_its_prior_value() {
        let mut caller = model();
        for joint in &mut caller.joints {
            joint
                .axis
                .as_mut()
                .unwrap()
                .try_set_xyz([0., 3., 4.])
                .unwrap();
        }
        assert!(
            caller
                .joints
                .iter()
                .all(|j| j.axis.as_ref().unwrap().xyz == [0., 0.6, 0.8])
        );
        let before = format!("{caller:?}");
        assert_eq!(parse(&caller).unwrap().unwrap().len(), 2);
        assert_eq!(format!("{caller:?}"), before);
        for invalid in [
            [0.; 3],
            [1e-300, 0., 0.],
            [f64::NAN, 0., 1.],
            [1e308, 1e308, 0.],
        ] {
            assert!(
                caller.joints[0]
                    .axis
                    .as_mut()
                    .unwrap()
                    .try_set_xyz(invalid)
                    .is_err()
            );
            assert_eq!(format!("{caller:?}"), before);
        }
        caller.joints[0].axis.as_mut().unwrap().xyz = [0., 3., 4.];
        assert!(matches!(
            parse(&caller),
            Err(LoadError::Unsupported {
                feature: "joint controller frames",
                ..
            })
        ));
    }
}

#[cfg(test)]
mod local_joint_frame_dom_tests {
    use super::*;

    fn model() -> Model {
        sdformat::load_str(
            include_str!("../../../../examples/m9_joint_controllers.sdf"),
            "local-joint-frame-dom.sdf",
            &sdformat::NoResources,
        )
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|m| m.name == "arm")
        .unwrap()
    }
    fn unchanged(caller: &Model, before: &str, bits: ([u64; 3], [u64; 4]), index: usize) {
        assert_eq!(format!("{caller:?}"), before);
        assert_eq!(
            caller.joints[index]
                .pose
                .transform
                .translation
                .map(f64::to_bits),
            bits.0
        );
        assert_eq!(
            caller.joints[index]
                .pose
                .transform
                .rotation
                .map(f64::to_bits),
            bits.1
        );
    }
    fn bits(caller: &Model, index: usize) -> ([u64; 3], [u64; 4]) {
        (
            caller.joints[index]
                .pose
                .transform
                .translation
                .map(f64::to_bits),
            caller.joints[index]
                .pose
                .transform
                .rotation
                .map(f64::to_bits),
        )
    }
    #[test]
    fn local_joint_dom_finite_translations_and_unit_frames_preserve_every_caller_bit() {
        let reference = parse(&model()).unwrap().unwrap();
        for translation in [
            [0.1, -0.25, 0.5],
            [-1., 2., -3.],
            [f64::from(f32::MAX), 0., 0.],
        ] {
            for rotation in [
                [0., 0., 1., 0.],
                [0.5, 0.5, 0.5, 0.5],
                [-0.5, 0.5, -0.5, 0.5],
                [0., 0., 0., -1.],
            ] {
                for index in 0..2 {
                    for child_relative in [false, true] {
                        let mut caller = model();
                        caller.joints[index].pose.transform = sdformat::Transform {
                            translation,
                            rotation,
                        };
                        caller.joints[index].pose.relative_to = if child_relative {
                            caller.joints[index].child.clone()
                        } else {
                            String::new()
                        };
                        let before = format!("{caller:?}");
                        let bits = bits(&caller, index);
                        let plans = parse(&caller).unwrap().unwrap();
                        assert_eq!(plans.len(), 2);
                        for (actual, wanted) in plans.iter().zip(&reference) {
                            assert_eq!(actual.publication, wanted.publication);
                            assert_eq!(
                                (
                                    actual.joint.as_str(),
                                    actual.max_velocity,
                                    actual.max_force,
                                    actual.initial_target
                                ),
                                (
                                    wanted.joint.as_str(),
                                    wanted.max_velocity,
                                    wanted.max_force,
                                    wanted.initial_target
                                )
                            );
                        }
                        unchanged(&caller, &before, bits, index);
                    }
                }
            }
        }
    }
    #[test]
    fn local_joint_dom_nonfinite_overflow_and_nonunit_frames_are_never_repaired() {
        let invalid = [
            ([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
            ([0., f64::INFINITY, 0.], [0., 0., 0., 1.]),
            ([0., 0., f64::NEG_INFINITY], [0., 0., 0., 1.]),
            ([f64::from(f32::MAX) * 2., 0., 0.], [0., 0., 0., 1.]),
            ([0.; 3], [0.; 4]),
            ([0.; 3], [0., 0., 0., 2.]),
            ([0.; 3], [f64::NAN, 0., 0., 1.]),
            ([0.; 3], [0., f64::INFINITY, 0., 1.]),
            ([0.; 3], [0., 0., f64::NEG_INFINITY, 1.]),
            ([0.; 3], [1e300, 0., 0., 1.]),
        ];
        for (translation, rotation) in invalid {
            for index in 0..2 {
                let mut caller = model();
                caller.joints[index].pose.transform = sdformat::Transform {
                    translation,
                    rotation,
                };
                let before = format!("{caller:?}");
                let bits = bits(&caller, index);
                assert!(
                    matches!(
                        parse(&caller),
                        Err(LoadError::Unsupported {
                            feature: "joint controller frames",
                            ..
                        })
                    ),
                    "joint={index}, translation={translation:?}, rotation={rotation:?}"
                );
                unchanged(&caller, &before, bits, index);
            }
        }
    }
    #[test]
    fn local_joint_dom_quaternion_unit_band_checks_both_sides_without_normalization() {
        for sign in [-1., 1.] {
            for (delta, admitted) in [
                (4e-13, true),
                (-4e-13, true),
                (6e-13, false),
                (-6e-13, false),
            ] {
                for index in 0..2 {
                    let mut caller = model();
                    caller.joints[index].pose.transform = sdformat::Transform {
                        translation: [0.25, -0.5, 1.],
                        rotation: [0., 0., 0., sign * (1. + delta)],
                    };
                    let q = caller.joints[index].pose.transform.rotation;
                    assert_eq!(
                        (q.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() <= 1e-12,
                        admitted
                    );
                    let before = format!("{caller:?}");
                    let bits = bits(&caller, index);
                    if admitted {
                        assert_eq!(parse(&caller).unwrap().unwrap().len(), 2);
                    } else {
                        assert!(matches!(
                            parse(&caller),
                            Err(LoadError::Unsupported {
                                feature: "joint controller frames",
                                ..
                            })
                        ));
                    }
                    unchanged(&caller, &before, bits, index);
                }
            }
        }
    }
    #[test]
    fn local_joint_dom_frame_admission_does_not_authorize_other_axis_link_or_frame_domains() {
        // Keep the exact former LINK-Q input, including both existing local
        // JOINT poses, as a positive without mutating caller bits or records.
        let mut caller = model();
        for joint in &mut caller.joints {
            joint.pose.transform = sdformat::Transform {
                translation: [0.25, -0.5, 1.],
                rotation: [0.5; 4],
            };
        }
        let reference = parse(&caller).unwrap().unwrap();
        caller.links[0].pose.transform.rotation = [0., 0., 1., 0.];
        let before = format!("{caller:?}");
        let plans = parse(&caller).unwrap().unwrap();
        assert_eq!(format!("{caller:?}"), before);
        assert_eq!(
            caller.links[0].pose.transform.rotation.map(f64::to_bits),
            [0.0_f64, 0.0, 1.0, 0.0].map(f64::to_bits)
        );
        assert_eq!(plans.len(), reference.len());
        for (actual, expected) in plans.iter().zip(&reference) {
            assert_eq!(actual.joint, expected.joint);
            assert_eq!(actual.publication, expected.publication);
            assert_eq!(
                (actual.max_velocity, actual.max_force, actual.initial_target),
                (
                    expected.max_velocity,
                    expected.max_force,
                    expected.initial_target
                )
            );
        }
        for mutate in [
            (|m: &mut Model| m.links[0].pose.relative_to = "arm".into()) as fn(&mut Model),
            |m| m.joints[0].pose.relative_to = m.joints[1].child.clone(),
            |m| m.joints[0].axis.as_mut().unwrap().expressed_in = "base".into(),
            |m| m.joints[0].axis.as_mut().unwrap().xyz = [2., 0., 0.],
            |m| m.joints[0].axis2 = m.joints[0].axis.clone(),
            |m| m.joints[0].kind = JointKind::Fixed,
        ] {
            let mut caller = model();
            for joint in &mut caller.joints {
                joint.pose.transform = sdformat::Transform {
                    translation: [0.25, -0.5, 1.],
                    rotation: [0.5; 4],
                };
            }
            mutate(&mut caller);
            let before = format!("{caller:?}");
            assert!(matches!(
                parse(&caller),
                Err(LoadError::Unsupported {
                    feature: "joint controller frames",
                    ..
                })
            ));
            assert_eq!(format!("{caller:?}"), before);
        }
    }
}

#[cfg(test)]
mod rotated_link_frame_dom_tests {
    use super::*;

    fn model() -> Model {
        sdformat::load_str(
            include_str!("../../../../examples/m9_joint_controllers.sdf"),
            "rotated-link-frame-dom.sdf",
            &sdformat::NoResources,
        )
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|m| m.name == "arm")
        .unwrap()
    }
    fn bits(caller: &Model, index: usize) -> ([u64; 3], [u64; 4]) {
        (
            caller.links[index]
                .pose
                .transform
                .translation
                .map(f64::to_bits),
            caller.links[index]
                .pose
                .transform
                .rotation
                .map(f64::to_bits),
        )
    }
    fn unchanged(caller: &Model, before: &str, original: ([u64; 3], [u64; 4]), index: usize) {
        assert_eq!(format!("{caller:?}"), before);
        assert_eq!(bits(caller, index), original);
    }
    #[test]
    fn rotated_link_dom_admits_finite_unit_frames_without_changing_caller_or_parameters() {
        let reference = parse(&model()).unwrap().unwrap();
        for translation in [
            [0.25, -0.5, 1.],
            [-1., 2., -3.],
            [f64::from(f32::MAX), 0., 0.],
        ] {
            for rotation in [
                [0., 0., 1., 0.],
                [0.5; 4],
                [-0.5, 0.5, -0.5, 0.5],
                [0., 0., 0., -1.],
            ] {
                for index in 0..2 {
                    let mut caller = model();
                    caller.links[index].pose.transform = sdformat::Transform {
                        translation,
                        rotation,
                    };
                    let before = format!("{caller:?}");
                    let original = bits(&caller, index);
                    let plans = parse(&caller).unwrap().unwrap();
                    assert_eq!(plans.len(), reference.len());
                    for (actual, expected) in plans.iter().zip(&reference) {
                        assert_eq!(actual.publication, expected.publication);
                        assert_eq!(
                            (
                                actual.joint.as_str(),
                                actual.max_velocity,
                                actual.max_force,
                                actual.initial_target
                            ),
                            (
                                expected.joint.as_str(),
                                expected.max_velocity,
                                expected.max_force,
                                expected.initial_target
                            )
                        );
                    }
                    unchanged(&caller, &before, original, index);
                }
            }
        }
        // The already admitted literal identity orientation still permits every
        // finite f64 LINK translation, including values not representable as f32.
        for translation in [[1e100, 0., 0.], [-f64::MAX, 0., 0.], [0., -0., 0.]] {
            for index in 0..2 {
                let mut caller = model();
                caller.links[index].pose.transform.translation = translation;
                let before = format!("{caller:?}");
                let original = bits(&caller, index);
                assert_eq!(parse(&caller).unwrap().unwrap().len(), 2);
                unchanged(&caller, &before, original, index);
            }
        }
    }
    #[test]
    fn rotated_link_dom_nonrepresentable_or_nonunit_frames_are_never_repaired() {
        for (translation, rotation) in [
            ([f64::NAN, 0., 0.], [0.5; 4]),
            ([0., f64::INFINITY, 0.], [0.5; 4]),
            ([0., 0., f64::NEG_INFINITY], [0.5; 4]),
            ([f64::from(f32::MAX) * 2., 0., 0.], [0.5; 4]),
            ([f64::from(f32::MAX) * 2., 0., 0.], [0., 0., 0., -1.]),
            ([0.; 3], [0.; 4]),
            ([0.; 3], [0., 0., 0., 2.]),
            ([0.; 3], [f64::NAN, 0., 0., 1.]),
            ([0.; 3], [0., f64::INFINITY, 0., 1.]),
            ([0.; 3], [0., 0., f64::NEG_INFINITY, 1.]),
            ([0.; 3], [1e300, 0., 0., 1.]),
        ] {
            for index in 0..2 {
                let mut caller = model();
                caller.links[index].pose.transform = sdformat::Transform {
                    translation,
                    rotation,
                };
                let before = format!("{caller:?}");
                let original = bits(&caller, index);
                assert!(
                    matches!(
                        parse(&caller),
                        Err(LoadError::Unsupported {
                            feature: "joint controller frames",
                            ..
                        })
                    ),
                    "link={index}, translation={translation:?}, rotation={rotation:?}"
                );
                unchanged(&caller, &before, original, index);
            }
        }
    }
    #[test]
    fn rotated_link_dom_unit_band_is_closed_on_both_sides_without_normalization() {
        for sign in [-1., 1.] {
            for (delta, admitted) in [
                (4e-13, true),
                (-4e-13, true),
                (6e-13, false),
                (-6e-13, false),
            ] {
                for index in 0..2 {
                    let mut caller = model();
                    let rotation = [0., 0., 0., sign * (1. + delta)];
                    caller.links[index].pose.transform = sdformat::Transform {
                        translation: [0.25, -0.5, 1.],
                        rotation,
                    };
                    assert_eq!(
                        (rotation.into_iter().map(|v| v * v).sum::<f64>() - 1.).abs() <= 1e-12,
                        admitted
                    );
                    let before = format!("{caller:?}");
                    let original = bits(&caller, index);
                    if admitted {
                        assert_eq!(parse(&caller).unwrap().unwrap().len(), 2);
                    } else {
                        assert!(matches!(
                            parse(&caller),
                            Err(LoadError::Unsupported {
                                feature: "joint controller frames",
                                ..
                            })
                        ));
                    }
                    unchanged(&caller, &before, original, index);
                }
            }
        }
    }
    #[test]
    fn rotated_link_dom_does_not_expand_relative_frames_axes_topology_or_joint_domains() {
        for mutate in [
            (|m: &mut Model| m.links[0].pose.relative_to = "arm".into()) as fn(&mut Model),
            |m| m.links[1].pose.relative_to = "base".into(),
            |m| m.joints[0].pose.relative_to = m.joints[1].child.clone(),
            |m| m.joints[0].pose.transform.rotation = [0.; 4],
            |m| m.joints[0].axis.as_mut().unwrap().xyz = [2., 0., 0.],
            |m| m.joints[0].axis.as_mut().unwrap().expressed_in = "base".into(),
            |m| m.joints[0].axis2 = m.joints[0].axis.clone(),
            |m| m.joints[0].kind = JointKind::Fixed,
        ] {
            let mut caller = model();
            for link in &mut caller.links {
                link.pose.transform.rotation = [0.5; 4];
            }
            mutate(&mut caller);
            let before = format!("{caller:?}");
            assert!(matches!(
                parse(&caller),
                Err(LoadError::Unsupported {
                    feature: "joint controller frames",
                    ..
                })
            ));
            assert_eq!(format!("{caller:?}"), before);
        }
        for index in 0..2 {
            let mut caller = model();
            caller.links[index].pose.transform.translation[0] = f64::INFINITY;
            assert!(matches!(
                parse(&caller),
                Err(LoadError::Unsupported {
                    feature: "joint controller frames",
                    ..
                })
            ));
        }
    }
}
