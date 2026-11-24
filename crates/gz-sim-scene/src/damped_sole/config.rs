// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

pub(crate) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::DampedSole" && p.filename == "gz-sim-damped-sole-system"
}

/// This model contributes only a fixed observation hierarchy, so it cannot
/// introduce any rigid contact or external joint across the coating boundary.
pub(crate) fn observer_model(model: &Model) -> bool {
    model.is_static
        && model.joints.is_empty()
        && model.models.is_empty()
        && model.plugins.is_empty()
        && model.links.iter().all(|link| link.collisions.is_empty())
}

pub(crate) struct Plan {
    pub(super) world: String,
    pub(super) foot: String,
    pub(super) supports: Vec<(String, DVec3, SoleMaterial)>,
    pub(super) body: SoleBody,
    pub(super) upper: Option<(Option<String>, GuardBox)>,
    pub(super) side: Option<(String, f64)>,
    pub(super) ankle: bool,
    pub(super) substeps: u32,
    pub(super) depth: u8,
    pub(super) flat_elastic_fast_path: bool,
    pub(super) flat_damped_fast_path: bool,
    pub(super) accuracy: Option<SoleAccuracy>,
    pub(super) panel_work_tolerance: Option<f64>,
}

pub(crate) fn parse(
    sdf: &Sdf,
    world: &sdformat::World,
    engine: PhysicsEngine,
) -> Result<Option<Plan>, LoadError> {
    let err = |reason: &str| invalid(&world.name, reason);
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(p) = plugins.next() else {
        return Ok(None);
    };
    if plugins.next().is_some() || engine != PhysicsEngine::MultiFidelity {
        return Err(err(
            "use one DampedSole plugin with local multifidelity physics",
        ));
    }
    if world.plugins.iter().any(|p| {
        free_rotation::is_plugin(p)
            || primary_contact::sdf::is_plugin(p)
            || deformable::is_plugin(p)
    }) {
        return Err(err(
            "requires an isolated sole and support with exclusive physics ownership",
        ));
    }
    let xml = roxmltree::Document::parse(&p.xml).map_err(|e| err(&e.to_string()))?;
    let root = xml.root_element();
    if !root.has_tag_name("plugin")
        || root.attributes().len() != 2
        || root.namespaces().next().is_some()
    {
        return Err(err("invalid plugin attributes"));
    }
    let mut fields = BTreeMap::new();
    let mut names = BTreeMap::new();
    for n in root.children() {
        if n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty() {
            continue;
        }
        let name = n.tag_name().name();
        let mut children = n.children();
        let text = children
            .next()
            .filter(|v| v.is_text())
            .and_then(|v| v.text())
            .unwrap_or_default()
            .trim();
        if !n.is_element()
            || n.namespaces().next().is_some()
            || n.attributes().any(|a| {
                name != "support"
                    || !matches!(
                        a.name(),
                        "deck_stiffness" | "damping" | "friction" | "slip_speed"
                    )
                    || a.namespace().is_some()
            })
            || children.next().is_some()
            || text.is_empty()
        {
            return Err(err("expected nonempty scalar fields"));
        }
        if name == "support" {
            if names.insert(text, n).is_some() {
                return Err(err("duplicate support link"));
            }
        } else if fields.insert(name, text).is_some() {
            return Err(err("duplicate scalar field"));
        }
    }
    let adaptive = fields.contains_key("max_substeps");
    let mut take = |name| {
        fields
            .remove(name)
            .ok_or_else(|| err(&format!("missing {name}")))
    };
    let foot = take("foot")?;
    let physical = world.models.iter().filter(|m| !observer_model(m)).count();
    if names.is_empty() || physical != names.len() + 1 {
        return Err(err(
            "enroll each physical model as the foot or a support. Other models must be static collisionless observers",
        ));
    }
    if names.contains_key(foot) {
        return Err(err("foot and support must be distinct"));
    }
    let foot_k = take("foot_stiffness")?;
    let deck_k = take("deck_stiffness")?;
    let damping = take("damping")?;
    let mu = take("friction")?;
    let speed = take("slip_speed")?;
    let number = |text: &str| {
        text.parse::<f64>()
            .map_err(|_| err("invalid material parameter"))
    };
    let material = |stiffness, damping, mu, speed| {
        SoleMaterial::new(
            number(foot_k)?,
            number(stiffness)?,
            number(damping)?,
            number(mu)?,
            number(speed)?,
        )
        .map_err(|e| err(&e.to_string()))
    };
    let default_material = material(deck_k, damping, mu, speed)?;
    let substeps = take("substeps")?
        .parse::<u32>()
        .ok()
        .filter(|n| *n > 0)
        .ok_or_else(|| err("substeps must be positive"))?;
    let depth = take("quadrature_depth")?
        .parse::<u8>()
        .ok()
        .filter(|n| *n <= 5)
        .ok_or_else(|| err("quadrature_depth must be 0 through 5"))?;
    let accuracy = if adaptive {
        let accuracy = SoleAccuracy {
            min_substeps: substeps,
            max_substeps: take("max_substeps")?
                .parse()
                .map_err(|_| err("invalid max_substeps"))?,
            min_depth: depth,
            max_depth: take("max_quadrature_depth")?
                .parse()
                .map_err(|_| err("invalid max_quadrature_depth"))?,
            position_m: number(take("position_tolerance")?)?,
            rotation_rad: number(take("rotation_tolerance")?)?,
            impulse_ns: number(take("impulse_tolerance")?)?,
            angular_impulse_nms: number(take("angular_impulse_tolerance")?)?,
            heat_j: number(take("heat_tolerance")?)?,
        };
        if !accuracy.is_valid() {
            return Err(err("invalid adaptive tolerances or grid bounds"));
        }
        Some(accuracy)
    } else {
        None
    };
    let panel_work_tolerance = match (accuracy, fields.remove("work_tolerance")) {
        (Some(_), Some(value)) => {
            let value = number(value)?;
            if !value.is_finite() || value < 0. {
                return Err(err("work_tolerance must be finite and nonnegative"));
            }
            Some(value)
        }
        (Some(policy), None) if names.len() > 1 => Some(policy.heat_j),
        (None, Some(_)) => return Err(err("work_tolerance requires an adaptive grid")),
        _ => None,
    };
    let mut enabled = |name: &str| match fields.remove(name) {
        None | Some("false") => Ok(false),
        Some("true") => Ok(true),
        _ => Err(err(&format!("{name} must be true or false"))),
    };
    let ankle = enabled("native_ankle")?;
    let flat_elastic_fast_path = enabled("flat_elastic_fast_path")?;
    let flat_damped_fast_path = enabled("flat_damped_fast_path")?;
    let side = match (
        fields.remove("upper_negative_x_support"),
        fields.remove("upper_corner_stiffness"),
    ) {
        (None, None) => None,
        (Some(name), Some(stiffness)) => {
            let k = number(stiffness)?;
            if !names.contains_key(name)
                || !k.is_finite()
                || k <= 0.
                || adaptive
                || flat_elastic_fast_path
                || flat_damped_fast_path
            {
                return Err(err(
                    "upper contact requires an enrolled panel, positive corner stiffness and a fixed contact grid",
                ));
            }
            Some((name.to_owned(), k))
        }
        _ => {
            return Err(err(
                "provide both upper_negative_x_support and upper_corner_stiffness",
            ));
        }
    };
    if !fields.is_empty() {
        return Err(err("unknown field"));
    }
    let resolve = |name: &str| {
        world
            .models
            .iter()
            .find_map(|m| {
                let [link] = m.links.as_slice() else {
                    return None;
                };
                if link.scoped_name != name || !m.joints.is_empty() || !m.models.is_empty() {
                    return None;
                }
                let [collision] = link.collisions.as_slice() else {
                    return None;
                };
                let geometry_pose = sdf
                    .resolve_pose(&collision.pose, &m.scoped_name, name, name)
                    .ok()?;
                if !identity(geometry_pose) {
                    return None;
                }
                let Geometry::Box { size } = collision.geometry else {
                    return None;
                };
                Some((m, link, DVec3::from(size) * 0.5))
            })
            .ok_or_else(|| err(&format!("{name} must be a free top-level single box link")))
    };
    let (model, link) = world
        .models
        .iter()
        .find_map(|m| {
            m.links
                .iter()
                .find(|l| l.scoped_name == foot)
                .map(|l| (m, l))
        })
        .ok_or_else(|| err("missing sole foot link"))?;
    let upper_box =
        |link: &sdformat::Link, c: &sdformat::Collision| -> Result<GuardBox, LoadError> {
            let Geometry::Box { size } = c.geometry else {
                return Err(err("coated foot requires box colliders"));
            };
            let pose = sdf.resolve_pose(&c.pose, &model.scoped_name, &link.scoped_name, foot)?;
            Ok(GuardBox {
                center: pose.translation.into(),
                rotation: DQuat::from_array(pose.rotation),
                half: DVec3::from(size) * 0.5,
            })
        };
    let centered = |g: &GuardBox| {
        identity(Transform {
            translation: g.center.into(),
            rotation: g.rotation.to_array(),
        })
    };
    if ankle
        && (side.is_some()
            || adaptive
            || flat_elastic_fast_path
            || flat_damped_fast_path
            || names.len() != 1
            || !matches!(model.joints.as_slice(), [j] if j.parent == link.name && matches!(j.kind,sdformat::JointKind::Fixed|sdformat::JointKind::Revolute)))
    {
        return Err(err(
            "native_ankle requires a lower-rooted revolute or fixed sole, one static deck and a fixed material grid",
        ));
    }
    let (lower, upper) = match (
        model.links.as_slice(),
        model.joints.as_slice(),
        link.collisions.as_slice(),
    ) {
        ([_], [], [sole]) => (upper_box(link, sole)?, None),
        ([_], [], [a, b]) => {
            let (a, b) = (upper_box(link, a)?, upper_box(link, b)?);
            match (centered(&a), centered(&b)) {
                (true, false) => (a, Some((None, b))),
                (false, true) => (b, Some((None, a))),
                _ => {
                    return Err(err(
                        "two-box compound needs exactly one collision centered on the sole link",
                    ));
                }
            }
        }
        ([a, b], [joint], [sole]) if joint.kind == sdformat::JointKind::Fixed || ankle => {
            let other = if a.scoped_name == foot { b } else { a };
            let [c] = other.collisions.as_slice() else {
                return Err(err("welded foot expects one box on each link"));
            };
            (
                upper_box(link, sole)?,
                Some((Some(other.scoped_name.clone()), upper_box(other, c)?)),
            )
        }
        _ => {
            return Err(err(
                "coated foot requires one free box or a two-box compound or floating weld",
            ));
        }
    };
    if side.is_some() && upper.is_none() {
        return Err(err("upper side contact requires a second collision box"));
    }
    if !centered(&lower) || !model.models.is_empty() {
        return Err(err("sole collision must be centered on its link"));
    }
    let half_extents = lower.half;
    if model.is_static
        || model
            .plugins
            .iter()
            .any(|p| !prescribed_motion::is_initial_velocity(p))
    {
        return Err(err(
            "foot must be a dynamic box with optional InitialVelocity",
        ));
    }
    let mut supports = Vec::with_capacity(names.len());
    let single_support = names.len() == 1;
    for (support, node) in names {
        let (deck, _, deck_half) = resolve(support)?;
        if ankle && (!deck.is_static || !deck.plugins.is_empty()) {
            return Err(err("native_ankle requires a static deck"));
        }
        if deck
            .plugins
            .iter()
            .any(|p| !prescribed_motion::is_plugin(p))
        {
            return Err(err(
                "support requires a fixed box or prescribed twist; rotation requires a single support",
            ));
        }
        let motion = prescribed_motion::parse(deck, engine, true)?;
        if !deck.is_static && motion.is_none()
            || motion.is_some_and(|m| {
                m.path.is_some()
                    || ((!single_support || side.is_some()) && m.twist.angular != [0.; 3])
            })
        {
            return Err(err(
                "support requires a fixed box or prescribed twist; rotation requires a single support",
            ));
        }
        supports.push((
            support.to_owned(),
            deck_half,
            if node.attributes().len() == 0 {
                default_material
            } else {
                material(
                    node.attribute("deck_stiffness").unwrap_or(deck_k),
                    node.attribute("damping").unwrap_or(damping),
                    node.attribute("friction").unwrap_or(mu),
                    node.attribute("slip_speed").unwrap_or(speed),
                )?
            },
        ));
    }
    let inertial = sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, foot, foot)?;
    let [ix, iy, iz, ixy, ixz, iyz] = link.inertial.inertia;
    if !matches!(upper, Some((Some(_), _)))
        && (!identity(Transform {
            translation: [0.; 3],
            ..inertial
        }) || [ixy, ixz, iyz] != [0.; 3])
    {
        return Err(err("sole inertia must be diagonal in link axes"));
    }
    Ok(Some(Plan {
        world: world.name.clone(),
        foot: foot.into(),
        upper,
        side,
        ankle,
        supports,
        body: SoleBody {
            half_extents,
            foot_offset: -DVec3::from(inertial.translation),
            mass: link.inertial.mass,
            inertia: DVec3::new(ix, iy, iz),
        },
        substeps,
        depth,
        accuracy,
        panel_work_tolerance,
        flat_elastic_fast_path,
        flat_damped_fast_path,
    }))
}
