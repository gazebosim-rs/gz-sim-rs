// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_xpbd::{PadSample, Particle, SolverConfig};
use roxmltree::Node;
use std::str::FromStr;

pub(crate) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::Deformables" && plugin.filename == "gz-sim-deformables-system"
}

fn fields(node: Node<'_, '_>, allowed: &[&str], container: bool) -> Result<(), String> {
    if node.namespaces().next().is_some()
        || node.attributes().any(|a| !allowed.contains(&a.name()))
        || node.children().any(|n| {
            !(n.is_comment()
                || n.is_text() && n.text().unwrap_or("").trim().is_empty()
                || container && n.is_element())
        })
    {
        return Err(format!(
            "unexpected content in <{}>",
            node.tag_name().name()
        ));
    }
    Ok(())
}

fn attribute<'a>(node: Node<'a, '_>, name: &str) -> Result<&'a str, String> {
    node.attribute(name)
        .ok_or_else(|| format!("missing {name} on <{}>", node.tag_name().name()))
}

fn number<T: FromStr>(node: Node<'_, '_>, name: &str) -> Result<T, String> {
    attribute(node, name)?
        .parse()
        .map_err(|_| format!("invalid {name} on <{}>", node.tag_name().name()))
}

fn vector(value: &str) -> Result<[f64; 3], String> {
    let mut tokens = value.split_whitespace();
    let mut values = [0.0_f64; 3];
    for v in &mut values {
        *v = tokens
            .next()
            .and_then(|t| t.parse().ok())
            .filter(|f: &f64| f.is_finite())
            .ok_or_else(|| format!("expected three finite coordinates in '{value}'"))?;
    }
    if tokens.next().is_some() {
        return Err(format!("too many coordinates in '{value}'"));
    }
    Ok(values)
}

fn pad_sample(node: Node<'_, '_>) -> Result<PadSample, String> {
    fields(node, &["particles", "weights", "stiffness_weight"], false)?;
    let mut ids = attribute(node, "particles")?.split_whitespace();
    let mut particles = [0; 3];
    for id in &mut particles {
        *id = ids
            .next()
            .and_then(|v| v.parse().ok())
            .ok_or("pad_sample requires three unsigned particle indices")?;
    }
    if ids.next().is_some() {
        return Err("pad_sample requires exactly three particle indices".into());
    }
    Ok(PadSample {
        particles,
        weights: vector(attribute(node, "weights")?)?,
        stiffness_weight: number(node, "stiffness_weight")?,
    })
}

mod cloth_laws;

fn soft_body(node: Node<'_, '_>, gravity: [f64; 3], dt: Duration) -> Result<SoftBody, String> {
    let cloth = node.has_tag_name("cloth");
    let common = [
        "name",
        "substeps",
        "conserve_angular_momentum",
        "capsule_contact",
        "iterations",
        "radius",
        "self_contact",
        "pad_loads",
        "friction",
        "stretch",
        "bend",
    ];
    let mut allowed = common.to_vec();
    if cloth {
        allowed.extend([
            "columns",
            "shear",
            "pad_contact",
            "self_friction",
            "self_contact_loads",
            "capsule_contact_history",
        ]);
    }
    fields(node, &allowed, true)?;
    let config = SolverConfig {
        gravity,
        substeps: number(node, "substeps")?,
        iterations: number(node, "iterations")?,
        particle_radius: number(node, "radius")?,
        self_contact: number(node, "self_contact")?,
        friction: number(node, "friction")?,
    };
    if u128::from(config.substeps) > dt.as_nanos() {
        return Err("soft substeps exceed public step in nanoseconds".into());
    }
    let is_sample = |n: Node<'_, '_>| cloth && n.has_tag_name("pad_sample");
    let particles = node
        .children()
        .filter(|n| n.is_element() && !is_sample(*n) && !(cloth && cloth_laws::is_law(*n)))
        .map(|n| {
            if !n.has_tag_name("particle") {
                return Err(format!("unexpected <{}>", n.tag_name().name()));
            }
            fields(n, &["position", "mass", "velocity", "pinned"], false)?;
            let mass: f64 = number(n, "mass")?;
            if !mass.is_finite() || mass <= 0.0 || !mass.recip().is_finite() {
                return Err("invalid particle mass".into());
            }
            let pinned = n
                .attribute("pinned")
                .unwrap_or("false")
                .parse::<bool>()
                .map_err(|_| "invalid pinned flag")?;
            Ok(Particle {
                position: vector(attribute(n, "position")?)?,
                velocity: vector(n.attribute("velocity").unwrap_or("0 0 0"))?,
                inverse_mass: if pinned { 0.0 } else { mass.recip() },
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let stretch = number(node, "stretch")?;
    let bend = number(node, "bend")?;
    let body = if cloth {
        cloth_laws::build(
            node,
            particles,
            number(node, "columns")?,
            stretch,
            number(node, "shear")?,
            bend,
            config,
        )?
    } else {
        SoftBody::cable(particles, stretch, bend, config).map_err(|e| e.to_string())?
    };
    let samples = node
        .children()
        .filter(|n| is_sample(*n))
        .map(pad_sample)
        .collect::<Result<Vec<_>, _>>()?;
    let body = if samples.is_empty() {
        body
    } else {
        body.with_pad_samples(samples)
            .map_err(|e| format!("invalid pad_sample: {e}"))?
    };
    let body = match node.attribute("pad_loads") {
        None | Some("false") => body,
        Some("true") => body.with_pad_loads(),
        Some(_) => return Err("pad_loads must be true or false".into()),
    };
    let body = match node.attribute("conserve_angular_momentum") {
        None | Some("false") => body,
        Some("true") => body.with_angular_momentum_conservation(),
        Some(_) => return Err("conserve_angular_momentum must be true or false".into()),
    };
    let body = match node.attribute("capsule_contact"){
        None | Some("particles")=>body,
        Some("material")=>body.with_capsule_material_contact().map_err(|e|format!("material capsule contact requires a frictionless cable or a cloth with positive skin: {e}"))?,
        Some(_)=>return Err("capsule_contact must be particles or material".into()),
    };
    let body = match node.attribute("capsule_contact_history") {
        None | Some("false") => body,
        Some("true") => body.with_capsule_contact_history().map_err(|e| {
            format!("capsule contact history requires frictional material cloth: {e}")
        })?,
        _ => return Err("capsule_contact_history must be true or false".into()),
    };
    let body = if node.attribute("self_friction").is_some() {
        body.with_self_contact_friction(number(node, "self_friction")?)
            .map_err(|e| e.to_string())?
    } else {
        body
    };
    let body = match node.attribute("self_contact_loads") {
        None | Some("false") => body,
        Some("true") => body
            .with_self_contact_loads()
            .map_err(|_| "self_contact_loads requires a self-friction surface")?,
        Some(_) => return Err("self_contact_loads must be true or false".into()),
    };
    match node.attribute("pad_contact") {
        None | Some("particles") => Ok(body),
        Some("surface") => body.with_pad_surface_contact().map_err(|e| e.to_string()),
        Some(_) => Err("pad_contact must be particles or surface".into()),
    }
}

fn find_link<'a>(
    models: &'a [sdformat::Model],
    name: &str,
    ancestor_static: bool,
) -> Option<(&'a sdformat::Link, bool, bool)> {
    models.iter().find_map(|m| {
        let fixed = ancestor_static || m.is_static;
        m.links
            .iter()
            .find(|l| l.scoped_name == name)
            .map(|l| (l, fixed, m.plugins.iter().any(prescribed_motion::is_plugin)))
            .or_else(|| find_link(&m.models, name, fixed))
    })
}

fn rigid_surface(
    node: Node<'_, '_>,
    world: &sdformat::World,
    links: &BTreeMap<String, u64>,
    ecs: &World,
    physics: &PhysicsBackend,
    capsule_pair: &[(BodyId, SoftRigidInertia)],
) -> Result<Surface, String> {
    fields(
        node,
        &["link", "coupling", "linear_trajectory", "normal_compliance"],
        false,
    )?;
    let normal_compliance = match node.attribute("normal_compliance") {
        None => 0.,
        Some(_)
            if node.has_tag_name("box")
                && node.attribute("coupling") == Some("articulated_wrench") =>
        {
            let compliance = number::<f64>(node, "normal_compliance")?;
            if !compliance.is_finite() || compliance < 0. {
                return Err("normal_compliance must be finite and nonnegative (m/N)".into());
            }
            compliance
        }
        Some(_) => return Err("normal_compliance requires an articulated_wrench box".into()),
    };
    let forced = match node.attribute("linear_trajectory") {
        None | Some("native_chord") => false,
        Some("force_aware")
            if node.has_tag_name("capsule") && node.attribute("coupling") == Some("two_way") =>
        {
            true
        }
        _ => return Err("linear_trajectory=force_aware requires a finite two-way capsule".into()),
    };
    let name = attribute(node, "link")?;
    let (sdf, fixed, prescribed) = find_link(&world.models, name, false)
        .ok_or_else(|| format!("unknown deformable boundary link '{name}'"))?;
    let prescribed_or_static = fixed || prescribed;
    let [collision] = sdf.collisions.as_slice() else {
        return Err(format!("expected one collision on '{name}'"));
    };
    let shape = match (&collision.geometry, node.tag_name().name()) {
        (Geometry::Sphere { radius }, "sphere") => ReceiverShape::Sphere(*radius),
        (Geometry::Capsule { radius, length }, "capsule") => ReceiverShape::Capsule {
            radius: *radius,
            half_length: 0.5 * length,
        },
        (Geometry::Box { size }, "box") => ReceiverShape::Cuboid(size.map(|x| x * 0.5)),
        _ => return Err(format!("expected matching centered geometry on '{name}'")),
    };
    let link = *links
        .get(name)
        .ok_or("missing loaded rigid boundary identity")?;
    let entity = entity_by_id(ecs, link).ok_or("missing rigid boundary entity")?;
    let collision_pose = ecs
        .get::<Children>(entity)
        .and_then(|children| {
            children
                .iter()
                .find(|&e| ecs.get::<EntityKind>(e) == Some(&EntityKind::Collision))
                .and_then(|e| ecs.get::<LocalPose>(e))
                .map(|p| Transform {
                    translation: p.0.translation,
                    rotation: p.0.rotation,
                })
        })
        .ok_or("missing loaded boundary collision pose")?;
    let static_box = matches!(shape, ReceiverShape::Cuboid(_))
        && node.attribute("coupling") == Some("one_way")
        && fixed
        && !prescribed;
    if !static_box && !identity(collision_pose) {
        return Err(format!("expected centered geometry on '{name}'"));
    }
    let body = ecs
        .get::<BodyHandle>(entity)
        .ok_or("missing rigid body")?
        .id();
    let mobility = match attribute(node, "coupling")? {
        "one_way" if static_box || matches!(shape, ReceiverShape::Sphere(_)) => None,
        "one_way" if matches!(shape, ReceiverShape::Capsule { .. }) => {
            if !prescribed_or_static
                || ecs
                    .get::<LinkCenterOfMass>(entity)
                    .is_some_and(|com| !centered(com.0))
            {
                return Err(format!(
                    "one-way capsule '{name}' requires a prescribed or static centered link"
                ));
            }
            capsule_axis(
                physics
                    .state(body)
                    .ok_or("missing capsule construction state")?,
            )
            .map_err(|e| format!("{name}: {e}"))?;
            None
        }
        "one_way" => {
            return Err("one-way cuboid boundaries require a static SDF link".into());
        }
        "articulated_wrench" => {
            if !matches!(shape, ReceiverShape::Cuboid(_))
                || prescribed_or_static
                || physics.is_jointed(body) != Some(true)
                || physics.is_dynamic(body) != Some(true)
                || ecs
                    .get::<LinkCenterOfMass>(entity)
                    .is_some_and(|com| !centered(com.0))
            {
                return Err(format!(
                    "articulated_wrench requires a dynamic jointed cuboid with centered mass on '{name}'"
                ));
            }
            Some(ReceiverMass::ArticulatedWrench { normal_compliance })
        }
        "two_way" => {
            let backend = physics
                .downcast_ref::<MultiFidelityBackend>()
                .expect("validated engine");
            Some(match shape {
                ReceiverShape::Capsule { .. } => {
                    let mass = match capsule_pair.iter().find(|(id, _)| *id == body) {
                        Some((_, mass)) => *mass,
                        None => backend
                            .soft_capsule_inertia(body)
                            .map_err(|e| format!("{name}: {e}"))?,
                    };
                    if forced {
                        ReceiverMass::ForceAware(mass)
                    } else {
                        ReceiverMass::Principal(mass)
                    }
                }
                ReceiverShape::Sphere(_) => ReceiverMass::from_native(
                    backend
                        .soft_sphere_inertia(body)
                        .map_err(|e| format!("{name}: {e}"))?,
                ),
                ReceiverShape::Cuboid(_) => ReceiverMass::Principal(
                    backend
                        .soft_cuboid_inertia(body)
                        .map_err(|e| format!("{name}: {e}"))?,
                ),
            })
        }
        mode => return Err(format!("unknown coupling mode '{mode}'")),
    };
    Ok(Surface::Rigid {
        link,
        body,
        shape,
        mobility,
        static_box_pose: static_box.then_some(collision_pose),
    })
}

fn parse(
    world: &sdformat::World,
    xml: &str,
    links: &BTreeMap<String, u64>,
    ecs: &World,
    physics: &PhysicsBackend,
    dt: Duration,
) -> Result<Deformables, String> {
    let doc = roxmltree::Document::parse(xml).map_err(|e| e.to_string())?;
    let root = doc.root_element();
    fields(
        root,
        &[
            "name",
            "filename",
            "worker_threads",
            "worker_processes",
            "worker_transport",
            "worker_warm_budget_ms",
            "worker_packed_replies",
        ],
        true,
    )?;
    let packed_replies = match root.attribute("worker_packed_replies") {
        None | Some("false") => false,
        Some("true") => true,
        _ => return Err("worker_packed_replies must be true or false".into()),
    };
    #[cfg(not(feature = "distributed-physics"))]
    let _ = packed_replies;
    let warm_ms = root
        .attribute("worker_warm_budget_ms")
        .map(str::parse::<u64>)
        .transpose()
        .map_err(|_| "invalid warm worker budget")?;
    if warm_ms.is_some_and(|n| !(10..=6000).contains(&n)) {
        return Err("worker_warm_budget_ms must be between 10 and 6000".into());
    }
    let rpc = root.attribute("worker_transport").unwrap_or("pipe");
    if !matches!(rpc, "pipe" | "loopback_zenoh") {
        return Err("worker_transport must be pipe or loopback_zenoh".into());
    }
    let processes = root
        .attribute("worker_processes")
        .unwrap_or("1")
        .parse::<u8>()
        .map_err(|_| "invalid material worker process count")?;
    if !(1..=4).contains(&processes) {
        return Err("material worker_processes must be between one and four".into());
    }
    let threads = root
        .attribute("worker_threads")
        .unwrap_or("1")
        .parse::<u8>()
        .map_err(|_| "invalid material worker thread budget")?;
    if !(1..=4).contains(&threads) {
        return Err("material worker_threads must be between one and four".into());
    }
    let mut surfaces = Vec::new();
    let mut visuals = Vec::new();
    let mut visual_ids = std::collections::BTreeSet::new();
    let mut impact_barriers = BTreeMap::new();
    let mut automatic_impact = None;
    let mut initial = BTreeMap::new();
    let mut rigid_links = std::collections::BTreeSet::new();
    let native_capsules = root
        .children()
        .filter(|n| n.has_tag_name("capsule") && n.attribute("coupling") == Some("two_way"))
        .collect::<Vec<_>>();
    let mut capsule_pair = Vec::new();
    if let [a, b] = native_capsules.as_slice() {
        let resolve = |node: Node<'_, '_>| -> Result<BodyId, String> {
            let name = attribute(node, "link")?;
            let id = links
                .get(name)
                .ok_or_else(|| format!("unknown capsule boundary '{name}'"))?;
            let e = entity_by_id(ecs, *id).ok_or("missing capsule link")?;
            Ok(ecs
                .get::<BodyHandle>(e)
                .ok_or("missing native capsule body")?
                .id())
        };
        let ids = [resolve(*a)?, resolve(*b)?];
        let mass = physics
            .downcast_ref::<MultiFidelityBackend>()
            .expect("validated engine")
            .soft_capsule_pair_inertia(ids)
            .map_err(|e| e.to_string())?;
        capsule_pair.extend(ids.into_iter().zip(mass));
    }
    for node in root.children().filter(Node::is_element) {
        match node.tag_name().name() {
            "cable" | "cloth" => {
                let name = attribute(node, "name")?;
                if name.trim().is_empty() || initial.contains_key(name) {
                    return Err(format!("empty or duplicate soft body name '{name}'"));
                }
                let body =
                    soft_body(node, world.gravity, dt).map_err(|e| format!("{name}: {e}"))?;
                initial.insert(name.to_string(), body);
            }
            "particle_visual" => {
                fields(node, &["material", "particle", "visual"], false)?;
                let material = attribute(node, "material")?.to_string();
                let particle = number(node, "particle")?;
                let name = attribute(node, "visual")?;
                let mut matches = ecs.iter_entities().filter(|e| {
                    e.get::<EntityKind>() == Some(&EntityKind::Visual)
                        && e.get::<Name>().is_some_and(|n| n.0 == name)
                });
                let target = matches
                    .next()
                    .ok_or_else(|| format!("missing particle visual '{name}'"))?;
                let id = target
                    .get::<EntityId>()
                    .ok_or("visual has no public identity")?
                    .get();
                if matches.next().is_some() || !visual_ids.insert(id) {
                    return Err(format!("duplicate particle visual '{name}'"));
                }
                let (link, leaf) = name
                    .rsplit_once("::")
                    .ok_or("particle visual requires a qualified link and visual")?;
                let sphere = find_link(&world.models, link, false)
                    .and_then(|(l, _, _)| l.visuals.iter().find(|v| v.name == leaf))
                    .is_some_and(|v| matches!(v.geometry, Some(Geometry::Sphere { .. })));
                if !sphere {
                    return Err("a particle visual requires an authored sphere".into());
                }
                visuals.push(particle_visual::Binding {
                    material,
                    particle,
                    visual: id,
                });
            }
            "articulated_impact" => {
                fields(
                    node,
                    &[
                        "material",
                        "time_ns",
                        "detection",
                        "restitution",
                        "max_contact_step_ns",
                        "max_slip_transition_step_ns",
                    ],
                    true,
                )?;
                let time =
                    match (node.attribute("time_ns"), node.attribute("detection")) {
                        (None,Some("first_impact"|"recurrent_contact"|"persistent_contact"))=>None,
                        (Some(_), None) => {
                            let time = Duration::from_nanos(number(node, "time_ns")?);
                            if time.is_zero() || !time.as_nanos().is_multiple_of(dt.as_nanos()) {
                                return Err(
                                    "articulated impact time must be a positive whole public tick"
                                        .into(),
                                );
                            }
                            Some(time)
                        }
                        _ => return Err(
                            "select time_ns, first_impact, recurrent_contact or persistent_contact for an articulated impact"
                                .into(),
                        ),
                    };
                let sustained = node.attribute("detection") == Some("persistent_contact");
                let returning_step = if sustained
                    || node.attribute("detection") == Some("recurrent_contact")
                {
                    let width = if node.attribute("max_contact_step_ns").is_some() {
                        Duration::from_nanos(number(node, "max_contact_step_ns")?)
                    } else {
                        dt.min(Duration::from_micros(250))
                    };
                    if width < Duration::from_nanos(8)
                        || width > dt
                        || dt.as_nanos().div_ceil(width.as_nanos()) > 4096
                    {
                        return Err("recurrent material substep must fit the public clock and interval budget".into());
                    }
                    Some(width)
                } else {
                    if node.attribute("max_contact_step_ns").is_some() {
                        return Err(
                            "max_contact_step_ns requires recurrent material contact".into()
                        );
                    }
                    None
                };
                let slip_transition_step = if node
                    .attribute("max_slip_transition_step_ns")
                    .is_some()
                {
                    let cap = Duration::from_nanos(number(node, "max_slip_transition_step_ns")?);
                    if !sustained
                        || cap < Duration::from_nanos(8)
                        || returning_step.is_none_or(|h| cap > h)
                    {
                        return Err(
                            "slip transition step must fit the persistent contact interval".into(),
                        );
                    }
                    Some(cap)
                } else {
                    None
                };
                let restitution: f64 = number(node, "restitution")?;
                if sustained && restitution != 0. {
                    return Err("persistent normal support requires zero restitution".into());
                }
                if !(0.0..=1.0).contains(&restitution) {
                    return Err("normal restitution must be between zero and one".into());
                }
                let mut sites = Vec::new();
                for n in node.children().filter(Node::is_element) {
                    if !n.has_tag_name("sphere") {
                        return Err("articulated impacts require native sphere links".into());
                    }
                    fields(n, &["link"], false)?;
                    let name = attribute(n, "link")?;
                    let id = *links
                        .get(name)
                        .ok_or_else(|| format!("unknown articulated sphere link '{name}'"))?;
                    let body = entity_by_id(ecs, id)
                        .and_then(|e| ecs.get::<BodyHandle>(e))
                        .ok_or("missing articulated sphere body")?;
                    physics
                        .downcast_ref::<MultiFidelityBackend>()
                        .expect("validated engine")
                        .articulated_sphere_geometry(body.id())
                        .map_err(|e| format!("{name}: {e}"))?;
                    if sites.contains(&id)
                        || sites.len() >= gz_physics_multifidelity::MAX_ARTICULATED_CONTACT_SITES
                    {
                        return Err(
                            "duplicate sphere link or articulated contact site limit exceeded"
                                .into(),
                        );
                    }
                    sites.push(id);
                }
                if sites.is_empty() {
                    return Err("articulated impacts require at least one sphere".into());
                }
                let event = articulated_impact::ScheduledImpact {
                    material: attribute(node, "material")?.into(),
                    restitution,
                    links: sites,
                    returning_step,
                    slip_transition_step,
                    sustained,
                };
                if let Some(time) = time {
                    if automatic_impact.is_some() {
                        return Err(
                            "an automatically detected first impact owns the approach".into()
                        );
                    }
                    if impact_barriers.insert(time, event).is_some() {
                        return Err(
                            "collect all articulated impact spheres at one time into one batch"
                                .into(),
                        );
                    }
                } else if automatic_impact.is_none() && impact_barriers.is_empty() {
                    automatic_impact = Some(event);
                } else {
                    return Err("one automatically detected first impact owns the approach".into());
                }
            }
            "sphere" | "box" | "capsule" => {
                if !rigid_links.insert(attribute(node, "link")?) {
                    return Err("duplicate rigid boundary link".into());
                }
                surfaces.push(rigid_surface(
                    node,
                    world,
                    links,
                    ecs,
                    physics,
                    &capsule_pair,
                )?);
            }
            "plane" => {
                fields(node, &["point", "normal"], false)?;
                let normal = DVec3::from_array(vector(attribute(node, "normal")?)?)
                    .try_normalize()
                    .ok_or("invalid plane normal")?;
                surfaces.push(Surface::Plane {
                    point: vector(attribute(node, "point")?)?,
                    normal: normal.to_array(),
                });
            }
            tag => return Err(format!("unknown deformable element <{tag}>")),
        }
    }
    let finite_receiver = surfaces.iter().any(|s| {
        matches!(
            s,
            Surface::Rigid {
                mobility: Some(_),
                ..
            }
        )
    });
    if finite_receiver
        && let Some(native) = physics.downcast_ref::<MultiFidelityBackend>()
        && surfaces.iter().any(
            |s| matches!(s,Surface::Rigid{body,..} if native.translation_sample(*body).is_some()),
        )
    {
        return Err("prescribed curved boundaries currently require a one-way soft world".into());
    }
    if surfaces.iter().any(|s| {
        matches!(
            s,
            Surface::Rigid {
                mobility: Some(ReceiverMass::ForceAware(_)),
                ..
            }
        )
    }) && (initial.len() != 1
        || !initial
            .values()
            .all(SoftBody::uses_capsule_material_contact))
    {
        return Err("force-aware capsule motion requires one body with material contact".into());
    }
    if initial.len() > 1
        && initial
            .values()
            .any(SoftBody::uses_capsule_material_contact)
        && surfaces.iter().any(|s| {
            matches!(
                s,
                Surface::Rigid {
                    shape: ReceiverShape::Capsule { .. },
                    mobility: Some(_),
                    ..
                }
            )
        })
    {
        return Err("two-way material capsule contact requires one connected soft body".into());
    }
    if initial.values().any(SoftBody::uses_pad_surface_contact) {
        if initial.len() != 1 {
            return Err("surface pad contact requires one triangulated cloth".into());
        }
        let mut pads = 0;
        for s in &surfaces {
            match s {
                Surface::Plane { .. }
                | Surface::Rigid {
                    static_box_pose: Some(_),
                    ..
                } => {}
                Surface::Rigid {
                    shape: ReceiverShape::Sphere(_),
                    mobility: Some(ReceiverMass::Isotropic(_)),
                    ..
                } => pads += 1,
                _ => {
                    return Err(
                        "surface pads require two-way centered spheres with isotropic inertia"
                            .into(),
                    );
                }
            }
        }
        if pads > gz_physics_xpbd::MAX_SURFACE_PADS {
            return Err("surface pad count exceeds the solver limit".into());
        }
    }
    if initial.is_empty() {
        return Err("expected at least one cable or cloth".into());
    }
    if surfaces.iter().any(|s| {
        matches!(
            s,
            Surface::Rigid {
                mobility: Some(_),
                ..
            }
        )
    }) && !initial
        .values()
        .map(|b| b.config().substeps)
        .all(|n| n == initial.first_key_value().unwrap().1.config().substeps)
    {
        return Err("two-way bodies require a common soft substep clock".into());
    }
    if let Some(event) = automatic_impact.as_ref()
        && let Some(cap) = event.slip_transition_step
        && initial.get(&event.material).is_none_or(|b| {
            b.config().friction <= 0. || cap.as_nanos() < 2 * u128::from(b.config().substeps)
        })
    {
        return Err("slip transition refinement requires friction and enough time for the material substeps".into());
    }
    if automatic_impact.as_ref().is_some_and(|event| {
        event.sustained
            && initial
                .get(&event.material)
                .is_none_or(|b| b.particles().iter().any(|p| p.inverse_mass <= 0.))
    }) {
        return Err("persistent contact requires a movable finite cable or sheet".into());
    }
    if impact_barriers
        .values()
        .chain(automatic_impact.iter())
        .any(|event| {
            initial.len() != 1
                || !initial
                    .get(&event.material)
                    .is_some_and(SoftBody::supports_articulated_impact)
                || !surfaces.is_empty()
        })
    {
        return Err(
            "articulated impact requires one finite-compliance material with a positive skin and no other enrolled boundaries; tangential impacts require a cloth"
                .into(),
        );
    }
    let samples = initial
        .iter()
        .map(|(name, body)| {
            (
                name.clone(),
                DeformableSample {
                    movable_mass_kg: body
                        .particles()
                        .iter()
                        .filter(|p| p.inverse_mass > 0.)
                        .map(|p| p.inverse_mass.recip())
                        .sum(),
                    particles: body.particles().to_vec(),
                    ..Default::default()
                },
            )
        })
        .collect();
    for v in &visuals {
        if initial
            .get(&v.material)
            .is_none_or(|b| v.particle >= b.particles().len())
        {
            return Err(format!(
                "particle visual points outside material '{}' at {}",
                v.material, v.particle
            ));
        }
    }
    if usize::from(processes) > initial.len() {
        return Err("worker_processes exceeds the number of independent materials".into());
    }
    Ok(Deformables {
        #[cfg(feature = "distributed-physics")]
        worker_processes: processes,
        #[cfg(feature = "distributed-physics")]
        warm_budget: warm_ms.map(std::time::Duration::from_millis),
        #[cfg(feature = "distributed-physics")]
        packed_replies,
        #[cfg(feature = "distributed-physics")]
        worker_transport: if rpc == "pipe" {
            gz_physics_distributed::moving::WorkerTransport::Pipe
        } else {
            gz_physics_distributed::moving::WorkerTransport::LoopbackZenoh
        },
        #[cfg(feature = "distributed-physics")]
        worker_threads: threads,
        #[cfg(feature = "distributed-physics")]
        remote_material: false,
        visuals,
        automatic_impact,
        automatic_consumed: false,
        impact_barriers,
        pending_accelerations: BTreeMap::new(),
        current: initial.clone(),
        initial,
        samples,
        starts: vec![None; surfaces.len()],
        accelerations: vec![None; surfaces.len()],
        applied_forcing: vec![],
        pad_wrenches: BTreeMap::new(),
        articulated_wrenches: surfaces.iter().any(|s| {
            matches!(
                s,
                Surface::Rigid {
                    mobility: Some(ReceiverMass::ArticulatedWrench { .. }),
                    ..
                }
            )
        }),
        surfaces,
        boundaries: Vec::new(),
        receivers: Vec::new(),
        mobility: Vec::new(),
        active_slots: Vec::new(),
    })
}

pub(crate) fn install(
    sim: &mut Simulation,
    world: &sdformat::World,
    links: &BTreeMap<String, u64>,
    physics: Option<&PhysicsBackend>,
) -> Result<(), LoadError> {
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(());
    };
    let error = |reason| LoadError::Deformable {
        world: world.name.clone(),
        reason,
    };
    if plugins.next().is_some() {
        return Err(error("duplicate Deformables system".into()));
    }
    let physics = physics
        .filter(|p| p.downcast_ref::<MultiFidelityBackend>().is_some())
        .ok_or_else(|| error("Deformables requires the multifidelity backend".into()))?;
    let soft = parse(
        world,
        &plugin.xml,
        links,
        sim.world(),
        physics,
        sim.step_duration(),
    )
    .map_err(error)?;
    if !soft.impact_barriers.is_empty() {
        sim.add_systems(
            Phase::PhysicsCommit,
            articulated_impact::at_barrier
                .after(advance)
                .before(crate::sync_bodies),
        );
    }
    if soft.automatic_impact.is_none() {
        sim.add_systems(
            Phase::PhysicsPrepare,
            start_positions.after(crate::inputs::apply),
        );
        sim.add_systems(Phase::PhysicsCommit, advance.before(crate::sync_bodies));
    }
    let has_visuals = !soft.visuals.is_empty();
    sim.world_mut().insert_resource(soft);
    if has_visuals {
        let mut first = Schedule::default();
        first.add_systems(particle_visual::sync);
        first.run(sim.world_mut());
        sim.add_systems(
            Phase::PhysicsCommit,
            particle_visual::sync
                .after(crate::sync_attached_poses)
                .after(advance),
        );
        sim.add_systems(
            Phase::Reset,
            particle_visual::sync
                .after(crate::sync_attached_poses)
                .after(reset),
        );
    }
    sim.add_systems(Phase::Reset, reset);
    Ok(())
}
