// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_gpu::{
    EnvironmentState as State, SolidSphere, SpherePairLane, SpherePlaneLane,
    SupportedSpherePairLane,
};
use roxmltree::Node;
use std::collections::{BTreeMap, BTreeSet};
pub(crate) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::GpuBatch" && p.filename == "gz-sim-gpu-batch-system"
}
fn err(entity: &str, why: impl Into<String>) -> LoadError {
    LoadError::GpuScene {
        entity: entity.into(),
        reason: why.into(),
    }
}
pub(super) fn shape(node: Node<'_, '_>, attrs: &[&str], children: bool) -> bool {
    node.namespaces().next().is_none()
        && node
            .attributes()
            .all(|a| a.namespace().is_none() && attrs.contains(&a.name()))
        && node.children().all(|c| {
            c.is_comment()
                || c.is_text() && c.text().unwrap_or("").trim().is_empty()
                || children && c.is_element()
        })
}
fn vector(x: Option<&str>) -> Option<[f64; 3]> {
    let Some(x) = x else {
        return Some([0.; 3]);
    };
    let mut v = x.split_ascii_whitespace().map(str::parse::<f64>);
    let p = [v.next()?.ok()?, v.next()?.ok()?, v.next()?.ok()?];
    (v.next().is_none() && p.into_iter().all(f64::is_finite)).then_some(p)
}
pub(super) enum ShapeData {
    Sphere(SolidSphere),
    Plane { normal: [f64; 3], offset: f64 },
}
fn shapes(
    sdf: &sdformat::Sdf,
    world: &sdformat::World,
) -> Result<BTreeMap<String, ShapeData>, LoadError> {
    world
        .models
        .iter()
        .map(|model| model_shape(sdf, world, model))
        .collect()
}
pub(super) fn model_shape(
    sdf: &sdformat::Sdf,
    world: &sdformat::World,
    m: &sdformat::Model,
) -> Result<(String, ShapeData), LoadError> {
    model_shape_with_inertia(sdf, world, m, false)
}
pub(super) fn model_shape_with_inertia(
    sdf: &sdformat::Sdf,
    world: &sdformat::World,
    m: &sdformat::Model,
    planar_payload: bool,
) -> Result<(String, ShapeData), LoadError> {
    let name = &m.scoped_name;
    require(
        m.models.is_empty()
            && m.joints.is_empty()
            && m.links.len() == 1
            && m.plugins.is_empty()
            && !m.enable_wind
            && !m.self_collide,
        name,
        "GPU environments require one ordinary link per top-level model",
    )?;
    let l = &m.links[0];
    let link = &l.scoped_name;
    require(
        l.collisions.len() == 1
            && l.sensors
                .iter()
                .all(|sensor| sensor.imu().is_some() || sensor.altimeter().is_some())
            && l.lights.is_empty()
            && l.projectors.is_empty()
            && l.particle_emitters.is_empty()
            && l.velocity_decay.is_none(),
        link,
        "GPU links require one collider and support only IMU and altimeter sensors without velocity decay",
    )?;
    let collision = &l.collisions[0];
    let inertial = sdf.resolve_pose(&l.inertial.pose, name, link, link)?;
    require(
        identity(sdf.resolve_pose(&collision.pose, name, link, link)?)
            && (planar_payload || centered(inertial.translation)),
        link,
        "GPU link origin must coincide with the collider and center of mass",
    )?;
    let p = sdf.pose_of(link, &world.name)?;
    let s = match &collision.geometry {
        Geometry::Sphere { radius } if !m.is_static => {
            let mass = l.inertial.mass;
            let i = 0.4 * mass * radius * radius;
            let expected = [i, i, i, 0., 0., 0.];
            // The carrying gripper checks its attached mass frame and inertia tensor
            // in world coordinates after resolving the complete robot topology.
            if !planar_payload {
                require(
                    i.is_finite()
                        && i > 0.
                        && l.inertial
                            .inertia
                            .iter()
                            .zip(expected)
                            .all(|(observed, target)| {
                                (observed - target).abs() <= 64. * f64::EPSILON * i
                            }),
                    link,
                    "GPU sphere requires centered uniform solid-sphere inertia (2 m r² / 5)",
                )?;
            }
            ShapeData::Sphere(SolidSphere {
                position: p.translation,
                orientation: p.rotation,
                radius: *radius,
                inverse_mass: 1. / mass,
                velocity: [0.; 3],
                angular_velocity: [0.; 3],
            })
        }
        Geometry::Plane { normal, size } if m.is_static => {
            let local = DVec3::from_array(*normal);
            require(
                local.is_finite()
                    && (local.length_squared() - 1.).abs() < 2e-12
                    && size.iter().all(|x| *x > 0. && x.is_finite()),
                link,
                "finite GPU plane with unit local normal",
            )?;
            let n = DQuat::from_array(p.rotation) * local;
            ShapeData::Plane {
                normal: n.to_array(),
                offset: DVec3::from_array(p.translation).dot(n),
            }
        }
        _ => return Err(err(link, "expected a dynamic sphere or a static plane")),
    };
    Ok((link.clone(), s))
}
pub(crate) struct Plan {
    pub(super) adapter: String,
    pub(super) initial_cpu: Vec<u64>,
    pub(super) native_pairs: Vec<(u64, PairSolver)>,
    pub(super) native_workers: Option<(usize, bool)>,
    pub(super) automatic: Option<automatic::AutoPlacementConfig>,
    pub(super) initial: Vec<Environment>,
    pub(super) links: Vec<(String, u64, BodySlot)>,
    pub(super) every: Option<u64>,
    pub(super) pair_workers: Option<usize>,
    pub(super) block: Option<(u32, gz_physics_gpu::SphereRotation)>,
    pub(super) planes: Vec<String>,
}
impl Plan {
    pub(crate) fn uses_processes(&self) -> bool {
        self.pair_workers.is_some()
    }
    pub(crate) fn load(
        sdf: &sdformat::Sdf,
        w: &sdformat::World,
        step: Duration,
        adapter: Option<&str>,
    ) -> Result<Self, LoadError> {
        let adapter = adapter.filter(|s| !s.trim().is_empty()).ok_or_else(|| {
            err(
                &w.name,
                "select a physical Vulkan device with --gpu-adapter NAME",
            )
        })?;
        require(
            w.joints.is_empty() && w.plugins.len() == 1 && is_plugin(&w.plugins[0]),
            &w.name,
            "GPU world requires one GpuBatch world plugin and no world joints",
        )?;
        let mut available = shapes(sdf, w)?;
        let doc = roxmltree::Document::parse(&w.plugins[0].xml)
            .map_err(|e| err(&w.name, e.to_string()))?;
        let root = doc.root_element();
        if !shape(root, &["name", "filename"], true) {
            return Err(err(&w.name, "invalid GpuBatch plugin attributes or text"));
        }
        let mut plan = Self {
            adapter: adapter.into(),
            initial_cpu: vec![],
            native_pairs: vec![],
            native_workers: None,
            automatic: None,
            initial: vec![],
            links: vec![],
            every: None,
            block: None,
            pair_workers: None,
            planes: vec![],
        };
        let mut ids = BTreeSet::new();
        for node in root.children().filter(Node::is_element) {
            if node.has_tag_name("resident_block") {
                if !shape(node, &["ticks", "rotation", "workers"], false) || plan.block.is_some() {
                    return Err(err(&w.name, "expected one resident_block element"));
                }
                let ticks = node
                    .attribute("ticks")
                    .and_then(|v| v.parse::<u32>().ok())
                    .filter(|v| (1..=gz_physics_gpu::MAX_BLOCK_TICKS).contains(v))
                    .ok_or_else(|| err(&w.name, "resident_block ticks must be 1 through 4096"))?;
                let rotation = match node.attribute("rotation") {
                    None | Some("midpoint") => gz_physics_gpu::SphereRotation::Midpoint,
                    Some("fourth-order") => gz_physics_gpu::SphereRotation::FourthOrder,
                    _ => {
                        return Err(err(
                            &w.name,
                            "resident_block rotation must be midpoint or fourth-order",
                        ));
                    }
                };
                if let Some(v) = node.attribute("workers") {
                    plan.pair_workers = Some(
                        v.parse::<usize>()
                            .ok()
                            .filter(|n| {
                                (1..=gz_physics_gpu::pair_process::MAX_PAIR_WORKERS).contains(n)
                            })
                            .ok_or_else(|| {
                                err(&w.name, "resident_block workers must be 1 through 16")
                            })?,
                    );
                }
                plan.block = Some((ticks, rotation));
                continue;
            }
            if node.has_tag_name("native_staging") {
                if !shape(node, &["workers", "overlap_gpu"], false) || plan.native_workers.is_some()
                {
                    return Err(err(
                        &w.name,
                        "expected one native_staging element with workers",
                    ));
                }
                let overlap = match node.attribute("overlap_gpu") {
                    None | Some("false") => false,
                    Some("true") => true,
                    _ => {
                        return Err(err(
                            &w.name,
                            "native_staging overlap_gpu must be true or false",
                        ));
                    }
                };
                plan.native_workers = Some((
                    node.attribute("workers")
                        .and_then(|v| v.parse().ok())
                        .filter(|n| (1..=gz_physics_gpu::MAX_NATIVE_WORKERS).contains(n))
                        .ok_or_else(|| {
                            err(&w.name, "native_staging workers must be between 1 and 4")
                        })?,
                    overlap,
                ));
                continue;
            }
            if node.has_tag_name("automatic_placement") {
                let attrs = [
                    "window_ticks",
                    "min_residency_ticks",
                    "reprobe_ticks",
                    "hysteresis_percent",
                ];
                if !shape(node, &attrs, false) || plan.automatic.is_some() {
                    return Err(err(&w.name, "expected one automatic_placement element"));
                }
                let mut c = automatic::AutoPlacementConfig::default();
                let invalid = || err(&w.name, "invalid automatic placement timing or hysteresis");
                if let Some(v) = node.attribute(attrs[0]) {
                    c.window_ticks = v.parse().map_err(|_| invalid())?;
                }
                if let Some(v) = node.attribute(attrs[1]) {
                    c.min_residency_ticks = v.parse().map_err(|_| invalid())?;
                }
                if let Some(v) = node.attribute(attrs[2]) {
                    c.reprobe_ticks = v.parse().map_err(|_| invalid())?;
                }
                if let Some(v) = node.attribute(attrs[3]) {
                    c.hysteresis_percent = v.parse().map_err(|_| invalid())?;
                }
                if !c.valid() {
                    return Err(invalid());
                }
                plan.automatic = Some(c);
                continue;
            }
            if node.has_tag_name("observe") {
                if !shape(node, &["every"], false) || plan.every.is_some() {
                    return Err(err(
                        &w.name,
                        "expected one <observe every='positive ticks'/>",
                    ));
                }
                plan.every = Some(
                    node.attribute("every")
                        .and_then(|x| x.parse::<u64>().ok())
                        .filter(|&x| x > 0)
                        .ok_or_else(|| err(&w.name, "invalid observation cadence"))?,
                );
                continue;
            }
            let id = node
                .attribute("id")
                .and_then(|v| v.parse::<u64>().ok())
                .ok_or_else(|| err(&w.name, "expected explicit integer environment id"))?;
            let scope = format!("{}::environment[{id}]", w.name);
            if !node.has_tag_name("environment")
                || !shape(
                    node,
                    &[
                        "id",
                        "profile",
                        "friction",
                        "restitution",
                        "execution",
                        "native_handoff",
                    ],
                    true,
                )
                || !ids.insert(id)
            {
                return Err(err(&scope, "unknown fields or duplicate environment id"));
            }
            let profile = node
                .attribute("profile")
                .ok_or_else(|| err(&scope, "missing profile"))?;
            let marked = match node.attribute("native_handoff") {
                None | Some("false") => false,
                Some("true") => true,
                _ => return Err(err(&scope, "native_handoff must be true or false")),
            };
            if marked {
                require(
                    matches!(profile, "sphere-pair" | "sphere-plane"),
                    &scope,
                    "native handoff requires a sphere-pair or sphere-plane profile",
                )?;
                let initial = match node.attribute("execution").unwrap_or("gpu") {
                    "native" => PairSolver::NativeRapier,
                    "gpu" => PairSolver::SpecializedGpu,
                    _ => {
                        return Err(err(
                            &scope,
                            "native_handoff execution must be native or gpu",
                        ));
                    }
                };
                plan.native_pairs.push((id, initial));
                require(
                    plan.native_pairs.len() <= gz_physics_gpu::MAX_NATIVE_PAIRS,
                    &scope,
                    "too many guarded native pairs",
                )?;
            } else {
                match node.attribute("execution").unwrap_or("gpu") {
                    "gpu" => {}
                    "cpu" => plan.initial_cpu.push(id),
                    _ => {
                        return Err(err(
                            &scope,
                            "execution must be cpu or gpu for an unmarked environment",
                        ));
                    }
                }
            }
            let friction = node
                .attribute("friction")
                .and_then(|v| v.parse::<f64>().ok())
                .ok_or_else(|| err(&scope, "missing or invalid friction"))?;
            let restitution = node
                .attribute("restitution")
                .unwrap_or("0")
                .parse::<f64>()
                .map_err(|_| err(&scope, "invalid restitution"))?;
            let (mut a, mut b, mut plane) = (None, None, None);
            for part in node.children().filter(Node::is_element) {
                let role = part.tag_name().name();
                let link = part
                    .attribute("link")
                    .ok_or_else(|| err(&scope, "missing scoped model::link reference"))?;
                let link = format!("{}::{link}", w.name);
                let value = available
                    .remove(&link)
                    .ok_or_else(|| err(&link, "unresolved or multiply-owned physical link"))?;
                match (role, value) {
                    ("plane", ShapeData::Plane { normal, offset })
                        if plane.is_none() && shape(part, &["link"], false) =>
                    {
                        plane = Some((normal, offset));
                        plan.planes.push(link);
                    }
                    ("a" | "b", ShapeData::Sphere(mut s))
                        if shape(part, &["link", "linear", "angular"], false) =>
                    {
                        s.velocity = vector(part.attribute("linear")).ok_or_else(|| {
                            err(&link, "expected finite world linear velocity triple")
                        })?;
                        s.angular_velocity =
                            vector(part.attribute("angular")).ok_or_else(|| {
                                err(&link, "expected finite world angular velocity triple")
                            })?;
                        let (slot, body) = if role == "a" {
                            (&mut a, BodySlot::A)
                        } else {
                            (&mut b, BodySlot::B)
                        };
                        if slot.replace(s).is_some() {
                            return Err(err(&scope, "duplicate body role"));
                        }
                        plan.links.push((link, id, body));
                    }
                    _ => {
                        return Err(err(
                            &link,
                            "role, physical geometry or attributes differ from the selected profile",
                        ));
                    }
                }
            }
            let a = a.ok_or_else(|| err(&scope, "missing a sphere"))?;
            let pair = |b| SpherePairLane {
                a,
                b,
                gravity: w.gravity,
                friction,
                restitution,
                normal_impulse: 0.,
                tangential_impulse: [0.; 3],
                contact_normal: [0.; 3],
                impact_time: -1.,
            };
            let state = match (profile, b, plane) {
                ("sphere-plane", None, Some((normal, offset))) if restitution == 0. => {
                    State::SpherePlane(SpherePlaneLane {
                        position: a.position,
                        orientation: a.orientation,
                        velocity: a.velocity,
                        angular_velocity: a.angular_velocity,
                        gravity: w.gravity,
                        plane_normal: normal,
                        plane_offset: offset,
                        radius: a.radius,
                        inverse_mass: a.inverse_mass,
                        friction,
                        normal_impulse: 0.,
                        tangential_impulse: [0.; 3],
                    })
                }
                ("sphere-pair", Some(b), None) => State::SpherePair(pair(b)),
                ("supported-pair", Some(b), Some((normal, offset))) => {
                    State::SupportedSpherePair(SupportedSpherePairLane {
                        pair: pair(b),
                        plane_normal: normal,
                        plane_offset: offset,
                        support_normal_impulses: [0.; 2],
                    })
                }
                _ => {
                    return Err(err(
                        &scope,
                        "unsupported profile, participants or restitution",
                    ));
                }
            };
            let environment = Environment { id, state };
            if marked {
                environment
                    .validate_native_pair()
                    .map_err(|e| err(&scope, e.to_string()))?;
            }
            plan.initial.push(environment);
        }
        if let Some(link) = available.keys().next() {
            return Err(err(link, "physical link has no GPU environment owner"));
        }
        require(
            !plan.initial.is_empty(),
            &w.name,
            "empty GPU environment set",
        )?;
        require(
            plan.automatic.is_none()
                || plan.initial_cpu.is_empty()
                || plan.initial_cpu.len() == plan.initial.len(),
            &w.name,
            "automatic placement requires a uniform initial execution owner",
        )?;
        require(
            plan.automatic.is_none() || plan.native_pairs.is_empty(),
            &w.name,
            "automatic scalar/GPU placement cannot own guarded native pairs",
        )?;
        require(
            plan.native_workers.is_none() || !plan.native_pairs.is_empty(),
            &w.name,
            "native_staging requires marked native contacts",
        )?;
        if let Some((ticks, _)) = plan.block {
            require(
                plan.pair_workers.is_none_or(|n| n <= plan.initial.len()),
                &w.name,
                "each resident_block worker needs a nonempty environment range",
            )?;
            require(
                step == gz_physics_gpu::STEP * ticks,
                &w.name,
                "resident_block needs a public max_step_size equal to ticks times 1 ms",
            )?;
            require(
                plan.initial_cpu.is_empty()
                    && plan.native_pairs.is_empty()
                    && plan.native_workers.is_none()
                    && plan.automatic.is_none()
                    && plan
                        .initial
                        .iter()
                        .all(|e| matches!(&e.state, State::SpherePair(_))),
                &w.name,
                "resident_block requires independent GPU sphere-pair environments",
            )?;
        } else {
            require(
                step == gz_physics_gpu::STEP,
                &w.name,
                "GPU batch requires a 1 ms SDF or override step",
            )?;
        }
        Ok(plan)
    }
}
