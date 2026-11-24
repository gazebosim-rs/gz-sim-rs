// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use roxmltree::Node;
use std::collections::BTreeSet;

pub(super) fn reason(why: &str) -> LoadError {
    LoadError::GpuScene {
        entity: "parallel-gripper".into(),
        reason: why.into(),
    }
}
pub(super) fn attr<'a>(n: Node<'a, '_>, k: &str) -> Result<&'a str, LoadError> {
    n.attribute(k)
        .ok_or_else(|| reason("missing gripper attribute"))
}
pub(super) fn num(n: Node<'_, '_>, k: &str) -> Result<f64, LoadError> {
    attr(n, k)?
        .parse::<f64>()
        .ok()
        .filter(|x| x.is_finite())
        .ok_or_else(|| reason("invalid numeric drive or target"))
}
pub(super) fn plain(l: &sdformat::Link) -> bool {
    l.properties == sdformat::LinkProperties::default()
        && l.sensors.is_empty()
        && l.velocity_decay.is_none()
        && l.lights.is_empty()
        && l.projectors.is_empty()
        && l.particle_emitters.is_empty()
        && l.inertial.fluid_added_mass.is_none()
}
pub(crate) struct Plan {
    pub(super) adapter: String,
    pub(super) floor: String,
    pub(super) rows: Vec<Row>,
    pub(super) track: Option<Box<carry::Plan>>,
}
pub(super) struct Row {
    pub(super) id: u64,
    half_width: f64,
    pub(super) lane: ParallelGripperLane,
    pub(super) links: [String; 4],
    pub(super) poses: [Transform; 4],
    pub(super) joints: [sdformat::Joint; 3],
    pub(super) local: [Transform; 3],
    pub(super) commands: BTreeMap<u64, [f64; 3]>,
}
impl Plan {
    pub(in crate::gpu_scene) fn load(
        sdf: &sdformat::Sdf,
        w: &sdformat::World,
        dt: Duration,
        adapter: Option<&str>,
        root: Node<'_, '_>,
    ) -> Result<Self, LoadError> {
        let adapter = adapter
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| reason("select hardware with --gpu-adapter NAME"))?;
        let nodes: Vec<_> = root.children().filter(Node::is_element).collect();
        if nodes
            .iter()
            .any(|n| n.attribute("profile") == Some("carrying-gripper"))
        {
            return carry::Plan::load(sdf, w, dt, adapter, root, &nodes);
        }
        require(
            !nodes.is_empty()
                && config::shape(root, &["name", "filename"], true)
                && w.models.len() == 2 * nodes.len() + 1
                && w.joints.is_empty()
                && dt == gz_physics_gpu::STEP
                && w.gravity[0] == 0.
                && w.gravity[1] == 0.
                && w.gravity[2] <= 0.,
            &w.name,
            "independent gripper world, vertical gravity and a 1 ms clock",
        )?;
        let mut floors = w.models.iter().filter(|m| m.is_static);
        let f = floors.next().ok_or_else(|| reason("missing table"))?;
        require(
            floors.next().is_none(),
            &w.name,
            "one static table required",
        )?;
        let (floor, ground, floor_mu) = planar_tool_config::plane(sdf, w, f)?;
        let models: BTreeMap<_, _> = w.models.iter().map(|m| (m.name.as_str(), m)).collect();
        let mut claimed = BTreeSet::new();
        let mut rows = BTreeMap::new();
        let mut strips = Vec::with_capacity(nodes.len());
        for env in nodes {
            require(
                env.has_tag_name("environment")
                    && config::shape(
                        env,
                        &[
                            "id", "profile", "model", "object", "plane", "lift", "left", "right",
                        ],
                        true,
                    )
                    && env.attribute("profile") == Some("parallel-gripper")
                    && format!("{}::{}", w.name, attr(env, "plane")?) == floor,
                &w.name,
                "independent gripper world needs a named profile and shared table",
            )?;
            let model = |k: &str| {
                models
                    .get(attr(env, k)?)
                    .copied()
                    .ok_or_else(|| reason("unknown gripper or sphere model"))
            };
            let m = model("model")?;
            let obj = model("object")?;
            require(
                claimed.insert(&m.name) && claimed.insert(&obj.name),
                &m.name,
                "duplicate gripper or sphere ownership",
            )?;
            let row = Row::load(sdf, w, env, (m, obj), (ground, floor_mu), None)?;
            let y = row.poses[3].translation[1];
            strips.push((y - row.half_width, y + row.half_width));
            require(
                rows.insert(row.id, row).is_none(),
                &m.name,
                "duplicate gripper environment id",
            )?;
        }
        strips.sort_by(|a, b| a.0.total_cmp(&b.0));
        require(
            strips.windows(2).all(|w| w[0].1 < w[1].0),
            &w.name,
            "gripper environments require disjoint lateral swept volumes",
        )?;
        Ok(Self {
            adapter: adapter.to_owned(),
            floor,
            rows: rows.into_values().collect(),
            track: None,
        })
    }
}
impl Row {
    pub(super) fn load(
        sdf: &sdformat::Sdf,
        w: &sdformat::World,
        env: Node<'_, '_>,
        (m, obj): (&sdformat::Model, &sdformat::Model),
        (ground, floor_mu): (f64, f64),
        track: Option<&str>,
    ) -> Result<Self, LoadError> {
        let id = attr(env, "id")?
            .parse()
            .map_err(|_| reason("invalid environment id"))?;
        require(
            !m.is_static
                && m.joints.len() == 3 + usize::from(track.is_some())
                && m.links.len() == 3 + usize::from(track.is_some())
                && m.models.is_empty()
                && m.plugins.iter().all(joint_state_publisher::is_plugin)
                && !m.enable_wind
                && !m.self_collide
                && m.raw_namespace.is_none(),
            &m.name,
            "single ordinary three-slider model and named fixed plane required",
        )?;
        let (ball_name, data) = config::model_shape(sdf, w, obj)?;
        let config::ShapeData::Sphere(ball) = data else {
            return Err(reason("free solid sphere required"));
        };
        let ball_link = &obj.links[0];
        let ball_mu = planar_tool_config::law(&ball_link.collisions[0])?;
        require(
            plain(ball_link),
            &ball_name,
            "ordinary dynamic sphere with gravity and no sensors required",
        )?;
        let joint = |key: &str| -> Result<&sdformat::Joint, LoadError> {
            let name = attr(env, key)?;
            m.joints
                .iter()
                .find(|j| j.name == name)
                .ok_or_else(|| reason("unknown slider joint"))
        };
        let joints = [joint("lift")?, joint("left")?, joint("right")?];
        let link = |j: &sdformat::Joint| {
            m.links
                .iter()
                .find(|l| l.name == j.child)
                .ok_or_else(|| reason("unknown slider child"))
        };
        let links = [link(joints[0])?, link(joints[1])?, link(joints[2])?];
        require(
            joints[0].parent == track.unwrap_or("world")
                && joints[1].parent == links[0].name
                && joints[2].parent == links[0].name
                && links[1].name != links[2].name
                && links[0].collisions.is_empty(),
            &m.name,
            "expected collision-free lift carriage and two distinct moving fingertips",
        )?;
        let mut names = links.map(|l| l.scoped_name.clone()).to_vec();
        names.push(ball_name);
        let names: [String; 4] = names.try_into().unwrap();
        let poses = [
            sdf.pose_of(&names[0], &w.name)?,
            sdf.pose_of(&names[1], &w.name)?,
            sdf.pose_of(&names[2], &w.name)?,
            sdf.pose_of(&names[3], &w.name)?,
        ];
        require(
            (poses[1].translation[2] - poses[2].translation[2]).abs() < 1e-12
                && poses[..3].iter().all(|p| {
                    centered([p.rotation[0], p.rotation[1], p.rotation[2]])
                        && (p.translation[1] - ball.position[1]).abs() < 1e-12
                }),
            &m.name,
            "axis-aligned coplanar sliders with level finger centers required",
        )?;
        let position = [
            poses[1].translation[2],
            poses[1].translation[0],
            poses[2].translation[0],
        ];
        let mut lane = ParallelGripperLane {
            position,
            speed: [0.; 3],
            mass: links.map(|l| l.inertial.mass),
            half_size: [[0.; 2]; 2],
            ball: [ball.position[0], ball.position[2]],
            velocity: [0.; 2],
            angle: 0.,
            spin: 0.,
            radius: ball.radius,
            ball_mass: ball_link.inertial.mass,
            gravity: -w.gravity[2],
            ground,
            target: position,
            stiffness: [0.; 3],
            damping: [0.; 3],
            force_limit: [0.; 3],
            friction: [0., 0., floor_mu.min(ball_mu)],
            normal_impulse: [0.; 3],
            tangent_impulse: [0.; 3],
            support_impulse: [0.; 2],
            lower: [0.; 3],
            upper: [0.; 3],
        };
        let mut local = [Transform::default(); 3];
        let mut half_width = ball.radius;
        for i in 0..3 {
            let j = joints[i];
            let l = links[i];
            let joint_name = format!("{}::{}", m.scoped_name, j.name);
            let axis = j
                .axis
                .as_ref()
                .ok_or_else(|| reason("missing slider axis"))?;
            let dir = sdf.resolve_axis(axis, &joint_name, &w.name)?;
            let delta = DVec3::from_array(dir) - if i == 0 { DVec3::Z } else { DVec3::X };
            let frame = sdf.resolve_pose(&l.inertial.pose, &m.scoped_name, &names[i], &names[i])?;
            let [xx, yy, zz, xy, xz, yz] = l.inertial.inertia;
            require(
                plain(l)
                    && identity(frame)
                    && [xx, yy, zz].iter().all(|v| v.is_finite() && *v > 0.)
                    && xx + yy >= zz
                    && xx + zz >= yy
                    && yy + zz >= xx
                    && [xy, xz, yz] == [0.; 3]
                    && j.kind == sdformat::JointKind::Prismatic
                    && j.axis2.is_none()
                    && j.sensors.is_empty()
                    && j.physics_xml.is_none()
                    && axis.mimic.is_none()
                    && axis.dynamics.damping == 0.
                    && axis.dynamics.friction == 0.
                    && axis.dynamics.spring_stiffness == 0.
                    && axis.dynamics.spring_reference == 0.
                    && axis.velocity == f64::INFINITY
                    && delta.length() < 1e-12
                    && identity(sdf.pose_of(&joint_name, &names[i])?),
                &l.name,
                "centered ordinary slider, finite travel, effort limit, aligned axis and no extra dynamics required",
            )?;
            local[i] = sdf.pose_of(&joint_name, &m.scoped_name)?;
            lane.lower[i] = position[i] + axis.lower;
            lane.upper[i] = position[i] + axis.upper;
            lane.force_limit[i] = axis.effort;
            if i > 0 {
                let [c] = l.collisions.as_slice() else {
                    return Err(reason("each fingertip needs one finite box"));
                };
                let Geometry::Box { size } = c.geometry else {
                    return Err(reason("only finite box fingertip collisions are supported"));
                };
                require(
                    size[1] >= 2. * ball.radius
                        && identity(sdf.resolve_pose(
                            &c.pose,
                            &m.scoped_name,
                            &names[i],
                            &names[i],
                        )?),
                    &l.name,
                    "centered finite pad covering the sphere's width required",
                )?;
                half_width = half_width.max(size[1] * 0.5);
                lane.half_size[i - 1] = [size[0] * 0.5, size[2] * 0.5];
                lane.friction[i - 1] = ball_mu.min(planar_tool_config::law(c)?);
            }
        }
        let mut commands = BTreeMap::new();
        let mut driven = BTreeSet::new();
        for n in env.children().filter(Node::is_element) {
            match n.tag_name().name() {
                "track-drive" | "move" | "release" if track.is_some() => continue,
                "drive"
                    if config::shape(n, &["joint", "stiffness", "damping", "target"], false) =>
                {
                    let name = attr(n, "joint")?;
                    let i = joints
                        .iter()
                        .position(|j| j.name == name)
                        .ok_or_else(|| reason("unknown gripper drive"))?;
                    require(driven.insert(i), name, "duplicate gripper drive")?;
                    lane.stiffness[i] = num(n, "stiffness")?;
                    lane.damping[i] = num(n, "damping")?;
                    lane.target[i] = position[i] + num(n, "target")?;
                }
                "target" if config::shape(n, &["tick", "lift", "left", "right"], false) => {
                    let tick = attr(n, "tick")?
                        .parse::<u64>()
                        .ok()
                        .filter(|&k| k > 0)
                        .ok_or_else(|| reason("invalid target tick"))?;
                    let q = [
                        num(n, "lift")? + position[0],
                        num(n, "left")? + position[1],
                        num(n, "right")? + position[2],
                    ];
                    require(
                        (0..3).all(|i| (lane.lower[i]..=lane.upper[i]).contains(&q[i]))
                            && commands.insert(tick, q).is_none(),
                        &m.name,
                        "duplicate or out-of-range gripper target",
                    )?;
                }
                _ => return Err(reason("unknown gripper drive element")),
            }
        }
        require(
            driven.len() == 3 && lane.valid(),
            &m.name,
            "invalid drive or gripper geometry, initial contact or travel envelope",
        )?;
        Ok(Self {
            id,
            half_width,
            lane,
            links: names,
            poses,
            joints: joints.map(Clone::clone),
            local,
            commands,
        })
    }
}
