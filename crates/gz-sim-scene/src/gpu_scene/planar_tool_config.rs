// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Admission for homogeneous fixed-base planar arms. All geometry and inertia are read from SDF.
use super::*;
use gz_physics_gpu::{PlanarArmLane, PlanarToolLane};
use roxmltree::Node;
use std::collections::{BTreeMap, BTreeSet};
pub(super) struct Arm {
    pub id: u64,
    pub links: [String; 2],
    pub joints: [sdformat::Joint; 2],
    pub joint_poses: [(Transform, Transform); 2],
    pub base: [f64; 3],
    pub origin: [f64; 2],
    pub lane: PlanarToolLane,
    pub commands: BTreeMap<u64, [f64; 2]>,
    pub controllers: [Option<joint_controller::Parameters>; 2],
}
pub(crate) struct Plan {
    pub(super) adapter: String,
    pub(super) arms: Vec<Arm>,
    pub(super) plane: String,
}
fn err(why: &'static str) -> LoadError {
    LoadError::GpuScene {
        entity: "planar-tool".into(),
        reason: why.into(),
    }
}
fn attr<'a>(n: Node<'a, '_>, k: &str) -> Result<&'a str, LoadError> {
    n.attribute(k)
        .ok_or_else(|| err("missing planar-tool attribute"))
}
fn num(n: Node<'_, '_>, k: &str) -> Result<f64, LoadError> {
    attr(n, k)?
        .parse::<f64>()
        .ok()
        .filter(|x| x.is_finite())
        .ok_or_else(|| err("nonfinite planar-tool value"))
}
fn near(v: [f64; 3], w: [f64; 3]) -> bool {
    (DVec3::from_array(v) - DVec3::from_array(w)).length() <= 1e-10
}
fn exact_x(v: [f64; 3]) -> bool {
    v[0].is_finite() && v[1].abs() <= 1e-10 && v[2].abs() <= 1e-10
}
fn law(c: &sdformat::Collision) -> Result<f64, LoadError> {
    if c.surface.source.is_none() {
        Ok(0.5)
    } else {
        local_isotropic_friction(&c.surface)
            .map(|x| x.coefficient())
            .ok_or_else(|| err("tool contact requires isotropic Coulomb friction"))
    }
}
fn plane(
    sdf: &sdformat::Sdf,
    w: &sdformat::World,
    m: &sdformat::Model,
) -> Result<(String, f64, f64), LoadError> {
    let [link] = m.links.as_slice() else {
        return Err(err("tool floor needs one static link"));
    };
    let [c] = link.collisions.as_slice() else {
        return Err(err("tool floor needs one plane"));
    };
    let Geometry::Plane { normal, .. } = c.geometry else {
        return Err(err("tool floor requires a plane"));
    };
    let pose = sdf.pose_of(&link.scoped_name, &w.name)?;
    require(
        m.is_static
            && m.models.is_empty()
            && m.joints.is_empty()
            && m.plugins.is_empty()
            && identity(sdf.resolve_pose(
                &c.pose,
                &m.scoped_name,
                &link.scoped_name,
                &link.scoped_name,
            )?)
            && near(
                (DQuat::from_array(pose.rotation) * DVec3::from_array(normal)).to_array(),
                [0., 0., 1.],
            ),
        &m.name,
        "tool floor requires one horizontal upward plane",
    )?;
    Ok((link.scoped_name.clone(), pose.translation[2], law(c)?))
}
fn link_angle(p: Transform) -> Result<f64, LoadError> {
    let rot = DQuat::from_array(p.rotation);
    let x = rot * DVec3::X;
    if !near((rot * DVec3::Y).to_array(), [0., 1., 0.]) {
        return Err(err("arm links must stay in the world XZ plane"));
    }
    Ok(x.z.atan2(x.x))
}
fn turn(a: f64) -> [f64; 2] {
    [a.cos(), a.sin()]
}
fn load_arm(
    n: Node<'_, '_>,
    sdf: &sdformat::Sdf,
    w: &sdformat::World,
    m: &sdformat::Model,
    floor: f64,
    floor_mu: f64,
) -> Result<Arm, LoadError> {
    let [first, second] = m.joints.as_slice() else {
        return Err(err("planar-tool requires exactly two revolute joints"));
    };
    let joints = if first.parent == "world" {
        [first, second]
    } else {
        [second, first]
    };
    require(
        !m.is_static
            && m.links.len() == 2
            && m.models.is_empty()
            && m.plugins
                .iter()
                .all(|p| joint_controller::is_plugin(p) || joint_state_publisher::is_plugin(p))
            && m.raw_namespace.is_none()
            && !m.self_collide
            && !m.enable_wind
            && joints[0].parent == "world"
            && joints[1].parent == joints[0].child
            && joints[1].child != joints[0].child,
        &m.name,
        "expected world-rooted two-link tool chain",
    )?;
    let link = |j: &sdformat::Joint| {
        m.links
            .iter()
            .find(|l| l.name == j.child)
            .ok_or_else(|| err("joint child link missing"))
    };
    let links = [link(joints[0])?, link(joints[1])?];
    let ids = links.map(|l| l.scoped_name.clone());
    let wp = [
        sdf.pose_of(&ids[0], &w.name)?,
        sdf.pose_of(&ids[1], &w.name)?,
    ];
    let abs = [link_angle(wp[0])?, link_angle(wp[1])?];
    let relative = abs[1] - abs[0];
    let origin = [abs[0], relative.sin().atan2(relative.cos())];
    let offset = sdf.pose_of(&ids[1], &ids[0])?.translation;
    let [tool] = links[1].collisions.as_slice() else {
        return Err(err("forearm requires one spherical tool collision"));
    };
    let Geometry::Sphere { radius } = tool.geometry else {
        return Err(err("forearm requires a spherical tool collision"));
    };
    let tip = sdf
        .resolve_pose(&tool.pose, &m.scoped_name, &ids[1], &ids[1])?
        .translation;
    require(
        links[0].collisions.is_empty()
            && exact_x(offset)
            && offset[0] > 0.
            && exact_x(tip)
            && tip[0] > 0.,
        &m.name,
        "upper must be collision-free with elbow and round tool along local +X",
    )?;
    let mut arm = PlanarArmLane {
        angle: origin.map(turn),
        speed: [0.; 2],
        length: [offset[0], tip[0]],
        center: [0.; 2],
        mass: [0.; 2],
        inertia: [0.; 2],
        tip_mass: 0.,
        gravity: -w.gravity[2],
        target: [[1., 0.]; 2],
        stiffness: [0.; 2],
        damping: [0.; 2],
        effort_limit: [0.; 2],
        effort: [0.; 2],
    };
    let mut controllers = [None, None];
    for controller in joint_controller::planar_parameters(m)? {
        let index = joints
            .iter()
            .position(|j| j.name == controller.joint)
            .expect("parsed joint");
        controllers[index] = Some(controller);
    }
    let mut poses = [(Transform::default(), Transform::default()); 2];
    for i in 0..2 {
        let l = links[i];
        let j = joints[i];
        let name = format!("{}::{}", m.scoped_name, j.name);
        let a = j.axis.as_ref().ok_or_else(|| err("missing hinge axis"))?;
        poses[i] = (
            sdf.pose_of(&name, &m.scoped_name)?,
            sdf.pose_of(&name, &w.name)?,
        );
        require(
            j.kind == sdformat::JointKind::Revolute
                && j.axis2.is_none()
                && j.sensors.is_empty()
                && j.physics_xml.is_none()
                && a.mimic.is_none()
                && near(sdf.resolve_axis(a, &name, &w.name)?, [0., -1., 0.])
                && identity(sdf.pose_of(&name, &ids[i])?)
                && a.lower == f64::NEG_INFINITY
                && a.upper == f64::INFINITY
                && (controllers[i].is_some() || a.velocity == f64::INFINITY)
                && a.effort.is_finite()
                && a.effort > 0.
                && a.dynamics.damping == 0.
                && a.dynamics.friction == 0.
                && a.dynamics.spring_stiffness == 0.
                && a.dynamics.spring_reference == 0.,
            &name,
            "tool hinges require local pivots, parallel world -Y axes, finite effort and unlimited range/speed with free joint dynamics",
        )?;
        let com = sdf.resolve_pose(&l.inertial.pose, &m.scoped_name, &ids[i], &ids[i])?;
        let [xx, yy, zz, xy, xz, yz] = l.inertial.inertia;
        require(
            exact_x(com.translation)
                && (0.0..=arm.length[i]).contains(&com.translation[0])
                && near(
                    (DQuat::from_array(com.rotation) * DVec3::Y).to_array(),
                    [0., 1., 0.],
                )
                && [xx, yy, zz].iter().all(|v| v.is_finite() && *v > 0.)
                && xy == 0.
                && xz == 0.
                && yz == 0.
                && xx + yy >= zz
                && yy + zz >= xx
                && xx + zz >= yy
                && l.inertial.mass.is_finite()
                && l.inertial.mass > 0.
                && l.sensors.is_empty()
                && l.velocity_decay.is_none(),
            &ids[i],
            "tool links require ordinary rigid mass and aligned physical inertia along the link",
        )?;
        arm.mass[i] = l.inertial.mass;
        arm.center[i] = com.translation[0];
        arm.inertia[i] = yy;
        arm.effort_limit[i] = a.effort;
    }
    let mut driven = BTreeSet::new();
    let mut commands = BTreeMap::new();
    for item in n.children().filter(Node::is_element) {
        match item.tag_name().name() {
            "drive" if config::shape(item, &["joint", "stiffness", "damping", "target"], false) => {
                let joint = attr(item, "joint")?;
                let i = joints
                    .iter()
                    .position(|j| j.name == joint)
                    .ok_or_else(|| err("unknown tool drive joint"))?;
                require(driven.insert(i), joint, "one drive per tool joint")?;
                let d = num(item, "damping")?;
                if controllers[i].is_some() {
                    require(
                        d > 0. && config::shape(item, &["joint", "damping"], false),
                        joint,
                        "controlled tool drives require only a positive motor damping",
                    )?;
                } else {
                    let k = num(item, "stiffness")?;
                    require(k >= 0. && d >= 0., joint, "nonnegative tool drive gains")?;
                    arm.stiffness[i] = k;
                    arm.target[i] = turn(origin[i] + num(item, "target")?);
                }
                arm.damping[i] = d;
            }
            "target" if config::shape(item, &["tick", "shoulder", "elbow"], false) => {
                let tick = attr(item, "tick")?
                    .parse::<u64>()
                    .ok()
                    .filter(|t| *t > 0)
                    .ok_or_else(|| err("invalid target tick"))?;
                if commands
                    .insert(
                        tick,
                        [
                            num(item, "shoulder")? + origin[0],
                            num(item, "elbow")? + origin[1],
                        ],
                    )
                    .is_some()
                {
                    return Err(err("duplicate target tick"));
                }
            }
            _ => return Err(err("unsupported planar-tool drive or target")),
        }
    }
    require(
        commands.is_empty() || controllers.iter().all(Option::is_none),
        &m.name,
        "scripted targets conflict with standard joint controllers",
    )?;
    require(driven.len() == 2, &m.name, "both joints require a drive")?;
    let lane = PlanarToolLane {
        arm,
        ground_z: floor - wp[0].translation[2],
        radius,
        friction: law(tool)?.min(floor_mu),
        normal_impulse: 0.,
        tangent_impulse: 0.,
        first_contact_offset: -1.,
        first_normal_impulse: 0.,
        first_tangent_impulse: 0.,
        turns: [0.; 2],
        velocity_mode: controllers.each_ref().map(|p| f64::from(p.is_some())),
        velocity_target: [0.; 2],
    };
    require(lane.gap() >= 0., &m.name, "tool starts below the floor")?;
    Ok(Arm {
        id: attr(n, "id")?
            .parse()
            .map_err(|_| err("invalid tool environment id"))?,
        base: wp[0].translation,
        origin,
        links: ids,
        joints: joints.map(Clone::clone),
        joint_poses: poses,
        lane,
        controllers,
        commands,
    })
}
impl Plan {
    pub(super) fn load(
        sdf: &sdformat::Sdf,
        w: &sdformat::World,
        step: Duration,
        adapter: Option<&str>,
        root: Node<'_, '_>,
    ) -> Result<Self, LoadError> {
        let adapter = adapter
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| err("select hardware with --gpu-adapter NAME"))?;
        require(
            step == gz_physics_gpu::STEP
                && w.joints.is_empty()
                && w.gravity[0] == 0.
                && w.gravity[1] == 0.
                && w.gravity[2] <= 0.
                && config::shape(root, &["name", "filename"], true),
            &w.name,
            "tool world requires a 1 ms step, downward gravity and one GpuBatch plugin",
        )?;
        let mut floors = w.models.iter().filter(|m| m.is_static);
        let floor = floors.next().ok_or_else(|| err("tool floor missing"))?;
        if floors.next().is_some() {
            return Err(err("tool world requires one static floor"));
        }
        let (plane, height, mu) = plane(sdf, w, floor)?;
        let mut arms = BTreeMap::new();
        let mut models = BTreeSet::new();
        for n in root.children().filter(Node::is_element) {
            require(
                n.has_tag_name("environment")
                    && config::shape(n, &["id", "profile", "model", "plane"], true)
                    && n.attribute("profile") == Some("planar-tool")
                    && n.attribute("plane")
                        .is_some_and(|name| format!("{}::{name}", w.name) == plane),
                &w.name,
                "expected independent planar-tool environments on the declared floor",
            )?;
            let model = attr(n, "model")?;
            let m = w
                .models
                .iter()
                .find(|m| m.name == model && !m.is_static)
                .ok_or_else(|| err("unknown tool model"))?;
            let arm = load_arm(n, sdf, w, m, height, mu)?;
            if !models.insert(model) || arms.insert(arm.id, arm).is_some() {
                return Err(err("duplicate tool environment or model"));
            }
        }
        require(
            models.len() + 1 == w.models.len(),
            &w.name,
            "every dynamic model needs a tool environment",
        )?;
        let arms: Vec<Arm> = arms.into_values().collect();
        require(
            arms.iter()
                .filter(|a| a.controllers.iter().any(Option::is_some))
                .count()
                <= joint_controller::MAX_CONTROLLER_OWNERS,
            &w.name,
            "multiple joint-controller chains",
        )?;
        let mut topics = BTreeSet::new();
        require(
            arms.iter()
                .flat_map(|a| a.controllers.iter().flatten())
                .all(|c| topics.insert(&c.publication.topic)),
            &w.name,
            "duplicate tool joint-controller topics",
        )?;
        // Each sphere remains in its arm's fixed Y plane. Refuse independent
        // lanes whose swept tool discs can overlap physically.
        for (i, a) in arms.iter().enumerate() {
            for b in &arms[..i] {
                let r = a.lane.radius + b.lane.radius;
                let reach = a
                    .lane
                    .arm
                    .length
                    .iter()
                    .chain(&b.lane.arm.length)
                    .sum::<f64>()
                    + r;
                require(
                    (a.base[1] - b.base[1]).abs() > r
                        || (a.base[0] - b.base[0]).hypot(a.base[2] - b.base[2]) > reach,
                    &w.name,
                    "independent GPU arms require disjoint swept tool volumes",
                )?;
            }
        }
        Ok(Self {
            adapter: adapter.into(),
            arms,
            plane,
        })
    }
}
