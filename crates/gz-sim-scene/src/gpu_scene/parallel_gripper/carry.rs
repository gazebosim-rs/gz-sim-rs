// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A horizontal slider carrying the vertical gripper, with a finite named receiving shelf.
use super::*;
use configuration::{attr, num, plain, reason};
use gz_physics_gpu::{BoxFlightMaterial, CarryGripperLane, simulation::install_carry_gripper};
use roxmltree::Node;
#[derive(Clone, Copy)]
struct Launch {
    open_tick: u64,
    impulse_tick: u64,
    impulse: [f64; 3],
    point: [f64; 3],
}
#[derive(Clone, Copy)]
struct Move {
    track: f64,
    lift: f64,
    after_release: bool,
}
#[derive(Clone, Copy)]
struct Policy {
    after: u64,
    dwell: u64,
    fraction: f64,
    maximum_fraction: f64,
    speed: f64,
    gap: f64,
    open: [f64; 2],
}
/// The exact public tick at which the jaws received an opening command after shelf loading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReleaseReceipt {
    pub tick: u64,
    pub shelf_link: u64,
    pub sustained_ticks: u64,
    pub upward_force_n: f64,
    pub clearance_m: f64,
}
pub(super) struct Plan {
    id: u64,
    lane: CarryGripperLane,
    track: String,
    joint: sdformat::Joint,
    pose: Transform,
    local: Transform,
    shelf: String,
    moves: BTreeMap<u64, Move>,
    policy: Policy,
    flight: Option<(BoxFlightMaterial, Launch)>,
}
#[derive(Resource)]
pub(super) struct Binding {
    pub(super) id: u64,
    pub(super) track: u64,
    pub(super) joint: u64,
    shelf: (u64, WorldPose),
    moves: BTreeMap<u64, Move>,
    policy: Policy,
    dwell: u64,
    launch: Option<Launch>,
    receipt: Option<ReleaseReceipt>,
}
fn positive_tick(n: Node<'_, '_>, k: &str) -> Result<u64, LoadError> {
    attr(n, k)?
        .parse::<u64>()
        .ok()
        .filter(|&t| t > 0)
        .ok_or_else(|| reason("invalid carrying-gripper tick"))
}
impl Plan {
    pub(super) fn load(
        sdf: &sdformat::Sdf,
        w: &sdformat::World,
        dt: Duration,
        adapter: &str,
        root: Node<'_, '_>,
        nodes: &[Node<'_, '_>],
    ) -> Result<configuration::Plan, LoadError> {
        let [env] = nodes else {
            return Err(reason("one horizontal carrying gripper per world"));
        };
        require(
            env.has_tag_name("environment")
                && config::shape(root, &["name", "filename"], true)
                && config::shape(
                    *env,
                    &[
                        "id", "profile", "model", "object", "plane", "lift", "left", "right",
                        "track", "shelf",
                    ],
                    true,
                )
                && w.models.len() == 4
                && w.joints.is_empty()
                && dt == gz_physics_gpu::STEP
                && w.gravity[..2] == [0., 0.]
                && w.gravity[2] <= 0.,
            &w.name,
            "one four-model horizontal carrier, vertical gravity and a 1 ms clock",
        )?;
        let model = |key: &str| {
            w.models
                .iter()
                .find(|m| env.attribute(key) == Some(&m.name))
                .ok_or_else(|| reason("unknown carrier or free object model"))
        };
        let robot = model("model")?;
        let object = model("object")?;
        let static_link = |key: &str| -> Result<_, LoadError> {
            let name = attr(*env, key)?;
            let (model, link) = name
                .split_once("::")
                .ok_or_else(|| reason("static support needs model::link"))?;
            w.models
                .iter()
                .find(|m| {
                    m.is_static && m.name == model && m.links.len() == 1 && m.links[0].name == link
                })
                .ok_or_else(|| reason("unknown static support link"))
        };
        let floor = static_link("plane")?;
        let receiving = static_link("shelf")?;
        require(
            floor.name != receiving.name && robot.name != object.name,
            &w.name,
            "distinct source plane, shelf, free sphere and carrier required",
        )?;
        let (floor_name, ground, floor_mu) = planar_tool_config::plane(sdf, w, floor)?;
        let track_joint = robot
            .joints
            .iter()
            .find(|j| env.attribute("track") == Some(&j.name))
            .ok_or_else(|| reason("unknown track joint"))?;
        let rail = robot
            .links
            .iter()
            .find(|l| l.name == track_joint.child)
            .ok_or_else(|| reason("unknown track body"))?;
        let row = configuration::Row::load(
            sdf,
            w,
            *env,
            (robot, object),
            (ground, floor_mu),
            Some(&rail.name),
        )?;
        let joint_name = format!("{}::{}", robot.scoped_name, track_joint.name);
        let pose = sdf.pose_of(&rail.scoped_name, &w.name)?;
        let axis = track_joint
            .axis
            .as_ref()
            .ok_or_else(|| reason("missing rail axis"))?;
        let physical_inertia = rail.inertial.inertia;
        let [xx, yy, zz, xy, xz, yz] = physical_inertia;
        require(
            track_joint.kind == sdformat::JointKind::Prismatic
                && track_joint.parent == "world"
                && track_joint.axis2.is_none()
                && track_joint.sensors.is_empty()
                && track_joint.physics_xml.is_none()
                && !row.links.contains(&rail.scoped_name)
                && !row.joints.iter().any(|j| j.name == track_joint.name)
                && axis.velocity == f64::INFINITY
                && axis.mimic.is_none()
                && axis.dynamics.damping == 0.
                && axis.dynamics.friction == 0.
                && axis.dynamics.spring_stiffness == 0.
                && axis.dynamics.spring_reference == 0.
                && (DVec3::from_array(sdf.resolve_axis(axis, &joint_name, &w.name)?) - DVec3::X)
                    .length()
                    < 1e-12
                && identity(sdf.pose_of(&joint_name, &rail.scoped_name)?)
                && centered([pose.rotation[0], pose.rotation[1], pose.rotation[2]])
                && plain(rail)
                && rail.collisions.is_empty()
                && identity(sdf.resolve_pose(
                    &rail.inertial.pose,
                    &robot.scoped_name,
                    &rail.scoped_name,
                    &rail.scoped_name,
                )?)
                && [xx, yy, zz].iter().all(|v| v.is_finite() && *v > 0.)
                && xx + yy >= zz
                && yy + zz >= xx
                && xx + zz >= yy
                && [xy, xz, yz] == [0.; 3],
            &robot.name,
            "world-rooted collision-free physical track with aligned X axis and finite stops required",
        )?;
        let link = &receiving.links[0];
        let [c] = link.collisions.as_slice() else {
            return Err(reason("one finite receiving box required"));
        };
        let Geometry::Box { size } = c.geometry else {
            return Err(reason("only a finite receiving box is admitted"));
        };
        let shelf_pose = sdf.pose_of(&link.scoped_name, &w.name)?;
        require(
            receiving.joints.is_empty()
                && receiving.models.is_empty()
                && receiving.plugins.is_empty()
                && plain(link)
                && centered([
                    shelf_pose.rotation[0],
                    shelf_pose.rotation[1],
                    shelf_pose.rotation[2],
                ])
                && identity(sdf.resolve_pose(
                    &c.pose,
                    &receiving.scoped_name,
                    &link.scoped_name,
                    &link.scoped_name,
                )?)
                && (shelf_pose.translation[1] - row.poses[3].translation[1]).abs()
                    + row.payload_half.map_or(row.lane.radius, |s| s[1])
                    <= size[1] * 0.5,
            &receiving.name,
            "horizontal static shelf covering the full transverse width required",
        )?;
        let mut drives = env.children().filter(|n| n.has_tag_name("track-drive"));
        let drive = drives.next().ok_or_else(|| reason("missing rail drive"))?;
        require(
            drives.next().is_none()
                && config::shape(drive, &["stiffness", "damping", "target"], false),
            &robot.name,
            "one rail drive required",
        )?;
        let body = &object.links[0];
        let mass_pose = sdf.resolve_pose(
            &body.inertial.pose,
            &object.scoped_name,
            &body.scoped_name,
            &body.scoped_name,
        )?;
        let initial_rotation = DQuat::from_array(row.poses[3].rotation);
        let center = initial_rotation * DVec3::from_array(mass_pose.translation);
        // Only the world transverse axis must be principal. Other products of
        // inertia and a rotated inertial frame can still preserve planar motion.
        let direction =
            (initial_rotation * DQuat::from_array(mass_pose.rotation)).inverse() * DVec3::Y;
        let [ix, iy, iz, ixy, ixz, iyz] = body.inertial.inertia;
        let tensor = glam::DMat3::from_cols_array(&[ix, ixy, ixz, ixy, iy, iyz, ixz, iyz, iz]);
        let applied = tensor * direction;
        let angular_inertia = direction.dot(applied);
        let trace = ix + iy + iz;
        let box_half = row.payload_half.map_or([0.; 2], |s| [s[0], s[2]]);
        let payload_size = box_half[0].max(box_half[1]).max(row.lane.radius);
        let bound = if let Some(half) = row.payload_half {
            let rotation = DQuat::from_array(mass_pose.rotation).inverse();
            [DVec3::X, DVec3::Y, DVec3::Z]
                .into_iter()
                .enumerate()
                .all(|(i, axis)| {
                    let n = rotation * axis;
                    let second_moment = (trace * 0.5 - n.dot(tensor * n)) / body.inertial.mass
                        + mass_pose.translation[i].powi(2);
                    second_moment <= half[i] * half[i] * (1. + 64. * f64::EPSILON)
                })
        } else {
            trace * 0.5 <= body.inertial.mass * (row.lane.radius.powi(2) - center.length_squared())
        };
        require(
            center.y.abs() <= 64. * f64::EPSILON * payload_size
                && (applied - direction * angular_inertia).length() <= 64. * f64::EPSILON * trace
                && bound,
            &body.scoped_name,
            "carried body needs an in-plane mass center and a transverse principal axis with mass moments inside its geometry",
        )?;
        let x_axis = initial_rotation * DVec3::X;
        let box_heading = if row.payload_half.is_some() {
            x_axis.z.atan2(x_axis.x)
        } else {
            0.
        };
        let lane = CarryGripperLane {
            com_offset: [center.x, center.z],
            angular_inertia,
            hand: row.lane,
            track: pose.translation[0],
            track_speed: 0.,
            track_mass: rail.inertial.mass,
            track_origin: pose.translation[0],
            track_target: pose.translation[0] + num(drive, "target")?,
            track_stiffness: num(drive, "stiffness")?,
            track_damping: num(drive, "damping")?,
            track_effort: axis.effort,
            track_lower: pose.translation[0] + axis.lower,
            track_upper: pose.translation[0] + axis.upper,
            shelf_center: [shelf_pose.translation[0], shelf_pose.translation[2]],
            shelf_half: [size[0] * 0.5, size[2] * 0.5],
            shelf_friction: planar_tool_config::law(c)?
                .min(planar_tool_config::law(&object.links[0].collisions[0])?),
            shelf_normal: 0.,
            shelf_tangent: 0.,
            shelf_up: 0.,
            box_half,
            box_heading,
            point_impulse: [[0.; 2]; 4],
        };
        require(
            lane.valid() && row.commands.is_empty(),
            &robot.name,
            "carry travel must clear the shelf and use track moves",
        )?;
        let mut moves = BTreeMap::new();
        for m in env.children().filter(|n| n.has_tag_name("move")) {
            require(
                config::shape(m, &["tick", "track", "lift", "after_release"], false),
                &robot.name,
                "unknown carry move",
            )?;
            let after_release = match m.attribute("after_release") {
                None | Some("false") => false,
                Some("true") => true,
                _ => return Err(reason("invalid after_release flag")),
            };
            let step = Move {
                track: lane.track_origin + num(m, "track")?,
                lift: lane.hand.position[0] + num(m, "lift")?,
                after_release,
            };
            require(
                (lane.track_lower..=lane.track_upper).contains(&step.track)
                    && (lane.hand.lower[0]..=lane.hand.upper[0]).contains(&step.lift)
                    && moves.insert(positive_tick(m, "tick")?, step).is_none(),
                &robot.name,
                "duplicate or out-of-range carrier command",
            )?;
        }
        let mut releases = env.children().filter(|n| n.has_tag_name("release"));
        let r = releases
            .next()
            .ok_or_else(|| reason("missing shelf release rule"))?;
        require(
            releases.next().is_none()
                && config::shape(
                    r,
                    &[
                        "after",
                        "dwell",
                        "weight_fraction",
                        "max_weight_fraction",
                        "max_speed",
                        "max_gap",
                        "left",
                        "right",
                    ],
                    false,
                ),
            &robot.name,
            "one shelf release rule required",
        )?;
        let policy = Policy {
            after: positive_tick(r, "after")?,
            dwell: positive_tick(r, "dwell")?,
            fraction: num(r, "weight_fraction")?,
            maximum_fraction: num(r, "max_weight_fraction")?,
            speed: num(r, "max_speed")?,
            gap: num(r, "max_gap")?,
            open: [
                row.lane.position[1] + num(r, "left")?,
                row.lane.position[2] + num(r, "right")?,
            ],
        };
        require(
            policy.fraction > 0.
                && policy.fraction <= 1.
                && policy.maximum_fraction >= policy.fraction
                && policy.speed >= 0.
                && policy.gap >= 0.
                && (0..2).all(|j| {
                    (row.lane.lower[j + 1]..=row.lane.upper[j + 1]).contains(&policy.open[j])
                })
                && policy.open[0] < row.lane.target[1]
                && policy.open[1] > row.lane.target[2],
            &robot.name,
            "invalid shelf load threshold or jaw release coordinates",
        )?;
        let mut nodes = env.children().filter(|n| n.has_tag_name("box-flight"));
        let flight = if let Some(node) = nodes.next() {
            let open_tick = positive_tick(node, "open_tick")?;
            let impulse_tick = positive_tick(node, "impulse_tick")?;
            let vec = |name| {
                config::vector(Some(attr(node, name)?))
                    .ok_or_else(|| reason("finite three-component release input required"))
            };
            let impulse = vec("impulse")?;
            let point = vec("point")?;
            let half = row
                .payload_half
                .ok_or_else(|| reason("spatial release requires a finite cuboid"))?;
            let inertia_rotation = glam::DMat3::from_quat(DQuat::from_array(mass_pose.rotation));
            let local_tensor = inertia_rotation * tensor * inertia_rotation.transpose();
            require(
                nodes.next().is_none()
                    && config::shape(
                        node,
                        &["open_tick", "impulse_tick", "impulse", "point"],
                        false,
                    )
                    && impulse_tick > open_tick
                    && (0..3).all(|i| point[i].abs() <= half[i])
                    && [
                        local_tensor.x_axis.y,
                        local_tensor.x_axis.z,
                        local_tensor.y_axis.z,
                    ]
                    .iter()
                    .all(|v| v.abs() <= 64. * f64::EPSILON * trace),
                &object.name,
                "one open-then-impulse sequence at a cuboid material point with box-axis principal inertia required",
            )?;
            Some((
                BoxFlightMaterial {
                    inertia: [
                        local_tensor.x_axis.x,
                        local_tensor.y_axis.y,
                        local_tensor.z_axis.z,
                    ],
                    half_depth: half[1],
                    initial_orientation: initial_rotation.to_array(),
                    transverse_origin: row.poses[3].translation[1],
                },
                Launch {
                    open_tick,
                    impulse_tick,
                    impulse,
                    point,
                },
            ))
        } else {
            None
        };
        let plan = Self {
            flight,
            id: row.id,
            lane,
            track: rail.scoped_name.clone(),
            joint: track_joint.clone(),
            pose,
            local: sdf.pose_of(&joint_name, &robot.scoped_name)?,
            shelf: link.scoped_name.clone(),
            moves,
            policy,
        };
        Ok(configuration::Plan {
            adapter: adapter.into(),
            floor: floor_name,
            rows: vec![row],
            track: Some(Box::new(plan)),
        })
    }
}
pub(super) fn install(
    sim: &mut Simulation,
    adapter: &str,
    p: Plan,
    links: &BTreeMap<String, u64>,
) -> Result<MixedBatchOwner, LoadError> {
    let mut batch = CarryGripperBatch::new(adapter, vec![p.lane])?;
    if let Some((material, _)) = p.flight {
        batch.enable_box_flight(material)?;
    }
    let owner = batch.owner();
    let track = links[&p.track];
    let e = gz_sim_core::entity_by_id(sim.world(), track).unwrap();
    let parent = sim.world().get::<ChildOf>(e).unwrap().parent();
    let name = format!(
        "{}::{}",
        sim.world().get::<Name>(parent).unwrap().0,
        p.joint.name
    );
    let joint = spawn_node(sim, Some(parent), &name, EntityKind::Joint, p.local, p.pose)?;
    sim.world_mut().entity_mut(joint).insert((
        JointDefinition(Box::new(p.joint)),
        JointSample {
            state: gz_physics::JointState {
                position: 0.,
                velocity: 0.,
            },
            time: Duration::ZERO,
            epoch: 0,
        },
    ));
    let joint = sim.world().get::<EntityId>(joint).unwrap().get();
    let shelf = links[&p.shelf];
    let se = gz_sim_core::entity_by_id(sim.world(), shelf).unwrap();
    let pose = *sim.world().get::<WorldPose>(se).unwrap();
    install_carry_gripper(sim, batch)?;
    sim.world_mut().insert_resource(Binding {
        id: p.id,
        track,
        joint,
        shelf: (shelf, pose),
        moves: p.moves,
        policy: p.policy,
        dwell: 0,
        launch: p.flight.map(|(_, launch)| launch),
        receipt: None,
    });
    sim.add_systems(Phase::Reset, |mut b: ResMut<Binding>| {
        b.dwell = 0;
        b.receipt = None;
    });
    Ok(owner)
}
pub(super) fn valid(w: &World) -> bool {
    let Some(b) = w.get_resource::<Binding>() else {
        return true;
    };
    let node = |id| gz_sim_core::entity_by_id(w, id).and_then(|e| w.get_entity(e).ok());
    node(b.shelf.0).is_some_and(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Link)
            && e.get::<WorldPose>().is_some_and(|p| {
                p.translation == b.shelf.1.translation && p.rotation == b.shelf.1.rotation
            })
    }) && node(b.joint).is_some_and(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Joint)
            && e.contains::<JointDefinition>()
            && e.contains::<JointSample>()
    }) && node(b.track).is_some_and(|e| {
        e.get::<EntityKind>() == Some(&EntityKind::Link)
            && e.contains::<Velocity>()
            && e.contains::<WorldPose>()
    })
}
/// Last committed track, hand, free body and separately measured shelf impulse.
pub fn observe(
    sim: &Simulation,
    id: u64,
) -> Result<(&CarryGripperLane, Option<ReleaseReceipt>), GpuError> {
    if sim.is_faulted() || !super::valid(sim.world()) {
        return Err(GpuError::StaleBoundary);
    }
    let b = sim
        .world()
        .get_resource::<Binding>()
        .ok_or(GpuError::StaleBoundary)?;
    if b.id != id {
        return Err(GpuError::UnknownEnvironment(id));
    }
    Ok((
        &sim.world().resource::<CarryGripperBatch>().lanes()[0],
        b.receipt,
    ))
}
/// Set track, lift, left and right slider displacements from the authored pose.
pub fn set_targets(sim: &mut Simulation, id: u64, relative: [f64; 4]) -> Result<(), GpuError> {
    let (l, _) = observe(sim, id)?;
    let origin = sim.world().resource::<super::Binding>().rows[0].origin;
    let target = [
        l.track_origin + relative[0],
        origin[0] + relative[1],
        origin[1] + relative[2],
        origin[2] + relative[3],
    ];
    sim.world_mut()
        .resource_mut::<CarryGripperBatch>()
        .set_targets(0, target)
}
pub(super) fn prepare(world: &mut World) {
    let info = world.resource::<UpdateInfo>();
    if info.paused {
        return;
    }
    let tick = info.iteration;
    let l = world.resource::<CarryGripperBatch>().lanes()[0];
    let h = l.hand;
    let mut b = world.resource_mut::<Binding>();
    let mut target = [l.track_target, h.target[0], h.target[1], h.target[2]];
    if let Some(m) = b.moves.get(&tick)
        && (!m.after_release || b.receipt.is_some())
    {
        target[0] = m.track;
        target[1] = m.lift;
    }
    let launch = b.launch;
    if launch.is_some_and(|l| l.open_tick == tick) {
        target[2..].copy_from_slice(&b.policy.open);
    }
    if b.receipt.is_none() && tick > b.policy.after && launch.is_none_or(|l| tick < l.open_tick) {
        let [width, height] = l.payload_reach();
        let clearance = h.ball[1] - height - l.shelf_center[1] - l.shelf_half[1];
        let force = l.shelf_up / gz_physics_gpu::STEP.as_secs_f64();
        let supported = (h.ball[0] - l.shelf_center[0]).abs() < l.shelf_half[0] - width
            && clearance.abs() <= b.policy.gap
            && l.settling_speed() <= b.policy.speed
            && force >= b.policy.fraction * h.ball_mass * h.gravity
            && force <= b.policy.maximum_fraction * h.ball_mass * h.gravity
            && force > 0.
            && h.normal_impulse[2] == 0.;
        b.dwell = if supported {
            b.dwell.saturating_add(1)
        } else {
            0
        };
        if b.dwell >= b.policy.dwell {
            target[2..].copy_from_slice(&b.policy.open);
            b.receipt = Some(ReleaseReceipt {
                tick,
                shelf_link: b.shelf.0,
                sustained_ticks: b.dwell,
                upward_force_n: force,
                clearance_m: clearance,
            });
        }
    }
    if target != [l.track_target, h.target[0], h.target[1], h.target[2]]
        && let Err(e) = world
            .resource_mut::<CarryGripperBatch>()
            .set_targets(0, target)
    {
        world.resource::<gz_sim_core::StepFault>().report(e);
    }
    if let Some(launch) = launch
        && launch.impulse_tick == tick
        && let Err(error) = world
            .resource_mut::<CarryGripperBatch>()
            .release_box(launch.impulse, launch.point)
    {
        world.resource::<gz_sim_core::StepFault>().report(error);
    }
}
