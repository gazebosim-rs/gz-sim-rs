// Copyright (C) 2021, 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Loaded noiseless ground-truth odometry, separate from wheel integration.
use super::*;
use std::collections::BTreeSet;

/// Native pose-derived odometry mode; omitted XML dimensions select `Two`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GroundTruthOdometryDimensions {
    Two,
    Three,
}

#[derive(Component, Clone, Debug, PartialEq)]
pub struct GroundTruthOdometryPublication {
    pub dimensions: GroundTruthOdometryDimensions,
    pub model_name: String,
    pub odom_frame: String,
    pub robot_base_frame: String,
    pub odom_topic: String,
    pub covariance_topic: String,
    pub tf_topic: String,
}
#[derive(Component, Clone, Copy, Debug)]
pub struct GroundTruthOdometrySample {
    pub time: Duration,
    pub epoch: u64,
    pub revision: u64,
    /// Native dimensions=2 retains orientation but publishes position z=0.
    pub pose: WorldPose,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, thiserror::Error)]
pub enum GroundTruthOdometryError {
    #[error("invalid ground-truth odometry ownership or metadata")]
    Identity,
    #[error("invalid ground-truth odometry pose")]
    Pose,
    #[error("ground-truth odometry clock is backward or unrepresentable")]
    Time,
    #[error("ground-truth odometry arithmetic is not finite")]
    Arithmetic,
    #[error("ground-truth odometry revision overflow")]
    Revision,
}
#[derive(Debug, thiserror::Error)]
#[error("ground-truth odometry model {entity}: {source}")]
pub struct GroundTruthOdometryScheduleFault {
    pub entity: u64,
    #[source]
    pub source: GroundTruthOdometryError,
}
#[derive(Clone, Default)]
struct Mean {
    values: [f64; 10],
    count: usize,
}
impl Mean {
    fn push(&mut self, value: f64) -> Result<f64, GroundTruthOdometryError> {
        if self.count == 10 {
            self.values.copy_within(1..10, 0);
        } else {
            self.count += 1;
        }
        self.values[self.count - 1] = value;
        // Match native accumulate oldest-to-newest, not a rolling-sum recurrence.
        let mean = self.values[..self.count]
            .iter()
            .copied()
            .fold(0.0, |a, b| a + b)
            / self.count as f64;
        mean.is_finite()
            .then_some(mean)
            .ok_or(GroundTruthOdometryError::Arithmetic)
    }
}
#[derive(Component, Clone)]
pub(super) struct Publisher {
    owner: u64,
    publication: GroundTruthOdometryPublication,
    period: Duration,
    previous_time: Option<Duration>,
    previous_pose: WorldPose,
    last_publication: Duration,
    revision: u64,
    means: [Mean; 3],
    spatial_means: Option<[Mean; 3]>,
}
impl Publisher {
    fn new(owner: u64, publication: GroundTruthOdometryPublication, period: Duration) -> Self {
        let spatial_means = (publication.dimensions == GroundTruthOdometryDimensions::Three)
            .then(|| std::array::from_fn(|_| Mean::default()));
        Self {
            owner,
            publication,
            period,
            previous_time: None,
            previous_pose: WorldPose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            last_publication: Duration::ZERO,
            revision: 0,
            means: std::array::from_fn(|_| Mean::default()),
            spatial_means,
        }
    }
}
#[derive(Resource)]
pub(super) struct Installed;
pub(super) struct Plan {
    publication: GroundTruthOdometryPublication,
    period: Duration,
}
pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::OdometryPublisher"
        && plugin.filename == "gz-sim-odometry-publisher-system"
}
fn invalid(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}
fn topic(value: &str) -> bool {
    value.starts_with('/')
        && value.len() > 1
        && value.len() <= 65535
        && !value
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
        && !["~", "//", "@", ":="]
            .iter()
            .any(|part| value.contains(part))
}
fn frame(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 65535
        && !value
            .bytes()
            .any(|b| b.is_ascii_control() || b.is_ascii_whitespace())
}
fn period(rate: f64) -> Option<Duration> {
    if !rate.is_finite() {
        return None;
    }
    if rate <= 0.0 {
        return Some(Duration::ZERO);
    }
    let nanos = (1.0 / rate) * 1e9;
    (nanos.is_finite() && nanos < i64::MAX as f64).then(|| Duration::from_nanos(nanos as u64))
}
pub(super) fn parse(model: &Model) -> Result<Option<Plan>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    let bad = || invalid(model, "ground-truth odometry configuration");
    if plugins.next().is_some() || plugin.xml.len() > 65536 {
        return Err(bad());
    }
    let doc = roxmltree::Document::parse(&plugin.xml).map_err(|_| bad())?;
    let root = doc.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root
            .attributes()
            .any(|a| a.namespace().is_some() || !matches!(a.name(), "name" | "filename"))
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(bad());
    }
    let mut publication = GroundTruthOdometryPublication {
        dimensions: GroundTruthOdometryDimensions::Two,
        model_name: model.name.clone(),
        odom_frame: format!("{}/odom", model.name),
        robot_base_frame: format!("{}/base_footprint", model.name),
        odom_topic: format!("/model/{}/odometry", model.name),
        covariance_topic: format!("/model/{}/odometry_with_covariance", model.name),
        tf_topic: format!("/model/{}/pose", model.name),
    };
    let mut frequency = Duration::from_millis(20);
    let mut seen = BTreeSet::new();
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element()
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|n| !n.is_text())
            || plugin.xml[child.range()].contains("<![CDATA[")
            || !seen.insert(child.tag_name().name())
        {
            return Err(bad());
        }
        let text = child.text().unwrap_or_default().trim();
        match child.tag_name().name() {
            "dimensions" if text == "2" => {}
            "dimensions" if text == "3" => {
                publication.dimensions = GroundTruthOdometryDimensions::Three;
            }
            "odom_publish_frequency" => {
                frequency = period(text.parse().map_err(|_| bad())?).ok_or_else(bad)?
            }
            "odom_frame" | "robot_base_frame" => {
                if !frame(text) {
                    return Err(bad());
                }
                if child.tag_name().name() == "odom_frame" {
                    publication.odom_frame = text.into();
                } else {
                    publication.robot_base_frame = text.into();
                }
            }
            "odom_topic" | "odom_covariance_topic" | "tf_topic" => {
                if !topic(text) {
                    return Err(bad());
                }
                match child.tag_name().name() {
                    "odom_topic" => publication.odom_topic = text.into(),
                    "odom_covariance_topic" => publication.covariance_topic = text.into(),
                    _ => publication.tf_topic = text.into(),
                }
            }
            "gaussian_noise" => {
                let value: f64 = text.parse().map_err(|_| bad())?;
                if !value.is_finite() || value != 0.0 {
                    return Err(bad());
                }
            }
            "xyz_offset" | "rpy_offset" | "xyz_global_offset" | "rpy_global_offset" => {
                let values: Result<Vec<f64>, _> = text.split_whitespace().map(str::parse).collect();
                let values = values.map_err(|_| bad())?;
                if values.len() != 3 || values.iter().any(|v| !v.is_finite() || *v != 0.0) {
                    return Err(bad());
                }
            }
            _ => return Err(bad()),
        }
    }
    let topics = [
        &publication.odom_topic,
        &publication.covariance_topic,
        &publication.tf_topic,
    ];
    if publication.model_name.is_empty()
        || publication.model_name.contains("::")
        || !topics.iter().all(|t| topic(t))
        || topics.into_iter().collect::<BTreeSet<_>>().len() != 3
        || !frame(&publication.odom_frame)
        || !frame(&publication.robot_base_frame)
    {
        return Err(bad());
    }
    Ok(Some(Plan {
        publication,
        period: frequency,
    }))
}
impl Plan {
    pub(super) fn validate(
        &self,
        model: &Model,
        sim: &Simulation,
        engine: Option<&str>,
        placement: (bool, bool),
    ) -> Result<(), LoadError> {
        require(
            engine == Some("rapier")
                && placement.0
                && model.models.is_empty()
                && model.raw_namespace.is_none(),
            &model.scoped_name,
            "ground-truth odometry requires a top-level flat Rapier model",
        )?;
        require(
            !sim.world()
                .iter_entities()
                .any(|e| e.contains::<Publisher>()),
            &model.scoped_name,
            "multiple ground-truth odometry models",
        )
    }
}
pub(super) fn build(
    _sdf: &Sdf,
    sim: &mut Simulation,
    model: &Model,
    entity: Entity,
    _backend: &PhysicsBackend,
    plan: &Plan,
) -> Result<(), LoadError> {
    let owner = sim
        .world()
        .get::<EntityId>(entity)
        .ok_or_else(|| invalid(model, "missing ground-truth model identity"))?
        .get();
    sim.world_mut().entity_mut(entity).insert((
        plan.publication.clone(),
        Publisher::new(owner, plan.publication.clone(), plan.period),
    ));
    Ok(())
}
// Native scalar quaternion operations use xyzw storage here, wxyz evaluation order.
fn mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [x, y, z, w] = a;
    let [u, v, s, t] = b;
    [
        w * u + x * t + y * s - z * v,
        w * v - x * s + y * t + z * u,
        w * s + x * v - y * u + z * t,
        w * t - x * u - y * v - z * s,
    ]
}
fn inverse(q: [f64; 4]) -> [f64; 4] {
    let [x, y, z, w] = q;
    let s = w * w + x * x + y * y + z * z;
    [-x / s, -y / s, -z / s, w / s]
}
// Pose3's six-angle zero constructor yields +0 translation/quaternion xyz, +1 w:
// SetFromEuler(0, 0, 0), followed by Normalize. Preserve those signed zeros.
fn native_zero_offset() -> WorldPose {
    WorldPose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    }
}
fn native_pose_mul(a: WorldPose, b: WorldPose) -> WorldPose {
    // Pose3::CoordPositionAdd uses quaternion multiplication, not the optimized
    // Quaternion::operator*(Vector3) used by Pose3::Inverse below.
    let rotated = mul(
        a.rotation,
        mul(
            [b.translation[0], b.translation[1], b.translation[2], 0.0],
            inverse(a.rotation),
        ),
    );
    WorldPose {
        translation: std::array::from_fn(|i| a.translation[i] + rotated[i]),
        rotation: mul(a.rotation, b.rotation),
    }
}
fn native_zero_offset_inverse() -> WorldPose {
    let offset = native_zero_offset();
    let rotation = inverse(offset.rotation);
    // For the fixed zero translation, unary negation retains the native
    // multiplication-by-minus-one signed-zero result.
    let vector = offset.translation.map(|v| -v);
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let axis = [rotation[0], rotation[1], rotation[2]];
    let uv = cross(axis, vector);
    let uuv = cross(axis, uv);
    let uv = uv.map(|v| v * (2.0 * rotation[3]));
    let uuv = uuv.map(|v| v * 2.0);
    WorldPose {
        translation: std::array::from_fn(|i| (vector[i] + uv[i]) + uuv[i]),
        rotation,
    }
}
fn valid_pose(pose: WorldPose) -> bool {
    pose.translation
        .into_iter()
        .chain(pose.rotation)
        .all(f64::is_finite)
        && (DQuat::from_array(pose.rotation).length_squared() - 1.0).abs() <= 1e-12
}
fn velocities(
    previous: WorldPose,
    pose: WorldPose,
    dt: f64,
) -> Result<[f64; 3], GroundTruthOdometryError> {
    let [x, y, z, w] = pose.rotation;
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    let sarg = -2.0 * (x * z - w * y);
    let yaw = if (sarg - 1.0).abs() < 1e-15 || (sarg + 1.0).abs() < 1e-15 {
        0.0
    } else {
        (2.0 * (x * y + w * z)).atan2(w * w + x * x - y * y - z * z)
    };
    let (sin, cos) = ((yaw as f32).sin() as f64, (yaw as f32).cos() as f64);
    let dx = pose.translation[0] - previous.translation[0];
    let dy = pose.translation[1] - previous.translation[1];
    let mut current = pose.rotation;
    let a = current;
    let b = previous.rotation;
    if a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < 0.0 {
        current = current.map(|v| -v);
    }
    let delta = mul(current, inverse(previous.rotation));
    let squared = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
    let angular = if squared <= 1e-12 {
        [0.0; 3]
    } else {
        let angle = 2.0 * delta[3].acos();
        let inv_length = 1.0 / squared.sqrt();
        [
            delta[0] * inv_length * (angle / dt),
            delta[1] * inv_length * (angle / dt),
            delta[2] * inv_length * (angle / dt),
        ]
    };
    let body = mul(
        inverse(pose.rotation),
        mul([angular[0], angular[1], angular[2], 0.0], pose.rotation),
    );
    let result = [
        (cos * dx + sin * dy) / dt,
        (cos * dy - sin * dx) / dt,
        body[2],
    ];
    result
        .into_iter()
        .all(f64::is_finite)
        .then_some(result)
        .ok_or(GroundTruthOdometryError::Arithmetic)
}
// Keep the accepted planar function above unchanged: its f32 yaw trigonometry
// is deliberately different from the native spatial quaternion rotation.
fn spatial_velocities(
    previous: WorldPose,
    pose: WorldPose,
    dt: f64,
) -> Result<[f64; 6], GroundTruthOdometryError> {
    let rotate_reverse = |value: [f64; 3]| {
        let rotated = mul(
            inverse(pose.rotation),
            mul([value[0], value[1], value[2], 0.0], pose.rotation),
        );
        [rotated[0], rotated[1], rotated[2]]
    };
    let displacement = std::array::from_fn(|i| pose.translation[i] - previous.translation[i]);
    let linear = rotate_reverse(displacement).map(|value| value / dt);
    let mut current = pose.rotation;
    let a = current;
    let b = previous.rotation;
    if a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < 0.0 {
        current = current.map(|value| -value);
    }
    let delta = mul(current, inverse(previous.rotation));
    let squared = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
    let (axis, angle) = if squared <= 1e-12 {
        ([1.0, 0.0, 0.0], 0.0)
    } else {
        let inv_length = 1.0 / squared.sqrt();
        (
            [
                delta[0] * inv_length,
                delta[1] * inv_length,
                delta[2] * inv_length,
            ],
            2.0 * delta[3].acos(),
        )
    };
    // The sign adjustment selects only rotationDiff. Native body rotation uses
    // the original current quaternion, including its original IEEE signs.
    let angular = rotate_reverse(axis.map(|value| (angle / dt) * value));
    let values = [
        linear[0], linear[1], linear[2], angular[0], angular[1], angular[2],
    ];
    values
        .into_iter()
        .all(f64::is_finite)
        .then_some(values)
        .ok_or(GroundTruthOdometryError::Arithmetic)
}

impl Publisher {
    fn advance(
        &mut self,
        pose: WorldPose,
        time: UpdateInfo,
    ) -> Result<Option<GroundTruthOdometrySample>, GroundTruthOdometryError> {
        let Some(previous_time) = self.previous_time else {
            self.previous_time = Some(time.sim_time);
            return Ok(None);
        };
        let elapsed = time
            .sim_time
            .checked_sub(previous_time)
            .filter(|d| d.as_nanos() < i64::MAX as u128)
            .ok_or(GroundTruthOdometryError::Time)?;
        if elapsed.as_secs_f64() <= 1e-6 {
            return Ok(None);
        }
        if !valid_pose(pose) {
            return Err(GroundTruthOdometryError::Pose);
        }
        // Input is already WORLD pose. Even admitted zero offsets execute the
        // native scalar operations; omitting them changes IEEE signed-zero bits.
        let pose = native_pose_mul(
            native_pose_mul(native_zero_offset_inverse(), pose),
            native_zero_offset(),
        );
        let (linear, angular) = match self.publication.dimensions {
            GroundTruthOdometryDimensions::Two => {
                let values = velocities(self.previous_pose, pose, elapsed.as_secs_f64())?;
                let means = [
                    self.means[0].push(values[0])?,
                    self.means[1].push(values[1])?,
                    self.means[2].push(values[2])?,
                ];
                ([means[0], means[1], 0.0], [0.0, 0.0, means[2]])
            }
            GroundTruthOdometryDimensions::Three => {
                let values = spatial_velocities(self.previous_pose, pose, elapsed.as_secs_f64())?;
                let extra = self
                    .spatial_means
                    .as_mut()
                    .ok_or(GroundTruthOdometryError::Identity)?;
                (
                    // Native adds DblNormal(0, 0), which returns +0 without RNG.
                    // Keep the addition, including underflowed mean -0 -> +0.
                    [
                        self.means[0].push(values[0])?,
                        self.means[1].push(values[1])?,
                        extra[0].push(values[2])?,
                    ]
                    .map(|value| value + 0.0),
                    [
                        extra[1].push(values[3])?,
                        extra[2].push(values[4])?,
                        self.means[2].push(values[5])?,
                    ]
                    .map(|value| value + 0.0),
                )
            }
        };
        self.previous_pose = pose;
        self.previous_time = Some(time.sim_time);
        let diff = time
            .sim_time
            .checked_sub(self.last_publication)
            .ok_or(GroundTruthOdometryError::Time)?;
        if !diff.is_zero() && diff < self.period {
            return Ok(None);
        }
        self.revision = self
            .revision
            .checked_add(1)
            .ok_or(GroundTruthOdometryError::Revision)?;
        self.last_publication = time.sim_time;
        let mut published = pose;
        if self.publication.dimensions == GroundTruthOdometryDimensions::Two {
            published.translation[2] = 0.0;
        }
        Ok(Some(GroundTruthOdometrySample {
            time: time.sim_time,
            epoch: time.epoch,
            revision: self.revision,
            pose: published,
            linear,
            angular,
        }))
    }
}
fn sample(world: &mut World) {
    let time = *world.resource::<UpdateInfo>();
    if time.paused {
        return;
    }
    let entities: Vec<_> = world
        .iter_entities()
        .filter(|e| e.contains::<Publisher>())
        .map(|e| e.id())
        .collect();
    for entity in entities {
        let owner = world
            .get::<Publisher>(entity)
            .expect("selected publisher")
            .owner;
        let reading = (|| {
            let publisher = world.get::<Publisher>(entity).expect("selected publisher");
            if world.get::<EntityId>(entity).map(EntityId::get) != Some(owner)
                || world.get::<EntityKind>(entity) != Some(&EntityKind::Model)
                || world.get::<GroundTruthOdometryPublication>(entity)
                    != Some(&publisher.publication)
            {
                return Err(GroundTruthOdometryError::Identity);
            }
            let pose = world
                .get::<WorldPose>(entity)
                .copied()
                .ok_or(GroundTruthOdometryError::Pose)?;
            world
                .get_mut::<Publisher>(entity)
                .expect("selected publisher")
                .advance(pose, time)
        })();
        match reading {
            Ok(Some(value)) => {
                world.entity_mut(entity).insert(value);
            }
            Ok(None) => {}
            Err(source) => {
                world
                    .entity_mut(entity)
                    .remove::<GroundTruthOdometrySample>();
                world
                    .resource::<StepFault>()
                    .report(GroundTruthOdometryScheduleFault {
                        entity: owner,
                        source,
                    });
                return;
            }
        }
    }
}
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>()
        || !sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>())
    {
        return;
    }
    sim.add_systems(Phase::PostStep, sample);
    sim.add_systems(Phase::Reset, reset);
    sim.world_mut().insert_resource(Installed);
}
pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>()
        || sim
            .world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>())
    {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PostStep, sample)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}
fn reset(mut commands: Commands, publishers: Query<Entity, With<Publisher>>) {
    for entity in &publishers {
        commands
            .entity(entity)
            .remove::<GroundTruthOdometrySample>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn metadata() -> GroundTruthOdometryPublication {
        GroundTruthOdometryPublication {
            dimensions: GroundTruthOdometryDimensions::Two,
            model_name: "rig".into(),
            odom_frame: "rig/odom".into(),
            robot_base_frame: "rig/base".into(),
            odom_topic: "/odom".into(),
            covariance_topic: "/cov".into(),
            tf_topic: "/tf".into(),
        }
    }
    fn clock(nanos: u64) -> UpdateInfo {
        UpdateInfo {
            start_time: Duration::ZERO,
            sim_time: Duration::from_nanos(nanos),
            dt: Duration::ZERO,
            iteration: 0,
            epoch: 3,
            paused: false,
        }
    }
    fn pose(x: f64, yaw: f64) -> WorldPose {
        WorldPose {
            translation: [x, 0.0, 3.0],
            rotation: [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()],
        }
    }
    #[test]
    fn zero_time_initializes_then_microsecond_suppression_preserves_identity_history() {
        let mut p = Publisher::new(1, metadata(), Duration::ZERO);
        assert!(p.advance(pose(2.0, 0.0), clock(0)).unwrap().is_none());
        for n in [0, 1, 1000] {
            assert!(p.advance(pose(7.0, 0.0), clock(n)).unwrap().is_none());
        }
        let first = p.advance(pose(2.0, 0.0), clock(2000)).unwrap().unwrap();
        assert_eq!(first.linear, [1_000_000.0, 0.0, 0.0]);
        assert_eq!(first.pose.translation, [2.0, 0.0, 0.0]);
        assert_eq!((first.epoch, first.revision), (3, 1));
        for n in 2..=10 {
            let value = p.advance(pose(2.0, 0.0), clock(n * 2000)).unwrap().unwrap();
            assert_eq!(value.linear[0], 1_000_000.0 / n as f64);
        }
        assert_eq!(
            p.advance(pose(2.0, 0.0), clock(22000))
                .unwrap()
                .unwrap()
                .linear,
            [0.0; 3]
        );
    }
    #[test]
    fn world_negative_zero_rotation_is_composed_before_history_and_publication() {
        let mut publisher = Publisher::new(1, metadata(), Duration::ZERO);
        // This is WORLD input directly, without the fixture's parent conversion.
        let world_pose = WorldPose {
            translation: [2.0, -1.0, 3.0],
            rotation: [-0.0, -0.0, 0.0, 1.0],
        };
        assert!(publisher.advance(world_pose, clock(0)).unwrap().is_none());
        assert_eq!(publisher.previous_pose.translation, [0.0; 3]);
        let sample = publisher
            .advance(world_pose, clock(1_000_000))
            .unwrap()
            .unwrap();
        let expected_rotation = [0.0_f64, 0.0, 0.0, 1.0].map(f64::to_bits);
        assert_eq!(sample.pose.rotation.map(f64::to_bits), expected_rotation);
        assert_eq!(
            publisher.previous_pose.rotation.map(f64::to_bits),
            expected_rotation
        );
        assert_eq!(
            publisher.previous_pose.translation.map(f64::to_bits),
            world_pose.translation.map(f64::to_bits)
        );
        assert_eq!(sample.pose.translation, [2.0, -1.0, 0.0]);
        assert_eq!(sample.linear, [2000.0, -1000.0, 0.0]);
        assert_eq!(publisher.previous_time, Some(Duration::from_millis(1)));
        assert_eq!(publisher.revision, 1);
        assert!(publisher.means.iter().all(|mean| mean.count == 1));
        let held = publisher.advance(world_pose, clock(1_001_000)).unwrap();
        assert!(held.is_none());
        assert_eq!(
            publisher.previous_pose.rotation.map(f64::to_bits),
            expected_rotation
        );
        assert!(publisher.means.iter().all(|mean| mean.count == 1));
    }
    #[test]
    fn history_advances_before_throttle_and_means_recompute_oldest_first() {
        let mut p = Publisher::new(1, metadata(), Duration::from_millis(20));
        p.advance(pose(0.0, 0.0), clock(0)).unwrap();
        for n in 1..20 {
            assert!(
                p.advance(pose(n as f64 * 0.001, 0.0), clock(n * 1_000_000))
                    .unwrap()
                    .is_none()
            );
        }
        let value = p
            .advance(pose(0.02, 0.0), clock(20_000_000))
            .unwrap()
            .unwrap();
        assert!((value.linear[0] - 1.0).abs() < 1e-14);
        assert_eq!(p.means[0].count, 10);
        assert_eq!(value.time, Duration::from_millis(20));
        assert!(
            p.advance(pose(0.021, 0.0), clock(21_000_000))
                .unwrap()
                .is_none()
        );
        assert!(
            p.advance(pose(0.041, 0.0), clock(41_000_000))
                .unwrap()
                .is_some()
        );
        let mut mean = Mean::default();
        for v in [1e16, 1.0, -1e16] {
            mean.push(v).unwrap();
        }
        assert_eq!(mean.values[..3], [1e16, 1.0, -1e16]);
        assert_eq!(mean.push(1.0).unwrap(), 0.25);
    }
    #[test]
    fn body_float_trigonometry_and_quaternion_continuity_are_not_euler_differences() {
        let before = pose(0.0, 0.6);
        let mut after = before;
        after.translation = [0.002, 0.003, 9.0];
        let velocity = velocities(before, after, 0.001).unwrap();
        let sin = (0.6_f64 as f32).sin() as f64;
        let cos = (0.6_f64 as f32).cos() as f64;
        assert!((velocity[0] - (cos * 0.002 + sin * 0.003) / 0.001).abs() < 1e-12);
        assert!((velocity[1] - (cos * 0.003 - sin * 0.002) / 0.001).abs() < 1e-12);
        assert_eq!(velocity[2], 0.0);
        let before = pose(0.0, std::f64::consts::PI - 0.01);
        let after = pose(0.0, std::f64::consts::PI + 0.01);
        let expected = velocities(before, after, 0.02).unwrap();
        let mut negated = after;
        negated.rotation = after.rotation.map(|v| -v);
        assert_eq!(velocities(before, negated, 0.02).unwrap(), expected);
        assert!((expected[2] - 1.0).abs() < 1e-10);
        let tilt = WorldPose {
            translation: [0.0; 3],
            rotation: DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2).to_array(),
        };
        assert!(
            velocities(tilt, tilt, 0.01)
                .unwrap()
                .into_iter()
                .all(|v| v.abs() < 1e-12)
        );
    }
    #[test]
    fn invalid_math_clock_and_revision_are_typed_without_clamping() {
        assert!(!valid_pose(WorldPose {
            translation: [f64::NAN, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0]
        }));
        assert!(!valid_pose(WorldPose {
            translation: [0.0; 3],
            rotation: [0.0; 4]
        }));
        let mut p = Publisher::new(1, metadata(), Duration::ZERO);
        p.advance(pose(0.0, 0.0), clock(10_000)).unwrap();
        assert_eq!(
            p.advance(pose(0.0, 0.0), clock(0)).unwrap_err(),
            GroundTruthOdometryError::Time
        );
        let mut bad = pose(0.0, 0.0);
        bad.rotation = [0.0; 4];
        assert_eq!(
            p.advance(bad, clock(20_000)).unwrap_err(),
            GroundTruthOdometryError::Pose
        );
        p.revision = u64::MAX;
        assert_eq!(
            p.advance(pose(0.0, 0.0), clock(20_000)).unwrap_err(),
            GroundTruthOdometryError::Revision
        );
        let mut mean = Mean::default();
        mean.push(f64::MAX).unwrap();
        assert_eq!(
            mean.push(f64::MAX).unwrap_err(),
            GroundTruthOdometryError::Arithmetic
        );
        assert_eq!(
            velocities(pose(-f64::MAX, 0.0), pose(f64::MAX, 0.0), 0.001).unwrap_err(),
            GroundTruthOdometryError::Arithmetic
        );
        for rate in [0.0, -1.0, f64::MAX] {
            assert_eq!(period(rate), Some(Duration::ZERO));
        }
        for rate in [f64::NAN, f64::INFINITY, f64::MIN_POSITIVE] {
            assert_eq!(period(rate), None);
        }
        assert_eq!(period(50.0), Some(Duration::from_millis(20)));
    }
    #[test]
    fn absence_retirement_and_live_metadata_fault_do_not_republish_stale_readings() {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        install(&mut sim);
        retire_if_unused(&mut sim).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
        let entity = sim
            .world_mut()
            .spawn((EntityKind::Model, metadata(), pose(0.0, 0.0)))
            .id();
        let id = gz_sim_core::assign_id(sim.world_mut(), entity).unwrap();
        sim.world_mut()
            .entity_mut(entity)
            .insert(Publisher::new(id, metadata(), Duration::ZERO));
        install(&mut sim);
        install(&mut sim);
        sim.step_exact(2).unwrap();
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(entity)
                .is_some()
        );
        sim.world_mut()
            .get_mut::<GroundTruthOdometryPublication>(entity)
            .unwrap()
            .odom_frame = "changed".into();
        assert_eq!(
            sim.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(entity)
                .is_none()
        );
        let fault = sim
            .world()
            .resource::<StepFault>()
            .error()
            .unwrap()
            .downcast_ref::<GroundTruthOdometryScheduleFault>()
            .unwrap();
        assert_eq!(
            (fault.entity, fault.source),
            (id, GroundTruthOdometryError::Identity)
        );
        sim.world_mut().despawn(entity);
        retire_if_unused(&mut sim).unwrap();
        assert!(!sim.world().contains_resource::<Installed>());
    }
    fn configured(contents: &str) -> Model {
        let mut model = sdformat::load_str(
            include_str!("../../../examples/m34_odometry_publisher.sdf"),
            "m34.sdf",
            &sdformat::NoResources,
        )
        .unwrap()
        .worlds
        .remove(0)
        .models
        .remove(0);
        model.plugins[0].xml = format!(
            "<plugin name='{}' filename='{}'>{contents}</plugin>",
            model.plugins[0].name, model.plugins[0].filename
        );
        model
    }
    #[test]
    fn closed_configuration_keeps_zero_defaults_and_rejects_scalar_smuggling_or_unsupported_fields()
    {
        let plan = parse(&configured("")).unwrap().unwrap();
        assert_eq!(plan.period, Duration::from_millis(20));
        assert_eq!(plan.publication.odom_topic, "/model/rig/odometry");
        assert_eq!(
            plan.publication.covariance_topic,
            "/model/rig/odometry_with_covariance"
        );
        assert_eq!(plan.publication.tf_topic, "/model/rig/pose");
        assert_eq!(plan.publication.robot_base_frame, "rig/base_footprint");
        assert_eq!(
            plan.publication.dimensions,
            GroundTruthOdometryDimensions::Two
        );
        assert_eq!(
            parse(&configured("<dimensions>3</dimensions>"))
                .unwrap()
                .unwrap()
                .publication
                .dimensions,
            GroundTruthOdometryDimensions::Three
        );
        let positive = "<!-- root --> <dimensions> 2 </dimensions><gaussian_noise>-0</gaussian_noise><xyz_offset>0 0 0</xyz_offset><rpy_offset>0 0 0</rpy_offset><xyz_global_offset>0 0 0</xyz_global_offset><rpy_global_offset>0 0 0</rpy_global_offset><odom_publish_frequency>90909090.90909092</odom_publish_frequency>";
        assert_eq!(
            parse(&configured(positive)).unwrap().unwrap().period,
            Duration::from_nanos(11)
        );
        let mut absent = configured("");
        absent.plugins.clear();
        assert!(parse(&absent).unwrap().is_none());
        for content in [
            "<dimensions>4</dimensions>",
            "<dimensions>bad</dimensions>",
            "<dimensions>2</dimensions><dimensions>2</dimensions>",
            "<gaussian_noise>NaN</gaussian_noise>",
            "<gaussian_noise>bad</gaussian_noise>",
            "<gaussian_noise>1</gaussian_noise>",
            "<xyz_offset>0 0</xyz_offset>",
            "<xyz_offset>0 0 1</xyz_offset>",
            "<xyz_offset>0 0 bad</xyz_offset>",
            "<odom_publish_frequency>inf</odom_publish_frequency>",
            "<odom_publish_frequency>bad</odom_publish_frequency>",
            "<odom_frame> </odom_frame>",
            "<robot_base_frame>a b</robot_base_frame>",
            "<tf_topic>/model/rig/odometry</tf_topic>",
            "<odom_topic>//bad</odom_topic>",
            "<odom_topic>relative</odom_topic>",
            "<unknown>0</unknown>",
            "<dimensions>2<!-- gap -->garbage</dimensions>",
            "<dimensions><nested/></dimensions>",
            "<dimensions><![CDATA[2]]></dimensions>",
            "<dimensions x='2'>2</dimensions>",
            "<?bad value?>",
        ] {
            assert!(parse(&configured(content)).is_err(), "{content}");
        }
        for root in [
            "<plugin/>",
            "<other/>",
            "<plugin name='gz::sim::systems::OdometryPublisher' filename='gz-sim-odometry-publisher-system' x='1'/>",
            "<plugin xmlns='urn:other'/>",
            "<",
        ] {
            let mut model = configured("");
            model.plugins[0].xml = root.into();
            assert!(parse(&model).is_err());
        }
        let mut duplicate = configured("");
        duplicate.plugins.push(duplicate.plugins[0].clone());
        assert!(parse(&duplicate).is_err());
        let mut oversized = configured("");
        oversized.plugins[0].xml = " ".repeat(65537);
        assert!(parse(&oversized).is_err());
        let mut scoped = configured("");
        scoped.name = "nested::rig".into();
        assert!(parse(&scoped).is_err());
    }

    #[test]
    fn actual_native_inputs_replay_exact_bits_through_private_estimator_and_pause_dispatch() {
        mod reference {
            include!("../tests/fixtures/m34_native_odometry.rs");
        }
        use reference::Publication;
        let modes: Vec<_> = reference::CASES.iter().map(|(mode, _, _)| *mode).collect();
        assert_eq!(
            modes,
            [
                "identity", "initial", "motion", "rate", "micro", "paused", "delayed", "tilt",
                "sign"
            ]
        );
        let mut inputs = 0;
        let mut observed = 0;
        let mut unobserved = 0;
        let mut suppressed = 0;
        let bits = |sample: GroundTruthOdometrySample| {
            (
                [
                    sample.pose.translation[0].to_bits(),
                    sample.pose.translation[1].to_bits(),
                    sample.pose.translation[2].to_bits(),
                    sample.pose.rotation[0].to_bits(),
                    sample.pose.rotation[1].to_bits(),
                    sample.pose.rotation[2].to_bits(),
                    sample.pose.rotation[3].to_bits(),
                ],
                sample.linear.map(f64::to_bits),
                sample.angular.map(f64::to_bits),
            )
        };
        for &(mode, period_ns, ticks) in reference::CASES {
            let publication = metadata();
            let mut world = World::new();
            world.insert_resource(StepFault::default());
            let entity = world.spawn((EntityKind::Model, publication.clone())).id();
            let owner = gz_sim_core::assign_id(&mut world, entity).unwrap();
            let mut scalar = Publisher::new(owner, publication, Duration::from_nanos(period_ns));
            world.entity_mut(entity).insert(scalar.clone());
            let mut iteration = 0;
            for &(time_ns, dt_ns, paused, input_bits, expected) in ticks {
                inputs += 1;
                let input = input_bits.map(f64::from_bits);
                let model_pose = WorldPose {
                    translation: [input[0], input[1], input[2]],
                    rotation: [input[3], input[4], input[5], input[6]],
                };
                // The oracle records a MODEL Pose component. Native Util::worldPose
                // left-composes its identity parent's Pose before UpdateOdometry.
                // Both paths below receive WORLD input, as production requires.
                let pose = native_pose_mul(native_zero_offset(), model_pose);
                assert!(
                    valid_pose(pose),
                    "invalid actual reference input: {mode}/{time_ns}"
                );
                let sim_time = Duration::from_nanos(time_ns);
                let dt = Duration::from_nanos(dt_ns);
                let info = UpdateInfo {
                    start_time: sim_time.checked_sub(dt).unwrap(),
                    sim_time,
                    dt,
                    iteration,
                    epoch: 7,
                    paused,
                };
                if !paused {
                    iteration += 1;
                }
                // Publisher::advance's private contract is unpaused input. In parallel,
                // exercise the real production dispatcher on EVERY row, including both
                // paused calls: an accidental paused advance is not hidden by this test.
                let actual = if paused {
                    None
                } else {
                    scalar.advance(pose, info).unwrap()
                };
                let previous = world.get::<Publisher>(entity).unwrap().revision;
                world.entity_mut(entity).insert(pose);
                world.insert_resource(info);
                sample(&mut world);
                assert!(
                    world.resource::<StepFault>().error().is_none(),
                    "dispatch fault: {mode}/{time_ns}"
                );
                let state = world.get::<Publisher>(entity).unwrap();
                assert_eq!(
                    state.previous_time, scalar.previous_time,
                    "history clock: {mode}/{time_ns}"
                );
                assert_eq!(
                    state.previous_pose.translation.map(f64::to_bits),
                    scalar.previous_pose.translation.map(f64::to_bits)
                );
                assert_eq!(
                    state.previous_pose.rotation.map(f64::to_bits),
                    scalar.previous_pose.rotation.map(f64::to_bits)
                );
                assert_eq!(
                    (state.last_publication, state.revision),
                    (scalar.last_publication, scalar.revision)
                );
                for (dispatch, direct) in state.means.iter().zip(&scalar.means) {
                    assert_eq!(dispatch.count, direct.count);
                    assert_eq!(
                        dispatch.values.map(f64::to_bits),
                        direct.values.map(f64::to_bits)
                    );
                }
                let dispatched = (state.revision != previous)
                    .then(|| *world.get::<GroundTruthOdometrySample>(entity).unwrap());
                assert_eq!(
                    dispatched.map(bits),
                    actual.map(bits),
                    "direct/dispatch: {mode}/{time_ns}"
                );
                match expected {
                    Publication::Suppressed => {
                        suppressed += 1;
                        assert!(
                            actual.is_none(),
                            "native suppressed publication: {mode}/{time_ns}"
                        );
                    }
                    Publication::Unobserved => {
                        unobserved += 1;
                        assert_eq!(mode, "delayed");
                        assert!(!paused && time_ns > 0 && time_ns < 16_000_000);
                        // This is the documented Rust no-demand policy, not a fabricated
                        // observed native packet. The later actual packets bind history.
                        assert!(
                            actual.is_some(),
                            "Rust history must continue without demand"
                        );
                    }
                    Publication::Observed((pose_bits, linear_bits, angular_bits, _wire_bytes)) => {
                        observed += 1;
                        let actual = actual.unwrap_or_else(|| {
                            panic!("missing native-time sample: {mode}/{time_ns}")
                        });
                        assert_eq!((actual.time, actual.epoch), (sim_time, 7));
                        assert_eq!(
                            bits(actual),
                            (pose_bits, linear_bits, angular_bits),
                            "actual native numeric bits: {mode}/{time_ns}"
                        );
                    }
                }
            }
        }
        assert_eq!(
            (inputs, observed, unobserved, suppressed),
            (301, 221, 15, 65)
        );
    }

    fn spatial_metadata() -> GroundTruthOdometryPublication {
        GroundTruthOdometryPublication {
            dimensions: GroundTruthOdometryDimensions::Three,
            ..metadata()
        }
    }
    fn spatial_pose(position: [f64; 3]) -> WorldPose {
        WorldPose {
            translation: position,
            rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
    #[test]
    fn spatial_startup_retains_z_and_six_independent_ten_sample_histories() {
        let mut publisher = Publisher::new(1, spatial_metadata(), Duration::ZERO);
        let input = spatial_pose([2.0, -1.0, 3.0]);
        assert!(publisher.advance(input, clock(0)).unwrap().is_none());
        for nanos in [0, 1, 1000] {
            assert!(
                publisher
                    .advance(spatial_pose([9.0; 3]), clock(nanos))
                    .unwrap()
                    .is_none()
            );
        }
        assert_eq!(publisher.previous_pose.translation, [0.0; 3]);
        let first = publisher.advance(input, clock(1_000_000)).unwrap().unwrap();
        assert_eq!(first.pose.translation, [2.0, -1.0, 3.0]);
        assert_eq!(first.linear, [2000.0, -1000.0, 3000.0]);
        assert_eq!(first.angular, [0.0; 3]);
        for count in 2..=10 {
            let sample = publisher
                .advance(input, clock(count * 1_000_000))
                .unwrap()
                .unwrap();
            assert_eq!(
                sample.linear,
                first.linear.map(|value| value / count as f64)
            );
            assert_eq!(sample.angular, [0.0; 3]);
        }
        assert_eq!(
            publisher
                .advance(input, clock(11_000_000))
                .unwrap()
                .unwrap()
                .linear,
            [0.0; 3]
        );
        assert!(publisher.means.iter().all(|mean| mean.count == 10));
        assert!(
            publisher
                .spatial_means
                .as_ref()
                .unwrap()
                .iter()
                .all(|mean| mean.count == 10)
        );
        assert!(
            Publisher::new(1, metadata(), Duration::ZERO)
                .spatial_means
                .is_none()
        );
    }
    #[test]
    fn spatial_throttle_preserves_all_six_histories_and_current_body_rotation() {
        let mut every = Publisher::new(1, spatial_metadata(), Duration::ZERO);
        let mut limited = Publisher::new(1, spatial_metadata(), Duration::from_millis(20));
        let mut input = spatial_pose([0.0; 3]);
        every.advance(input, clock(0)).unwrap();
        limited.advance(input, clock(0)).unwrap();
        for tick in 1..=20 {
            input.translation = [
                tick as f64 * 0.001,
                tick as f64 * -0.002,
                tick as f64 * 0.003,
            ];
            input.rotation = DQuat::from_euler(
                glam::EulerRot::XYZ,
                tick as f64 * 0.002,
                tick as f64 * 0.003,
                tick as f64 * 0.004,
            )
            .to_array();
            let observed = every
                .advance(input, clock(tick * 1_000_000))
                .unwrap()
                .unwrap();
            let throttled = limited.advance(input, clock(tick * 1_000_000)).unwrap();
            if tick < 20 {
                assert!(throttled.is_none());
            } else {
                let throttled = throttled.unwrap();
                assert_eq!(
                    throttled.linear.map(f64::to_bits),
                    observed.linear.map(f64::to_bits)
                );
                assert_eq!(
                    throttled.angular.map(f64::to_bits),
                    observed.angular.map(f64::to_bits)
                );
                assert!(throttled.angular.into_iter().all(|value| value != 0.0));
                assert_eq!(throttled.revision, 1);
            }
        }
        let same = spatial_velocities(input, input, 0.001).unwrap();
        assert_eq!(same, [0.0; 6]);
        let mut signed = input;
        signed.rotation = signed.rotation.map(|value| -value);
        assert_eq!(spatial_velocities(input, signed, 0.001).unwrap(), same);
        let previous = spatial_pose([0.0; 3]);
        let tiny = WorldPose {
            rotation: [1e-7, 0.0, 0.0, (1.0_f64 - 1e-14).sqrt()],
            ..previous
        };
        assert_eq!(
            &spatial_velocities(previous, tiny, 0.001).unwrap()[3..],
            &[0.0; 3]
        );
    }
    #[test]
    fn spatial_mode_is_owned_metadata_and_invalid_math_removes_stale_sample() {
        let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
        let entity = sim
            .world_mut()
            .spawn((
                EntityKind::Model,
                spatial_metadata(),
                spatial_pose([0.0; 3]),
            ))
            .id();
        let id = gz_sim_core::assign_id(sim.world_mut(), entity).unwrap();
        sim.world_mut().entity_mut(entity).insert(Publisher::new(
            id,
            spatial_metadata(),
            Duration::ZERO,
        ));
        install(&mut sim);
        sim.step_exact(2).unwrap();
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(entity)
                .is_some()
        );
        sim.world_mut()
            .get_mut::<GroundTruthOdometryPublication>(entity)
            .unwrap()
            .dimensions = GroundTruthOdometryDimensions::Two;
        assert_eq!(
            sim.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        assert!(
            sim.world()
                .get::<GroundTruthOdometrySample>(entity)
                .is_none()
        );
        let fault = sim
            .world()
            .resource::<StepFault>()
            .error()
            .unwrap()
            .downcast_ref::<GroundTruthOdometryScheduleFault>()
            .unwrap();
        assert_eq!(
            (fault.entity, fault.source),
            (id, GroundTruthOdometryError::Identity)
        );

        let mut publisher = Publisher::new(1, spatial_metadata(), Duration::ZERO);
        publisher.advance(spatial_pose([0.0; 3]), clock(0)).unwrap();
        let bad = WorldPose {
            translation: [f64::NAN, 0.0, 0.0],
            ..spatial_pose([0.0; 3])
        };
        assert_eq!(
            publisher.advance(bad, clock(1_000_000)).unwrap_err(),
            GroundTruthOdometryError::Pose
        );
        assert_eq!(publisher.previous_time, Some(Duration::ZERO));
        assert_eq!(
            spatial_velocities(
                spatial_pose([-f64::MAX; 3]),
                spatial_pose([f64::MAX; 3]),
                0.001
            )
            .unwrap_err(),
            GroundTruthOdometryError::Arithmetic
        );
        // A finite nonunit delta can make acos invalid; no normalization or
        // clamping converts this private arithmetic failure to a reading.
        let nonunit = WorldPose {
            rotation: [0.1, 0.0, 0.0, 2.0],
            ..spatial_pose([0.0; 3])
        };
        assert_eq!(
            spatial_velocities(spatial_pose([0.0; 3]), nonunit, 0.001).unwrap_err(),
            GroundTruthOdometryError::Arithmetic
        );
        publisher.spatial_means = None;
        assert_eq!(
            publisher
                .advance(spatial_pose([0.0; 3]), clock(1_000_000))
                .unwrap_err(),
            GroundTruthOdometryError::Identity
        );
    }

    #[test]
    fn actual_three_dimensional_native_inputs_match_scalar_and_dispatcher_bits() {
        mod reference {
            include!("../tests/fixtures/m38_native_odometry_3d.rs");
        }
        use reference::Publication;
        let modes: Vec<_> = reference::CASES.iter().map(|(mode, _, _)| *mode).collect();
        assert_eq!(
            modes,
            [
                "identity",
                "stationary_tilted",
                "translate_x",
                "translate_y",
                "translate_z",
                "rotate_roll",
                "rotate_pitch",
                "rotate_yaw",
                "tilted_screw",
                "quaternion_sign",
                "angular_threshold",
                "rate",
                "micro",
                "pause_equal",
                "reload/0",
                "reload/1"
            ]
        );
        let mut inputs = 0;
        let mut observed = 0;
        let mut suppressed = 0;
        let bits = |sample: GroundTruthOdometrySample| {
            (
                [
                    sample.pose.translation[0].to_bits(),
                    sample.pose.translation[1].to_bits(),
                    sample.pose.translation[2].to_bits(),
                    sample.pose.rotation[0].to_bits(),
                    sample.pose.rotation[1].to_bits(),
                    sample.pose.rotation[2].to_bits(),
                    sample.pose.rotation[3].to_bits(),
                ],
                sample.linear.map(f64::to_bits),
                sample.angular.map(f64::to_bits),
            )
        };
        for (case_index, &(mode, period_ns, ticks)) in reference::CASES.iter().enumerate() {
            let epoch = 7 + case_index as u64;
            let publication = spatial_metadata();
            let mut world = World::new();
            world.insert_resource(StepFault::default());
            let entity = world.spawn((EntityKind::Model, publication.clone())).id();
            let owner = gz_sim_core::assign_id(&mut world, entity).unwrap();
            let mut scalar = Publisher::new(owner, publication, Duration::from_nanos(period_ns));
            world.entity_mut(entity).insert(scalar.clone());
            let mut iteration = 0;
            for &(time_ns, dt_ns, paused, input_bits, expected) in ticks {
                inputs += 1;
                let input = input_bits.map(f64::from_bits);
                let model_pose = WorldPose {
                    translation: [input[0], input[1], input[2]],
                    rotation: [input[3], input[4], input[5], input[6]],
                };
                // The oracle records a MODEL Pose component. Native Util::worldPose
                // left-composes its identity parent's Pose before UpdateOdometry.
                // Both paths below receive WORLD input, as production requires.
                let pose = native_pose_mul(native_zero_offset(), model_pose);
                assert!(
                    valid_pose(pose),
                    "invalid actual reference input: {mode}/{time_ns}"
                );
                let sim_time = Duration::from_nanos(time_ns);
                let dt = Duration::from_nanos(dt_ns);
                let info = UpdateInfo {
                    start_time: sim_time.checked_sub(dt).unwrap(),
                    sim_time,
                    dt,
                    iteration,
                    epoch,
                    paused,
                };
                if !paused {
                    iteration += 1;
                }
                // Publisher::advance's private contract is unpaused input. In parallel,
                // exercise the real production dispatcher on EVERY row, including both
                // paused calls: an accidental paused advance is not hidden by this test.
                let actual = if paused {
                    None
                } else {
                    scalar.advance(pose, info).unwrap()
                };
                let previous = world.get::<Publisher>(entity).unwrap().revision;
                let held = world.get::<GroundTruthOdometrySample>(entity).copied();
                world.entity_mut(entity).insert(pose);
                world.insert_resource(info);
                sample(&mut world);
                assert!(
                    world.resource::<StepFault>().error().is_none(),
                    "dispatch fault: {mode}/{time_ns}"
                );
                let state = world.get::<Publisher>(entity).unwrap();
                assert_eq!(
                    state.previous_time, scalar.previous_time,
                    "history clock: {mode}/{time_ns}"
                );
                assert_eq!(
                    state.previous_pose.translation.map(f64::to_bits),
                    scalar.previous_pose.translation.map(f64::to_bits)
                );
                assert_eq!(
                    state.previous_pose.rotation.map(f64::to_bits),
                    scalar.previous_pose.rotation.map(f64::to_bits)
                );
                assert_eq!(
                    (state.last_publication, state.revision),
                    (scalar.last_publication, scalar.revision)
                );
                for (dispatch, direct) in state.means.iter().zip(&scalar.means) {
                    assert_eq!(dispatch.count, direct.count);
                    assert_eq!(
                        dispatch.values.map(f64::to_bits),
                        direct.values.map(f64::to_bits)
                    );
                }
                for (dispatch, direct) in state
                    .spatial_means
                    .as_ref()
                    .unwrap()
                    .iter()
                    .zip(scalar.spatial_means.as_ref().unwrap())
                {
                    assert_eq!(dispatch.count, direct.count);
                    assert_eq!(
                        dispatch.values.map(f64::to_bits),
                        direct.values.map(f64::to_bits)
                    );
                }
                let dispatched = (state.revision != previous)
                    .then(|| *world.get::<GroundTruthOdometrySample>(entity).unwrap());
                assert_eq!(
                    dispatched.map(bits),
                    actual.map(bits),
                    "direct/dispatch: {mode}/{time_ns}"
                );
                match expected {
                    Publication::Suppressed => {
                        suppressed += 1;
                        assert!(
                            actual.is_none(),
                            "native suppressed publication: {mode}/{time_ns}"
                        );
                        assert_eq!(state.revision, previous);
                        assert_eq!(
                            world
                                .get::<GroundTruthOdometrySample>(entity)
                                .copied()
                                .map(bits),
                            held.map(bits)
                        );
                        assert_eq!(
                            world
                                .get::<GroundTruthOdometrySample>(entity)
                                .map(|s| (s.time, s.epoch, s.revision)),
                            held.map(|s| (s.time, s.epoch, s.revision))
                        );
                    }
                    Publication::Observed((pose_bits, linear_bits, angular_bits, _wire_bytes)) => {
                        observed += 1;
                        let actual = actual.unwrap_or_else(|| {
                            panic!("missing native-time sample: {mode}/{time_ns}")
                        });
                        assert_eq!((actual.time, actual.epoch), (sim_time, epoch));
                        assert_eq!(
                            bits(actual),
                            (pose_bits, linear_bits, angular_bits),
                            "actual native numeric bits: {mode}/{time_ns}"
                        );
                    }
                }
            }
        }
        assert_eq!((inputs, observed, suppressed), (510, 429, 81));
    }
}
