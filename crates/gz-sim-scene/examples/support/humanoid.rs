// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Floating humanoid model, smooth reach trajectory, and torque-limited actuators.
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_physics::{JointId, JointVelocityMotor, PhysicsBackend};
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, JointDefinition, JointHandle, Name, WorldPose};
use std::{collections::BTreeMap, error::Error, fmt::Write};

pub const REACH_SECONDS: f64 = 1.5;
pub const HAND_RADIUS: f64 = 0.045;

struct Segment {
    link: String,
    center: [f64; 3],
    size: [f64; 3],
    mass: f64,
    joint: String,
    parent: String,
    anchor: [f64; 3],
    axis: [u8; 3],
    angle: f64,
    torque: f64,
}

pub struct Humanoid {
    segments: Vec<Segment>,
}

const X: [u8; 3] = [1, 0, 0];
const Y: [u8; 3] = [0, 1, 0];
const Z: [u8; 3] = [0, 0, 1];
const PELVIS: [f64; 3] = [0., 0., 0.91];
const PELVIS_MASS: f64 = 10.;

// Neutral pose dimensions in metres; axis directions use +X forward, +Y left, +Z up.
// Each row describes a joint and its child in one serial chain. The columns are
// joint, child link, axis, bearing height, COM height, box size, kg, reach radians, Nm.
// The neutral left and right limbs are mirror images. Each hand uses a sphere.
struct Bearing(
    &'static str,
    &'static str,
    [u8; 3],
    f64,
    f64,
    [f64; 3],
    f64,
    f64,
    f64,
);

#[rustfmt::skip]
const TRUNK: &[Bearing] = &[
    Bearing("waist_yaw",   "lumbar",  Z, 0.990, 1.020, [0.21, 0.25, 0.07],  1.8, 0.00,  80.),
    Bearing("spine_roll",  "abdomen", X, 1.055, 1.090, [0.22, 0.26, 0.07],  3.0, 0.00,  80.),
    Bearing("spine_pitch", "thorax",  Y, 1.125, 1.250, [0.25, 0.36, 0.25], 15.0, 0.10, 110.),
    Bearing("neck_yaw",    "head",    Z, 1.390, 1.510, [0.19, 0.16, 0.23],  4.5, 0.00,  12.),
];

#[rustfmt::skip]
const LEG: &[Bearing] = &[
    Bearing("hip_yaw",     "hip_rotor", Z, 0.87, 0.870, [0.07, 0.07, 0.05], 0.60,  0.00,  70.),
    Bearing("hip_roll",    "hip_yoke",  X, 0.85, 0.850, [0.07, 0.07, 0.05], 0.60,  0.00,  85.),
    Bearing("hip_pitch",   "thigh",     Y, 0.84, 0.650, [0.13, 0.12, 0.38], 7.00, -0.12, 120.),
    Bearing("knee_pitch",  "shin",      Y, 0.46, 0.275, [0.10, 0.09, 0.37], 3.50,  0.24, 100.),
    Bearing("ankle_pitch", "ankle",     Y, 0.09, 0.080, [0.07, 0.07, 0.05], 0.45, -0.12,  55.),
    Bearing("ankle_roll",  "foot",      X, 0.07, 0.035, [0.24, 0.10, 0.07], 1.10,  0.00,  45.),
];

#[rustfmt::skip]
const ARM: &[Bearing] = &[
    Bearing("shoulder_pitch", "shoulder_rotor", Y, 1.365, 1.365, [0.055, 0.055, 0.045], 0.45, -0.65, 55.),
    Bearing("shoulder_roll",  "shoulder_yoke",  X, 1.355, 1.355, [0.055, 0.055, 0.045], 0.40, -0.20, 45.),
    Bearing("shoulder_yaw",   "upper_arm",      Z, 1.350, 1.205, [0.075, 0.080, 0.290], 2.10,  0.00, 28.),
    Bearing("elbow_flexion",  "forearm",        Y, 1.060, 0.950, [0.065, 0.070, 0.220], 1.25, -0.65, 35.),
    Bearing("forearm_roll",   "wrist_rotor",    Z, 0.840, 0.830, [0.050, 0.050, 0.040], 0.35,  0.00, 12.),
    Bearing("wrist_pitch",    "wrist",          Y, 0.810, 0.800, [0.050, 0.050, 0.035], 0.20,  0.20, 10.),
    Bearing("wrist_roll",     "hand",           X, 0.780, 0.740, [0.; 3],                 0.50,  0.00,  8.),
];

impl Humanoid {
    pub fn new() -> Self {
        let mut model = Self {
            segments: Vec::with_capacity(30),
        };
        model.chain("", 0., "pelvis", TRUNK);
        for (side, sign) in [("left_", 1.), ("right_", -1.)] {
            model.chain(side, sign * 0.10, "pelvis", LEG);
            model.chain(side, sign * 0.235, "thorax", ARM);
        }
        model
    }

    fn chain(&mut self, prefix: &str, y: f64, root: &str, chain: &[Bearing]) {
        let mut parent = root.to_owned();
        for &Bearing(joint, link, axis, anchor_z, center_z, size, mass, left_angle, torque) in chain
        {
            let angle = if y < 0. && axis != Y {
                -left_angle
            } else {
                left_angle
            };
            let center_x = if link == "foot" { 0.06 } else { 0. };
            let child = format!("{prefix}{link}");
            self.segments.push(Segment {
                link: child.clone(),
                center: [center_x, y, center_z],
                size,
                mass,
                joint: format!("{prefix}{joint}"),
                parent,
                anchor: [0., y, anchor_z],
                axis,
                angle,
                torque,
            });
            parent = child;
        }
    }

    /// Add tool mass at each hand's center. The link inertia is recomputed for
    /// that sphere and the same mass is used for center-of-mass measurements.
    pub fn with_hand_load(mut self, kilograms: f64) -> Self {
        assert!(kilograms.is_finite() && kilograms >= 0.);
        for link in &mut self.segments {
            if link.link.ends_with("_hand") {
                link.mass += kilograms;
            }
        }
        self
    }

    /// Generate the original gravity-free bilateral reach fixture.
    pub fn world_sdf(&self, world_children: &str) -> String {
        self.world([0.; 3], 0.001, true, world_children)
    }

    pub fn world(&self, gravity: [f64; 3], dt: f64, reduced: bool, world_children: &str) -> String {
        let [gx, gy, gz] = gravity;
        let mut xml = format!(
            r#"<sdf version="1.12"><world name="default"><gravity>{gx} {gy} {gz}</gravity>
<physics name="one_ms" type="ignored"><max_step_size>{dt}</max_step_size></physics>
<model name="humanoid">"#
        );
        if reduced {
            xml.push_str(r#"<plugin filename="gz-sim-switchable-articulation-system" name="gz::sim::systems::SwitchableArticulation"/>"#);
        }
        let mut link = |name: &str, [x, y, z]: [f64; 3], [a, b, c]: [f64; 3], m: f64| {
            let (geometry, [ix, iy, iz]) = if name.ends_with("_hand") {
                (
                    format!("<sphere><radius>{HAND_RADIUS}</radius></sphere>"),
                    [0.4 * m * HAND_RADIUS.powi(2); 3],
                )
            } else {
                (
                    format!("<box><size>{a} {b} {c}</size></box>"),
                    [
                        m * (b * b + c * c) / 12.,
                        m * (a * a + c * c) / 12.,
                        m * (a * a + b * b) / 12.,
                    ],
                )
            };
            writeln!(xml, r#"<link name="{name}"><pose>{x} {y} {z} 0 0 0</pose>
<inertial><mass>{m}</mass><inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz></inertia></inertial>
<collision name="shape"><geometry>{geometry}</geometry></collision>
<visual name="body"><geometry>{geometry}</geometry></visual></link>"#).unwrap();
        };
        link("pelvis", PELVIS, [0.26, 0.32, 0.19], PELVIS_MASS);
        for s in &self.segments {
            link(&s.link, s.center, s.size, s.mass);
        }
        for s in &self.segments {
            let [x, y, z] = s.anchor;
            let [ax, ay, az] = s.axis;
            writeln!(
                xml,
                r#"<joint name="{}" type="revolute"><parent>{}</parent><child>{}</child>
<pose relative_to="__model__">{x} {y} {z} 0 0 0</pose>
<axis><xyz expressed_in="__model__">{ax} {ay} {az}</xyz></axis></joint>"#,
                s.joint, s.parent, s.link
            )
            .unwrap();
        }
        write!(xml, "</model>{world_children}</world></sdf>").unwrap();
        xml
    }

    /// Forward kinematics in the neutral pelvis frame. Each link map takes a
    /// point in the original SDF frame through the commanded parent/child
    /// rotation about the original bearing. No simulated pose enters the target.
    pub fn desired_hands(&self, time: f64) -> [DVec3; 2] {
        let (fraction, _) = trajectory(time);
        let mut transforms = BTreeMap::from([("pelvis", (DQuat::IDENTITY, DVec3::ZERO))]);
        let mut hands = [DVec3::ZERO; 2];
        for segment in &self.segments {
            let (parent_rotation, parent_translation) = transforms[segment.parent.as_str()];
            let axis = DVec3::from_array(segment.axis.map(f64::from));
            let rotation = parent_rotation * DQuat::from_axis_angle(axis, fraction * segment.angle);
            let anchor = DVec3::from_array(segment.anchor);
            let translation = parent_translation + parent_rotation * anchor - rotation * anchor;
            transforms.insert(&segment.link, (rotation, translation));
            if segment.link.ends_with("_hand") {
                let side = usize::from(segment.link.starts_with("right_"));
                hands[side] = translation + rotation * DVec3::from_array(segment.center)
                    - DVec3::from_array(PELVIS);
            }
        }
        hands
    }

    /// Resolve the model from its loaded scene, checking the native tree against the SDF design.
    pub fn bind(&self, sim: &Simulation) -> Controller {
        let prefix = "default::humanoid::";
        let entities: BTreeMap<_, _> = sim
            .world()
            .iter_entities()
            .filter_map(|e| Some((e.get::<Name>()?.0.strip_prefix(prefix)?, e.id())))
            .collect();
        assert_eq!(
            entities
                .values()
                .filter(|&&e| sim.world().get::<JointHandle>(e).is_some())
                .count(),
            30
        );
        assert_eq!(
            entities
                .values()
                .filter(|&&e| sim.world().get::<BodyHandle>(e).is_some())
                .count(),
            31
        );
        let mut drives = Vec::new();
        let mut links = vec![(entities["pelvis"], PELVIS_MASS)];
        for s in &self.segments {
            let entity = entities[s.joint.as_str()];
            let def = &sim.world().get::<JointDefinition>(entity).unwrap().0;
            assert_eq!((&def.parent, &def.child), (&s.parent, &s.link));
            let handle = sim.world().get::<JointHandle>(entity).unwrap();
            assert_eq!(
                sim.world()
                    .get::<gz_sim_core::EntityId>(entities[s.link.as_str()])
                    .unwrap()
                    .get(),
                handle.child()
            );
            drives.push((handle.id(), s.angle, s.torque));
            links.push((entities[s.link.as_str()], s.mass));
        }
        Controller {
            drives,
            links,
            pelvis: entities["pelvis"],
            hands: [entities["left_hand"], entities["right_hand"]],
        }
    }
}

pub struct Controller {
    drives: Vec<(JointId, f64, f64)>,
    links: Vec<(Entity, f64)>,
    pelvis: Entity,
    hands: [Entity; 2],
}

fn trajectory(time: f64) -> (f64, f64) {
    let u = (time / REACH_SECONDS).clamp(0., 1.);
    (
        u * u * u * (10. + u * (-15. + 6. * u)),
        30. * u * u * (1. - u).powi(2) / REACH_SECONDS,
    )
}

impl Controller {
    /// Apply feedforward reference velocity plus position feedback through bounded joint torque.
    /// Returns the maximum tracking error in radians at this committed instant.
    pub fn drive(&self, sim: &mut Simulation) -> Result<f64, Box<dyn Error>> {
        let (position, velocity) = trajectory(sim.elapsed().as_secs_f64());
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let mut worst = 0_f64;
        for &(joint, angle, torque) in &self.drives {
            let actual = backend.joint_state(joint).ok_or("reach joint missing")?;
            let error = position * angle - actual.position;
            assert!(error.is_finite() && actual.velocity.is_finite());
            worst = worst.max(error.abs());
            backend.set_joint_velocity_servo(
                joint,
                Some(JointVelocityMotor {
                    target_velocity: (velocity * angle + 10. * error).clamp(-2.5, 2.5),
                    max_force: torque,
                }),
            )?;
        }
        Ok(worst)
    }

    pub fn hold_residual(&self, sim: &Simulation) -> Result<(f64, f64), Box<dyn Error>> {
        let backend = sim.world().resource::<PhysicsBackend>();
        let mut residual = (0_f64, 0_f64);
        for &(id, angle, _) in &self.drives {
            let state = backend.joint_state(id).ok_or("reach joint missing")?;
            assert!(state.velocity.is_finite() && state.position.is_finite());
            residual = (
                residual.0.max((state.position - angle).abs()),
                residual.1.max(state.velocity.abs()),
            );
        }
        Ok(residual)
    }

    /// Aggregate linear momentum. The SDF centers every inertia at its link origin.
    pub fn momentum(&self, sim: &Simulation) -> DVec3 {
        self.links
            .iter()
            .map(|&(link, mass)| {
                mass * DVec3::from_array(
                    sim.world()
                        .get::<gz_sim_scene::Velocity>(link)
                        .unwrap()
                        .linear,
                )
            })
            .sum()
    }

    pub fn mass(&self) -> f64 {
        self.links.iter().map(|(_, mass)| mass).sum()
    }

    pub fn center_of_mass(&self, sim: &Simulation) -> DVec3 {
        let mut mass = 0.;
        let mut moment = DVec3::ZERO;
        for &(link, m) in &self.links {
            let pose = sim.world().get::<WorldPose>(link).unwrap();
            assert!(DQuat::from_array(pose.rotation).is_normalized());
            moment += m * DVec3::from_array(pose.translation);
            mass += m;
        }
        moment / mass
    }

    pub fn hands_in_pelvis(&self, sim: &Simulation) -> [DVec3; 2] {
        let pelvis = sim.world().get::<WorldPose>(self.pelvis).unwrap();
        self.hands.map(|hand| {
            let pose = sim.world().get::<WorldPose>(hand).unwrap();
            DQuat::from_array(pelvis.rotation).inverse()
                * (DVec3::from_array(pose.translation) - DVec3::from_array(pelvis.translation))
        })
    }
}

impl Default for Humanoid {
    fn default() -> Self {
        Self::new()
    }
}
