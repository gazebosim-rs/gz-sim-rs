// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3, EulerRot};
use serde::Deserialize;
use std::fmt::Display;

#[derive(Deserialize)]
pub(super) struct Donor {
    pub case: u32,
    pub stroke: u32,
    pub horizon_s: f64,
    pub drive: [f64; 3],
    pub deck: Deck,
    pub initial: Incoming,
    pub foot_half_extents_m: [f64; 3],
    pub upper_half_extents_m: [f64; 3],
}

#[derive(Deserialize)]
pub(super) struct Deck {
    pub center_m: [f64; 3],
    pub half_extents_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

#[derive(Deserialize)]
pub(super) struct Incoming {
    pub com: [f64; 3],
    pub foot: [f64; 3],
    pub upper: [f64; 3],
    pub velocity: [f64; 3],
    pub omega: [f64; 3],
    pub orientation: [f64; 4],
}

#[derive(Deserialize)]
pub(super) struct Material {
    pub foot_stiffness_n_m3: f64,
    pub deck_stiffness_n_m3: f64,
    pub mu: f64,
    pub slip_regularization_m_s: f64,
}

fn xyz(p: DVec3) -> impl Display {
    format!("{} {} {}", p.x, p.y, p.z)
}

fn pose(position: [f64; 3], orientation: [f64; 4]) -> String {
    let (yaw, pitch, roll) = DQuat::from_array(orientation).to_euler(EulerRot::ZYX);
    format!("{} {roll} {pitch} {yaw}", xyz(position.into()))
}

impl Donor {
    pub fn com_offset(&self) -> DVec3 {
        DQuat::from_array(self.initial.orientation).inverse()
            * (DVec3::from_array(self.initial.com) - DVec3::from_array(self.initial.foot))
    }

    pub fn xml(
        &self,
        m: &Material,
        damping: f64,
        substeps: u32,
        depth: u8,
        adaptive: bool,
    ) -> String {
        let refinement = if adaptive {
            "<max_substeps>1024</max_substeps><max_quadrature_depth>4</max_quadrature_depth>
            <position_tolerance>0.0000001</position_tolerance><rotation_tolerance>0.000001</rotation_tolerance>
            <impulse_tolerance>0.0001</impulse_tolerance><angular_impulse_tolerance>0.00001</angular_impulse_tolerance>
            <heat_tolerance>0.00002</heat_tolerance>"
        } else {
            ""
        };
        let start = &self.initial;
        let initial_pose = pose(start.foot, start.orientation);
        let deck_pose = pose(self.deck.center_m, self.deck.orientation_xyzw);
        let com = xyz(self.com_offset());
        let upper = xyz(DQuat::from_array(start.orientation).inverse()
            * (DVec3::from_array(start.upper) - DVec3::from_array(start.foot)));
        let foot_size = xyz(2. * DVec3::from(self.foot_half_extents_m));
        let upper_size = xyz(2. * DVec3::from(self.upper_half_extents_m));
        let deck_size = xyz(2. * DVec3::from(self.deck.half_extents_m));
        let drive = DVec3::from(self.drive);
        let linear = xyz(DVec3::from(start.velocity) + drive);
        let spin = xyz(start.omega.into());
        format!(
            r#"<sdf version="1.12"><world name="default">
  <gravity>0 0 -9.81</gravity><physics name="fixed" type="ignored"><max_step_size>0.0005</max_step_size></physics>
  <plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system">
    <foot>default::foot::body</foot><support>default::deck::body</support>
    <foot_stiffness>{}</foot_stiffness><deck_stiffness>{}</deck_stiffness>
    <damping>{damping}</damping><friction>{}</friction><slip_speed>{}</slip_speed>
    <substeps>{substeps}</substeps><quadrature_depth>{depth}</quadrature_depth>{refinement}
  </plugin>
  <model name="foot"><pose>{initial_pose}</pose>
    <plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system">
      <linear_velocity>{linear}</linear_velocity><angular_velocity>{spin}</angular_velocity></plugin>
    <link name="body"><inertial><pose>{com} 0 0 0</pose><mass>2</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial>
      <collision name="sole"><geometry><box><size>{foot_size}</size></box></geometry></collision>
      <visual name="foot"><geometry><box><size>{foot_size}</size></box></geometry></visual>
      <visual name="upper"><pose>{upper} 0 0 0</pose><geometry><box><size>{upper_size}</size></box></geometry></visual>
    </link></model>
  <model name="deck"><pose>{deck_pose}</pose>
    <plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system">
      <linear_velocity>{}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
    <link name="body"><collision name="top"><geometry><box><size>{deck_size}</size></box></geometry></collision>
      <visual name="deck"><geometry><box><size>{deck_size}</size></box></geometry></visual></link></model>
</world></sdf>"#,
            m.foot_stiffness_n_m3,
            m.deck_stiffness_n_m3,
            m.mu,
            m.slip_regularization_m_s,
            xyz(drive)
        )
    }
}
