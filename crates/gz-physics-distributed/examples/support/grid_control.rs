// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, RotationSpec, Shape};
use gz_physics_distributed::physics::Body;
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{collections::BTreeMap, error::Error, time::Duration};

pub struct Control {
    native: MultiFidelityBackend,
    ids: Vec<u64>,
}
pub type Pairs = BTreeMap<(u64, u64), f64>;
pub type Supports = BTreeMap<u64, f64>;

impl Control {
    pub fn new(bodies: &[Body], h: Duration) -> Result<Self, Box<dyn Error>> {
        let mut native = MultiFidelityBackend::new([0., 0., -9.81])?;
        native.set_partition_step(h)?;
        native.set_partition_ground_material(0., 0.5)?;
        let mut ids = Vec::new();
        for b in bodies {
            let id = native.spawn_partition_box(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: b.size.map(|s| s * 0.5),
                    },
                    position: b.position,
                    velocity: b.linear,
                    mass: Some(b.mass),
                },
                RotationSpec {
                    orientation: b.orientation,
                    angular_velocity: b.angular,
                },
                false,
                b.friction,
            )?;
            assert_eq!(id.raw(), ids.len() as u64);
            ids.push(b.id);
        }
        Ok(Self { native, ids })
    }
    pub fn step(&mut self, h: Duration) -> Result<(), Box<dyn Error>> {
        self.native.step(h)?;
        Ok(())
    }
    pub fn state(&self, index: usize) -> BodyState {
        self.native.state(BodyId::from_raw(index as u64)).unwrap()
    }
    pub fn contacts(&self, pairs: &mut Pairs, supports: &mut Supports) {
        for p in self
            .native
            .partition_ground_contacts()
            .into_iter()
            .filter(|p| p.active)
        {
            assert!(
                supports
                    .insert(self.ids[p.body.raw() as usize], p.normal_impulse)
                    .is_none()
            );
        }
        for p in self
            .native
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| p.active)
        {
            assert!(
                pairs
                    .insert(
                        (self.ids[p.a.raw() as usize], self.ids[p.b.raw() as usize]),
                        p.normal_impulse
                    )
                    .is_none()
            );
        }
    }
}
pub fn exact(b: &Body, n: BodyState) -> bool {
    b.position.map(f64::to_bits) == n.position.map(f64::to_bits)
        && b.linear.map(f64::to_bits) == n.velocity.map(f64::to_bits)
        && b.orientation.map(f64::to_bits) == n.orientation.map(f64::to_bits)
        && b.angular.map(f64::to_bits) == n.angular_velocity.map(f64::to_bits)
}
pub fn state_error(b: &Body, n: BodyState) -> [f64; 4] {
    let norm = |a: [f64; 3], b: [f64; 3]| (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]);
    let dot: f64 = b
        .orientation
        .iter()
        .zip(n.orientation)
        .map(|(x, y)| x * y)
        .sum();
    let half_chord = b
        .orientation
        .iter()
        .zip(n.orientation)
        .map(|(x, y)| (x - dot.signum() * y).powi(2))
        .sum::<f64>()
        .sqrt()
        * 0.5;
    [
        norm(b.position, n.position),
        norm(b.linear, n.velocity),
        4. * half_chord.min(1.).asin(),
        norm(b.angular, n.angular_velocity),
    ]
}
pub fn impulses<K: Ord + Copy>(a: &BTreeMap<K, f64>, b: &BTreeMap<K, f64>) -> (bool, f64) {
    let exact = a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|((i, j), (k, l))| i == k && j.to_bits() == l.to_bits());
    let peak = a
        .keys()
        .chain(b.keys())
        .map(|k| (a.get(k).unwrap_or(&0.) - b.get(k).unwrap_or(&0.)).abs())
        .fold(0., f64::max);
    (exact, peak)
}
