// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::BodyState;
use gz_physics_tpe::{Contacts, ModelBounds, Motion, install};
use gz_sim_core::Simulation;
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut sim = Simulation::new(Duration::from_millis(1))?;
    install(&mut sim)?;
    let mut entities = Vec::new();
    let mut ordinals = BTreeMap::new();
    for i in 0..1000 {
        let fixed = (i / 2) % 2 == 1;
        let x = (i / 2) as f64 * 3.0 + (i % 2) as f64 * 0.75;
        let entity = sim
            .world_mut()
            .spawn((
                Motion::new(BodyState {
                    position: [x, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    velocity: [if fixed { 0.0 } else { 0.01 }, 0.0, 0.0],
                    angular_velocity: [0.0; 3],
                })?,
                ModelBounds::new([-0.5; 3], [0.5; 3], fixed, u16::MAX)?,
            ))
            .id();
        entities.push(entity);
        ordinals.insert(entity, i);
    }
    // Prime schedule, spatial index and both output buffers without advancing motion.
    sim.tick()?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<Contacts>().as_slice().len(), 250);
    let start = Instant::now();
    sim.step_exact(500)?;
    let nanos = start.elapsed().as_nanos();
    assert_eq!(sim.elapsed(), Duration::from_millis(500));
    let mut position_checksum = 0.0;
    for (i, entity) in entities.iter().enumerate() {
        let state = sim.world().get::<Motion>(*entity).unwrap().state();
        let expected = (i / 2) as f64 * 3.0
            + (i % 2) as f64 * 0.75
            + if (i / 2) % 2 == 0 { 0.005 } else { 0.0 };
        assert!((state.position[0] - expected).abs() < 1e-9);
        assert_eq!(state.position[1..], [0.0; 2]);
        assert_eq!(state.orientation, [0.0, 0.0, 0.0, 1.0]);
        position_checksum += state.position[0];
    }
    let contacts = sim.world().resource::<Contacts>().as_slice();
    assert_eq!(contacts.len(), 250);
    let mut contact_checksum = 0.0;
    let mut seen = BTreeSet::new();
    for contact in contacts {
        let a = ordinals[&contact.models[0]];
        let b = ordinals[&contact.models[1]];
        assert_ne!(a, b);
        assert_eq!(a / 2, b / 2);
        assert_eq!((a / 2) % 2, 0);
        assert!(seen.insert(a / 2));
        assert!((contact.point[0] - ((a / 2) as f64 * 3.0 + 0.375 + 0.005)).abs() < 1e-9);
        assert_eq!(contact.point[1..], [0.0; 2]);
        contact_checksum += contact.point[0];
    }
    assert_eq!(seen, (0..500).step_by(2).collect());
    println!(
        "{{\"nanos\":{nanos},\"models\":1000,\"steps\":500,\"contacts\":250,\"position_checksum\":{position_checksum:.17},\"contact_checksum\":{contact_checksum:.17}}}"
    );
    Ok(())
}
