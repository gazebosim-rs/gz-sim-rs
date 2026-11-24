// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Serialize, Deserialize)]
struct LegacyConstruction {
    particles: Vec<Particle>,
    constraints: Vec<DistanceConstraint>,
    bending: Vec<DihedralConstraint>,
    config: SolverConfig,
    topology: MaterialTopology,
    conserve_momentum: bool,
}

fn cloth() -> SoftBody {
    let nodes = [[0., 0., 0.], [0.2, 0., 0.01], [0., 0.2, 0.], [0.2, 0.2, 0.]]
        .into_iter()
        .enumerate()
        .map(|(i, position)| Particle {
            position,
            velocity: [0., 0., if i == 1 { 0.1 } else { 0. }],
            inverse_mass: 1.,
        })
        .collect();
    SoftBody::cloth(nodes, 2, 0.001, 0.002, 0.05, SolverConfig::default()).unwrap()
}

#[test]
fn cold_reconstruction_retains_damping_and_legacy_bytes_remain_readable() {
    let plain = cloth();
    let base_id = plain.one_way_identity().unwrap();
    let bytes = bincode::serialize(&plain.one_way_construction().unwrap()).unwrap();
    let old: LegacyConstruction = bincode::deserialize(&bytes).unwrap();
    assert!(matches!(old.topology, MaterialTopology::Cloth(2)));
    assert_eq!(bincode::serialize(&old).unwrap(), bytes);
    let old_seed = bincode::deserialize(&bincode::serialize(&old).unwrap()).unwrap();
    assert_eq!(
        SoftBody::from_one_way_construction(&old_seed)
            .unwrap()
            .one_way_identity()
            .unwrap(),
        base_id
    );
    let mut live = plain.with_bending_damping(0.015).unwrap();
    let id = live.one_way_identity().unwrap();
    assert_ne!(id, base_id);
    let bytes = bincode::serialize(&live.one_way_construction().unwrap()).unwrap();
    assert!(bincode::deserialize::<LegacyConstruction>(&bytes).is_err());
    let mut cold =
        SoftBody::from_one_way_construction(&bincode::deserialize(&bytes).unwrap()).unwrap();
    assert_eq!(id, cold.one_way_identity().unwrap());
    let dt = Duration::from_micros(500);
    for _ in 0..4 {
        live.step(dt, &[]).unwrap();
    }
    let state = live.one_way_state().unwrap();
    cold.install_one_way_state(state).unwrap();
    for _ in 0..12 {
        live.step(dt, &[]).unwrap();
        cold.step(dt, &[]).unwrap();
        assert_eq!(live.particles(), cold.particles());
    }
    assert_eq!(
        cold.with_bending_damping(0.)
            .unwrap()
            .one_way_identity()
            .unwrap(),
        base_id
    );
}

#[test]
fn invalid_damped_construction_refuses_without_reinterpreting_legacy_rows() {
    let mut seed = cloth().one_way_construction().unwrap();
    for coefficient in [0., -1., f64::INFINITY] {
        seed.topology = ConstructionTopology::DampedCloth {
            columns: 2,
            coefficient,
        };
        assert!(matches!(
            SoftBody::from_one_way_construction(&seed),
            Err(XpbdError::MaterialCheckpoint)
        ));
    }
}
