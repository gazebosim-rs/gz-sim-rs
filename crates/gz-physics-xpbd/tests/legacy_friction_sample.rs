// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "one-way-checkpoint")]
use gz_physics_xpbd::*;
use sha2::{Digest, Sha256};
use std::time::Duration;
#[test]
fn golden() {
    let mut body = SoftBody::new(vec![Particle{position:[0.,0.,0.005],velocity:[0.;3],inverse_mass:2.5}, Particle{position:[0.,0.15,0.005],velocity:[0.15,0.,0.],inverse_mass:1.25}],vec![],SolverConfig{gravity:[6.,0.,-10.],iterations:4,substeps:1,friction:0.1,particle_radius:0.005,self_contact:false}).unwrap().with_static_plane_friction(0.45).unwrap();
    let seed=bincode::serialize(&body.one_way_construction().unwrap()).unwrap();
    println!("SEED {}",seed.iter().map(|v|format!("{v:02x}")).collect::<String>());
    println!("LAW {}",body.one_way_identity().unwrap().iter().map(|v|format!("{v:02x}")).collect::<String>());
    for gravity in [6.,2.] { for boundary in [Boundary::Plane{point:[0.;3],normal:[0.,0.,1.],velocity:[0.;3]},Boundary::StaticCuboid{center:[0.,0.,-0.05],half_extents:[2.,0.4,0.05],orientation:[0.,0.,0.,1.]}] {
        body=SoftBody::from_one_way_construction(&bincode::deserialize(&seed).unwrap()).unwrap();
        body.set_gravity([gravity, 0., -10.]).unwrap();
        let mut h=Sha256::new();
        for i in 0..25 {
            body.step(Duration::from_millis(5),&[boundary]).unwrap();
            if gravity==2. && matches!(boundary,Boundary::StaticCuboid{..}) {println!("T {i} {:?}",body.one_way_state().unwrap());}
            h.update(bincode::serialize(&body.one_way_state().unwrap()).unwrap());
        }
        println!("TRACE {:x}",h.finalize());
    }}
}
