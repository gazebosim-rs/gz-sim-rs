use gz_physics_xpbd::{Boundary,SoftBody,Particle,SolverConfig};
use sha2::{Digest,Sha256};
use std::time::Duration;
fn main() {
 let body=SoftBody::new(vec![Particle{position:[0.,0.,0.125],velocity:[-0.005,0.,0.],inverse_mass:1.}],vec![],SolverConfig{gravity:[3.,0.,-10.],friction:0.2,particle_radius:0.125,substeps:1,..Default::default()}).unwrap().with_static_plane_friction(0.6).unwrap();
 std::fs::write(std::env::args().nth(1).unwrap(),bincode::serialize(&body.one_way_construction().unwrap()).unwrap()).unwrap();
 println!("{:?}",body.one_way_identity().unwrap());
 for belt in [0.,0.02] {let mut body=body.clone(); let mut hash=Sha256::new();
 for _ in 0..10 {body.step(Duration::from_millis(10), &[Boundary::Plane{point:[0.;3],normal:[0.,0.,1.],velocity:[belt,0.,0.]}]).unwrap();hash.update(bincode::serialize(&body.one_way_state().unwrap()).unwrap());}
 println!("{belt} {:?} {:x}",body.particles()[0],hash.finalize());}
}
