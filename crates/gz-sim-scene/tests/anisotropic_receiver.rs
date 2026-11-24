// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, Inertia, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_physics_xpbd::Particle;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, WorldPose, deformable::bodies, load_world_with_sdf_step,
};
use sdformat::NoResources;
use std::time::Duration;

const WORLD: &str = include_str!("../../../examples/anisotropic_sphere_impact.sdf");
const MASS: f64 = 2.;
const MOMENTS: DVec3 = DVec3::new(0.06, 0.09, 0.11);

// Reconstruct the signed SDF roll, pitch, yaw independently of the contact path.
fn principal_frame() -> DQuat {
    DQuat::from_rotation_z(0.5) * DQuat::from_rotation_y(0.15) * DQuat::from_rotation_x(-0.3)
}
fn native_reference(shape: Shape) -> (MultiFidelityBackend, BodyId) {
    let mut physics = MultiFidelityBackend::new([0.; 3]).unwrap();
    let id = physics
        .spawn_link(
            BodySpec {
                position: [0.; 3],
                velocity: [0.; 3],
                mass: Some(MASS),
                shape,
            },
            Default::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: principal_frame().to_array(),
                tensor: [MOMENTS.x, MOMENTS.y, MOMENTS.z, 0., 0., 0.],
            }),
        )
        .unwrap();
    (physics, id)
}
fn linear(state: BodyState) -> DVec3 {
    MASS * DVec3::from(state.velocity)
}
fn spin_momentum(state: BodyState) -> DVec3 {
    let frame = DQuat::from_array(state.orientation) * principal_frame();
    frame * (MOMENTS * (frame.conjugate() * DVec3::from(state.angular_velocity)))
}
fn angular_about(state: BodyState, origin: DVec3) -> DVec3 {
    spin_momentum(state) + (DVec3::from(state.position) - origin).cross(linear(state))
}

#[derive(Debug, PartialEq)]
struct Sample {
    rigid: BodyState,
    soft: Vec<Particle>,
}
impl Sample {
    fn capture(sim: &Simulation) -> Self {
        let receiver = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::receiver::shell")
            })
            .unwrap();
        let pose = receiver.get::<WorldPose>().unwrap();
        let velocity = receiver.get::<Velocity>().unwrap();
        Self {
            rigid: BodyState {
                position: pose.translation,
                orientation: pose.rotation,
                velocity: velocity.linear,
                angular_velocity: velocity.angular,
            },
            soft: bodies(sim).next().unwrap().1.particles().to_vec(),
        }
    }
    fn totals(&self) -> ([DVec3; 2], f64) {
        let mut p = linear(self.rigid);
        let mut l = angular_about(self.rigid, DVec3::ZERO);
        let mut energy = 0.5
            * (p.dot(DVec3::from(self.rigid.velocity))
                + spin_momentum(self.rigid).dot(DVec3::from(self.rigid.angular_velocity)));
        for pt in &self.soft {
            let v = DVec3::from(pt.velocity);
            let mv = v / pt.inverse_mass;
            p += mv;
            l += DVec3::from(pt.position).cross(mv);
            energy += 0.5 * mv.dot(v);
        }
        ([p, l], energy)
    }
    fn gap(&self, shape: Shape) -> f64 {
        let center = DVec3::from(self.rigid.position);
        let q = DQuat::from_array(self.rigid.orientation);
        self.soft
            .iter()
            .map(|p| {
                let relative = DVec3::from(p.position) - center;
                match shape {
                    Shape::Sphere { radius } => relative.length() - radius - 0.04,
                    Shape::Box { half_extents } => {
                        let delta = (q.conjugate() * relative).abs() - DVec3::from(half_extents);
                        delta.max(DVec3::ZERO).length() + delta.max_element().min(0.) - 0.04
                    }
                    _ => unreachable!(),
                }
            })
            .fold(f64::INFINITY, f64::min)
    }
}

#[test]
fn glancing_contact_with_rotated_unequal_principal_moments() {
    qualify(WORLD, Shape::Sphere { radius: 0.5 });
}
#[test]
fn cuboid_contact_rotates_and_translates_under_a_glancing_cable_impact() {
    qualify(
        include_str!("../../../examples/cuboid_soft_impact.sdf"),
        Shape::Box {
            half_extents: [0.425, 0.27, 0.2],
        },
    );
}
fn qualify(world: &str, shape: Shape) {
    let mut previous_drift = f64::INFINITY;
    for n in [1, 2, 4] {
        let xml = world.replace("substeps=\"4\"", &format!("substeps=\"{n}\""));
        let mut sim = load_world_with_sdf_step(
            &xml,
            "impact.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let (mut native, id) = native_reference(shape);
        let first = Sample::capture(&sim);
        let (initial, k) = first.totals();
        let mut last = first.rigid;
        let mut max_err = [0.0_f64; 2];
        let mut peak_reaction = [0.0_f64; 2];
        let mut min_gap = f64::INFINITY;
        let mut peak_k = k;
        let mut nonparallel = 0.0_f64;
        let mut contact_ticks = 0;
        let mut replay = Vec::new();
        for _ in 0..800 {
            // Compute the uncoupled rigid endpoint independently on each tick,
            // so the impulse balance excludes the native gyro integrator's drift.
            native.set_body_state(id, last).unwrap();
            native.step(Duration::from_millis(1)).unwrap();
            let free = native.state(id).unwrap();
            sim.step_exact(1).unwrap();
            let sample = Sample::capture(&sim);
            let state = sample.rigid;
            let (momentum, energy) = sample.totals();
            let reaction = bodies(&sim).next().unwrap().1.last_boundary_impulses()[0];
            let origin = DVec3::from(last.position);
            let delivered = [
                linear(state) - linear(free),
                angular_about(state, origin) - angular_about(free, origin),
            ];
            for (i, observed) in [reaction.linear, reaction.angular].into_iter().enumerate() {
                max_err[i] = max_err[i].max(momentum[i].distance(initial[i]));
                peak_reaction[i] =
                    peak_reaction[i].max(delivered[i].distance(DVec3::from(observed)));
            }
            min_gap = min_gap.min(sample.gap(shape));
            peak_k = peak_k.max(energy);
            nonparallel = nonparallel.max(
                DVec3::from(state.angular_velocity)
                    .cross(spin_momentum(state))
                    .length(),
            );
            contact_ticks += usize::from(reaction.linear != [0.; 3]);
            last = state;
            replay.push(sample);
        }
        let (_, final_k) = replay.last().unwrap().totals();
        println!(
            "substeps={n} active_ticks={contact_ticks} peak |ΔP|={:e} |ΔL|={:e} min gap={min_gap:e} m K={k:.7}->{final_k:.7} peak={peak_k:.7} J max |ω×Iω|={nonparallel:.7} peak native-referenced |J-Jsoft|={:e} |torque impulse error|={:e}",
            max_err[0], max_err[1], peak_reaction[0], peak_reaction[1]
        );
        assert!(contact_ticks > 0 && nonparallel > 0.005);
        assert!(max_err[0] < 1e-10 && peak_reaction[0] < 1e-12 && peak_reaction[1] < 1e-12);
        // The cable projection and native gyro solve have finite integration
        // error. Require first-order refinement of the complete trajectory.
        assert!(max_err[1] < 3e-4 && max_err[1] < 0.6 * previous_drift);
        previous_drift = max_err[1];
        assert!(min_gap > -1e-6 && peak_k <= k + 1e-5 && final_k < 0.8 * k);
        sim.reset().unwrap();
        assert_eq!(Sample::capture(&sim), first);
        for expected in replay {
            sim.step_exact(1).unwrap();
            assert_eq!(Sample::capture(&sim), expected);
        }
    }
}

#[test]
fn cuboid_rejects_unsupported_shapes_mobility_and_other_rigid_colliders_at_load() {
    let world = include_str!("../../../examples/cuboid_soft_impact.sdf");
    for xml in [
        world.replace("coupling=\"two_way\"","coupling=\"one_way\""),
        world.replace("<box link=", "<sphere link="),
        world.replace("<model name=\"receiver\">", "<model name=\"receiver\"><static>true</static>"),
        world.replace("</world>",r#"<model name="neighbor"><static>true</static><pose>5 0 0 0 0 0</pose><link name="l"><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model></world>"#),
    ]{
        assert!(load_world_with_sdf_step(&xml,"guard.sdf","default",&NoResources,PhysicsEngine::MultiFidelity).is_err());
    }
}
