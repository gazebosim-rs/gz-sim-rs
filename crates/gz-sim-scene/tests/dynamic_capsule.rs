// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::{BoundaryImpulse, Particle};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
const CABLE: &str = include_str!("../../../examples/dynamic_capsule_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/dynamic_capsule_cloth.sdf");
const M: f64 = 0.4;
const I: DVec3 = DVec3::new(0.055, 0.055, 0.009);
fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "dynamic_capsule.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn rigid(sim: &Simulation) -> BodyState {
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::roller::guide")
        })
        .unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(link.get::<BodyHandle>().unwrap().id())
        .unwrap()
}
fn momentum(s: BodyState, points: &[Particle]) -> (DVec3, DVec3, f64) {
    let v = DVec3::from(s.velocity);
    let w = DVec3::from(s.angular_velocity);
    let q = DQuat::from_array(s.orientation);
    let spin = q * (I * (q.conjugate() * w));
    let mut linear = M * v;
    let mut angular = DVec3::from(s.position).cross(linear) + spin;
    let mut energy = (linear.dot(v) + spin.dot(w)) * 0.5;
    for p in points {
        let v = DVec3::from(p.velocity);
        let mv = v / p.inverse_mass;
        linear += mv;
        angular += DVec3::from(p.position).cross(mv);
        energy += 0.5 * mv.dot(v);
    }
    (linear, angular, energy)
}
fn step(sim: &mut Simulation) -> (BodyState, Vec<Particle>, BoundaryImpulse) {
    sim.step_exact(1)
        .unwrap_or_else(|e| panic!("after {:?}: {e} {:?}", sim.elapsed(), sim.fault()));
    let sample = bodies(sim).next().unwrap().1;
    (
        rigid(sim),
        sample.particles().to_vec(),
        sample.last_boundary_impulses()[0],
    )
}
#[test]
fn barrel_cable_and_endcap_cloth_transfer_momentum_to_a_free_native_capsule() {
    for (name, xml, skin) in [("barrel", CABLE, 0.025), ("endcap", CLOTH, 0.018)] {
        for substeps in [2, 4, 8] {
            let xml = xml.replace("substeps=\"4\"", &format!("substeps=\"{substeps}\""));
            let mut sim = load(&xml).unwrap();
            let start = rigid(&sim);
            let (p0, l0, e0) = momentum(start, bodies(&sim).next().unwrap().1.particles());
            let (mut dp, mut dl, mut reaction_error, mut gap_error, mut max_spin, mut energy_gain) =
                (0f64, 0f64, 0f64, 0f64, 0f64, 0f64);
            let mut native_torque_error = 0f64;
            let mut soft_torque_error = 0f64;
            let mut previous_points = bodies(&sim).next().unwrap().1.particles().to_vec();
            let mut refined = 0;
            let mut max_refinements = 0;
            let mut total = DVec3::ZERO;
            let mut first = None;
            let mut prior = start;
            let mut trace = vec![];
            for tick in 1..=500 {
                let (s, ps, j) = step(&mut sim);
                let retries = bodies(&sim).next().unwrap().1.last_sweep_refinements();
                refined += usize::from(retries > 0);
                max_refinements = max_refinements.max(retries);
                if j.linear != [0.; 3] {
                    first.get_or_insert(tick);
                }
                total += DVec3::from(j.linear);
                let (p, l, e) = momentum(s, &ps);
                dp = dp.max(p.distance(p0));
                dl = dl.max(l.distance(l0));
                energy_gain = energy_gain.max(e - e0);
                let base = DVec3::from(prior.position);
                let native_l = |state: BodyState| {
                    let q = DQuat::from_array(state.orientation);
                    q * (I * (q.conjugate() * DVec3::from(state.angular_velocity)))
                        + (DVec3::from(state.position) - base)
                            .cross(M * DVec3::from(state.velocity))
                };
                let soft_l = |points: &[Particle]| -> DVec3 {
                    points
                        .iter()
                        .map(|p| {
                            (DVec3::from(p.position) - base)
                                .cross(DVec3::from(p.velocity) / p.inverse_mass)
                        })
                        .sum()
                };
                native_torque_error = native_torque_error
                    .max((native_l(s) - native_l(prior) - DVec3::from(j.angular)).length());
                soft_torque_error = soft_torque_error.max(
                    (soft_l(&ps) - soft_l(&previous_points) + DVec3::from(j.angular)).length(),
                );
                previous_points.clone_from(&ps);
                reaction_error = reaction_error.max(
                    (M * (DVec3::from(s.velocity) - DVec3::from(prior.velocity))
                        - DVec3::from(j.linear))
                    .length(),
                );
                let q = DQuat::from_array(s.orientation);
                let c = DVec3::from(s.position);
                for p in &ps {
                    let x = q.conjugate() * (DVec3::from(p.position) - c);
                    let nearest = DVec3::Z * x.z.clamp(-0.52, 0.52);
                    gap_error = gap_error.max(0.2 + skin - x.distance(nearest));
                }
                max_spin = max_spin.max(DVec3::from(s.angular_velocity).length());
                prior = s;
                trace.push((s, ps, j));
            }
            let v = DVec3::from(prior.velocity);
            println!(
                "{name} n={substeps} hit={first:?} max dP={dp:e} dL={dl:e} dE+={energy_gain:e} rigid reaction error={reaction_error:e} native dL-J={native_torque_error:e} soft dL+J={soft_torque_error:e} refined ticks={refined} max doublings={max_refinements} max penetration={gap_error:e} spin={max_spin:.5} final rigid v={v:?} impulse={total:?}"
            );
            assert!(
                first.is_some()
                    && dp < 1e-9
                    && dl < 0.0008
                    && reaction_error < 1e-9
                    && gap_error < 1e-6
                    && max_spin > 0.2
                    && energy_gain < 1e-5
            );
            if name == "barrel" {
                assert!(v.z < -0.1);
            } else {
                assert!(v.y < -0.1);
            }
            if substeps == 4 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(rigid(&sim), start);
                for wanted in trace {
                    assert_eq!(step(&mut sim), wanted);
                }
            }
        }
    }
}

#[test]
fn a_dynamic_capsule_requires_an_isolated_freely_rotating_native_link() {
    for xml in [CABLE.replace("<link name=\"guide\">","<static>true</static><link name=\"guide\">"),
                CABLE.replace("    <plugin name=\"gz::sim::systems::Deformables\"", "    <model name=\"obstacle\"><static>true</static><link name=\"body\"><collision name=\"s\"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model><plugin name=\"gz::sim::systems::Deformables\"")] {
        let e=load(&xml).err().unwrap(); assert!(e.to_string().contains("isolated single free centered"),"{e}");
    }
}
