// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::{BoundaryImpulse, Particle};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
use sdformat::NoResources;

const WORLD: &str = include_str!("../../../examples/capsule_cable_guide.sdf");
const G: f64 = 9.81;
const DT: f64 = 0.002;

fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "capsule-guide.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn guide(sim: &Simulation) -> (u64, BodyState) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::roller::guide")
        })
        .unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        sim.world()
            .resource::<PhysicsBackend>()
            .state(e.get::<BodyHandle>().unwrap().id())
            .unwrap(),
    )
}
type Trace = Vec<(Vec<Particle>, Vec<BoundaryImpulse>)>;
fn frame(sim: &Simulation) -> Trace {
    bodies(sim)
        .map(|(_, s)| (s.particles().to_vec(), s.last_boundary_impulses().to_vec()))
        .collect()
}
fn momentum(points: &[Particle]) -> DVec3 {
    points
        .iter()
        .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
        .sum()
}
#[test]
fn finite_spinning_guide_carries_the_crown_and_the_endcap_sheds_the_short_cable() {
    for count in [1, 2, 4, 16] {
        let xml = WORLD.replace("substeps=\"4\"", &format!("substeps=\"{count}\""));
        let mut sim = load(&xml).unwrap();
        let (id, initial) = guide(&sim);
        let axis = DQuat::from_array(initial.orientation) * DVec3::Z;
        assert!(axis.distance(DVec3::Y) < 1e-12);
        let mut expected = Vec::new();
        let h = DT / count as f64;
        let flight_height = 1.02 - (0.5 + 0.2 + 0.025);
        let first_substep =
            (((1. + 8. * flight_height / (G * h * h)).sqrt() - 1.) * 0.5).ceil() as usize;
        let first_tick = first_substep.div_ceil(count);
        for epoch in 0..=1 {
            sim.set_paused(true);
            sim.tick().unwrap();
            assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
            let mut residual = 0f64;
            let mut first = [None; 2];
            let mut peak_x = 0f64;
            let mut cap_tip_y = 0f64;
            let mut min_gap = 1f64;
            for tick in 1..=500 {
                let before = frame(&sim);
                sim.step_exact(1)
                    .unwrap_or_else(|e| panic!("tick {tick}: {e:?}; {:?}", sim.fault()));
                let after = frame(&sim);
                let (_, s) = guide(&sim);
                assert!((s.position[0] - 0.15 * DT * tick as f64).abs() < 3e-12);
                assert!(DVec3::from(s.angular_velocity).distance(6. * axis) < 1e-11);
                for (i, ((p0, _), (p1, r))) in before.iter().zip(&after).enumerate() {
                    let grav = -DVec3::Z * (G * DT * 0.03 * p1.len() as f64);
                    let error = momentum(p1) - momentum(p0) - grav
                        + DVec3::from(r[0].linear)
                        + DVec3::from(r[1].linear);
                    residual = residual.max(error.length());
                    if r[0].linear != [0.; 3] && first[i].is_none() {
                        first[i] = Some(tick);
                    }
                    for p in p1 {
                        let d = DVec3::from(p.position) - DVec3::from(s.position);
                        let gap = (d - axis * d.dot(axis).clamp(-0.52, 0.52)).length() - 0.225;
                        min_gap = min_gap.min(gap);
                    }
                }
                // Lexical order is cap, crown. The straight crown lands at all
                // its points simultaneously and sees +1.2 m/s of surface slip.
                if tick == first_tick {
                    assert_eq!(first[1], Some(tick));
                    let expected_vx = 0.15 + 1.2f64.min(0.55 * G * h * first_substep as f64);
                    for p in &after[1].0 {
                        assert!(
                            (p.velocity[0] - expected_vx).abs() < 0.012,
                            "{count} {} vs {expected_vx}",
                            p.velocity[0]
                        );
                    }
                }
                peak_x = peak_x.max(after[1].0[0].position[0] - s.position[0]);
                cap_tip_y = cap_tip_y.max(after[0].0.last().unwrap().position[1]);
                if epoch == 0 {
                    expected.push((s, after));
                } else {
                    assert_eq!((s, after), expected[tick - 1]);
                }
            }
            println!(
                "substeps {count} epoch {epoch} first cap/crown {first:?} predicted crown {first_tick} max impulse residual {residual:e} N s; minimum capsule gap {min_gap:e} m; crown transverse travel {peak_x:.6} m; cap tip y {cap_tip_y:.6} m"
            );
            assert!(first[0].unwrap() > first_tick);
            // The crown travels more than a guide diameter. The tip of the
            // cap cable slides outward by at least its own contact radius.
            assert!(peak_x > 0.4 && cap_tip_y > 0.76 + 0.025);
            assert!(residual < 1e-9 && min_gap > -0.002);
            if epoch == 0 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(guide(&sim), (id, initial));
            }
        }
    }
}
#[test]
fn prescribed_capsule_rejects_tilt_and_dynamic_coupling() {
    for (xml, reason) in [
        (WORLD.replace("0 6 0", "6 0 0"), "fixed world axis"),
        (
            WORLD.replace("one_way", "two_way"),
            "requires an isolated single free centered",
        ),
    ] {
        let e = load(&xml).err().expect("unsupported capsule accepted");
        assert!(e.to_string().contains(reason), "{e}");
    }
}

#[test]
fn cloth_responds_to_axial_guide_friction_and_matches_its_total_contact_impulse() {
    let xml = include_str!("../../../examples/capsule_cloth_guide.sdf");
    for count in [1, 2, 4] {
        let mut drifts = Vec::new();
        for sign in [-1, 0, 1] {
            let xml = xml
                .replace("0 6 0", &format!("0 {} 0", sign * 6))
                .replace("substeps=\"4\"", &format!("substeps=\"{count}\""));
            let mut sim = load(&xml).unwrap();
            let mut residual = 0f64;
            let mut max_gap_error = 0f64;
            let mut max_extension = 0f64;
            let mut peak = 0f64;
            let mut total = DVec3::ZERO;
            let mut first = None;
            for tick in 1..=240 {
                let before = momentum(bodies(&sim).next().unwrap().1.particles());
                sim.step_exact(1).unwrap();
                let (_, state) = guide(&sim);
                let body = bodies(&sim).next().unwrap().1;
                let ps = body.particles();
                let impulse = body.last_boundary_impulses();
                if impulse[0].linear != [0.; 3] {
                    first.get_or_insert(tick);
                }
                total += DVec3::from(impulse[0].linear);
                let delta = momentum(ps) - before
                    + DVec3::Z * (0.15 * G * DT)
                    + DVec3::from(impulse[0].linear)
                    + DVec3::from(impulse[1].linear);
                residual = residual.max(delta.length());
                let dx = ps
                    .iter()
                    .map(|p| p.position[0] - state.position[0])
                    .sum::<f64>()
                    / ps.len() as f64;
                peak = peak.max(dx.abs());
                for (i, p) in ps.iter().enumerate() {
                    let d = DVec3::from(p.position) - DVec3::from(state.position);
                    max_gap_error =
                        max_gap_error.max(0.218 - (d - DVec3::Y * d.y.clamp(-0.52, 0.52)).length());
                    if i % 5 > 0 {
                        max_extension = max_extension.max(
                            DVec3::from(p.position).distance(ps[i - 1].position.into()) - 0.18,
                        );
                    }
                    if i >= 5 {
                        max_extension = max_extension.max(
                            DVec3::from(p.position).distance(ps[i - 5].position.into()) - 0.07,
                        );
                    }
                }
                if tick == 240 {
                    drifts.push(dx);
                }
            }
            println!(
                "cloth substeps {count} spin {} rad/s first {:?} max closure {residual:e} N s gap error {max_gap_error:e} m max extension {max_extension:e} m travel {peak} m total roller impulse {total:?}",
                sign * 6,
                first
            );
            assert!(
                first.is_some()
                    && residual < 1e-9
                    && max_gap_error < 0.002
                    && max_extension < 0.003
            );
        }
        assert!(
            drifts[0] < -0.11 && drifts[2] > 0.11 && drifts[1].abs() < 0.025,
            "{drifts:?}"
        );
    }
}

#[test]
fn fixed_capsule_works_without_a_motion_plugin_and_tilting_edits_fault_before_soft_publication() {
    let mut fixed = WORLD.to_owned();
    let pos = fixed
        .find("      <plugin name=\"gz::sim::systems::PrescribedMotion\"")
        .unwrap();
    let end = pos + fixed[pos..].find("</plugin>").unwrap() + "</plugin>".len();
    fixed.replace_range(pos..end, "");
    fixed = fixed
        .replace(
            "<model name=\"roller\">",
            "<model name=\"roller\"><static>true</static>",
        )
        .replace("<kinematic>true</kinematic>", "");
    let mut sim = load(&fixed).unwrap();
    sim.step_exact(130).unwrap();
    let press = bodies(&sim).find(|(name, _)| *name == "crown").unwrap().1;
    assert!(press.last_boundary_impulses()[0].linear[2] < 0.);
    assert_eq!(guide(&sim).1.velocity, [0.; 3]);
    let mut moving = load(WORLD).unwrap();
    let (id, _) = guide(&moving);
    let before = frame(&moving);
    gz_sim_scene::queue_link_world_velocity(&mut moving, id, [0.15, 0., 0.], [3., 0., 0.]).unwrap();
    assert!(moving.step_exact(1).is_err());
    assert_eq!(moving.elapsed(), std::time::Duration::ZERO);
    assert_eq!(frame(&moving), before);
}
