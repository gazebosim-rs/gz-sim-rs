// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::Particle;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
const CABLE: &str = include_str!("../../../examples/spinning_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/spinning_cloth.sdf");
fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "rotational_constraints.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn points(sim: &Simulation) -> &[Particle] {
    bodies(sim).next().unwrap().1.particles()
}
fn soft(ps: &[Particle], origin: DVec3) -> (DVec3, DVec3, f64) {
    let mut p = DVec3::ZERO;
    let mut l = DVec3::ZERO;
    let mut k = 0.;
    for a in ps {
        let v = DVec3::from(a.velocity);
        let mv = v / a.inverse_mass;
        p += mv;
        l += (DVec3::from(a.position) - origin).cross(mv);
        k += 0.5 * mv.dot(v);
    }
    (p, l, k)
}
fn guide(sim: &Simulation) -> BodyState {
    let body = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|s| s.0 == "default::roller::guide")
        })
        .unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body.get::<BodyHandle>().unwrap().id())
        .unwrap()
}
fn rigid(s: BodyState) -> DVec3 {
    let q = DQuat::from_array(s.orientation);
    q * (DVec3::new(0.055, 0.055, 0.009) * (q.conjugate() * DVec3::from(s.angular_velocity)))
        + DVec3::from(s.position).cross(DVec3::from(s.velocity) * 0.4)
}
#[test]
fn authored_rotating_cable_and_sheet_follow_free_rotation_and_reset() {
    let center = DVec3::Z * 0.8;
    for (name, xml) in [("cable", CABLE), ("cloth", CLOTH)] {
        for conserve in [false, true] {
            let xml = if conserve {
                xml.to_owned()
            } else {
                xml.replace(
                    "conserve_angular_momentum=\"true\"",
                    "conserve_angular_momentum=\"false\"",
                )
            };
            let mut sim = load(&xml).unwrap();
            let initial = points(&sim).to_vec();
            let (_, l0, k0) = soft(&initial, center);
            let mut d_l = 0f64;
            let mut d_k = 0f64;
            let mut max_position = 0f64;
            let mut history = vec![];
            for tick in 1..=400 {
                sim.step_exact(1).unwrap();
                let q = DQuat::from_rotation_x(0.06 * tick as f64);
                let ps = points(&sim);
                let (p, l, k) = soft(ps, center);
                assert!(p.length() < 1e-11);
                d_l = d_l.max(l.distance(l0));
                d_k = d_k.max((k - k0).abs());
                for (a, b) in initial.iter().zip(ps) {
                    max_position = max_position.max(
                        (center + q * (DVec3::from(a.position) - center))
                            .distance(b.position.into()),
                    );
                }
                if conserve {
                    history.push(ps.to_vec());
                }
            }
            println!(
                "{name} conserve={conserve}: 4 s max relative L error={:e}, relative energy error={:e}, particle oracle error={max_position:e} m",
                d_l / l0.length(),
                d_k / k0
            );
            if conserve {
                assert!(d_l / l0.length() < 1e-10 && d_k / k0 < 1e-6 && max_position < 0.01);
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(points(&sim), initial);
                for pose in history {
                    sim.step_exact(1).unwrap();
                    assert_eq!(points(&sim), pose);
                }
            } else {
                assert!(d_l / l0.length() > 0.10 && d_k / k0 > 0.1 && max_position > 0.1);
            }
        }
    }
    let e = load(&CABLE.replace(
        "conserve_angular_momentum=\"true\"",
        "conserve_angular_momentum=\"typo\"",
    ))
    .err()
    .unwrap();
    assert!(e.to_string().contains("must be true or false"), "{e}");
}

#[test]
fn applied_capsule_torque_accounts_for_the_free_soft_bodys_angular_momentum_change() {
    for (name, xml) in [
        (
            "barrel",
            include_str!("../../../examples/dynamic_capsule_cable.sdf"),
        ),
        (
            "cap",
            include_str!("../../../examples/dynamic_capsule_cloth.sdf"),
        ),
    ] {
        for n in [2, 4, 8] {
            let mut drift = vec![];
            for conserve in [false, true] {
                let xml = xml
                    .replace(
                        "self_contact=",
                        &format!("conserve_angular_momentum=\"{conserve}\" self_contact="),
                    )
                    .replace("substeps=\"4\"", &format!("substeps=\"{n}\""));
                let mut sim = load(&xml).unwrap();
                let initial = guide(&sim);
                let (p0, l_soft, _) = soft(points(&sim), DVec3::ZERO);
                let p0 = p0 + DVec3::from(initial.velocity) * 0.4;
                let l0 = l_soft + rigid(initial);
                let (mut closed, mut dl, mut dp, mut retries) = (0f64, 0f64, 0f64, 0);
                let mut first = None;
                let mut native_defect = DVec3::ZERO;
                let mut corrected_error = 0f64;
                for tick in 1..=500 {
                    let before = guide(&sim);
                    let origin = DVec3::from(before.position);
                    let prior_l = soft(points(&sim), origin).1;
                    sim.step_exact(1).unwrap_or_else(|e| {
                        panic!(
                            "{name} n={n} conserve={conserve} tick={tick} {e}: {:?}",
                            sim.fault()
                        )
                    });
                    let sample = bodies(&sim).next().unwrap().1;
                    let j = sample.last_boundary_impulses()[0];
                    retries += usize::from(sample.last_sweep_refinements() > 0);
                    if j.linear != [0.; 3] {
                        first.get_or_insert(tick);
                    }
                    let (p, l, _) = soft(sample.particles(), DVec3::ZERO);
                    let after = guide(&sim);
                    native_defect += rigid(after)
                        - rigid(before)
                        - DVec3::from(j.angular)
                        - origin.cross(DVec3::from(j.linear));
                    corrected_error =
                        corrected_error.max((l + rigid(after) - l0 - native_defect).length());
                    dl = dl.max((l + rigid(after)).distance(l0));
                    dp = dp.max((p + 0.4 * DVec3::from(after.velocity)).distance(p0));
                    let next_l = soft(sample.particles(), origin).1;
                    closed = closed.max((next_l - prior_l + DVec3::from(j.angular)).length());
                }
                println!(
                    "{name} n={n} conserve={conserve} hit={first:?} max soft angular residual per tick={closed:e} total orbit+spin dL={dl:e} dP={dp:e} residual after native torque accounting={corrected_error:e} refined ticks={retries}"
                );
                assert!(dp < 1e-10 && first == Some(if name == "barrel" { 76 } else { 72 }));
                if conserve {
                    assert!(closed < 1e-11 && corrected_error < 1e-11);
                }
                drift.push(dl);
            }
            assert!(drift[1] < drift[0], "{name} {n}: {drift:?}");
        }
    }
}

#[test]
fn preserving_native_receiver_rotation_closes_total_orbital_and_spin_momentum() {
    let opt = "<plugin name=\"gz::sim::systems::FreeRotation\" filename=\"gz-sim-free-rotation-system\"><link>default::roller::guide</link></plugin>";
    for (name, xml) in [
        (
            "barrel",
            include_str!("../../../examples/dynamic_capsule_cable.sdf"),
        ),
        (
            "endcap",
            include_str!("../../../examples/dynamic_capsule_cloth.sdf"),
        ),
    ] {
        let xml = xml.replace(
            "self_contact=",
            "conserve_angular_momentum=\"true\" self_contact=",
        );
        for conserve in [false, true] {
            let xml = xml.replace(
                "</world>",
                &format!("{}</world>", if conserve { opt } else { "" }),
            );
            let mut sim = load(&xml).unwrap();
            let initial = guide(&sim);
            let (p, l, _) = soft(points(&sim), DVec3::ZERO);
            let p0 = p + 0.4 * DVec3::from(initial.velocity);
            let l0 = l + rigid(initial);
            let (mut error_p, mut error_l) = (0f64, 0f64);
            let mut hits = 0;
            for _ in 0..500 {
                sim.step_exact(1).unwrap();
                let receiver = guide(&sim);
                let (p, l, _) = soft(points(&sim), DVec3::ZERO);
                error_p = error_p.max((p + 0.4 * DVec3::from(receiver.velocity)).distance(p0));
                error_l = error_l.max((l + rigid(receiver)).distance(l0));
                hits += usize::from(
                    bodies(&sim).next().unwrap().1.last_boundary_impulses()[0].linear != [0.; 3],
                );
            }
            println!(
                "{name} isolated rotor={conserve}: 500 ticks {hits} impulses, max total |dP|={error_p:e} |dL|={error_l:e}"
            );
            assert!(hits > 0 && error_p < 1e-10);
            if conserve {
                assert!(error_l < 1e-10);
            } else {
                assert!(error_l > 1e-8);
            }
        }
    }
}
