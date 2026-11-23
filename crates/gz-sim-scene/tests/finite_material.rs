// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::MaterialFeature;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
const CABLE: &str = include_str!("../../../examples/finite_material_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/finite_material_cloth.sdf");
fn guide(s: &Simulation) -> BodyState {
    let b = s
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::roller::guide")
        })
        .unwrap();
    s.world()
        .resource::<PhysicsBackend>()
        .state(b.get::<BodyHandle>().unwrap().id())
        .unwrap()
}
fn conserved(sim: &Simulation) -> (DVec3, DVec3, f64) {
    let s = guide(sim);
    let q = DQuat::from_array(s.orientation)
        * DQuat::from_rotation_z(0.13)
        * DQuat::from_rotation_y(-0.20)
        * DQuat::from_rotation_x(0.15);
    let v = DVec3::from(s.velocity);
    let omega = DVec3::from(s.angular_velocity);
    let spin = q * (DVec3::new(0.008, 0.009, 0.0035) * (q.inverse() * omega));
    let mut p = 0.4 * v;
    let mut l = DVec3::from(s.position).cross(p) + spin;
    let mut e = 0.5 * (p.dot(v) + spin.dot(omega));
    for node in bodies(sim).next().unwrap().1.particles() {
        let v = DVec3::from(node.velocity);
        let mv = v / node.inverse_mass;
        p += mv;
        l += DVec3::from(node.position).cross(mv);
        e += 0.5 * v.dot(mv);
    }
    (p, l, e)
}
#[test]
fn spinning_finite_capsules_take_real_attributed_impulses_from_material_interiors() {
    for (kind, xml) in [("cable", CABLE), ("cloth", CLOTH)] {
        for n in [1, 2, 4, 8] {
            for material in [false, true] {
                let xml = xml
                    .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
                    .replace(
                        "capsule_contact=\"material\"",
                        &format!(
                            "capsule_contact=\"{}\"",
                            if material { "material" } else { "particles" }
                        ),
                    );
                let mut sim = load_world_with_sdf_step(
                    &xml,
                    "finite_material.sdf",
                    "default",
                    &sdformat::NoResources,
                    PhysicsEngine::MultiFidelity,
                )
                .unwrap();
                let (p0, l0, e0) = conserved(&sim);
                let v0 = DVec3::from(guide(&sim).velocity);
                let mut first = None;
                let (mut max_dp, mut max_dl, mut gain, mut max_sum, mut max_closure) =
                    (0f64, 0f64, 0f64, 0f64, 0f64);
                let mut cumulative = DVec3::ZERO;
                let mut net_angular = DVec3::ZERO;
                let mut history = vec![];
                let mut contact_ticks = 0;
                let mut retries = 0;
                for tick in 1..=350 {
                    let start = guide(&sim);
                    sim.step_exact(1).unwrap_or_else(|e| {
                        panic!(
                            "{kind} n={n}, material={material}, tick={tick}: {e}, {:?}",
                            sim.fault()
                        )
                    });
                    let (p, l, e) = conserved(&sim);
                    let s = bodies(&sim).next().unwrap().1;
                    let b = guide(&sim);
                    max_dp = max_dp.max(p.distance(p0));
                    max_dl = max_dl.max(l.distance(l0));
                    gain = gain.max(e - e0);
                    let r = s.last_boundary_impulses()[0];
                    let j = DVec3::from(r.linear);
                    if j != DVec3::ZERO {
                        contact_ticks += 1;
                    }
                    let loads = s.last_capsule_material_loads();
                    if material {
                        for load in loads {
                            assert!(load.two_way && load.boundary == 0);
                            if first.is_none() {
                                first = Some((
                                    tick,
                                    (tick - 1) as f64 * 0.002 + load.first_time,
                                    load.feature,
                                ));
                            }
                            let nodes = load.nodal.iter().map(|j| DVec3::from(*j)).sum::<DVec3>();
                            max_closure =
                                max_closure.max((nodes + DVec3::from(load.linear)).length());
                        }
                        max_sum = max_sum
                            .max(j.distance(loads.iter().map(|v| DVec3::from(v.linear)).sum()));
                    } else {
                        assert!(loads.is_empty());
                    }
                    cumulative += j;
                    net_angular += DVec3::from(r.angular) + DVec3::from(start.position).cross(j);
                    max_closure = max_closure
                        .max((0.4 * (DVec3::from(b.velocity) - v0)).distance(cumulative));
                    retries += s.last_sweep_refinements();
                    if material && n == 4 {
                        history.push((b, s.particles().to_vec(), loads.to_vec(), r));
                    }
                }
                let (_, _, e) = conserved(&sim);
                println!(
                    "{kind} material={material} n={n} first={first:?}, contacts={contact_ticks} retries={retries}, net rigid impulse={cumulative:?} world angular impulse={net_angular:?}, max |dP|={max_dp:e}, |dL|={max_dl:e}, added energy={gain:e} J, final loss={:e} J, attribution={max_sum:e} closure={max_closure:e}",
                    e0 - e
                );
                assert!(
                    max_dp < 1e-10
                        && max_dl < 1e-9
                        && gain < 1e-8
                        && max_sum < 1e-10
                        && max_closure < 1e-10
                );
                if material {
                    let (_, t, id) = first.unwrap();
                    assert!((0.015..0.25).contains(&t));
                    assert!(matches!(
                        (kind, id),
                        ("cable", MaterialFeature::Segment(_))
                            | ("cloth", MaterialFeature::Triangle(_))
                    ));
                    assert!(cumulative.length() > 0.03);
                    if n == 4 {
                        reset_loaded_world(&mut sim, |_| ()).unwrap();
                        for (b, p, j, r) in history {
                            sim.step_exact(1).unwrap();
                            let s = bodies(&sim).next().unwrap().1;
                            assert_eq!(guide(&sim), b);
                            assert_eq!(s.particles(), p);
                            assert_eq!(s.last_capsule_material_loads(), j);
                            assert_eq!(s.last_boundary_impulses()[0], r);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn convergence_and_true_separation_against_two_finer_public_clocks() {
    use parry3d_f64::{
        math::Pose,
        query,
        shape::{Segment, Triangle},
    };
    for (name, xml) in [("cable", CABLE), ("cloth", CLOTH)] {
        let mut histories = vec![];
        for (ms, n, ratio) in [
            ("0.000125", 4, 16),
            ("0.00025", 4, 8),
            ("0.002", 1, 1),
            ("0.002", 2, 1),
            ("0.002", 4, 1),
            ("0.002", 8, 1),
        ] {
            let xml = xml
                .replace("<max_step_size>0.002", &format!("<max_step_size>{ms}"))
                .replace("substeps=\"4\"", &format!("substeps=\"{n}\""));
            let mut sim = load_world_with_sdf_step(
                &xml,
                "convergence.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
            .unwrap();
            let mut hist = vec![];
            let mut min_gap = f64::INFINITY;
            for _ in 0..350 {
                sim.step_exact(ratio).unwrap();
                let g = guide(&sim);
                let center = DVec3::from(g.position);
                let q = DQuat::from_array(g.orientation);
                let core = Segment::new(
                    center - q * (DVec3::Z * 0.15),
                    center + q * (DVec3::Z * 0.15),
                );
                let sample = bodies(&sim).next().unwrap().1;
                let nodes = sample
                    .particles()
                    .iter()
                    .map(|p| DVec3::from(p.position))
                    .collect::<Vec<_>>();
                let separation = if name == "cable" {
                    query::distance(
                        &Pose::IDENTITY,
                        &core,
                        &Pose::IDENTITY,
                        &Segment::new(nodes[0], nodes[1]),
                    )
                    .unwrap()
                    .distance
                } else {
                    [[0, 1, 3], [0, 3, 2]]
                        .into_iter()
                        .map(|[a, b, c]| {
                            query::distance(
                                &Pose::IDENTITY,
                                &core,
                                &Pose::IDENTITY,
                                &Triangle::new(nodes[a], nodes[b], nodes[c]),
                            )
                            .unwrap()
                            .distance
                        })
                        .fold(f64::INFINITY, f64::min)
                } - 0.10;
                min_gap = min_gap.min(separation);
                hist.push((g, nodes));
            }
            assert!(min_gap > -6e-8, "{name} {ms} n={n} gap={min_gap:e}");
            histories.push((ms, n, hist, min_gap));
        }
        let reference = &histories[0].2;
        for (ms, n, hist, min_gap) in &histories[1..] {
            let (mut dx, mut dv, mut spin, mut nodes, mut angle) = (0f64, 0f64, 0f64, 0f64, 0f64);
            for ((a, p), (b, q)) in hist.iter().zip(reference) {
                dx = dx.max(DVec3::from(a.position).distance(b.position.into()));
                dv = dv.max(DVec3::from(a.velocity).distance(b.velocity.into()));
                spin =
                    spin.max(DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()));
                angle = angle.max(
                    DQuat::from_array(a.orientation)
                        .angle_between(DQuat::from_array(b.orientation)),
                );
                for (x, y) in p.iter().zip(q) {
                    nodes = nodes.max(x.distance(*y));
                }
            }
            println!(
                "{name} public={ms}s substeps={n}: max differences vs 0.125 ms public/4 soft: rigid position={dx:e} m velocity={dv:e} m/s orientation={angle:e} rad spin={spin:e} rad/s soft position={nodes:e} m. Min independent material clearance={min_gap:e} m."
            );
            assert!(dx < 0.012 && nodes < 0.025 && angle < 0.12);
        }
    }
}
