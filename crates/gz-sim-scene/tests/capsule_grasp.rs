// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/capsule_grasp.rs"]
mod grasp;
use glam::{DQuat, DVec3};
use grasp::*;
use gz_sim_scene::reset_loaded_world;
use parry3d_f64::{
    math::Pose,
    query,
    shape::{Segment, Triangle},
};

#[test]
fn force_driven_hold_balances_weight_true_material_clearance_and_exact_replay() {
    let mut run = Grasp::new(Mode::Closed, WORLD.into(), SQUEEZE).unwrap();
    let mut trace = vec![];
    let (mut min_gap, mut min_pair, mut min_vertex) = (f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let (mut max_budget, mut support, mut traction, mut approach, mut during, mut released) =
        (0f64, 0., 0., 0usize, 0usize, 0usize);
    for tick in 1..=1050 {
        run.step().unwrap();
        let jaws = run.jaws();
        let s = run.cloth();
        let nodes = s
            .particles()
            .iter()
            .map(|p| DVec3::from(p.position))
            .collect::<Vec<_>>();
        let cores = jaws.map(|b| {
            let c = DVec3::from(b.position);
            let h = DQuat::from_array(b.orientation) * DVec3::Z * HALF_LENGTH;
            Segment::new(c - h, c + h)
        });
        let d = query::distance(&Pose::IDENTITY, &cores[0], &Pose::IDENTITY, &cores[1])
            .unwrap()
            .distance
            - 2. * RADIUS;
        min_pair = min_pair.min(d);
        for core in &cores {
            for &q in &nodes {
                let p = Segment::new(q, q);
                let d = query::distance(&Pose::IDENTITY, core, &Pose::IDENTITY, &p)
                    .unwrap()
                    .distance
                    - RADIUS
                    - SKIN;
                min_vertex = min_vertex.min(d);
            }
            for [a, b, c] in [[0, 1, 3], [0, 3, 2]] {
                let tri = Triangle::new(nodes[a], nodes[b], nodes[c]);
                let d = query::distance(&Pose::IDENTITY, core, &Pose::IDENTITY, &tri)
                    .unwrap()
                    .distance
                    - RADIUS
                    - SKIN;
                min_gap = min_gap.min(d);
            }
        }
        for l in s.last_capsule_friction_loads() {
            assert!(l.two_way);
            assert!(
                (l.last_weights.iter().sum::<f64>() - 1.).abs() < 1e-12
                    || l.normal_impulse_sum == 0.
            );
            assert!(l.normal_weighted_slip >= 0. && l.slip_distance >= 0.);
            assert!(l.tangent_impulse_sum <= 0.65 * l.normal_impulse_sum + 1e-14);
            let tangent = DVec3::from(l.last_tangent_impulse);
            let slip = DVec3::from(l.last_slip_velocity);
            if slip.length() > 1e-7 && l.last_normal_impulse > 1e-12 {
                let bound = 0.65 * l.last_normal_impulse;
                // At a sliding endpoint the tangent lies on the disk boundary and
                // opposes slip. Tiny impulses amplify absolute solver tolerances.
                assert!((tangent.length() - bound).abs() < 2e-13 && tangent.dot(slip) <= 1e-14);
            }
            if l.normal_impulse_sum > 0. {
                max_budget = max_budget.max(l.tangent_impulse_sum / l.normal_impulse_sum);
            }
        }
        if (570..=690).contains(&tick) {
            let j = s.last_boundary_impulses();
            assert!(j[0].linear[1] > 0. && j[1].linear[1] < 0.);
            support -= j[0].linear[2] + j[1].linear[2];
            traction -= s
                .last_capsule_friction_loads()
                .iter()
                .map(|f| f.linear[2])
                .sum::<f64>();
            assert!(run.center().x > 0.045 && run.center().z > 0.12);
            during += 1;
        } else if tick < 570 {
            approach += 1;
        } else if tick > 900 {
            assert!(
                s.last_capsule_material_loads().is_empty()
                    && s.last_capsule_friction_loads().is_empty()
            );
            released += 1;
        }
        trace.push((
            jaws,
            s.particles().to_vec(),
            s.last_boundary_impulses().to_vec(),
            s.last_capsule_material_loads().to_vec(),
            s.last_capsule_friction_loads().to_vec(),
        ));
    }
    assert!(min_gap > -2e-8 && min_pair > 0.0049 && min_vertex > 0.04);
    println!(
        "Closed: native pair min exact gap {min_pair:e} m, nearest node gap {min_vertex:e} m, worst true complete-skin clearance {min_gap:e} m, max Coulomb ratio {max_budget:e}. {approach}/{during}/{released} approach/hold/released ticks, support {} N, tangential {} N.",
        support / (during as f64 * 0.001),
        traction / (during as f64 * 0.001)
    );
    assert!((support / (during as f64 * 0.001) - GRAVITY * CLOTH_MASS).abs() < 0.00015);
    reset_loaded_world(&mut run.sim, |_| ()).unwrap();
    assert!(
        run.cloth().last_capsule_material_loads().is_empty()
            && run.cloth().last_capsule_friction_loads().is_empty()
    );
    for (j, p, imp, n, f) in trace {
        run.step().unwrap();
        let s = run.cloth();
        assert_eq!(run.jaws(), j);
        assert_eq!(s.particles(), p);
        assert_eq!(s.last_boundary_impulses(), imp);
        assert_eq!(s.last_capsule_material_loads(), n);
        assert_eq!(s.last_capsule_friction_loads(), f);
    }
}

#[test]
fn static_coulomb_limit_brackets_the_observed_grip_threshold_with_controls() {
    let threshold = CLOTH_MASS * GRAVITY / (2. * 0.65);
    assert!((0.015..0.018).contains(&threshold));
    for (mode, squeeze, carry) in [
        (Mode::Closed, 0.015, false),
        (Mode::Closed, 0.018, true),
        (Mode::Open, SQUEEZE, false),
        (Mode::NoFriction, SQUEEZE, false),
        (Mode::Particles, SQUEEZE, false),
    ] {
        let mut run = Grasp::new(mode, WORLD.into(), squeeze).unwrap();
        let mut j = 0.;
        let mut count = 0;
        for tick in 1..=690 {
            run.step().unwrap();
            if tick >= 570 {
                let r = run.cloth().last_boundary_impulses();
                j -= r[0].linear[2] + r[1].linear[2];
                count += 1;
            }
        }
        let mean = j / (count as f64 * 0.001);
        let dz = run.center().z - 0.0625;
        println!(
            "{mode:?} commanded {squeeze} N/jaw, ideal {threshold:.7} N/jaw, held height change {dz:.7} m, average jaw support {mean:.7} N"
        );
        if carry {
            assert!(dz > 0.035 && (mean - CLOTH_MASS * GRAVITY).abs() < 0.0003);
        } else {
            assert!(dz < 0.001 && mean.abs() < 1e-6);
        }
    }
}
