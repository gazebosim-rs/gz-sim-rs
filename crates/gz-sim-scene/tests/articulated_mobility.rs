// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
#[path = "support/gripper_cartesian.rs"]
mod independent;
use glam::DVec3;
use gz_sim_scene::articulated_contact;
use independent::cartesian;
use rapier3d_f64::na::DVector;

#[test]
fn floating_gripper_matches_cartesian_revolute_oracle_and_real_link_momenta() {
    let mut g = Gripper::new(WORLD).unwrap();
    let (mut worst_k, mut worst_v, mut worst_p, mut worst_l, mut worst_e) =
        (0f64, 0f64, 0f64, 0f64, 0f64);
    let mut cross = 0f64;
    for k in 0..64usize {
        let s = g.states();
        let before = g.sample().unwrap();
        let kcart = cartesian(s, CENTER, false, LOCAL_POINTS);
        assert_eq!((before.dofs, before.links), (8, 3));
        let mut impulses = [DVec3::ZERO; 3];
        if k % 5 == 0 {
            // Equal, opposite impulses demonstrate distinct net moment and internal motion.
            impulses[1] = DVec3::new(0.012, -0.02, 0.017);
            impulses[2] = -impulses[1];
        } else {
            let t = k as f64;
            impulses[1 + (k % 2)] =
                0.011 * DVec3::new((1.3 * t).sin(), (2.1 * t).cos(), (0.75 * t).sin());
        }
        let flat = DVector::from_iterator(9, impulses.iter().flat_map(|p| p.to_array()));
        let delta = &kcart * flat;
        for i in 0..3 {
            assert!(
                DVec3::from(before.world_points[i]).distance(pose(s[i], LOCAL_POINTS[i])) < 2e-13
            );
            worst_v = worst_v.max(DVec3::from(before.velocities[i]).distance(point_velocity(
                i,
                s[i],
                LOCAL_POINTS[i],
            )));
            for source in 0..3 {
                let block = before.mobility(i, source).unwrap();
                for a in 0..3 {
                    for b in 0..3 {
                        worst_k =
                            worst_k.max((block[a][b] - kcart[(3 * i + a, 3 * source + b)]).abs());
                    }
                }
            }
        }
        let got = g.apply(impulses).unwrap();
        assert_eq!(got.before, before);
        let after = g.states();
        for i in 0..3 {
            assert_eq!(s[i].position, after[i].position);
            assert_eq!(s[i].orientation, after[i].orientation);
            let expected = DVec3::from(before.velocities[i])
                + DVec3::new(delta[3 * i], delta[3 * i + 1], delta[3 * i + 2]);
            let measured = point_velocity(i, after[i], LOCAL_POINTS[i]);
            worst_v = worst_v
                .max(measured.distance(expected))
                .max(measured.distance(got.after_velocities[i].into()));
            if impulses[i] == DVec3::ZERO {
                cross = cross.max(measured.distance(before.velocities[i].into()));
            }
        }
        let (p0, l0, e0) = budgets(s);
        let (p1, l1, e1) = budgets(after);
        worst_p = worst_p.max((p1 - p0 - impulses.iter().sum::<DVec3>()).length());
        let angular: DVec3 = before
            .world_points
            .iter()
            .zip(impulses)
            .map(|(p, j)| DVec3::from(*p).cross(j))
            .sum();
        worst_l = worst_l.max((l1 - l0 - angular).length());
        worst_e = worst_e.max((e1 - e0 - got.kinetic_energy_change).abs());
        g.sim.step_exact(1).unwrap();
    }
    println!(
        "64 moving configurations, 5184 Cartesian mobility coefficients, maximum block {worst_k:e} kg^-1, point velocity {worst_v:e} m/s, P {worst_p:e} Ns, L {worst_l:e} Nms, energy {worst_e:e} J, unloaded site response {cross:e} m/s"
    );
    assert!(
        worst_k < 2e-11
            && worst_v < 2e-12
            && worst_p < 2e-12
            && worst_l < 2e-12
            && worst_e < 2e-12
            && cross > 0.001
    );
}

#[test]
fn invalid_batches_are_atomic_and_an_engaged_native_contact_is_refused() {
    let mut g = Gripper::new(WORLD).unwrap();
    let sites = g.sites();
    let state = g.states();
    let initial = g.sample().unwrap();
    for bad in [
        vec![],
        vec![[0.; 3]; 4],
        vec![[0.; 3], [f64::NAN, 0., 0.], [0.; 3]],
        vec![[f64::MAX; 3]; 3],
    ] {
        assert!(articulated_contact::apply(&mut g.sim, &sites, |_| Ok(bad)).is_err());
        assert_eq!(g.states(), state);
        assert_eq!(g.sample().unwrap(), initial);
    }
    assert!(
        articulated_contact::apply(&mut g.sim, &sites, |_| Err(
            gz_physics::PhysicsError::InvalidValue
        ))
        .is_err()
    );
    assert_eq!(g.states(), state);
    assert_eq!(g.apply([DVec3::ZERO; 3]).unwrap().kinetic_energy_change, 0.);
    let xml=WORLD.replace("</world>","<model name='block'><static>true</static><pose>0.32 0.29 0.58 0 0 0</pose><link name='fixed'><collision name='ball'><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model></world>");
    let mut collision = Gripper::new(&xml).unwrap();
    let original = collision.states();
    assert!(collision.sample().is_err());
    assert_eq!(collision.states(), original);
}

#[test]
fn grounded_gripper_projects_the_world_hinge_and_preserves_only_its_free_axis_moment() {
    let mut g = Gripper::new(&centered_fixture(true)).unwrap();
    let mut worst = 0f64;
    let mut axis_error = 0f64;
    let mut support = 0f64;
    for k in 0..48 {
        let state = g.states();
        let points = g.sample().unwrap();
        assert_eq!((points.links, points.dofs), (4, 3));
        let independent = cartesian(state, [[0.; 3]; 3], true, LOCAL_POINTS);
        for a in 0..9 {
            for b in 0..9 {
                let value = points.mobility(a / 3, b / 3).unwrap()[a % 3][b % 3];
                worst = worst.max((value - independent[(a, b)]).abs());
            }
        }
        let t = k as f64;
        let impulse = [
            DVec3::ZERO,
            DVec3::new(0.01 * t.cos(), -0.007, 0.018),
            DVec3::new(-0.006, 0.003 * t.sin(), -0.004),
        ];
        let receipt = g.apply(impulse).unwrap();
        let after = g.states();
        assert!(DVec3::from(after[0].velocity).length() < 1e-12);
        let anchor = pose(state[0], [0.; 3]);
        let mut dp = DVec3::ZERO;
        let mut dl = DVec3::ZERO;
        let mut de = 0.;
        for i in 0..3 {
            let a = state[i];
            let b = after[i];
            let v0 = DVec3::from(a.velocity);
            let v1 = DVec3::from(b.velocity);
            let w0 = DVec3::from(a.angular_velocity);
            let w1 = DVec3::from(b.angular_velocity);
            let pi = MASS[i] * (v1 - v0);
            dp += pi - impulse[i];
            dl += (pose(a, [0.; 3]) - anchor).cross(pi) + angular_mass(i, a, w1 - w0, false)
                - (pose(a, LOCAL_POINTS[i]) - anchor).cross(impulse[i]);
            de += 0.5
                * (MASS[i] * (v1.length_squared() - v0.length_squared())
                    + w1.dot(angular_mass(i, b, w1, false))
                    - w0.dot(angular_mass(i, a, w0, false)));
        }
        assert!((de - receipt.kinetic_energy_change).abs() < 1e-13);
        axis_error = axis_error.max(dl.z.abs());
        support = support.max(dp.length());
        g.sim.step_exact(1).unwrap();
    }
    println!(
        "48 grounded moving configurations, 3888 Cartesian mobility coefficients, max {worst:e} kg^-1, angular impulse residual about free ground hinge {axis_error:e} Nms, measured support impulse {support:e} Ns"
    );
    assert!(worst < 2e-11 && axis_error < 1e-12 && support > 0.01);
}

#[test]
fn authored_limits_motors_prescribed_motion_and_redirected_handles_refuse_before_writes() {
    use gz_physics::{JointVelocityMotor, PhysicsBackend};
    use gz_sim_core::entity_by_id;
    use gz_sim_scene::{ArticulationTier, BodyHandle, JointHandle, set_model_fidelity};
    let mut g = Gripper::new(WORLD).unwrap();
    let sites = g.sites();
    let baseline = g.states();
    set_model_fidelity(&mut g.sim, g.id, ArticulationTier::Prescribed).unwrap();
    assert!(
        articulated_contact::apply(&mut g.sim, &sites, |_| panic!("unavailable mobility")).is_err()
    );
    assert_eq!(g.states(), baseline);
    set_model_fidelity(&mut g.sim, g.id, ArticulationTier::Dynamic).unwrap();
    let joint = g
        .sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<JointHandle>().map(JointHandle::id))
        .unwrap();
    g.sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_motor(
            joint,
            Some(JointVelocityMotor {
                target_velocity: 0.,
                max_force: 0.5,
            }),
        )
        .unwrap();
    assert!(g.sample().is_err());
    assert_eq!(g.states(), baseline);
    g.sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_motor(joint, None)
        .unwrap();
    assert!(g.sample().is_ok());
    let (a, b) = (
        entity_by_id(g.sim.world(), g.ids[1].0).unwrap(),
        entity_by_id(g.sim.world(), g.ids[2].0).unwrap(),
    );
    let x = g
        .sim
        .world_mut()
        .entity_mut(a)
        .take::<BodyHandle>()
        .unwrap();
    let y = g
        .sim
        .world_mut()
        .entity_mut(b)
        .take::<BodyHandle>()
        .unwrap();
    g.sim.world_mut().entity_mut(a).insert(y);
    g.sim.world_mut().entity_mut(b).insert(x);
    assert!(g.sample().is_err());
    assert_eq!(g.states(), baseline);
    let mut stopped =
        Gripper::new(&WORLD.replace("<lower>-0.42</lower>", "<lower>0</lower>")).unwrap();
    let before = stopped.states();
    assert!(stopped.sample().is_err());
    assert_eq!(stopped.states(), before);
}

#[test]
fn simultaneous_site_permutation_and_identical_site_superposition_preserve_physical_response() {
    let mut a = Gripper::new(WORLD).unwrap();
    let mut b = Gripper::new(WORLD).unwrap();
    let mut c = Gripper::new(WORLD).unwrap();
    let k = 2;
    let sites = a.sites();
    let impulses = [
        DVec3::new(0.015, -0.01, 0.017),
        DVec3::new(-0.03, 0.04, 0.022),
        DVec3::new(0.01, 0.02, -0.015),
    ];
    a.apply(impulses).unwrap();
    let bsites = [sites[k], sites[0], sites[1]];
    articulated_contact::apply(&mut b.sim, &bsites, |_| {
        Ok(vec![
            impulses[k].to_array(),
            impulses[0].to_array(),
            impulses[1].to_array(),
        ])
    })
    .unwrap();
    let csites = [sites[0], sites[1], sites[2], sites[1]];
    articulated_contact::apply(&mut c.sim, &csites, |_| {
        Ok(vec![
            impulses[0].to_array(),
            (0.75 * impulses[1]).to_array(),
            impulses[2].to_array(),
            (0.25 * impulses[1]).to_array(),
        ])
    })
    .unwrap();
    for (i, s) in a.states().into_iter().enumerate() {
        for other in [b.states()[i], c.states()[i]] {
            assert_eq!(s.position, other.position);
            assert_eq!(s.orientation, other.orientation);
            assert!(DVec3::from(s.velocity).distance(other.velocity.into()) < 1e-15);
            assert!(
                DVec3::from(s.angular_velocity).distance(other.angular_velocity.into()) < 1e-15
            );
        }
    }
}
