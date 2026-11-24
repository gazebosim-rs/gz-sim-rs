// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

struct Rig {
    backend: RapierBackend,
    foot: BodyId,
    tip: BodyId,
    upper: Option<(BodyId, JointId)>,
    hinge: JointId,
}
fn rig(merge: bool, enable_lock: bool) -> Rig {
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let mut link = |position, mass, center, tensor| {
        b.spawn_articulation_link(
            BodySpec {
                position,
                mass: Some(mass),
                velocity: [0.11, -0.17, 0.07],
                shape: Shape::Sphere { radius: 0.03 },
            },
            RotationSpec::default(),
            Some(Inertia {
                center,
                orientation: [0., 0., 0., 1.],
                tensor,
            }),
        )
        .unwrap()
    };
    let moment = [0.0201, 0.0201, 0.025, 0., 0., 0.];
    let foot = if merge {
        link([0.; 3], 2., [0., 0., 0.07], [0.05, 0.05, 0.05, 0., 0., 0.])
    } else {
        link([0.; 3], 1., [0.; 3], moment)
    };
    let upper = (!merge).then(|| link([0., 0., 0.14], 1., [0.; 3], moment));
    let tip = link(
        [0.15, -0.03, 0.42],
        1.4,
        [0.; 3],
        [0.017, 0.022, 0.028, 0., 0., 0.],
    );
    let upper = upper.map(|u| {
        let j = b
            .add_limited_revolute(Some(foot), u, [0., 0., 0.07], [1., 0., 0.], [0., 0.])
            .unwrap();
        (u, j)
    });
    let hinge = b
        .add_limited_revolute(
            Some(upper.map_or(foot, |p| p.0)),
            tip,
            [0.07, 0.04, 0.28],
            [0., 1., 0.],
            [-0.8, 0.6],
        )
        .unwrap();
    if enable_lock {
        b.enable_exact_revolute_locks(foot).unwrap();
    }
    Rig {
        backend: b,
        foot,
        tip,
        upper,
        hinge,
    }
}
fn sites(r: &Rig) -> [ArticulatedContactSite; 2] {
    [
        ArticulatedContactSite {
            body: r.foot,
            local_point: [0.12, -0.06, -0.06],
        },
        ArticulatedContactSite {
            body: r.tip,
            local_point: [-0.04, 0.05, 0.03],
        },
    ]
}
fn vclose(a: [f64; 3], b: [f64; 3]) {
    assert!(
        (Vector::from_array(a) - Vector::from_array(b)).length() < 1e-12,
        "{a:?}, {b:?}"
    );
}
#[test]
fn locked_child_moves_with_its_parent_and_preserves_cross_site_mobility() {
    let mut tree = rig(false, true);
    let mut fused = rig(true, false);
    let dt = Duration::from_micros(500);
    let ts = sites(&tree);
    let fs = sites(&fused);
    for pass in 0..2 {
        if pass == 1 {
            tree.backend.reset().unwrap();
            fused.backend.reset().unwrap();
        }
        assert_eq!(
            tree.backend
                .articulated_sweep(&[tree.foot, tree.tip], dt)
                .unwrap(),
            fused
                .backend
                .articulated_sweep(&[fused.foot, fused.tip], dt)
                .unwrap()
        );
        let ta = tree.backend.prepare_articulated_contacts(&ts).unwrap();
        let fa = fused.backend.prepare_articulated_contacts(&fs).unwrap();
        assert_eq!(
            (
                ta.sample().links,
                fa.sample().links,
                ta.sample().dofs,
                fa.sample().dofs
            ),
            (3, 2, 7, 7)
        );
        for i in 0..2 {
            vclose(ta.sample().velocities[i], fa.sample().velocities[i]);
            vclose(ta.sample().world_points[i], fa.sample().world_points[i]);
            for j in 0..2 {
                for (a, b) in ta
                    .sample()
                    .mobility(i, j)
                    .unwrap()
                    .into_iter()
                    .zip(fa.sample().mobility(i, j).unwrap())
                {
                    vclose(a, b);
                }
            }
        }
        let impulses = [[0.3, -0.8, 1.2], [-0.1, 0.5, -0.31]];
        let a = ta.commit(&impulses).unwrap();
        let b = fa.commit(&impulses).unwrap();
        for (x, y) in a.after_velocities.into_iter().zip(b.after_velocities) {
            vclose(x, y);
        }
        assert!((a.kinetic_energy_change - b.kinetic_energy_change).abs() < 1e-12);
        let upper = tree.upper.unwrap();
        let root = tree.backend.state(tree.foot).unwrap();
        let u = tree.backend.state(upper.0).unwrap();
        let fr = fused.backend.state(fused.foot).unwrap();
        vclose(root.angular_velocity, fr.angular_velocity);
        vclose(u.angular_velocity, fr.angular_velocity);
        vclose(
            ((Vector::from_array(root.velocity) + Vector::from_array(u.velocity)) * 0.5).to_array(),
            fr.velocity,
        );
        assert_eq!(tree.backend.joint_state(upper.1).unwrap().velocity, 0.);
        let jt = tree.backend.joint_state(tree.hinge).unwrap().velocity;
        let jf = fused.backend.joint_state(fused.hinge).unwrap().velocity;
        assert!((jt - jf).abs() < 1e-12, "{jt} vs {jf} on impulse");
        // One native boundary also publishes exactly the locked/fused jump
        // within roundoff. Longer integrations use different tree coordinates.
        let h = Duration::from_nanos(1);
        tree.backend.step(h).unwrap();
        fused.backend.step(h).unwrap();
        let jt = tree.backend.joint_state(tree.hinge).unwrap().velocity;
        let jf = fused.backend.joint_state(fused.hinge).unwrap().velocity;
        assert!((jt - jf).abs() < 1e-12, "{jt} vs {jf} after one nanosecond");
    }
}
#[test]
fn ordinary_engaged_stops_and_a_driven_sibling_remain_excluded() {
    for enable in [false, true] {
        let mut r = rig(false, enable);
        if enable {
            r.backend
                .set_joint_velocity_motor(
                    r.hinge,
                    Some(JointVelocityMotor {
                        target_velocity: 1.,
                        max_force: 1.,
                    }),
                )
                .unwrap();
        }
        let s = sites(&r);
        let before = [r.backend.state(r.foot), r.backend.state(r.tip)];
        assert!(matches!(
            r.backend.prepare_articulated_contacts(&s),
            Err(PhysicsError::Unsupported { .. })
        ));
        assert!(matches!(
            r.backend
                .articulated_sweep(&[r.foot], Duration::from_micros(500)),
            Err(PhysicsError::Unsupported { .. })
        ));
        assert_eq!(before, [r.backend.state(r.foot), r.backend.state(r.tip)]);
    }
}
