// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn lane() -> PlanarArmLane {
    PlanarArmLane {
        angle: [[1., 0.]; 2],
        speed: [0.; 2],
        length: [1.; 2],
        center: [0.5; 2],
        mass: [1.; 2],
        inertia: [1. / 12.; 2],
        tip_mass: 0.,
        gravity: 9.81,
        target: [[1., 0.]; 2],
        stiffness: [0.; 2],
        damping: [0.; 2],
        effort_limit: [1.; 2],
        effort: [0.; 2],
    }
}
fn one(mut l: PlanarArmLane) -> PlanarArmLane {
    let v = bytemuck::cast_mut::<_, [f64; WIDTH]>(&mut l);
    for (i, value) in tick(v, &[0.001, 0., 1., 0.5]) {
        v[i] = value;
    }
    l
}
#[test]
fn horizontal_hinges_obey_coupled_mass_and_payload() {
    let mut l = lane();
    for payload in [0., 0.25, 10.] {
        l.tip_mass = payload;
        let after = one(l);
        // Horizontal unit rods: COM inertias 1/12; payload Jacobian [2,1].
        let m11 = 8. / 3. + 4. * payload;
        let m12 = 5. / 6. + 2. * payload;
        let m22 = 1. / 3. + payload;
        let [w1, w2] = after.speed.map(|w| w / 0.001);
        assert!((m11 * w1 + m12 * w2 + l.gravity * (2. + 2. * payload)).abs() < 1e-12);
        assert!((m12 * w1 + m22 * w2 + l.gravity * (0.5 + payload)).abs() < 1e-12);
        assert_eq!(after.effort, [0.; 2]);
    }
}
#[test]
fn relative_targets_saturate_and_counter_rotate_the_links() {
    let mut l = lane();
    l.gravity = 0.;
    l.target = [[0., 1.], [0., -1.]];
    l.stiffness = [7.; 2];
    let result = one(l);
    assert_eq!(result.effort, [1., -1.]);
    assert!(result.speed[0] > 0. && result.speed[1] < 0.);
    l.target = l.angle;
    let resting = one(l);
    assert_eq!(resting.speed, [0.; 2]);
    assert_eq!(resting.points(), [[1., 0.], [2., 0.]]);
    l.angle = [[0., 1.], [0., 1.]];
    assert_eq!(l.points(), [[0., 1.], [-1., 1.]]);
}
#[test]
fn invalid_geometry_and_numeric_state_are_rejected_before_device_selection() {
    let good = lane();
    assert!(good.valid());
    let invalid: [fn(&mut PlanarArmLane); 17] = [
        |v| v.speed[0] = f64::NAN,
        |v| v.angle[1] = [0.9, 0.],
        |v| v.target[0] = [0., 0.],
        |v| v.tip_mass = -0.1,
        |v| v.gravity = -1.,
        |v| v.effort[0] = 1.,
        |v| v.length[0] = 0.,
        |v| v.center[1] = -1.,
        |v| v.mass[1] = 0.,
        |v| v.inertia[0] = 0.,
        |v| v.stiffness[0] = -1.,
        |v| v.damping[0] = -1.,
        |v| v.effort_limit[0] = -1.,
        |v| v.center[0] = 2.,
        |v| v.mass[1] = f64::MAX,
        |v| v.inertia = [1e300; 2],
        |v| v.speed[0] = f64::MAX,
    ];
    for tweak in invalid {
        let mut bad = good;
        tweak(&mut bad);
        assert!(!bad.valid());
        assert!(matches!(
            PlanarArmBatch::new("fake", vec![good, bad]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}

#[test]
#[ignore = "requires GZ_GPU_ADAPTER selecting an f64 Vulkan device"]
fn arm_commands_preserve_pose_and_clock_and_reset_to_construction() {
    let name = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = crate::simulation::planar_arm_simulation(&name, vec![lane()]).unwrap();
    sim.step_exact(1).unwrap();
    let mut batch = sim.world_mut().resource_mut::<PlanarArmBatch>();
    let before = batch.lanes()[0];
    assert!(matches!(
        batch.set_target_angles(0, [f64::NAN, 0.]),
        Err(GpuError::InvalidArmTarget(0))
    ));
    assert!(matches!(
        batch.set_target_angles(1, [0.; 2]),
        Err(GpuError::InvalidArmTarget(1))
    ));
    assert_eq!(
        bytemuck::bytes_of(&before),
        bytemuck::bytes_of(&batch.lanes()[0])
    );
    batch.set_target_angles(0, [2., -1.]).unwrap();
    assert_eq!(batch.tick(), 1);
    assert_eq!(batch.lanes()[0].angle, before.angle);
    assert_eq!(batch.lanes()[0].speed, before.speed);
    assert_ne!(batch.lanes()[0].target, before.target);
    sim.reset().unwrap();
    assert_eq!(
        bytemuck::bytes_of(&lane()),
        bytemuck::bytes_of(&sim.world().resource::<PlanarArmBatch>().lanes()[0])
    );
}
