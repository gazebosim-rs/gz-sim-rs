// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    GpuError, SpherePlaneBatch, SpherePlaneLane as Lane, simulation::sphere_plane_simulation,
};
use std::error::Error;

fn supported() -> Lane {
    Lane {
        position: [0., 0., 0.2],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
        orientation: [0., 0., 0., 1.],
        gravity: [0., 0., -10.],
        plane_normal: [0., 0., 1.],
        plane_offset: 0.,
        radius: 0.2,
        inverse_mass: 0.5,
        friction: 0.5,
        normal_impulse: 0.,
        tangential_impulse: [0.; 3],
    }
}

#[test]
fn invalid_fixture_is_rejected_before_selecting_device() {
    let s = supported();
    for bad in [
        Lane {
            angular_velocity: [1e300, 0., 0.],
            ..s
        },
        Lane {
            gravity: [f64::NAN, 0., 1.],
            ..s
        },
        Lane {
            plane_normal: [0., 0., 2.],
            ..s
        },
        Lane {
            orientation: [0.; 4],
            ..s
        },
        Lane { radius: 0., ..s },
        Lane {
            inverse_mass: 0.,
            ..s
        },
        Lane {
            inverse_mass: 1e-320,
            ..s
        },
        Lane {
            radius: 1e-180,
            ..s
        },
        Lane { radius: 1e180, ..s },
        Lane {
            inverse_mass: f64::MAX,
            radius: 10.,
            ..s
        },
        Lane {
            friction: -0.01,
            ..s
        },
        Lane {
            position: [0., 0., 0.19],
            ..s
        },
        Lane {
            normal_impulse: 0.001,
            ..s
        },
        Lane {
            tangential_impulse: [1., 0., 0.],
            ..s
        },
    ] {
        assert!(matches!(
            SpherePlaneBatch::new("unused", vec![s, bad]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}

#[test]
#[ignore = "requires an explicitly selected Vulkan device with native shader f64"]
fn spin_slip_grazing_and_reversed_normal_have_expected_impulses() -> Result<(), Box<dyn Error>> {
    let n = std::env::var("GZ_GPU_ADAPTER")?;
    let s = supported();
    let initial = vec![
        s,
        // High spin and no translation. Slip at the lowest point is -X.
        Lane {
            angular_velocity: [0., 50., 0.],
            ..s
        },
        // Pure rolling plus twist about the normal is torque-free.
        Lane {
            velocity: [1., 0., 0.],
            angular_velocity: [0., 5., 9.],
            ..s
        },
        // An initial gap with both endpoints on opposite sides of the plane.
        Lane {
            position: [0., 0., 0.3],
            velocity: [0., 0., -300.],
            ..s
        },
        Lane {
            velocity: [0., 0., 2.],
            ..s
        },
        // A plane above the sphere; force and angular impulse rotate with it.
        Lane {
            position: [-0.2, 0., 0.],
            gravity: [10., 0., 0.],
            plane_normal: [-1., 0., 0.],
            ..s
        },
        Lane {
            velocity: [1., 0., 0.],
            friction: 0.,
            ..s
        },
    ];
    let mut sim = sphere_plane_simulation(&n, initial.clone())?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<SpherePlaneBatch>().tick(), 0);
    sim.step_exact(1)?;
    let got = sim.world().resource::<SpherePlaneBatch>().lanes().to_vec();
    let near = |a: f64, b: f64| assert!((a - b).abs() < 1e-12, "{a} vs {b}");
    near(got[0].normal_impulse, 0.02);
    assert_eq!(got[0].velocity, [0.; 3]);
    near(got[1].tangential_impulse[0], 0.01);
    near(
        got[1].angular_velocity[1],
        50. - 0.2 * 0.01 / (0.4 * 2. * 0.2 * 0.2),
    );
    near(got[2].tangential_impulse[0], 0.);
    near(got[2].angular_velocity[2], 9.);
    near(got[3].normal_impulse, 600.02);
    near(got[3].position[2], 0.2);
    near(got[4].normal_impulse, 0.);
    near(got[4].velocity[2], 1.99);
    near(got[5].normal_impulse, 0.02);
    near(got[5].position[0], -0.2);
    near(got[6].tangential_impulse[0], 0.);
    near(got[6].velocity[0], 1.);
    for lane in &got {
        near(lane.orientation.iter().map(|x| x * x).sum(), 1.);
    }
    sim.reset()?;
    assert_eq!(sim.world().resource::<SpherePlaneBatch>().lanes(), initial);
    sim.step_exact(1)?;
    assert_eq!(sim.world().resource::<SpherePlaneBatch>().lanes(), got);
    Ok(())
}
