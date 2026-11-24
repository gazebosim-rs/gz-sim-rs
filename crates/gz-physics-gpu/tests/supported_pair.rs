// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{
    GpuError, SolidSphere as Body, SpherePairLane, SupportedSpherePairBatch,
    SupportedSpherePairLane as Lane, simulation::supported_sphere_pair_simulation,
};
use std::error::Error;
fn pair() -> Lane {
    let a = Body {
        position: [-0.101, 0., 0.1],
        velocity: [2., 0., 0.],
        angular_velocity: [0., 0., 3.],
        orientation: [0., 0., 0., 1.],
        radius: 0.1,
        inverse_mass: 1.,
    };
    let b = Body {
        position: [0.101, 0., 0.1],
        velocity: [-2., 0., 0.],
        angular_velocity: [0., 0., -2.],
        inverse_mass: 0.25,
        ..a
    };
    Lane {
        pair: SpherePairLane {
            a,
            b,
            gravity: [1., 0., -10.],
            friction: 0.1,
            restitution: 0.5,
            normal_impulse: 0.,
            tangential_impulse: [0.; 3],
            contact_normal: [0.; 3],
            impact_time: -1.,
        },
        plane_normal: [0., 0., 1.],
        plane_offset: 0.,
        support_normal_impulses: [0.; 2],
    }
}
#[test]
fn mismatched_radii_lift_off_spin_and_adhesive_support_are_refused() {
    let s = pair();
    let p = s.pair;
    // Translating both shapes a long way along the plane leaves its
    // normal-coordinate and tangent-velocity admission unchanged.
    let shifted = SpherePairLane {
        a: Body {
            position: [1e14, -0.101, 0.099],
            velocity: [1e12, 0., 0.01],
            ..p.a
        },
        b: Body {
            position: [1e14, 0.101, 0.1],
            ..p.b
        },
        ..p
    };
    for invalid in [
        Lane { pair: shifted, ..s },
        Lane {
            pair: SpherePairLane {
                a: Body {
                    position: [1e14, -0.101, 0.1],
                    ..shifted.a
                },
                ..shifted
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                a: Body {
                    angular_velocity: [0.001, 0., 1e12],
                    ..p.a
                },
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                b: Body {
                    radius: 0.09,
                    ..p.b
                },
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                a: Body {
                    position: [-0.101, 0., 0.10001],
                    ..p.a
                },
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                b: Body {
                    velocity: [-2., 0., 1e-7],
                    ..p.b
                },
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                a: Body {
                    angular_velocity: [0.1, 0., 0.],
                    ..p.a
                },
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                gravity: [0., 0., 0.],
                ..p
            },
            ..s
        },
        Lane {
            pair: SpherePairLane {
                gravity: [0., 0., 10.],
                ..p
            },
            ..s
        },
        Lane {
            plane_normal: [0., 0., 1.1],
            ..s
        },
        Lane {
            plane_normal: [f64::NAN, 0., 1.],
            ..s
        },
        Lane {
            plane_offset: f64::NAN,
            ..s
        },
        Lane {
            support_normal_impulses: [0.1, 0.],
            ..s
        },
    ] {
        assert!(matches!(
            SupportedSpherePairBatch::new("unused", vec![s, invalid]),
            Err(GpuError::InvalidLane(1))
        ));
    }
}
#[test]
#[ignore = "requires an explicitly selected native-f64 Vulkan adapter"]
fn actual_three_contacts_supply_the_two_weights_and_pair_momentum() -> Result<(), Box<dyn Error>> {
    let gpu = std::env::var("GZ_GPU_ADAPTER")?;
    let s = pair();
    let mut sim = supported_sphere_pair_simulation(&gpu, vec![s])?;
    sim.tick()?;
    assert_eq!(sim.world().resource::<SupportedSpherePairBatch>().tick(), 0);
    sim.step_exact(1)?;
    let a = sim.world().resource::<SupportedSpherePairBatch>().lanes()[0];
    assert!(a.pair.normal_impulse > 4.);
    assert_eq!(a.support_normal_impulses, [0.01, 0.04]);
    for x in [a.pair.a, a.pair.b] {
        assert_eq!(x.position[2], 0.1);
        assert_eq!(x.velocity[2], 0.);
        assert_eq!(&x.angular_velocity[..2], &[0., 0.]);
    }
    let v0 = s.pair.a.velocity[0] + 4. * s.pair.b.velocity[0];
    let v1 = a.pair.a.velocity[0] + 4. * a.pair.b.velocity[0];
    assert!((v1 - v0 - 5. * 0.001).abs() < 1e-12);
    sim.reset()?;
    assert_eq!(
        sim.world().resource::<SupportedSpherePairBatch>().lanes(),
        [s]
    );
    sim.step_exact(1)?;
    assert_eq!(
        sim.world().resource::<SupportedSpherePairBatch>().lanes(),
        [a]
    );
    Ok(())
}
