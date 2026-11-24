// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{MASS, Rig, WORLD};
use glam::{DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;

const STIFFNESS: [f64; 2] = [15_000., 60_000.];

/// A rectangular 200 g block fits entirely on each pad. Its four face corners
/// provide a control for accidentally applying the full pad stiffness four times.
fn block_world(stiffness: [f64; 2]) -> String {
    let material = format!(
        r#"<model name="gripper">
          <plugin name="gz::sim::systems::ContactMaterial" filename="gz-sim-contact-material-system">
            <normal_stiffness link="left">{}</normal_stiffness>
            <normal_stiffness link="right">{}</normal_stiffness>
          </plugin>"#,
        stiffness[0], stiffness[1]
    );
    // Extents 30 x 20 x 24 mm. Gravity and shear are disabled so the independently
    // measured overlap tests only the normal material law.
    WORLD.replace(r#"<model name="gripper">"#, &material)
        .replace("<sphere><radius>0.015</radius></sphere>", "<box><size>0.03 0.02 0.024</size></box>")
        .replace("<ixx>1.8e-05</ixx><iyy>1.8e-05</iyy><izz>1.8e-05</izz>",
                 "<ixx>0.00001626666666666667</ixx><iyy>0.0000246</iyy><izz>0.00002166666666666667</izz>")
        .replace("<mu>0.5</mu><mu2>0.5</mu2>", "<mu>0</mu><mu2>0</mu2>")
}

fn measured_faces(rig: &Rig) -> ([f64; 2], [f64; 2], [usize; 2]) {
    let backend = rig.sim.world().resource::<PhysicsBackend>();
    let parcel = backend.state(rig.parcel).unwrap();
    let mut force = [0.; 2];
    let mut points = [0; 2];
    let overlap = std::array::from_fn(|side| {
        let pad = backend.state(rig.pads[side]).unwrap();
        let inward = if side == 0 { DVec3::X } else { DVec3::NEG_X };
        let n = DQuat::from_array(pad.orientation) * inward;
        let block_normal = DQuat::from_array(parcel.orientation) * inward;
        let face_center = DVec3::from(parcel.position) - block_normal * 0.015;
        0.010 - n.dot(face_center - DVec3::from(pad.position))
    });
    for patch in backend
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_patches()
    {
        let side = rig
            .pads
            .iter()
            .position(|p| {
                (patch.a == *p && patch.b == rig.parcel) || (patch.b == *p && patch.a == rig.parcel)
            })
            .expect("only the two pad faces touch");
        for point in patch.points {
            assert!(point.normal_impulse > 0.);
            points[side] += 1;
            force[side] += point.normal_impulse / rig.sim.step_duration().as_secs_f64();
        }
    }
    (overlap, force, points)
}

#[test]
fn a_four_point_face_shares_one_pad_spring_when_loading_and_unloading() {
    for stiffness in [STIFFNESS, [STIFFNESS[1], STIFFNESS[0]]] {
        let mut rig = Rig::new(&block_world(stiffness), 200, 0.);
        for drive in [3., 6., 3.] {
            rig.drive([drive; 2]);
            rig.sim.step_exact(30_000).unwrap();
            let (overlap, force, points) = measured_faces(&rig);
            eprintln!(
                "stiffness={stiffness:?} drive_n={drive} contact_points={points:?} overlap_m={overlap:?} normal_n={force:?}"
            );
            assert_eq!(points, [4; 2]);
            for side in 0..2 {
                assert!((force[side] - drive).abs() < 0.01);
                assert!((overlap[side] - drive / stiffness[side]).abs() < 2e-6);
            }
            assert!((MASS * rig.state().velocity[0]).abs() < 1e-5);
        }
    }
}

/// Compare matched 100 µs public ticks. Contact readout, input submission and
/// sorting the timing samples all occur outside the measured section.
#[test]
fn compare_step_cost_during_grip_slip_and_release() {
    use std::time::Instant;
    const STIFF_WORLD: &str = include_str!("../../../../examples/worlds/local_stiff_pad_grasp.sdf");
    for batch in 0..5 {
        let worlds = if batch % 2 == 0 {
            [WORLD, STIFF_WORLD]
        } else {
            [STIFF_WORLD, WORLD]
        };
        for world in worlds {
            let mut rig = Rig::new(world, 100, -super::GRAVITY);
            for (operation, drive) in [("hold", 3.), ("slip", 1.5), ("release", -3.)] {
                rig.drive([drive; 2]);
                rig.sim.step_exact(200).unwrap();
                let start = rig.state();
                let mut times_ns = [0u64; 500];
                for ns in &mut times_ns {
                    let clock = Instant::now();
                    rig.sim.step_exact(1).unwrap();
                    *ns = u64::try_from(clock.elapsed().as_nanos()).unwrap();
                }
                let loads = rig.read();
                let end = rig.state();
                for normal in loads.normal {
                    let expected = drive.max(0.);
                    assert!((normal / 0.0001 - expected).abs() < 0.005);
                }
                if operation == "hold" {
                    assert!((end.position[2] - start.position[2]).abs() < 1e-6);
                } else {
                    assert!(end.position[2] < start.position[2] - 0.001);
                }
                times_ns.sort_unstable();
                let material = if world == WORLD {
                    "default"
                } else {
                    "30000N/m"
                };
                println!(
                    "cost batch={batch} material={material} operation={operation} p50_ns={} p95_ns={} p99_ns={}",
                    times_ns[249], times_ns[474], times_ns[494]
                );
            }
        }
    }
}
