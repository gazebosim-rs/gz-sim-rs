// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::{Value, json};
const MATCHED: &str = include_str!(
    "../../../../docs/measurements/local-manipulation/gpu-shelf-glance-mirrored-states.json"
);
fn point(v: &Value) -> DVec3 {
    serde_json::from_value::<[f64; 3]>(v.clone())
        .unwrap()
        .into()
}
fn matched_input(s: &Value) -> ContactLane {
    let q = DQuat::from_array(serde_json::from_value(s["orientation"].clone()).unwrap());
    let offset = DVec3::new(0.005, 0., -0.002);
    let inertia = DVec3::new(0.00001450666666666667, 0.00002602666666667, 0.0000192);
    // The hand has already separated. Replay the same three shelf kernels with a
    // readback at each phase. Only the relevant material/shelf words are consumed.
    let mut input = ContactLane::zeroed();
    input.half = [0.024, 0.012, 0.020];
    input.center = offset.to_array();
    input.lane.carry.hand.ball_mass = 0.08;
    input.lane.carry.hand.gravity = 9.81;
    input.lane.carry.shelf_center = [0.18, 0.007];
    input.lane.carry.shelf_half = [0.06, 0.007];
    input.lane.carry.shelf_friction = 0.45;
    input.lane.inertia = inertia.to_array();
    input.lane.body = FreeBoxState {
        mass_center: (point(&s["position"]) + q * offset).to_array(),
        orientation: q.to_array(),
        center_velocity: point(&s["center_velocity"]).to_array(),
        momentum: (q * (inertia * (q.inverse() * point(&s["omega"])))).to_array(),
    };
    input
}

struct Probe {
    output: ContactLane,
    torques: [DVec3; 8],
    steps: Vec<Value>,
    speeds: Vec<[f64; 8]>,
}
fn probe(adapter: &str, mut input: ContactLane, split: Option<(usize, f64)>) -> Probe {
    input.split_time = split.map_or(0., |s| s.1);
    let offset = DVec3::from(input.center);
    let inertia = DVec3::from(input.lane.inertia);
    let mut device =
        ResidentBatch::new_profile(adapter, vec![input], |_| true, predict::<0>, None).unwrap();
    let pass = |f| {
        device
            .gpu
            .dispatch(
                f,
                0,
                device.device_info().state_bytes,
                size_of::<ContactLane>() as u64,
                1,
                None,
            )
            .unwrap()
    };
    let solve_finish = [pass(solve), pass(finish)];
    let mut torques = [DVec3::ZERO; 8];
    let mut per_step = Vec::new();
    let mut point_speeds = Vec::new();
    let mut schedule = vec![(pass(predict::<0>), H); SUBSTEPS];
    if let Some(split) = split {
        schedule.splice(
            split.0..=split.0,
            [
                (pass(predict::<1>), split.1),
                (pass(predict::<2>), H - split.1),
            ],
        );
    }
    let mut elapsed = 0.;
    for (stage, dt) in schedule {
        elapsed += dt;
        device
            .advance_sequence(1, Some(&[stage]), |_| Ok(()))
            .unwrap();
        let incoming = device.committed[0].lane.body;
        let q = DQuat::from_array(incoming.orientation);
        let omega = q * ((q.inverse() * DVec3::from(incoming.momentum)) / inertia);
        device
            .advance_sequence(1, Some(&solve_finish), |_| Ok(()))
            .unwrap();
        let result = &device.committed[0];
        let mut impulse = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let end = result.lane.body;
        let out_omega = q * ((q.inverse() * DVec3::from(end.momentum)) / inertia);
        let mut speeds = [0.; 8];
        for (i, j) in result.current.iter().copied().enumerate() {
            let r = q
                * (DVec3::from_array(std::array::from_fn(|axis| {
                    if i & (1 << axis) == 0 {
                        -input.half[axis]
                    } else {
                        input.half[axis]
                    }
                })) - offset);
            let velocity = DVec3::from(end.center_velocity) + out_omega.cross(r);
            speeds[i] = velocity.x.hypot(velocity.y);
            let j = DVec3::from(j);
            assert!(j.z >= 0. && j.x.hypot(j.y) <= 0.45 * j.z + 1e-15);
            impulse += j;
            let a = r.cross(j);
            torque += a;
            torques[i] += a;
        }
        point_speeds.push(speeds);
        assert!(
            (0.08 * (DVec3::from(end.center_velocity) - DVec3::from(incoming.center_velocity))
                - impulse)
                .length()
                < 1e-12
        );
        assert!(
            (DVec3::from(end.momentum) - DVec3::from(incoming.momentum) - torque).length() < 1e-13
        );
        per_step.push(json!({"end_us":elapsed*1e6,"incoming_q":q.to_array(),"incoming_v":incoming.center_velocity,"incoming_omega":omega.to_array(),"vertex_impulses_ns":result.current,"angular_impulse_nms":torque.to_array(),"outgoing_v":end.center_velocity}));
    }
    assert!((elapsed - 0.001).abs() < 1e-18);
    Probe {
        output: device.committed[0],
        torques,
        steps: per_step,
        speeds: point_speeds,
    }
}

#[test]
#[ignore = "requires native f64 Vulkan hardware"]
fn matched_glancing_corner_resolves_each_device_substep_and_closes_the_wrench() {
    let src: Value = serde_json::from_str(MATCHED).unwrap();
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut all = serde_json::Map::new();
    for (name, data) in src.as_object().unwrap() {
        let s = &data["gpu"]["incoming"];
        let input = matched_input(s);
        let previous = SpatialShelfLoads {
            world_impulses: serde_json::from_value(
                data["gpu_preceding_contact"]["vertices"].clone(),
            )
            .unwrap(),
            ..Default::default()
        };
        let split = competing_corner(&input, &previous).unwrap();
        assert_eq!(split.0, 3);
        let mut trials = serde_json::Map::new();
        for corrected in [false, true] {
            let probe = probe(&adapter, input, corrected.then_some(split));
            let output = probe.output;
            let torques = probe.torques;
            if !corrected {
                for (i, j) in output.loads.world_impulses.iter().enumerate() {
                    assert!(
                        DVec3::from(*j).distance(point(&data["gpu_contact"]["vertices"][i]))
                            < 1e-11
                    );
                }
                assert!(
                    torques
                        .iter()
                        .copied()
                        .sum::<DVec3>()
                        .distance(point(&data["gpu_contact"]["angular"]))
                        < 1e-13
                );
            }
            trials.insert(
            if corrected {"retimed"}else{"uniform"}.to_owned(),
            json!({"substeps":probe.steps,"corner_angular_impulses_nms":torques.map(|a|a.to_array())}),
        );
        }
        all.insert(name.clone(), Value::Object(trials));
    }
    println!("glancing_shelf_device {}", Value::Object(all));
}

#[test]
fn splitting_requires_one_supported_corner_and_an_isolated_new_arrival() {
    let mut input = ContactLane::zeroed();
    input.half = [0.024, 0.012, 0.020];
    input.lane.inertia = [0.0000145, 0.000026, 0.0000192];
    input.lane.carry.hand.gravity = 9.81;
    let q = DQuat::from_rotation_x(0.4) * DQuat::from_rotation_y(-0.02);
    let old = q * (-DVec3::from(input.half));
    let edge = q * (2. * input.half[0] * DVec3::X);
    input.lane.body.orientation = q.to_array();
    input.lane.body.mass_center[2] = -old.z;
    let motion = |v: &mut ContactLane, dt: f64| {
        let omega = DVec3::Y * (edge.z / (dt * edge.x));
        v.lane.body.momentum =
            (q * (DVec3::from(v.lane.inertia) * (q.inverse() * omega))).to_array();
        v.lane.body.center_velocity[2] = -omega.cross(old).z;
    };
    let mut previous = SpatialShelfLoads::default();
    assert!(competing_corner(&input, &previous).is_none());
    previous.world_impulses[0][2] = 1e-5;
    motion(&mut input, 0.000430);
    let (i, dt) = competing_corner(&input, &previous).unwrap();
    assert_eq!(i, 3);
    assert!((dt + 3. * H - 0.000430).abs() < 1e-15);
    let mut extra = previous;
    extra.world_impulses[2][2] = 1e-6;
    assert!(competing_corner(&input, &extra).is_none());
    let mut separated = input;
    separated.lane.body.mass_center[2] += 0.001;
    assert!(competing_corner(&separated, &previous).is_none());
    let mut departing = input;
    departing.lane.body.center_velocity[2] += 0.5;
    assert!(competing_corner(&departing, &previous).is_none());
    for t in [0.0015, 4. * H, 4. * H - 0.1e-9, -0.00043] {
        motion(&mut input, t);
        assert!(competing_corner(&input, &previous).is_none());
    }
    // Two distinct new neighbors are both sweeping toward the shelf near the
    // current support. Their order needs a different event law.
    let mut neighbors = input;
    let q = DQuat::from_rotation_x(0.02) * DQuat::from_rotation_y(-0.02);
    let old = q * (-DVec3::from(input.half));
    let omega = DVec3::new(-50., 50., 0.);
    neighbors.lane.body.orientation = q.to_array();
    neighbors.lane.body.mass_center[2] = -old.z;
    neighbors.lane.body.center_velocity[2] = -omega.cross(old).z;
    neighbors.lane.body.momentum =
        (q * (DVec3::from(input.lane.inertia) * (q.inverse() * omega))).to_array();
    assert!(competing_corner(&neighbors, &previous).is_none());
    let mut flat = input;
    flat.lane.body.orientation = DQuat::IDENTITY.to_array();
    flat.lane.body.mass_center[2] = input.half[2];
    flat.lane.body.momentum = [0.; 3];
    flat.lane.body.center_velocity = [0.; 3];
    assert!(competing_corner(&flat, &previous).is_none());
}

#[test]
#[ignore = "requires native f64 Vulkan hardware"]
fn mirrored_edge_stops_sliding_under_its_actual_corner_loads() {
    let src: Value = serde_json::from_str(include_str!(
        "../../../../docs/measurements/local-manipulation/gpu-shelf-stick-states.json"
    ))
    .unwrap();
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut cases = serde_json::Map::new();
    for (name, case) in src.as_object().unwrap() {
        let ids = if name == "oblique_plus" {
            [1, 0]
        } else {
            [3, 2]
        };
        let mut trials = Vec::new();
        for row in case.as_array().unwrap() {
            for extra in [0., 0.020] {
                let mut input = matched_input(&row["state"]);
                input.lane.body.center_velocity[0] += extra;
                let previous = SpatialShelfLoads {
                    world_impulses: serde_json::from_value(row["preceding"]["vertices"].clone())
                        .unwrap(),
                    ..Default::default()
                };
                // With both corners already supporting the box, ordinary public stepping uses eight equal intervals.
                assert!(competing_corner(&input, &previous).is_none());
                let p = probe(&adapter, input, None);
                if extra == 0. {
                    for (i, j) in p.output.loads.world_impulses.iter().enumerate() {
                        assert!(
                            DVec3::from(*j).distance(point(&row["next"]["vertices"][i])) < 1e-12
                        );
                    }
                    assert!(
                        DVec3::from(p.output.lane.body.center_velocity)
                            .distance(point(&row["next_state"]["center_velocity"]))
                            < 1e-12
                    );
                }
                let mut steps = Vec::new();
                for (step, speeds) in p.steps.iter().zip(p.speeds) {
                    let u = ids.map(|i| speeds[i]);
                    let j = ids.map(|i| point(&step["vertex_impulses_ns"][i]));
                    let ratio = j.map(|v| v.x.hypot(v.y) / v.z);
                    assert!(j.iter().all(|v| v.z > 0.));
                    if extra > 0. {
                        assert!(
                            u.iter().all(|s| *s > 0.01)
                                && ratio.iter().all(|r| (*r - 0.45).abs() < 1e-6)
                        );
                    }
                    if extra == 0. {
                        if row["anchor_ms"] == 2240 && step["end_us"].as_f64().unwrap() <= 750. {
                            assert!(
                                u.iter().all(|v| *v > 0.00015)
                                    && ratio.iter().all(|r| (*r - 0.45).abs() < 1e-12)
                            );
                        } else {
                            assert!(u.iter().all(|v| *v < 5e-6) && ratio.iter().all(|r| *r < 0.40));
                        }
                    }
                    steps.push(json!({"end_us":step["end_us"],"slips_m_s":u,"point_impulses_ns":j.map(|v|v.to_array()),"shear_ratios":ratio}));
                }
                trials
                    .push(json!({"anchor_ms":row["anchor_ms"],"extra_x_m_s":extra,"steps":steps}));
            }
        }
        cases.insert(name.clone(), json!(trials));
    }
    println!("glancing_stick_device {}", Value::Object(cases));
}
