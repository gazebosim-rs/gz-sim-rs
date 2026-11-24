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
#[test]
#[ignore = "requires native f64 Vulkan hardware"]
fn matched_glancing_corner_resolves_each_device_substep_and_closes_the_wrench() {
    let src: Value = serde_json::from_str(MATCHED).unwrap();
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut all = serde_json::Map::new();
    for (name, data) in src.as_object().unwrap() {
        let s = &data["gpu"]["incoming"];
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
            input.split_time = split.1;
            let mut device =
                ResidentBatch::new_profile(&adapter, vec![input], |_| true, predict::<0>, None)
                    .unwrap();
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
            let mut schedule = vec![(pass(predict::<0>), H); SUBSTEPS];
            if corrected {
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
                for (i, j) in result.current.iter().copied().enumerate() {
                    let r = q
                        * (DVec3::from_array(std::array::from_fn(|axis| {
                            if i & (1 << axis) == 0 {
                                -input.half[axis]
                            } else {
                                input.half[axis]
                            }
                        })) - offset);
                    let j = DVec3::from(j);
                    assert!(j.z >= 0. && j.x.hypot(j.y) <= 0.45 * j.z + 1e-15);
                    impulse += j;
                    let a = r.cross(j);
                    torque += a;
                    torques[i] += a;
                }
                let end = result.lane.body;
                assert!(
                    (0.08
                        * (DVec3::from(end.center_velocity)
                            - DVec3::from(incoming.center_velocity))
                        - impulse)
                        .length()
                        < 1e-12
                );
                assert!(
                    (DVec3::from(end.momentum) - DVec3::from(incoming.momentum) - torque).length()
                        < 1e-13
                );
                per_step.push(json!({"end_us":elapsed*1e6,"incoming_q":q.to_array(),"incoming_v":incoming.center_velocity,"incoming_omega":omega.to_array(),"vertex_impulses_ns":result.current,"angular_impulse_nms":torque.to_array(),"outgoing_v":end.center_velocity}));
            }
            let output = device.committed[0];
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
            assert!((elapsed - 0.001).abs() < 1e-18);
            trials.insert(
            if corrected {"retimed"}else{"uniform"}.to_owned(),
            json!({"substeps":per_step,"corner_angular_impulses_nms":torques.map(|a|a.to_array())}),
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
