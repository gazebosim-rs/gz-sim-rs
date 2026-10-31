// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{
    msgs::{CameraInfo, Clock, Double, Float, Imu, Quaternion, Vector3d},
    prost::Message,
};

fn check_wire(message: &impl Message, wire: &[u8]) {
    assert_eq!(message.encoded_len(), wire.len());
    assert_eq!(message.encode_to_vec(), wire);
    let mut exact = vec![0; wire.len()];
    let mut remaining = exact.as_mut_slice();
    message.encode(&mut remaining).unwrap();
    assert!(remaining.is_empty());
    assert_eq!(exact, wire);
}

#[test]
fn native_signed_zero_and_positive_zero_default_omission() {
    for (value, suffix) in [(0.0_f64, "positive"), (-0.0, "negative")] {
        let double = if suffix == "positive" {
            include_bytes!("fixtures/double-positive-zero.bin").as_slice()
        } else {
            include_bytes!("fixtures/double-negative-zero.bin").as_slice()
        };
        let vector = if suffix == "positive" {
            include_bytes!("fixtures/vector3d-positive-zero.bin").as_slice()
        } else {
            include_bytes!("fixtures/vector3d-negative-zero.bin").as_slice()
        };
        let quaternion = if suffix == "positive" {
            include_bytes!("fixtures/quaternion-positive-zero.bin").as_slice()
        } else {
            include_bytes!("fixtures/quaternion-negative-zero.bin").as_slice()
        };
        check_wire(
            &Double {
                data: value,
                ..Default::default()
            },
            double,
        );
        check_wire(
            &Vector3d {
                x: value,
                y: value,
                z: value,
                ..Default::default()
            },
            vector,
        );
        check_wire(
            &Quaternion {
                x: value,
                y: value,
                z: value,
                w: value,
                ..Default::default()
            },
            quaternion,
        );
        assert_eq!(
            Double::decode(double).unwrap().data.to_bits(),
            value.to_bits()
        );
        let decoded = Vector3d::decode(vector).unwrap();
        assert_eq!(
            [decoded.x, decoded.y, decoded.z].map(f64::to_bits),
            [value.to_bits(); 3]
        );
        let decoded = Quaternion::decode(quaternion).unwrap();
        assert_eq!(
            [decoded.x, decoded.y, decoded.z, decoded.w].map(f64::to_bits),
            [value.to_bits(); 4]
        );
    }
    for (value, wire) in [
        (
            0.0_f32,
            include_bytes!("fixtures/float-positive-zero.bin").as_slice(),
        ),
        (
            -0.0,
            include_bytes!("fixtures/float-negative-zero.bin").as_slice(),
        ),
    ] {
        check_wire(
            &Float {
                data: value,
                ..Default::default()
            },
            wire,
        );
        assert_eq!(Float::decode(wire).unwrap().data.to_bits(), value.to_bits());
    }
}

#[test]
fn complete_native_clock_imu_and_camera_packets() {
    let clock = include_bytes!("fixtures/clock.bin");
    let message = Clock::decode(clock.as_slice()).unwrap();
    check_wire(&message, clock);
    let sim = message.sim.unwrap();
    assert_eq!((sim.sec, sim.nsec), (3, 4));
    let header = message.header.unwrap();
    assert_eq!(header.data.len(), 3);
    assert_eq!(
        header
            .data
            .iter()
            .map(|entry| entry.key.as_str())
            .collect::<Vec<_>>(),
        ["frame_id", "seq", "frame_id"]
    );
    let imu = include_bytes!("fixtures/imu.bin");
    check_wire(&Imu::decode(imu.as_slice()).unwrap(), imu);
    let camera = include_bytes!("fixtures/camera-info.bin");
    assert_eq!(camera.len(), 325);
    check_wire(&CameraInfo::decode(camera.as_slice()).unwrap(), camera);
}

#[test]
fn generated_names_match_native_zenoh_type_attachments() {
    use gz_sim_proto::{
        msgs::{Boolean, Empty, Image, WorldControl},
        prost::Name,
    };
    assert_eq!(Clock::full_name(), "gz.msgs.Clock");
    assert_eq!(Imu::full_name(), "gz.msgs.IMU");
    assert_eq!(Image::full_name(), "gz.msgs.Image");
    assert_eq!(CameraInfo::full_name(), "gz.msgs.CameraInfo");
    assert_eq!(Double::full_name(), "gz.msgs.Double");
    assert_eq!(WorldControl::full_name(), "gz.msgs.WorldControl");
    assert_eq!(Boolean::full_name(), "gz.msgs.Boolean");
    assert_eq!(Empty::full_name(), "gz.msgs.Empty");
}

// Guard the dependency repair's preexisting numeric default behavior; this is
// not an additional schema or a generic Messages runtime requirement.
#[derive(Clone, PartialEq, Message)]
struct NanDefaultGuard {
    #[prost(float, tag = "1", default = "nan")]
    a: f32,
    #[prost(double, tag = "2", default = "nan")]
    b: f64,
}

#[test]
fn nan_defaults_keep_existing_numeric_inequality_semantics() {
    let message = NanDefaultGuard::default();
    assert!(message.a.is_nan() && message.b.is_nan());
    let mut wire = vec![0x0d];
    wire.extend_from_slice(&message.a.to_bits().to_le_bytes());
    wire.push(0x11);
    wire.extend_from_slice(&message.b.to_bits().to_le_bytes());
    check_wire(&message, &wire);
    let decoded = NanDefaultGuard::decode(wire.as_slice()).unwrap();
    assert_eq!(decoded.a.to_bits(), message.a.to_bits());
    assert_eq!(decoded.b.to_bits(), message.b.to_bits());
}
