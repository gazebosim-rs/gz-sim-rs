// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::prost::Message;

#[test]
fn rgb_frame_strips_alpha_preserves_rows_header_and_roundtrips() {
    let rgba: Vec<u8> = (0..64 * 64)
        .flat_map(|index| [index as u8, (index >> 6) as u8, 255 - index as u8, 0x5a])
        .collect();
    let message = image_packet(&rgba, Duration::new(123, 456), "robot/camera", 0).unwrap();
    assert_eq!((message.width, message.height, message.step), (64, 64, 192));
    assert_eq!(message.pixel_format_type, PixelFormatType::RgbInt8 as i32);
    assert_eq!(message.data.len(), RGB_BYTES);
    for (input, output) in rgba.chunks_exact(4).zip(message.data.chunks_exact(3)) {
        assert_eq!(output, &input[..3]);
    }
    assert_eq!(
        message.header.as_ref().unwrap().stamp,
        Some(Time {
            sec: 123,
            nsec: 456
        })
    );
    assert_eq!(
        message.header.as_ref().unwrap().data,
        vec![
            Map {
                key: "frame_id".into(),
                value: vec!["robot/camera".into()]
            },
            Map {
                key: "seq".into(),
                value: vec!["0".into()]
            },
        ]
    );
    assert_eq!(
        Image::decode(message.encode_to_vec().as_slice()).unwrap(),
        message
    );
    let next = image_packet(&rgba, Duration::ZERO, "", u64::MAX).unwrap();
    assert_eq!(
        next.header.unwrap().data[1].value,
        vec![u64::MAX.to_string()]
    );
}

#[test]
fn camera_info_has_exact_native_matrix_distortion_and_header_shape() {
    let values = IntrinsicValues {
        fx: 30.0,
        fy: 31.0,
        cx: 20.0,
        cy: 21.0,
    };
    let info = camera_info_packet(Duration::new(4, 5), "frame", values).unwrap();
    assert_eq!((info.width, info.height), (64, 64));
    let header = info.header.as_ref().unwrap();
    assert_eq!(header.stamp, Some(Time { sec: 4, nsec: 5 }));
    assert_eq!(
        header.data,
        vec![Map {
            key: "frame_id".into(),
            value: vec!["frame".into()]
        }]
    );
    let distortion = info.distortion.as_ref().unwrap();
    assert_eq!(distortion.model, DistortionModelType::PlumbBob as i32);
    assert_eq!(distortion.k, vec![0.0; 5]);
    assert_eq!(
        info.intrinsics.as_ref().unwrap().k,
        vec![30., 0., 20., 0., 31., 21., 0., 0., 1.]
    );
    assert_eq!(
        info.projection.as_ref().unwrap().p,
        vec![30., 0., 20., 0., 0., 31., 21., 0., 0., 0., 1., 0.]
    );
    assert_eq!(
        info.rectification_matrix,
        vec![1., 0., 0., 0., 1., 0., 0., 0., 1.]
    );
    assert_eq!(
        CameraInfo::decode(info.encode_to_vec().as_slice()).unwrap(),
        info
    );
}

#[test]
fn camera_info_matches_pinned_native_sensor_wire() {
    let info = camera_info_packet(
        Duration::new(4, 5),
        "frame",
        IntrinsicValues {
            fx: 30.0,
            fy: 31.0,
            cx: 20.0,
            cy: 21.0,
        },
    )
    .unwrap();
    let native = include_bytes!("../fixtures/native_camera_info.bin");
    assert_eq!(info.encode_to_vec(), native);
    assert_eq!(CameraInfo::decode(native.as_slice()).unwrap(), info);
}

#[test]
fn malformed_lengths_timestamps_and_intrinsics_fail() {
    let rgba = vec![0; RGBA_BYTES];
    for length in [0, RGBA_BYTES - 1] {
        assert_eq!(
            image_packet(&rgba[..length], Duration::ZERO, "", 0),
            Err(CameraMessageError::InvalidFrame)
        );
    }
    let mut too_long = rgba.clone();
    too_long.push(0);
    assert_eq!(
        image_packet(&too_long, Duration::ZERO, "", 0),
        Err(CameraMessageError::InvalidFrame)
    );
    let overflow = Duration::from_secs(i64::MAX as u64 + 1);
    assert_eq!(
        image_packet(&rgba, overflow, "", 0),
        Err(CameraMessageError::TimeRange)
    );
    let good = IntrinsicValues {
        fx: 1.0,
        fy: 2.0,
        cx: 3.0,
        cy: 4.0,
    };
    assert_eq!(
        camera_info_packet(overflow, "", good),
        Err(CameraMessageError::TimeRange)
    );
    for values in [
        IntrinsicValues { fx: 0.0, ..good },
        IntrinsicValues { fy: -1.0, ..good },
        IntrinsicValues {
            fx: f64::NAN,
            ..good
        },
        IntrinsicValues {
            fy: f64::INFINITY,
            ..good
        },
        IntrinsicValues {
            cx: f64::NAN,
            ..good
        },
        IntrinsicValues {
            cy: f64::NEG_INFINITY,
            ..good
        },
    ] {
        assert_eq!(
            camera_info_packet(Duration::ZERO, "", values),
            Err(CameraMessageError::InvalidIntrinsics)
        );
    }
}

#[test]
fn metric_depth_preserves_little_endian_metres_infinity_and_canonical_header() {
    let values: Vec<f32> = (0..64 * 64)
        .map(|index| {
            if index % 64 == 0 {
                f32::INFINITY
            } else {
                1.25 + index as f32 / 100.0
            }
        })
        .collect();
    let bytes: Vec<u8> = values
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();
    let message = depth_image_packet(&bytes, Duration::new(7, 123), "rig/depth", 9).unwrap();
    assert_eq!((message.width, message.height, message.step), (64, 64, 256));
    assert_eq!(message.pixel_format_type, PixelFormatType::RFloat32 as i32);
    assert_eq!(message.pixel_format_type, 13);
    assert_eq!(message.data, bytes);
    assert_eq!(message.data.len(), 16_384);
    let header = message.header.as_ref().unwrap();
    assert_eq!(header.stamp, Some(Time { sec: 7, nsec: 123 }));
    assert_eq!(header.data[0].key, "frame_id");
    assert_eq!(header.data[0].value, ["rig/depth"]);
    assert_eq!(header.data[1].key, "seq");
    assert_eq!(header.data[1].value, ["9"]);
    assert_eq!(
        Image::decode(message.encode_to_vec().as_slice()).unwrap(),
        message
    );
}

#[test]
fn metric_depth_refuses_invalid_size_values_and_unrepresentable_time() {
    let valid: Vec<u8> = (0..64 * 64).flat_map(|_| 2.0_f32.to_le_bytes()).collect();
    for size in [0, 16_383, 16_385] {
        assert_eq!(
            depth_image_packet(&vec![0; size], Duration::ZERO, "depth", 0),
            Err(CameraMessageError::InvalidDepthFrame)
        );
    }
    for value in [0.0, -0.0, -1.0, f32::NEG_INFINITY, f32::NAN] {
        let mut invalid = valid.clone();
        invalid[128..132].copy_from_slice(&value.to_le_bytes());
        assert_eq!(
            depth_image_packet(&invalid, Duration::ZERO, "depth", 0),
            Err(CameraMessageError::InvalidDepthFrame)
        );
    }
    assert_eq!(
        depth_image_packet(&valid, Duration::from_secs(u64::MAX), "depth", 0),
        Err(CameraMessageError::TimeRange)
    );
}

#[test]
fn distinct_projection_preserves_complete_canonical_info_and_independent_matrices() {
    let k = IntrinsicValues {
        fx: 30.0,
        fy: 31.0,
        cx: 20.0,
        cy: 21.0,
    };
    let p = IntrinsicValues {
        fx: 52.0,
        fy: 36.0,
        cx: 11.0,
        cy: 45.0,
    };
    let mut expected =
        CameraInfo::decode(include_bytes!("../fixtures/native_camera_info.bin").as_slice())
            .unwrap();
    expected.projection.as_mut().unwrap().p = vec![
        52.0, 0.0, 11.0, 0.0, 0.0, 36.0, 45.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let actual = camera_info_packet_with_projection(Duration::new(4, 5), "frame", k, p).unwrap();
    assert_eq!(actual, expected);
    let wire = actual.encode_to_vec();
    assert_eq!(wire, expected.encode_to_vec());
    assert_eq!(CameraInfo::decode(wire.as_slice()).unwrap(), actual);
    assert_eq!(
        actual.intrinsics.unwrap().k,
        [30.0, 0.0, 20.0, 0.0, 31.0, 21.0, 0.0, 0.0, 1.0]
    );
}

#[test]
fn same_projection_constructor_preserves_legacy_camera_info_bytes() {
    for k in [
        IntrinsicValues {
            fx: 30.0,
            fy: 31.0,
            cx: 20.0,
            cy: 21.0,
        },
        IntrinsicValues {
            fx: 40.0,
            fy: 52.0,
            cx: -4.0,
            cy: 96.0,
        },
        IntrinsicValues {
            fx: f64::MAX,
            fy: f64::MIN_POSITIVE,
            cx: 0.0,
            cy: 0.0,
        },
    ] {
        for time in [Duration::ZERO, Duration::new(4, 5)] {
            let mut expected =
                CameraInfo::decode(include_bytes!("../fixtures/native_camera_info.bin").as_slice())
                    .unwrap();
            let header = expected.header.as_mut().unwrap();
            header.stamp = Some(Time {
                sec: time.as_secs() as i64,
                nsec: time.subsec_nanos() as i32,
            });
            header.data[0].value = vec!["rig/camera".into()];
            expected.intrinsics.as_mut().unwrap().k =
                vec![k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0];
            expected.projection.as_mut().unwrap().p = vec![
                k.fx, 0.0, k.cx, 0.0, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0,
            ];
            let old = camera_info_packet(time, "rig/camera", k).unwrap();
            let new = camera_info_packet_with_projection(time, "rig/camera", k, k).unwrap();
            assert_eq!(old, expected);
            assert_eq!(new.encode_to_vec(), expected.encode_to_vec());
            assert_eq!(old, new);
            assert_eq!(old.encode_to_vec(), new.encode_to_vec());
        }
    }
}

#[test]
fn projection_packets_reject_invalid_fields_in_each_matrix_with_legacy_error_order() {
    let good = IntrinsicValues {
        fx: 40.0,
        fy: 52.0,
        cx: 20.0,
        cy: 44.0,
    };
    let overflow = Duration::from_secs(i64::MAX as u64 + 1);
    for invalid in [
        IntrinsicValues { fx: 0.0, ..good },
        IntrinsicValues { fy: 0.0, ..good },
        IntrinsicValues { fx: -1.0, ..good },
        IntrinsicValues { fy: -1.0, ..good },
        IntrinsicValues {
            fx: f64::NAN,
            ..good
        },
        IntrinsicValues {
            fy: f64::INFINITY,
            ..good
        },
        IntrinsicValues {
            cx: f64::NAN,
            ..good
        },
        IntrinsicValues {
            cy: f64::NEG_INFINITY,
            ..good
        },
    ] {
        assert_eq!(
            camera_info_packet_with_projection(Duration::ZERO, "", invalid, good),
            Err(CameraMessageError::InvalidIntrinsics)
        );
        assert_eq!(
            camera_info_packet_with_projection(Duration::ZERO, "", good, invalid),
            Err(CameraMessageError::InvalidProjection)
        );
        assert_eq!(
            camera_info_packet_with_projection(overflow, "", invalid, invalid),
            Err(CameraMessageError::InvalidIntrinsics)
        );
        assert_eq!(
            camera_info_packet_with_projection(overflow, "", good, invalid),
            Err(CameraMessageError::InvalidProjection)
        );
        assert_eq!(
            camera_info_packet(overflow, "", invalid),
            Err(CameraMessageError::InvalidIntrinsics)
        );
    }
    assert_eq!(
        camera_info_packet_with_projection(overflow, "", good, good),
        Err(CameraMessageError::TimeRange)
    );
}
