// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_msgs::prost::Message;

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
