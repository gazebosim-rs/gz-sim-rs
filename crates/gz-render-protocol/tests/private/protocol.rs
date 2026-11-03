// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

struct ScriptedReader<'a> {
    bytes: &'a [u8],
    error_once: Option<io::ErrorKind>,
}
impl Read for ScriptedReader<'_> {
    fn read(&mut self, output: &mut [u8]) -> io::Result<usize> {
        if let Some(error) = self.error_once.take() {
            return Err(error.into());
        }
        self.bytes.read(output)
    }
}

#[test]
fn typed_roundtrip_preserves_large_f64_and_frame() {
    let request = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 5,
            seconds: u64::MAX,
            nanoseconds: 999_999_999,
        },
        origin: [1e12, -1e12, 0.25],
        visuals: vec![Visual {
            id: 1,
            translation: [1e12 + 1.25, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [1.0; 3],
            color: [0.5; 3],
        }],
    };
    let mut wire = Vec::new();
    write_request(&mut wire, &request).unwrap();
    assert_eq!(read_request(&mut wire.as_slice()).unwrap(), Some(request));
    let frame = Reply::Frame {
        request_id: 9,
        stamp: Stamp {
            epoch: 5,
            seconds: 3,
            nanoseconds: 4,
        },
        rgba: vec![255; PIXELS],
    };
    wire.clear();
    write_reply(&mut wire, &frame).unwrap();
    assert_eq!(read_reply(&mut wire.as_slice()).unwrap(), Some(frame));
}
#[test]
fn rejects_truncated_oversize_and_invalid_messages() {
    assert_eq!(read_request(&mut &[][..]).unwrap(), None);
    assert_eq!(
        read_request(&mut &[1, 2][..]).unwrap_err().kind(),
        io::ErrorKind::UnexpectedEof
    );
    assert_eq!(
        read_request(&mut (MAX_MESSAGE as u32 + 1).to_le_bytes().as_slice())
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidData
    );
    assert_eq!(
        read_request(&mut &[3, 0, 0, 0, b'{'][..])
            .unwrap_err()
            .kind(),
        io::ErrorKind::UnexpectedEof
    );
    assert!(
        write_reply(
            &mut Vec::new(),
            &Reply::Frame {
                request_id: 9,
                stamp: Stamp {
                    epoch: 0,
                    seconds: 0,
                    nanoseconds: 0
                },
                rgba: vec![]
            }
        )
        .is_err()
    );
    assert!(
        write_reply(
            &mut Vec::new(),
            &Reply::Error {
                kind: FailureKind::Protocol,
                request_id: None,
                message: "x".repeat(1025)
            }
        )
        .is_err()
    );
    let bad = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 0,
            seconds: 0,
            nanoseconds: 1_000_000_000,
        },
        origin: [0.0; 3],
        visuals: vec![],
    };
    assert!(write_request(&mut Vec::new(), &bad).is_err());
}

#[test]
fn all_control_messages_and_typed_failures_roundtrip() {
    for request in [
        Request::Capture { request_id: 42 },
        Request::Cancel { request_id: 42 },
        Request::Shutdown,
    ] {
        let mut wire = Vec::new();
        write_request(&mut wire, &request).unwrap();
        assert_eq!(read_request(&mut wire.as_slice()).unwrap(), Some(request));
    }
    for reply in [
        Reply::Ready {
            version: PROTOCOL_VERSION,
        },
        Reply::CancelAccepted { request_id: 42 },
        Reply::Busy { request_id: 42 },
        Reply::SnapshotBusy { request_id: 42 },
        Reply::SnapshotAccepted { request_id: 42 },
        Reply::Cancelled { request_id: 42 },
        Reply::Error {
            kind: FailureKind::Device,
            request_id: Some(42),
            message: "device lost".into(),
        },
    ] {
        let mut wire = Vec::new();
        write_reply(&mut wire, &reply).unwrap();
        assert_eq!(read_reply(&mut wire.as_slice()).unwrap(), Some(reply));
    }
}

#[test]
fn rejects_invalid_values_on_both_wire_directions() {
    let invalid_request = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [f64::NAN, 0.0, 0.0],
        visuals: vec![],
    };
    assert!(write_request(&mut Vec::new(), &invalid_request).is_err());
    let invalid_visual = Visual {
        id: 0,
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
        size: [1.0; 3],
        color: [0.0; 3],
    };
    let invalid_request = Request::Snapshot {
        request_id: 1,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        origin: [0.0; 3],
        visuals: vec![invalid_visual],
    };
    let mut wire = Vec::new();
    encode(&mut wire, &invalid_request).unwrap();
    assert_eq!(
        read_request(&mut wire.as_slice()).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    let invalid_reply = Reply::Frame {
        request_id: 1,
        stamp: Stamp {
            epoch: 1,
            seconds: 0,
            nanoseconds: 0,
        },
        rgba: vec![],
    };
    wire.clear();
    encode(&mut wire, &invalid_reply).unwrap();
    assert_eq!(
        read_reply(&mut wire.as_slice()).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    assert_eq!(
        read_reply(&mut &[1, 0, 0, 0, b'!'][..]).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
}

#[test]
fn interrupted_input_retries_and_write_failures_propagate() {
    struct Broken;
    impl Write for Broken {
        fn write(&mut self, _: &[u8]) -> io::Result<usize> {
            Err(io::ErrorKind::BrokenPipe.into())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let mut wire = Vec::new();
    write_request(&mut wire, &Request::Shutdown).unwrap();
    let mut input = ScriptedReader {
        bytes: &wire,
        error_once: Some(io::ErrorKind::Interrupted),
    };
    assert_eq!(read_request(&mut input).unwrap(), Some(Request::Shutdown));
    wire.clear();
    write_reply(
        &mut wire,
        &Reply::Ready {
            version: PROTOCOL_VERSION,
        },
    )
    .unwrap();
    let mut input = ScriptedReader {
        bytes: &wire,
        error_once: Some(io::ErrorKind::Interrupted),
    };
    assert_eq!(
        read_reply(&mut input).unwrap(),
        Some(Reply::Ready {
            version: PROTOCOL_VERSION
        })
    );
    for reply_direction in [false, true] {
        let mut input = ScriptedReader {
            bytes: &[1, 0, 0, 0, b'!'],
            error_once: Some(io::ErrorKind::Interrupted),
        };
        let error = if reply_direction {
            read_reply(&mut input).unwrap_err()
        } else {
            read_request(&mut input).unwrap_err()
        };
        assert_eq!(error.kind(), io::ErrorKind::InvalidData);
    }
    assert_eq!(
        read_request(&mut &[1, 0, 0, 0, b'!'][..])
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidData
    );
    assert_eq!(
        write_request(&mut Broken, &Request::Shutdown)
            .unwrap_err()
            .kind(),
        io::ErrorKind::BrokenPipe
    );
}

#[test]
fn oversized_serialization_and_input_errors_are_explicit() {
    let mut output = Vec::new();
    assert_eq!(
        encode(&mut output, &"x".repeat(MAX_MESSAGE))
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidData
    );
    assert!(output.is_empty());
    assert_eq!(
        read_request(&mut ScriptedReader {
            bytes: &[],
            error_once: Some(io::ErrorKind::PermissionDenied)
        })
        .unwrap_err()
        .kind(),
        io::ErrorKind::PermissionDenied
    );
    assert_eq!(
        read_reply(&mut ScriptedReader {
            bytes: &[],
            error_once: Some(io::ErrorKind::PermissionDenied)
        })
        .unwrap_err()
        .kind(),
        io::ErrorKind::PermissionDenied
    );
    assert_eq!(
        read_reply(&mut (MAX_MESSAGE as u32 + 1).to_le_bytes().as_slice())
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidData
    );
    assert_eq!(
        read_reply(&mut &[1, 2][..]).unwrap_err().kind(),
        io::ErrorKind::UnexpectedEof
    );
}

#[test]
fn reply_stream_errors_and_partial_body_writes_are_reported() {
    struct FailsAfterHeader {
        bytes: Vec<u8>,
    }
    impl Write for FailsAfterHeader {
        fn write(&mut self, input: &[u8]) -> io::Result<usize> {
            if self.bytes.len() >= 4 {
                return Err(io::ErrorKind::BrokenPipe.into());
            }
            self.bytes.extend_from_slice(input);
            Ok(input.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let mut request_output = FailsAfterHeader { bytes: vec![] };
    assert_eq!(
        write_request(&mut request_output, &Request::Shutdown)
            .unwrap_err()
            .kind(),
        io::ErrorKind::BrokenPipe
    );
    assert_eq!(request_output.bytes.len(), 4);
    let mut reply_output = FailsAfterHeader { bytes: vec![] };
    assert_eq!(
        write_reply(
            &mut reply_output,
            &Reply::Ready {
                version: PROTOCOL_VERSION
            }
        )
        .unwrap_err()
        .kind(),
        io::ErrorKind::BrokenPipe
    );
    assert_eq!(reply_output.bytes.len(), 4);
    assert_eq!(read_reply(&mut &[][..]).unwrap(), None);
    assert_eq!(
        read_reply(&mut &[2, 0, 0, 0, b'!'][..]).unwrap_err().kind(),
        io::ErrorKind::UnexpectedEof
    );
}

#[test]
fn serialization_errors_and_invalid_identifiers_are_rejected() {
    struct FailingSerialize;
    impl Serialize for FailingSerialize {
        fn serialize<S: serde::Serializer>(&self, _: S) -> Result<S::Ok, S::Error> {
            Err(serde::ser::Error::custom("serialization failed"))
        }
    }
    let mut output = Vec::new();
    assert_eq!(
        encode(&mut output, &FailingSerialize).unwrap_err().kind(),
        io::ErrorKind::Other
    );
    assert!(output.is_empty());
    for request in [
        Request::Capture { request_id: 0 },
        Request::Cancel { request_id: 0 },
    ] {
        assert_eq!(
            write_request(&mut output, &request).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
    for reply in [
        Reply::CancelAccepted { request_id: 0 },
        Reply::SnapshotAccepted { request_id: 0 },
        Reply::Busy { request_id: 0 },
        Reply::SnapshotBusy { request_id: 0 },
        Reply::Cancelled { request_id: 0 },
    ] {
        assert_eq!(
            write_reply(&mut output, &reply).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}

#[test]
fn snapshot_rejects_excess_visuals_and_nonfinite_components() {
    fn visual(id: u64) -> Visual {
        Visual {
            id,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [1.0; 3],
            color: [1.0; 3],
        }
    }
    fn snapshot(request_id: u64, visuals: Vec<Visual>) -> Request {
        Request::Snapshot {
            request_id,
            stamp: Stamp {
                epoch: 1,
                seconds: 0,
                nanoseconds: 0,
            },
            origin: [0.0; 3],
            visuals,
        }
    }
    assert!(!snapshot(0, vec![]).wire_valid());
    assert!(!snapshot(1, (1..=129).map(visual).collect()).wire_valid());
    let mut invalid_position = visual(1);
    invalid_position.translation[1] = f64::INFINITY;
    assert!(!snapshot(1, vec![invalid_position]).wire_valid());
    let mut invalid_color = visual(1);
    invalid_color.color[0] = f32::NAN;
    assert!(!snapshot(1, vec![invalid_color]).wire_valid());
    assert!(snapshot(1, (1..=128).map(visual).collect()).wire_valid());
}

#[test]
fn all_worker_failure_kinds_roundtrip() {
    for kind in [
        FailureKind::Startup,
        FailureKind::InvalidSnapshot,
        FailureKind::Capture,
        FailureKind::Readback,
        FailureKind::Device,
        FailureKind::Shader,
        FailureKind::Protocol,
        FailureKind::UnknownCapture,
        FailureKind::StaleRequest,
    ] {
        let reply = Reply::Error {
            kind,
            request_id: Some(7),
            message: "failure".into(),
        };
        let mut wire = Vec::new();
        write_reply(&mut wire, &reply).unwrap();
        assert_eq!(read_reply(&mut wire.as_slice()).unwrap(), Some(reply));
    }
}

#[test]
fn frame_rejects_zero_id_and_invalid_time_even_with_full_image() {
    for (request_id, nanoseconds) in [(0, 0), (1, 1_000_000_000)] {
        let frame = Reply::Frame {
            request_id,
            stamp: Stamp {
                epoch: 1,
                seconds: 0,
                nanoseconds,
            },
            rgba: vec![0; PIXELS],
        };
        assert_eq!(
            write_reply(&mut Vec::new(), &frame).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}

#[test]
fn calibrated_snapshot_roundtrips_and_rejects_invalid_wire_fields() {
    let mut request = Request::CalibratedSnapshot {
        request_id: 3,
        stamp: Stamp {
            epoch: 2,
            seconds: 7,
            nanoseconds: 9,
        },
        origin: [1e12, -1e12, 0.5],
        visuals: vec![],
        camera: Box::new(Camera {
            width: 64,
            height: 64,
            fx: 32.0,
            fy: 31.0,
            cx: 16.0,
            cy: 40.0,
            skew: 0.0,
            tx: 0.0,
            ty: 0.0,
            near: 0.1,
            far: 10.0,
            translation: [1e12 + 0.25, -1e12, 0.5],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }),
    };
    let mut wire = Vec::new();
    write_request(&mut wire, &request).unwrap();
    assert_eq!(
        read_request(&mut wire.as_slice()).unwrap().as_ref(),
        Some(&request)
    );
    if let Request::CalibratedSnapshot { ref mut camera, .. } = request {
        camera.fx = f64::NAN;
    }
    assert!(!request.wire_valid());
    assert!(write_request(&mut Vec::new(), &request).is_err());
}

fn depth_snapshot() -> Request {
    Request::CalibratedDepthSnapshot {
        request_id: 3,
        stamp: Stamp {
            epoch: 2,
            seconds: 7,
            nanoseconds: 9,
        },
        origin: [1e12, -1e12, 0.5],
        visuals: vec![Visual {
            id: 41,
            translation: [1e12 + 2.0, -1e12, 0.5],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [1.0; 3],
            color: [0.25, 0.5, 0.75],
        }],
        camera: Box::new(Camera {
            width: 64,
            height: 64,
            fx: 32.0,
            fy: 31.0,
            cx: 16.0,
            cy: 40.0,
            skew: 0.0,
            tx: 0.0,
            ty: 0.0,
            near: 0.1,
            far: 10.0,
            translation: [1e12 + 0.25, -1e12, 0.5],
            rotation: [0.0, 0.0, 0.0, 1.0],
        }),
    }
}

fn depth_reply(metres_le: Vec<u8>) -> Reply {
    Reply::DepthFrame {
        request_id: 9,
        stamp: Stamp {
            epoch: u64::MAX,
            seconds: u64::MAX,
            nanoseconds: 999_999_999,
        },
        metres_le,
    }
}

fn reject_depth_reply(reply: &Reply) {
    let mut output = Vec::new();
    assert_eq!(
        write_reply(&mut output, reply).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
    assert!(output.is_empty());
    encode(&mut output, reply).unwrap();
    assert_eq!(
        read_reply(&mut output.as_slice()).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
}

#[test]
fn calibrated_depth_snapshot_preserves_all_wire_fields_and_version_three() {
    assert_eq!(PROTOCOL_VERSION, 3);
    let request = depth_snapshot();
    let mut wire = Vec::new();
    write_request(&mut wire, &request).unwrap();
    assert_eq!(read_request(&mut wire.as_slice()).unwrap(), Some(request));
    let payload: serde_json::Value = serde_json::from_slice(&wire[4..]).unwrap();
    assert!(payload.get("CalibratedDepthSnapshot").is_some());
    assert!(payload.get("CalibratedSnapshot").is_none());
}

#[test]
fn calibrated_depth_rejects_invalid_identity_time_scene_and_calibration_values() {
    for field in 0..9 {
        let mut request = depth_snapshot();
        if let Request::CalibratedDepthSnapshot {
            request_id,
            stamp,
            origin,
            visuals,
            camera,
        } = &mut request
        {
            match field {
                0 => *request_id = 0,
                1 => stamp.nanoseconds = 1_000_000_000,
                2 => origin[1] = f64::INFINITY,
                3 => visuals[0].id = 0,
                4 => visuals[0].size[2] = f64::NEG_INFINITY,
                5 => visuals[0].color[1] = f32::NAN,
                6 => camera.fx = f64::NAN,
                7 => camera.translation[0] = f64::INFINITY,
                8 => camera.rotation[3] = f64::NEG_INFINITY,
                _ => unreachable!(),
            }
        } else {
            unreachable!();
        }
        assert!(!request.wire_valid(), "field {field}");
        let mut wire = Vec::new();
        assert_eq!(
            write_request(&mut wire, &request).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
        assert!(wire.is_empty());
        encode(&mut wire, &request).unwrap();
        assert_eq!(
            read_request(&mut wire.as_slice()).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}

#[test]
fn calibrated_depth_reuses_visual_limit_and_leaves_semantic_camera_checks_to_renderer() {
    let mut request = depth_snapshot();
    if let Request::CalibratedDepthSnapshot {
        visuals, camera, ..
    } = &mut request
    {
        camera.width = 0;
        camera.fx = -1.0;
        camera.near = 2.0;
        camera.far = 1.0;
        while visuals.len() < MAX_VISUALS {
            visuals.push(Visual {
                id: visuals.len() as u64 + 1000,
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0],
                size: [1.0; 3],
                color: [1.0; 3],
            });
        }
    }
    assert!(request.wire_valid());
    let mut wire = Vec::new();
    write_request(&mut wire, &request).unwrap();
    assert_eq!(
        read_request(&mut wire.as_slice()).unwrap().as_ref(),
        Some(&request)
    );
    if let Request::CalibratedDepthSnapshot { visuals, .. } = &mut request {
        visuals.push(Visual {
            id: 999,
            translation: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            size: [1.0; 3],
            color: [1.0; 3],
        });
    }
    assert!(!request.wire_valid());
    assert!(write_request(&mut Vec::new(), &request).is_err());
    wire.clear();
    encode(&mut wire, &request).unwrap();
    assert!(read_request(&mut wire.as_slice()).is_err());
}

#[test]
fn depth_frame_roundtrip_preserves_positive_infinity_and_raw_float_words() {
    let words = [
        [0, 0, 128, 63],  // 1 metre
        [0, 0, 160, 63],  // 1.25 metres
        [0, 0, 128, 127], // +INF background
        f32::MIN_POSITIVE.to_le_bytes(),
        f32::from_bits(1).to_le_bytes(),
        f32::MAX.to_le_bytes(),
    ];
    let mut bytes = Vec::with_capacity(PIXELS);
    for pixel in 0..64 * 64 {
        bytes.extend_from_slice(&words[pixel % words.len()]);
    }
    let reply = depth_reply(bytes);
    let mut wire = Vec::new();
    write_reply(&mut wire, &reply).unwrap();
    assert_eq!(
        u32::from_le_bytes(wire[..4].try_into().unwrap()) as usize,
        wire.len() - 4
    );
    assert!(wire.len() - 4 <= MAX_MESSAGE);
    let payload: serde_json::Value = serde_json::from_slice(&wire[4..]).unwrap();
    let values = payload["DepthFrame"]["metres_le"].as_array().unwrap();
    assert_eq!(values.len(), PIXELS);
    assert!(values.iter().all(|value| value.as_u64().is_some()));
    assert_eq!(
        &values[8..12],
        &[0, 0, 128, 127].map(serde_json::Value::from)
    );
    assert!(!String::from_utf8_lossy(&wire[4..]).contains("null"));
    assert_eq!(read_reply(&mut wire.as_slice()).unwrap(), Some(reply));
}

#[test]
fn depth_word_validation_uses_little_endian_not_host_or_big_endian_sign() {
    let mut bytes = 1.0_f32.to_le_bytes().repeat(64 * 64);
    // Little-endian +1.0000153; the same bytes interpreted big-endian are negative.
    bytes[..4].copy_from_slice(&[0x80, 0x00, 0x80, 0x3f]);
    let reply = depth_reply(bytes.clone());
    let mut wire = Vec::new();
    write_reply(&mut wire, &reply).unwrap();
    assert_eq!(read_reply(&mut wire.as_slice()).unwrap(), Some(reply));
    bytes[..4].copy_from_slice(&[0x3f, 0x80, 0x00, 0x80]);
    reject_depth_reply(&depth_reply(bytes));
}

#[test]
fn depth_frame_rejects_nan_signed_zero_negative_and_negative_infinity_at_every_position() {
    for bits in [
        0x0000_0000_u32,
        0x8000_0000,
        0xbf80_0000,
        0x8000_0001,
        0xff80_0000,
        0x7fc0_0000,
        0x7f80_0001,
        0xffc0_0000,
        0xff80_0001,
    ] {
        for pixel in [0, 2048, 4095] {
            let mut bytes = 1.0_f32.to_le_bytes().repeat(64 * 64);
            bytes[pixel * 4..pixel * 4 + 4].copy_from_slice(&bits.to_le_bytes());
            reject_depth_reply(&depth_reply(bytes));
        }
    }
}

#[test]
fn depth_frame_rejects_nonfixed_lengths_zero_id_and_invalid_stamp_both_directions() {
    for length in [0, PIXELS - 4, PIXELS - 1, PIXELS + 1, PIXELS + 4] {
        let mut bytes = 1.0_f32.to_le_bytes().repeat(64 * 64 + 1);
        bytes.truncate(length);
        reject_depth_reply(&depth_reply(bytes));
    }
    for (request_id, nanoseconds) in [(0, 0), (9, 1_000_000_000)] {
        reject_depth_reply(&Reply::DepthFrame {
            request_id,
            stamp: Stamp {
                epoch: 1,
                seconds: 0,
                nanoseconds,
            },
            metres_le: 1.0_f32.to_le_bytes().repeat(64 * 64),
        });
    }
}

#[test]
fn depth_and_rgb_keep_distinct_reply_tags_in_the_same_bounded_stream() {
    let rgb = Reply::Frame {
        request_id: 4,
        stamp: Stamp {
            epoch: 2,
            seconds: 7,
            nanoseconds: 8,
        },
        rgba: vec![0; PIXELS],
    };
    let depth = depth_reply(f32::INFINITY.to_le_bytes().repeat(64 * 64));
    let mut stream = Vec::new();
    write_reply(&mut stream, &rgb).unwrap();
    write_reply(&mut stream, &depth).unwrap();
    let mut input = stream.as_slice();
    assert_eq!(read_reply(&mut input).unwrap(), Some(rgb));
    assert_eq!(read_reply(&mut input).unwrap(), Some(depth));
    assert_eq!(read_reply(&mut input).unwrap(), None);
}
