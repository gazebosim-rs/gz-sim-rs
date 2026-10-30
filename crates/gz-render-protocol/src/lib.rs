// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde::{Deserialize, Serialize};
use std::io::{self, Read, Write};

pub const PROTOCOL_VERSION: u16 = 2;
const MAX_MESSAGE: usize = 131_072;
const MAX_VISUALS: usize = 128;
const PIXELS: usize = 64 * 64 * 4;

#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub struct Visual {
    pub id: u64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub size: [f64; 3],
    pub color: [f32; 3],
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct Stamp {
    pub epoch: u64,
    pub seconds: u64,
    pub nanoseconds: u32,
}
/// Pinhole calibration and Gazebo-frame world pose for one snapshot.
///
/// Pixels start at the upper left. The pose uses Gazebo's +X forward,
/// +Y left, +Z up camera axes. This worker currently supports 64×64 output
/// and rejects nonzero skew or projection translation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Camera {
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub skew: f64,
    pub tx: f64,
    pub ty: f64,
    pub near: f64,
    pub far: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum Request {
    Snapshot {
        request_id: u64,
        stamp: Stamp,
        origin: [f64; 3],
        visuals: Vec<Visual>,
    },
    CalibratedSnapshot {
        request_id: u64,
        stamp: Stamp,
        origin: [f64; 3],
        visuals: Vec<Visual>,
        camera: Box<Camera>,
    },
    Capture {
        request_id: u64,
    },
    Cancel {
        request_id: u64,
    },
    Shutdown,
}
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum FailureKind {
    Startup,
    InvalidSnapshot,
    Capture,
    Readback,
    Device,
    Shader,
    Protocol,
    UnknownCapture,
    StaleRequest,
}
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum Reply {
    Ready {
        version: u16,
    },
    CancelAccepted {
        request_id: u64,
    },
    SnapshotAccepted {
        request_id: u64,
    },
    Busy {
        request_id: u64,
    },
    SnapshotBusy {
        request_id: u64,
    },
    Cancelled {
        request_id: u64,
    },
    Frame {
        request_id: u64,
        stamp: Stamp,
        rgba: Vec<u8>,
    },
    Error {
        kind: FailureKind,
        request_id: Option<u64>,
        message: String,
    },
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}
fn valid_stamp(stamp: &Stamp) -> bool {
    stamp.nanoseconds < 1_000_000_000
}
fn valid_snapshot(request_id: u64, stamp: &Stamp, origin: &[f64; 3], visuals: &[Visual]) -> bool {
    request_id != 0
        && valid_stamp(stamp)
        && origin.iter().all(|x| x.is_finite())
        && visuals.len() <= MAX_VISUALS
        && visuals.iter().all(|v| {
            v.id != 0
                && v.translation
                    .iter()
                    .chain(&v.rotation)
                    .chain(&v.size)
                    .all(|x| x.is_finite())
                && v.color.iter().all(|x| x.is_finite())
        })
}
impl Request {
    /// Checks wire bounds and finite values; the renderer validates scene semantics separately.
    pub fn wire_valid(&self) -> bool {
        match self {
            Self::Snapshot {
                request_id,
                stamp,
                origin,
                visuals,
            } => valid_snapshot(*request_id, stamp, origin, visuals),
            Self::CalibratedSnapshot {
                request_id,
                stamp,
                origin,
                visuals,
                camera,
            } => {
                valid_snapshot(*request_id, stamp, origin, visuals)
                    && [
                        camera.fx,
                        camera.fy,
                        camera.cx,
                        camera.cy,
                        camera.skew,
                        camera.tx,
                        camera.ty,
                        camera.near,
                        camera.far,
                    ]
                    .iter()
                    .chain(&camera.translation)
                    .chain(&camera.rotation)
                    .all(|x| x.is_finite())
            }
            Self::Capture { request_id } | Self::Cancel { request_id } => *request_id != 0,
            Self::Shutdown => true,
        }
    }
}
impl Reply {
    fn valid(&self) -> bool {
        match self {
            Self::Frame {
                request_id,
                stamp,
                rgba,
            } => *request_id != 0 && valid_stamp(stamp) && rgba.len() == PIXELS,
            Self::CancelAccepted { request_id }
            | Self::SnapshotAccepted { request_id }
            | Self::SnapshotBusy { request_id }
            | Self::Busy { request_id }
            | Self::Cancelled { request_id } => *request_id != 0,
            Self::Error { message, .. } => message.len() <= 1024,
            _ => true,
        }
    }
}
fn encode<T: Serialize>(output: &mut impl Write, message: &T) -> io::Result<()> {
    let bytes = serde_json::to_vec(message).map_err(io::Error::other)?;
    if bytes.len() > MAX_MESSAGE {
        return Err(invalid("message too large"));
    }
    output.write_all(&(bytes.len() as u32).to_le_bytes())?;
    output.write_all(&bytes)
}
fn decode<T: for<'a> Deserialize<'a>>(input: &mut impl Read) -> io::Result<Option<T>> {
    let mut header = [0; 4];
    loop {
        match input.read(&mut header[..1]) {
            Ok(0) => return Ok(None),
            Ok(_) => break,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
    input.read_exact(&mut header[1..])?;
    let size = u32::from_le_bytes(header) as usize;
    if size > MAX_MESSAGE {
        return Err(invalid("message too large"));
    }
    let mut bytes = vec![0; size];
    input.read_exact(&mut bytes)?;
    serde_json::from_slice(&bytes)
        .map(Some)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}
pub fn write_request(output: &mut impl Write, request: &Request) -> io::Result<()> {
    if !request.wire_valid() {
        return Err(invalid("invalid request"));
    }
    encode(output, request)
}
pub fn read_request(input: &mut impl Read) -> io::Result<Option<Request>> {
    let request: Option<Request> = decode(input)?;
    if request.as_ref().is_some_and(|r| !r.wire_valid()) {
        return Err(invalid("invalid request"));
    }
    Ok(request)
}
pub fn write_reply(output: &mut impl Write, reply: &Reply) -> io::Result<()> {
    if !reply.valid() {
        return Err(invalid("invalid reply"));
    }
    encode(output, reply)
}
pub fn read_reply(input: &mut impl Read) -> io::Result<Option<Reply>> {
    let reply: Option<Reply> = decode(input)?;
    if reply.as_ref().is_some_and(|r| !r.valid()) {
        return Err(invalid("invalid reply"));
    }
    Ok(reply)
}

#[cfg(test)]
#[path = "../tests/private/protocol.rs"]
mod tests;
