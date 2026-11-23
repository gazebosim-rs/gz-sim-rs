// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Framing for a trusted local resident-pair worker. The result includes its measured device transfers.
use crate::{
    PairBlockAssignment as Job, PairBlockContinuation as Cue, PairBlockExecutor, PairBlockReceipt,
};
use std::io::{self, Read, Write};

const MISS: u8 = 0;
const DONE: u8 = 1;
const ERROR: u8 = 2;
const ROWS: u8 = 3;
pub(super) const ASK_ROWS: &[u8] = b"GZGPFMT1";
const ROWS_READY: &[u8] = b"GZGPROW1";
const HEADER: usize = 26;
const MAX: usize = Job::MAX_BYTES + HEADER;
const READY: &[u8; 8] = b"GZGPWK01";
pub(super) enum Response {
    Miss,
    Done {
        receipt: PairBlockReceipt,
        reused: bool,
        upload: u64,
        readback: u64,
        dispatch_ns: Option<u64>,
    },
    Error(String),
}
fn invalid(s: impl ToString) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.to_string())
}
pub(super) fn send(w: &mut impl Write, data: &[u8]) -> io::Result<()> {
    if data.is_empty() || data.len() > MAX {
        return Err(invalid("invalid GPU worker frame size"));
    }
    w.write_all(&(data.len() as u32).to_le_bytes())?;
    w.write_all(data)?;
    w.flush()
}
pub(super) fn receive(r: &mut impl Read) -> io::Result<Option<Vec<u8>>> {
    let mut head = [0; 4];
    loop {
        match r.read(&mut head[..1]) {
            Ok(0) => return Ok(None),
            Ok(_) => break,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        }
    }
    r.read_exact(&mut head[1..])?;
    let n = u32::from_le_bytes(head) as usize;
    if n == 0 || n > MAX {
        return Err(invalid("invalid GPU worker frame length"));
    }
    let mut bytes = vec![0; n];
    r.read_exact(&mut bytes)?;
    Ok(Some(bytes))
}
pub(super) fn handshake(r: &mut impl Read) -> io::Result<()> {
    if receive(r)?.as_deref() == Some(READY) {
        Ok(())
    } else {
        Err(invalid("incompatible GPU worker handshake"))
    }
}
pub(super) fn check_format_reply(data: &[u8]) -> io::Result<()> {
    // Earlier workers report an unknown record as a bounded error and continue serving
    // the self-contained format. Both reply formats remain accepted after negotiation.
    if data == ROWS_READY || matches!(data,[ERROR, msg @ ..] if msg.len()<=4096) {
        Ok(())
    } else {
        Err(invalid("incompatible GPU worker format negotiation"))
    }
}
pub(super) fn response(
    data: &[u8],
    barrier: &crate::PairBlockBarrier<'_>,
    ordinal: usize,
) -> io::Result<Response> {
    match data {
        [MISS] => Ok(Response::Miss),
        [ERROR, message @ ..] if message.len() <= 4096 => Ok(Response::Error(
            String::from_utf8_lossy(message).into_owned(),
        )),
        [kind @ (DONE | ROWS), reused @ (0 | 1), ..] if data.len() > HEADER => {
            let n = |i| u64::from_le_bytes(data[i..i + 8].try_into().unwrap());
            Ok(Response::Done {
                receipt: if *kind == ROWS {
                    barrier.observation_receipt(ordinal, &data[HEADER..])
                } else {
                    PairBlockReceipt::from_bytes(&data[HEADER..])
                }
                .map_err(invalid)?,
                reused: *reused == 1,
                upload: n(2),
                readback: n(10),
                dispatch_ns: (n(18) != u64::MAX).then(|| n(18)),
            })
        }
        _ => Err(invalid("invalid GPU worker reply")),
    }
}
/// Serve requests until the owning simulator closes its input. A cache miss changes no device state.
pub fn serve(adapter: &str, mut input: impl Read, mut output: impl Write) -> io::Result<()> {
    send(&mut output, READY)?;
    let mut executor = PairBlockExecutor::new(adapter);
    let mut rows = false;
    while let Some(bytes) = receive(&mut input)? {
        if bytes == ASK_ROWS {
            rows = true;
            send(&mut output, ROWS_READY)?;
            continue;
        }
        let mut job = || -> Result<_, crate::GpuError> {
            if bytes.len() == Cue::BYTES {
                Ok(executor
                    .try_continue(Cue::from_bytes(&bytes)?)?
                    .map(|(r, m)| (r, m, true)))
            } else {
                executor.execute(Job::from_bytes(&bytes)?).map(Some)
            }
        };
        let result = job().and_then(|r| {
            let Some((receipt, m, warm)) = r else {
                return Ok(vec![MISS]);
            };
            let mut data = Vec::new();
            data.extend_from_slice(&[if rows { ROWS } else { DONE }, u8::from(warm)]);
            for n in [
                m.uploaded_bytes,
                m.readback_bytes,
                m.gpu_dispatch.map_or(u64::MAX, |d| {
                    d.as_nanos().min(u128::from(u64::MAX - 1)) as u64
                }),
            ] {
                data.extend_from_slice(&n.to_le_bytes());
            }
            data.extend_from_slice(&if rows {
                receipt.observation_bytes()?
            } else {
                receipt.to_bytes()?
            });
            Ok(data)
        });
        let response = result.unwrap_or_else(|e| {
            let mut data = vec![ERROR];
            data.extend(e.to_string().bytes().take(4096));
            data
        });
        send(&mut output, &response)?;
    }
    Ok(())
}
