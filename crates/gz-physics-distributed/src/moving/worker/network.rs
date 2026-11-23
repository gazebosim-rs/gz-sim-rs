// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Managed loopback TCP sessions. Bootstrap and controller lifetime use private pipes.
use super::*;
use crate::transport::{config, endpoint_valid};
use std::num::NonZeroU16;
use std::process::{ChildStdin, ChildStdout};
use zenoh::{Wait, handlers::FifoChannel};

pub(super) const OFFER: &[u8; 8] = b"GZPZNH01";

/// The remote worker binds one loopback port. Its controller connects at
/// the local end of the same SSH forward.
#[derive(Clone, Copy)]
pub(crate) struct ForwardedPorts {
    pub remote: NonZeroU16,
    pub local: NonZeroU16,
}

#[derive(Serialize, Deserialize)]
struct Ready {
    digest: [u8; 32],
    endpoint: String,
}

fn key(token: [u8; 16]) -> String {
    format!(
        "gz/physics/native/v1/{}",
        uuid::Uuid::from_bytes(token).simple()
    )
}
fn unavailable(error: impl std::fmt::Display) -> io::Error {
    io::Error::other(error.to_string())
}

/// The pipe offer and the loopback handshake share the caller's startup deadline.
fn connect_until(endpoint: &str, deadline: Instant) -> io::Result<zenoh::Session> {
    let mut cfg = config(Some(endpoint))?;
    let remaining_ms = deadline
        .saturating_duration_since(Instant::now())
        .as_millis();
    // Zenoh interprets zero as an unlimited connection timeout.
    if remaining_ms == 0 {
        return Err(io::ErrorKind::TimedOut.into());
    }
    cfg.insert_json5("connect/timeout_ms", &remaining_ms.to_string())
        .map_err(unavailable)?;
    let session = zenoh::open(cfg).wait().map_err(|error| {
        // Only a connection to an already validated, offered endpoint failed.
        // Preserve eligibility for bounded worker replacement.
        let kind = if Instant::now() >= deadline {
            io::ErrorKind::TimedOut
        } else {
            io::ErrorKind::ConnectionAborted
        };
        io::Error::new(kind, error.to_string())
    })?;
    if Instant::now() >= deadline {
        return Err(io::ErrorKind::TimedOut.into());
    }
    Ok(session)
}

#[cfg(test)]
mod tests;

pub(super) fn serve<C, A>(
    output: &mut impl Write,
    boot: &[u8],
    token: [u8; 16],
    mut commands: Commands,
    mut handle: impl FnMut(u64, C, &mut u64) -> Result<A>,
    forward: Option<ForwardedPorts>,
) -> io::Result<()>
where
    C: DeserializeOwned,
    A: Serialize,
{
    // EOF fences all native state even when the controller disappears between RPCs.
    std::thread::Builder::new()
        .name("physics-owner".into())
        .spawn(|| {
            let mut byte = [0; 1];
            let mut owner = io::stdin().lock();
            loop {
                match owner.read(&mut byte) {
                    Ok(0) => std::process::exit(0),
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                    _ => std::process::exit(1),
                }
            }
        })?;
    let mut cfg = config(None)?;
    let bind = forward.map(|p| format!("tcp/127.0.0.1:{}", p.remote));
    if let Some(address) = &bind {
        cfg.insert_json5("listen/endpoints", &serde_json::to_string(&[address])?)
            .map_err(unavailable)?;
    }
    let session = zenoh::open(cfg).wait().map_err(unavailable)?;
    let key = key(token);
    let queries = session
        .declare_queryable(key.as_str())
        .complete(true)
        .with(FifoChannel::new(8))
        .wait()
        .map_err(unavailable)?;
    let endpoint = session
        .info()
        .locators()
        .wait()
        .iter()
        .map(ToString::to_string)
        .find(|v| endpoint_valid(v) && bind.as_ref().is_none_or(|expected| expected == v))
        .ok_or_else(|| unavailable("worker did not bind the requested loopback locator"))?;
    let endpoint = forward.map_or(endpoint, |p| format!("tcp/127.0.0.1:{}", p.local));
    let digest = Sha256::digest(boot).into();
    write_frame(output, &encode(&Ready { digest, endpoint })?)?;
    loop {
        let q = queries.recv().map_err(unavailable)?;
        let Some(payload) = q.payload().filter(|p| p.len() <= LIMIT) else {
            let _ = q.reply_err("invalid native worker payload").wait();
            continue;
        };
        let bytes = payload.to_bytes();
        match commands.apply(&bytes, &mut handle) {
            Ok(reply) => {
                let _ = q.reply(key.as_str(), reply.to_vec()).express(true).wait();
            }
            Err(error) => {
                let _ = q.reply_err("invalid native worker command").wait();
                return Err(error);
            }
        }
    }
}

pub(super) fn exchange(
    input: &mut ChildStdin,
    output: &mut ChildStdout,
    commands: Receiver<(Vec<u8>, Instant)>,
    responses: &Sender<io::Result<Vec<u8>>>,
    token: [u8; 16],
) -> io::Result<()> {
    let (boot, deadline) = commands.recv().map_err(unavailable)?;
    let mut offer = Vec::with_capacity(OFFER.len() + boot.len());
    offer.extend_from_slice(OFFER);
    offer.extend_from_slice(&boot);
    write_frame(input, &offer)?;
    let ready: Ready = codec()
        .deserialize(&read_frame(output)?)
        .map_err(unavailable)?;
    if ready.digest[..] != Sha256::digest(&boot)[..] {
        return Err(unavailable("Zenoh offer does not match worker recipe"));
    }
    let session = connect_until(&ready.endpoint, deadline)?;
    let key = key(token);
    responses
        .send(Ok(ready.digest.to_vec()))
        .map_err(unavailable)?;
    for (cmd, until) in commands {
        let remaining = until
            .checked_duration_since(Instant::now())
            .ok_or(io::ErrorKind::TimedOut)?;
        let replies = session
            .get(key.as_str())
            .express(true)
            .consolidation(zenoh::query::ConsolidationMode::None)
            .payload(cmd)
            .timeout(remaining)
            .with(FifoChannel::new(1))
            .wait()
            .map_err(unavailable)?;
        let reply = replies
            .recv_deadline(until)
            .map_err(|e| io::Error::new(io::ErrorKind::BrokenPipe, e.to_string()))?
            .ok_or(io::ErrorKind::TimedOut)?;
        let sample = reply.result().map_err(unavailable)?;
        if sample.key_expr().as_str() != key || sample.payload().len() > LIMIT {
            return Err(unavailable("unexpected native Zenoh reply"));
        }
        responses
            .send(Ok(sample.payload().to_bytes().into_owned()))
            .map_err(unavailable)?;
    }
    Ok(())
}
