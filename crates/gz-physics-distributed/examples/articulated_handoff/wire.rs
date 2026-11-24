// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bincode::Options;
use serde::de::DeserializeOwned;
use std::io::{Read, Write};
const MAGIC: &[u8; 8] = b"GZHNG001";
const META: usize = 512;
fn encoding() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(META as u64)
        .reject_trailing_bytes()
}
pub(super) fn encode<T: Serialize>(x: &T) -> Result<Vec<u8>> {
    Ok(encoding().serialize(x)?)
}
pub(super) fn frame<T: Serialize>(mut out: impl Write, header: &T, image: &[u8]) -> Result<()> {
    let bytes = encode(header)?;
    if bytes.is_empty() || bytes.len() > META || !matches!(image.len(), 0 | 369 | 377) {
        return Err("invalid hinge frame size".into());
    }
    out.write_all(MAGIC)?;
    out.write_all(&(bytes.len() as u16).to_le_bytes())?;
    out.write_all(&(image.len() as u16).to_le_bytes())?;
    out.write_all(&bytes)?;
    out.write_all(image)?;
    out.flush()?;
    Ok(())
}
pub(super) fn read<T: DeserializeOwned>(mut input: impl Read) -> Result<(T, Vec<u8>)> {
    let mut prefix = [0; 12];
    input.read_exact(&mut prefix)?;
    let n = u16::from_le_bytes(prefix[8..10].try_into()?) as usize;
    let m = u16::from_le_bytes(prefix[10..12].try_into()?) as usize;
    if &prefix[..8] != MAGIC || !(1..=META).contains(&n) || !matches!(m, 0 | 369 | 377) {
        return Err("unknown or oversized hinge frame".into());
    }
    let mut raw = vec![0; n];
    input.read_exact(&mut raw)?;
    let header = encoding().deserialize(&raw)?;
    let mut image = vec![0; m];
    input.read_exact(&mut image)?;
    Ok((header, image))
}
