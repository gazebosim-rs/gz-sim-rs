// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

pub fn patch(
    src: &[u8],
    file: &Path,
    change: impl FnOnce(&mut Value, &mut Value),
) -> Result<(), Box<dyn std::error::Error>> {
    let m = u32::from_le_bytes(src[40..44].try_into()?) as usize;
    let end = 44 + m;
    let native_tag = &src[end..end + 8];
    let mut metadata: Value = serde_json::from_slice(&src[44..end])?;
    let mut native: Value = serde_json::from_slice(&src[end + 8..])?;
    change(&mut metadata, &mut native);
    let meta = serde_json::to_vec(&metadata)?;
    let mut body = Vec::new();
    body.extend_from_slice(&(meta.len() as u32).to_le_bytes());
    body.extend(meta);
    body.extend(native_tag);
    body.extend(serde_json::to_vec(&native)?);
    let mut bytes = Vec::from(&src[..8]);
    bytes.extend(Sha256::digest(&body));
    bytes.extend(body);
    fs::write(file, bytes)?;
    Ok(())
}
