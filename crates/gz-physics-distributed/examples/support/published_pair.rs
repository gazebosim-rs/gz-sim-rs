// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the exact checkpoint bytes consumed by the bounded public decoder.
use gz_physics_distributed::{
    articulation::ArticulationError,
    multi_sphere::{PublishedCut, read_published_cut},
};
use std::{
    fs::OpenOptions,
    io::{self, Read},
    path::Path,
};

pub(super) fn snapshot(path: &Path) -> Result<(PublishedCut, Vec<u8>), ArticulationError> {
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(target_os = "linux")]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags(rustix::fs::OFlags::NONBLOCK.bits() as i32);
    }
    let file = options.open(path)?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected a regular checkpoint file",
        )
        .into());
    }
    let mut reader = Capture {
        file,
        bytes: Vec::new(),
    };
    let cut = read_published_cut(&mut reader)?;
    Ok((cut, reader.bytes))
}
struct Capture {
    file: std::fs::File,
    bytes: Vec<u8>,
}
impl Read for Capture {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = self.file.read(buf)?;
        self.bytes.extend_from_slice(&buf[..n]);
        Ok(n)
    }
}
