// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded JSON-line IO for the local scene recording, using maintained serde.
//! A refused stream is terminal; callers cannot resume after a malformed record.
use serde::{Serialize, de::DeserializeOwned};
use std::io::{self, BufRead, Cursor, Read};

pub(super) const LINE_BYTES: usize = 256 * 1024;
pub(super) const FILE_BYTES: u64 = 64 * 1024 * 1024;
pub(super) const STREAM_LINES: u32 = 4098; // Header, at most 4096 frames, footer.

#[derive(Clone, Copy)]
pub(super) struct Limits {
    pub line_bytes: usize,
    pub file_bytes: u64,
    pub lines: u32,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            line_bytes: LINE_BYTES,
            file_bytes: FILE_BYTES,
            lines: STREAM_LINES,
        }
    }
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

pub(super) struct Lines<R> {
    reader: R,
    limits: Limits,
    bytes: u64,
    lines: u32,
    refused: bool,
}

impl<R: BufRead> Lines<R> {
    pub fn new(reader: R, limits: Limits) -> io::Result<Self> {
        if limits.line_bytes == 0
            || limits.line_bytes > LINE_BYTES
            || limits.file_bytes < limits.line_bytes as u64
            || limits.file_bytes > FILE_BYTES
            || limits.lines == 0
            || limits.lines > STREAM_LINES
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid recording IO bounds",
            ));
        }
        Ok(Self {
            reader,
            limits,
            bytes: 0,
            lines: 0,
            refused: false,
        })
    }

    pub fn offset(&self) -> u64 {
        self.bytes
    }

    pub fn next_record<T: DeserializeOwned>(&mut self) -> io::Result<Option<T>> {
        if self.refused {
            return Err(invalid("recording stream was already refused"));
        }
        let result = self.read();
        if result.is_err() {
            self.refused = true;
        }
        result
    }

    fn read<T: DeserializeOwned>(&mut self) -> io::Result<Option<T>> {
        let remaining = self.limits.file_bytes - self.bytes;
        // One sentinel byte detects overflow without ever buffering the rest of
        // an attacker-controlled line. The file budget also includes newlines.
        let cap = (self.limits.line_bytes as u64).min(remaining) + 1;
        let mut bytes = Vec::with_capacity(cap as usize);
        self.reader
            .by_ref()
            .take(cap)
            .read_until(b'\n', &mut bytes)?;
        if bytes.is_empty() {
            return Ok(None);
        }
        if bytes.len() as u64 > remaining {
            return Err(invalid("recording file exceeds byte bound"));
        }
        if bytes.len() > self.limits.line_bytes {
            return Err(invalid("recording line exceeds byte bound"));
        }
        if self.lines == self.limits.lines {
            return Err(invalid("recording exceeds record-count bound"));
        }
        if bytes.last() != Some(&b'\n') {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "unterminated recording line",
            ));
        }
        let value = serde_json::from_slice(&bytes)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        self.bytes += bytes.len() as u64;
        self.lines += 1;
        Ok(Some(value))
    }
}

/// Serialize before touching the output file, with allocation bounded from the
/// first byte. `serde_json::to_vec` followed by a length check is not sufficient.
pub(super) fn encode<T: Serialize + ?Sized>(value: &T, limit: usize) -> io::Result<Vec<u8>> {
    if limit == 0 || limit > LINE_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid recording line bound",
        ));
    }
    // A standard Cursor over a fixed slice cannot grow its backing storage.
    let mut bytes = vec![0; limit];
    let length = {
        let mut output = Cursor::new(&mut bytes[..limit - 1]);
        serde_json::to_writer(&mut output, value)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
        output.position() as usize
    };
    bytes[length] = b'\n';
    bytes.truncate(length + 1);
    Ok(bytes)
}

#[cfg(test)]
#[path = "lines_tests.rs"]
mod tests;
