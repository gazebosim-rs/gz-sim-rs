// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Unprivileged Linux per-thread scheduler accounting, sampled only on request.
use serde::Serialize;
use std::{
    fs::{self, File},
    os::unix::fs::FileExt,
    sync::Arc,
};

pub(super) struct ThreadStat {
    tid: u32,
    file: File,
}

#[derive(Serialize)]
pub(super) struct Sample {
    tid: u32,
    runtime_ns: u64,
    runqueue_ns: u64,
    slices: u64,
}

thread_local! {
    static STAT: Option<Arc<ThreadStat>> = ThreadStat::open().map(Arc::new);
}

impl ThreadStat {
    fn open() -> Option<Self> {
        let path = fs::read_link("/proc/thread-self").ok()?;
        let tid = path.file_name()?.to_str()?.parse().ok()?;
        let file = File::open("/proc/thread-self/schedstat").ok()?;
        Some(Self { tid, file })
    }

    pub(super) fn current() -> Option<Arc<Self>> {
        STAT.with(Clone::clone)
    }

    pub(super) fn sample(&self) -> Option<Sample> {
        let mut buf = [0u8; 128];
        let n = self.file.read_at(&mut buf, 0).ok()?;
        let mut fields = std::str::from_utf8(&buf[..n]).ok()?.split_whitespace();
        Some(Sample {
            tid: self.tid,
            runtime_ns: fields.next()?.parse().ok()?,
            runqueue_ns: fields.next()?.parse().ok()?,
            slices: fields.next()?.parse().ok()?,
        })
    }
}
