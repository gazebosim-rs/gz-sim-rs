// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in, per-process timing for the stock pipe protocol and its durable barrier.
//! Set GZ_MULTI_SPHERE_TRACE_DIR to an existing private tmpfs directory.
//! Each request emits one JSON line after taking its final timestamp.
use super::Command;
use serde::Serialize;
use std::{
    cell::RefCell,
    fs::{File, OpenOptions},
    io::Write,
    path::PathBuf,
    sync::OnceLock,
    time::Instant,
};

static DIR: OnceLock<Option<PathBuf>> = OnceLock::new();

#[derive(Default, Serialize)]
pub(crate) struct Costs {
    pub(crate) native_step_ns: u64,
    pub(crate) checkpoint_ns: u64,
    pub(crate) receipt_encode_ns: u64,
    pub(crate) request_encode_ns: u64,
    pub(crate) receipt_decode_ns: u64,
    pub(crate) archive_encode_ns: u64,
    pub(crate) file_sync_ns: u64,
    pub(crate) dir_sync_ns: u64,
}
#[derive(Serialize)]
struct Event {
    role: &'static str,
    command: &'static str,
    tick: u64,
    slot: Option<u8>,
    elapsed_ns: u64,
    #[serde(flatten)]
    costs: Costs,
}

thread_local! {
    static EVENTS: RefCell<Option<Event>> = const { RefCell::new(None) };
    static FILE: RefCell<Option<File>> = const { RefCell::new(None) };
}

pub(crate) struct Scope(Instant);
impl Scope {
    pub(crate) fn begin(
        role: &'static str,
        command: &'static str,
        tick: u64,
        slot: Option<u8>,
    ) -> Option<Self> {
        let dir = DIR
            .get_or_init(|| std::env::var_os("GZ_MULTI_SPHERE_TRACE_DIR").map(PathBuf::from))
            .as_ref()?;
        FILE.with_borrow_mut(|file| {
            file.get_or_insert_with(|| {
                let path = dir.join(format!("pair-{}.jsonl", std::process::id()));
                OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(path)
                    .expect("open sphere-pair timing file")
            });
        });
        EVENTS.with_borrow_mut(|event| {
            debug_assert!(event.is_none());
            *event = Some(Event {
                role,
                command,
                tick,
                slot,
                elapsed_ns: 0,
                costs: Costs::default(),
            });
        });
        Some(Self(Instant::now()))
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        let elapsed_ns = self.0.elapsed().as_nanos() as u64;
        let mut event = EVENTS.take().expect("active timing scope");
        event.elapsed_ns = elapsed_ns;
        let mut row = serde_json::to_vec(&event).expect("serialize sphere-pair timing");
        row.push(b'\n');
        FILE.with_borrow_mut(|file| {
            file.as_mut()
                .unwrap()
                .write_all(&row)
                .expect("write sphere-pair timing")
        });
    }
}
pub(super) fn worker_command(cmd: &Command) {
    EVENTS.with_borrow_mut(|event| {
        if let Some(event) = event {
            event.command = cmd.name();
        }
    });
}
pub(crate) fn measure<T>(field: impl FnOnce(&mut Costs) -> &mut u64, op: impl FnOnce() -> T) -> T {
    if !EVENTS.with_borrow(Option::is_some) {
        return op();
    }
    let timer = Instant::now();
    let result = op();
    let elapsed = timer.elapsed().as_nanos() as u64;
    EVENTS.with_borrow_mut(|event| *field(&mut event.as_mut().unwrap().costs) += elapsed);
    result
}
