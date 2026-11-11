// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Scope cleanup for tests which deliberately stall a real callback.
use std::sync::{Arc, Condvar, Mutex};

pub(crate) struct CallbackRelease {
    gate: Arc<(Mutex<bool>, Condvar)>,
}

impl CallbackRelease {
    pub(crate) fn new(gate: Arc<(Mutex<bool>, Condvar)>) -> Self {
        Self { gate }
    }

    pub(crate) fn release(&self) {
        let (lock, condition) = &*self.gate;
        // Cleanup must still release a callback while another assertion unwinds.
        let mut released = lock.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        *released = true;
        condition.notify_all();
    }
}

impl Drop for CallbackRelease {
    fn drop(&mut self) {
        self.release();
    }
}
