// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy_ecs::prelude::*;
use std::{error::Error, sync::OnceLock};

/// First reported system error. A reported fault makes this simulation terminal.
/// Shared reporting keeps otherwise independent Bevy systems parallel.
#[derive(Resource, Default, Debug)]
pub struct StepFault {
    error: OnceLock<Box<dyn Error + Send + Sync>>,
}

impl StepFault {
    pub fn report(&self, error: impl Error + Send + Sync + 'static) {
        let _ = self.error.set(Box::new(error));
    }

    pub fn error(&self) -> Option<&(dyn Error + Send + Sync + 'static)> {
        self.error.get().map(Box::as_ref)
    }
}

pub(crate) fn healthy(fault: Res<StepFault>) -> bool {
    fault.error().is_none()
}
