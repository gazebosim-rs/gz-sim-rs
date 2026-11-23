// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::articulation::ledger::Ledger;
use std::{ops::Deref, sync::Arc};

/// A subgraph's operations remain private until its containing world publishes its cut.
pub(super) enum FleetSession {
    Standalone(Ledger),
    Contained(Arc<Ledger>),
}
impl From<Ledger> for FleetSession {
    fn from(ledger: Ledger) -> Self {
        Self::Standalone(ledger)
    }
}
impl Deref for FleetSession {
    type Target = Ledger;
    fn deref(&self) -> &Ledger {
        match self {
            Self::Standalone(l) => l,
            Self::Contained(l) => l,
        }
    }
}
impl FleetSession {
    pub fn standalone(&self) -> bool {
        matches!(self, Self::Standalone(_))
    }
}
