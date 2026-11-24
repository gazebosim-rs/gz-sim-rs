// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe C++ subscriptions on the sole routing peer of a private test network.

use std::time::{Duration, Instant};
use zenoh::{Wait, pubsub::Publisher};

pub struct SubscriptionRoutes(Vec<Publisher<'static>>);

impl SubscriptionRoutes {
    pub fn new(session: &zenoh::Session, keys: impl IntoIterator<Item = String>) -> Self {
        Self(
            keys.into_iter()
                .map(|key| session.declare_publisher(key).wait().unwrap())
                .collect(),
        )
    }

    /// A local C++ Subscribe/Unsubscribe return precedes its declaration
    /// reaching the routing peer. Hold the simulation clock at this boundary.
    pub fn wait(&self, expected: bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        for route in &self.0 {
            let changes = route.matching_listener().wait().unwrap();
            while route.matching_status().wait().unwrap().matching() != expected {
                let event = changes
                    .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                    .unwrap();
                assert!(
                    event.is_some(),
                    "subscription route {} did not reach matching={expected}",
                    route.key_expr()
                );
            }
        }
    }
}
