// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn admit(query: &str, replay_enabled: bool) -> bool {
    let callback = LocalViewer {
        scan: Arc::new(AtomicBool::new(false)),
        lifecycle: Arc::new(AtomicBool::new(false)),
        lifecycle_enabled: true,
        replay_enabled,
        credit: Arc::new(AtomicBool::new(false)),
    };
    let request = Request::builder()
        .uri(format!("/?{query}"))
        .body(())
        .unwrap();
    callback.on_request(&request, Response::new(())).is_ok()
}
#[test]
fn explicit_replay_and_scan_domains_preserve_live_defaults() {
    assert!(admit("", false));
    assert!(admit("scan=1", false));
    assert!(admit("lifecycle=1&lifecycle_credit=1&scan=1", false));
    for query in [
        "replay=1",
        "replay=0",
        "lifecycle=1&lifecycle_credit=1&replay=1",
    ] {
        assert!(!admit(query, false));
    }
    let replay = "replay=1&lifecycle=1&lifecycle_credit=1";
    assert!(admit(replay, true));
    for query in [
        "",
        "replay=1",
        "replay=1&lifecycle=1",
        "lifecycle=1&lifecycle_credit=1",
        "replay=0&lifecycle=1&lifecycle_credit=1",
        "replay=1&replay=1&lifecycle=1&lifecycle_credit=1",
    ] {
        assert!(!admit(query, true), "{query}");
    }
    for scan in ["scan=1", "scan=0", "scan="] {
        assert!(!admit(&format!("{replay}&{scan}"), true));
    }
}
