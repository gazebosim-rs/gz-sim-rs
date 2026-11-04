// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn exact_credit_echo_is_connection_local_monotonic_and_single_use() {
    let mut credit = PublicationCredit::new();
    assert_eq!(credit.reserve(), Some(1));
    assert_eq!(credit.reserve(), None);
    assert!(credit.ready(r#"{"version":1,"kind":"publication_ready","token":1}"#));
    assert!(!credit.ready(r#"{"version":1,"kind":"publication_ready","token":1}"#));
    assert_eq!(credit.reserve(), Some(2));
    assert!(!credit.ready(r#"{"version":1,"kind":"publication_ready","token":1}"#));
    assert_eq!(credit.awaiting, Some(2));
    assert!(credit.ready(r#"{"version":1,"kind":"publication_ready","token":2}"#));
    let mut reconnected = PublicationCredit::new();
    assert!(!reconnected.ready(r#"{"version":1,"kind":"publication_ready","token":2}"#));
    assert_eq!(reconnected.reserve(), Some(1));
    credit.next = u64::MAX;
    assert_eq!(credit.reserve(), None);
    assert_eq!(credit.awaiting, None);
    assert_eq!(credit.next, u64::MAX);
}

#[test]
fn malformed_credit_never_releases_the_outstanding_publication() {
    let mut credit = PublicationCredit::new();
    assert_eq!(credit.reserve(), Some(1));
    for text in [
        "not json",
        r#"{"version":1,"kind":"publication_ready"}"#,
        r#"{"version":null,"kind":"publication_ready","token":1}"#,
        r#"{"version":2,"kind":"publication_ready","token":1}"#,
        r#"{"version":1,"kind":"other","token":1}"#,
        r#"{"version":1,"kind":"publication_ready","token":0}"#,
        r#"{"version":1,"kind":"publication_ready","token":-1}"#,
        r#"{"version":1,"kind":"publication_ready","token":1.0}"#,
        r#"{"version":1,"kind":"publication_ready","token":null}"#,
        r#"{"version":1,"kind":"publication_ready","token":2}"#,
        r#"{"version":1,"kind":"publication_ready","token":1,"extra":true}"#,
        r#"{"version":1,"kind":"publication_ready","token":1,"token":1}"#,
        r#"{"version":1,"version":1,"kind":"publication_ready","token":1}"#,
        r#"{"version":1,"kind":"publication_ready","kind":"publication_ready","token":1}"#,
        r#"{"version":1,"kind":"publication_ready","token":18446744073709551616}"#,
    ] {
        assert!(!credit.ready(text), "{text}");
        assert_eq!(credit.awaiting, Some(1), "{text}");
        assert_eq!(credit.next, 2, "{text}");
    }
}
