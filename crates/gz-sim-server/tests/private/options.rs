/*
 * Copyright (C) 2014, 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn native_topic_namespace_and_partition_grammar() {
    // TopicUtils_TEST.cc: testTopics, testNamespaces and testPartitions.
    let max = "a".repeat(65535);
    let too_long = "a".repeat(65536);
    for name in [
        "abc", "/abc", "abc/de", "a", "abc/", "/abc/", "/abc/d", "/abc/d/e", &max,
    ] {
        assert!(valid_name(name));
    }
    for name in [
        "/abcde",
        "abcde",
        "abcde/",
        "/abcde/",
        "/abcde/fg",
        "/abcde/fg/",
        "",
        &max,
    ] {
        assert!(valid_name(name));
    }
    for name in [
        "/",
        " ",
        "~a",
        "topic1 ",
        "abc//def",
        "ab~cd",
        "~/",
        "~",
        "@partition",
        "topic:=",
        "ns ",
        "~/abcde",
        "~abcde",
        "@namespace",
        "namespace:=",
        &too_long,
    ] {
        assert!(!valid_name(name), "{name:?}");
        for (partition, namespace, topic) in [(name, "", "ok"), ("", name, "ok"), ("", "", name)] {
            assert_eq!(
                qualified_name(partition, namespace, topic),
                Err(SetupError::InvalidName)
            );
        }
    }
    assert_eq!(qualified_name("", "", ""), Err(SetupError::InvalidName));
    // The native grammar permits non-ASCII spaces, tabs and embedded NUL.
    for name in ["a\tb", "a\u{a0}b", "a\0b", "é"] {
        assert!(valid_name(name));
    }
    assert!(valid_name(&"é".repeat(32767)));
    assert!(!valid_name(&"é".repeat(32768)));
}

#[test]
fn native_qualification_cross_product_and_exact_byte_boundary() {
    // Complete TopicUtils_TEST.cc:testFullyQualifiedName table and cross product.
    let long = "a".repeat(65536);
    let good = "a".repeat(65532);
    let good_partition = format!("@/{good}@");
    let good_namespace = format!("/{good}/");
    let partitions = [
        ("@partition", None),
        ("@partition/@", None),
        ("@@", None),
        (long.as_str(), None),
        ("partition", Some("@/partition@")),
        ("", Some("@@")),
        ("partition/", Some("@/partition@")),
        (good.as_str(), Some(good_partition.as_str())),
    ];
    let namespaces = [
        ("~ns", None),
        (long.as_str(), None),
        ("", Some("/")),
        ("abc", Some("/abc/")),
        (good.as_str(), Some(good_namespace.as_str())),
    ];
    let topics = [
        ("~/def", None),
        ("~/def/", None),
        ("~def", None),
        ("~def/", None),
        (long.as_str(), None),
        ("/def", Some("/def")),
        ("def/", Some("def")),
        ("def/ghi", Some("def/ghi")),
        ("def/ghi/", Some("def/ghi")),
        (good.as_str(), Some(good.as_str())),
    ];
    for (partition, p) in partitions {
        for (namespace, ns) in namespaces {
            for (topic, t) in topics {
                let expected = match (p, ns, t) {
                    (Some(p), Some(_), Some(t)) if topic.starts_with('/') => {
                        Some(format!("{p}{t}"))
                    }
                    (Some(p), Some(ns), Some(t)) => Some(format!("{p}{ns}{t}")),
                    _ => None,
                }
                .filter(|name| name.len() <= 65535);
                assert_eq!(qualified_name(partition, namespace, topic).ok(), expected);
            }
        }
    }
    for (partition, namespace, topic, expected) in [
        ("/p/", "/ns/", "t/", "@/p@/ns/t"),
        ("/p", "ns", "/t/", "@/p@/t"),
        ("", "", "t", "@@/t"),
    ] {
        assert_eq!(
            qualified_name(partition, namespace, topic).unwrap(),
            expected
        );
    }
    assert_eq!(qualified_name("", "", &good).unwrap().len(), 65535);
    assert_eq!(
        qualified_name("", "", &(good + "a")),
        Err(SetupError::InvalidName)
    );
}

#[test]
fn native_sanitizer_is_one_pass_and_preserves_other_utf8_bytes() {
    // TopicUtils_TEST.cc:asValidTopic, plus interacting substitutions.
    for topic in [
        "/abc",
        "abc/de",
        "a",
        "ABC/",
        "/abc/d",
        "/abc/d/e",
        "a(bc)d-e_f=h+i.j",
    ] {
        assert_eq!(as_valid_topic(topic), topic);
    }
    for (input, expected) in [
        ("a b  c", "a_b__c"),
        ("a@b@c", "abc"),
        ("a:=b:=c", "abc"),
        ("a//b/c", "ab/c"),
        ("a~b~c", "abc"),
        ("a///b", "a/b"),
        ("////a", "a"),
        ("a/~/b", ""),
        (":~=", ""),
        ("é\t\u{a0}x\0", "é\t\u{a0}x\0"),
        ("", ""),
        ("@@@", ""),
        ("~@~", ""),
        ("/", ""),
        ("///", ""),
    ] {
        assert_eq!(as_valid_topic(input), expected, "{input:?}");
    }
    assert_eq!(as_valid_topic(&"a".repeat(65535)).len(), 65535);
    assert!(as_valid_topic(&"a".repeat(65536)).is_empty());
}

#[test]
fn remaps_are_bounded_exact_single_hop_and_rejections_preserve_state() {
    // NodeOptions_TEST.cc:WrongTopicRemap and native raw TopicRemap lookup.
    let mut options = ServerOptions {
        capacity: 3,
        ..Default::default()
    };
    assert_eq!(
        options.add_topic_remap("  ", "/bar"),
        Err(SetupError::InvalidName)
    );
    assert_eq!(
        options.add_topic_remap("/foo", "  "),
        Err(SetupError::InvalidName)
    );
    options.add_topic_remap("from", "to").unwrap();
    options.add_topic_remap("to", "from").unwrap();
    options.add_topic_remap("self", "self").unwrap();
    assert_eq!(options.topic_remap("from"), Some("to"));
    assert_eq!(options.topic_remap("to"), Some("from"));
    assert_eq!(options.topic_remap("self"), Some("self"));
    for absent in ["/from", "from/", "FROM", "", "@/p@/from"] {
        assert_eq!(options.topic_remap(absent), None);
    }
    let saved = options.clone();
    assert_eq!(
        options.add_topic_remap("from", "bad name"),
        Err(SetupError::InvalidName)
    );
    assert_eq!(
        options.add_topic_remap("from", "other"),
        Err(SetupError::AlreadyRemapped)
    );
    assert_eq!(
        options.add_topic_remap("new", "other"),
        Err(SetupError::QueueFull)
    );
    for invalid in [
        "",
        "/",
        "a b",
        "a~b",
        "a@b",
        "a//b",
        "a:=b",
        &"a".repeat(65536),
    ] {
        assert_eq!(
            options.add_topic_remap(invalid, "valid"),
            Err(SetupError::InvalidName)
        );
        assert_eq!(
            options.add_topic_remap("valid", invalid),
            Err(SetupError::InvalidName)
        );
    }
    assert_eq!(options, saved);
    assert_eq!(options.validate(), Ok(()));
    let mut zero = ServerOptions {
        capacity: 0,
        ..Default::default()
    };
    assert_eq!(
        zero.add_topic_remap("from", "to"),
        Err(SetupError::QueueFull)
    );
    assert!(zero.topic_remaps.is_empty());
}

#[test]
fn direct_field_edits_are_validated_without_mutation() {
    let options = ServerOptions::default();
    assert_eq!(
        (options.capacity, options.max_message_bytes),
        (1000, 16 * 1024 * 1024)
    );
    assert_eq!(
        (&options.partition, &options.namespace),
        (&String::new(), &String::new())
    );
    assert!(options.topic_remaps.is_empty());
    options.validate().unwrap();
    let mut boundary = ServerOptions {
        capacity: i32::MAX as usize,
        max_message_bytes: i64::MAX as usize,
        ..Default::default()
    };
    boundary.validate().unwrap();
    for capacity in [0, i32::MAX as usize + 1] {
        boundary.capacity = capacity;
        assert_eq!(boundary.validate(), Err(SetupError::InvalidConfig));
    }
    boundary.capacity = 1;
    for size in [0, i64::MAX as usize + 1] {
        boundary.max_message_bytes = size;
        assert_eq!(boundary.validate(), Err(SetupError::InvalidConfig));
    }
    for invalid in ["", "a b", "/", "a//b"] {
        for (from, to) in [(invalid, "ok"), ("ok", invalid)] {
            let mut edited = options.clone();
            edited.topic_remaps.insert(from.into(), to.into());
            let saved = edited.clone();
            assert_eq!(edited.validate(), Err(SetupError::InvalidName));
            assert_eq!(edited, saved);
        }
    }
    let mut edited = ServerOptions {
        capacity: 1,
        ..Default::default()
    };
    edited
        .topic_remaps
        .extend([("a".into(), "b".into()), ("c".into(), "d".into())]);
    assert_eq!(edited.validate(), Err(SetupError::InvalidConfig));
    for (partition, namespace) in [("a b", ""), ("", "a~b"), (&"p".repeat(65535), "")] {
        let edited = ServerOptions {
            partition: partition.into(),
            namespace: namespace.into(),
            ..Default::default()
        };
        assert_eq!(edited.validate(), Err(SetupError::InvalidName));
    }
}

#[cfg(unix)]
#[test]
fn identity_precedence_failures_and_encoding_are_explicit() {
    use nix::errno::Errno;
    use std::os::unix::ffi::OsStringExt;
    for user in ["user", "", "user\u{fffd}"] {
        assert_eq!(
            partition_identity(Ok("host".into()), Some(user.into()), || panic!(
                "USER bypasses NSS"
            ))
            .unwrap(),
            format!("host:{user}")
        );
    }
    assert_eq!(
        partition_identity(Ok("host".into()), None, || Ok(Some("account".into()))).unwrap(),
        "host:account"
    );
    let invalid = OsString::from_vec(vec![255]);
    for result in [
        partition_identity(Err(Errno::EIO), None, || {
            panic!("hostname error stops lookup")
        }),
        partition_identity(Ok("host".into()), None, || Err(Errno::EIO)),
        partition_identity(Ok("host".into()), None, || Ok(None)),
        partition_identity(Ok(invalid.clone()), None, || {
            panic!("invalid hostname stops lookup")
        }),
        partition_identity(Ok("host".into()), Some(invalid.clone()), || unreachable!()),
        partition_identity(Ok("host".into()), None, || {
            Ok(Some("account\u{fffd}".into()))
        }),
    ] {
        assert!(matches!(result, Err(SetupError::Identity(_))), "{result:?}");
    }
    assert!(matches!(
        captured_options(Some(invalid), || unreachable!()),
        Err(SetupError::Identity(_))
    ));
}

#[test]
fn explicit_partition_bypasses_identity_and_is_validated() {
    for partition in ["", "explicit", "/partition/"] {
        assert_eq!(
            captured_options(Some(partition.into()), || panic!(
                "explicit partition bypasses identity"
            ))
            .unwrap()
            .partition,
            partition
        );
    }
    assert_eq!(
        captured_options(Some("invalid partition".into()), || unreachable!()),
        Err(SetupError::InvalidName)
    );
    assert_eq!(
        captured_options(None, || Err(SetupError::Identity("lookup failed".into()))),
        Err(SetupError::Identity("lookup failed".into()))
    );
}

#[cfg(unix)]
#[test]
fn environment_capture_uses_only_partition_and_native_identity() {
    use nix::unistd::{User, gethostname, getuid};
    let host = gethostname().unwrap().into_string().unwrap();
    let account = User::from_uid(getuid()).unwrap().unwrap().name;
    for (partition, user, expected) in [
        (Some("explicit"), Some("unused"), "explicit".into()),
        (Some(""), None, "".into()),
        (None, Some("chosen"), format!("{host}:chosen")),
        (None, Some(""), format!("{host}:")),
        (None, None, format!("{host}:{account}")),
    ] {
        let mut command = std::process::Command::new(std::env::current_exe().unwrap());
        command
            .args([
                "options::tests::environment_child",
                "--exact",
                "--nocapture",
            ])
            .env("GZ_SIM_OPTIONS_CHILD_EXPECT", expected)
            .env("GZ_IP", "invalid")
            .env("GZ_DISCOVERY_MSG_PORT", "-1")
            .env("GZ_TRANSPORT_IMPLEMENTATION", "discarded")
            .env("GZ_TRANSPORT_SNDHWM", "0")
            .env("GZ_TRANSPORT_RCVHWM", "0")
            .env("GZ_RELAY", "invalid")
            .env("GZ_TRANSPORT_USERNAME", "unused")
            .env("GZ_TRANSPORT_PASSWORD", "unused")
            .env_remove("GZ_PARTITION")
            .env_remove("USER");
        if let Some(partition) = partition {
            command.env("GZ_PARTITION", partition);
        }
        if let Some(user) = user {
            command.env("USER", user);
        }
        let output = command.output().unwrap();
        assert!(
            output.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}

#[test]
fn environment_child() {
    let Ok(expected) = std::env::var("GZ_SIM_OPTIONS_CHILD_EXPECT") else {
        return;
    };
    assert_eq!(
        ServerOptions::from_env().unwrap(),
        ServerOptions {
            partition: expected,
            ..Default::default()
        }
    );
}

#[test]
fn setup_errors_explain_application_failures() {
    use std::error::Error;
    for error in [
        SetupError::InvalidName,
        SetupError::InvalidConfig,
        SetupError::AlreadyRemapped,
        SetupError::QueueFull,
        SetupError::Timeout,
        SetupError::Identity("hostname unavailable".into()),
    ] {
        assert!(!error.to_string().is_empty());
        assert!(error.source().is_none());
    }
    assert!(
        SetupError::Identity("hostname unavailable".into())
            .to_string()
            .contains("hostname unavailable")
    );
}

#[test]
fn public_topic_boundary_normalizes_separately_and_remaps_once() {
    let mut options = ServerOptions {
        partition: "part".into(),
        namespace: "ns".into(),
        ..Default::default()
    };
    assert_eq!(as_valid_topic("image with spaces"), "image_with_spaces");
    assert_eq!(
        options
            .topic_key(&as_valid_topic("image with spaces"))
            .unwrap(),
        "@/part@/ns/image_with_spaces"
    );
    assert_eq!(
        options.topic_key("/absolute/image").unwrap(),
        "@/part@/absolute/image"
    );
    options
        .add_topic_remap("/source/image", "relative/image")
        .unwrap();
    options
        .add_topic_remap("relative/image", "/must-not-chain")
        .unwrap();
    assert_eq!(
        options.topic_key("/source/image").unwrap(),
        "@/part@/ns/relative/image"
    );
    assert_eq!(
        options.topic_key("relative/image").unwrap(),
        "@/part@/must-not-chain"
    );
    for invalid in ["", "@", "bad name", "/", "a//b"] {
        assert!(
            matches!(options.topic_key(invalid), Err(SetupError::InvalidName)),
            "{invalid}"
        );
    }
}
