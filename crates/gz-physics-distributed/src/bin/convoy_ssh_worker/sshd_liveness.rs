// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check a supplied OpenSSH effective configuration for a prospective worker login.
use super::{MAX_CONFIG, invalid};
use serde::Serialize;
use std::{
    ffi::OsString,
    io::{self, Read, Write},
    num::NonZeroU64,
};

pub(super) const FLAG: &str = "--check-sshd-liveness";

#[derive(Serialize)]
struct Policy {
    client_alive_interval_seconds: u32,
    client_alive_count_max: u32,
    estimated_idle_detection_seconds: u64,
    budget_seconds: NonZeroU64,
}

fn policy(input: impl Read, budget_seconds: NonZeroU64) -> io::Result<Policy> {
    let mut text = String::new();
    input.take(MAX_CONFIG + 1).read_to_string(&mut text)?;
    if text.len() as u64 > MAX_CONFIG {
        return Err(invalid("effective sshd configuration exceeds 64 KiB"));
    }
    let (mut interval, mut count) = (None, None);
    for line in text.lines() {
        let mut fields = line.split_ascii_whitespace();
        let setting = fields.next().unwrap_or_default();
        if setting.eq_ignore_ascii_case("match") || setting.eq_ignore_ascii_case("include") {
            return Err(invalid(
                "expected expanded sshd -T output for the worker login",
            ));
        }
        let value = if setting.eq_ignore_ascii_case("clientaliveinterval") {
            &mut interval
        } else if setting.eq_ignore_ascii_case("clientalivecountmax") {
            &mut count
        } else {
            continue;
        };
        if value.is_some() {
            return Err(invalid(format!("duplicate {setting} in sshd -T output")));
        }
        let number = fields
            .next()
            .and_then(|field| field.parse::<i32>().ok())
            .and_then(|n| u32::try_from(n).ok())
            .filter(|_| fields.next().is_none())
            .ok_or_else(|| invalid(format!("invalid {setting} in sshd -T output")))?;
        if number == 0 {
            return Err(invalid(format!("{setting} 0 disables idle peer detection")));
        }
        *value = Some(number);
    }
    let client_alive_interval_seconds =
        interval.ok_or_else(|| invalid("missing clientaliveinterval in sshd -T output"))?;
    let client_alive_count_max =
        count.ok_or_else(|| invalid("missing clientalivecountmax in sshd -T output"))?;
    // Allow a full interval before the first probe and one for each missed reply.
    // Active traffic or a delayed sshd event loop can postpone the check.
    let estimated_idle_detection_seconds =
        u64::from(client_alive_interval_seconds) * (1 + u64::from(client_alive_count_max));
    if estimated_idle_detection_seconds > budget_seconds.get() {
        return Err(invalid(format!(
            "estimated idle detection of {estimated_idle_detection_seconds}s exceeds {budget_seconds}s budget"
        )));
    }
    Ok(Policy {
        client_alive_interval_seconds,
        client_alive_count_max,
        estimated_idle_detection_seconds,
        budget_seconds,
    })
}

pub(super) fn check(args: &[OsString], input: impl Read, mut output: impl Write) -> io::Result<()> {
    let [budget] = args else {
        return Err(invalid("expected --check-sshd-liveness BUDGET_SECONDS"));
    };
    let budget = budget
        .to_str()
        .and_then(|s| s.parse::<NonZeroU64>().ok())
        .ok_or_else(|| invalid("expected a positive whole-second liveness budget"))?;
    serde_json::to_writer(&mut output, &policy(input, budget)?)?;
    writeln!(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn audit(input: &[u8], budget: &str) -> io::Result<serde_json::Value> {
        let mut output = Vec::new();
        check(&[budget.into()], input, &mut output)?;
        Ok(serde_json::from_slice(&output)?)
    }

    #[test]
    fn compares_idle_probe_schedule_to_requested_budget() {
        let input =
            b"port 2222\r\nclientalivecountmax 2\nallowusers ubuntu\nclientaliveinterval\t1\n";
        let result = audit(input, "3").unwrap();
        assert_eq!(result["budget_seconds"], 3);
        assert_eq!(result["client_alive_interval_seconds"], 1);
        assert_eq!(result["client_alive_count_max"], 2);
        assert_eq!(result["estimated_idle_detection_seconds"], 3);
        assert!(
            audit(input, "2")
                .unwrap_err()
                .to_string()
                .contains("exceeds")
        );
        let large = b"clientaliveinterval 2147483647\nclientalivecountmax 2147483647\n";
        assert_eq!(
            audit(large, "4611686018427387904").unwrap()["estimated_idle_detection_seconds"],
            4611686016279904256u64
        );
    }

    #[test]
    fn rejects_incomplete_ambiguous_or_disabled_effective_settings() {
        for (input, reason) in [
            ("clientaliveinterval 1", "missing clientalivecountmax"),
            ("clientalivecountmax 1", "missing clientaliveinterval"),
            ("clientaliveinterval 0\nclientalivecountmax 2", "disables"),
            ("clientalivecountmax 0\nclientaliveinterval 2", "disables"),
            ("clientaliveinterval 1\nClientAliveInterval 1", "duplicate"),
            (
                "Match User ubuntu\nclientaliveinterval 1\nclientalivecountmax 2",
                "expanded",
            ),
            ("Include /etc/ssh/sshd_config.d/*.conf", "expanded"),
            ("clientalivecountmax 1\nclientalivecountmax 2", "duplicate"),
            ("clientaliveinterval -1", "invalid"),
            ("clientaliveinterval 2s", "invalid"),
            ("clientalivecountmax 2 extra", "invalid"),
            ("clientalivecountmax 2147483648", "invalid"),
            ("clientalivecountmax", "invalid"),
        ] {
            assert!(
                audit(input.as_bytes(), "3")
                    .unwrap_err()
                    .to_string()
                    .contains(reason),
                "{input}"
            );
        }
    }

    #[test]
    fn bounds_input_before_processing_the_settings() {
        let mut input = b"clientaliveinterval 1\nclientalivecountmax 2\n".to_vec();
        input.resize(MAX_CONFIG as usize, b' ');
        assert!(audit(&input, "3").is_ok());
        input.push(b' ');
        assert!(
            audit(&input, "3")
                .unwrap_err()
                .to_string()
                .contains("64 KiB")
        );
        assert!(audit(&[255], "3").is_err());
    }

    #[test]
    fn refuses_missing_or_invalid_budget_before_reading() {
        let mut output = Vec::new();
        for args in [
            vec![],
            vec!["1".into(), "2".into()],
            vec!["-1".into()],
            vec!["0".into()],
        ] {
            assert!(check(&args, &b""[..], &mut output).is_err());
        }
        assert!(output.is_empty());
    }
}
