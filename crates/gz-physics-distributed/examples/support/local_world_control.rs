// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::Value;

pub fn physics_and_layout(row: &Value) -> Result<Value, Box<dyn std::error::Error>> {
    let mut fields = row.as_object().ok_or("expected benchmark object")?.clone();
    for key in ["elapsed_ms", "tick_p50_us", "tick_p95_us"] {
        let v = fields
            .remove(key)
            .and_then(|v| v.as_f64())
            .ok_or("missing timing")?;
        if !v.is_finite() || v <= 0. {
            return Err("invalid timing".into());
        }
    }
    for key in ["tick_p99_us", "round", "bound"] {
        fields.remove(key);
    }
    Ok(fields.into())
}
