// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Summarize or compare complete 100 Hz observations of the stock drive world.
use drive_native_reference::{HEADER, LAST, STRIDE};
use serde_json::{Value, json};
use std::{error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DT: f64 = 0.01;

#[derive(Clone, Copy)]
struct State {
    chassis: [f64; 2],
    yaw: f64,
    wheel: [f64; 2],
    axle: [f64; 2],
}

fn load(path: &Path) -> Result<Vec<State>> {
    let csv = fs::read_to_string(path)?;
    let mut lines = csv.lines();
    if lines.next() != Some(HEADER) {
        return Err("unexpected drive CSV header".into());
    }
    let mut samples = Vec::new();
    for (index, line) in lines.enumerate() {
        let mut fields = line.split(',');
        let tick: u64 = fields.next().ok_or("missing tick")?.parse()?;
        let numbers = fields
            .map(str::parse::<f64>)
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if tick != index as u64 * STRIDE
            || numbers.len() != 13
            || !numbers.iter().all(|x| x.is_finite())
        {
            return Err(format!(
                "malformed sample in {} at row {}",
                path.display(),
                index + 2
            )
            .into());
        }
        samples.push(State {
            chassis: [numbers[0], numbers[1]],
            yaw: numbers[5],
            wheel: [numbers[6], numbers[7]],
            axle: [numbers[10], numbers[11]],
        });
    }
    if samples.len() != (LAST / STRIDE + 1) as usize {
        return Err("incomplete trace".into());
    }
    Ok(samples)
}

fn delta(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [b[0] - a[0], b[1] - a[1]]
}
fn norm(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}
fn wrapped(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

struct Motion {
    distance: f64,
    lateral_speed: f64,
    yaw_rate: f64,
}
fn motion(a: State, b: State) -> Motion {
    let yaw_delta = wrapped(b.yaw - a.yaw);
    let (s, c) = (a.yaw + 0.5 * yaw_delta).sin_cos();
    let [dx, dy] = delta(a.axle, b.axle);
    Motion {
        distance: dx.hypot(dy),
        lateral_speed: (-s * dx + c * dy) / DT,
        yaw_rate: yaw_delta / DT,
    }
}
fn distance(samples: &[State]) -> f64 {
    samples
        .windows(2)
        .map(|pair| motion(pair[0], pair[1]).distance)
        .sum()
}

fn phase(samples: &[State]) -> Value {
    let start = samples[0];
    let end = samples[samples.len() - 1];
    let mut lateral_max = 0.0_f64;
    let mut lateral_path = 0.0;
    for pair in samples.windows(2) {
        let speed = motion(pair[0], pair[1]).lateral_speed.abs();
        lateral_max = lateral_max.max(speed);
        lateral_path += speed * DT;
    }
    json!({
        "axle_delta_world_xy_m":delta(start.axle,end.axle),
        "chassis_delta_world_xy_m":delta(start.chassis,end.chassis),
        "axle_path_m":distance(samples),
        "yaw_change_rad":wrapped(end.yaw-start.yaw),
        "wheel_angle_change_rad":delta(start.wheel,end.wheel),
        "peak_abs_axle_lateral_speed_m_s":lateral_max,
        "integrated_abs_axle_lateral_motion_m":lateral_path,
    })
}

fn report(samples: &[State]) -> Value {
    let braking = &samples[450..];
    // The first 10 ms interval in the final sustained run below both speed thresholds.
    let last_moving = braking.windows(2).rposition(|pair| {
        let m = motion(pair[0], pair[1]);
        m.distance / DT >= 0.01 || m.yaw_rate.abs() >= 0.02
    });
    let settle_start = last_moving.map_or(0, |i| i + 1);
    let stopped_after = (settle_start < braking.len() - 1).then_some(settle_start as f64 * DT);
    let final_state = samples[samples.len() - 1];
    json!({
        "samples":samples.len(),
        "settle":phase(&samples[..=50]),
        "straight":phase(&samples[50..=250]),
        "left_arc":phase(&samples[250..=450]),
        "brake":phase(braking),
        "stopped_after_brake_command_s":stopped_after,
        "final_chassis_xy_m":final_state.chassis,
        "final_chassis_yaw_rad":final_state.yaw,
        "final_wheel_angles_rad":final_state.wheel
    })
}

fn compare(reference: &[State], candidate: &[State]) -> Value {
    let mut xy = 0.0_f64;
    let mut yaw = 0.0_f64;
    let mut left = 0.0_f64;
    let mut right = 0.0_f64;
    let mut lateral = 0.0_f64;
    for (a, b) in reference.iter().zip(candidate) {
        xy = xy.max(norm(delta(a.chassis, b.chassis)));
        yaw = yaw.max(wrapped(a.yaw - b.yaw).abs());
        let wheel = delta(a.wheel, b.wheel);
        left = left.max(wheel[0].abs());
        right = right.max(wheel[1].abs());
    }
    for (a, b) in reference.windows(2).zip(candidate.windows(2)) {
        lateral = lateral
            .max((motion(a[0], a[1]).lateral_speed - motion(b[0], b[1]).lateral_speed).abs());
    }
    let brake = (distance(&reference[450..]) - distance(&candidate[450..])).abs();
    let mut passed = true;
    let mut metrics = serde_json::Map::new();
    for (name, error, limit) in [
        ("max_chassis_xy_error_m", xy, 0.10),
        ("max_chassis_heading_error_rad", yaw, 0.10),
        ("max_left_wheel_angle_error_rad", left, 0.15),
        ("max_right_wheel_angle_error_rad", right, 0.15),
        ("braking_axle_path_error_m", brake, 0.03),
        ("max_axle_lateral_speed_error_m_s", lateral, 0.05),
    ] {
        passed &= error <= limit;
        metrics.insert(
            name.to_owned(),
            json!({"error":error,"limit":limit,"pass":error<=limit}),
        );
    }
    json!({"pass":passed,"metrics":metrics})
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if !(2..=3).contains(&args.len()) {
        return Err("usage: metrics NATIVE.csv OUTPUT.json [CANDIDATE.csv]".into());
    }
    let native = load(Path::new(&args[0]))?;
    let mut result = json!({"native":report(&native)});
    let mut passed = true;
    if let Some(path) = args.get(2) {
        let candidate = load(Path::new(path))?;
        let comparison = compare(&native, &candidate);
        passed = comparison["pass"] == true;
        result["candidate"] = report(&candidate);
        result["comparison"] = comparison;
    }
    fs::write(
        &args[1],
        format!("{}\n", serde_json::to_string_pretty(&result)?),
    )?;
    if !passed {
        return Err("drive comparison exceeded thresholds; see output report".into());
    }
    Ok(())
}
