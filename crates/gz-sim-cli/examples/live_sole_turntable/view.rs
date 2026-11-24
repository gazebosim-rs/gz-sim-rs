// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeSet, time::Duration};
pub(super) fn write(dir: &Path, runs: &[Run]) -> Result<()> {
    let output = dir.join("viewer");
    fs::create_dir(&output)?;
    let mut ticks = (1..=END).step_by(5).collect::<BTreeSet<_>>();
    let mut events = serde_json::Map::new();
    for (name, t) in [
        ("applied_torque", PULSE_BEGIN),
        ("torque_pulse_ends", PULSE_END),
        ("stop_command", STOP + 1),
        ("coasting_foot", END),
    ] {
        ticks.insert(t);
        events.insert(name.to_owned(), json!(t));
    }
    for t in events.values_mut() {
        let n = t.as_u64().unwrap() as usize;
        *t = json!(ticks.iter().position(|v| *v == n).unwrap());
    }
    let max_work = runs
        .iter()
        .flat_map(|r| &r.samples)
        .map(|s| s.cumulative_spin_work_j)
        .fold(0., f64::max);
    let cuboid = |size: [f64; 3]| json!({"kind":"box","size":size});
    let mut archive = Vec::new();
    for (seq, &tick) in ticks.iter().enumerate() {
        let mut frame = soft_replay::Frame::default();
        for (lane, run) in runs.iter().enumerate() {
            let s = &run.samples[tick - 1];
            let lane_x = 0.81 * (lane as f64 - 1.);
            let offset = lane_x * DVec3::X;
            let cyan = [0.15, 0.84, 0.77, 1.];
            let gold = [0.98, 0.69, 0.17, 1.];
            let silver = [0.75, 0.82, 0.92, 1.];
            let q = DQuat::from_array(s.table_orientation_xyzw);
            let sole = DQuat::from_array(s.orientation_xyzw);
            frame.add(
                &format!("{} driven table", run.name),
                offset - 0.025 * DVec3::Z,
                q,
                cuboid([0.58, 0.42, 0.05]),
                [0.15, 0.3, 0.4, 1.],
            );
            frame.add(
                &format!("{} table direction", run.name),
                offset + q * DVec3::new(0.225, 0., 0.003),
                q,
                cuboid([0.09, 0.012, 0.006]),
                cyan,
            );
            frame.add(
                &format!("{} foot", run.name),
                offset + DVec3::from(s.center_m),
                sole,
                cuboid([0.24, 0.12, 0.12]),
                silver,
            );
            frame.add(
                &format!("{} foot direction", run.name),
                offset + DVec3::from(s.center_m) + sole * DVec3::new(0.075, 0., 0.063),
                sole,
                cuboid([0.07, 0.01, 0.006]),
                gold,
            );
            let center = offset + DVec3::new(0., -0.34, -0.02);
            frame.edge(
                &format!("{} work scale", run.name),
                center - 0.20 * DVec3::X,
                center + 0.20 * DVec3::X,
                0.40,
                0.006,
                [0.55, 0.61, 0.69, 1.],
            );
            frame.add(
                &format!("{} rotational work", run.name),
                center + (s.cumulative_spin_work_j / max_work - 0.5) * 0.4 * DVec3::X,
                DQuat::IDENTITY,
                cuboid([0.013, 0.033, 0.01]),
                cyan,
            );
        }
        archive.push(frame.finish(Duration::from_micros(500 * tick as u64), seq + 1));
    }
    gz_sim_cli::export_archive_at(
        &output.join("replay"),
        Default::default(),
        0,
        Duration::from_micros(500),
        archive,
    )?;
    fs::write(
        output.join("frames.json"),
        serde_json::to_vec_pretty(
            &json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"top","sole_turntable":true,
      "origin_ns":500000,"captures":events}),
        )?,
    )?;
    Ok(())
}
