// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Policy, Receipt, Result};
use plotters::prelude::*;
use std::path::Path;

/// Native receipt order is discrete along the rows. The clock column stays at the measured time.
pub(super) fn draw(running: &Policy, paused: &Policy, path: &Path) -> Result<()> {
    let mut rows: Vec<(&str, Option<&Receipt>, &Receipt)> = Vec::new();
    for b in &paused.boundary {
        let text = match b.label.as_str() {
            "before_edit" => "Before removing B",
            "after_edit" => "After removing B",
            "paused_tick_1" => "Paused tick 1",
            "paused_tick_2" => "Paused tick 2",
            "paused_tick_3" => "Paused tick 3",
            _ => return Err("unknown paused receipt".into()),
        };
        rows.push((
            text,
            running.boundary.iter().find(|a| a.label == b.label),
            b,
        ));
    }
    for (label, key) in [
        ("Before policy check", "before_prepare"),
        ("After check, before solve", "prepare"),
        ("First completed solve", "commit"),
    ] {
        rows.push((label, Some(running.get(key)?), paused.get(key)?));
    }
    let canvas = SVGBackend::new(path, (1220, 562)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let ink = RGBColor(40, 50, 61);
    let font = |n| ("sans-serif", n).into_font().color(&ink);
    canvas.draw(&Text::new(
        "Automatic promotion when a supporting pad is removed",
        (24, 15),
        font(26),
    ))?;
    canvas.draw(&Text::new("Running and paused-edit observations on the same simulation clock. The rightmost columns show the reported normal impulse on pad A.",(24,53),font(16)))?;
    for (x, s) in [
        (24, "Observation"),
        (284, "Time (ms)"),
        (398, "Pad B"),
        (488, "Running tier"),
        (630, "Paused-edit tier"),
        (815, "Pause latch"),
        (951, "Paused edit"),
        (1080, "Always dynamic"),
    ] {
        canvas.draw(&Text::new(s, (x, 104), font(17)))?;
    }
    for (x, s) in [(951, "A (mN·s)"), (1080, "A (mN·s)")] {
        canvas.draw(&Text::new(s, (x, 130), font(15)))?;
    }
    for (i, (label, active, hold)) in rows.into_iter().enumerate() {
        let y = 170 + 43 * i as i32;
        let color = if hold.tier == "dynamic" {
            RGBColor(7, 107, 97)
        } else {
            ink
        };
        if i % 2 == 0 {
            canvas.draw(&Rectangle::new(
                [(14, y - 8), (1205, y + 28)],
                RGBColor(244, 246, 247).filled(),
            ))?;
        }
        for (x, text) in [
            (24, label.to_owned()),
            (284, format!("{:.3}", hold.time_s * 1000.)),
            (
                398,
                if hold.support_b_present {
                    "present"
                } else {
                    "gone"
                }
                .into(),
            ),
            (488, active.map(|a| a.tier.as_str()).unwrap_or("—").into()),
            (630, hold.tier.clone()),
            (815, if hold.paused { "paused" } else { "running" }.into()),
            (
                951,
                format!("{:.6}", hold.sample.normal_impulses_ns[0] * 1e3),
            ),
            (
                1080,
                format!("{:.6}", hold.reference.normal_impulses_ns[0] * 1e3),
            ),
        ] {
            canvas.draw(&Text::new(
                text,
                (x, y),
                ("sans-serif", 16)
                    .into_font()
                    .color(if x == 488 || x == 630 { &color } else { &ink }),
            ))?;
        }
    }
    canvas.draw(&Text::new("The three paused ticks have zero elapsed simulation time. Their running-tier cells are empty because only the paused lane issued those ticks.",(24,525),font(16)))?;
    canvas.present()?;
    Ok(())
}
