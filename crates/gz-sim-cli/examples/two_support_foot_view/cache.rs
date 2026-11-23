// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use glam::DVec3;
use plotters::prelude::*;
use std::path::Path;
#[derive(serde::Deserialize)]
struct Seed {
    points: u32,
    normal_ns: f64,
    tangent_on_foot_ns: [f64; 3],
}

pub(super) fn draw(summary: &serde_json::Value, path: &Path) -> Result<()> {
    use serde::Deserialize;
    let stages = [
        ("Before demotion", "/demotion/switched_before"),
        ("After demotion", "/demotion/switched_after"),
        ("Before B removal", "/loss/switched_before"),
        ("After B removal", "/loss/switched_after"),
        ("After promotion, before solve", "/promotion/switched_after"),
        (
            "After first resumed step",
            "/promotion/first_resumed/switched_cache",
        ),
    ];
    let mut rows = Vec::new();
    let mut peak = 0.0_f64;
    for (label, key) in stages {
        let v = summary
            .pointer(key)
            .ok_or_else(|| format!("no cache measurement at {key}"))?;
        let seeds: [Option<Seed>; 2] = Deserialize::deserialize(v)?;
        for seed in seeds.iter().flatten() {
            peak = peak
                .max(seed.normal_ns)
                .max(DVec3::from(seed.tangent_on_foot_ns).length());
        }
        rows.push((label, seeds));
    }
    if peak <= 0. {
        return Err("no initially loaded ground cache".into());
    }
    let root = SVGBackend::new(path, (1010, 450)).into_drawing_area();
    root.fill(&WHITE)?;
    let ink = RGBColor(41, 52, 65);
    let font = |size| ("sans-serif", size).into_font().color(&ink);
    root.draw(&Text::new(
        "Stored contact impulses on the switched foot",
        (24, 16),
        font(25),
    ))?;
    root.draw(&Text::new(
        "Normal impulse and magnitude of net world-frame tangential impulse (mN·s)",
        (24, 55),
        font(16),
    ))?;
    for (c, name) in ["Support A  ·  surviving", "Support B  ·  removed"]
        .into_iter()
        .enumerate()
    {
        root.draw(&Text::new(name, (310 + 350 * c as i32, 100), font(19)))?;
    }
    let shades = [RGBColor(165, 109, 19), RGBColor(130, 76, 154)];
    for (r, (label, seeds)) in rows.into_iter().enumerate() {
        let y = 143 + r as i32 * 49;
        root.draw(&Text::new(label, (24, y), font(15)))?;
        for (c, seed) in seeds.into_iter().enumerate() {
            let x = 310 + 350 * c as i32;
            if let Some(v) = seed {
                let tang = DVec3::from(v.tangent_on_foot_ns).length();
                root.draw(&Text::new(
                    format!(
                        "{} pts     {:.5} normal    {:.5} tangent",
                        v.points,
                        1000. * v.normal_ns,
                        1000. * tang
                    ),
                    (x, y),
                    font(14),
                ))?;
                for (offset, value) in [(0, v.normal_ns), (155, tang)] {
                    root.draw(&Rectangle::new(
                        [(x + offset, y + 22), (x + offset + 125, y + 26)],
                        RGBColor(225, 230, 233).filled(),
                    ))?;
                    let len = (125. * value / peak).round() as i32;
                    if len > 0 {
                        root.draw(&Rectangle::new(
                            [(x + offset, y + 22), (x + offset + len, y + 26)],
                            shades[c].filled(),
                        ))?;
                    }
                }
            } else {
                root.draw(&Text::new("Removed", (x, y), font(15)))?;
            }
        }
    }
    root.present()?;
    Ok(())
}
