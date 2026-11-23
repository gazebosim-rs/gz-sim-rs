// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! SVGs from the same measured records as the replay and regression checks.
use super::*;
use std::fmt::Write as _;

struct Figure(String);

impl Figure {
    fn new(title: &str, subtitle: &str, height: u32) -> Self {
        let mut image = Self(format!(
            r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 {height}" width="1000" height="{height}"><title>{title}</title><style>text{{font:16px system-ui,sans-serif;fill:#183643}}.title{{font-size:28px;font-weight:650}}.head{{font-weight:650}}.small{{font-size:14px}}</style><rect width="100%" height="100%" fill="#fbfaf7"/>"##
        ));
        image.label(28, 44, "title", title);
        image.label(28, 75, "small", subtitle);
        image
    }
    fn label(&mut self, x: u32, y: u32, class: &str, text: &str) {
        let safe = text.replace('&', "&amp;").replace('<', "&lt;");
        write!(
            self.0,
            r#"<text x="{x}" y="{y}" class="{class}">{safe}</text>"#
        )
        .unwrap();
    }
    fn dot(&mut self, x: f64, y: u32, color: &str) {
        write!(
            self.0,
            r#"<circle cx="{x:.2}" cy="{y}" r="4" fill="{color}"/>"#
        )
        .unwrap();
    }
    fn line(&mut self, x: u32, end: u32, y: u32) {
        write!(self.0, r##"<path d="M{x} {y} H{end}" stroke="#cbd5d4"/>"##).unwrap();
    }
    fn save(mut self, file: &Path) -> Result<()> {
        self.0.push_str("</svg>\n");
        fs::write(file, self.0)?;
        Ok(())
    }
}
fn value(row: &Value, key: &str) -> f64 {
    row[key].as_f64().expect("numeric measurement")
}
fn nominal(r: &Value) -> bool {
    r["cycle"] == 3 && r["bottom_dwell_shift_us"] == 0
}

fn numerics(out: &Path, rows: &[Value]) -> Result<()> {
    let mut f = Figure::new(
        "Touchdown impulse by solver setting",
        "Nominal third approach. Table in N·s for the always-dynamic foot. Dots show 5 ms impulse for both histories.",
        705,
    );
    for (x, s) in [
        (28, "Solver setting"),
        (258, "First tick"),
        (350, "5 ms"),
        (446, "Stroke"),
        (552, "5 ms impulse  ·  N·s"),
    ] {
        f.label(x, 124, "head", s);
    }
    let lo = 2.8;
    let scale = 440.;
    for tick in [2.8, 3.0, 3.2, 3.4, 3.6] {
        f.label(
            (555. + (tick - lo) * scale) as u32,
            150,
            "small",
            &format!("{tick:.1}"),
        );
    }
    for (i, r) in rows
        .iter()
        .filter(|r| nominal(r) && r["history"] == "reference")
        .enumerate()
    {
        let y = 179 + 28 * i as u32;
        let other = rows
            .iter()
            .find(|s| nominal(s) && s["history"] == "switched" && s["policy"] == r["policy"])
            .unwrap();
        f.label(28, y, "", r["policy"].as_str().unwrap());
        for (x, k) in [
            (258, "first_outer_ns"),
            (350, "window_ns"),
            (446, "stroke_ns"),
        ] {
            f.label(x, y, "", &format!("{:.4}", value(r, k)));
        }
        f.line(562, 950, y - 5);
        for (s, dy, color) in [(r, 8, "#006f72"), (other, 1, "#b34520")] {
            f.dot(563. + (value(s, "window_ns") - lo) * scale, y - dy, color);
        }
    }
    f.label(28,674,"small","Teal · always dynamic     Orange · switched     First tick and 5 ms use the same outer-clock boundaries across settings.");
    f.save(&out.join("numerics.svg"))
}

fn gating(out: &Path, rows: &[Value]) -> Result<()> {
    let mut f = Figure::new(
        "When does the extra solver work matter?",
        "Timing and duration use the third always-dynamic approach. The prospective trigger uses only pre-solve body states.",
        896,
    );
    let offsets = [
        "coarse",
        "early_10",
        "late_9",
        "first_only",
        "first_two",
        "fixed_10",
        "long_20",
    ];
    for (x, s) in [
        (28, "Fixed window"),
        (250, "First tick · N·s"),
        (465, "5 ms · N·s"),
        (680, "Stroke · N·s"),
    ] {
        f.label(x, 118, "head", s);
    }
    for (i, name) in offsets.iter().enumerate() {
        let r = rows
            .iter()
            .find(|r| nominal(r) && r["history"] == "reference" && r["policy"] == *name)
            .unwrap();
        let y = 146 + 24 * i as u32;
        f.label(28, y, "", name);
        for (x, k) in [
            (250, "first_outer_ns"),
            (465, "window_ns"),
            (680, "stroke_ns"),
        ] {
            f.label(x, y, "", &format!("{:.5}", value(r, k)));
        }
    }
    f.label(
        28,
        348,
        "head",
        "Prospective two-tick, 16-iteration policy minus four substeps for ten ticks",
    );
    f.label(28,375,"small","Δt in outer ticks against measured coarse onset. Signed impulse differences in mN·s (1 mN·s = 0.001 N·s).");
    for (x, s) in [
        (28, "History"),
        (170, "Stroke"),
        (255, "Dwell shift · µs"),
        (422, "Δt"),
        (510, "First tick"),
        (660, "5 ms"),
        (825, "Stroke total"),
    ] {
        f.label(x, 416, "head", s);
    }
    for (i, r) in rows
        .iter()
        .filter(|r| r["policy"] == "gate_2_iter16")
        .enumerate()
    {
        let same = |s: &&Value| {
            ["cycle", "history", "bottom_dwell_shift_us"]
                .iter()
                .all(|k| s[k] == r[k])
                && s["policy"] == "fixed_10"
        };
        let base = rows.iter().find(same).unwrap();
        let y = 445 + 25 * i as u32;
        f.label(28, y, "", r["history"].as_str().unwrap());
        for (x, k) in [(170, "cycle"), (255, "bottom_dwell_shift_us")] {
            f.label(x, y, "", &r[k].to_string());
        }
        f.label(
            422,
            y,
            "",
            &(r["trigger_ticks"][0].as_i64().unwrap() - r["coarse_onset_tick"].as_i64().unwrap())
                .to_string(),
        );
        for (x, k) in [
            (510, "first_outer_ns"),
            (660, "window_ns"),
            (825, "stroke_ns"),
        ] {
            f.label(
                x,
                y,
                "",
                &format!("{:+.3}", 1000. * (value(r, k) - value(base, k))),
            );
        }
    }
    f.label(28, 735, "head", "Continuous run · 1 mm clearance to rearm");
    let continuous: Vec<Value> = serde_json::from_slice(&fs::read(out.join("continuous.json"))?)?;
    for (i, r) in continuous
        .iter()
        .filter(|r| r["policy"] == "gate_2_iter16")
        .enumerate()
    {
        let ticks = r["trigger_events"]
            .as_array()
            .unwrap()
            .iter()
            .map(|e| e["tick"].to_string())
            .collect::<Vec<_>>()
            .join(", ");
        f.label(
            28,
            772 + 35 * i as u32,
            "",
            &format!(
                "{}  ·  triggered at ticks {}  ·  {} selected of {}  ·  {} selected while parked",
                r["history"].as_str().unwrap(),
                ticks,
                r["selected_ticks"],
                r["outer_steps"],
                r["selected_parked_ticks"]
            ),
        );
    }
    f.label(28,855,"small","Each held-out trial starts from the untreated history. The continuous runs carry changes in trajectory into later approaches.");
    f.save(&out.join("gate.svg"))
}

fn latency(out: &Path, rows: &[Value]) -> Result<()> {
    let mut f = Figure::new(
        "Full Bevy step cost around touchdown",
        "Nearest-rank p95 in µs per outer tick. Each row pools 30 runs in six interleaved blocks. Totals are mean µs over ten ticks.",
        650,
    );
    for (x, s) in [
        (28, "History"),
        (153, "Solver setting"),
        (361, "First two"),
        (465, "Ten ticks"),
        (575, "Full run"),
        (680, "Total"),
        (771, "Ten-tick p95"),
    ] {
        f.label(x, 125, "head", s);
    }
    let hi = rows
        .iter()
        .map(|r| value(r, "p95_window_us"))
        .fold(0., f64::max)
        .ceil();
    for (i, r) in rows.iter().enumerate() {
        let y = 158 + 29 * i as u32 + u32::from(i >= 7) * 14;
        f.label(28, y, "", r["history"].as_str().unwrap());
        f.label(153, y, "", r["policy"].as_str().unwrap());
        for (x, k) in [
            (361, "p95_first_two_us"),
            (465, "p95_window_us"),
            (575, "p95_continuation_us"),
            (680, "mean_window_total_us"),
        ] {
            f.label(x, y, "", &format!("{:.2}", value(r, k)));
        }
        f.line(778, 957, y - 5);
        f.dot(
            778. + 179. * value(r, "p95_window_us") / hi,
            y - 5,
            if r["history"] == "reference" {
                "#006f72"
            } else {
                "#b34520"
            },
        );
    }
    f.label(28,603,"small",&format!("Dot scale 0–{hi:.0} µs. Stock uses the ordinary scene stages. The other settings include native impulse readback."));
    f.save(&out.join("latency.svg"))
}

pub(super) fn draw(out: &Path, cases: &Value, timings: &Value) -> Result<()> {
    let rows = cases["runs"].as_array().expect("control runs");
    numerics(out, rows)?;
    gating(out, rows)?;
    latency(out, timings["rows"].as_array().expect("timing rows"))
}
