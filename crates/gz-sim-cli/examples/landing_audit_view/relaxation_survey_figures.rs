// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::primary_survey::{color_limit, facet_panels, ramp_color};
use plotters::prelude::*;

#[derive(Clone, Copy)]
enum Field {
    Effect(Profile, Profile),
    Distance(Profile, Profile),
}
const FIELDS: [Field; 8] = [
    Field::Effect(CAND, FINE),
    Field::Effect(CAND, ALT),
    Field::Effect(Profile::uniform(8, 16), FINE),
    Field::Effect(Profile::onset(8), FINE),
    Field::Distance(SENSITIVITY[0].0, SENSITIVITY[0].1),
    Field::Distance(SENSITIVITY[1].0, SENSITIVITY[1].1),
    Field::Distance(SENSITIVITY[2].0, SENSITIVITY[2].1),
    Field::Distance(ALT, FINE),
];
fn short(profile: Profile) -> String {
    format!(
        "{} × 4, r{}{}",
        profile.calls,
        profile.relaxation,
        if profile.onset_only { " onset" } else { "" }
    )
}
impl Field {
    fn metric(self, c: &Cell, w: usize) -> std::result::Result<f64, Missing> {
        match self {
            Self::Effect(p, r) => c.change(p, r, w),
            Self::Distance(a, b) => c.pair(a, b, w).map(|v| v.com_mm),
        }
    }
    fn signed(self) -> bool {
        matches!(self, Self::Effect(_, _))
    }
    fn heading(self) -> String {
        match self {
            Self::Effect(p, r) => format!(
                "{} vs 8 × 4, r1 · change in COM distance from {}",
                short(p),
                short(r)
            ),
            Self::Distance(a, b) => format!("COM distance between {} and {}", short(a), short(b)),
        }
    }
    fn file(self) -> String {
        match self {
            Self::Effect(p, r) => format!("effect-{}-ref-{}", p.label(), r.label()),
            Self::Distance(a, b) => format!("distance-{}-{}", a.label(), b.label()),
        }
    }
}
const PENDING: RGBColor = RGBColor(217, 219, 223);
const NO_PRIMARY: RGBColor = RGBColor(79, 78, 83);
const LOST: RGBColor = RGBColor(107, 47, 124);
const UNPAIRED: RGBColor = RGBColor(142, 125, 95);
fn map(c: &Census, out: &Path, field: Field, w: usize, limit: f64) -> Result<()> {
    let t = WINDOWS[w] / 2;
    let path = out.join(format!("{}-{t}ms.svg", field.file()));
    let root = SVGBackend::new(&path, (1180, 835)).into_drawing_area();
    root.fill(&WHITE)?;
    let heading = format!("{} at {t} ms", field.heading());
    let (head, body) = root
        .titled(&heading, ("sans-serif", 23))?
        .split_vertically(86);
    let mut positions = BTreeMap::new();
    let mut counts = [0usize; 3];
    for cell in &c.cells {
        let v = field.metric(cell, w);
        counts[match v {
            Ok(_) => 0,
            Err(Missing::Disjoint) => 1,
            Err(Missing::DefaultPending) => 2,
        }] += 1;
        positions.insert(
            (
                cell.site.offset_mm,
                cell.site.drive_percent,
                cell.site.holds,
                cell.site.stroke,
            ),
            v,
        );
    }
    let no_primary = c.completed.len() * 3 - c.cells.len();
    head.draw(&Text::new(
        format!(
            "{} / 1,250 fixtures · {} matched landings at this horizon · {} qualifying donor cuts",
            c.completed.len(),
            counts[0],
            c.cells.len()
        ),
        (35, 9),
        ("sans-serif", 16),
    ))?;
    let x = 35;
    let width = 405;
    for i in 0..width {
        let frac = f64::from(i) / f64::from(width - 1);
        let v = if field.signed() {
            (2. * frac - 1.) * limit
        } else {
            frac * limit
        };
        head.draw(&Rectangle::new(
            [(x + i, 30), (x + i + 1, 42)],
            ramp_color(v, limit, field.signed()).filled(),
        ))?;
    }
    use plotters::style::text_anchor::{HPos, Pos, VPos};
    let labels = if field.signed() {
        [
            format!("≤ −{limit:.3} mm · closer"),
            "0".into(),
            format!("farther · ≥ +{limit:.3} mm"),
        ]
    } else {
        [
            "0 mm".into(),
            "COM distance".into(),
            format!("≥ {limit:.3} mm"),
        ]
    };
    for ((p, anchor), label) in [
        (x, HPos::Left),
        (x + width / 2, HPos::Center),
        (x + width, HPos::Right),
    ]
    .into_iter()
    .zip(labels)
    {
        head.draw(&Text::new(
            label,
            (p, 45),
            ("sans-serif", 13)
                .into_text_style(&head)
                .pos(Pos::new(anchor, VPos::Top)),
        ))?;
    }
    for ((x, y), color, label, n) in [
        (
            (492, 26),
            PENDING,
            "Pending fixture",
            c.scheduled - c.completed.len(),
        ),
        (
            (775, 26),
            NO_PRIMARY,
            "No qualifying donor primary",
            no_primary,
        ),
        ((492, 49), LOST, "Disjoint support footprint", counts[1]),
        ((775, 49), UNPAIRED, "Default comparison pending", counts[2]),
    ] {
        head.draw(&Rectangle::new([(x, y), (x + 14, y + 14)], color.filled()))?;
        head.draw(&Text::new(
            format!("{label} ({n})"),
            (x + 20, y + 12),
            ("sans-serif", 14),
        ))?;
    }
    let done = c
        .completed
        .iter()
        .map(|n| n * 613 % 1250)
        .collect::<BTreeSet<_>>();
    facet_panels(&body, |x, y, hold, stroke| {
        let key = (
            (165 + 5 * x) as u16,
            (76 + 2 * y) as u16,
            hold != 0,
            stroke + 1,
        );
        match positions.get(&key) {
            Some(Ok(v)) => (true, ramp_color(*v, limit, field.signed())),
            Some(Err(Missing::Disjoint)) => (false, LOST),
            Some(Err(Missing::DefaultPending)) => (false, UNPAIRED),
            None if done.contains(&(x * 50 + y * 2 + hold)) => (false, NO_PRIMARY),
            None => (false, PENDING),
        }
    })?;
    root.present()?;
    Ok(())
}
fn historical(c: &Census, out: &Path, w: usize) -> Result<()> {
    let t = WINDOWS[w] / 2;
    let path = out.join(format!("historical-{t}ms.svg"));
    let root = SVGBackend::new(&path, (1280, 1000)).into_drawing_area();
    root.fill(&WHITE)?;
    let limit = color_limit(
        c.cells
            .iter()
            .flat_map(|cell| {
                PROFILES.into_iter().flat_map(move |p| {
                    [FINE, ALT]
                        .into_iter()
                        .filter_map(move |r| cell.pair(p, r, w).ok().map(|v| v.com_mm))
                })
            })
            .collect(),
    );
    let file_title = format!("Historical outer supports · center-of-mass distance at {t} ms");
    let head = root.titled(&file_title, ("sans-serif", 25))?;
    head.draw(&Text::new(format!("Distances in mm · {} matched landings · blue saturates at {limit:.3} mm (pooled p95) · all native solves use four temporal iterations",c.cells.len()),(22,16),("sans-serif",15)))?;
    let panels = head.margin(30, 10, 15, 15).split_evenly((2, 1));
    let mut order = c.cells.iter().collect::<Vec<_>>();
    order.sort_by_key(|c| (c.site.holds, c.site.drive_percent, c.site.stroke));
    for (a, r) in panels.into_iter().zip([FINE, ALT]) {
        a.draw(&Text::new(
            format!("Empirical reference · {}", short(r)),
            (15, 15),
            ("sans-serif", 19),
        ))?;
        let x0 = 176;
        let y0 = 69;
        let width = 87;
        let height = 24;
        for (j, c) in order.iter().enumerate() {
            let s = c.site;
            let x = x0 + j as i32 * width;
            if j == 0
                || (order[j - 1].site.holds, order[j - 1].site.drive_percent)
                    != (s.holds, s.drive_percent)
            {
                a.draw(&Text::new(
                    format!(
                        "{}% · {}",
                        s.drive_percent,
                        if s.holds { "holds" } else { "dynamic" }
                    ),
                    (x + 3, y0 - 30),
                    ("sans-serif", 15),
                ))?;
            }
            a.draw(&Text::new(
                format!("Stroke {}", s.stroke),
                (x + 3, y0 - 11),
                ("sans-serif", 12),
            ))?;
        }
        for (i, p) in PROFILES.into_iter().enumerate() {
            let y = y0 + i as i32 * height;
            a.draw(&Text::new(short(p), (10, y + 15), ("sans-serif", 14)))?;
            for (j, c) in order.iter().enumerate() {
                let x = x0 + j as i32 * width;
                let val = c.pair(p, r, w).ok().map(|p| p.com_mm);
                let color = val.map_or(LOST, |v| ramp_color(v, limit, false));
                let ink = if color.1 < 120 { WHITE } else { BLACK };
                a.draw(&Rectangle::new(
                    [(x, y), (x + width - 1, y + height - 1)],
                    color.filled(),
                ))?;
                let label = val.map_or_else(|| "Disjoint".into(), |v| format!("{v:.3}"));
                a.draw(&Text::new(
                    label,
                    (x + 8, y + 15),
                    ("sans-serif", 13).into_font().color(&ink),
                ))?;
            }
        }
    }
    root.present()?;
    Ok(())
}
fn policy_ranges(c: &Census, out: &Path) -> Result<()> {
    let path = out.join("relaxation-diagnostics.svg");
    let root = SVGBackend::new(&path, (1090, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let root = root.titled(
        "Fixed-time relaxation on frictional primary landings",
        ("sans-serif", 25),
    )?;
    root.draw(&Text::new(
        format!(
            "{} matched native cuts · dot = median · line = median to p95 · n = available landings",
            c.cells.len()
        ),
        (29, 14),
        ("sans-serif", 16),
    ))?;
    let charts = root.margin(30, 5, 6, 5).split_evenly((2, 2));
    let profiles = [
        BASE,
        Profile::uniform(8, 4),
        CAND,
        Profile::uniform(8, 16),
        Profile::onset(8),
        FINE,
    ];
    for (k, title, unit) in [
        (0, "Primary impulse", "Integrated pair normal impulse (N·s)"),
        (
            1,
            "Residual normal velocity on loaded rows",
            "Impulse-weighted |normal velocity| (m/s)",
        ),
        (
            2,
            "5 ms from the matched cut",
            "COM distance from uniform 4096 r8 (mm)",
        ),
        (
            3,
            "13 ms from the matched cut",
            "COM distance from uniform 4096 r8 (mm)",
        ),
    ] {
        let mut summaries = vec![];
        let mut max = 0.0f64;
        for p in profiles {
            let vals = c
                .cells
                .iter()
                .filter_map(|c| match k {
                    0 => c.endpoint(p, 0).ok().map(|s| s.normal_ns),
                    1 => c
                        .endpoint(p, 0)
                        .ok()
                        .and_then(|s| s.impulse_weighted_abs_post_normal_m_s),
                    2 => c.pair(p, FINE, 1).ok().map(|v| v.com_mm),
                    _ => c.pair(p, FINE, 2).ok().map(|v| v.com_mm),
                })
                .collect::<Vec<_>>();
            let s = stats(vals);
            let two = s["p50"].as_f64().zip(s["p95"].as_f64());
            if let Some((_, p95)) = two {
                max = max.max(p95);
            }
            summaries.push((two, s["n"].as_u64().unwrap_or(0)));
        }
        let mut chart = ChartBuilder::on(&charts[k])
            .margin(10)
            .caption(title, ("sans-serif", 18))
            .x_label_area_size(43)
            .y_label_area_size(114)
            .build_cartesian_2d(0.0..max.max(1e-12) * 1.27, -0.5..5.5)?;
        chart
            .configure_mesh()
            .disable_y_mesh()
            .y_labels(6)
            .y_label_formatter(&|y: &f64| {
                let n = y.round() as i32;
                if (0..6).contains(&n) {
                    short(profiles[(5 - n) as usize])
                } else {
                    String::new()
                }
            })
            .x_desc(unit)
            .max_light_lines(1)
            .draw()?;
        for (j, (quart, n)) in summaries.into_iter().enumerate() {
            let y = (5 - j) as f64;
            if let Some((p50, p95)) = quart {
                let color = RGBColor(26, 96, 105);
                chart.draw_series([PathElement::new(
                    vec![(p50, y), (p95, y)],
                    color.stroke_width(3),
                )])?;
                chart.draw_series([Circle::new((p50, y), 4, color.filled())])?;
                chart.draw_series([Text::new(
                    format!("n={n}"),
                    (p95 + max * 0.02, y),
                    ("sans-serif", 11),
                )])?;
            }
        }
    }
    root.present()?;
    Ok(())
}
pub(super) fn render(c: &Census, out: &Path) -> Result<()> {
    policy_ranges(c, out)?;
    if c.study {
        for w in [1, 2] {
            historical(c, out, w)?;
        }
        return Ok(());
    }
    for signed in [true, false] {
        let fields: Vec<_> = FIELDS
            .into_iter()
            .filter(|f| f.signed() == signed)
            .collect();
        let mut magnitudes = vec![];
        for field in &fields {
            for w in [1, 2] {
                magnitudes.extend(
                    c.cells
                        .iter()
                        .filter_map(|c| field.metric(c, w).ok().map(f64::abs)),
                );
            }
        }
        let limit = color_limit(magnitudes);
        for field in fields {
            for w in [1, 2] {
                map(c, out, field, w, limit)?;
            }
        }
    }
    Ok(())
}
