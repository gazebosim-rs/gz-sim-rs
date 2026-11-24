use super::{AMBER, GRID, INK, Result, TEAL};
use plotters::prelude::*;
use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize)]
pub(super) struct Summary {
    #[serde(flatten)]
    pub(super) hit: Outcome,
    pub(super) near_miss: Outcome,
    inclined: Pair,
    coarse_step_admission: Retry,
}
#[derive(Deserialize)]
struct Pair {
    hit: Outcome,
    near_miss: Outcome,
}
#[derive(Deserialize)]
pub(super) struct Outcome {
    dt_s: f64,
    steps: u64,
    world_pitch_rad: f64,
    first_promotion_tick: Option<u64>,
    first_contact_tick: Option<u64>,
    summed_normal_impulse_ns: f64,
    reference_summed_normal_impulse_ns: f64,
}
#[derive(Deserialize)]
struct Retry {
    dt_s: f64,
    settle_s: f64,
    extra_ticks_to_admission: u64,
    hinge_rate_rad_s: f64,
    post_retry: Events,
}
#[derive(Deserialize)]
struct Events {
    first_promotion_tick: u64,
    first_contact_tick: u64,
    reference_first_contact_tick: u64,
}

pub(super) fn inclined(summary: &Summary, out: &Path) -> Result<()> {
    let root = SVGBackend::new(out, (1160, 425)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "Local strike and near miss in two reference frames",
        ("sans-serif", 25),
    )?;
    let rows = [
        ("Strike", &summary.hit),
        ("Strike", &summary.inclined.hit),
        ("Near miss", &summary.near_miss),
        ("Near miss", &summary.inclined.near_miss),
    ];
    let horizon = summary.hit.dt_s * summary.hit.steps as f64 * 1e3;
    let x = |ms: f64| 198 + (ms / horizon * 570.).round() as i32;
    body.draw(&Text::new(
        "After launch (ms)",
        (196, 14),
        ("sans-serif", 16).into_font().color(&INK),
    ))?;
    body.draw(&Text::new(
        "Summed tip impulse (N·s)",
        (831, 14),
        ("sans-serif", 16).into_font().color(&INK),
    ))?;
    for (i, label) in ["Automatic", "Reference"].iter().enumerate() {
        body.draw(&Text::new(
            *label,
            (835 + i as i32 * 123, 42),
            ("sans-serif", 15).into_font().color(&INK),
        ))?;
    }
    for n in 0..=5 {
        let t = n as f64 * horizon / 5.;
        let px = x(t);
        body.draw(&PathElement::new([(px, 56), (px, 297)], GRID))?;
        body.draw(&Text::new(
            format!("{t:.0}"),
            (px - 6, 40),
            ("sans-serif", 14).into_font().color(&INK),
        ))?;
    }
    let dynamic = RGBColor(65, 93, 137);
    for (i, (name, row)) in rows.into_iter().enumerate() {
        let y = 82 + i as i32 * 62;
        let end = row.dt_s * row.steps as f64 * 1e3;
        if (end - horizon).abs() > 1e-7 {
            return Err("rotated run length differs".into());
        }
        body.draw(&Text::new(
            format!("{name} · {:.2} rad", row.world_pitch_rad),
            (16, y + 5),
            ("sans-serif", 17).into_font().color(&INK),
        ))?;
        body.draw(&PathElement::new(
            [(x(0.), y), (x(end), y)],
            TEAL.stroke_width(8),
        ))?;
        if let Some(tick) = row.first_promotion_tick {
            let start = x(tick as f64 * row.dt_s * 1e3);
            body.draw(&PathElement::new(
                [(start, y), (x(end), y)],
                dynamic.stroke_width(8),
            ))?;
            body.draw(&PathElement::new(
                [(start, y - 8), (start, y + 8)],
                INK.stroke_width(2),
            ))?;
        }
        if let Some(tick) = row.first_contact_tick {
            let hit = x(tick as f64 * row.dt_s * 1e3);
            body.draw(&Polygon::new(
                [(hit, y + 4), (hit - 6, y + 15), (hit + 6, y + 15)],
                AMBER.filled(),
            ))?;
        }
        for (j, impulse) in [
            row.summed_normal_impulse_ns,
            row.reference_summed_normal_impulse_ns,
        ]
        .into_iter()
        .enumerate()
        {
            body.draw(&Text::new(
                format!("{impulse:.5}"),
                (835 + j as i32 * 123, y + 5),
                ("sans-serif", 17).into_font().color(&INK),
            ))?;
        }
    }
    for (px, label, color) in [
        (198, "Prescribed", TEAL),
        (335, "Dynamic", dynamic),
        (458, "First tip impulse", AMBER),
    ] {
        body.draw(&Rectangle::new([(px, 331), (px + 16, 338)], color.filled()))?;
        body.draw(&Text::new(
            label,
            (px + 23, 340),
            ("sans-serif", 15).into_font().color(&INK),
        ))?;
    }
    body.draw(&Text::new(
        "Pitch rotates the complete scene, gravity and launch direction together.",
        (198, 370),
        ("sans-serif", 15).into_font().color(&INK),
    ))?;
    root.present()?;
    Ok(())
}

pub(super) fn admission(summary: &Summary, out: &Path) -> Result<()> {
    let retry = &summary.coarse_step_admission;
    let launch = retry.settle_s + retry.dt_s * retry.extra_ticks_to_admission as f64;
    let switch = launch + retry.dt_s * retry.post_retry.first_promotion_tick as f64;
    let impact = launch + retry.dt_s * retry.post_retry.first_contact_tick as f64;
    if retry.post_retry.first_contact_tick != retry.post_retry.reference_first_contact_tick {
        return Err("reference contact differs in admission figure".into());
    }
    let root = SVGBackend::new(out, (1140, 450)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        &format!(
            "Settling, admission and impact at the {:.0} ms step",
            retry.dt_s * 1e3
        ),
        ("sans-serif", 25),
    )?;
    let (main, footer) = body.split_vertically(341);
    let mut chart = ChartBuilder::on(&main)
        .margin(15)
        .y_label_area_size(185)
        .x_label_area_size(43)
        .build_cartesian_2d((retry.settle_s - 0.065)..(impact + 0.14), -0.5..3.55)?;
    let labels = [
        "First impulse",
        "Switch to dynamic",
        "Demotion and launch",
        "Demotion deferred",
    ];
    chart
        .configure_mesh()
        .x_desc("Simulation clock (s)")
        .y_labels(5)
        .max_light_lines(1)
        .y_label_formatter(&|y: &f64| {
            let n = y.round();
            if (n - y).abs() < 0.01 && (0.0..=3.0).contains(&n) {
                labels[n as usize].to_string()
            } else {
                String::new()
            }
        })
        .disable_y_mesh()
        .light_line_style(GRID)
        .draw()?;
    let events = [
        (retry.settle_s, 3., INK),
        (launch, 2., TEAL),
        (switch, 1., RGBColor(65, 93, 137)),
        (impact, 0., AMBER),
    ];
    chart.draw_series(LineSeries::new(
        events.iter().map(|&(t, y, _)| (t, y)),
        GRID.stroke_width(2),
    ))?;
    for (t, y, color) in events {
        chart.draw_series(std::iter::once(Circle::new((t, y), 7, color.filled())))?;
        chart.draw_series(std::iter::once(Text::new(
            format!("{t:.3} s"),
            (t + 0.020, y + 0.10),
            ("sans-serif", 17).into_font().color(&INK),
        )))?;
    }
    footer.draw(&Text::new(format!("At the first request the hinge was moving at {:.2e} rad/s. Demotion succeeded {} native steps later.",retry.hinge_rate_rad_s,retry.extra_ticks_to_admission),(35,17),("sans-serif",16).into_font().color(&INK)))?;
    footer.draw(&Text::new(format!("After launch the automatic arm promoted on step {}. Both runs recorded the first impulse on step {}.",retry.post_retry.first_promotion_tick,retry.post_retry.first_contact_tick),(35,43),("sans-serif",16).into_font().color(&INK)))?;
    root.present()?;
    Ok(())
}
