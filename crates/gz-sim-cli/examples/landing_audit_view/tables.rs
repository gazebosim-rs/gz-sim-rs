// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::{Path, Result, read};
use plotters::prelude::*;
use serde::Deserialize;

struct Cell {
    value: String,
    fill: RGBColor,
}
fn cell(value: impl ToString) -> Cell {
    Cell {
        value: value.to_string(),
        fill: WHITE,
    }
}
fn pct(value: f64) -> Cell {
    // Keep the sign in the label for monochrome displays.
    let shade = (value.abs() / 20.).min(1.) * 0.60;
    let hue = if value < 0. {
        RGBColor(79, 157, 191)
    } else {
        RGBColor(218, 136, 64)
    };
    let blend = |c| (255. - shade * (255. - f64::from(c))) as u8;
    Cell {
        value: format!("{value:+.2}%"),
        fill: RGBColor(blend(hue.0), blend(hue.1), blend(hue.2)),
    }
}
fn table(
    path: &Path,
    title: &str,
    subtitle: &str,
    headers: &[&str],
    rows: Vec<Vec<Cell>>,
) -> Result<()> {
    let w = 1260;
    let h = 118 + 32 * rows.len() as i32;
    let root = SVGBackend::new(path, (w as u32, h as u32)).into_drawing_area();
    root.fill(&WHITE)?;
    let text = RGBColor(31, 49, 61);
    root.draw(&Text::new(
        title,
        (22, 14),
        ("sans-serif", 26).into_font().color(&text),
    ))?;
    root.draw(&Text::new(
        subtitle,
        (22, 52),
        ("sans-serif", 16).into_font().color(&text),
    ))?;
    let column = (w - 36) / headers.len() as i32;
    for (i, header) in headers.iter().enumerate() {
        root.draw(&Text::new(
            *header,
            (22 + i as i32 * column, 84),
            ("sans-serif", 16)
                .into_font()
                .style(FontStyle::Bold)
                .color(&text),
        ))?;
    }
    for (n, row) in rows.iter().enumerate() {
        if row.len() != headers.len() {
            return Err("figure row has incorrect width".into());
        }
        let y = 110 + n as i32 * 32;
        for (i, c) in row.iter().enumerate() {
            let x = 18 + i as i32 * column;
            root.draw(&Rectangle::new(
                [(x, y), (x + column - 2, y + 30)],
                c.fill.filled(),
            ))?;
            root.draw(&PathElement::new(
                [(x, y), (x + column - 2, y)],
                RGBColor(223, 229, 232),
            ))?;
            root.draw(&Text::new(
                c.value.as_str(),
                (x + 4, y + 5),
                ("sans-serif", 16).into_font().color(&text),
            ))?;
        }
    }
    root.present()?;
    Ok(())
}

#[derive(Deserialize)]
struct Matrix {
    runs: Vec<MatrixRun>,
}
#[derive(Deserialize)]
struct MatrixRun {
    step_us: u64,
    bottom_dwell_shift_us: i64,
    cycles: Vec<MatrixCycle>,
}
#[derive(Deserialize)]
struct MatrixCycle {
    cycle: u64,
    landing_offset_us: i64,
    switched_landing: Impact,
    reference_landing: Impact,
    switched_recontacts: Vec<Recontact>,
    reference_recontacts: Vec<Recontact>,
    motion_interval_impulses_ns: Stroke,
}
#[derive(Deserialize)]
struct Stroke {
    switched: f64,
    reference: f64,
}
#[derive(Deserialize)]
struct Recontact {
    time_s: f64,
    first_ns: f64,
    peak_free_gap_m: f64,
}
#[derive(Deserialize)]
struct Impact {
    first_ns: f64,
    second_ns: f64,
    sum_5ms_ns: f64,
    sum_50ms_ns: f64,
    time_s: f64,
    first_momentum_balance_ns: f64,
    window_momentum_residual_ns: f64,
    estimated_gap_crossing_fraction: Option<f64>,
    reference_clock_5ms_ns: f64,
}
fn change(a: f64, b: f64) -> f64 {
    (a / b - 1.) * 100.
}
pub(super) fn sweep(audit: &Path, out: &Path) -> Result<()> {
    let matrix: Matrix = read(&audit.join("sweep/matrix.json"))?;
    let mut timing = vec![];
    let mut conservation = vec![];
    let mut recontacts = vec![];
    for trial in matrix.runs {
        for c in trial.cycles {
            let a = &c.switched_landing;
            let b = &c.reference_landing;
            timing.push(vec![
                cell(trial.step_us),
                cell(trial.bottom_dwell_shift_us),
                cell(c.cycle),
                cell(c.landing_offset_us),
                pct(change(a.first_ns, b.first_ns)),
                pct(change(a.first_ns + a.second_ns, b.first_ns + b.second_ns)),
                pct(change(a.sum_5ms_ns, b.sum_5ms_ns)),
                pct(change(a.reference_clock_5ms_ns, b.reference_clock_5ms_ns)),
                pct(change(a.sum_50ms_ns, b.sum_50ms_ns)),
                pct(change(
                    c.motion_interval_impulses_ns.switched,
                    c.motion_interval_impulses_ns.reference,
                )),
            ]);
            if trial.bottom_dwell_shift_us == 0 {
                for (name, landing, events) in [
                    ("Switched", a, &c.switched_recontacts),
                    ("Dynamic", b, &c.reference_recontacts),
                ] {
                    let crossing = landing
                        .estimated_gap_crossing_fraction
                        .map(|v| format!("{v:.3}"))
                        .unwrap_or_else(|| "—".into());
                    conservation.push(vec![
                        cell(trial.step_us),
                        cell(c.cycle),
                        cell(name),
                        cell(format!("{:.3}", landing.time_s * 1000.)),
                        cell(crossing),
                        cell(format!("{:.6}", landing.first_ns)),
                        cell(format!("{:.6}", landing.first_momentum_balance_ns)),
                        cell(format!("{:+.2}", 1e6 * landing.window_momentum_residual_ns)),
                    ]);
                    if events.is_empty() {
                        recontacts.push(vec![
                            cell(trial.step_us),
                            cell(c.cycle),
                            cell(name),
                            cell(format!("{:.3}", landing.time_s * 1000.)),
                            cell("—"),
                            cell("—"),
                            cell("—"),
                            cell("—"),
                        ]);
                    }
                    for (index, event) in events.iter().enumerate() {
                        recontacts.push(vec![
                            cell(trial.step_us),
                            cell(c.cycle),
                            cell(name),
                            cell(format!("{:.3}", landing.time_s * 1000.)),
                            cell(index + 1),
                            cell(format!("{:.3}", (event.time_s - landing.time_s) * 1000.)),
                            cell(format!("{:.4}", event.first_ns)),
                            cell(format!("{:.1}", event.peak_free_gap_m * 1e6)),
                        ]);
                    }
                }
            }
        }
    }
    table(
        &out.join("timing-matrix.svg"),
        "Impact sensitivity to dwell timing and native step",
        "Signed switched error against its paired dynamic run. Orange is higher and blue is lower. Shared 5 ms starts on the reference landing clock.",
        &[
            "Step µs",
            "Dwell shift µs",
            "Cycle",
            "Onset Δ µs",
            "Step 1",
            "Steps 1+2",
            "Own 5 ms",
            "Shared 5 ms",
            "Own 50 ms",
            "Full stroke",
        ],
        timing,
    )?;
    table(
        &out.join("balance.svg"),
        "Vertical momentum check at primary touchdown",
        "Unshifted commands. Crossing is the pre-solve geometric estimate in units of one step. The last column is the signed 5 ms budget residual.",
        &[
            "Step µs",
            "Cycle",
            "Lane",
            "Onset ms",
            "Crossing",
            "First N·s",
            "Balance N·s",
            "Residual µN·s",
        ],
        conservation,
    )?;
    table(
        &out.join("recontacts.svg"),
        "Secondary landings after the primary touchdown",
        "Unshifted commands. Delay is measured from this lane’s primary landing. A dash indicates no subsequent resolved separation and reload.",
        &[
            "Step µs",
            "Cycle",
            "Lane",
            "Primary ms",
            "Reload",
            "Delay ms",
            "First N·s",
            "Free gap µm",
        ],
        recontacts,
    )
}
#[derive(Deserialize)]
struct Matched {
    crossovers: Vec<Crossover>,
    masks: Vec<Mask>,
    height_sensitivity: Vec<GapTrial>,
}
#[derive(Deserialize)]
struct Crossover {
    cut_tick: u64,
    motion: String,
    geometry: String,
    treatment: String,
    landing_tick: u64,
    first_ns: f64,
    second_ns: f64,
    window_ns: f64,
}
#[derive(Deserialize)]
struct GapTrial {
    source_mask: u8,
    factor: f64,
    adjusted_gap_m: f64,
    pad_height_offset_m: f64,
    landing_tick: u64,
    first_ns: f64,
    second_ns: f64,
    window_ns: f64,
}
#[derive(Deserialize)]
struct Mask {
    requested_mask: u8,
    realized_mask: u8,
    landing_tick: u64,
    first_ns: f64,
    second_ns: f64,
    window_ns: f64,
}
pub(super) fn matched(audit: &Path, out: &Path) -> Result<()> {
    let summary: Matched = read(&audit.join("matched/summary.json"))?;
    let mut rows = vec![];
    for c in summary.crossovers {
        rows.push(vec![
            cell(c.cut_tick),
            cell(c.motion),
            cell(c.geometry),
            cell(c.treatment),
            cell(c.landing_tick),
            cell(format!("{:.6}", c.first_ns)),
            cell(format!("{:.6}", c.second_ns)),
            cell(format!("{:.6}", c.window_ns)),
        ]);
    }
    table(
        &out.join("matched-motion.svg"),
        "Matching incoming motion before the third touchdown",
        "The motion donor supplies kinematics. The receiver supplies collision history. Each cut is followed by native forward solves.",
        &[
            "Cut tick",
            "Motion",
            "Receiver",
            "Treatment",
            "Landing tick",
            "First N·s",
            "Second N·s",
            "5 ms N·s",
        ],
        rows,
    )?;
    let gap = summary
        .height_sensitivity
        .into_iter()
        .map(|r| {
            vec![
                cell(r.source_mask),
                cell(format!("{:.2}", r.factor)),
                cell(format!("{:+.3}", r.pad_height_offset_m * 1e6)),
                cell(format!("{:.3}", r.adjusted_gap_m * 1e6)),
                cell(r.landing_tick),
                cell(format!("{:.6}", r.first_ns)),
                cell(format!("{:.6}", r.second_ns)),
                cell(format!("{:.6}", r.window_ns)),
            ]
        })
        .collect();
    table(
        &out.join("gap.svg"),
        "Third landing after shifting only the pad height",
        "The interpolation factor is applied to the measured incoming gap difference. Each modification starts at the final cut, tick 1423.",
        &[
            "Source mask",
            "Factor",
            "Pad shift µm",
            "Gap µm",
            "Landing tick",
            "First N·s",
            "Second N·s",
            "5 ms N·s",
        ],
        gap,
    )?;
    let reference = summary
        .masks
        .iter()
        .find(|m| m.requested_mask == 0)
        .ok_or("always-dynamic mask missing")?;
    let base = (reference.first_ns, reference.window_ns);
    let rows = summary
        .masks
        .into_iter()
        .map(|m| {
            let cycles = (0..3)
                .filter(|i| m.realized_mask & (1 << i) != 0)
                .map(|i| (i + 1).to_string())
                .collect::<Vec<_>>();
            vec![
                cell(m.requested_mask),
                cell(if cycles.is_empty() {
                    "None".into()
                } else {
                    cycles.join(", ")
                }),
                cell(m.landing_tick),
                cell(format!("{:.6}", m.first_ns)),
                pct(change(m.first_ns, base.0)),
                cell(format!("{:.6}", m.second_ns)),
                cell(format!("{:.6}", m.window_ns)),
                pct(change(m.window_ns, base.1)),
            ]
        })
        .collect();
    table(
        &out.join("history.svg"),
        "Which prior tier transitions change the third landing?",
        "Eight hold combinations on the same 500 µs lift schedule. Signed errors use the unheld, always-dynamic row as the reference.",
        &[
            "Mask",
            "Held cycles",
            "Landing tick",
            "First N·s",
            "First Δ",
            "Second N·s",
            "5 ms N·s",
            "5 ms Δ",
        ],
        rows,
    )
}
#[derive(Deserialize)]
struct Admission {
    runs: Vec<Policy>,
    heldout: Vec<Policy>,
    early_wake: Vec<Policy>,
}
#[derive(Deserialize)]
struct Policy {
    fixture: String,
    dwell_ms: u64,
    prescribed_steps: u64,
    total_steps: u64,
    cycles: Vec<PolicyCycle>,
    lead_us: Option<u64>,
}
#[derive(Deserialize)]
struct PolicyCycle {
    first_pct: f64,
    window_pct: f64,
}
fn policy_row(run: &Policy, control: u64) -> Result<Vec<Cell>> {
    let [c1, c2, c3] = run.cycles.as_slice() else {
        return Err("expected three policy landings".into());
    };
    Ok(vec![
        cell(run.fixture.replace('_', " ")),
        cell(control),
        cell(run.prescribed_steps),
        cell(format!(
            "{:.1}%",
            100. * run.prescribed_steps as f64 / run.total_steps as f64
        )),
        pct(c1.first_pct),
        pct(c2.first_pct),
        pct(c3.first_pct),
        pct(c3.window_pct),
    ])
}
#[derive(Deserialize)]
struct Timings {
    trials: Vec<TimeTrial>,
}
#[derive(Deserialize)]
struct TimeTrial {
    block: u64,
    dwell_ms: Option<u64>,
    p50_ns: u64,
    p95_ns: u64,
    total_ns: u64,
}
pub(super) fn admission(audit: &Path, out: &Path) -> Result<()> {
    let data: Admission = read(&audit.join("admission/summary.json"))?;
    let headers = [
        "Fixture",
        "Dwell ms",
        "Held steps",
        "Held share",
        "L1 step Δ",
        "L2 step Δ",
        "L3 step Δ",
        "L3 5 ms Δ",
    ];
    let rows = data
        .runs
        .iter()
        .chain(&data.heldout)
        .map(|r| policy_row(r, r.dwell_ms))
        .collect::<Result<_>>()?;
    table(
        &out.join("admission.svg"),
        "Sustained contact eligibility before switching tiers",
        "Held share is the fraction of native steps spent prescribed. Signed impulse errors use the always-dynamic run on the same fixture.",
        &headers,
        rows,
    )?;
    let prior: Admission = read(&audit.join("admission/before-input-order-fix.json"))?;
    let mut changes = Vec::new();
    for after in &data.runs {
        let before = prior
            .runs
            .iter()
            .find(|r| r.dwell_ms == after.dwell_ms)
            .ok_or("missing before-fix control")?;
        let a = before.cycles.last().ok_or("missing prior landing")?;
        let b = after.cycles.last().ok_or("missing updated landing")?;
        changes.push(vec![
            cell(after.dwell_ms),
            cell(before.prescribed_steps),
            cell(after.prescribed_steps),
            pct(a.first_pct),
            pct(b.first_pct),
            pct(a.window_pct),
            pct(b.window_pct),
        ]);
    }
    table(
        &out.join("lifecycle.svg"),
        "Applying support commands before admission",
        "Canonical three-stroke trial. Before and after refer to the ordering fix. Each signed discrepancy compares with its paired dynamic run.",
        &[
            "Dwell ms",
            "Before held",
            "After held",
            "Before L3 step",
            "After L3 step",
            "Before L3 5 ms",
            "After L3 5 ms",
        ],
        changes,
    )?;
    let mut wake_headers = headers;
    wake_headers[1] = "Wake lead µs";
    let rows = data
        .early_wake
        .iter()
        .map(|r| policy_row(r, r.lead_us.ok_or("missing wake lead")?))
        .collect::<Result<_>>()?;
    table(
        &out.join("wake.svg"),
        "Promoting before the commanded support departure",
        "Wake lead is measured from the departure command. Held share covers the complete run. Orange is a larger reaction and blue is smaller.",
        &wake_headers,
        rows,
    )?;
    let times: Timings = read(&audit.join("admission/timings.json"))?;
    let mut rows = Vec::new();
    let policies = times
        .trials
        .iter()
        .map(|t| t.dwell_ms)
        .collect::<std::collections::BTreeSet<_>>();
    for policy in policies {
        let matching: Vec<_> = times
            .trials
            .iter()
            .filter(|t| t.dwell_ms == policy)
            .collect();
        let format_metric = |get: fn(&TimeTrial) -> u64, divisor: f64| {
            let mut values = matching.iter().map(|t| get(t)).collect::<Vec<_>>();
            values.sort_unstable();
            let n = values.len();
            let mid = (values[(n - 1) / 2] as f64 + values[n / 2] as f64) * 0.5 / divisor;
            format!(
                "{mid:.2}  [{:.2}–{:.2}]",
                values[0] as f64 / divisor,
                values[n - 1] as f64 / divisor
            )
        };
        let blocks = matching
            .iter()
            .map(|t| t.block)
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        rows.push(vec![
            cell(
                policy
                    .map(|d| format!("{d} ms"))
                    .unwrap_or_else(|| "Always dynamic".into()),
            ),
            cell(blocks),
            cell(format_metric(|t| t.p50_ns, 1000.)),
            cell(format_metric(|t| t.p95_ns, 1000.)),
            cell(format_metric(|t| t.total_ns, 1e6)),
        ]);
    }
    table(
        &out.join("runtime.svg"),
        "Local wall time across interleaved trial blocks",
        "Each cell shows the median [minimum–maximum] of per-trial measurements. One run includes the complete three-departure schedule.",
        &[
            "Admission dwell",
            "Blocks",
            "Tick p50 µs",
            "Tick p95 µs",
            "Complete run ms",
        ],
        rows,
    )
}

#[derive(Deserialize)]
struct RigidReport {
    runs: Vec<SingleBody>,
}
#[derive(Deserialize)]
struct SingleBody {
    dt_us: u64,
    cycles: Vec<RigidCycle>,
}
#[derive(Deserialize)]
struct RigidCycle {
    cycle: usize,
    ideal_impact_s: f64,
    ideal_first_ns: f64,
    ideal_5ms_ns: f64,
    landings: Vec<BodyImpact>,
    peak_gap_m: f64,
}
#[derive(Deserialize)]
struct BodyImpact {
    time_s: f64,
    first_ns: f64,
    window_ns: f64,
}
pub(super) fn rigid(audit: &Path, articulated: &[super::Trace], out: &Path) -> Result<()> {
    let report: RigidReport = read(&audit.join("rigid/summary.json"))?;
    let mut rows = Vec::new();
    let mut rigid_clocks = Vec::new();
    for run in report.runs {
        let native = articulated
            .iter()
            .find(|r| r.step_us == run.dt_us)
            .ok_or("missing paired timestep")?;
        for c in run.cycles {
            let control = &native
                .cycles
                .iter()
                .find(|v| v.cycle == c.cycle)
                .ok_or("missing paired native landing")?
                .reference_landing;
            let hit = c.landings.first().ok_or("missing rigid body impact")?;
            rows.push(vec![
                cell(run.dt_us),
                cell(c.cycle),
                cell(format!("{:.3}", c.ideal_first_ns)),
                cell(format!("{:.3}", hit.first_ns)),
                cell(format!("{:.3}", control.first_ns)),
                cell(format!("{:.3}", c.ideal_5ms_ns)),
                cell(format!("{:.3}", hit.window_ns)),
                cell(format!("{:.3}", control.sum_5ms_ns)),
            ]);
            rigid_clocks.push(vec![
                cell(run.dt_us),
                cell(c.cycle),
                cell(format!("{:.3}", 1e3 * c.ideal_impact_s)),
                cell(format!("{:.3}", 1e3 * hit.time_s)),
                cell(format!("{:.3}", 1e3 * control.time_s)),
                cell(format!("{:.3}", 1e3 * c.peak_gap_m)),
                cell(c.landings.len() - 1),
            ]);
        }
    }
    table(
        &out.join("rigid.svg"),
        "Ideal plastic catch, single rigid body and fixed-joint articulation",
        "All values in N·s. The analytical model starts level and stationary at departure. Native columns use the first measured landing on each grid.",
        &[
            "Step µs",
            "Cycle",
            "Ideal catch",
            "Rigid first",
            "Jointed first",
            "Ideal 5 ms",
            "Rigid 5 ms",
            "Jointed 5 ms",
        ],
        rows,
    )?;
    table(
        &out.join("rigid-clock.svg"),
        "Touchdown clock and single-body recontacts",
        "Times run from the end of settlement. Jointed is the always-dynamic articulated control. The last two columns describe the single-body run.",
        &[
            "Step µs",
            "Cycle",
            "Ideal ms",
            "Rigid ms",
            "Jointed ms",
            "Max gap mm",
            "Later landings",
        ],
        rigid_clocks,
    )
}
