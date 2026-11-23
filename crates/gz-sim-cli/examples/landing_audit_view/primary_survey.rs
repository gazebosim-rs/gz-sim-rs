// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Factorial heatmaps and deterministic outlier selection from complete native case files.
use super::*;
use plotters::prelude::{
    ChartBuilder, Color, IntoDrawingArea, RGBColor, Rectangle, SVGBackend, Text, WHITE,
};
use plotters::style::IntoTextStyle;
use plotters::style::text_anchor::{HPos, Pos, VPos};
use serde::Serialize;
use std::io::{BufWriter, Write};

#[path = "primary_ranks.rs"]
mod ranks;

const SIDE: usize = 25;
const FIXTURES: usize = SIDE * SIDE * 2;
const STROKES: usize = 3;
const WINDOWS: [u64; 2] = [10, 26];
const GRIDS: [u32; 4] = [512, 1024, 2048, 4096];

#[derive(Deserialize)]
struct Saved {
    format: String,
    index: usize,
    outer_us: u64,
    horizon: u64,
    result: Case,
}
#[derive(Deserialize)]
struct Case {
    fixture: Fixture,
    first_disjoint_tick: Option<u64>,
    trials: Vec<Trial>,
}
#[derive(Deserialize)]
struct Fixture {
    offset_m: f64,
    speed_scale: f64,
    switched: bool,
}
#[derive(Deserialize)]
struct Trial {
    stroke: usize,
    cut: Cut,
    paths: Vec<Branch>,
}
#[derive(Deserialize)]
struct Cut {
    tick: u64,
}
#[derive(Deserialize)]
struct Branch {
    method: String,
    calls: Option<u32>,
    iterations: Option<u32>,
    record: Record,
}
#[derive(Deserialize)]
struct Record {
    samples: Vec<Endpoint>,
}
#[derive(Deserialize)]
struct Endpoint {
    i: u64,
    com_m: [f64; 3],
}
impl Trial {
    fn endpoints(
        &self,
        method: &str,
        calls: Option<u32>,
        iterations: Option<u32>,
    ) -> Result<[DVec3; 2]> {
        let mut matching = self
            .paths
            .iter()
            .filter(|p| p.method == method && p.calls == calls && p.iterations == iterations);
        let p = matching.next().ok_or("missing survey path")?;
        if matching.next().is_some() {
            return Err("duplicate survey path".into());
        }
        let mut endpoints = [DVec3::ZERO; 2];
        for (i, tick) in WINDOWS.iter().enumerate() {
            let mut matches = p.record.samples.iter().filter(|r| r.i == *tick);
            endpoints[i] = DVec3::from(
                matches
                    .next()
                    .ok_or("missing absolute-time endpoint")?
                    .com_m,
            );
            if matches.next().is_some() || !endpoints[i].is_finite() {
                return Err("invalid survey endpoint".into());
            }
        }
        Ok(endpoints)
    }
}

#[derive(Clone, Copy, Serialize)]
struct Location {
    index: usize,
    offset_mm: u16,
    drive_percent: u16,
    holds: bool,
    stroke: usize,
    cut_tick: u64,
}
#[derive(Clone, Copy, Serialize)]
struct Metrics {
    adaptive_vs_4096_mm: f64,
    onset_8x8_vs_4096_mm: f64,
    change_mm: f64,
    successive_grid_distance_mm: [f64; 3],
    change_by_reference_mm: [f64; 4],
}
#[derive(Clone, Copy, Serialize)]
struct Cell {
    site: Location,
    at_5ms: Metrics,
    at_13ms: Metrics,
}
impl Cell {
    fn at(self, window: usize) -> Metrics {
        if window == 0 {
            self.at_5ms
        } else {
            self.at_13ms
        }
    }
}
struct Census {
    // Slot order follows physical layout, independently of execution order.
    completed: Vec<bool>,
    disjoint: usize,
    cells: Vec<Option<Cell>>,
}

impl Census {
    fn load(dir: &Path) -> Result<Self> {
        let mut c = Self {
            completed: vec![false; FIXTURES],
            disjoint: 0,
            cells: vec![None; FIXTURES * STROKES],
        };
        for i in 0..FIXTURES {
            let p = dir.join("cases").join(format!("case-{i:04}.json"));
            let file = match fs::File::open(&p) {
                Ok(f) => f,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
                Err(e) => return Err(e.into()),
            };
            let saved: Saved = serde_json::from_reader(BufReader::new(file))?;
            let slot = i * 613 % FIXTURES;
            let x = slot / (2 * SIDE);
            let y = (slot / 2) % SIDE;
            let holds = slot % 2 == 1;
            let mm = 165 + 5 * x as u16;
            let percent = 76 + 2 * y as u16;
            let f = saved.result.fixture;
            if saved.format != "outer-primary-survey-v1"
                || saved.index != i
                || saved.outer_us != 500
                || saved.horizon != 26
                || (f.offset_m - f64::from(mm) / 1000.).abs() > 1e-12
                || (f.speed_scale - f64::from(percent) / 100.).abs() > 1e-12
                || f.switched != holds
            {
                return Err(format!("inconsistent survey fixture {i}").into());
            }
            c.completed[slot] = true;
            c.disjoint += usize::from(saved.result.first_disjoint_tick.is_some());
            for t in saved.result.trials {
                if !(1..=STROKES).contains(&t.stroke) {
                    return Err(format!("unknown stroke in case {i}").into());
                }
                let idx = slot * STROKES + t.stroke - 1;
                if c.cells[idx].is_some() {
                    return Err(format!("duplicate stroke in case {i}").into());
                }
                let adaptive = t.endpoints("adaptive", None, None)?;
                let onset = t.endpoints("onset", Some(8), Some(8))?;
                let sequence = GRIDS.map(|n| t.endpoints("uniform", Some(n), Some(4)));
                let [a, b, d, e] = sequence;
                let grids = [a?, b?, d?, e?];
                let at = |k: usize| {
                    let fine = grids[3][k];
                    let base = adaptive[k].distance(fine) * 1000.;
                    let candidate = onset[k].distance(fine) * 1000.;
                    Metrics {
                        adaptive_vs_4096_mm: base,
                        onset_8x8_vs_4096_mm: candidate,
                        change_mm: candidate - base,
                        change_by_reference_mm: grids.map(|grid| {
                            onset[k].distance(grid[k]) * 1000.
                                - adaptive[k].distance(grid[k]) * 1000.
                        }),
                        successive_grid_distance_mm: std::array::from_fn(|j| {
                            grids[j][k].distance(grids[j + 1][k]) * 1000.
                        }),
                    }
                };
                c.cells[idx] = Some(Cell {
                    site: Location {
                        index: i,
                        offset_mm: mm,
                        drive_percent: percent,
                        holds,
                        stroke: t.stroke,
                        cut_tick: t.cut.tick,
                    },
                    at_5ms: at(0),
                    at_13ms: at(1),
                });
            }
        }
        if !c.completed.iter().any(|x| *x) {
            return Err("no completed case files".into());
        }
        Ok(c)
    }
    fn completed(&self) -> usize {
        self.completed.iter().filter(|x| **x).count()
    }
    fn present(&self) -> Vec<Cell> {
        self.cells.iter().flatten().copied().collect()
    }
}

// Nearest rank; true p95 for count 1 equals the sole measured value.
fn nearest(sorted: &[f64], numerator: usize) -> Option<f64> {
    sorted
        .get((numerator * sorted.len()).div_ceil(100).saturating_sub(1))
        .copied()
}
fn distribution(mut ns: Vec<f64>) -> Value {
    ns.sort_unstable_by(f64::total_cmp);
    json!({"n":ns.len(),"min":ns.first(),"p50":nearest(&ns,50),"p95":nearest(&ns,95),"max":ns.last(),
        "mean":(!ns.is_empty()).then(||ns.iter().sum::<f64>()/ns.len() as f64)})
}
fn stat(cells: &[Cell], at: usize) -> Value {
    let values: Vec<_> = cells.iter().map(|c| c.at(at)).collect();
    let improved = values.iter().filter(|m| m.change_mm < 0.).count();
    let worse = values.iter().filter(|m| m.change_mm > 0.).count();
    json!({"n":values.len(),"grid_ranking":ranks::Summary::from_values(&values),"improved":improved,"tied":values.len()-improved-worse,"worsened":worse,
        "adaptive_vs_4096_mm":distribution(values.iter().map(|m|m.adaptive_vs_4096_mm).collect()),
        "onset_8x8_vs_4096_mm":distribution(values.iter().map(|m|m.onset_8x8_vs_4096_mm).collect()),
        "change_mm":distribution(values.iter().map(|m|m.change_mm).collect()),
        "successive_mm":(0..3).map(|j|distribution(values.iter().map(|m|m.successive_grid_distance_mm[j]).collect())).collect::<Vec<_>>()})
}
fn largest(cells: &[Cell]) -> Option<&Cell> {
    cells.iter().max_by(|a, b| {
        a.at_13ms
            .change_mm
            .total_cmp(&b.at_13ms.change_mm)
            .then_with(|| b.site.index.cmp(&a.site.index))
            .then_with(|| b.site.stroke.cmp(&a.site.stroke))
    })
}

#[derive(Clone, Copy)]
enum Layer {
    Candidate,
    Grid(usize),
}
impl Layer {
    fn value(self, c: Cell, at: usize) -> f64 {
        match self {
            Self::Candidate => c.at(at).change_mm,
            Self::Grid(j) => c.at(at).successive_grid_distance_mm[j],
        }
    }
    fn name(self) -> String {
        match self {
            Self::Candidate => "Onset refinement · change in COM error against 4,096".into(),
            Self::Grid(j) => format!(
                "Grid sensitivity · {} to {} native solves",
                GRIDS[j],
                GRIDS[j + 1]
            ),
        }
    }
    fn color(self, value: f64, limit: f64) -> RGBColor {
        ramp_color(value, limit, matches!(self, Self::Candidate))
    }
}

pub(super) fn ramp_color(value: f64, limit: f64, signed: bool) -> RGBColor {
    let start = RGBColor(249, 248, 241);
    let end = match signed {
        true if value < 0. => RGBColor(0, 117, 119),
        true => RGBColor(174, 57, 16),
        false => RGBColor(32, 75, 135),
    };
    let t = (value.abs() / limit).min(1.);
    let blend = |a: u8, b: u8| (f64::from(a) + (f64::from(b) - f64::from(a)) * t).round() as u8;
    RGBColor(
        blend(start.0, end.0),
        blend(start.1, end.1),
        blend(start.2, end.2),
    )
}

pub(super) fn color_limit(mut values: Vec<f64>) -> f64 {
    values.sort_unstable_by(f64::total_cmp);
    nearest(&values, 95).unwrap_or(0.).max(1e-9)
}
fn heatmap(path: &Path, census: &Census, layer: Layer, at: usize, limit: f64) -> Result<()> {
    let root = SVGBackend::new(path, (1180, 825)).into_drawing_area();
    root.fill(&WHITE)?;
    let title = format!("{} after {} ms", layer.name(), WINDOWS[at] / 2);
    let (heading, body) = root
        .titled(&title, ("sans-serif", 25))?
        .split_vertically(76);
    let present = census.cells.iter().flatten().count();
    let absent = census.completed() * STROKES - present;
    let pending = FIXTURES - census.completed();
    heading.draw(&Text::new(format!("{} / {} fixtures recorded   ·   {} matched landings   ·   {} unavailable landings in recorded fixtures",census.completed(),FIXTURES,present,absent),(34,15),("sans-serif",16)))?;
    let labels = match layer {
        Layer::Candidate => [
            format!("≤ −{limit:.3} mm · improves"),
            "0".into(),
            format!("worsens · ≥ +{limit:.3} mm"),
        ],
        Layer::Grid(_) => [
            "0 mm".into(),
            "COM distance".into(),
            format!("≥ {limit:.3} mm"),
        ],
    };
    let x0 = 35;
    let width = 498i32;
    for x in 0..width {
        let t = f64::from(x) / f64::from(width - 1);
        let v = match layer {
            Layer::Candidate => (2. * t - 1.) * limit,
            Layer::Grid(_) => t * limit,
        };
        heading.draw(&Rectangle::new(
            [(x0 + x, 29), (x0 + x + 1, 40)],
            layer.color(v, limit).filled(),
        ))?;
    }
    for ((x, anchor), label) in [
        (x0, HPos::Left),
        (x0 + width / 2, HPos::Center),
        (x0 + width, HPos::Right),
    ]
    .into_iter()
    .zip(labels)
    {
        let style = ("sans-serif", 13)
            .into_text_style(&heading)
            .pos(Pos::new(anchor, VPos::Top));
        heading.draw(&Text::new(label, (x, 45), style))?;
    }
    let pending_color = RGBColor(211, 215, 219);
    let unavailable = RGBColor(64, 65, 71);
    for (x, color, text) in [
        (595, pending_color, format!("Pending fixture ({pending})")),
        (
            835,
            unavailable,
            format!("No primary at this stroke ({absent})"),
        ),
    ] {
        heading.draw(&Rectangle::new([(x, 31), (x + 16, 46)], color.filled()))?;
        heading.draw(&Text::new(text, (x + 22, 44), ("sans-serif", 14)))?;
    }
    facet_panels(&body, |x, y, hold, stroke| {
        let slot = x * 2 * SIDE + y * 2 + hold;
        let val = census.cells[slot * STROKES + stroke];
        let color = val.map_or(
            if census.completed[slot] {
                unavailable
            } else {
                pending_color
            },
            |c| layer.color(layer.value(c, at), limit),
        );
        (val.is_some(), color)
    })?;
    root.present()?;
    Ok(())
}
pub(super) fn facet_panels(
    body: &plotters::drawing::DrawingArea<SVGBackend<'_>, plotters::coord::Shift>,
    tile: impl Fn(usize, usize, usize, usize) -> (bool, RGBColor),
) -> Result<()> {
    for (panel, area) in body.split_evenly((2, 3)).into_iter().enumerate() {
        let hold = panel / STROKES;
        let stroke = panel % STROKES;
        let mut measured = 0usize;
        let tiles = (0..SIDE)
            .flat_map(|x| (0..SIDE).map(move |y| (x, y)))
            .map(|(x, y)| {
                let (available, color) = tile(x, y, hold, stroke);
                measured += usize::from(available);
                let cx = (165 + 5 * x) as f64;
                let cy = (76 + 2 * y) as f64;
                Rectangle::new([(cx - 2.5, cy - 1.), (cx + 2.5, cy + 1.)], color.filled())
            })
            .collect::<Vec<_>>();
        let history = if hold == 0 {
            "Always dynamic"
        } else {
            "Hold eligible"
        };
        let mut chart = ChartBuilder::on(&area)
            .margin(10)
            .caption(
                format!("{history} · stroke {} · {measured}/625", stroke + 1),
                ("sans-serif", 17),
            )
            .x_label_area_size(46)
            .y_label_area_size(56)
            .build_cartesian_2d(162.5..287.5, 75.0..125.0)?;
        chart
            .configure_mesh()
            .x_desc("Support offset (mm)")
            .y_desc("Drive speed (%)")
            .x_labels(6)
            .y_labels(5)
            .x_label_formatter(&|v| format!("{v:.0}"))
            .y_label_formatter(&|v| format!("{v:.0}"))
            .disable_mesh()
            .draw()?;
        chart.draw_series(tiles)?;
    }
    Ok(())
}

pub(super) fn export(dir: &Path, out: &Path) -> Result<()> {
    let census = Census::load(dir)?;
    let present = census.present();
    fs::create_dir_all(out)?;
    let mut raw = BufWriter::new(fs::File::create(out.join("cells.jsonl"))?);
    for cell in &present {
        serde_json::to_writer(&mut raw, cell)?;
        raw.write_all(b"\n")?;
    }
    raw.flush()?;
    let candidate_limit = color_limit(
        present
            .iter()
            .flat_map(|c| [c.at_5ms.change_mm.abs(), c.at_13ms.change_mm.abs()])
            .collect(),
    );
    let grid_limit = color_limit(
        present
            .iter()
            .flat_map(|c| {
                c.at_5ms
                    .successive_grid_distance_mm
                    .into_iter()
                    .chain(c.at_13ms.successive_grid_distance_mm)
            })
            .collect(),
    );
    for (at, t) in WINDOWS.into_iter().enumerate() {
        heatmap(
            &out.join(format!("onset-{}ms.svg", t / 2)),
            &census,
            Layer::Candidate,
            at,
            candidate_limit,
        )?;
        for j in 0..3 {
            heatmap(
                &out.join(format!(
                    "grid-{}-{}-{}ms.svg",
                    GRIDS[j],
                    GRIDS[j + 1],
                    t / 2
                )),
                &census,
                Layer::Grid(j),
                at,
                grid_limit,
            )?;
        }
    }
    ranks::plot(
        &out.join("reference-ranking.svg"),
        &present,
        census.completed(),
    )?;
    let mut panels = vec![];
    for holds in [false, true] {
        for stroke in 1..=STROKES {
            let rows = present
                .iter()
                .copied()
                .filter(|c| c.site.holds == holds && c.site.stroke == stroke)
                .collect::<Vec<_>>();
            panels.push(json!({"holds":holds,"stroke":stroke,"at_5ms":stat(&rows,0),"at_13ms":stat(&rows,1)}));
        }
    }
    let complete = census.completed();
    let completed_indices: Vec<_> = (0..FIXTURES)
        .filter(|i| census.completed[i * 613 % FIXTURES])
        .collect();
    let summary = json!({"schema":"primary-survey-view-v1","expected_fixtures":FIXTURES,"completed_fixtures":complete,
        "completed_case_indices":completed_indices,"pending_fixtures":FIXTURES-complete,"available_landings":present.len(),"unavailable_landings_in_recorded_fixtures":complete*STROKES-present.len(),
        "fixtures_with_disjoint_footprint":census.disjoint,"grid_sequence":GRIDS,"color_saturation_p95_mm":{"onset":candidate_limit,"grid":grid_limit},
        "ranking_axes":["onset closer","tie","adaptive closer"],
        "ranking_comparison":"ordinary Euclidean COM distance using each selected uniform reference, rows for the coarser and columns for the finer grid",
        "selection_rule":"maximum (8x8-onset absolute COM error minus adaptive absolute COM error) at 13 ms against uniform 4096; ties choose lowest case index then earliest stroke",
        "selection_is_final":complete==FIXTURES,"selection":largest(&present),"at_5ms":stat(&present,0),"at_13ms":stat(&present,1),"panels":panels});
    fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn worst_ties_choose_lowest_case_then_first_stroke() {
        let m = Metrics {
            adaptive_vs_4096_mm: 0.,
            onset_8x8_vs_4096_mm: 1.,
            change_mm: 1.,
            successive_grid_distance_mm: [0.; 3],
            change_by_reference_mm: [1.; 4],
        };
        let make = |index, stroke| Cell {
            site: Location {
                index,
                stroke,
                offset_mm: 165,
                drive_percent: 76,
                holds: false,
                cut_tick: 0,
            },
            at_5ms: m,
            at_13ms: m,
        };
        let samples = [make(2, 1), make(0, 3), make(0, 1)];
        let best = largest(&samples).unwrap();
        assert_eq!((best.site.index, best.site.stroke), (0, 1));
    }
}
