// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay both native lanes and show how the sole's loaded corners change at touchdown.
#[path = "landing_audit_view/contact_energy.rs"]
mod contact_energy;
#[path = "landing_audit_view/edge_compliance.rs"]
mod edge_compliance;
#[path = "landing_audit_view/edge_slip.rs"]
mod edge_slip;
#[path = "landing_audit_view/figures.rs"]
mod figures;
#[path = "landing_audit_view/material_friction.rs"]
mod material_friction;
#[path = "landing_audit_view/primary_survey.rs"]
mod primary_survey;
#[path = "landing_audit_view/rebound.rs"]
mod rebound;
#[path = "landing_audit_view/relaxation_survey.rs"]
mod relaxation_survey;
mod soft_replay;
#[path = "landing_audit_view/touchdown.rs"]
mod touchdown;
use clap::{Parser, Subcommand};
use glam::{DQuat, DVec3};
use serde::{Deserialize, de::DeserializeOwned};
use serde_json::{Value, json};
use soft_replay::Frame;
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    io::BufReader,
    path::{Path, PathBuf},
    time::Duration,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Build the native-step replay from one sweep file.
    Replay { trace: PathBuf, output: PathBuf },
    /// Plot the measured rate, history and admission controls.
    Figures { audit: PathBuf, output: PathBuf },
    /// Compare two local touchdown runs on measured common boundaries.
    Touchdown {
        traces: PathBuf,
        #[arg(value_enum)]
        comparison: touchdown::Comparison,
        output: PathBuf,
    },
    /// Replay the moving lift's work and the foot's mechanical energy on a common clock.
    Energy {
        audit: PathBuf,
        #[arg(value_enum)]
        comparison: contact_energy::Comparison,
        output: PathBuf,
    },
    /// Replay the measured recontact and its solver effort near a selected native tick.
    Rebound {
        audit: PathBuf,
        #[arg(value_enum)]
        deck: rebound::Deck,
        output: PathBuf,
        #[arg(long)]
        first_tick: u64,
        #[arg(long)]
        last_tick: u64,
        #[arg(long)]
        event_tick: u64,
    },
    /// Replay the solver budget used at a partially unloaded second landing.
    ReboundBudget {
        audit: PathBuf,
        output: PathBuf,
        #[arg(long)]
        event_tick: u64,
    },
    /// Compare matched native free flight to its continuous ballistic trajectory.
    ReboundFlight { audit: PathBuf, output: PathBuf },
    /// Compare a live weak recontact request with the current policy and fine reference.
    WeakRecontact { audit: PathBuf, output: PathBuf },
    /// Compare adaptive and fine continuations at the outer-support primary landing.
    OuterDivergence { audit: PathBuf, output: PathBuf },
    /// Compare three matched native runs described by comparison.json.
    NativeComparison { audit: PathBuf, output: PathBuf },
    /// Replay fixture 80's measured transition and both trajectory controls.
    EdgeSlip {
        previous: PathBuf,
        current: PathBuf,
        reference: PathBuf,
        receipt: PathBuf,
        timing: PathBuf,
        output: PathBuf,
    },
    /// Replay how the specified elastic layers carry the load along the foot edge.
    EdgeCompliance {
        reference: PathBuf,
        fixture: PathBuf,
        output: PathBuf,
    },
    /// Show how sole contact develops from free approach and both matched material restarts.
    FreeLanding {
        reference: PathBuf,
        fixture: PathBuf,
        output: PathBuf,
    },
    /// Compare sole forces, energy dissipation and rebound with three damping settings.
    DampedLanding {
        reference: PathBuf,
        fixture: PathBuf,
        output: PathBuf,
    },
    /// Compare forces when the elastic foot begins at corner contact or at line contact.
    CornerPreload {
        reference: PathBuf,
        control: PathBuf,
        fixture: PathBuf,
        output: PathBuf,
    },
    /// Compare the previous and revised runs at the late change of support in fixture 80.
    EdgeRoll {
        previous: PathBuf,
        current: PathBuf,
        reference: PathBuf,
        previous_receipts: PathBuf,
        current_receipts: PathBuf,
        output: PathBuf,
    },
    /// Plot the matched primary-landing survey over support placement and drive speed.
    PrimarySurvey { audit: PathBuf, output: PathBuf },
    /// Plot native and analytical motion for the authored ball friction study.
    MaterialFriction { audit: PathBuf, output: PathBuf },
    /// Diagnose the effects of relaxation in strata defined by the independent finite first feature.
    FeatureRelaxation {
        finite: PathBuf,
        relaxation_survey: PathBuf,
        default_survey: PathBuf,
        output: PathBuf,
    },
    /// Score fixed and causal post-contact relaxation spans against both independent native grids.
    ContinuationSpans {
        audit: PathBuf,
        relaxation_survey: PathBuf,
        default_survey: PathBuf,
        output: PathBuf,
    },
    /// Compare frictional relaxation at matched primary cuts against two empirical grids.
    RelaxationSurvey {
        audit: PathBuf,
        output: PathBuf,
        /// Matching default-grid survey directory for factorial runs.
        #[arg(long)]
        default_survey: Option<PathBuf>,
    },
    /// Plot the local impact balance and independent rigid-body controls.
    EnergyFigures { audit: PathBuf, output: PathBuf },
    /// Plot touchdown reaction integrals and outer-step p95 cost.
    TouchdownFigures { audit: PathBuf, output: PathBuf },
}
#[derive(Deserialize, PartialEq)]
struct Body {
    angular_velocity: Option<[f64; 3]>,
    velocity: Option<[f64; 3]>,
    position: [f64; 3],
    orientation: [f64; 4],
}
#[derive(Deserialize)]
struct Point {
    foot_corner: [i8; 2],
    normal_impulse_ns: f64,
}
#[derive(Deserialize)]
struct Lane {
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    vertical_gap_m: f64,
    points: Vec<Point>,
}
#[derive(Deserialize)]
struct Step {
    tick: u64,
    time_s: f64,
    prescribed: bool,
    switched: Lane,
    reference: Lane,
}
#[derive(Deserialize, PartialEq)]
struct Geometry {
    foot_half_extents: [f64; 3],
    upper_half_extents: Option<[f64; 3]>,
    foot_com_local: Option<[f64; 3]>,
    pad_half_extents: [f64; 3],
    opponent_half_extents: Option<[f64; 3]>,
}
#[derive(Deserialize)]
struct Landing {
    tick: u64,
    time_s: f64,
    first_ns: f64,
    second_ns: f64,
    sum_5ms_ns: f64,
}
#[derive(Deserialize)]
struct Recontact {
    tick: u64,
}
#[derive(Deserialize)]
struct Cycle {
    cycle: usize,
    switched_landing: Landing,
    reference_landing: Landing,
    switched_recontacts: Vec<Recontact>,
    reference_recontacts: Vec<Recontact>,
}
#[derive(Deserialize)]
struct Trace {
    step_us: u64,
    geometry: Geometry,
    frames: Vec<Step>,
    cycles: Vec<Cycle>,
}

fn read<T: DeserializeOwned>(p: &Path) -> Result<T> {
    Ok(serde_json::from_reader(BufReader::new(fs::File::open(p)?))?)
}
const LANE: [[f64; 4]; 2] = [[0.14, 0.84, 0.73, 1.], [0.42, 0.70, 1., 1.]];
const PAD: [f64; 4] = [0.99, 0.67, 0.19, 1.];
const IDLE: [f64; 4] = [0.53, 0.61, 0.66, 1.];
const UPPER: [f64; 4] = [0.85, 0.88, 0.94, 1.];
const SOLE: [f64; 4] = [0.14, 0.19, 0.24, 1.];
fn cuboid(half: [f64; 3]) -> Value {
    json!({"kind":"box","size":half.map(|v|2.*v)})
}
fn corners(lane: &Lane) -> Result<[f64; 4]> {
    let mut j = [0.; 4];
    for p in &lane.points {
        if !p.normal_impulse_ns.is_finite() || p.normal_impulse_ns < 0. {
            return Err("invalid point impulse".into());
        }
        let quadrant = match p.foot_corner {
            [-1, -1] => 0,
            [-1, 1] => 1,
            [1, -1] => 2,
            [1, 1] => 3,
            _ => return Err("invalid sole corner".into()),
        };
        j[quadrant] += p.normal_impulse_ns;
    }
    Ok(j)
}
struct LaneDisplay<'a> {
    label: &'a str,
    color: [f64; 4],
    offset: f64,
    sole_z: f64,
    full: f64,
}
impl LaneDisplay<'_> {
    /// Place a normalized linear reading below one lane's measured sole map.
    fn gauge(
        &self,
        f: &mut Frame,
        quantity: &str,
        half_width: f64,
        z: f64,
        fraction: f64,
        color: [f64; 4],
    ) {
        let name = self.label;
        let x = self.offset;
        f.edge(
            &format!("{name}_{quantity}_scale"),
            DVec3::new(0., x - half_width, z),
            DVec3::new(0., x + half_width, z),
            2. * half_width,
            0.004,
            IDLE,
        );
        f.add(
            &format!("{name}_{quantity}"),
            DVec3::new(0.023, x + (2. * fraction - 1.) * half_width, z),
            DQuat::IDENTITY,
            cuboid([0.011; 3]),
            color,
        );
    }
    fn add_bodies(
        &self,
        f: &mut Frame,
        geometry: &Geometry,
        foot: &Body,
        upper: Option<&Body>,
        pad: &Body,
        opponent: Option<&Body>,
    ) -> Result<()> {
        let Self { label, offset, .. } = *self;
        let view = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let transform = |p: DVec3| view * p + DVec3::Y * offset;
        if upper.is_some() != geometry.upper_half_extents.is_some()
            || opponent.is_some() != geometry.opponent_half_extents.is_some()
        {
            return Err("optional shape and measured native body disagree".into());
        }
        let boxes = [
            Some(("foot", foot, geometry.foot_half_extents, self.color)),
            upper
                .zip(geometry.upper_half_extents)
                .map(|(u, h)| ("upper", u, h, UPPER)),
            Some(("pad", pad, geometry.pad_half_extents, PAD)),
            opponent
                .zip(geometry.opponent_half_extents)
                .map(|(b, h)| ("opponent", b, h, [0.49, 0.45, 0.76, 1.])),
        ];
        for (name, b, half, col) in boxes.into_iter().flatten() {
            let p = DVec3::from(b.position);
            let q = DQuat::from_array(b.orientation);
            if !p.is_finite()
                || !q.is_normalized()
                || half.iter().any(|v| !v.is_finite() || *v <= 0.)
            {
                return Err("invalid measured transform".into());
            }
            f.add(
                &format!("{label}_{name}"),
                transform(p),
                view * q,
                cuboid(half),
                col,
            );
        }
        Ok(())
    }
    fn add(
        &self,
        f: &mut Frame,
        geometry: &Geometry,
        bodies: [&Body; 3],
        impulses: [f64; 4],
    ) -> Result<()> {
        self.add_bodies(f, geometry, bodies[0], Some(bodies[1]), bodies[2], None)?;
        self.add_corners(f, geometry, impulses)
    }
    /// Fixed material corners follow the measured link transform. The second
    /// marker moves along the outward face normal in proportion to quadrant load.
    fn add_face_markers(
        &self,
        frame: &mut Frame,
        geometry: &Geometry,
        foot: &Body,
        loads: [f64; 4],
        top: bool,
    ) {
        let [hx, hy, hz] = geometry.foot_half_extents;
        let rotation = DQuat::from_array(foot.orientation);
        let view = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let normal = DVec3::Z * if top { 1. } else { -1. };
        let size = hx.min(hy);
        for (i, j) in loads.into_iter().enumerate() {
            let corner = DVec3::new(
                if i >= 2 { hx } else { -hx },
                if i % 2 == 1 { hy } else { -hy },
                normal.z * hz,
            );
            for (name, local, radius, color) in [
                ("face_corner", corner, 0.04 * size, [0.40, 0.35, 0.22, 1.]),
                (
                    "face_load",
                    corner + normal * (0.6 * size * j / self.full),
                    0.06 * size,
                    [0.97, 0.82, 0.2, 1.],
                ),
            ] {
                let pos =
                    view * (DVec3::from(foot.position) + rotation * local) + DVec3::Y * self.offset;
                frame.add(
                    &format!("{}_{name}_{i}", self.label),
                    pos,
                    DQuat::IDENTITY,
                    json!({"kind":"sphere","radius":radius}),
                    color,
                );
            }
        }
    }
    fn add_corners(&self, f: &mut Frame, geometry: &Geometry, impulses: [f64; 4]) -> Result<()> {
        let Self {
            label,
            offset,
            sole_z,
            full,
            ..
        } = *self;
        let [hx, hy, _] = geometry.foot_half_extents;
        // Diagram plane: right is local +X, up is local +Y. Corner load is the
        // sum of the solved native points in that sole quadrant for this one tick.
        f.add(
            &format!("{label}_sole_map"),
            DVec3::new(-0.014, offset, sole_z),
            DQuat::IDENTITY,
            cuboid([0.005, hx * 1.22, hy * 1.22]),
            SOLE,
        );
        for (i, j) in impulses.into_iter().enumerate() {
            let x = offset + if i >= 2 { hx * 0.57 } else { -hx * 0.57 };
            let z = sole_z + if i % 2 == 1 { hy * 0.55 } else { -hy * 0.55 };
            let start = x - hx * 0.42;
            let length = hx * 0.84;
            f.edge(
                &format!("{label}_corner_scale_{i}"),
                DVec3::new(0.001, start, z),
                DVec3::new(0.001, start + length, z),
                length,
                0.004,
                IDLE,
            );
            f.add(
                &format!("{label}_corner_{i}"),
                DVec3::new(0.026, start + length * j / full, z),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.010}),
                self.color,
            );
        }
        Ok(())
    }
}

fn export(trace_path: &Path, out: &Path) -> Result<()> {
    let trace: Trace = read(trace_path)?;
    let dt = trace
        .step_us
        .checked_mul(1000)
        .filter(|v| *v > 0)
        .ok_or("invalid step duration")?;
    let width = 2. * trace.geometry.pad_half_extents[0];
    let mut full = 0f64;
    let mut min_z = f64::INFINITY;
    let mut available = BTreeSet::new();
    let mut flight = (0, 0.);
    for s in &trace.frames {
        if available.last().is_some_and(|t| *t >= s.tick)
            || (s.time_s - s.tick as f64 * dt as f64 * 1e-9).abs() > 1e-10
        {
            return Err("unordered recorded clock".into());
        }
        available.insert(s.tick);
        for lane in [&s.switched, &s.reference] {
            if !lane.normal_impulse_ns.is_finite() || lane.normal_impulse_ns < 0. {
                return Err("invalid solved reaction".into());
            }
            let j = corners(lane)?;
            if (j.iter().sum::<f64>() - lane.normal_impulse_ns).abs() > 1e-8 {
                return Err(
                    format!("point sum differs from native reaction at tick {}", s.tick).into(),
                );
            }
            full = j.into_iter().fold(full, f64::max);
            min_z = min_z.min(lane.pad.position[2] - trace.geometry.pad_half_extents[2]);
        }
        if s.switched.normal_impulse_ns == 0. && s.switched.vertical_gap_m > flight.1 {
            flight = (s.tick, s.switched.vertical_gap_m);
        }
    }
    if trace.cycles.len() != 3 || full <= 0. || !available.contains(&0) {
        return Err("incomplete lift trial".into());
    }
    let mut captures = BTreeMap::from([("airborne".to_string(), flight.0)]);
    // The producer already retains every native solve around each landing and
    // decimates quiet travel. Keep every supplied observation in the archive.
    let ticks = available;
    for c in &trace.cycles {
        for (n, l) in [
            ("switched", &c.switched_landing),
            ("reference", &c.reference_landing),
        ] {
            let index = trace
                .frames
                .binary_search_by_key(&l.tick, |s| s.tick)
                .map_err(|_| "landing sample absent")?;
            let s = &trace.frames[index];
            let actual = if n == "switched" {
                &s.switched
            } else {
                &s.reference
            };
            if (s.time_s - l.time_s).abs() > 1e-10
                || (actual.normal_impulse_ns - l.first_ns).abs() > 1e-10
                || l.first_ns <= 0.
            {
                return Err("landing summary disagrees with native trace".into());
            }
        }
        captures.insert(format!("landing_{}", c.cycle), c.switched_landing.tick);
    }
    let third = &trace.cycles[2];
    captures.insert("third_second_step".into(), third.switched_landing.tick + 1);
    if third.switched_landing.tick != third.reference_landing.tick {
        captures.insert("reference_landing_3".into(), third.reference_landing.tick);
    }
    for (label, events) in [
        ("switched", &third.switched_recontacts),
        ("reference", &third.reference_recontacts),
    ] {
        for (index, event) in events.iter().enumerate() {
            captures.insert(format!("{label}_reload_{}", index + 1), event.tick);
        }
    }
    if !captures.values().all(|t| ticks.contains(t)) {
        return Err("impact capture absent".into());
    }
    let spacing = 1.25 * width;
    let sole_z = min_z - 0.26 * width;
    let [hx, hy, _] = trace.geometry.foot_half_extents;
    let mut archive = Vec::new();
    for s in &trace.frames {
        let mut f = Frame::default();
        for (k, (label, lane, offset)) in [
            ("switched", &s.switched, -spacing / 2.),
            ("reference", &s.reference, spacing / 2.),
        ]
        .into_iter()
        .enumerate()
        {
            LaneDisplay {
                label,
                color: LANE[k],
                offset,
                sole_z,
                full,
            }
            .add(
                &mut f,
                &trace.geometry,
                [&lane.foot, &lane.upper, &lane.pad],
                corners(lane)?,
            )?;
            let z = sole_z - hy * 1.62;
            let ends = [offset - hx, offset + hx];
            f.edge(
                &format!("{label}_tier_scale"),
                DVec3::new(0., ends[0], z),
                DVec3::new(0., ends[1], z),
                2. * hx,
                0.004,
                IDLE,
            );
            let dynamic = k == 1 || !s.prescribed;
            f.add(
                &format!("{label}_tier"),
                DVec3::new(0.011, ends[usize::from(dynamic)], z),
                DQuat::IDENTITY,
                cuboid([0.008; 3]),
                LANE[k],
            );
        }
        archive.push(f.finish(
            Duration::from_nanos(s.tick.checked_mul(dt).ok_or("clock overflow")?),
            archive.len() + 1,
        ));
    }
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
    fs::write(
        out.join("frames.json"),
        serde_json::to_vec_pretty(
            &json!({"kind":"landing-audit-v1","dt_ns":dt,"ticks":ticks,"captures":captures,"spacing":spacing,"sole_z":sole_z,"full_corner_ns":full,"trace_file":trace_path.file_name().ok_or("no trace name")?.to_str().ok_or("non UTF-8 trace name")?}),
        )?,
    )?;
    Ok(())
}

fn main() -> Result<()> {
    match Args::parse().command {
        Command::Replay { trace, output } => export(&trace, &output),
        Command::Figures { audit, output } => figures::draw(&audit, &output),
        Command::Touchdown {
            traces,
            comparison,
            output,
        } => touchdown::export(&traces, comparison, &output),
        Command::Energy {
            audit,
            comparison,
            output,
        } => contact_energy::export(&audit, comparison, &output),
        Command::Rebound {
            audit,
            deck,
            output,
            first_tick,
            last_tick,
            event_tick,
        } => rebound::export(&audit, deck, &output, first_tick, last_tick, event_tick),
        Command::ReboundBudget {
            audit,
            output,
            event_tick,
        } => rebound::budget::export(&audit, &output, event_tick),
        Command::ReboundFlight { audit, output } => rebound::flight::export(&audit, &output),
        Command::WeakRecontact { audit, output } => {
            rebound::weak::export(&audit, &output, rebound::weak::Study::Weak)
        }
        Command::MaterialFriction { audit, output } => material_friction::export(&audit, &output),
        Command::FeatureRelaxation {
            finite,
            relaxation_survey,
            default_survey,
            output,
        } => relaxation_survey::feature::export(
            &finite,
            &relaxation_survey,
            &default_survey,
            &output,
        ),
        Command::ContinuationSpans {
            audit,
            relaxation_survey,
            default_survey,
            output,
        } => relaxation_survey::spans::export(&audit, &relaxation_survey, &default_survey, &output),
        Command::RelaxationSurvey {
            audit,
            output,
            default_survey,
        } => relaxation_survey::export(&audit, &output, default_survey.as_deref()),
        Command::PrimarySurvey { audit, output } => primary_survey::export(&audit, &output),
        Command::NativeComparison { audit, output } => {
            rebound::weak::export(&audit, &output, rebound::weak::Study::Native)
        }
        Command::DampedLanding {
            reference,
            fixture,
            output,
        } => edge_compliance::free_landing::export_damped(&reference, &fixture, &output),
        Command::FreeLanding {
            reference,
            fixture,
            output,
        } => edge_compliance::free_landing::export(&reference, &fixture, &output),
        Command::CornerPreload {
            reference,
            control,
            fixture,
            output,
        } => edge_compliance::corner::export(&reference, &control, &fixture, &output),
        Command::EdgeCompliance {
            reference,
            fixture,
            output,
        } => edge_compliance::export(&reference, &fixture, &output),
        Command::EdgeRoll {
            previous,
            current,
            reference,
            previous_receipts,
            current_receipts,
            output,
        } => edge_slip::roll::export(
            &previous,
            &current,
            &reference,
            &previous_receipts,
            &current_receipts,
            &output,
        ),
        Command::EdgeSlip {
            previous,
            current,
            reference,
            receipt,
            timing,
            output,
        } => edge_slip::export(&previous, &current, &reference, &receipt, &timing, &output),
        Command::OuterDivergence { audit, output } => {
            rebound::weak::export(&audit, &output, rebound::weak::Study::Outer)
        }
        Command::EnergyFigures { audit, output } => contact_energy::figures(&audit, &output),
        Command::TouchdownFigures { audit, output } => touchdown::figures(&audit, &output),
    }
}
