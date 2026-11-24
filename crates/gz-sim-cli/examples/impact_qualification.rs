// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent readback of published local rigid-body qualification data.
use clap::{Parser, Subcommand, ValueEnum};
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    io::{BufReader, BufWriter, Write},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[path="impact_qualification/closure_cost.rs"]
mod closure_cost;
#[path = "impact_qualification/convergence.rs"]
mod convergence;
#[path = "impact_qualification/edge.rs"]
mod edge;
#[path = "impact_qualification/edge_compliance.rs"]
mod edge_compliance;
#[path = "impact_qualification/edge_restart.rs"]
mod edge_restart;
#[path = "impact_qualification/elastic_comparison.rs"]
mod elastic_comparison;
#[path = "impact_qualification/corner_load_audit.rs"]
mod corner_load_audit;
#[path = "impact_qualification/sole_load_audit.rs"]
mod sole_load_audit;
#[path = "impact_qualification/finite.rs"]
mod finite;
#[path = "impact_qualification/long.rs"]
mod long;
#[path = "impact_qualification/outgoing.rs"]
mod outgoing;
#[path="impact_qualification/sequence.rs"]
mod sequence;
#[path = "impact_qualification/pressure.rs"]
mod pressure;
#[path = "impact_qualification/propagation.rs"]
mod propagation;
#[path = "impact_qualification/registered.rs"]
mod registered;
#[path = "impact_qualification/transfer.rs"]
mod transfer;
#[derive(Parser)]
struct Args {
    #[command(subcommand)]
    mode: Mode,
}
#[derive(Subcommand)]
enum Mode {
    /// Sweep the sole damping from the same incoming state.
    DampedLanding {
        donor: PathBuf, undamped: PathBuf, output: PathBuf,
        #[arg(long, default_value_t = 0.3)] light_damping: f64,
        #[arg(long, default_value_t = 0.8)] stronger_damping: f64,
    },
    /// Infer normal damping from a centered flat-drop speed pair (negligible gravity during contact).
    FitFlatDrop { incoming_m_s: f64, outgoing_m_s: f64 },
    /// Compare whole-sole elastic landings starting from free approach, corner or line.
    FreeLanding { donor: PathBuf, corner: PathBuf, rigid: PathBuf, output: PathBuf },
    /// Compare launching the same elastic ridge at the corner and whole-line arrivals.
    CornerPreload { reference: PathBuf, corner_donor: PathBuf, late_material: PathBuf, output: PathBuf },
    /// Simulate two elastic layers on the finite foot edge from the matched rigid donor.
    EdgeCompliance { reference: PathBuf, output: PathBuf },
    /// Compare every saved field before and after the reduced-link observer correction.
    Observer {
        #[arg(value_enum)]
        schema: Schema,
        before: PathBuf,
        after: PathBuf,
        output: PathBuf,
    },
    /// Compare finite primary geometry and interventions against both historical 4,096-call controls.
    Finite {
        geometry: PathBuf,
        policy: PathBuf,
        grid: PathBuf,
        relaxed: PathBuf,
    },
    Sequence {directory:PathBuf,registered:PathBuf,previous:Option<PathBuf>},
    Launch {directory:PathBuf,spatial:PathBuf,mechanics:PathBuf},
    Spatial {directory:PathBuf,transfer:PathBuf,mechanics:PathBuf},
    InternalTransfer {directory:PathBuf,sliding:PathBuf,sticking:PathBuf,mechanics:PathBuf},
    Vertex {directory:PathBuf,policy:PathBuf,mechanics:PathBuf},
    SecondaryPolicy {directory:PathBuf,mechanics:PathBuf},
    ClosureCost {registered:PathBuf, baseline:PathBuf, output:PathBuf},
    Outgoing { directory:PathBuf,registered:PathBuf, previous:Option<PathBuf> },
    RegisteredWork {
        directory: PathBuf,
        tight: PathBuf,
        closure: PathBuf,
    },
    Wrench {
        directory: PathBuf,
        precise: PathBuf,
        registered: Option<PathBuf>,
    },
    Closure {
        directory: PathBuf,
        precise: PathBuf,
    },
    LineSticking {
        directory: PathBuf,
        precise: PathBuf,
    },
    RegisteredSecond {
        directory: PathBuf,
        mechanics: PathBuf,
        #[arg(long)]
        coarse: Option<PathBuf>,
    },
    SecondPoint {
        directory: PathBuf,
        arcs: PathBuf,
    },
    Transfer {
        directory: PathBuf,
        load: PathBuf,
    },
    TransferPolicy {
        directory: PathBuf,
        mechanics: PathBuf,
        primary: PathBuf,
        grid: PathBuf,
        relax: PathBuf,
    },
    FiniteLoad {
        directory: PathBuf,
        policy: PathBuf,
    },
    FiniteRevision {
        before: PathBuf,
        after: PathBuf,
    },
    /// Compare finite interventions to independent three-stroke runs.
    PhysicalHandoff {directory:PathBuf,episodes:PathBuf,whole:PathBuf,suffix:PathBuf},
    FineSuffix {directory:PathBuf,whole:PathBuf,deep:PathBuf},
    CompactLong {directory:PathBuf,short:PathBuf,deep:PathBuf,previous:PathBuf},
    WholeLong { directory: PathBuf, short: PathBuf, deep: PathBuf },
    /// Check full-sole loads and compare each trajectory with the recorded native runs.
    SoleLoadAudit { edge_roll: PathBuf, directory: PathBuf },
    /// Independently reconstruct dynamic contact pressure and check impulse and work.
    DampingAudit { free_landing: PathBuf, directory: PathBuf },
    /// Independently reconstruct the clipped ridge loads and compare matched impulse histories.
    CornerLoadAudit { rigid: PathBuf, late_material: PathBuf, directory: PathBuf },
    /// Measure material-dependent motion against archived native and rigid pressure runs.
    EdgeComplianceCompare { edge_roll: PathBuf, directory: PathBuf },
    EdgeSlip { before: PathBuf, after: PathBuf, output: PathBuf, #[arg(long)] field_primary: Option<PathBuf> },
    ColdField {directory:PathBuf,primary:PathBuf,episodes:PathBuf},
    ColdPoint {directory:PathBuf,episodes:PathBuf,observed:PathBuf},
    ActiveEpisode {directory:PathBuf,whole:PathBuf,previous:Option<PathBuf>},
    CompactPrimary {directory:PathBuf,subdivided:PathBuf},
    PrimaryChange {directory:PathBuf,baseline:PathBuf},
    WholePrimary {directory:PathBuf,previous:PathBuf,initial:PathBuf},
    FineOnset {directory:PathBuf,deep:PathBuf},
    SuccessorOnset {directory:PathBuf,episodes:PathBuf,suffix:PathBuf},
    InternalLong {directory:PathBuf,warm:PathBuf,cold:PathBuf,short:PathBuf},
    PhysicalSecondaryLong {directory:PathBuf,warm:PathBuf,clocked:PathBuf},
    SecondaryLong { directory:PathBuf, registered:PathBuf, deep:PathBuf },
    RetimedLong {
        directory: PathBuf,
        finite: PathBuf,
        propagation: PathBuf,
        convergence: PathBuf,
        deep: PathBuf,
    },
    FiniteLong {
        directory: PathBuf,
        propagation: PathBuf,
        convergence: PathBuf,
    },
    /// Reconstruct the continuous trajectories and verify the fine control repeats.
    Deep {
        directory: PathBuf,
        convergence: PathBuf,
        finite_long: PathBuf,
    },
    /// Compute same-suffix trajectory and subsequent landing discrepancies.
    Propagation {
        directory: PathBuf,
    },
    /// Summarize the independent SDF arrival-phase experiment.
    Arrivals {
        directory: PathBuf,
    },
    /// Validate all impulse and budget rows in the symmetric sole-patch census.
    Patch {
        directory: PathBuf,
    },
    /// Check sustained loading against signed reactions and an optional independently saved native control.
    EdgeFollow {
        directory: PathBuf,
        baseline: Option<PathBuf>,
    },
    /// Compare actual fractional impacts to the release and preceding-boundary ballistic controls.
    Ballistic {
        directory: PathBuf,
    },
    /// Check each three-stroke continuous control against the independent propagation campaign.
    Convergence {
        directory: PathBuf,
        propagation: PathBuf,
    },
}
#[derive(Clone, Copy, Debug, ValueEnum)]
enum Schema {
    Planar,
    Relaxation,
    Spatial,
}
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(BufReader::new(fs::File::open(p)?))?)
}
fn write(p: &Path, v: &Value) -> Result<()> {
    let mut w = BufWriter::new(fs::File::create(p)?);
    serde_json::to_writer(&mut w, v)?;
    w.flush()?;
    Ok(())
}
fn array(v: &Value) -> Result<&[Value]> {
    v.as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| "missing array".into())
}
fn num(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "missing finite measurement".into())
}
fn uint(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "missing unsigned count".into())
}
#[derive(Default)]
struct Series(Vec<(f64, usize)>);
impl Series {
    fn push(&mut self, x: f64, k: usize) {
        self.0.push((x, k));
    }
    fn stats(mut self) -> Value {
        let a = &mut self.0;
        if a.is_empty() {
            return Value::Null;
        }
        a.sort_unstable_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
        let q = |p: usize| a[(p * a.len()).div_ceil(100) - 1].0;
        json!({"n":a.len(),"mean":a.iter().map(|v|v.0).sum::<f64>() / a.len() as f64,"median":q(50),"p95":q(95),"p99":q(99),"max":a.last().unwrap().0,"max_case":a.last().unwrap().1})
    }
}
fn changes(
    a: &Value,
    b: &Value,
    schema: Schema,
    path: &mut Vec<String>,
    counts: &mut BTreeMap<String, u64>,
) -> Result<()> {
    if a == b {
        return Ok(());
    }
    let changed = match schema {
        Schema::Planar => ["first", "max_point_momentum_residual_ns"].as_slice(),
        Schema::Relaxation => [
            "max_post_closing_m_s",
            "impulse_weighted_abs_post_normal_m_s",
        ]
        .as_slice(),
        Schema::Spatial => [
            "slip_error_m_s",
            "foot_spin_error_rad_s",
            "relative_point_velocity_m_s",
            "max_native_momentum_residual_ns",
            "impulse_error_ns",
        ]
        .as_slice(),
    };
    if let Some(field) = path.last()
        && changed.contains(&field.as_str())
    {
        *counts.entry(field.clone()).or_default() += 1;
        return Ok(());
    }
    match (a, b) {
        (Value::Object(x), Value::Object(y)) if x.keys().eq(y.keys()) => {
            for (k, v) in x {
                path.push(k.clone());
                changes(v, &y[k], schema, path, counts)?;
                path.pop();
            }
        }
        (Value::Array(x), Value::Array(y)) if x.len() == y.len() => {
            path.push("[]".into());
            for (v, w) in x.iter().zip(y) {
                changes(v, w, schema, path, counts)?;
            }
            path.pop();
        }
        _ => {
            return Err(format!("unexpected data change at {}: {a} vs {b}", path.join(".")).into());
        }
    }
    Ok(())
}
fn observer(schema: Schema, old: &Path, new: &Path, dest: &Path) -> Result<()> {
    let files = if old.is_file() {
        vec![(old.to_owned(), new.to_owned())]
    } else {
        let mut rows = vec![];
        for entry in fs::read_dir(old)? {
            let f = entry?;
            if f.path().extension().is_some_and(|e| e == "json") {
                rows.push((f.path(), new.join(f.file_name())));
            }
        }
        rows.sort();
        rows
    };
    if files.is_empty() {
        return Err("no archived samples".into());
    }
    let mut counts = BTreeMap::new();
    for (a, b) in &files {
        changes(&read(a)?, &read(b)?, schema, &mut vec![], &mut counts)
            .map_err(|e| format!("{}: {e}", a.display()))?;
    }
    write(
        dest,
        &json!({"schema":format!("{schema:?}"),"files":files.len(),"all_other_fields_identical":true,"changed_diagnostics":counts}),
    )
}
fn arrivals(dir: &Path) -> Result<()> {
    let doc = read(&dir.join("study.json"))?;
    let rows = array(&doc["rows"])?;
    let mut groups: BTreeMap<(String, &str, &str), Series> = BTreeMap::new();
    let mut projected = 0;
    let mut missed = vec![];
    let mut early = 0;
    let mut unaff = 0;
    for (k, row) in rows.iter().enumerate() {
        let at = uint(&row["prefix"])? as f64 * 500_000. + num(&row["phase_ns"])?;
        let p = &row["projected"];
        match uint(&p["projected"])? {
            1 => projected += 1,
            0 => missed.push(k),
            _ => return Err("duplicate projection".into()),
        }
        for kind in ["ordinary", "projected"] {
            let r = &row[kind];
            if num(&r["prior_reaction_ns"])? > 0. {
                if kind == "ordinary" {
                    early += 1;
                } else {
                    unaff += 1;
                }
            }
            for group in [
                "all".into(),
                format!("prefix_{}", row["prefix"]),
                format!("rotation_{}", row["rotation"]),
                format!("mu_{}", row["mu"]),
            ] {
                for (field, v) in [
                    (
                        "impact_interval_error_ns",
                        Some(num(&r["first_interval_impulse_error_ns"])?),
                    ),
                    ("predictor_and_step_cost_ns", r["impact_tick_ns"].as_f64()),
                    (
                        "first_loaded_time_abs_error_ns",
                        r["first_load_ns"].as_u64().map(|t| (t as f64 - at).abs()),
                    ),
                    (
                        "projected_time_abs_error_ns",
                        r["projection_ns"].as_u64().map(|t| (t as f64 - at).abs()),
                    ),
                    (
                        "projected_vector_error_ns",
                        r["projection_error_ns"].as_f64(),
                    ),
                ] {
                    if let Some(v) = v {
                        groups
                            .entry((group.clone(), kind, field))
                            .or_default()
                            .push(v, k);
                    }
                }
            }
        }
    }
    let stats=groups.into_iter().map(|((group,path,field),v)|json!({"group":group,"path":path,"measurement":field,"statistics":v.stats()})).collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"cases":rows.len(),"projected_once":projected,"no_projection_indices":missed,"ordinary_prior_positive":early,"projected_prior_positive":unaff,"rows":stats}),
    )
}
fn main() -> Result<()> {
    match Args::parse().mode {
        Mode::DampedLanding {donor,undamped,output,light_damping,stronger_damping} => edge_compliance::free_landing::damped_landing::study(&donor,&undamped,&output,light_damping,stronger_damping),
        Mode::FitFlatDrop { incoming_m_s, outgoing_m_s } => {
            let damping = gz_physics::DampedAreaFoundation::coefficient_from_flat_impact(incoming_m_s, outgoing_m_s)
                .ok_or("expected finite normal speeds with 0 < outgoing <= incoming and a representable coefficient")?;
            println!("{}", json!({"incoming_m_s":incoming_m_s,"outgoing_m_s":outgoing_m_s,
                "restitution":outgoing_m_s/incoming_m_s,"normal_damping_s_m":damping}));
            Ok(())
        }
        Mode::FreeLanding {donor,corner,rigid,output} => edge_compliance::free_landing::study(&donor,&corner,&rigid,&output),
        Mode::CornerPreload {reference,corner_donor,late_material,output} => edge_compliance::corner_preload::study(&reference,&corner_donor,&late_material,&output),
        Mode::EdgeCompliance { reference, output } => edge_compliance::study(&reference, &output),
        Mode::Observer {
            schema,
            before,
            after,
            output,
        } => observer(schema, &before, &after, &output),
        Mode::Finite {
            geometry,
            policy,
            grid,
            relaxed,
        } => finite::summarize(&geometry, &policy, &grid, &relaxed),
        Mode::EdgeFollow {
            directory,
            baseline,
        } => edge::summarize(&directory, baseline.as_deref()),
        Mode::Patch { directory } => registered::patch(&directory),
        Mode::Ballistic { directory } => registered::ballistic(&directory),
        Mode::Convergence {
            directory,
            propagation,
        } => convergence::summarize(&directory, &propagation),
        Mode::Launch{directory,spatial,mechanics}=>sequence::internal::launch(&directory,&spatial,&mechanics),
        Mode::Spatial{directory,transfer,mechanics}=>sequence::internal::spatial(&directory,&transfer,&mechanics),
        Mode::InternalTransfer{directory,sliding,sticking,mechanics}=>sequence::internal::summarize(&directory,&sliding,&sticking,&mechanics),
        Mode::Vertex{directory,policy,mechanics}=>sequence::vertex::summarize(&directory,&policy,&mechanics),
        Mode::SecondaryPolicy{directory,mechanics}=>sequence::policy(&directory,&mechanics),
        Mode::Sequence{directory,registered,previous}=>sequence::summarize(&directory,&registered,previous.as_deref()),
        Mode::ClosureCost{registered,baseline,output}=>closure_cost::summarize(&registered,&baseline,&output),
        Mode::Outgoing{directory,registered,previous}=>outgoing::summarize(&directory,&registered,previous.as_deref()),
        Mode::RegisteredWork {
            directory,
            tight,
            closure,
        } => pressure::registered(&directory, &tight, &closure),
        Mode::Wrench { directory, precise, registered } => pressure::wrench(&directory, &precise, registered.as_deref()),
        Mode::Closure { directory, precise } => pressure::closure(&directory, &precise),
        Mode::LineSticking { directory, precise } => transfer::line_sticking(&directory, &precise),
        Mode::RegisteredSecond {
            directory,
            mechanics,
            coarse,
        } => transfer::registered(&directory, &mechanics, coarse.as_deref()),
        Mode::SecondPoint { directory, arcs } => transfer::second(&directory, &arcs),
        Mode::Transfer { directory, load } => transfer::mechanics(&directory, &load),
        Mode::TransferPolicy {
            directory,
            mechanics,
            primary,
            grid,
            relax,
        } => transfer::policy(&directory, &mechanics, &primary, &grid, &relax),
        Mode::FiniteLoad { directory, policy } => finite::post_load(&directory, &policy),
        Mode::FiniteRevision { before, after } => finite::compare_revisions(&before, &after),
        Mode::PhysicalHandoff{directory,episodes,whole,suffix}=>long::handoff::summarize(&directory,&episodes,&whole,&suffix),
        Mode::FineSuffix {directory,whole,deep}=>long::suffix::summarize(&directory,&whole,&deep),
        Mode::WholeLong {directory,short,deep} => long::whole::summarize(&directory,&short,&deep,None),
        Mode::CompactLong {directory,short,deep,previous} => long::whole::summarize(&directory,&short,&deep,Some(&previous)),
        Mode::DampingAudit { free_landing, directory } => sole_load_audit::damping::run(&free_landing, &directory),
        Mode::SoleLoadAudit { edge_roll, directory } => sole_load_audit::run(&edge_roll, &directory),
        Mode::CornerLoadAudit { rigid, late_material, directory } => corner_load_audit::run(&rigid, &late_material, &directory),
        Mode::EdgeComplianceCompare {edge_roll,directory}=>elastic_comparison::compare(&edge_roll,&directory),
        Mode::EdgeSlip {before,after,output,field_primary}=>edge_restart::compare(&before,&after,&output,field_primary.as_deref()),
        Mode::ColdField{directory,primary,episodes}=>sequence::cold_point::field(&directory,&primary,&episodes),
        Mode::ColdPoint{directory,episodes,observed}=>sequence::cold_point::summarize(&directory,&episodes,&observed),
        Mode::ActiveEpisode{directory,whole,previous}=>sequence::episode::summarize(&directory,&whole,previous.as_deref()),
        Mode::CompactPrimary{directory,subdivided}=>sequence::compact::summarize(&directory,&subdivided),
        Mode::PrimaryChange{directory,baseline}=>sequence::primary_change::summarize(&directory,&baseline),
        Mode::WholePrimary{directory,previous,initial}=>sequence::whole_primary::summarize(&directory,&previous,&initial),
        Mode::SuccessorOnset{directory,episodes,suffix}=>long::fine_onset::successor(&directory,&episodes,&suffix),
        Mode::FineOnset{directory,deep}=>long::fine_onset::summarize(&directory,&deep),
        Mode::InternalLong{directory,warm,cold,short}=>long::internal::summarize(&directory,&warm,&cold,&short),
        Mode::PhysicalSecondaryLong{directory,warm,clocked}=>long::physical::summarize(&directory,&warm,&clocked),
        Mode::SecondaryLong{directory,registered,deep}=>long::secondary::summarize(&directory,&registered,&deep),
        Mode::RetimedLong {
            directory,
            finite,
            propagation,
            convergence,
            deep,
        } => long::retimed(&directory, &finite, &propagation, &convergence, &deep),
        Mode::FiniteLong {
            directory,
            propagation,
            convergence,
        } => long::finite(&directory, &propagation, &convergence),
        Mode::Deep {
            directory,
            convergence,
            finite_long,
        } => long::deep(&directory, &convergence, &finite_long),
        Mode::Propagation { directory } => propagation::summarize(&directory),
        Mode::Arrivals { directory } => arrivals(&directory),
    }
}
