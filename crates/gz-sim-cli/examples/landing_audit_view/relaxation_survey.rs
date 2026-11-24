// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched comparisons of fixed-time relaxation and collision refresh at primary landings.
use super::*;
use serde::Serialize;
use std::io::{BufWriter, Write};

#[path = "relaxation_continuation.rs"]
mod continuation;
#[path = "feature_relaxation.rs"]
pub(super) mod feature;
#[path = "relaxation_survey_figures.rs"]
mod figures;
#[path = "relaxation_signature.rs"]
mod signature;
#[path = "continuation_spans.rs"]
pub(super) mod spans;
#[path = "relaxation_strata.rs"]
mod strata;

const FORMAT: &str = "frictional-primary-relaxation-v1";
const WINDOWS: [u64; 3] = [1, 10, 26];
const LABELS: [&str; 3] = ["primary", "at_5ms", "at_13ms"];
const FIXTURE_COUNT: usize = 1250;
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
struct Profile {
    calls: u32,
    relaxation: u32,
    onset_only: bool,
}
impl Profile {
    const fn uniform(calls: u32, relaxation: u32) -> Self {
        Self {
            calls,
            relaxation,
            onset_only: false,
        }
    }
    const fn onset(calls: u32) -> Self {
        Self {
            calls,
            relaxation: 8,
            onset_only: true,
        }
    }
    fn label(self) -> String {
        format!(
            "{}x4_r{}{}",
            self.calls,
            self.relaxation,
            if self.onset_only { "_onset" } else { "" }
        )
    }
}
const BASE: Profile = Profile::uniform(8, 1);
const CAND: Profile = Profile::uniform(8, 8);
const FINE: Profile = Profile::uniform(4096, 8);
const ALT: Profile = Profile::uniform(4096, 1);
const PROFILES: [Profile; 14] = [
    BASE,
    Profile::uniform(8, 4),
    CAND,
    Profile::uniform(8, 16),
    Profile::onset(8),
    Profile::uniform(16, 1),
    Profile::uniform(16, 8),
    Profile::onset(16),
    Profile::uniform(64, 8),
    Profile::uniform(256, 8),
    Profile::uniform(1024, 8),
    FINE,
    Profile::uniform(256, 16),
    Profile::uniform(1024, 16),
];
const SENSITIVITY: [(Profile, Profile); 4] = [
    (Profile::uniform(256, 8), Profile::uniform(1024, 8)),
    (Profile::uniform(1024, 8), FINE),
    (Profile::uniform(1024, 8), Profile::uniform(1024, 16)),
    (ALT, FINE),
];
const CANDIDATES: [Profile; 3] = [CAND, Profile::uniform(8, 16), Profile::onset(8)];
#[derive(Deserialize)]
struct Batch {
    format: String,
    cases: Vec<RawCase>,
}
#[derive(Deserialize)]
struct Saved<T> {
    format: String,
    index: usize,
    outer_us: u32,
    horizon: u32,
    result: T,
}
#[derive(Deserialize, PartialEq, Clone)]
struct Fixture {
    name: String,
    offset_m: f64,
    speed_scale: f64,
    switched: bool,
}
#[derive(Deserialize)]
struct RawCase {
    fixture: Fixture,
    first_disjoint_tick: Option<u64>,
    trials: Vec<Trial>,
}
#[derive(Deserialize)]
struct Trial {
    stroke: usize,
    cut: Value,
    paths: Vec<RawPath>,
}
#[derive(Deserialize)]
struct RawPath {
    profile: Profile,
    first: Option<First>,
    first_disjoint_tick: Option<u64>,
    samples: Vec<Endpoint>,
}
#[derive(Clone, Copy, Deserialize, PartialEq)]
struct First {
    end_ns: u64,
    normal_ns: f64,
}
#[derive(Clone, Copy, Deserialize, PartialEq)]
struct Endpoint {
    i: u64,
    com_m: [f64; 3],
    velocity_m_s: [f64; 3],
    spin_rad_s: [f64; 3],
    orientation_xyzw: [f64; 4],
    cumulative_corners_ns: [f64; 4],
    cumulative_normal_ns: f64,
    cumulative_tangent_ns: [f64; 3],
    normal_ns: f64,
    impulse_weighted_abs_post_normal_m_s: Option<f64>,
    footprint_gap_m: Option<f64>,
    positive_point_count: Option<usize>,
    max_post_closing_m_s: Option<f64>,
}
#[derive(Deserialize)]
struct OriginalCase {
    fixture: Fixture,
    first_disjoint_tick: Option<u64>,
    trials: Vec<OriginalTrial>,
}
#[derive(Deserialize)]
struct OriginalTrial {
    stroke: usize,
    cut: Value,
    paths: Vec<OriginalPath>,
}
#[derive(Deserialize)]
struct OriginalPath {
    method: String,
    calls: Option<u32>,
    iterations: Option<u32>,
    record: OriginalSamples,
}
#[derive(Deserialize)]
struct OriginalSamples {
    first_loaded_native_end_ns: Option<u64>,
    samples: Vec<Endpoint>,
}
#[derive(Clone, Copy)]
struct Observation {
    second: Endpoint,
    ends: [Endpoint; 3],
    overlap_lost: Option<u64>,
}
impl Observation {
    fn parse(
        samples: &[Endpoint],
        overlap_lost: Option<u64>,
        observed_residuals: bool,
    ) -> Result<Self> {
        let get = |tick| -> Result<Endpoint> {
            let mut matches = samples.iter().filter(|s| s.i == tick);
            let &s = matches.next().ok_or("missing relaxation endpoint")?;
            if matches.next().is_some()
                || !DVec3::from(s.com_m).is_finite()
                || !DVec3::from(s.cumulative_tangent_ns).is_finite()
                || !s.normal_ns.is_finite()
                || s.normal_ns < 0.
                || !s.cumulative_normal_ns.is_finite()
                || s.cumulative_normal_ns < 0.
                || s.impulse_weighted_abs_post_normal_m_s
                    .is_some_and(|r| !r.is_finite() || r < 0.)
                || observed_residuals
                    && (s.normal_ns > 0.) != s.impulse_weighted_abs_post_normal_m_s.is_some()
            {
                return Err("invalid relaxation endpoint".into());
            }
            Ok(s)
        };
        let [one, five, end] = WINDOWS.map(get);
        Ok(Self {
            second: get(2)?,
            ends: [one?, five?, end?],
            overlap_lost,
        })
    }
}
#[derive(Clone, Copy, Serialize)]
struct Site {
    index: usize,
    offset_mm: u16,
    drive_percent: u16,
    holds: bool,
    stroke: usize,
    cut_tick: u64,
}
struct Cell {
    cut: Value,
    baseline_first: Option<u64>,
    site: Site,
    first_receipt: continuation::Receipt,
    incoming: Option<signature::Signature>,
    paths: BTreeMap<Profile, Observation>,
}
#[derive(Clone, Copy, Serialize)]
struct Pair {
    com_mm: f64,
    cumulative_normal_delta_ns: f64,
    cumulative_tangent_distance_ns: f64,
    endpoint_normal_delta_ns: f64,
    weighted_residual_delta_m_s: Option<f64>,
}
#[derive(Clone, Copy, PartialEq)]
enum Missing {
    DefaultPending,
    Disjoint,
}
impl Cell {
    fn endpoint(&self, p: Profile, w: usize) -> std::result::Result<Endpoint, Missing> {
        let o = self.paths.get(&p).ok_or(Missing::DefaultPending)?;
        if o.overlap_lost
            .is_some_and(|tick| tick <= self.site.cut_tick + WINDOWS[w])
        {
            return Err(Missing::Disjoint);
        }
        Ok(o.ends[w])
    }
    fn pair(&self, a: Profile, b: Profile, w: usize) -> std::result::Result<Pair, Missing> {
        let a = self.endpoint(a, w)?;
        let b = self.endpoint(b, w)?;
        Ok(Pair {
            com_mm: DVec3::from(a.com_m).distance(DVec3::from(b.com_m)) * 1000.,
            cumulative_normal_delta_ns: a.cumulative_normal_ns - b.cumulative_normal_ns,
            cumulative_tangent_distance_ns: DVec3::from(a.cumulative_tangent_ns)
                .distance(DVec3::from(b.cumulative_tangent_ns)),
            endpoint_normal_delta_ns: a.normal_ns - b.normal_ns,
            weighted_residual_delta_m_s: a
                .impulse_weighted_abs_post_normal_m_s
                .zip(b.impulse_weighted_abs_post_normal_m_s)
                .map(|(a, b)| a - b),
        })
    }
    fn change(
        &self,
        p: Profile,
        reference: Profile,
        w: usize,
    ) -> std::result::Result<f64, Missing> {
        Ok(self.pair(p, reference, w)?.com_mm - self.pair(BASE, reference, w)?.com_mm)
    }
}
struct Census {
    study: bool,
    scheduled: usize,
    completed: BTreeSet<usize>,
    matched_default: BTreeSet<usize>,
    donor_disjoint: BTreeSet<usize>,
    crosschecked_native_primaries: usize,
    rounded_native_primaries: Vec<(usize, usize, [f64; 4])>,
    loaded_shared_ticks: usize,
    cells: Vec<Cell>,
}
fn load_file<T: DeserializeOwned>(p: &Path) -> Result<Option<T>> {
    match fs::File::open(p) {
        Ok(f) => Ok(Some(serde_json::from_reader(BufReader::new(f))?)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(e.into()),
    }
}
impl Census {
    fn load(dir: &Path, original: Option<&Path>) -> Result<Self> {
        let mut c = Self {
            study: original.is_none(),
            scheduled: if original.is_none() { 4 } else { FIXTURE_COUNT },
            completed: BTreeSet::new(),
            matched_default: BTreeSet::new(),
            donor_disjoint: BTreeSet::new(),
            crosschecked_native_primaries: 0,
            rounded_native_primaries: Vec::new(),
            loaded_shared_ticks: 0,
            cells: vec![],
        };
        if let Some(base) = original {
            for i in 0..FIXTURE_COUNT {
                let filename = format!("case-{i:04}.json");
                let Some(saved) = load_file::<Saved<RawCase>>(&dir.join("cases").join(&filename))?
                else {
                    continue;
                };
                if saved.format != FORMAT
                    || saved.index != i
                    || saved.outer_us != 500
                    || saved.horizon != 26
                {
                    return Err(format!("invalid relaxation fixture {i}").into());
                }
                let k = i * 613 % FIXTURE_COUNT;
                let f = &saved.result.fixture;
                if f.offset_m != (165 + 5 * (k / 50)) as f64 / 1000.
                    || f.speed_scale != (76 + 2 * ((k / 2) % 25)) as f64 / 100.
                    || f.switched != (k % 2 == 1)
                {
                    return Err("relaxation fixture coordinate mismatch".into());
                }
                let old = load_file::<Saved<OriginalCase>>(&base.join("cases").join(filename))?;
                if let Some(ref o) = old
                    && (o.format != "outer-primary-survey-v1"
                        || o.index != i
                        || o.outer_us != 500
                        || o.horizon != 26
                        || o.result.fixture != *f
                        || o.result.first_disjoint_tick != saved.result.first_disjoint_tick
                        || o.result.trials.len() != saved.result.trials.len())
                {
                    return Err(format!("unmatched original fixture {i}").into());
                }
                c.add(i, saved.result, old.as_ref().map(|o| &o.result))?;
            }
        } else {
            let study: Batch = read(&dir.join("study.json"))?;
            if study.format != FORMAT || study.cases.len() != 4 {
                return Err("invalid four-fixture relaxation study".into());
            }
            for (i, case) in study.cases.into_iter().enumerate() {
                c.add(i, case, None)?;
            }
        }
        if c.completed.is_empty() {
            return Err("no completed relaxation fixtures".into());
        }
        Ok(c)
    }
    fn add(&mut self, index: usize, case: RawCase, old: Option<&OriginalCase>) -> Result<()> {
        let f = case.fixture;
        self.completed.insert(index);
        if case.first_disjoint_tick.is_some() {
            self.donor_disjoint.insert(index);
        }
        let mut strokes = BTreeSet::new();
        for t in case.trials {
            if !(1..=3).contains(&t.stroke) || !strokes.insert(t.stroke) {
                return Err("bad relaxation stroke".into());
            }
            let cut = t.cut["tick"].as_u64().ok_or("missing cut tick")?;
            let first_receipt = continuation::shared_receipt(&t.paths, cut)
                .map_err(|e| format!("fixture {index} stroke {}: {e}", t.stroke))?;
            let mut paths = BTreeMap::new();
            let mut baseline_first = None;
            for p in t.paths {
                if p.profile == BASE {
                    baseline_first = p.first.as_ref().map(|r| r.end_ns);
                }
                let o = Observation::parse(&p.samples, p.first_disjoint_tick, true)?;
                if paths.insert(p.profile, o).is_some() {
                    return Err("duplicate profile".into());
                }
            }
            let expected = PROFILES
                .into_iter()
                .chain(self.study.then_some(ALT))
                .collect::<BTreeSet<_>>();
            if paths.keys().copied().collect::<BTreeSet<_>>() != expected {
                return Err(format!(
                    "unexpected profile set in fixture {index}, stroke {}",
                    t.stroke
                )
                .into());
            }
            if let Some(c) = old {
                let o = c
                    .trials
                    .iter()
                    .find(|r| r.stroke == t.stroke)
                    .ok_or("missing corresponding default-grid stroke")?;
                if o.cut != t.cut {
                    return Err(format!(
                        "default grid did not start from identical native cut {index}/{}",
                        t.stroke
                    )
                    .into());
                }
                // Both experiment drivers also advance the very first tick at 8 × 4, r1.
                // Check one full solved state and its reactions before comparing high grids.
                let baseline = o
                    .paths
                    .iter()
                    .find(|p| p.method == "onset" && p.calls == Some(8) && p.iterations == Some(4))
                    .ok_or("missing shared eight-solve baseline")?;
                let measured = paths[&BASE].ends[0];
                let previous = baseline
                    .record
                    .samples
                    .iter()
                    .find(|e| e.i == 1)
                    .ok_or("missing first historical baseline tick")?;
                // After this shared interval the onset-only driver switches to
                // adaptive stepping. A first touch on a later tick can differ.
                let shared_load = baseline_first.filter(|t| *t <= 500_000);
                let previous_shared_load = baseline
                    .record
                    .first_loaded_native_end_ns
                    .filter(|t| *t <= 500_000);
                // The Bevy readback and the raw native-backend readback can renormalize a
                // nearly unit quaternion differently. Record non-bitwise transform/twist
                // components, with strict equality on every integrated contact quantity.
                let mut rounded = [0.; 4];
                let mut outside = false;
                for (idx, (a, b)) in [
                    (measured.com_m.as_slice(), previous.com_m.as_slice()),
                    (&measured.velocity_m_s, &previous.velocity_m_s),
                    (&measured.spin_rad_s, &previous.spin_rad_s),
                    (&measured.orientation_xyzw, &previous.orientation_xyzw),
                ]
                .into_iter()
                .enumerate()
                {
                    for (&x, &y) in a.iter().zip(b) {
                        let delta = (x - y).abs();
                        rounded[idx] = f64::max(rounded[idx], delta);
                        outside |= !x.is_finite()
                            || !y.is_finite()
                            || delta > 8. * f64::EPSILON * x.abs().max(y.abs()).max(1.);
                    }
                }
                if rounded.iter().any(|&v| v > 0.) {
                    self.rounded_native_primaries
                        .push((index, t.stroke, rounded));
                }
                if shared_load != previous_shared_load
                    || shared_load.is_some() != (measured.cumulative_normal_ns > 0.)
                    || outside
                    || measured.cumulative_corners_ns != previous.cumulative_corners_ns
                    || measured.cumulative_tangent_ns != previous.cumulative_tangent_ns
                    || measured.cumulative_normal_ns != previous.cumulative_normal_ns
                    || measured.normal_ns != previous.normal_ns
                {
                    return Err(format!(
                        "independent drivers disagree on shared 8 × 4 primary {index}/{}",
                        t.stroke
                    )
                    .into());
                }
                self.crosschecked_native_primaries += 1;
                self.loaded_shared_ticks += usize::from(shared_load.is_some());
                let mut matching = o.paths.iter().filter(|p| {
                    p.method == "uniform" && p.calls == Some(4096) && p.iterations == Some(4)
                });
                let p = matching.next().ok_or("missing historical 4096 path")?;
                if matching.next().is_some() {
                    return Err("duplicate historical default grid".into());
                }
                paths.insert(ALT, Observation::parse(&p.record.samples, None, false)?);
            }
            self.cells.push(Cell {
                first_receipt,
                incoming: signature::recover(&t.cut, f.offset_m)?,
                cut: t.cut,
                baseline_first,
                site: Site {
                    index,
                    offset_mm: (f.offset_m * 1000.).round() as u16,
                    drive_percent: (f.speed_scale * 100.).round() as u16,
                    holds: f.switched,
                    stroke: t.stroke,
                    cut_tick: cut,
                },
                paths,
            });
        }
        if self.study || old.is_some() {
            self.matched_default.insert(index);
        }
        Ok(())
    }
}
fn stats(mut data: Vec<f64>) -> Value {
    data.sort_unstable_by(f64::total_cmp);
    let n = data.len();
    let neg = data.partition_point(|x| *x < 0.);
    let pos = n - data.partition_point(|x| *x <= 0.);
    let q = |pct: usize| data.get((pct * n).div_ceil(100).saturating_sub(1));
    json!({"n":n,"negative":neg,"zero":n-neg-pos,"positive":pos,"min":data.first(),"p50":q(50),"p95":q(95),"max":data.last(),"mean":(!data.is_empty()).then(||data.iter().sum::<f64>()/n as f64)})
}
fn comparison(census: &Census, a: Profile, b: Profile, w: usize) -> Value {
    let mut absent = 0;
    let mut disjoint = 0;
    let pairs = census
        .cells
        .iter()
        .filter_map(|c| match c.pair(a, b, w) {
            Ok(s) => Some(s),
            Err(Missing::Disjoint) => {
                disjoint += 1;
                None
            }
            Err(Missing::DefaultPending) => {
                absent += 1;
                None
            }
        })
        .collect::<Vec<_>>();
    let absolute = census
        .cells
        .iter()
        .filter_map(|c| c.pair(a, b, w).ok().and_then(|_| c.endpoint(a, w).ok()))
        .collect::<Vec<_>>();
    let sum_normal: f64 = absolute
        .iter()
        .filter(|s| s.impulse_weighted_abs_post_normal_m_s.is_some())
        .map(|s| s.normal_ns)
        .sum();
    let weighted: f64 = absolute
        .iter()
        .filter_map(|s| {
            s.impulse_weighted_abs_post_normal_m_s
                .map(|r| r * s.normal_ns)
        })
        .sum();
    json!({"matched":pairs.len(),"default_pending":absent,"disjoint":disjoint,
        "com_mm":stats(pairs.iter().map(|p|p.com_mm).collect()),
        "cumulative_normal_delta_ns":stats(pairs.iter().map(|p|p.cumulative_normal_delta_ns).collect()),
        "cumulative_normal_abs_delta_ns":stats(pairs.iter().map(|p|p.cumulative_normal_delta_ns.abs()).collect()),
        "cumulative_tangent_distance_ns":stats(pairs.iter().map(|p|p.cumulative_tangent_distance_ns).collect()),
        "endpoint_normal_delta_ns":stats(pairs.iter().map(|p|p.endpoint_normal_delta_ns).collect()),
        "endpoint_normal_ns":stats(absolute.iter().map(|s|s.normal_ns).collect()),
        "cumulative_normal_ns":stats(absolute.iter().map(|s|s.cumulative_normal_ns).collect()),
        "endpoint_weighted_abs_post_normal_m_s":stats(absolute.iter().filter_map(|s|s.impulse_weighted_abs_post_normal_m_s).collect()),
        "pooled_impulse_weighted_abs_post_normal_m_s":(sum_normal>0.).then(||weighted/sum_normal),
        "endpoint_weighted_residual_delta_m_s":stats(pairs.iter().filter_map(|s|s.weighted_residual_delta_m_s).collect())})
}
fn summarize_change(c: &Census, p: Profile, w: usize) -> Value {
    let mut both = vec![];
    let mut same_sign = 0;
    let mut opposite = 0;
    let mut zero = 0;
    for cell in &c.cells {
        if let (Ok(a), Ok(b)) = (cell.change(p, FINE, w), cell.change(p, ALT, w)) {
            both.push((a, b));
            if a == 0. || b == 0. {
                zero += 1;
            } else if (a > 0.) == (b > 0.) {
                same_sign += 1;
            } else {
                opposite += 1;
            }
        }
    }
    let changed = |reference| {
        stats(
            c.cells
                .iter()
                .filter_map(|cell| cell.change(p, reference, w).ok())
                .collect(),
        )
    };
    json!({"against_4096_r8_mm":changed(FINE),"against_4096_r1_mm":changed(ALT),
        "both_references":both.len(),"same_nonzero_sign":same_sign,"opposite_nonzero_sign":opposite,"one_or_both_tied":zero})
}
fn choose(c: &Census) -> Option<&Cell> {
    c.cells
        .iter()
        .filter_map(|x| x.change(CAND, FINE, 2).ok().map(|v| (x, v)))
        .max_by(|(a, av), (b, bv)| {
            av.total_cmp(bv)
                .then_with(|| b.site.index.cmp(&a.site.index))
                .then_with(|| b.site.stroke.cmp(&a.site.stroke))
        })
        .map(|(x, _)| x)
}
fn series(c: &Census, a: Profile, b: Profile) -> Value {
    Value::Object(
        LABELS
            .into_iter()
            .enumerate()
            .map(|(i, k)| (k.to_owned(), comparison(c, a, b, i)))
            .collect(),
    )
}
fn selection(c: &Census) -> Value {
    let Some(best) = choose(c) else {
        return Value::Null;
    };
    let compare = |r| json!({"baseline_mm":best.pair(BASE,r,2).ok().map(|p|p.com_mm),"candidate_mm":best.pair(CAND,r,2).ok().map(|p|p.com_mm),"change_mm":best.change(CAND,r,2).ok()});
    json!({"site":best.site,"against_4096_r8":compare(FINE),"against_4096_r1":compare(ALT)})
}
// Store direct case coordinates and raw-recomputable paired metrics alongside archived native paths.
fn cell_output(c: &Cell) -> Value {
    let values = |p, r| match c.pair(p, r, 0) {
        Ok(primary) => {
            json!({"primary":primary,"at_5ms":c.pair(p,r,1).ok(),"at_13ms":c.pair(p,r,2).ok()})
        }
        Err(Missing::Disjoint) => json!({"unavailable":"disjoint footprint"}),
        Err(Missing::DefaultPending) => json!({"unavailable":"default-grid file pending"}),
    };
    let mut change = serde_json::Map::new();
    for p in CANDIDATES {
        change.insert(
            p.label(),
            json!({"against_4096_r8_mm":(0..3).map(|i|c.change(p,FINE,i).ok()).collect::<Vec<_>>(),
            "against_4096_r1_mm":(0..3).map(|i|c.change(p,ALT,i).ok()).collect::<Vec<_>>()}),
        );
    }
    let sensitivity =
        SENSITIVITY.map(|(a, b)| json!({"from":a.label(),"to":b.label(),"metrics":values(a,b)}));
    json!({"site":c.site,"incoming_kinematics":c.incoming,"primary_residual_m_s":PROFILES.iter().filter_map(|p|c.endpoint(*p,0).ok().map(|e|(p.label(),e.impulse_weighted_abs_post_normal_m_s))).collect::<BTreeMap<_,_>>(),
        "baseline_to_refined_grid":values(BASE,FINE),"candidate_to_refined_grid":values(CAND,FINE),"signed_com_change_mm":change,"grid_and_relaxation_distances":sensitivity})
}
pub(super) fn export(dir: &Path, out: &Path, original: Option<&Path>) -> Result<()> {
    let c = Census::load(dir, original)?;
    fs::create_dir_all(out)?;
    let mut rows = BufWriter::new(fs::File::create(out.join("relaxation-cells.jsonl"))?);
    for cell in &c.cells {
        serde_json::to_writer(&mut rows, &cell_output(cell))?;
        rows.write_all(b"\n")?;
    }
    rows.flush()?;
    let policies=PROFILES.iter().map(|p|{
        (p.label(),json!({"profile":p,"against_4096_r8":series(&c,*p,FINE),"against_4096_r1":series(&c,*p,ALT)}))
    }).collect::<BTreeMap<_,_>>();
    let effects = CANDIDATES
        .iter()
        .map(|p| {
            (
                p.label(),
                Value::Object(
                    LABELS
                        .into_iter()
                        .enumerate()
                        .map(|(i, k)| (k.to_owned(), summarize_change(&c, *p, i)))
                        .collect(),
                ),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let sensitivity =
        SENSITIVITY.map(|(a, b)| json!({"from":a.label(),"to":b.label(),"metrics":series(&c,a,b)}));
    let result = json!({"schema":"frictional-relaxation-summary-v1","study":c.study,"scheduled_fixtures":c.scheduled,"completed_fixtures":c.completed.len(),"completed_indices":c.completed,
        "pending_fixtures":c.scheduled-c.completed.len(),"original_grid_fixtures_joined":c.matched_default.len(),"identical_native_primary_outcomes_across_drivers":c.crosschecked_native_primaries-c.rounded_native_primaries.len(),
        "crosschecked_native_primary_outcomes_across_drivers":c.crosschecked_native_primaries,
        "nonbitwise_pose_or_twist":c.rounded_native_primaries,"nonbitwise_fields_order":["com_m","velocity_m_s","spin_rad_s","orientation_xyzw"],
        "maximum_relative_or_absolute_component_error":8.*f64::EPSILON,"checked_shared_first_ticks_with_load":c.loaded_shared_ticks,"checked_shared_first_ticks_without_load":c.crosschecked_native_primaries-c.loaded_shared_ticks,"original_grid_matched_indices":c.matched_default,
        "donor_disjoint_fixtures":c.donor_disjoint,"available_landings":c.cells.len(),"unavailable_landings_in_completed_fixtures":c.completed.len()*3-c.cells.len(),
        "profiles":policies,"signed_com_change_vs_8x4_r1":effects,"sensitivity":sensitivity,"outlier_rule":"largest increase of 13 ms COM distance from uniform 4096 r8 when uniform 8x4 r1 changes to r8; ties use lowest case index then earliest stroke",
        "outlier":selection(&c),"selection_final":c.completed.len()==c.scheduled,"all_reference_joins_complete":c.matched_default.len()==c.scheduled,
        "reference_note":"both high-resolution paths are empirical; default-grid campaign has no weighted normal velocity residual",
        "censoring":"a comparison is omitted from the first recorded loss of footprint overlap in either of its paths; missing donor strokes and default-grid files are counted separately"});
    fs::write(
        out.join("relaxation-summary.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    figures::render(&c, out)?;
    if !c.study {
        strata::export(&c, out)?;
        continuation::export(&c, out)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Most physical runs stay over the pad. Exercise the first boundary where a
    // footprint goes disjoint without needing to fabricate a measured donor file.
    #[test]
    fn disjoint_paths_and_unpublished_references_have_distinct_masks() {
        let endpoint = Endpoint {
            i: 1,
            com_m: [0.; 3],
            velocity_m_s: [0.; 3],
            spin_rad_s: [0.; 3],
            orientation_xyzw: [0., 0., 0., 1.],
            cumulative_corners_ns: [0.; 4],
            cumulative_normal_ns: 0.,
            cumulative_tangent_ns: [0.; 3],
            normal_ns: 0.,
            impulse_weighted_abs_post_normal_m_s: None,
            footprint_gap_m: None,
            positive_point_count: None,
            max_post_closing_m_s: None,
        };
        let present = Observation {
            second: endpoint,
            ends: [endpoint; 3],
            overlap_lost: None,
        };
        let cell = Cell {
            cut: Value::Null,
            baseline_first: None,
            first_receipt: continuation::Receipt::Unloaded,
            incoming: None,
            site: Site {
                index: 0,
                offset_mm: 165,
                drive_percent: 76,
                holds: false,
                stroke: 1,
                cut_tick: 200,
            },
            paths: BTreeMap::from([
                (BASE, present),
                (FINE, present),
                (
                    CAND,
                    Observation {
                        overlap_lost: Some(210),
                        ..present
                    },
                ),
            ]),
        };
        assert!(cell.change(CAND, FINE, 0).is_ok());
        assert!(matches!(cell.change(CAND, FINE, 1), Err(Missing::Disjoint)));
        assert!(matches!(cell.change(CAND, FINE, 2), Err(Missing::Disjoint)));
        assert!(matches!(
            cell.change(CAND, ALT, 0),
            Err(Missing::DefaultPending)
        ));
        assert_eq!(
            cell.pair(BASE, FINE, 0)
                .unwrap_or_else(|_| unreachable!())
                .weighted_residual_delta_m_s,
            None
        );
    }
}
