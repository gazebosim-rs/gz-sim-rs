// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read complete census batches and independently check the input lattice and targets.
use super::*;
use serde::Deserialize;
use std::collections::BTreeMap;

#[derive(Deserialize)]
struct Batch {
    format: String,
    first: usize,
    end: usize,
    cases: Vec<Row>,
}
#[derive(Deserialize)]
struct Row {
    index: usize,
    fixture: Case,
    analytic: Target,
    paths: Vec<Run>,
}
#[derive(Deserialize)]
struct Target {
    impulse_ns: [f64; 2],
    pitch_rad_s: f64,
    mode: String,
    sticking: bool,
}
#[derive(Deserialize)]
struct Run {
    profile: Profile,
    first: Sample,
    at_500us: Sample,
    end: Sample,
}
#[derive(Clone, Copy, Deserialize)]
struct Sample {
    impulse_ns: [f64; 3],
    spin_rad_s: [f64; 3],
    impulse_error_ns: f64,
    spin_error_rad_s: f64,
}
#[derive(Default)]
struct Errors {
    impulses: Vec<(f64, usize)>,
    spins: Vec<(f64, usize)>,
    under_both: usize,
}
impl Errors {
    fn add(&mut self, impulse: f64, spin: f64, index: usize) {
        self.impulses.push((impulse, index));
        self.spins.push((spin, index));
        self.under_both += usize::from(impulse <= 0.01 && spin <= 0.05);
    }
    fn report(mut self) -> Value {
        fn stats(v: &mut [(f64, usize)]) -> Value {
            v.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let q = |n: usize| v[(n * v.len()).div_ceil(100) - 1].0;
            let &(max, case) = v.last().unwrap();
            json!({"median":q(50),"p95":q(95),"p99":q(99),"max":max,"max_case":case})
        }
        json!({"count":self.impulses.len(),"within_0p01_ns_and_0p05_rad_s":self.under_both,
            "impulse_ns":stats(&mut self.impulses),"spin_rad_s":stats(&mut self.spins)})
    }
}
impl Sample {
    fn validate(self, t: PlanarImpactResponse) -> Result<()> {
        let j = DVec3::from(self.impulse_ns).distance(DVec3::new(t.impulse[0], 0., t.impulse[1]));
        let w = DVec3::from(self.spin_rad_s).distance(DVec3::Y * t.pitch_rate);
        if (j - self.impulse_error_ns).abs() > 1e-11 || (w - self.spin_error_rad_s).abs() > 1e-11 {
            return Err(
                "archived error disagrees with independently recomputed native vector distance"
                    .into(),
            );
        }
        Ok(())
    }
}
// Retain both body orders for signed comparisons. A physical reflection in x
// transforms polar impulses by (-x,y,z), and axial spins by (x,-y,-z).
type Onsets = [([f64; 3], [f64; 3]); PROFILES.len()];
pub(super) fn summarize(out: &Path) -> Result<()> {
    let mut groups: BTreeMap<(&str, usize, &str), Errors> = BTreeMap::new();
    let mut onsets = vec![None::<Onsets>; FIELD];
    let mut files =
        fs::read_dir(out.join("batches"))?.collect::<std::result::Result<Vec<_>, _>>()?;
    files.sort_unstable_by_key(|f| f.file_name());
    for file in files {
        if file.path().extension().is_none_or(|p| p != "json") {
            continue;
        }
        let batch: Batch =
            serde_json::from_reader(std::io::BufReader::new(fs::File::open(file.path())?))?;
        if batch.format != FORMAT
            || batch.end > FIELD
            || batch.first >= batch.end
            || batch.end - batch.first != batch.cases.len()
        {
            return Err("invalid batch header".into());
        }
        for (k, r) in (batch.first..batch.end).zip(batch.cases) {
            let c = Case::at(k);
            if k != r.index
                || r.fixture.pad_first != c.pad_first
                || [
                    r.fixture.offset - c.offset,
                    r.fixture.mu - c.mu,
                    r.fixture.closing - c.closing,
                    r.fixture.slip - c.slip,
                ]
                .into_iter()
                .any(|v| v.abs() > 1e-14)
                || r.paths.len() != PROFILES.len()
            {
                return Err(format!("case {k} inconsistent with declared lattice").into());
            }
            let target = c.analytic()?;
            let desc = describe(c)?;
            if r.analytic.mode != desc["mode"].as_str().unwrap()
                || r.analytic.sticking != target.sticking
                || (r.analytic.pitch_rad_s - target.pitch_rate).abs() > 1e-12
                || r.analytic
                    .impulse_ns
                    .iter()
                    .zip(target.impulse)
                    .any(|(x, y)| (x - y).abs() > 1e-12)
            {
                return Err("saved target disagrees with reconstructed physics".into());
            }
            let flat = k * 13007 % FIELD;
            let offset = flat / (2 * 2 * 13 * 17);
            let split = if offset.abs_diff(12) % 4 == 1 {
                "heldout_offset"
            } else {
                "training_offset"
            };
            let mode = match (target.bearing, target.sticking) {
                (Bearing::Face, true) => "face_stick",
                (Bearing::Face, false) => "face_slide",
                (_, true) => "edge_stick",
                (_, false) => "edge_slide",
            };
            let mut vectors = [([0.; 3], [0.; 3]); PROFILES.len()];
            for (i, p) in r.paths.into_iter().enumerate() {
                if p.profile != PROFILES[i] {
                    return Err("archived profile ordering changed".into());
                }
                for (when, s) in [("first", p.first), ("500us", p.at_500us), ("1ms", p.end)] {
                    s.validate(target)?;
                    for class in ["all", mode, split] {
                        groups.entry((class, i, when)).or_default().add(
                            s.impulse_error_ns,
                            s.spin_error_rad_s,
                            k,
                        );
                    }
                }
                vectors[i] = (p.at_500us.impulse_ns, p.at_500us.spin_rad_s);
            }
            if onsets[flat].replace(vectors).is_some() {
                return Err("duplicate physics case".into());
            }
        }
    }
    if onsets.iter().any(Option::is_none) {
        return Err("incomplete census".into());
    }
    for k in 0..FIELD {
        let flat = k * 13007 % FIELD;
        let order = flat ^ 1;
        let speed = (flat / 4) % 13;
        let offset = flat / (4 * 13 * 17);
        let mirror =
            (((24 - offset) * 17 + (flat / (4 * 13)) % 17) * 13 + 12 - speed) * 4 + flat % 4;
        for (name, pair, reflected) in [
            ("insertion_order", order, false),
            ("x_reflection", mirror, true),
        ] {
            if pair <= flat {
                continue;
            }
            let a = onsets[flat].as_ref().unwrap();
            let b = onsets[pair].as_ref().unwrap();
            for i in 0..PROFILES.len() {
                let (mut j, mut w) = b[i];
                if reflected {
                    j[0] *= -1.;
                    w[1] *= -1.;
                    w[2] *= -1.;
                }
                groups.entry((name, i, "500us")).or_default().add(
                    DVec3::from(a[i].0).distance(j.into()),
                    DVec3::from(a[i].1).distance(w.into()),
                    k,
                );
            }
        }
    }
    let rows=groups.into_iter().map(|((group,i,when),v)|json!({"group":group,"profile":PROFILES[i],"time":when,"statistics":v.report()})).collect::<Vec<_>>();
    write_json(
        &out.join("summary.json"),
        &json!({"format":FORMAT,"field":FIELD,"profiles":PROFILES.len(),"heldout":"absolute offset indices 1, 5, 9, each mirrored; every friction, speed, order","tolerances":{"impulse_ns":0.01,"spin_rad_s":0.05},"rows":rows}),
    )
}

pub(super) fn replay(out: &Path, index: usize) -> Result<()> {
    if index >= FIELD {
        return Err("case outside lattice".into());
    }
    let c = Case::at(index);
    let dir = out.join(format!("case-{index:05}"));
    fs::create_dir_all(dir.join("traces"))?;
    let mut lanes = vec![];
    let mut results = vec![];
    for (p, id, name) in [
        (PROFILES[0], "default", "Locked, 1 relaxation pass"),
        (PROFILES[2], "relaxed", "Locked, 32 relaxation passes"),
        (PROFILES[7], "rigid", "Rigid equivalent, point friction"),
    ] {
        let (r, frames) = trial(c, p, 16, true)?;
        results.push(r);
        write_json(
            &dir.join("traces").join(format!("{id}.json")),
            &json!({"geometry":{"foot_half_extents":FOOT,"upper_half_extents":TOP,"pad_half_extents":PAD},"frames":frames}),
        )?;
        lanes.push(json!({"name":name,"label":id,"file":format!("traces/{id}.json")}));
    }
    write_json(
        &dir.join("comparison.json"),
        &json!({"title":format!("Planar census, case {index}"),"fixture":c,"target":describe(c)?,"results":results,"lanes":lanes}),
    )
}
