// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reproducible grid-sensitivity survey. Each output file is one complete fixture.
use super::*;
use std::io::Write as _;

const CASE_COUNT: usize = 25 * 25 * 2;
const DEPTH: [u32; 7] = [64, 128, 256, 512, 1024, 2048, 4096];
const HORIZON: u64 = 26;
const FORMAT: &str = "outer-primary-survey-v1";

pub(super) fn fixture(index: usize) -> Fixture {
    let k = index * 613 % CASE_COUNT;
    // Both physical grids include their endpoints. The coprime stride lets
    // disjoint contiguous batches span the domain instead of one end of it.
    Fixture::new(
        "survey",
        (165 + 5 * (k / 50)) as f64 / 1000.,
        (76 + 2 * ((k / 2) % 25)) as f64 / 100.,
        !k.is_multiple_of(2),
    )
}

enum Treatment {
    Uniform(u32),
    Adaptive,
    Primary(Effort),
}
fn evolve(
    f: Fixture,
    cut: &ReboundFrame,
    method: Treatment,
    mut capture: Option<&mut Vec<ReboundFrame>>,
) -> Result<Value> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let onset_mode = match method {
        Treatment::Uniform(n) => Mode::Uniform(Effort::Split(n)),
        Treatment::Adaptive => ADAPTIVE,
        Treatment::Primary(e) => Mode::Uniform(e),
    };
    r.set_mode(onset_mode);
    if let Some(frames) = capture.as_mut() {
        frames.push(*cut);
    }
    let mut total = 0.;
    let mut tangent = DVec3::ZERO;
    let mut corners = [0.; 4];
    let mut frames = vec![];
    let mut first_ns = None;
    let mut native_calls = 0u64;
    for i in 1..=HORIZON {
        if i == 2 && matches!(method, Treatment::Primary(_)) {
            r.set_mode(ADAPTIVE);
        }
        r.advance_at(command(f, r.tick))?;
        let s = r.frame()?;
        if let Some(frames) = capture.as_mut() {
            frames.push(s);
        }
        native_calls += u64::from(s.native_steps);
        total += s.normal_impulse_ns;
        if let Some(k) = s.first_loaded_substep
            && first_ns.is_none()
        {
            let grid =
                gz_physics::NativeStepGrid::new(Duration::from_micros(STEP_US), s.native_steps)?;
            let elapsed = grid.intervals().take(k as usize).sum::<Duration>();
            first_ns = Some((i - 1) * STEP_US * 1000 + elapsed.as_nanos() as u64);
        }
        tangent += DVec3::from(
            s.tangent_on_foot_ns
                .ok_or("expected native per-point friction")?,
        );
        for (sum, j) in corners.iter_mut().zip(s.corner_ns) {
            *sum += j;
        }
        if matches!(i, 1 | 2 | 10 | HORIZON) {
            frames.push(json!({"i":i,"normal_ns":s.normal_impulse_ns,"cumulative_normal_ns":total,"cumulative_corners_ns":corners,
                "cumulative_tangent_ns":tangent.to_array(),"com_m":com(&s).to_array(),"velocity_m_s":velocity(&s).to_array(),
                "spin_rad_s":s.foot.angular_velocity,"orientation_xyzw":s.foot.orientation}));
        }
    }
    Ok(json!({"first_loaded_native_end_ns":first_ns,"native_calls":native_calls,"samples":frames}))
}

pub(super) fn cuts(f: Fixture) -> Result<([Option<ReboundFrame>; 3], Option<u64>)> {
    let mut rig = start(f, ADAPTIVE)?;
    let mut pre = rig.frame()?;
    let mut released = false;
    let mut cuts = [None; 3];
    let mut unavailable = None;
    while rig.tick < DURATION && cuts[2].is_none() {
        let i = rig.tick;
        let (stroke, _) = sweep::command(i * STEP_US, 0);
        if sweep::DEPART_US.contains(&(i * STEP_US)) {
            released = false;
        }
        rig.advance_at(command(f, i))?;
        let last = rig.case.samples.last().ok_or("missing post-solve sample")?;
        if rebound_gate::supported_gap(state(last.foot), state(last.pad)).is_none() {
            unavailable = Some(rig.tick);
            break;
        }
        let post = rig.frame()?;
        if i * STEP_US >= sweep::DEPART_US[stroke] && cuts[stroke].is_none() {
            if post.normal_impulse_ns > 0. && released {
                cuts[stroke] = Some(pre);
            }
            if post.normal_impulse_ns == 0. && post.gap_m > 1e-3 {
                released = true;
            }
        }
        pre = post;
    }
    Ok((cuts, unavailable))
}
fn collect(f: Fixture) -> Result<Value> {
    let (cuts, unavailable) = cuts(f)?;
    let mut trials = vec![];
    for (stroke, cut) in cuts.into_iter().enumerate() {
        if let Some(cut) = cut {
            let mut paths = vec![];
            for n in DEPTH {
                paths.push(json!({"method":"uniform","calls":n,"iterations":4,"record":evolve(f,&cut,Treatment::Uniform(n),None)?}));
            }
            paths.push(
                json!({"method":"adaptive","record":evolve(f,&cut,Treatment::Adaptive,None)?}),
            );
            for (calls, iterations) in [(4, 16), (8, 4), (8, 8), (16, 4)] {
                let effort = Effort::Grid {
                    calls,
                    iterations,
                    phase: GridPhase::Aligned,
                };
                paths.push(json!({"method":"onset","calls":calls,"iterations":iterations,"record":evolve(f,&cut,Treatment::Primary(effort),None)?}));
            }
            trials.push(json!({"stroke":stroke+1,"cut":snapshot(&cut),"paths":paths}));
        }
    }
    Ok(json!({"fixture":f,"first_disjoint_tick":unavailable,"trials":trials}))
}

pub(crate) fn write(out: &Path, first: usize, count: usize) -> Result<()> {
    write_cases(out, first, count, FORMAT, collect)
}

pub(super) fn write_cases(
    out: &Path,
    first: usize,
    count: usize,
    format: &str,
    collect: impl Fn(Fixture) -> Result<Value>,
) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|&i| i <= CASE_COUNT)
        .ok_or("survey range exceeds 1250 fixtures")?;
    if count == 0 {
        return Err("empty requested survey range".into());
    }
    let results = out.join("cases");
    fs::create_dir_all(&results)?;
    for i in first..end {
        let name = results.join(format!("case-{i:04}.json"));
        if name.exists() {
            let saved: Value = serde_json::from_reader(fs::File::open(&name)?)?;
            if saved["format"] != format
                || saved["index"] != i
                || saved["result"]["fixture"] != serde_json::to_value(fixture(i))?
            {
                return Err(format!("unexpected survey output in {}", name.display()).into());
            }
            continue;
        }
        let data = collect(fixture(i)).map_err(|e| format!("survey fixture {i}: {e}"))?;
        let tmp = name.with_extension(format!("partial-{}", std::process::id()));
        let mut w = std::io::BufWriter::new(fs::File::create(&tmp)?);
        serde_json::to_writer(
            &mut w,
            &json!({"format":format,"index":i,"outer_us":STEP_US,"horizon":HORIZON,"result":data}),
        )?;
        w.flush()?;
        drop(w);
        fs::rename(tmp, name)?;
        if (i - first).is_multiple_of(10) {
            eprintln!("completed fixture {i} of requested range {first}..{end}");
        }
    }
    Ok(())
}

/// Regenerate the measured survey lanes from the identical native incoming cut.
/// Cross-check all saved samples against that fixture's published results.
pub(super) fn archived_cut(
    out: &Path,
    index: usize,
    stroke: usize,
    format: &str,
) -> Result<(Fixture, ReboundFrame, Vec<Value>)> {
    if index >= CASE_COUNT || !(1..=3).contains(&stroke) {
        return Err("select a fixture in 0..1250 and a stroke in 1..=3".into());
    }
    let saved_file = out.join("cases").join(format!("case-{index:04}.json"));
    let saved: Value = serde_json::from_reader(fs::File::open(saved_file)?)?;
    let f = fixture(index);
    if saved["format"] != format
        || saved["index"] != index
        || saved["result"]["fixture"] != serde_json::to_value(f)?
    {
        return Err("survey fixture does not match the matrix coordinates".into());
    }
    let (cuts, _) = cuts(f)?;
    let cut = cuts[stroke - 1].ok_or("no released primary at the requested stroke")?;
    let row = saved["result"]["trials"]
        .as_array()
        .and_then(|rows| rows.iter().find(|r| r["stroke"] == stroke))
        .ok_or("missing verified survey stroke")?;
    let paths = row["paths"].as_array().ok_or("missing survey treatments")?;
    Ok((f, cut, paths.clone()))
}

pub(crate) fn replay(out: &Path, index: usize, stroke: usize) -> Result<()> {
    let (f, cut, paths) = archived_cut(out, index, stroke, FORMAT)?;
    let lanes = [
        (
            "adaptive",
            "Previous adaptive schedule",
            "adaptive",
            0,
            Treatment::Adaptive,
        ),
        (
            "onset",
            "8 × 8 on primary onset",
            "onset",
            8,
            Treatment::Primary(Effort::Grid {
                calls: 8,
                iterations: 8,
                phase: GridPhase::Aligned,
            }),
        ),
        (
            "fine",
            "4,096 × 4 continuously",
            "uniform",
            4096,
            Treatment::Uniform(4096),
        ),
    ];
    fs::create_dir_all(out.join("traces"))?;
    let mut entries = vec![];
    for (file, label, method, n, treatment) in lanes {
        let archived = paths
            .iter()
            .find(|p| {
                p["method"] == method
                    && (n == 0 || p["calls"] == n)
                    && (method != "onset" || p["iterations"] == 8)
            })
            .ok_or("survey case omits a requested replay treatment")?;
        let mut frames = Vec::new();
        let record = evolve(f, &cut, treatment, Some(&mut frames))?;
        // Compare canonical JSON after parsing both through the same float decoder.
        let reread: Value = serde_json::from_slice(&serde_json::to_vec(&record)?)?;
        if reread != archived["record"] {
            return Err(format!(
                "survey replay does not reproduce the archived {method} treatment"
            )
            .into());
        }
        trace(out, file, &frames)?;
        entries.push(json!({"file":format!("traces/{file}.json"), "label":file,"name":label}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":format!("Survey fixture {index}, landing {stroke}"),"lanes":entries}),
    )?;
    write_json(
        &out.join("replay-case.json"),
        &json!({"index":index,"fixture":f,"stroke":stroke,"cut_tick":cut.tick,"samples_validated_per_lane":4}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn survey_partitions_cover_the_full_factorial_once() {
        let keys = (0..CASE_COUNT)
            .map(|i| {
                let f = fixture(i);
                (f.offset_m.to_bits(), f.speed_scale.to_bits(), f.switched)
            })
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(keys.len(), CASE_COUNT);
    }
}
