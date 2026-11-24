// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Locate the first outer-support deviation and cross donor history with subsequent solver work.
use super::*;

#[path = "outer_intervention.rs"]
mod intervention;
#[path="outer_relaxation.rs"]
pub(crate) mod relaxation;
#[path="outer_interval.rs"]
pub(crate) mod interval;
#[path="outer_propagation.rs"]
pub(crate) mod propagation;
#[path="outer_survey.rs"]
pub(crate) mod survey;
#[path = "outer_grids.rs"]
pub(crate) mod grids;
#[path = "outer_cadence.rs"]
pub(crate) mod cadence;
#[path = "primary_policy.rs"]
pub(crate) mod primary_policy;
#[path = "outer_finite.rs"]
pub(crate) mod finite;
#[path = "outer_points.rs"]
pub(crate) mod points;

const ADAPTIVE: Mode = Mode::Weak(BUDGET);
const FINE: Mode = Mode::Uniform(Effort::Split(4));
const LIMITS: [f64; 4] = [1e-6, 10e-6, 100e-6, 1e-3];

fn restart(f: Fixture, donor: Mode, cut: &ReboundFrame) -> Result<Rig> {
    let mut rig = start(f, donor)?;
    while rig.tick < cut.tick {
        rig.advance_at(command(f, rig.tick))?;
    }
    let got = rig.frame()?;
    if !same_state(&got, cut)
        || got.solver_iterations != cut.solver_iterations
        || got.native_steps != cut.native_steps
        || got.first_loaded_substep != cut.first_loaded_substep
    {
        return Err(format!("{} failed to reproduce native cut at {}", f.name, cut.tick).into());
    }
    rig.case
        .lane
        .sim
        .world_mut()
        .resource_mut::<Step>()
        .record_tangents = true;
    Ok(rig)
}

fn branch(
    f: Fixture,
    donor: Mode,
    cut: &ReboundFrame,
    continuation: Mode,
    end: usize,
) -> Result<Vec<ReboundFrame>> {
    let mut rig = restart(f, donor, cut)?;
    rig.set_mode(continuation);
    let mut frames = vec![rig.frame()?];
    while rig.tick < end as u64 {
        rig.advance_at(command(f, rig.tick))?;
        frames.push(rig.frame()?);
    }
    Ok(frames)
}

fn distance(a: &ReboundFrame, b: &ReboundFrame) -> f64 {
    com(a).distance(com(b))
}
fn velocity(r: &ReboundFrame) -> DVec3 {
    (DVec3::from(r.foot.velocity) + DVec3::from(r.upper.velocity)) * 0.5
}
fn first_above(a: &[ReboundFrame], b: &[ReboundFrame], limit: f64) -> Option<usize> {
    a.iter().zip(b).position(|(x, y)| distance(x, y) > limit)
}

// First actual load following a supported gap above 1 mm on this commanded stroke.
// A positive receipt on the departure tick itself is not a landing.
fn primary(frames: &[ReboundFrame], stroke: usize) -> Result<usize> {
    let begin = (sweep::DEPART_US[stroke] / STEP_US) as usize + 1;
    let end = sweep::DEPART_US
        .get(stroke + 1)
        .map_or(frames.len(), |u| (u / STEP_US) as usize + 1);
    let mut released = false;
    for (offset, r) in frames[begin..end].iter().enumerate() {
        if r.normal_impulse_ns > 0. {
            if released {
                return Ok(begin + offset);
            }
        } else {
            released |= r.gap_m > 1e-3;
        }
    }
    Err(format!("no primary landing in stroke {}", stroke + 1).into())
}

fn snapshot(r: &ReboundFrame) -> Value {
    json!({"tick":r.tick,"com_m":com(r).to_array(),"com_velocity_m_s":velocity(r).to_array(),
        "foot_pose_xyzw":r.foot.orientation,"foot_angular_velocity_rad_s":r.foot.angular_velocity,
        "corners_ns":r.corner_ns,"tangent_on_foot_ns":r.tangent_on_foot_ns,"normal_ns":r.normal_impulse_ns,"gap_m":r.gap_m,
        "native_calls":r.native_steps,"iterations_per_call":r.solver_iterations,
        "first_loaded_substep":r.first_loaded_substep})
}

fn episode(a: &[ReboundFrame], b: &[ReboundFrame], at: usize) -> Value {
    let x = &a[at];
    let y = &b[at];
    json!({"tick":x.tick,"time_ms":x.time_us as f64/1000.,"distance_m":distance(x,y),
        "previous_distance_m":at.checked_sub(1).map(|i|distance(&a[i],&b[i])),
        "adaptive":snapshot(x),"four":snapshot(y)})
}

fn native_prefix_check(r: &[ReboundFrame], continuous: &[ReboundFrame]) -> Result<()> {
    for (a, b) in r.iter().zip(continuous) {
        if !same_state(a, b)
            || a.native_steps != b.native_steps
            || a.solver_iterations != b.solver_iterations
            || a.first_loaded_substep != b.first_loaded_substep
        {
            return Err(format!("own-donor restart diverged at tick {}", a.tick).into());
        }
    }
    Ok(())
}

// A and F use their own native donor and treatment. AF takes the fine grid from
// the adaptive donor, FA runs the live adaptive gate from the fine donor.
fn crossover(a: &ReboundFrame, af: &ReboundFrame, fa: &ReboundFrame, f: &ReboundFrame) -> Value {
    let local = com(a) - com(af);
    let history = com(af) - com(f);
    let opposite = com(fa) - com(f);
    json!({"tick":a.tick,"continuous_delta_m":(com(a)-com(f)).to_array(),"continuation_on_adaptive_donor_m":local.to_array(),
        "inherited_with_fine_continuation_m":history.to_array(),"continuation_on_fine_donor_m":opposite.to_array(),
        "interaction_m":(local-opposite).to_array(),"continuous_distance_m":distance(a,f),
        "continuation_distance_m":local.length(),"inherited_distance_m":history.length(),
        "continuation_velocity_delta_m_s":(velocity(a)-velocity(af)).to_array(),
        "adaptive":snapshot(a),"fine_from_adaptive":snapshot(af),"adaptive_from_fine":snapshot(fa),"fine":snapshot(f)})
}

// Vary how much of the adaptive history is inherited, then use the same fine
// continuation. All samples use absolute actuator time across every switch.
fn inherited_prefixes(
    f: Fixture,
    a: &[ReboundFrame],
    fine: &[ReboundFrame],
    primary_ticks: &[(usize, usize)],
    major: usize,
) -> Result<Vec<Value>> {
    let mut results = vec![];
    let mut cuts = vec![
        ("initial_settle", 0),
        ("first_departure", (sweep::DEPART_US[0] / STEP_US) as usize),
    ];
    let mut landmarks = vec![major];
    for (stroke, &(pa, pf)) in primary_ticks.iter().enumerate() {
        let depart = (sweep::DEPART_US[stroke] / STEP_US) as usize;
        if stroke > 0 {
            cuts.push(("later_departure", depart));
        }
        let first = pa.min(pf);
        cuts.extend([
            ("immediately_before_primary", first - 1),
            ("after_primary_onset", first),
        ]);
        landmarks.extend([depart, first - 1, first]);
    }
    landmarks.sort_unstable();
    landmarks.dedup();
    for (label, cut) in cuts {
        let frames = branch(f, ADAPTIVE, &a[cut], FINE, major)?;
        let samples = landmarks
            .iter()
            .copied()
            .filter(|&t| t >= cut)
            .map(|t| {
                let r = &frames[t - cut];
                let g = &fine[t];
                json!({"tick":t,"delta_m":(com(r)-com(g)).to_array(),"distance_m":distance(r,g),
                "delta_velocity_m_s":(velocity(r)-velocity(g)).to_array(),
                "normal_ns":r.normal_impulse_ns,"corner_ns":r.corner_ns})
            })
            .collect::<Vec<_>>();
        results.push(json!({"fine_after":label,"cut":cut,"samples":samples}));
    }
    Ok(results)
}

fn analyze(f: Fixture, out: &Path) -> Result<Value> {
    let (a, _) = run(f, ADAPTIVE)?;
    let (g, _) = run(f, FINE)?;
    if !same_state(&a[0], &g[0]) {
        return Err("continuous donors differ at initial state".into());
    }
    let milestones = LIMITS.map(|l| first_above(&a, &g, l));
    let major = milestones[3].ok_or("outer run never exceeded 1 mm")?;
    let mut strokes = vec![];
    for i in 0..sweep::DEPART_US.len() {
        let ap = primary(&a, i)?;
        let fp = primary(&g, i)?;
        if ap.min(fp) > major {
            break;
        }
        strokes.push((ap, fp));
    }
    let inherited = inherited_prefixes(f, &a, &g, &strokes, major)?;
    let mut segments = vec![];
    for (i, &(ap, fp)) in strokes.iter().enumerate() {
        let cut = ap.min(fp) - 1;
        let end = strokes.get(i + 1).map_or(major, |&(x, y)| x.min(y) - 1);
        let aa = branch(f, ADAPTIVE, &a[cut], ADAPTIVE, end)?;
        let af = branch(f, ADAPTIVE, &a[cut], FINE, end)?;
        let fa = branch(f, FINE, &g[cut], ADAPTIVE, end)?;
        let ff = branch(f, FINE, &g[cut], FINE, end)?;
        native_prefix_check(&aa, &a[cut..=end])?;
        native_prefix_check(&ff, &g[cut..=end])?;
        let mut indices = vec![cut, cut + 1, ap, fp, end];
        indices.extend(
            milestones
                .iter()
                .flatten()
                .copied()
                .filter(|&t| (cut..=end).contains(&t)),
        );
        indices.extend(
            [2, 4, 10, 20, 40]
                .map(|k| cut + k)
                .into_iter()
                .filter(|&t| t <= end),
        );
        let matched_limits = LIMITS.map(|l| first_above(&aa, &af, l).map(|n| n + cut));
        indices.extend(matched_limits.iter().flatten().copied());
        indices.sort_unstable();
        indices.dedup();
        let samples = indices
            .iter()
            .map(|t| {
                let j = t - cut;
                crossover(&aa[j], &af[j], &fa[j], &ff[j])
            })
            .collect::<Vec<_>>();
        let last = i + 1 == strokes.len();
        if last && f.name == "outer-fast" {
            trace(out, "adaptive", &aa)?;
            trace(out, "matched_fine", &af)?;
            trace(out, "continuous_fine", &ff)?;
        }
        let control = if last {
            Some(intervention::investigate(f, &aa, &af)?)
        } else {
            None
        };
        segments.push(json!({"stroke":i+1,"cut":cut,"end":end,"primary_ticks":[ap,fp],
            "first_matched_over_limit_ticks":matched_limits,"measurements":samples,"interventions":control}));
    }
    let mut marks = vec![];
    for (l, t) in LIMITS.iter().zip(milestones) {
        if let Some(t) = t {
            marks.push(json!({"limit_m":l,"boundary":episode(&a,&g,t)}));
        }
    }
    Ok(
        json!({"fixture":f,"initial":episode(&a,&g,0),"first_motion_tick":first_above(&a,&g,0.),
        "departure":episode(&a,&g,(sweep::DEPART_US[0]/STEP_US) as usize),
        "thresholds":marks,"segments":segments,"adaptive_prefix_fine_continuations":inherited}),
    )
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let cases = CASES
        .into_iter()
        .filter(|f| f.name.starts_with("outer-"))
        .map(|f| analyze(f, out))
        .collect::<Result<Vec<_>>>()?;
    write_json(
        &out.join("study.json"),
        &json!({"step_us":STEP_US,"limits_m":LIMITS,"cases":cases,
        "visual_fixture":"outer-fast","visual_interval":"preceding primary cut through first 1 mm"}),
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matched_primary_restarts_preserve_the_entire_own_donor_history() -> Result<()> {
        let f = CASES
            .iter()
            .copied()
            .find(|f| f.name == "outer-fast")
            .unwrap();
        let (a, _) = run(f, ADAPTIVE)?;
        let (g, _) = run(f, FINE)?;
        let cut = primary(&a, 1)?.min(primary(&g, 1)?) - 1;
        let end = cut + 12;
        let af = branch(f, ADAPTIVE, &a[cut], FINE, end)?;
        native_prefix_check(&branch(f, ADAPTIVE, &a[cut], ADAPTIVE, end)?, &a[cut..=end])?;
        native_prefix_check(&branch(f, FINE, &g[cut], FINE, end)?, &g[cut..=end])?;
        assert!(same_state(&a[cut], &af[0]));
        assert!(af[1].normal_impulse_ns > 0. && a[cut + 1].normal_impulse_ns > 0.);
        assert_ne!(af[1].corner_ns, a[cut + 1].corner_ns);
        // On this two-kilogram assembly the pad supplies the only external
        // horizontal impulse. The slightly inclined normal also contributes.
        let tangent = DVec3::from(af[1].tangent_on_foot_ns.expect("native point friction"));
        let delivered = 2. * (velocity(&af[1]) - velocity(&af[0]));
        assert!((delivered - tangent).truncate().length() < 1e-4);
        assert!(tangent.y < -0.06);
        assert_eq!(af[2].normal_impulse_ns, 0.);
        assert_eq!(af[2].tangent_on_foot_ns, Some([0.; 3]));
        Ok(())
    }
}
