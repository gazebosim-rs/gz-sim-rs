// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the hard-contact relaxation against independently reconstructed four-corner mobility.
use super::*;
use glam::DMat3;
const DT: Duration = Duration::from_nanos(62_500);
const RELAXATION: [usize; 7] = [0, 1, 2, 4, 8, 16, 32];
type Matrix = [[f64; 4]; 4];

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Control {
    Approach,
    Touching,
    StationaryPad,
    ZeroSlide,
    NoBias,
    NoWarmstart,
    FreshGeometry,
    PadFirst,
    PadFirstFreshGeometry,
    RebuildBoxFeatures,
    PadFirstRebuildBoxFeatures,
    WithdrawAfterImpulse,
}
fn initial(ideal: Ideal, mode: Control) -> State {
    let mut s = ideal.state(0);
    for b in [&mut s.foot, &mut s.upper, &mut s.pad] {
        if mode == Control::StationaryPad {
            b.velocity[2] -= PAD_SPEED;
        }
    }
    for b in [&mut s.foot, &mut s.upper] {
        if mode == Control::Touching {
            b.position[2] -= ideal.fixture.relative_closing_m_s * ARRIVAL_NS as f64 * 1e-9;
        }
        if mode == Control::ZeroSlide {
            b.velocity[0] = 0.;
            b.velocity[1] = 0.;
        }
    }
    s
}
fn arms(s: State) -> [DVec3; 4] {
    let q = DQuat::from_array(s.foot.orientation).normalize();
    corners().map(|p| q * p)
}
fn gaps(s: State) -> [f64; 4] {
    arms(s).map(|r| s.foot.position[2] + r.z - s.pad.position[2] - PAD_HALF[2])
}
fn speeds(s: State) -> [f64; 4] {
    let w = DVec3::from(s.foot.angular_velocity);
    arms(s).map(|r| s.foot.velocity[2] + w.cross(r).z - s.pad.velocity[2])
}
/// Independent spatial inertia assembled at the measured two-link COM.
fn physical_mobility(s: State) -> Matrix {
    let mut aggregate = DMat3::ZERO;
    for b in [s.foot, s.upper] {
        let q = DMat3::from_quat(DQuat::from_array(b.orientation).normalize());
        let r = DVec3::from(b.position) - s.com();
        let rr = DMat3::from_cols(r * r.x, r * r.y, r * r.z);
        aggregate += q * DMat3::from_diagonal(DVec3::new(0.0201, 0.0201, 0.025)) * q.transpose()
            + DMat3::IDENTITY * r.length_squared()
            - rr;
    }
    let inv = aggregate.inverse();
    let t = arms(s).map(|r| (r + DVec3::from(s.foot.position) - s.com()).cross(DVec3::Z));
    std::array::from_fn(|i| std::array::from_fn(|j| 0.5 + t[i].dot(inv * t[j])))
}
#[derive(Serialize)]
struct Event {
    time_ns: u64,
    pre: State,
    post: State,
    normal_ns: f64,
    corner_ns: [f64; 4],
    pre_gap_m: [f64; 4],
    post_gap_m: [f64; 4],
    pre_speed_m_s: [f64; 4],
    post_speed_m_s: [f64; 4],
    predicted_delta_speed_m_s: [f64; 4],
    native_mobility: Option<Matrix>,
    physical_mobility: Matrix,
    points: Vec<Value>,
}
struct Outcome {
    summary: Value,
    events: Vec<Event>,
    frames: Vec<Snapshot>,
    end: State,
}
fn trial(ideal: Ideal, tree: bool, relax: usize, mode: Control, detail: bool) -> Result<Outcome> {
    let s = initial(ideal, mode);
    let (mut native, ids) = fresh_in_order(
        s,
        tree,
        1,
        matches!(
            mode,
            Control::PadFirst
                | Control::PadFirstFreshGeometry
                | Control::PadFirstRebuildBoxFeatures
        ),
    )?;
    let mut before = state(&native, ids)?;
    let mut frames = vec![Snapshot {
        state: before,
        tick: 0,
        time_us: 0,
        native_steps: 0,
        selected: false,
        secondary_start: false,
        normal_impulse_ns: 0.,
        corner_ns: [0.; 4],
    }];
    let mut events = vec![];
    let mut total = 0.;
    let mut spike = 0.;
    let mut spike_spin = 0.;
    let mut spike_time = 0;
    let mut off = 0.;
    let mut outer = [0.; 4];
    let mut at_hit = None;
    let mut withdrawn = false;
    let mut max_mobility_delta = 0.0_f64;
    let mut max_spike_speed_error = 0.;
    let mut subsequent = 0.;
    let mut max_n_change = 0.0_f64;
    let mut max_tangent = 0.0_f64;
    for i in 1..=64 {
        let t = i * 62_500;
        let pre_arms = arms(before);
        let measured = if tree && i > 1 && detail {
            Some(native.landing_normal_mobility(ids.foot, pre_arms.map(|r| r.to_array()))?)
        } else {
            None
        };
        if matches!(
            mode,
            Control::RebuildBoxFeatures | Control::PadFirstRebuildBoxFeatures
        ) {
            native.step_landing_with_fresh_box_pairs(DT, relax, &[(ids.foot, ids.pad)])?;
        } else {
            native.step_landing_response_control(
                DT,
                relax,
                mode != Control::NoBias,
                mode != Control::NoWarmstart,
                !matches!(
                    mode,
                    Control::FreshGeometry | Control::PadFirstFreshGeometry
                ),
            )?;
        }
        let post = state(&native, ids)?;
        let mut corners = [0.; 4];
        let mut points = vec![];
        for patch in native.rigid_contact_patches() {
            let is_a = patch.a == ids.foot;
            let other = if is_a { patch.b } else { patch.a };
            if !(is_a || patch.b == ids.foot) || other != ids.pad {
                if patch.points.iter().any(|x| x.normal_impulse != 0.) {
                    return Err("unrelated active collision".into());
                }
                continue;
            }
            let normal = DVec3::from(patch.normal_on_a) * if is_a { 1. } else { -1. };
            for p in patch.points {
                let foot_lever = DVec3::from(if is_a { p.lever_a } else { p.lever_b });
                let foot_center = if tree {
                    DVec3::from(before.foot.position)
                } else {
                    before.com()
                };
                let offset = foot_center + foot_lever - DVec3::from(before.foot.position);
                let q = DQuat::from_array(before.foot.orientation).normalize();
                let local = q.conjugate() * offset;
                let k = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
                if p.normal_impulse > 0. {
                    max_n_change = max_n_change.max(normal.distance(DVec3::Z));
                    max_tangent = max_tangent.max(
                        p.tangent_impulse_on_a
                            .map_or(0., |x| DVec3::from(x).length()),
                    );
                }
                corners[k] += p.normal_impulse;
                if detail {
                    points.push(json!({"corner":k,"local_sole_m":local.to_array(),"normal":normal.to_array(),"normal_ns":p.normal_impulse,"foot_com_lever_m":foot_lever.to_array()}));
                }
            }
        }
        let j: f64 = corners.iter().sum();
        if let Some(k) = ideal.corner {
            off += j - corners[k];
        }
        for (outer, corner) in outer.iter_mut().zip(corners) {
            *outer += corner;
        }
        if j > 0. {
            let physical = physical_mobility(before);
            let pre_speed = speeds(before);
            let post_speed = speeds(post);
            let predicted = std::array::from_fn(|i| {
                physical[i]
                    .iter()
                    .zip(corners)
                    .map(|(x, y)| x * y)
                    .sum::<f64>()
            });
            if let Some(w) = measured {
                for (a, b) in w.iter().flatten().zip(physical.iter().flatten()) {
                    max_mobility_delta = max_mobility_delta.max((a - b).abs());
                }
            }
            if j > spike {
                spike = j;
                spike_time = t;
                spike_spin = DVec3::from(post.foot.angular_velocity).distance(ideal.spin);
                max_spike_speed_error = pre_speed
                    .iter()
                    .zip(post_speed)
                    .zip(predicted)
                    .map(|((a, b), c)| (b - a - c).abs())
                    .fold(0., f64::max);
                at_hit = Some(
                    json!({"pre_gap_m":gaps(before),"post_gap_m":gaps(post),"pre_speed_m_s":pre_speed,"post_speed_m_s":post_speed,
                    "corner_ns":corners,"predicted_delta_speed_m_s":predicted,"physical_mobility":physical,"native_mobility":measured}),
                );
            }
            if total > ideal.impulse * 0.9 {
                subsequent += j;
            }
            total += j;
            if detail {
                events.push(Event {
                    time_ns: t,
                    pre: before,
                    post,
                    normal_ns: j,
                    corner_ns: corners,
                    pre_gap_m: gaps(before),
                    post_gap_m: gaps(post),
                    pre_speed_m_s: pre_speed,
                    post_speed_m_s: post_speed,
                    predicted_delta_speed_m_s: predicted,
                    native_mobility: measured,
                    physical_mobility: physical,
                    points,
                });
            }
            if mode == Control::WithdrawAfterImpulse && !withdrawn && total > ideal.impulse * 0.9 {
                native.set_world_velocity(ids.pad, [0., 0., -2.], [0.; 3])?;
                withdrawn = true;
            }
        }
        before = state(&native, ids)?;
        if i % 8 == 0 {
            frames.push(Snapshot {
                state: before,
                tick: i / 8,
                time_us: t / 1000,
                native_steps: 8,
                selected: false,
                secondary_start: false,
                normal_impulse_ns: outer.iter().sum(),
                corner_ns: outer,
            });
            outer = [0.; 4];
        }
    }
    let end = before;
    if off.abs() > 1e-10
        || max_tangent > 1e-10
        || spike == 0.
        || (mode == Control::WithdrawAfterImpulse && !withdrawn)
    {
        return Err("invalid impact-response trial".into());
    }
    let summary = json!({"fixture":ideal.fixture.label,"representation":if tree{"locked_pair"}else{"single_rigid"},"relaxation":relax,"control":mode,
        "normal_ns":total,"normal_error_ns":total-ideal.impulse,"angular_error_rad_s":DVec3::from(end.foot.angular_velocity).distance(ideal.spin),
        "dominant_normal_ns":spike,"dominant_time_ns":spike_time,"dominant_angular_error_rad_s":spike_spin,"dominant":at_hit,
        "later_impulse_after_90pct_ns":subsequent,"end":end,"min_other_corner_gap_m":ideal.corner.map(|k|geometry(end,Some(k)).0),
        "max_loaded_normal_deviation_from_up":max_n_change,"max_reported_tangent_ns":max_tangent,
        "max_physical_vs_native_mobility":detail.then_some(max_mobility_delta).filter(|_|tree),"dominant_speed_closure_error_m_s":max_spike_speed_error});
    Ok(Outcome {
        summary,
        events,
        frames,
        end,
    })
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    for fixture in FIXTURES {
        let ideal = Ideal::new(fixture);
        for tree in [true, false] {
            for relax in RELAXATION {
                let detail = matches!(fixture.label, "level-fast" | "corner-a-fast")
                    && matches!(relax, 1 | 16);
                let result = trial(ideal, tree, relax, Control::Approach, detail)?;
                if detail {
                    let rep = if tree { "locked" } else { "rigid" };
                    write_json(
                        &out.join(format!("{}-{rep}-{relax}.json", fixture.label)),
                        &result.events,
                    )?;
                }
                if fixture.label == "level-fast" && tree {
                    if relax == 1 {
                        let reference = run(ideal, true, 8, 4, 1)?;
                        if reference.frames.last().unwrap().state != result.end {
                            return Err("direct and Bevy default solves differ".into());
                        }
                        save_visual(out, true, ideal, &reference)?;
                    } else if relax == 16 {
                        trace(out, "relaxed", &result.frames)?;
                    }
                }
                rows.push(result.summary);
            }
            if matches!(fixture.label, "level-fast" | "corner-a-fast") {
                for mode in [
                    Control::Touching,
                    Control::StationaryPad,
                    Control::ZeroSlide,
                    Control::NoBias,
                    Control::NoWarmstart,
                    Control::FreshGeometry,
                    Control::PadFirst,
                    Control::PadFirstFreshGeometry,
                    Control::RebuildBoxFeatures,
                    Control::PadFirstRebuildBoxFeatures,
                    Control::WithdrawAfterImpulse,
                ] {
                    for relax in [1, 16] {
                        rows.push(trial(ideal, tree, relax, mode, false)?.summary);
                    }
                }
            }
        }
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Unbiased relaxation in a level-face landing","lanes":[
        {"label":"locked","name":"Locked foot, 1 unbiased sweep","file":"traces/locked.json"},
        {"label":"relaxed","name":"Locked foot, 16 unbiased sweeps","file":"traces/relaxed.json"},
        {"label":"analytic","name":"Analytic plastic impact","file":"traces/analytic.json","analytical":true}]}),
    )?;
    write_json(
        &out.join("response.json"),
        &json!({"biased_sweeps":1,"temporal_iterations":4,"native_dt_ns":62500,"results":rows}),
    )
}
/// Balanced cyclic orders and their reversals on 56 independently constructed worlds per lane.
pub(crate) fn cost(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let mut rows = vec![];
    for fixture in [FIXTURES[1], FIXTURES[3]] {
        let ideal = Ideal::new(fixture);
        for tree in [true, false] {
            let mut endpoints = vec![];
            for r in RELAXATION {
                let end = trial(ideal, tree, r, Control::Approach, false)?.end;
                cost::measure(ideal, tree, 1, end, Some(r))?;
                endpoints.push(end);
            }
            let mut samples: [Vec<Vec<u64>>; 7] = Default::default();
            for rep in 0..56 {
                for route in 0..7 {
                    let j = (rep + route) % 7;
                    let j = if (rep / 7) % 2 == 0 { j } else { 6 - j };
                    samples[j].push(cost::measure(
                        ideal,
                        tree,
                        1,
                        endpoints[j],
                        Some(RELAXATION[j]),
                    )?);
                }
            }
            for (r, samples) in RELAXATION.into_iter().zip(samples) {
                let mut primary: Vec<_> = samples.iter().map(|x| x[4]).collect();
                let mut all: Vec<_> = samples.iter().flatten().copied().collect();
                rows.push(json!({"fixture":fixture.label,"representation":if tree{"locked_pair"}else{"single_rigid"},"relaxation":r,"primary_p50_ns":cost::rank(&mut primary,50),
                    "primary_p95_ns":cost::rank(&mut primary,95),"all_p95_ns":cost::rank(&mut all,95),"tick_ns":samples}));
            }
        }
    }
    write_json(
        &out.join("cost.json"),
        &json!({"repeats":56,"primary_tick":5,"temporal_iterations":4,"biased_sweeps":1,"native_dt_ns":62500,"results":rows}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unbiased_sweeps_converge_to_the_broad_face_impact() {
        let ideal = Ideal::new(FIXTURES[1]);
        let outcome = trial(ideal, true, 16, Control::Approach, true).unwrap();
        assert!(outcome.summary["normal_error_ns"].as_f64().unwrap().abs() < 1e-9);
        assert!(outcome.summary["angular_error_rad_s"].as_f64().unwrap() < 1e-8);
        assert!(
            outcome.summary["max_physical_vs_native_mobility"]
                .as_f64()
                .unwrap()
                < 1e-12
        );
    }

    #[test]
    fn tilted_primary_impulse_and_spin_match_the_isolated_corner_solution() {
        let ideal = Ideal::new(FIXTURES[3]);
        for c in [
            Control::WithdrawAfterImpulse,
            Control::RebuildBoxFeatures,
            Control::PadFirstRebuildBoxFeatures,
        ] {
            let result = trial(ideal, true, 1, c, false).unwrap().summary;
            assert!(
                result["normal_error_ns"].as_f64().unwrap().abs() < 1e-8,
                "{c:?} {result}"
            );
            assert!(result["angular_error_rad_s"].as_f64().unwrap() < 1e-8);
        }
    }
}
