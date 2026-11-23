// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fixed-time relaxation on frictional primary landings from matched donor states.
use super::*;
use gz_physics::DynamicBackend;
use gz_physics_multifidelity::ContactSolverIterations;
use interval::Solve;

#[path = "continuation_cost.rs"]
pub(crate) mod cost;
#[path = "continuation_span.rs"]
pub(crate) mod span;

const FORMAT: &str = "frictional-primary-relaxation-v1";
const DT: Duration = Duration::from_micros(STEP_US);
const HORIZON: u64 = 26;

#[derive(Clone, Copy, Debug, Serialize)]
struct Profile {
    calls: u32,
    relaxation: usize,
    onset_only: bool,
}
impl Profile {
    const fn all(calls: u32, relaxation: usize) -> Self {
        Self {
            calls,
            relaxation,
            onset_only: false,
        }
    }
    fn iterations(self, step: u64) -> ContactSolverIterations {
        ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: Some(if self.onset_only && step > 1 {
                1
            } else {
                self.relaxation
            }),
        }
    }
}
// Test successive temporal grids and the spatial fixed-time relaxation floor.
// Eight and sixteen native calls additionally isolate onset-only relaxation.
const PROFILES: [Profile; 14] = [
    Profile::all(8, 1),
    Profile::all(8, 4),
    Profile::all(8, 8),
    Profile::all(8, 16),
    Profile {
        calls: 8,
        relaxation: 8,
        onset_only: true,
    },
    Profile::all(16, 1),
    Profile::all(16, 8),
    Profile {
        calls: 16,
        relaxation: 8,
        onset_only: true,
    },
    Profile::all(64, 8),
    Profile::all(256, 8),
    Profile::all(1024, 8),
    Profile::all(4096, 8),
    Profile::all(256, 16),
    Profile::all(1024, 16),
];

fn path(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    mut capture: Option<&mut Vec<Value>>,
) -> Result<Value> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let lane = &mut r.case.lane;
    if lane.tier()? != Dynamic {
        return Err("primary cut requires a dynamic floating foot".into());
    }
    let ids = [lane.foot, lane.pad];
    let upper = lane.upper;
    let mut resource = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing local physics engine")?;
    let mut speed = r.last_speed;
    let mut normal = 0.;
    let mut tangent = DVec3::ZERO;
    let mut corners = [0.; 4];
    let mut first = None;
    let mut samples = vec![];
    let mut disjoint = None;
    if let Some(frames) = capture.as_mut() {
        frames.push(serde_json::to_value(cut.with_effort())?);
    }
    for step in 1..=HORIZON {
        let tick = cut.tick + step;
        let next = command(f, tick - 1);
        if next != speed {
            engine.set_world_velocity(ids[1], [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let mut measure = Solve::default();
        let receipt = engine.observe_contact_interval(
            DT,
            profile.calls,
            profile.iterations(step),
            |native| {
                let patches = native.contact_patches();
                // No contact is a meaningful native interval. Elapsed time must
                // always advance, even when there are no contact rows.
                if patches.is_empty() {
                    measure.elapsed = native.elapsed;
                    measure.steps += 1;
                } else {
                    measure.append(
                        native.elapsed,
                        native.state(ids[0]).expect("registered foot"),
                        native.state(ids[1]).expect("registered support"),
                        ids,
                        patches,
                    );
                }
            },
        )?;
        if measure.elapsed != DT
            || measure.steps != profile.calls
            || receipt.native_steps != profile.calls
        {
            return Err("incomplete solver interval".into());
        }
        let pair = receipt
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (ids[0].min(ids[1]), ids[0].max(ids[1])));
        if pair.and_then(|p| p.first_positive_end) != measure.first_end
            || (pair.map_or(0., |p| p.normal_impulse) - measure.normal).abs() > 1e-11
        {
            return Err("pair receipt and independent per-point observations disagree".into());
        }
        if first.is_none()
            && let Some(t) = measure.first_end
        {
            first = Some(json!({"end_ns":(step-1)*STEP_US*1000+t.as_nanos() as u64,
                "normal_ns":measure.first_native_normal}));
        }
        normal += measure.normal;
        tangent += measure.tangent;
        for (sum, j) in corners.iter_mut().zip(measure.corners) {
            *sum += j;
        }
        let foot = engine.state(ids[0]).ok_or("missing foot")?;
        let top = engine.state(upper).ok_or("missing upper")?;
        let pad = engine.state(ids[1]).ok_or("missing support")?;
        let gap = supported_gap(foot, pad);
        if gap.is_none() {
            disjoint.get_or_insert(tick);
        }
        let pos = (DVec3::from(foot.position) + DVec3::from(top.position)) * 0.5;
        let velocity = (DVec3::from(foot.velocity) + DVec3::from(top.velocity)) * 0.5;
        if matches!(step, 1 | 2 | 10 | HORIZON) {
            samples.push(json!({"i":step,"com_m":pos.to_array(),"velocity_m_s":velocity.to_array(),"spin_rad_s":foot.angular_velocity,
                "orientation_xyzw":foot.orientation,"footprint_gap_m":gap,"normal_ns":measure.normal,
                "positive_point_count":measure.positive_points,"max_post_closing_m_s":measure.max_post_closing,
                "impulse_weighted_abs_post_normal_m_s":(measure.normal>0.).then(||measure.loaded_normal_residual/measure.normal),
                "cumulative_normal_ns":normal,"cumulative_tangent_ns":tangent.to_array(),"cumulative_corners_ns":corners}));
        }
        if let Some(frames) = capture.as_mut() {
            frames.push(json!({"tick":tick,"time_us":tick*STEP_US,"foot":Body::from(foot),"upper":Body::from(top),"pad":Body::from(pad),
                "normal_impulse_ns":measure.normal,"corner_ns":measure.corners,"native_steps":profile.calls,"solver_iterations":4,"gap_m":gap,
                "first_loaded_substep":measure.first,"selected":true,"weak_start":false,"secondary_start":false}));
        }
    }
    Ok(json!({"profile":profile,"first":first,"first_disjoint_tick":disjoint,"samples":samples}))
}
fn collect(f: Fixture, include_default_reference: bool) -> Result<Value> {
    let (cuts, unavailable) = survey::cuts(f)?;
    let mut trials = vec![];
    for (stroke, cut) in cuts.into_iter().enumerate() {
        let Some(cut) = cut else {
            continue;
        };
        let mut paths = PROFILES
            .iter()
            .map(|&p| path(f, &cut, p, None))
            .collect::<Result<Vec<_>>>()?;
        if include_default_reference {
            paths.push(path(f, &cut, Profile::all(4096, 1), None)?);
        }
        trials.push(json!({"stroke":stroke+1,"cut":snapshot(&cut),"paths":paths}));
    }
    Ok(json!({"fixture":f,"first_disjoint_tick":unavailable,"trials":trials}))
}
pub(crate) fn survey(out: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(out, first, count, FORMAT, |f| collect(f, false))
}
pub(crate) fn study(out: &Path) -> Result<()> {
    let mut cases = vec![];
    fs::create_dir_all(out.join("traces"))?;
    for f in CASES.into_iter().filter(|f| f.name.starts_with("outer-")) {
        cases.push(collect(f, true)?);
        if f.name == "outer-fast" {
            let (cuts, _) = survey::cuts(f)?;
            let cut = cuts[1].ok_or("missing outer-fast second primary")?;
            comparison(
                out,
                f,
                &cut,
                None,
                false,
                "Unbiased relaxation on the outer frictional support",
            )?;
        }
    }
    write_json(
        &out.join("study.json"),
        &json!({"format":FORMAT,"cases":cases}),
    )
}

fn comparison(
    out: &Path,
    f: Fixture,
    cut: &ReboundFrame,
    archived: Option<&[Value]>,
    continuation: bool,
    title: &str,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut entries = vec![];
    let first_lane = if continuation {
        (
            "onset",
            "Eight passes on first interval only",
            Profile {
                onset_only: true,
                ..Profile::all(8, 8)
            },
        )
    } else {
        (
            "default",
            "8 refreshes, 1 relaxation pass",
            Profile::all(8, 1),
        )
    };
    for (name, title, profile) in [
        first_lane,
        (
            "relaxed",
            "8 refreshes, 8 relaxation passes",
            Profile::all(8, 8),
        ),
        (
            "fine",
            "4,096 refreshes, 8 relaxation passes",
            Profile::all(4096, 8),
        ),
    ] {
        let mut frames = vec![];
        let record = path(f, cut, profile, Some(&mut frames))?;
        if let Some(saved) = archived {
            let reread: Value = serde_json::from_slice(&serde_json::to_vec(&record)?)?;
            if !saved.contains(&reread) {
                return Err(
                    "relaxation replay differs from the corresponding archived solve".into(),
                );
            }
        }
        write_json(
            &out.join("traces").join(format!("{name}.json")),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        entries.push(json!({"file":format!("traces/{name}.json"),"label":name,"name":title}));
    }
    let mut manifest = json!({"title":title,"lanes":entries});
    if continuation {
        manifest["capture_ticks"] =
            json!({"shared_receipt":cut.tick+1,"first_different_budget":cut.tick+2});
    }
    write_json(&out.join("comparison.json"), &manifest)?;
    Ok(())
}

pub(crate) fn replay(out: &Path, index: usize, stroke: usize) -> Result<()> {
    replay_selected(out, index, stroke, false)
}

pub(crate) fn replay_continuation(out: &Path, index: usize, stroke: usize) -> Result<()> {
    replay_selected(out, index, stroke, true)
}

fn replay_selected(dir: &Path, index: usize, stroke: usize, continuation: bool) -> Result<()> {
    let (f, cut, paths) = survey::archived_cut(dir, index, stroke, FORMAT)?;
    let out = if continuation {
        dir.join("continuation")
    } else {
        dir.to_path_buf()
    };
    let title = if continuation {
        "Stopping relaxation after touchdown"
    } else {
        "Frictional relaxation"
    };
    comparison(
        &out,
        f,
        &cut,
        Some(&paths),
        continuation,
        &format!("{title}, fixture {index}, landing {stroke}"),
    )?;
    write_json(
        &out.join("replay-case.json"),
        &json!({"index":index,"fixture":f,"stroke":stroke,"cut_tick":cut.tick,"samples_validated_per_lane":4}),
    )
}
