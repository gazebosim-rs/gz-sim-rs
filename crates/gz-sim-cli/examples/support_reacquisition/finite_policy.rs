// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Intervene on the original primary solve, then follow common native updates.
use super::*;
#[path = "finite_cost.rs"]
pub(crate) mod cost;
use gz_physics_multifidelity::BoxContactActions;
const TAG: &str = "finite-projection-primary-v1";
const HORIZON: u64 = 26;
#[derive(Clone, Copy)]
struct Profile {
    id: &'static str,
    onset: u32,
    suffix: u32,
    first: bool,
    continued: bool,
    retime: bool,
}
const fn p(id: &'static str, onset: u32, suffix: u32, first: bool, continued: bool) -> Profile {
    Profile {
        id,
        onset,
        suffix,
        first,
        continued,
        retime: false,
    }
}
const PROFILES: [Profile; 6] = [
    p("native8", 8, 8, false, false),
    p("finite2", 2, 8, true, false),
    p("finite8", 8, 8, true, false),
    p("finite64", 64, 8, true, false),
    p("native64", 64, 8, false, false),
    p("finite8_continued", 8, 8, true, true),
];
const fn with_transfer(mut profile: Profile, id: &'static str) -> Profile {
    profile.retime = true;
    profile.id = id;
    profile
}
const TRANSFERS: [Profile; 8] = [
    p("finite4", 4, 8, true, false),
    with_transfer(p("finite4", 4, 8, true, false), "retimed4"),
    PROFILES[2],
    with_transfer(PROFILES[2], "retimed8"),
    p("finite16", 16, 8, true, false),
    with_transfer(p("finite16", 16, 8, true, false), "retimed16"),
    PROFILES[3],
    with_transfer(PROFILES[3], "retimed64"),
];
fn path(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    capture: bool,
) -> Result<(Value, Vec<Value>)> {
    measured_path(f, cut, profile, capture, None)
}
fn measured_path(
    f: Fixture,
    cut: &ReboundFrame,
    profile: Profile,
    capture: bool,
    mut timing: Option<&mut Vec<u64>>,
) -> Result<(Value, Vec<Value>)> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let lane = &mut r.case.lane;
    if lane.tier()? != Dynamic {
        return Err("expected dynamic donor".into());
    }
    let (a, b, upper) = (lane.foot, lane.pad, lane.upper);
    let mut resource = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("native engine")?;
    let pair = [(a, b)];
    let mut speed = r.last_speed;
    let mut frames = Vec::new();
    if capture {
        let mut initial = serde_json::to_value(cut.with_effort())?;
        initial.as_object_mut().unwrap().remove("corner_ns");
        frames.push(initial);
    }
    let mut first = None;
    let mut impacts = vec![];
    let mut samples = vec![];
    let mut total = 0;
    let mut normal = 0.;
    let mut tangent = DVec3::ZERO;
    let param = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(1),
    };
    for i in 1..=HORIZON {
        let tick = cut.tick + i;
        let next = command(f, tick - 1);
        if next != speed {
            engine.set_world_velocity(b, [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let calls = if i == 1 {
            profile.onset
        } else {
            profile.suffix
        };
        let clock = timing.as_ref().map(|_| std::time::Instant::now());
        let interval = if i == 1 && profile.first || i > 1 && profile.continued {
            engine
                .advance_box_contacts(
                    Duration::from_micros(STEP_US),
                    calls,
                    param,
                    BoxContactActions {
                        finite_point_impacts: &pair,
                        retime_finite_transfer: if profile.retime { &pair } else { &[] },
                        ..Default::default()
                    },
                    None,
                )?
                .interval
        } else {
            engine.step_contact_interval(Duration::from_micros(STEP_US), calls, param)?
        };
        if let (Some(clock), Some(cost)) = (clock, timing.as_deref_mut()) {
            cost.push(clock.elapsed().as_nanos().try_into()?);
        }
        total += interval.native_steps;
        let mut j = 0.;
        for q in interval.pairs {
            if (q.a, q.b) != (a.min(b), a.max(b)) {
                if q.normal_impulse > 1e-9 {
                    return Err("competing native load".into());
                }
                continue;
            }
            if let Some(t) = q.first_positive_end {
                first.get_or_insert((i - 1) * STEP_US * 1000 + t.as_nanos() as u64);
            }
            j += q.normal_impulse;
            tangent += if q.a == a { 1. } else { -1. }
                * DVec3::from(q.tangent_impulse_on_a.ok_or("missing friction")?);
        }
        normal += j;
        for hit in interval.finite_point_impacts {
            let h = hit.resultant;
            let mut row = json!({"i":i,"local_ns":h.elapsed.as_nanos()as u64,"feature":format!("{:?}",hit.feature),"point_m":h.point,
                "impulse_ns":h.impulse_on_foot,"after":h.after_relative_velocity,"before":h.before_relative_velocity,"normal":h.normal,"gap_m":h.gap,"sticking":h.sticking,"dissipated_energy_j":h.dissipated_energy});
            if let Some((limit, delta)) = hit.next_feature_boundary {
                row["retimed"] =
                    json!({"limit":format!("{limit:?}"),"after_impact_ns":delta.as_nanos()as u64});
            }
            impacts.push(row);
        }
        let foot = engine.state(a).ok_or("missing foot")?;
        let top = engine.state(upper).ok_or("missing upper")?;
        let pad = engine.state(b).ok_or("missing deck")?;
        if matches!(i, 1 | 2 | 10 | HORIZON) {
            let center = (DVec3::from(foot.position) + DVec3::from(top.position)) * 0.5;
            samples.push(json!({"i":i,"normal_ns":j,"cumulative_normal_ns":normal,"cumulative_tangent_ns":tangent.to_array(),
                "com_m":center.to_array(),"spin_rad_s":foot.angular_velocity,"orientation_xyzw":foot.orientation,
                "velocity_m_s":((DVec3::from(foot.velocity)+DVec3::from(top.velocity))*0.5).to_array()}));
        }
        if capture {
            frames.push(json!({"tick":tick,"time_us":tick*STEP_US,"foot":Body::from(foot),"upper":Body::from(top),"pad":Body::from(pad),
            "native_steps":calls,"solver_iterations":4,"normal_impulse_ns":j}));
        }
    }
    Ok((
        json!({"profile":profile.id,"native_calls":total,"first_loaded_native_end_ns":first,"impacts":impacts,"samples":samples}),
        frames,
    ))
}
fn collect(f: Fixture, profiles: &[Profile]) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, cut) in cuts.into_iter().enumerate() {
        if let Some(cut) = cut {
            let (flight, defect) = kinematics(f, &cut)?;
            let mut paths = vec![];
            for &p in profiles {
                paths.push(path(f, &cut, p, false)?.0);
            }
            trials.push(
                json!({"stroke":i+1,"cut":snapshot(&cut),"rigid_twist_residual":defect,
                "geometry":if defect<=1e-8{geometry(flight)?}else{Value::Null},"paths":paths}),
            );
        }
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, TAG, |f| collect(f, &PROFILES))
}
pub(crate) fn transfers(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, "finite-retimed-transfer-v1", |f| {
        collect(f, &TRANSFERS)
    })
}
pub(crate) fn replay(dir: &Path, k: usize, stroke: usize) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid replay selection".into());
    }
    let f = survey::fixture(k);
    let data: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let transfer = data["format"] == "finite-retimed-transfer-v1";
    let cut = survey::cuts(f)?.0[stroke - 1].ok_or("missing primary cut")?;
    let trial = data["result"]["trials"]
        .as_array()
        .unwrap()
        .iter()
        .find(|r| r["stroke"] == stroke)
        .ok_or("missing archived stroke")?;
    let out = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    for (mode, label) in [
        if transfer {
            (TRANSFERS[2], "Finite impact then uniform eight-call budget")
        } else {
            (PROFILES[0], "8 native calls")
        },
        if transfer {
            (
                TRANSFERS[3],
                "Measured feature timing at the same eight calls",
            )
        } else {
            (PROFILES[2], "Finite first impact then common eight")
        },
        (
            p("fine4096", 4096, 4096, false, false),
            "Continuous 4,096-call comparison",
        ),
    ] {
        let (record, frames) = path(f, &cut, mode, true)?;
        let reread: Value = serde_json::from_slice(&serde_json::to_vec(&record)?)?;
        if mode.id != "fine4096" && !trial["paths"].as_array().unwrap().contains(&reread) {
            return Err("replay differs from archive".into());
        }
        let file = format!("traces/{}.json", mode.id);
        write_json(
            &out.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames,"measurements":record}),
        )?;
        lanes.push(json!({"label":mode.id,"name":label,"file":file}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"pair_totals_only":true,"title":if transfer{"Native grid at the next finite contact feature"}else{"First geometric point impact on the finite outer support"},"lanes":lanes}),
    )
}
