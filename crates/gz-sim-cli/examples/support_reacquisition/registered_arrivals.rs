// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fractional cold-corner arrival through the public Bevy clock and standard SDF loader.
#[path = "registered_ballistic.rs"]
pub(crate) mod ballistic;
use super::*;
use std::time::Instant;

#[derive(Clone, Copy)]
struct Case {
    rot: DQuat,
    pad_v: DVec3,
    mu: f64,
    prefix: u64,
    phase: f64,
    pad_first: bool,
}
impl Case {
    fn ns(self) -> f64 {
        (self.prefix * 500_000) as f64 + self.phase
    }
    fn xml(self) -> String {
        self.xml_with_gravity(DVec3::ZERO)
    }
    fn xml_with_gravity(self, g: DVec3) -> String {
        let tau = self.ns() * 1e-9;
        let f = Setup::new(1., DVec3::new(0.2, 1., 0.25), 0.31, self.pad_v.z);
        let xyz = |v: DVec3| format!("{:.17} {:.17} {:.17}", v.x, v.y, v.z);
        let pose = |p: DVec3, q: DQuat| {
            let (yaw, pitch, roll) = q.to_euler(glam::EulerRot::ZYX);
            format!("{} {roll:.17} {pitch:.17} {yaw:.17}", xyz(p))
        };
        let fp = f.centers[0] + DVec3::Z * self.pad_v.z * tau - 0.5 * g * tau * tau;
        let dp = DVec3::new(-self.pad_v.x * tau, -self.pad_v.y * tau, -PAD_HALF[2]);
        let mut s = SDF
            .replace(
                "<gravity>0 0 0</gravity>",
                &format!("<gravity>{}</gravity>", xyz(self.rot * g)),
            )
            .replace(
                "0 0 0.12817698440714029 -0.18 0.31 0.15",
                &pose(self.rot * fp, self.rot * f.q[0]),
            )
            .replace(
                "<pose>0 0 -0.06 0 0 0</pose>",
                &format!("<pose>{}</pose>", pose(self.rot * dp, self.rot)),
            )
            .replace("1.1 -0.7 1.4", &xyz(self.rot * self.pad_v))
            .replace(
                "<mu>0.6</mu><mu2>0.6</mu2>",
                &format!("<mu>{}</mu><mu2>{}</mu2>", self.mu, self.mu),
            );
        if self.pad_first {
            let a = s.find("    <model name=\"rocker\">").unwrap();
            let d = s.find("    <model name=\"lift\">").unwrap();
            let end = s[d..].find("</model>").unwrap() + d + 8;
            s = format!("{}{}\n{}{}", &s[..a], &s[d..end], &s[a..d], &s[end..]);
        }
        s
    }
    fn target(self) -> Result<DVec3> {
        let c = Approach {
            frame: Setup::new(1., DVec3::new(0.2, 1., 0.25), 0.31, self.pad_v.z),
            slip: DVec3::new(-self.pad_v.x, -self.pad_v.y, 0.),
            spin: DVec3::ZERO,
            hinge: 0.,
            mu: self.mu,
        };
        Ok(self.rot * DVec3::from(c.target()?.impulse))
    }
}
#[derive(Clone, Copy, Debug, Serialize)]
struct Measurement {
    projected: u32,
    first_load_ns: Option<u64>,
    projection_ns: Option<u64>,
    gap_m: Option<f64>,
    first_interval_impulse_error_ns: f64,
    first_interval_normal_ns: f64,
    projection_error_ns: Option<f64>,
    prior_reaction_ns: f64,
    impact_tick_ns: u64,
}
#[derive(Clone, Copy)]
struct Budget {
    enabled: bool,
    steps: u32,
    relax: Option<usize>,
}
const DEFAULT: Budget = Budget {
    enabled: true,
    steps: 8,
    relax: None,
};
#[derive(Default)]
struct MotionSample {
    center: DVec3,
    relative: DQuat,
    spin: DVec3,
}
struct Trial {
    impact: Measurement,
    trace: Vec<Value>,
    motion: Vec<MotionSample>,
    calls: u32,
}
fn run(c: Case, budget: Budget, save: bool, follow: u64) -> Result<Trial> {
    let relaxation = budget
        .relax
        .map(|n| format!("<relaxation_iterations>{n}</relaxation_iterations>"))
        .unwrap_or_default();
    let xml = c.xml().replace(
        "<native_steps>8</native_steps>",
        &format!("<native_steps>{}</native_steps>{relaxation}", budget.steps),
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "arrival.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    primary_contact::set_enabled(&mut sim, budget.enabled)?;
    #[allow(deprecated)]
    let get = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<BodyHandle>()
            .unwrap()
            .id()
    };
    let (foot, top, pad) = (
        get("default::rocker::foot"),
        get("default::rocker::upper"),
        get("default::lift::pad"),
    );
    let mut frames = vec![];
    let mut motion = Vec::new();
    let mut hit_calls = 0;
    let hit_tick = c.prefix + 1;
    let target = c.target()?;
    let mut m = Measurement {
        projected: 0,
        first_load_ns: None,
        projection_ns: None,
        gap_m: None,
        first_interval_impulse_error_ns: 0.,
        first_interval_normal_ns: 0.,
        projection_error_ns: None,
        prior_reaction_ns: 0.,
        impact_tick_ns: 0,
    };
    for tick in 0..=hit_tick + if save { 24 } else { follow.max(1) } {
        let mut j = 0.;
        let mut calls = 0;
        if tick > 0 {
            let now = Instant::now();
            sim.step_exact(1)?;
            let duration = now.elapsed().as_nanos() as u64;
            if tick == hit_tick {
                m.impact_tick_ns = duration;
            }
            let start = (tick - 1) * 500_000;
            if let Some(r) = primary_contact::last_refinement(&sim).filter(|r| r.iteration == tick)
            {
                calls = r.interval.native_steps;
                for p in &r.requested_pairs {
                    if p.foot != foot || p.support != pad {
                        continue;
                    }
                    j += p.normal_impulse;
                    if let Some(t) = p.first_positive_end {
                        m.first_load_ns.get_or_insert(start + t.as_nanos() as u64);
                    }
                }
                for h in &r.interval.corner_impacts {
                    m.projected += 1;
                    m.projection_ns = Some(start + h.elapsed.as_nanos() as u64);
                    m.gap_m = Some(h.gap);
                    m.projection_error_ns = Some(DVec3::from(h.impulse_on_foot).distance(target));
                }
            } else {
                calls = 1;
                let backend = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                for p in backend.rigid_contact_observations() {
                    if (p.a, p.b) == (foot.min(pad), foot.max(pad)) {
                        j += p.normal_impulse;
                    }
                }
                if j > 0. {
                    m.first_load_ns.get_or_insert(tick * 500_000);
                }
            }
            if tick < hit_tick {
                m.prior_reaction_ns += j;
            }
        }
        let res = sim.world().resource::<PhysicsBackend>();
        let [a, b, d] = [foot, top, pad].map(|id| Body::from(res.state(id).unwrap()));
        if tick == hit_tick {
            hit_calls = calls;
            m.first_interval_normal_ns = j;
            m.first_interval_impulse_error_ns =
                (DVec3::from(a.velocity) + DVec3::from(b.velocity)).distance(target);
        }
        if follow > 0 && tick >= hit_tick {
            motion.push(MotionSample {
                center: 0.5 * (DVec3::from(a.position) + DVec3::from(b.position)),
                relative: DQuat::from_array(a.orientation).inverse()
                    * DQuat::from_array(b.orientation),
                spin: DVec3::from(a.angular_velocity),
            });
            if tick == hit_tick {
                primary_contact::set_enabled(&mut sim, false)?;
            }
        }
        if save {
            frames.push(json!({"tick":tick,"time_us":tick*500,"foot":a,"upper":b,"pad":d,"native_steps":calls,"solver_iterations":4,"normal_impulse_ns":j}));
        }
    }
    Ok(Trial {
        impact: m,
        trace: frames,
        motion,
        calls: hit_calls,
    })
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut n = 0;
    let mut accepted = 0;
    let mut max_j = 0.0_f64;
    let mut max_time = 0.0_f64;
    for (ri, rot) in [
        DQuat::IDENTITY,
        DQuat::from_euler(glam::EulerRot::ZYX, 0.12, 0.58, -0.83),
        DQuat::from_rotation_y(1.9),
    ]
    .into_iter()
    .enumerate()
    {
        for order in [false, true] {
            for prefix in [0, 1, 5, 25, 128] {
                for phase in [1.25, 197.3, 62_499.9, 138_197.65, 250_001.35, 499_997.1] {
                    for speed in [0.65, 1.4] {
                        for mu in [0., 0.2, 0.6, 1.] {
                            for dir in 0..4 {
                                let theta = 0.53 + dir as f64 * std::f64::consts::FRAC_PI_2;
                                let c = Case {
                                    rot,
                                    pad_v: DVec3::new(1.3 * theta.cos(), 1.3 * theta.sin(), speed),
                                    mu,
                                    prefix,
                                    phase,
                                    pad_first: order,
                                };
                                let show = ri == 1
                                    && !order
                                    && prefix == 5
                                    && phase == 138_197.65
                                    && speed == 1.4
                                    && mu == 0.6
                                    && dir == 1;
                                let Trial {
                                    impact: plain,
                                    trace: p,
                                    ..
                                } = run(
                                    c,
                                    Budget {
                                        enabled: false,
                                        ..DEFAULT
                                    },
                                    show,
                                    0,
                                )?;
                                let Trial {
                                    impact: hybrid,
                                    trace: h,
                                    ..
                                } = run(c, DEFAULT, show, 0)?;
                                n += 1;
                                accepted += usize::from(hybrid.projected == 1);
                                if let Some(t) = hybrid.projection_ns {
                                    max_time = max_time.max((t as f64 - c.ns()).abs());
                                }
                                if let Some(j) = hybrid.projection_error_ns {
                                    max_j = max_j.max(j);
                                }
                                if show {
                                    let lanes = [
                                        ("ordinary", "Ordinary SDF", p),
                                        ("hybrid", "Projected corner, same actuator clock", h),
                                    ];
                                    let mut entries = vec![];
                                    for (id, label, frames) in lanes {
                                        write_json(
                                            &out.join("traces").join(format!("{id}.json")),
                                            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
                                        )?;
                                        entries.push(json!({"label":id,"name":label,"file":format!("traces/{id}.json")}));
                                    }
                                    write_json(
                                        &out.join("comparison.json"),
                                        &json!({"title":"A free ankle lands between actuator updates","lanes":entries,"pair_totals_only":true,"expected_ns":c.ns(),"target_world_ns":c.target()?.to_array(),"measured":[plain,hybrid]}),
                                    )?;
                                }
                                rows.push(json!({"rotation":ri,"pad_first":order,"prefix":prefix,"phase_ns":phase,"closing":speed,"mu":mu,"direction":dir,"ordinary":plain,"projected":hybrid}));
                            }
                        }
                    }
                }
            }
        }
    }
    write_json(
        &out.join("study.json"),
        &json!({"cases":n,"projected_once":accepted,"max_projected_timing_error_ns":max_time,"max_projected_impulse_error_ns":max_j,"rows":rows}),
    )
}

/// Randomized profile order within a fixture. Common native continuation after
/// the impact isolates the one-tick budget in the following 10 ms of motion.
pub(crate) fn cost(out: &Path) -> Result<()> {
    const PROFILES: [(u32, usize); 11] = [
        (0, 1),
        (2, 1),
        (2, 8),
        (3, 1),
        (3, 8),
        (4, 1),
        (4, 8),
        (8, 1),
        (8, 8),
        (16, 8),
        (64, 8),
    ];
    let mut rows = Vec::new();
    let mut samples: Vec<Vec<[f64; 6]>> = vec![vec![]; PROFILES.len()];
    let mut seed = 0x7a41d56bu32;
    for i in 0..328 {
        let k = i % 64;
        let phase = [
            1.25, 197.3, 62_499.9, 138_197.65, 249_999.35, 312_001.2, 484_375.4, 499_997.1,
        ][k % 8];
        let a = 0.253 + (k / 8) as f64 * 0.79;
        let c = Case {
            rot: DQuat::from_euler(glam::EulerRot::ZYX, 0.3 * a, -0.23, 0.14 * a),
            pad_v: DVec3::new(a.cos(), a.sin(), if k % 3 == 0 { 0.65 } else { 1.4 }),
            mu: [0., 0.2, 0.6, 1.][(k / 4) % 4],
            prefix: 5,
            phase,
            pad_first: k % 2 == 1,
        };
        let mut order: Vec<_> = (0..PROFILES.len()).collect();
        for j in (1..order.len()).rev() {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            order.swap(j, seed as usize % (j + 1));
        }
        let mut trials = Vec::new();
        for id in order {
            let (calls, r) = PROFILES[id];
            let result = run(
                c,
                Budget {
                    enabled: calls != 0,
                    steps: calls.max(2),
                    relax: Some(r),
                },
                false,
                20,
            )?;
            if calls != 0
                && (result.impact.projected != 1
                    || result.calls != calls
                    || result.impact.projection_error_ns.is_none_or(|x| x > 1e-10))
            {
                return Err(format!(
                    "case {k}, requested {calls}, native calls {}, projections {}",
                    result.calls, result.impact.projected
                )
                .into());
            }
            trials.push((id, result));
        }
        if i < 8 {
            continue;
        }
        let reference = &trials
            .iter()
            .find(|t| t.0 == PROFILES.len() - 1)
            .unwrap()
            .1
            .motion;
        for (id, t) in &trials {
            let mut errs = [0.0_f64; 3];
            for (v, r) in t.motion.iter().zip(reference) {
                errs[0] = errs[0].max(v.center.distance(r.center));
                let q = v.relative * r.relative.conjugate();
                errs[1] = errs[1].max(2. * q.xyz().length().atan2(q.w.abs()));
                errs[2] = errs[2].max(v.spin.distance(r.spin));
            }
            let m = t.impact;
            let vals = [
                m.impact_tick_ns as f64,
                m.first_interval_impulse_error_ns,
                m.projection_error_ns.unwrap_or(0.),
                errs[0],
                errs[1],
                errs[2],
            ];
            samples[*id].push(vals);
            rows.push(json!({"fixture":k,"repetition":i,"profile":id,"calls":t.calls,"measured":m,"max_com_m":errs[0],"max_relative_angle_rad":errs[1],"max_spin_rad_s":errs[2]}));
        }
    }
    let c = Case {
        rot: DQuat::IDENTITY,
        pad_v: DVec3::new(1.1, -0.7, 1.4),
        mu: 0.6,
        prefix: 5,
        phase: 138_197.65,
        pad_first: false,
    };
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    for (steps, id, label) in [
        (0, "ordinary", "Ordinary native contact"),
        (2, "two", "Two native calls"),
        (8, "eight", "Eight native calls"),
    ] {
        let r = run(
            c,
            Budget {
                enabled: steps != 0,
                steps: steps.max(2),
                relax: Some(1),
            },
            true,
            24,
        )?;
        write_json(
            &out.join("traces").join(format!("{id}.json")),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":r.trace}),
        )?;
        lanes.push(
            json!({"label":id,"name":label,"file":format!("traces/{id}.json"),"impact":r.impact}),
        );
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Two native calls at a certified free-ankle landing","pair_totals_only":true,"lanes":lanes}),
    )?;
    let mut summary = vec![];
    for (id, s) in samples.iter().enumerate() {
        let stats:Vec<_>=(0..6).map(|j|{let mut a:Vec<_>=s.iter().map(|r|r[j]).collect();a.sort_by(f64::total_cmp);
            json!({"median":a[a.len().div_ceil(2)-1],"p95":a[(95*a.len()).div_ceil(100)-1],"max":a[a.len()-1]})}).collect();
        summary.push(json!({"profile":id,"steps":PROFILES[id].0,"relaxation":PROFILES[id].1,"n":s.len(),"cost_ns":stats[0],"impulse_error_ns":stats[1],"projected_error_ns":(id>0).then_some(&stats[2]),"trajectory_max_com_m":stats[3],"trajectory_max_joint_rad":stats[4],"trajectory_max_spin_rad_s":stats[5]}));
    }
    fs::create_dir_all(out)?;
    write_json(
        &out.join("results.json"),
        &json!({"frames_after_hit":20,"cases":64,"repetitions_per_case":5,"reference":"64 calls / 8 relaxation; all branches ordinary after impact","summary":summary,"rows":rows}),
    )
}
