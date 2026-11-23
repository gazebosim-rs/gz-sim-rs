// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The two belts start and stop repeatedly. An outside-zone policy can demote
//! only once the stationary support reaction and the requested dwell are present.
use super::*;
use gz_sim_scene::{
    fidelity_zone::{self, FidelityZone},
    queue_link_world_velocity,
};
#[derive(Clone, Copy)]
struct Trial {
    mu: f64,
    theta: f64,
    stop: f64,
    dwell: Duration,
}
struct Run {
    trace: Vec<Value>,
    summary: Value,
    states: Vec<BodyState>,
    pulses: Vec<[f64; 2]>,
    held: Vec<bool>,
}
const STARTS: [u64; 3] = [17, 111, 205];
const STROKE: u64 = 36;
const TOTAL: u64 = 298;
impl Trial {
    fn scene(self) -> String {
        SOURCE.replace(
            "<mu>0.31</mu><mu2>0.31</mu2>",
            &format!("<mu>{0}</mu><mu2>{0}</mu2>", self.mu),
        )
    }
    fn run(self, src: &str, tiered: bool, trace: bool) -> Result<Run> {
        let mut s = load(src, true)?;
        let ids = ids(&s)?;
        let model = model(&s, "default::rocker");
        let toe = bind(&s, "default::toe::pad")?;
        let heel = bind(&s, "default::lift::pad")?;
        let pose = DQuat::from_array(engine(&s).state(heel.1).unwrap().orientation);
        if tiered {
            fidelity_zone::configure_with_eligibility_dwell(
                &mut s,
                model,
                vec![FidelityZone {
                    center: [90., 0., 0.],
                    promote_radius: 0.5,
                    demote_radius: 1.,
                }],
                Duration::ZERO,
                self.dwell,
            )?;
        }
        let mut changes = vec![];
        let mut wakes = vec![];
        let mut contacts = vec![];
        let mut remain = false;
        let mut rec = vec![];
        let mut states = vec![];
        let mut pulses = vec![];
        let mut held = vec![];
        let mut prev = false;
        let mut speeds = vec![];
        for tick in 0..=TOTAL {
            if tick == 12 && tiered {
                // Last-writer replacement during pause leaves no committed motion.
                s.set_paused(true);
                let time = s.elapsed();
                queue_link_world_velocity(&mut s, toe.0, [0.03, 0., 0.], [0.; 3])?;
                queue_link_world_velocity(&mut s, toe.0, [0.; 3], [0.; 3])?;
                s.tick()?;
                if s.elapsed() != time {
                    return Err("pause advanced the actuator clock".into());
                }
                s.set_paused(false);
            }
            for (n, beg) in STARTS.into_iter().enumerate() {
                if tick == beg || tick == beg + STROKE {
                    let axis = pose * DVec2::from_angle(self.theta + n as f64 * 2.27).extend(0.);
                    let v = if tick == beg {
                        axis * (self.mu * 9.81 * self.stop)
                    } else {
                        DVec3::ZERO
                    };
                    s.set_paused(true);
                    let t0 = s.elapsed();
                    for (entity, _) in [heel, toe] {
                        queue_link_world_velocity(&mut s, entity, v.to_array(), [0.; 3])?;
                    }
                    s.tick()?;
                    if s.elapsed() != t0 {
                        return Err("paused belt command consumed simulation time".into());
                    }
                    s.set_paused(false);
                }
            }
            if tick > 0 {
                s.step_exact(1)?;
            }
            let prescribed = model_fidelity(&s, model)? == Prescribed;
            if prescribed != prev {
                changes.push(json!({"tick":tick,"prescribed":prescribed}));
                prev = prescribed;
            }
            if let Some(ev) =
                support_loss_promotion::last_promotion(&s, model).filter(|ev| ev.iteration == tick)
            {
                if ev.start_time != Duration::from_micros((tick - 1) * 500)
                    || ev.link != ids.foot
                    || ![toe.1, heel.1].contains(&ev.support)
                {
                    return Err("incorrect support-loss receipt".into());
                }
                wakes.push(tick);
            }
            let report = last_refinement(&s).filter(|r| r.iteration == tick);
            let next = report
                .and_then(|r| r.interval.shared_faces.last())
                .is_some_and(|r| r.comoving_rest);
            if next && !remain {
                contacts.push(tick);
            }
            remain = next;
            let b = engine(&s);
            let (f, u, p) = departure::ingest(b, &ids);
            let (calls, j) = observe(&s, &ids, toe.1, tick);
            let foot_speed = DVec3::from(f.velocity).length();
            let padspeed = DVec3::from(p.velocity).length();
            if prescribed && (foot_speed > 1e-12 || padspeed > 1e-12) {
                return Err(format!("moving assembly or pad demoted at {tick}").into());
            }
            if STARTS.contains(&tick) || STARTS.iter().any(|v| tick == v + STROKE) {
                speeds.push([foot_speed, padspeed]);
            }
            if trace {
                let mut a = row(tick, calls, j[0] + j[1], (f, u.unwrap(), p));
                a["opponent"] = wire(b.state(toe.1).unwrap());
                a["support_normal_ns"] = json!(j);
                rec.push(a);
            }
            states.push(f);
            pulses.push(j);
            held.push(prescribed);
        }
        if tiered && (wakes != STARTS || changes.len() != 7 || !prev) {
            return Err(format!("mu {} theta {} stop {} dwell {:?}: wakes {wakes:?} transitions {changes:?}, rests {contacts:?}",self.mu,self.theta,self.stop,self.dwell).into());
        }
        Ok(Run {
            trace: rec,
            summary: json!({"transitions":changes,"wakes":wakes,"rests":contacts,"foot_belt_speed_on_commands":speeds}),
            states,
            pulses,
            held,
        })
    }
}
pub(crate) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    for mu in [0.12, 0.31, 0.46] {
        for theta in [-1.1, 0.41] {
            for stop in [0.00121381976, 0.00749999975, 0.01200000025] {
                for ms in [1, 3] {
                    let trial = Trial {
                        mu,
                        theta,
                        stop,
                        dwell: Duration::from_millis(ms),
                    };
                    let src = trial.scene();
                    let selected = mu == 0.31 && theta == 0.41 && stop == 0.00749999975 && ms == 3;
                    let dynamic = trial.run(&src, false, selected)?;
                    let tiered = trial.run(&src, true, selected)?;
                    let mut errors = [0f64; 5];
                    let mut resumed = vec![];
                    for (i, (a, b)) in tiered.states.iter().zip(&dynamic.states).enumerate() {
                        errors[0] = errors[0]
                            .max(DVec3::from(a.position).distance(DVec3::from(b.position)));
                        errors[1] = errors[1]
                            .max(DVec3::from(a.velocity).distance(DVec3::from(b.velocity)));
                        errors[2] = errors[2].max(
                            DVec3::from(a.angular_velocity)
                                .distance(DVec3::from(b.angular_velocity)),
                        );
                        errors[3] = errors[3].max(
                            a.orientation
                                .iter()
                                .zip(b.orientation)
                                .map(|(a, b)| (a - b).abs())
                                .fold(0., f64::max),
                        );
                        if !tiered.held[i] {
                            for k in 0..2 {
                                errors[4] = errors[4]
                                    .max((tiered.pulses[i][k] - dynamic.pulses[i][k]).abs());
                            }
                        }
                        if STARTS.contains(&(i as u64)) {
                            resumed.push(json!({"tick":i,"registered_dynamic":dynamic.pulses[i],"switched":tiered.pulses[i]}));
                        }
                    }
                    rows.push(json!({"mu":mu,"heading":theta,"stop_s":stop,"dwell_ms":ms,"tiered":tiered.summary,"always_dynamic":dynamic.summary,"first_resumed":resumed,"errors_position_velocity_spin_quaternion_load":errors,"prescribed_ticks":tiered.held.iter().filter(|v|**v).count()}));
                    if selected {
                        let other = trial.run(&ordinary(), false, true)?;
                        let mut lanes = vec![];
                        for (name, label, run) in [
                            ("ordinary", "Ordinary always dynamic", other),
                            ("tiered", "Automatic demotion and wake", tiered),
                            ("dynamic", "Registered always dynamic", dynamic),
                        ] {
                            fs::write(
                                out.join(format!("traces/{name}.json")),
                                serde_json::to_vec(
                                    &json!({"geometry":geometry(),"frames":run.trace}),
                                )?,
                            )?;
                            lanes.push(json!({"label":name,"name":label,"file":format!("traces/{name}.json")}));
                        }
                        fs::write(out.join("parked.sdf"), src)?;
                        fs::write(
                            out.join("comparison.json"),
                            serde_json::to_vec_pretty(
                                &json!({"title":"Two moving belts repeatedly wake and repark a foot","pair_totals_only":true,"capture_ticks":{"first_wake":17,"second_wake":111,"third_wake":205,"parked_again":285},"lanes":lanes}),
                            )?,
                        )?;
                    }
                }
            }
        }
    }
    fs::write(out.join("cycles.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
