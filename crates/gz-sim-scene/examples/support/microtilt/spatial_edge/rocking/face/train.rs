// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Repeated flat crossings under gravity, checked by the Cartesian reference.
use super::super::transaction::frame;
use super::super::two_bearing::registered::native_state;
use super::*;
use gz_physics::{PlanarSoleRocking, RockingCycle};
const DRIFT: DVec3 = DVec3::new(0.037, -0.026, 0.);
struct Train {
    model: PlanarSoleRocking,
    shape: Shape,
    yaw: DQuat,
    sign: f64,
    cycles: Vec<(f64, RockingCycle)>,
    limit: [f64; 2],
}
impl Train {
    fn new(shape: Shape, inertia: f64, speed: f64, yaw: f64, sign: f64) -> Result<Self> {
        let mass = shape.mass;
        let model = PlanarSoleRocking {
            mass,
            center_height: -shape.vertices[0].z,
            half_width: shape.vertices[0].y.abs(),
            roll_inertia: inertia,
            gravity: -shape.gravity.z,
        };
        let shape = Shape {
            inertia: DMat3::from_diagonal(DVec3::new(inertia, inertia + 0.022, inertia + 0.012)),
            ..shape
        };
        let limit = model
            .settling_limit(speed, 2e-8)?
            .ok_or("unqualified rocking limit")?
            .time_bounds();
        let r = model.speed_ratio()?;
        let mut cycles = vec![];
        let mut t = 0.;
        let mut v = speed;
        while v > 1e-14 {
            let c = model.cycle(v)?.ok_or("unsafe training cycle")?;
            cycles.push((t, c));
            t += 2. * c.half_period;
            v *= r;
        }
        if !(limit[0]..=limit[1]).contains(&t) {
            return Err(format!("unbounded physical limit {t} vs {limit:?}").into());
        }
        Ok(Self {
            model,
            shape,
            yaw: DQuat::from_rotation_z(yaw),
            sign,
            cycles,
            limit,
        })
    }
    fn build(&self, t: f64, theta: f64, w: f64, h: f64, v: f64) -> State {
        let q = self.yaw * DQuat::from_rotation_x(theta);
        State {
            c: t * DRIFT + h * DVec3::Z,
            v: DRIFT + v * DVec3::Z,
            motion: Motion {
                q,
                w: self.yaw * DVec3::X * w,
            },
        }
    }
    fn eval(&self, t: f64) -> Result<State> {
        let k = self.cycles.partition_point(|x| x.0 <= t).saturating_sub(1);
        let (begin, c) = self.cycles[k];
        let dt = t - begin;
        if dt >= 2. * c.half_period {
            return Ok(self.build(t, 0., 0., self.model.center_height, 0.));
        }
        let s = c.sample(dt)?;
        let sign = if k % 2 == 0 { self.sign } else { -self.sign };
        Ok(self.build(
            t,
            sign * s.angle,
            sign * s.angular_speed,
            s.center_height,
            s.normal_velocity,
        ))
    }
    fn physical(&self) -> Result<Value> {
        let mut stats = [0f64; 7];
        let mut trials = vec![];
        let r = self.model.speed_ratio()?;
        let mut total = 0.;
        for (k, &(t, c)) in self.cycles.iter().enumerate() {
            let j = self.model.mass * self.model.half_width * c.launch_speed * (1. + r);
            total += j;
            if k >= 4 || c.launch_speed < 1e-6 {
                continue;
            }
            let sign = if k % 2 == 0 { self.sign } else { -self.sign };
            let sites = if sign > 0. { [0, 2] } else { [1, 3] };
            let start = self.build(
                t,
                0.,
                sign * c.launch_speed,
                self.model.center_height,
                self.model.half_width * c.launch_speed,
            );
            let arc = Arc {
                shape: self.shape,
                start,
                sites: Sites::Two(sites),
            };
            let mut spin = start.motion;
            let mut end = start;
            let mut loads = [0.; 2];
            let count = (2. * c.half_period / 0.0000625).ceil().max(8.) as u32;
            let dt = 2. * c.half_period / f64::from(count);
            for i in 0..count {
                let (next, js) = arc.advance_loaded(spin, dt)?;
                for (j, v) in loads.iter_mut().zip(js) {
                    *j += v;
                }
                spin = next;
                let rel = f64::from(i + 1) * dt;
                end = arc.state(rel, spin);
                if (i + 1) % 23 == 0 || i + 1 == count {
                    let y = c.sample(rel.min(2. * c.half_period))?;
                    let expect = self.build(
                        t + rel,
                        sign * y.angle,
                        sign * y.angular_speed,
                        y.center_height,
                        y.normal_velocity,
                    );
                    stats[0] = stats[0].max(expect.c.distance(end.c));
                    stats[1] = stats[1].max(expect.v.distance(end.v));
                    stats[2] = stats[2].max(end.motion.w.distance(expect.motion.w));
                    let force = arc.load(spin)?;
                    stats[3] = stats[3].max((force.iter().sum::<f64>() - y.supporting_force).abs());
                }
            }
            let jump = Sole {
                shape: self.shape,
                incoming: end,
            }
            .solve()?;
            let area = self.model.mass
                * (2. * c.half_period * self.model.gravity
                    - 2. * self.model.half_width * c.launch_speed);
            stats[4] = stats[4].max((area - loads.iter().sum::<f64>()).abs());
            stats[5] = stats[5].max((jump.total - j).abs());
            stats[6] = stats[6].max((jump.after.motion.w.length() - r * c.launch_speed).abs());
            if !matches!(jump.mode, Mode::Face | Mode::Edge(..)) {
                return Err(format!("unexpected rocking jump {:?}", jump.mode).into());
            }
            trials.push(json!({"cycle":k,"begin_s":t,"duration_s":2.*c.half_period,"launch_rad_s":c.launch_speed,"turned_rad":c.turn_angle,"before_and_after_ns":[area,j],"spatial_ns":[loads.iter().sum::<f64>(),jump.total],"incoming_w":end.motion.w.to_array(),"outgoing_w":jump.after.motion.w.to_array(),"mode":format!("{:?}",jump.mode)}));
        }
        let j0 = self.model.mass * self.model.half_width * self.cycles[0].1.launch_speed * (1. + r)
            / (1. - r);
        if (total - j0).abs() > 1e-11 || stats.iter().any(|v| !v.is_finite() || *v > 5e-9) {
            return Err(format!("rocking limit disagrees with Newton–Euler {stats:?}").into());
        }
        Ok(
            json!({"speed_ratio":r,"settled_s":self.limit,"sum_instantaneous_ns":j0,"resolved_sum_ns":total,"tested_spatial_cycles":trials,"errors_com_velocity_spin_force_smooth_impulse_speed":stats}),
        )
    }
    fn source(&self, i: Input, mode: &str, tilted: bool) -> Result<String> {
        let sole = Sole {
            shape: self.shape,
            incoming: self.eval(0.)?,
        };
        let src = observations::source_with(
            i,
            sole,
            sole.shape.gravity,
            frame(tilted),
            false,
            if matches!(mode, "transfer" | "limit") {
                "normal"
            } else {
                mode
            },
        );
        Ok(if mode == "limit" {
            src.replace("</project_frictionless_sole>","</project_frictionless_sole><follow_frictionless_edge>true</follow_frictionless_edge><resolve_rocking_limit>true</resolve_rocking_limit>")
        } else if mode == "transfer" {
            src.replace("</project_frictionless_sole>","</project_frictionless_sole><follow_frictionless_edge>true</follow_frictionless_edge>")
        } else {
            src
        })
    }
    fn actual(
        &self,
        i: Input,
        mode: &str,
        save: bool,
        tilted: bool,
    ) -> Result<(Value, Vec<Value>)> {
        let mut sim = load(&self.source(i, mode, tilted)?, true)?;
        let id = ids(&sim)?;
        let clocks = (self.limit[1] / DT).ceil() as u64;
        let last = clocks + 12;
        let mut last_state = self.eval(0.)?;
        let mut settled = last_state;
        let mut last_registered = 0;
        let mut events = vec![];
        let mut frames = vec![];
        let mut maxspin = 0f64;
        let mut total = 0.;
        let mut bounds = None;
        let mut jumps = 0.;
        let mut errors = [0f64; 5];
        let mut steps = 0u64;
        let (last_clock, last_cycle) = self.cycles.last().unwrap();
        let true_limit = *last_clock + 2. * last_cycle.half_period;
        for tick in 0..=last {
            let (calls, j) = if tick == 0 {
                (0, 0.)
            } else {
                sim.step_exact(1)?;
                if let Some(r) = last_refinement(&sim).filter(|r| r.iteration == tick) {
                    if !r.interval.sole_transfers.is_empty()
                        || !r.interval.two_bearing_edges.is_empty()
                    {
                        last_registered = tick;
                    }
                    for h in &r.interval.rocking_limits {
                        let x = h.response;
                        let start = (r.start_time + h.start).as_secs_f64();
                        let end = start + h.duration.as_secs_f64();
                        let mut expected = 0.;
                        let ratio = self.model.speed_ratio()?;
                        let mut first = None;
                        for (k, (t, c)) in self.cycles.iter().enumerate() {
                            let hit = *t + 2. * c.half_period;
                            if hit > start && hit <= end {
                                let j = self.model.mass
                                    * self.model.half_width
                                    * c.launch_speed
                                    * (1. + ratio);
                                expected += j;
                                first.get_or_insert((hit, j, k));
                            }
                        }
                        // The independent train sum was truncated below 1e-14 rad/s.
                        errors[0] = errors[0].max((x.integrated_impact_impulse - expected).abs());
                        if let (Some(p), Some((time, j, k))) = (x.first_impact, first) {
                            let delta = (time - start - p.seconds).abs();
                            errors[1] = errors[1].max(delta);
                            if delta > 2e-8 || (p.normal_impulse - j).abs() > 1e-7 {
                                return Err(format!("wrong native crossing {k} at {time} vs {} with impulse {} vs {j}",start+p.seconds,p.normal_impulse).into());
                            }
                            if save {
                                events.push(json!({"tick":tick,"time_s":start+p.seconds,"normal_ns":p.normal_impulse,"pressure_center":p.pressure_center,"counted_tail":x.includes_infinite_tail}));
                            }
                        }
                        if let Some(w) = x.settle_time_bounds {
                            let w = w.map(|v| v + start);
                            let distance = (w[0] - true_limit).max(true_limit - w[1]).max(0.);
                            if distance > 1e-8 {
                                return Err(format!(
                                    "physical collapse {true_limit} outside native window {w:?}, tick {tick}, mode {mode}, previous {bounds:?}, tilt {tilted}, angle {}, rate {}, response {x:?}",h.signed_initial_angle,h.initial_angular_rate
                                )
                                .into());
                            }
                            if bounds.replace(w).is_some() {
                                return Err("repeated finite collapse".into());
                            }
                        }
                        jumps += x.integrated_impact_impulse;
                        steps += 1;
                        last_registered = tick;
                    }
                    for h in &r.interval.sole_transfers {
                        events.push(json!({"tick":tick,"time_s":r.start_time.as_secs_f64()+h.pressure.physical_elapsed_seconds,"normal_ns":h.pressure.normal_impulse}));
                    }
                    (r.interval.native_steps, r.requested_pairs[0].normal_impulse)
                } else {
                    (
                        1,
                        engine(&sim)
                            .rigid_contact_observations()
                            .iter()
                            .filter(|x| {
                                [x.a, x.b].contains(&id.foot) && [x.a, x.b].contains(&id.pad)
                            })
                            .map(|x| x.normal_impulse)
                            .sum(),
                    )
                }
            };
            let x = native_state(&sim, &id, frame(tilted));
            if mode == "limit" {
                let oracle = self.eval(tick as f64 * DT)?;
                errors[2] = errors[2].max(x.c.distance(oracle.c));
                errors[3] = errors[3].max(x.v.distance(oracle.v));
                errors[4] = errors[4].max(x.motion.w.distance(oracle.motion.w));
                if tick > 0
                    && (last_registered != tick
                        || errors.iter().any(|v| !v.is_finite() || *v > 3e-7))
                {
                    return Err(format!("sustained limit missed {tick} last {last_registered}, tilted {tilted} I {} speed {} sign {} errors {errors:?}",self.model.roll_inertia,self.cycles[0].1.launch_speed,self.sign).into());
                }
            }
            if tick == clocks {
                settled = x;
            }
            if tick >= clocks {
                maxspin = maxspin.max(x.motion.w.length());
            }
            total += j;
            last_state = x;
            if save {
                let (f, u, p) = departure::ingest(engine(&sim), &id);
                frames.push(row(tick, calls, j, (f, u.unwrap(), p)));
            }
        }
        let baseline = self.eval(last as f64 * DT)?;
        let initial = self.eval(0.)?;
        let j =
            self.model.mass * (baseline.v.z - initial.v.z + self.model.gravity * last as f64 * DT);
        Ok((
            json!({"mode":mode,"tilted":tilted,"limit_steps":steps,"limit_errors":errors,"sum_registered_jumps_ns":jumps,"registered_settle_window_s":bounds,"settling_tick":clocks,"spin_at_settling_rad_s":settled.motion.w.length(),"max_spin_after_settling_rad_s":maxspin,"last_registered_tick":last_registered,"events":events,"last_tick":last,"end_com_error_m":last_state.c.distance(baseline.c),"end_speed_error_m_s":last_state.v.distance(baseline.v),"total_normal_ns":total,"expected_total_ns":j,"total_error_ns":(total-j).abs()}),
            frames,
        ))
    }
}
fn row(tick: u64, calls: u32, normal: f64, (f, u, p): (BodyState, BodyState, BodyState)) -> Value {
    json!({"tick":tick,"time_us":tick*500,"native_steps":calls,"solver_iterations":if calls==0{0}else{4},"selected":calls>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":normal,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)})
}
fn replay(out: &Path, i: Input, train: &Train, registered: bool) -> Result<()> {
    let mut runs = vec![];
    let last = (train.limit[1] / DT).ceil() as u64 + 12;
    for mode in [
        "ordinary",
        if registered { "limit" } else { "transfer" },
        "reference",
    ] {
        let (report, frames) = if mode == "reference" {
            let mut frames = vec![];
            let mut prev = train.eval(0.)?;
            for tick in 0..=last {
                let t = tick as f64 * DT;
                let x = train.eval(t)?;
                let j = if tick == 0 {
                    0.
                } else {
                    train.model.mass * (x.v.z - prev.v.z + train.model.gravity * DT)
                };
                frames.push(row(tick, 0, j, super::super::replay::bodies(i, t, x)));
                prev = x;
            }
            (json!({"mode":mode,"settlement_s":train.limit}), frames)
        } else {
            train.actual(i, mode, true, false)?
        };
        runs.push(report);
        fs::write(
            out.join(format!("traces/{mode}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":frames}),
            )?,
        )?;
    }
    let lanes=[("ordinary","Ordinary native solve"),(if registered{"limit"}else{"transfer"},if registered{"Registered finite rocking limit"}else{"Registered edge transfer"}),("reference","Independent finite settling time")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="reference"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A rocking sole loses energy at each return and settles in finite time","pair_totals_only":true,"capture_ticks":{"first_return":(2.*train.cycles[0].1.half_period/DT).ceil() as u64,"settled":last-12},"lanes":lanes}),
        )?,
    )?;
    fs::write(
        out.join("train.sdf"),
        train.source(i, if registered { "limit" } else { "transfer" }, false)?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&runs)?)?;
    Ok(())
}
pub(in super::super::super::super::super::super) fn run(
    out: &Path,
    registered: bool,
    benchmark: bool,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, path, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        yaw: 0.,
        opening: 0.3,
        pitch_rate: 0.6,
    });
    if benchmark {
        return cost(out, i, &Train::new(path.shape, 0.045, 0.45, 0.73, 1.)?);
    }
    let mut field = vec![];
    for inertia in [0.015, 0.02, 0.026, 0.045, 0.09, 0.17] {
        for speed in [0.16, 0.45, 0.9, 1.45] {
            for yaw in [0., 0.73] {
                for sign in [-1., 1.] {
                    let train = Train::new(path.shape, inertia, speed, yaw, sign)?;
                    let physical = train.physical()?;
                    let mut native = vec![];
                    if registered {
                        for tilted in [false, true] {
                            native.push(train.actual(i, "limit", false, tilted)?.0);
                        }
                    } else {
                        for mode in ["ordinary", "transfer"] {
                            native.push(train.actual(i, mode, false, false)?.0);
                        }
                    }
                    field.push(json!({"inertia":inertia,"speed":speed,"yaw":yaw,"sign":sign,"physical":physical,"native":native}));
                }
            }
        }
    }
    fs::write(out.join("cycles.json"), serde_json::to_vec(&field)?)?;
    replay(
        out,
        i,
        &Train::new(path.shape, 0.045, 0.45, 0.73, 1.)?,
        registered,
    )
}
// Linux's per-thread CPU clock separates execution from competing simulations
// on the same host. Wall time includes Bevy scheduling and all clock waits.
fn thread_cpu() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
        Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}
fn cost(out: &Path, i: Input, d: &Train) -> Result<()> {
    let mut runs = vec![];
    let models = ["ordinary", "transfer", "limit"];
    for rep in 0..30 {
        for offset in 0..3 {
            let mode = models[(rep + offset) % 3];
            let mut s = load(&d.source(i, mode, false)?, true)?;
            let mut ticks = vec![];
            for k in 1..=118 {
                let c = thread_cpu();
                let w = std::time::Instant::now();
                s.step_exact(1)?;
                let wall = w.elapsed().as_nanos() as u64;
                let cpu = thread_cpu().zip(c).map(|(a, b)| a - b);
                let kind = if k == 60 {
                    "first impact"
                } else if [83, 92, 95, 97, 98].contains(&k) {
                    "late returns"
                } else if k > 98 {
                    "settled"
                } else if k > 3 && k < 55 {
                    "edge before first return"
                } else {
                    "other"
                };
                ticks.push(json!({"tick":k,"kind":kind,"wall_ns":wall,"caller_cpu_ns":cpu}));
            }
            runs.push(json!({"mode":mode,"repetition":rep,"ticks":ticks}));
        }
    }
    fs::write(out.join("cost.json"), serde_json::to_vec(&runs)?)?;
    Ok(())
}
