// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched small-angle impact of a native locked tree and an inertia-equivalent free body.
use super::*;
use gz_physics_multifidelity::{ContactSegment, ContactSolverIterations, RigidContactPatch};
use gz_sim_scene::{primary_contact::last_box_timing, queue_link_world_wrench};
use std::{path::Path, time::Duration};
#[path = "microtilt/guard.rs"]
pub(super) mod guard;
#[path = "microtilt/oracle.rs"]
mod oracle;
#[path = "microtilt/spatial_edge.rs"]
pub(super) mod spatial_edge;
#[path = "microtilt/sliding.rs"]
mod sliding;
pub(super) fn sliding_law(out: &Path) -> Result<()> {
    sliding::run_all(out)
}
use oracle::PoseState;
const ARM: DVec3 = DVec3::new(0., 0., 0.07);
const PAD: DVec3 = DVec3::new(0., 0., 0.06);
const DONOR: u64 = 1530;
type Bodies = (BodyState, Option<BodyState>, BodyState);
fn engine(s: &Simulation) -> &MultiFidelityBackend {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn load(scene: &str, per_point: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        scene,
        "microtilt.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: per_point,
            ..Default::default()
        },
    )?)
}
fn ids(s: &Simulation) -> Result<Reading> {
    Ok(Reading {
        foot: bind(s, "default::rocker::foot")?.1,
        upper: bind(s, "default::rocker::upper").ok().map(|a| a.1),
        pad: bind(s, "default::lift::pad")?.1,
    })
}
#[derive(Clone, Copy)]
struct Seed {
    state: PoseState,
    pad: BodyState,
}
impl Seed {
    fn read((f, u, p): Bodies) -> Self {
        let q = DQuat::from_array(f.orientation);
        let c = if let Some(u) = u {
            (DVec3::from(f.position) + DVec3::from(u.position)) * 0.5
        } else {
            DVec3::from(f.position) + q * ARM
        };
        let v = if let Some(u) = u {
            (DVec3::from(f.velocity) + DVec3::from(u.velocity)) * 0.5
        } else {
            DVec3::from(f.velocity)
        };
        Self {
            state: PoseState {
                c: c - DVec3::from(p.position) - PAD,
                q,
                v: v - DVec3::from(p.velocity),
                w: DVec3::from(f.angular_velocity),
            },
            pad: p,
        }
    }
    fn transform(self, sx: f64, sy: f64, shift: f64) -> Self {
        let s = DVec3::new(sx, sy, 1.);
        let axial = (sx * sy) * s;
        let a = self.state;
        let orient = DQuat::from_xyzw(axial.x * a.q.x, axial.y * a.q.y, axial.z * a.q.z, a.q.w);
        Self {
            state: PoseState {
                q: orient,
                c: a.c * s + DVec3::Z * shift,
                w: a.w * axial,
                v: a.v * s,
            },
            ..self
        }
    }
    fn scene(self, mu: f64, rigid: bool, policy: &str) -> String {
        let a = self.state;
        let p = self.pad;
        let q = a.q;
        let (yaw, pitch, roll) = q.to_euler(glam::EulerRot::ZYX);
        let f = a.c + DVec3::from(p.position) + PAD - q * ARM;
        let v = a.v + DVec3::from(p.velocity);
        let vector = |x: DVec3| format!("{:.17} {:.17} {:.17}", x.x, x.y, x.z);
        let mut s=xml(false,mu,rigid,if matches!(policy, "prospective" | "guarded"){"refresh8"}else{policy})
        .replace("<pose>0 0 0.074 0 0 0</pose>",&format!("<pose>{} {roll:.17} {pitch:.17} {yaw:.17}</pose>",vector(f)))
        .replace("<pose>0 0 -0.06 0 0 0</pose>",&format!("<pose>{} 0 0 0</pose>",vector(p.position.into())))
        .replace("<linear_velocity>0 0 -0.55</linear_velocity><angular_velocity>0 0 0</angular_velocity>",
            &format!("<linear_velocity>{}</linear_velocity><angular_velocity>{}</angular_velocity>",vector(v),vector(a.w)))
        .replace("<linear_velocity>0 0 0.30</linear_velocity>",&format!("<linear_velocity>{}</linear_velocity>",vector(p.velocity.into())));
        if matches!(policy, "prospective" | "guarded") {
            s = s.replace(
                "</support>",
                "</support><prospective_timing>true</prospective_timing>",
            );
        }
        if policy == "guarded" {
            s = s.replace(
                "</prospective_timing>",
                "</prospective_timing><guard_separated_flight>true</guard_separated_flight>",
            );
        }
        s
    }
}
/// The same normal public commands that produced the original first return.
fn donor(material: Option<f64>) -> Result<(Seed, Seed)> {
    let mut s = load(&departure::source(material, "ordinary"), false)?;
    let rig = ids(&s)?;
    let pad = bind(&s, "default::lift::pad")?.0;
    let other = bind(&s, "default::edge_lift::pad")?.0;
    let witness = bind(&s, "default::witness::body")?.0;
    let mut previous = 0.;
    let mut early = None;
    for t in 0..DONOR {
        if t == DONOR - 4 {
            early = Some(Seed::read(departure::ingest(engine(&s), &rig)));
        }
        if t == 1400 {
            s.set_paused(true);
            assert!(s.tick()?.dt.is_zero());
        }
        let v = departure::cmd(t);
        if v != previous {
            for p in [pad, other] {
                queue_link_world_velocity(&mut s, p, [0., 0., v], [0.; 3])?;
            }
            previous = v;
        }
        queue_link_world_wrench(
            &mut s,
            witness,
            [if (t + 1) % 4 < 2 { 24. } else { -16. }, 0., 0.],
            [0.; 3],
        )?;
        s.step_exact(1)?;
    }
    Ok((
        early.unwrap(),
        Seed::read(departure::ingest(engine(&s), &rig)),
    ))
}
#[derive(Clone, Copy)]
struct Case {
    label: &'static str,
    mu: f64,
    material: Option<f64>,
    xs: f64,
    ys: f64,
    gap: f64,
}
#[derive(Debug, Clone, Copy)]
enum Grid {
    Ordinary,
    Uniform(u32),
    Prospective(u32),
    ExactLanding,
    AtEvent(u32),
    ApproachPasses(usize),
    Shoulder(u32),
}
impl Grid {
    fn name(self) -> String {
        match self {
            Self::Ordinary => "ordinary".into(),
            Self::Uniform(n) => format!("uniform{n}"),
            Self::Prospective(n) => format!("prospective{n}"),
            Self::ExactLanding => "arrival_boundary".into(),
            Self::AtEvent(n) => format!("exact_touch_{n}"),
            Self::ApproachPasses(n) => format!("approach_passes_{n}"),
            Self::Shoulder(n) => format!("shoulder{n}"),
        }
    }
}
fn near(a: PoseState, b: PoseState) -> f64 {
    (a.c - b.c)
        .length()
        .max((a.v - b.v).length())
        .max((a.w - b.w).length())
        .max((a.q - b.q).length())
}
fn local_arms() -> [DVec3; 4] {
    [(-0.18, -0.1), (-0.18, 0.1), (0.18, -0.1), (0.18, 0.1)].map(|(x, y)| DVec3::new(x, y, -0.13))
}
fn corner_gaps(s: PoseState) -> [f64; 4] {
    local_arms().map(|r| (s.c + s.q * r).z)
}
fn initial_arrivals(s: PoseState) -> Result<[f64; 4]> {
    let mut times = [0.; 4];
    for (k, arm) in local_arms().into_iter().enumerate() {
        let mut lo = 0.;
        let mut hi = DT;
        if corner_gaps(s)[k] <= 0. || corner_gaps(s.airborne(hi))[k] >= 0. {
            return Err("shoulder control requires a strictly airborne approaching sole".into());
        }
        for _ in 0..45 {
            let mid = (lo + hi) * 0.5;
            let p = s.airborne(mid);
            if (p.c + p.q * arm).z > 0. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        times[k] = 0.5 * (lo + hi);
    }
    Ok(times)
}
fn moments(
    patches: &[RigidContactPatch],
    r: &Reading,
    q: DQuat,
    rigid: bool,
) -> Result<(DVec3, DVec3, [f64; 4])> {
    let mut j = DVec3::ZERO;
    let mut h = DVec3::ZERO;
    let mut corners = [0.; 4];
    for patch in patches {
        let side = if patch.a == r.foot {
            1.
        } else if patch.b == r.foot {
            -1.
        } else {
            continue;
        };
        if [patch.a, patch.b].into_iter().all(|id| id != r.pad) {
            return Err("unexpected foot contact".into());
        }
        for p in &patch.points {
            let arm = DVec3::from(if side > 0. { p.lever_a } else { p.lever_b });
            let t = DVec3::from(
                p.tangent_impulse_on_a
                    .ok_or("per-point friction unavailable")?,
            ) * side;
            let point_j = p.normal_impulse * DVec3::from(patch.normal_on_a) * side + t;
            j += point_j;
            let local = q.conjugate() * arm;
            let index = usize::from(local.x > 0.) * 2 + usize::from(local.y > 0.);
            corners[index] += p.normal_impulse;
            h += (if rigid { arm } else { arm - q * ARM }).cross(point_j);
        }
    }
    Ok((j, h, corners))
}
fn sample(seed: Seed, c: Case, rigid: bool, grid: Grid, o: &oracle::Oracle) -> Result<Value> {
    let full = Duration::from_micros(500);
    let mut initial_seed = seed;
    let mut h = Duration::from_nanos(o.stop_ns);
    if let Grid::AtEvent(_) = grid {
        let mut pad = seed.pad;
        pad.position = (DVec3::from(pad.position) + DVec3::from(pad.velocity) * o.time).to_array();
        initial_seed = Seed {
            state: o.incoming,
            pad,
        };
        h = Duration::from_secs_f64(h.as_secs_f64() - o.time);
    }
    let xml = initial_seed.scene(c.mu, rigid, "ordinary");
    let mut sim = load(&xml, true)?;
    let r = ids(&sim)?;
    let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = physics.downcast_mut::<MultiFidelityBackend>().unwrap();
    let initial = Seed::read(departure::ingest(n, &r)).state;
    let start_error = near(initial, initial_seed.state);
    if start_error > 1e-12 {
        return Err(format!("unmatched incoming states {start_error:e}").into());
    }
    let budget = match grid {
        Grid::Ordinary => 4.into(),
        Grid::AtEvent(k) => ContactSolverIterations {
            temporal: 4,
            pgs: Some(k as usize),
            relaxation: Some(k as usize),
        },
        _ => ContactSolverIterations {
            temporal: 4,
            pgs: Some(1),
            relaxation: Some(8),
        },
    };
    let pair = [(r.foot, r.pad)];
    let fresh = if matches!(
        grid,
        Grid::Prospective(_) | Grid::ExactLanding | Grid::ApproachPasses(_) | Grid::Shoulder(_)
    ) {
        &pair[..]
    } else {
        &[]
    };
    // Collect the live planner's original 500 us grid on an independent
    // complete solve. Only the final interval crossing the common physical
    // cut is shortened; all preceding step durations and settings are fixed.
    let mut timing = None;
    let mut full_first = None;
    let full_grid: Vec<Duration> = match grid {
        Grid::Ordinary => vec![full],
        Grid::Uniform(k) => gz_physics::NativeStepGrid::new(full, k)?
            .intervals()
            .collect(),
        Grid::AtEvent(_) => vec![h],
        Grid::ExactLanding => {
            let onset = Duration::from_nanos((o.time * 1e9).floor() as u64);
            let pre = onset - Duration::from_micros(2);
            gz_physics::NativeStepGrid::new(pre, 7)?
                .intervals()
                .chain([onset - pre, full - onset])
                .collect()
        }
        Grid::Shoulder(k) => {
            // Fixed-budget diagnostic. In this strictly closing family the
            // four ballistic arrival estimates depend only on incoming state.
            let mut t = initial_arrivals(initial)?;
            t.sort_by(f64::total_cmp);
            let first = t[0];
            let pulse = (0.22 * (t[1] - first)).clamp(0.05e-6, 2e-6);
            let near = first - 0.2 * pulse;
            let mut times = vec![];
            let mut elapsed = 0.;
            while times.len() + 2 < k as usize {
                let delta = (0.75 * (first - elapsed)).min(near - elapsed);
                if delta < 20e-9 {
                    break;
                }
                let h = Duration::from_secs_f64(delta);
                elapsed += h.as_secs_f64();
                times.push(h);
            }
            let pulse = Duration::from_secs_f64((first + pulse) - elapsed);
            times.push(pulse);
            let remaining = full - times.iter().sum::<Duration>();
            let count = k - times.len() as u32;
            times.extend(gz_physics::NativeStepGrid::new(remaining, count)?.intervals());
            times
        }
        Grid::Prospective(_) | Grid::ApproachPasses(_) => {
            let k = if let Grid::Prospective(k) = grid {
                k
            } else {
                8
            };
            let mut entire = load(&xml, true)?;
            let pids = ids(&entire)?;
            let ep = [(pids.foot, pids.pad)];
            let mut p = entire.world_mut().resource_mut::<PhysicsBackend>();
            let (receipt, _, plan) = p
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .step_prospective_box_interval(full, k, budget, &ep, &ep)?;
            full_first = receipt
                .pairs
                .iter()
                .find_map(|q| q.first_positive_end)
                .map(|t| t.as_nanos() as u64);
            timing = Some(
                json!({"retimed":plan.retimed,"decline":plan.decline.map(|x|format!("{x:?}")),"durations_ns":plan.durations.iter().map(|d|d.as_nanos()as u64).collect::<Vec<_>>()}),
            );
            plan.durations
        }
    };
    let mut todo = h;
    let mut schedule_elapsed = Duration::ZERO;
    let segments: Vec<ContactSegment> = full_grid
        .iter()
        .copied()
        .map_while(|next| {
            if todo.is_zero() {
                return None;
            }
            let duration = next.min(todo);
            todo -= duration;
            schedule_elapsed += next;
            let iterations = if let Grid::ApproachPasses(k) = grid
                && full_first == Some(schedule_elapsed.as_nanos() as u64)
            {
                ContactSolverIterations {
                    relaxation: Some(k),
                    ..budget
                }
            } else {
                budget
            };
            Some(ContactSegment {
                duration,
                native_steps: 1,
                iterations,
            })
        })
        .collect();
    if !todo.is_zero() {
        return Err("recorded schedule stops before physical readback".into());
    }
    let mut j = DVec3::ZERO;
    let mut moment = DVec3::ZERO;
    let mut corners = [0.; 4];
    let mut last = initial;
    let mut callback_error = None;
    let mut first = None;
    let mut history = Vec::new();
    let mut callback = |s: gz_physics_multifidelity::ContactSubstep<'_>| {
        match moments(&s.contact_patches(), &r, last.q, rigid) {
            Ok((impulse, h, points)) => {
                if impulse.z > 0. {
                    first.get_or_insert(s.elapsed.as_nanos() as u64);
                    if c.label == "measured" && !matches!(grid,Grid::Uniform(n) if n>=128) {
                        let gaps = corner_gaps(last);
                        history.push(json!({"end_ns":s.elapsed.as_nanos()as u64,"duration_ns":s.duration.as_nanos()as u64,
                            "incoming_corner_gaps_m":gaps,"impulse_ns":impulse.to_array(),"quadrants_ns":points,"moment_nms":h.to_array()}));
                    }
                }
                j += impulse;
                moment += h;
                for i in 0..4 {
                    corners[i] += points[i];
                }
            }
            Err(e) => callback_error = Some(e),
        }
        last = Seed::read((
            s.state(r.foot).unwrap(),
            r.upper.and_then(|u| s.state(u)),
            s.state(r.pad).unwrap(),
        ))
        .state;
    };
    let receipt = n
        .observe_contact_schedule(h, &segments, fresh, &mut callback)?
        .0;
    if let Some(e) = callback_error {
        return Err(e);
    }
    let final_state = Seed::read(departure::ingest(n, &r)).state;
    if near(last, final_state) > 1e-12 {
        return Err("last sample disagrees with committed state".into());
    }
    let ideal_h = (o.endpoint.w - initial.w) * 0.05;
    let delta_h = (final_state.w - initial.w) * 0.05;
    Ok(
        json!({"grid":grid.name(),"single_rigid":rigid,"native_calls":receipt.native_steps,"initial_max_component_norm_difference":start_error,
     "first_positive_end_ns":first,"positive_native":history,"source_full_first_positive_ns":full_first,"shadow_duration_ns":h.as_nanos() as u64,"timing":timing,"measured":final_state.wire(),"native_point_impulse_ns":j.to_array(),"native_quadrants_ns":corners,
     "signed_point_moment_nms":moment.to_array(),"delta_h_nms":delta_h.to_array(),"angular_balance_nms":(delta_h-moment).length(),
     "impulse_error_ns":(j-o.impulse).length(),"vertical_impulse_error_ns":(j.z-o.impulse.z).abs(),"angular_impulse_error_nms":(delta_h-ideal_h).length(),
     "wrong_corner_normal_ns":corners.iter().enumerate().filter(|(i,_)|*i!=o.corner).map(|(_,v)|v).sum::<f64>(),
     "com_error_m":(final_state.c-o.endpoint.c).length(),"spin_error_rad_s":(final_state.w-o.endpoint.w).length()}),
    )
}
fn public_case(
    out: &Path,
    early: Seed,
    c: Case,
    rigid: bool,
    policy: &str,
    write_trace: bool,
) -> Result<Value> {
    let start = DONOR - 4;
    let mut s = load(&early.scene(c.mu, rigid, policy), true)?;
    let r = ids(&s)?;
    let pad = bind(&s, "default::lift::pad")?.0;
    let mut rows = vec![];
    let mut checkpoints = vec![];
    let mut events = vec![];
    let mut pre = None;
    let mut guards = Vec::new();
    let mut last_speed = early.pad.velocity[2];
    let first = Seed::read(departure::ingest(engine(&s), &r));
    let cold_error = near(first.state, early.state);
    if cold_error > 1e-12 {
        return Err("unmatched public cold start".into());
    }
    for t in start..=2160 {
        let (f, u, p) = departure::ingest(engine(&s), &r);
        if t == DONOR {
            pre = Some(Seed::read((f, u, p)).state.wire());
        }
        if t == 2160 {
            break;
        }
        let speed = departure::cmd(t);
        if speed != last_speed {
            queue_link_world_velocity(&mut s, pad, [0., 0., speed], [0.; 3])?;
            last_speed = speed;
        }
        let tick = t + 1;
        s.step_exact(1)?;
        let (f, u, p) = departure::ingest(engine(&s), &r);
        let state = Seed::read((f, u, p)).state;
        let evt = last_refinement(&s).filter(|e| e.iteration == tick - start);
        let loaded = if let Some(e) = evt {
            e.interval
                .pairs
                .iter()
                .find(|p| (p.a, p.b) == (r.foot.min(r.pad), r.foot.max(r.pad)))
                .map_or(0., |p| p.normal_impulse)
        } else {
            engine(&s)
                .rigid_contact_observations()
                .iter()
                .filter(|p| (p.a, p.b) == (r.foot.min(r.pad), r.foot.max(r.pad)))
                .map(|p| p.normal_impulse)
                .sum()
        };
        if policy == "guarded"
            && let Some(g) = gz_sim_scene::primary_contact::last_box_flight(&s)
                .filter(|g| g.iteration == tick - start)
        {
            let count = g.flight.guarded.iter().filter(|f| **f).count();
            if count > 0 || evt.is_some() {
                assert!(count != g.flight.guarded.len() || loaded == 0.);
                guards.push(json!({"tick":tick,"guarded":g.flight.guarded,"durations_ns":g.flight.durations.iter().map(|d|d.as_nanos() as u64).collect::<Vec<_>>(),"normal_ns":loaded}));
            }
        }
        if let Some(e) = evt {
            let plan = last_box_timing(&s).filter(|p| p.iteration == e.iteration);
            events.push(json!({"tick":tick,"native_calls":e.interval.native_steps,"normal_ns":loaded,"timing":plan.map(|p|format!("{p:?}"))}));
        }
        if [1531, 1532, 1540, 1612, 1672, 1732, 2120, 2160].contains(&tick) {
            let (yaw, pitch, roll) = state.q.to_euler(glam::EulerRot::ZYX);
            checkpoints.push(json!({"tick":tick,"state":state.wire(),"normal_ns":loaded,"angles_rad":[roll,pitch,yaw]}));
        }
        if write_trace
            && (t < 1570 || t.is_multiple_of(8) && t <= 2120 || matches!(tick, 1613 | 2120))
        {
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":evt.map_or(1,|e|e.interval.native_steps),"solver_iterations":4,
        "selected":evt.is_some(),"weak_start":false,"secondary_start":false,
        "foot":wire(f),"upper":u.map(wire),"pad":wire(p),"normal_impulse_ns":loaded,
        "gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
    }
    if write_trace {
        let mut geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]});
        if rigid {
            geometry
                .as_object_mut()
                .unwrap()
                .remove("upper_half_extents");
            geometry["foot_com_local"] = json!(ARM.to_array());
            for row in &mut rows {
                row.as_object_mut().unwrap().remove("upper");
            }
        }
        fs::write(
            out.join(format!("traces/{policy}.json")),
            serde_json::to_vec(&json!({"frames":rows,"geometry":geometry}))?,
        )?;
    }
    let mut row = json!({"policy":policy,"single_rigid":rigid,"cold_start_difference":cold_error,"before_landing":pre,"events":events,"states":checkpoints});
    if policy == "guarded" {
        row["guarded_native"] = json!(guards);
    }
    Ok(row)
}
pub(super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut results = vec![];
    let mut donors = vec![];
    let mut public = vec![];
    for (material, mu) in [(None, 0.5), (Some(0.2), 0.2), (Some(0.8), 0.8)] {
        let (early, incoming) = donor(material)?;
        donors.push(json!({"material":material,"mu":mu,"cold_time_tick":DONOR-4,"cold_state":early.state.wire(),"incoming_tick":DONOR,"incoming":incoming.state.wire(),"pad":wire(incoming.pad)}));
        for (label, xs, ys, gap) in [
            ("measured", 1., 1., 0.),
            ("reflect-x", -1., 1., 0.),
            ("reflect-y", 1., -1., 0.),
            ("reflect-xy", -1., -1., 0.),
            ("earlier", -1., 1., -0.00011),
            ("later", 1., -1., 0.00005),
        ] {
            let case = Case {
                label,
                mu,
                material,
                xs,
                ys,
                gap,
            };
            let seed = incoming.transform(xs, ys, gap);
            let o = oracle::solve(seed.state, mu)?;
            let mut rows = vec![];
            for rigid in [false, true] {
                for g in [
                    Grid::Ordinary,
                    Grid::Uniform(8),
                    Grid::Uniform(32),
                    Grid::Uniform(128),
                    Grid::Uniform(512),
                    Grid::Prospective(8),
                    Grid::Prospective(32),
                    Grid::ExactLanding,
                    Grid::AtEvent(8),
                    Grid::AtEvent(32),
                    Grid::ApproachPasses(0),
                    Grid::ApproachPasses(1),
                    Grid::Shoulder(8),
                    Grid::Shoulder(16),
                ] {
                    rows.push(sample(seed, case, rigid, g, &o)?);
                }
                for policy in ["ordinary", "relax8", "prospective"] {
                    let write = material.is_none() && label == "measured" && !rigid;
                    let p = public_case(
                        out,
                        early.transform(xs, ys, gap),
                        case,
                        rigid,
                        policy,
                        write,
                    )?;
                    public.push(json!({"case":label,"mu":mu,"result":p}));
                }
            }
            results.push(json!({"case":case.label,"material":case.material,"mu":mu,"reflection":[case.xs,case.ys],"gap_shift_m":case.gap,"incoming":seed.state.wire(),"physical":o.wire,"native":rows}));
        }
    }
    let lanes = [
        ("ordinary", "Locked foot · ordinary"),
        ("relax8", "Locked foot · uniform eight"),
        ("prospective", "Locked foot · prospective eight"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Measured microtilt · first return and braking","pair_totals_only":true,"sampled_ticks":true,"capture_ticks":{"brake_start":1613,"demotion_gate":2120},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("donors.json"), serde_json::to_vec_pretty(&donors)?)?;
    fs::write(out.join("first.json"), serde_json::to_vec(&results)?)?;
    fs::write(out.join("public.json"), serde_json::to_vec(&public)?)?;
    Ok(())
}
