// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Signed solved point impulses versus full spatial momentum of a locked two-link foot.
use super::*;
use gz_physics_multifidelity::RigidContactPatch;
use gz_sim_scene::{
    primary_contact::{set_enabled, set_refined_observer},
    queue_link_world_wrench,
};
use std::{
    collections::BTreeSet,
    path::Path,
    sync::{Arc, Mutex},
};
type State = (BodyState, Option<BodyState>, BodyState);
#[derive(Clone, Copy)]
struct Momentum {
    c: DVec3,
    p: DVec3,
    h: DVec3,
}
fn momentum((f, u, _): State) -> Momentum {
    let fp = DVec3::from(f.position);
    let fq = DQuat::from_array(f.orientation);
    let fv = DVec3::from(f.velocity);
    let fw = DVec3::from(f.angular_velocity);
    let Some(u) = u else {
        return Momentum {
            c: fp + fq * DVec3::Z * 0.07,
            p: fv * 2.,
            h: fw * 0.05,
        };
    };
    let up = DVec3::from(u.position);
    let uv = DVec3::from(u.velocity);
    let c = (fp + up) * 0.5;
    let link_spin =
        |q: DQuat, w: DVec3| q * ((q.conjugate() * w) * DVec3::new(0.0201, 0.0201, 0.025));
    let orbit = (fp - c).cross(fv) + (up - c).cross(uv);
    Momentum {
        c,
        p: fv + uv,
        h: orbit
            + link_spin(fq, fw)
            + link_spin(
                DQuat::from_array(u.orientation),
                DVec3::from(u.angular_velocity),
            ),
    }
}
struct Frame {
    tick: u64,
    elapsed: std::time::Duration,
    dt: f64,
    states: Vec<State>,
    patches: Vec<RigidContactPatch>,
}
type Buffer = Arc<Mutex<Vec<Frame>>>;
fn instrument(sim: &mut Simulation, ids: &[Reading], configured: bool) -> Result<Buffer> {
    let buf = Arc::new(Mutex::new(Vec::new()));
    if configured {
        let sink = Arc::clone(&buf);
        let focus = ids.to_vec();
        set_refined_observer(sim, move |t, sub| {
            let states = focus
                .iter()
                .map(|r| {
                    (
                        sub.state(r.foot).unwrap(),
                        r.upper.and_then(|id| sub.state(id)),
                        sub.state(r.pad).unwrap(),
                    )
                })
                .collect();
            sink.lock().unwrap().push(Frame {
                tick: t.iteration,
                elapsed: sub.elapsed,
                dt: sub.duration.as_secs_f64(),
                states,
                patches: sub.contact_patches(),
            });
        })?;
    }
    Ok(buf)
}
fn actual_frames(sim: &Simulation, ids: &[Reading], tick: u64, buf: &Buffer) -> Result<Vec<Frame>> {
    let mut frames = std::mem::take(&mut *buf.lock().unwrap());
    if let Some(r) = last_refinement(sim).filter(|r| r.iteration == tick) {
        if frames.len() != r.interval.native_steps as usize
            || frames
                .last()
                .is_none_or(|p| p.elapsed != sim.step_duration())
            || frames.iter().any(|p| p.tick != tick)
        {
            return Err(format!(
                "missing observed substep at {tick}: {} / {}",
                frames.len(),
                r.interval.native_steps
            )
            .into());
        }
    } else {
        if !frames.is_empty() {
            return Err("unexpected observations on ordinary tick".into());
        }
        let s = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        frames.push(Frame {
            tick,
            elapsed: sim.step_duration(),
            dt: sim.step_duration().as_secs_f64(),
            states: ids.iter().map(|r| departure::ingest(s, r)).collect(),
            patches: s.rigid_contact_patches(),
        });
    }
    Ok(frames)
}
#[derive(Default, Clone, Copy)]
struct Delivered {
    jn: DVec3,
    jt: DVec3,
    kn: DVec3,
    kt: DVec3,
    scalar: f64,
    peak_points: usize,
}
impl Delivered {
    fn add(&mut self, q: Self) {
        self.jn += q.jn;
        self.jt += q.jt;
        self.kn += q.kn;
        self.kt += q.kt;
        self.scalar += q.scalar;
        self.peak_points = self.peak_points.max(q.peak_points);
    }
    fn wire(self) -> Value {
        json!({"normal_linear_ns":self.jn.to_array(),"tangent_linear_ns":self.jt.to_array(),"normal_angular_nms":self.kn.to_array(),
    "tangent_angular_nms":self.kt.to_array(),"normal_scalar_ns":self.scalar,"max_positive_native_points":self.peak_points})
    }
}
fn applied(
    ids: Reading,
    before: State,
    patches: &[RigidContactPatch],
    frictionless: bool,
) -> Result<Delivered> {
    let mom = momentum(before);
    let mut out = Delivered::default();
    for patch in patches {
        let member = |b| b == ids.foot || ids.upper == Some(b);
        let left = member(patch.a);
        let right = member(patch.b);
        if left == right {
            continue;
        }
        let (body, other) = if left {
            (patch.a, patch.b)
        } else {
            (patch.b, patch.a)
        };
        if other != ids.pad {
            return Err("unexpected external contact on audited assembly".into());
        }
        let origin = if body == ids.foot {
            if before.1.is_none() {
                mom.c
            } else {
                before.0.position.into()
            }
        } else {
            DVec3::from(before.1.unwrap().position)
        };
        let sign = if left { 1. } else { -1. };
        let normal = DVec3::from(patch.normal_on_a) * sign;
        for p in &patch.points {
            let jn = normal * p.normal_impulse;
            let jt = match p.tangent_impulse_on_a {
                Some(t) => DVec3::from(t) * sign,
                None if frictionless => DVec3::ZERO,
                None => return Err("applied grouped friction impulse unavailable".into()),
            };
            // The audited side is dynamic. Rapier's frozen solver lever is
            // world-oriented and based at this link's COM. Shift it to the
            // incoming assembly COM before computing the delivered moment.
            let r = origin - mom.c + DVec3::from(if left { p.lever_a } else { p.lever_b });
            out.scalar += p.normal_impulse;
            out.jn += jn;
            out.jt += jt;
            out.kn += r.cross(jn);
            out.kt += r.cross(jt);
            out.peak_points += usize::from(p.normal_impulse > 0.);
        }
    }
    Ok(out)
}
#[derive(Clone)]
struct Ledger {
    initial: Momentum,
    pre: State,
    total: Delivered,
    gravity: DVec3,
    peak_linear: f64,
    peak_angular: f64,
    elapsed: f64,
}
impl Ledger {
    fn new(s: State) -> Self {
        Self {
            initial: momentum(s),
            pre: s,
            total: Default::default(),
            gravity: DVec3::ZERO,
            peak_linear: 0.,
            peak_angular: 0.,
            elapsed: 0.,
        }
    }
    fn advance(
        &mut self,
        ids: Reading,
        new: State,
        dt: f64,
        patches: &[RigidContactPatch],
        gravity: DVec3,
        frictionless: bool,
    ) -> Result<Delivered> {
        let q = applied(ids, self.pre, patches, frictionless)?;
        let before = momentum(self.pre);
        let after = momentum(new);
        let g = gravity * (2. * dt);
        self.peak_linear = self
            .peak_linear
            .max((after.p - before.p - g - q.jn - q.jt).length());
        self.peak_angular = self
            .peak_angular
            .max((after.h - before.h - q.kn - q.kt).length());
        self.gravity += g;
        self.total.add(q);
        self.pre = new;
        self.elapsed += dt;
        Ok(q)
    }
    fn sample(&self) -> Value {
        let m = momentum(self.pre);
        let h = m.h - self.initial.h;
        let p = m.p - self.initial.p;
        let q = self.total;
        let quat = DQuat::from_array(self.pre.0.orientation);
        let (yaw, pitch, roll) = quat.to_euler(glam::EulerRot::ZYX);
        json!({"com_m":m.c.to_array(),"h_nms":m.h.to_array(),"delta_h_nms":h.to_array(),"p_ns":m.p.to_array(),"roll_pitch_yaw_rad":[roll,pitch,yaw],
        "integrated":q.wire(),"external_gravity_ns":self.gravity.to_array(),"linear_residual_ns":(p-self.gravity-q.jn-q.jt).to_array(),
        "angular_residual_nms":(h-q.kn-q.kt).to_array(),"max_native_linear_residual_ns":self.peak_linear,"max_native_angular_residual_nms":self.peak_angular})
    }
}
fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn sole_gaps((f, _, pad): State) -> [f64; 4] {
    let q = DQuat::from_array(f.orientation);
    let p = DVec3::from(f.position);
    let deck = DVec3::from(pad.position);
    let qdeck = DQuat::from_array(pad.orientation);
    [(-0.18, -0.1), (-0.18, 0.1), (0.18, -0.1), (0.18, 0.1)].map(|(x, y)| {
        let bottom = p + q * DVec3::new(x, y, -0.06);
        (qdeck.conjugate() * (bottom - deck)).z - 0.06
    })
}
fn mix_run(out: &Path, mu: Option<f64>, policy: &str) -> Result<Value> {
    let scene = departure::source(mu, policy);
    let mut sim = load_world_with_runtime(
        &scene,
        "angular.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions::default(),
    )?;
    let mut pads = vec![];
    let ids = [("rocker", "lift"), ("edge_rocker", "edge_lift")].map(|(m, p)| {
        let (link, pad) = bind(&sim, &format!("default::{p}::pad")).unwrap();
        pads.push(link);
        Reading {
            foot: bind(&sim, &format!("default::{m}::foot")).unwrap().1,
            upper: Some(bind(&sim, &format!("default::{m}::upper")).unwrap().1),
            pad,
        }
    });
    let (witness, _) = bind(&sim, "default::witness::body")?;
    let selected = policy != "ordinary";
    if selected {
        set_enabled(&mut sim, false)?;
    }
    let buf = instrument(&mut sim, &ids, selected)?;
    let mut ledgers = vec![];
    let mut start_state = vec![];
    let mut checkpoints = vec![];
    let mut detailed = vec![];
    let mut view = vec![];
    let mut first: [Option<u64>; 2] = [None, None];
    let mut first_impact = vec![];
    let mut previous = 0.;
    let mut max_scalar_gap = 0f64;
    for boundary in 0..2300 {
        if boundary == 1472 {
            for r in &ids {
                let s = departure::ingest(native(&sim), r);
                ledgers.push(Ledger::new(s));
                start_state.push(json!([wire(s.0), s.1.map(wire), wire(s.2)]));
            }
            if selected {
                set_enabled(&mut sim, true)?;
            }
        }
        let tick = boundary + 1;
        queue_link_world_wrench(
            &mut sim,
            witness,
            [if tick % 4 < 2 { 24. } else { -16. }, 0., 0.],
            [0.; 3],
        )?;
        if boundary == 1400 {
            sim.set_paused(true);
            assert!(sim.tick()?.dt.is_zero());
        }
        let speed = departure::cmd(boundary);
        if speed != previous {
            for p in &pads {
                queue_link_world_velocity(&mut sim, *p, [0., 0., speed], [0.; 3])?;
            }
            previous = speed;
        }
        sim.step_exact(1)?;
        if boundary < 1472 {
            continue;
        }
        let frames = actual_frames(&sim, &ids, tick, &buf)?;
        let mut sums = [Delivered::default(); 2];
        let mut first_native = [Vec::new(), Vec::new()];
        for frame in &frames {
            for (i, r) in ids.iter().enumerate() {
                let prior = ledgers[i].pre;
                let part = ledgers[i].advance(
                    *r,
                    frame.states[i],
                    frame.dt,
                    &frame.patches,
                    -9.81 * DVec3::Z,
                    mu == Some(0.),
                )?;
                sums[i].add(part);
                if first[i].is_none()
                    && frame.patches.iter().any(|p| p.a == r.foot || p.b == r.foot)
                {
                    first_native[i].push(json!({"elapsed_ns":frame.elapsed.as_nanos() as u64,
                        "incoming_corner_gaps_m":sole_gaps(prior),"incoming": [wire(prior.0),wire(prior.1.unwrap()),wire(prior.2)],
                        "after":ledgers[i].sample(),"delivered":part.wire()}));
                }
            }
        }
        let req = last_refinement(&sim).filter(|r| r.iteration == tick);
        if req.is_some() {
            for (i, r) in ids.iter().enumerate() {
                if frames.last().unwrap().states[i] != departure::ingest(native(&sim), r) {
                    return Err(format!(
                        "observed endpoint does not match committed native state on tick {tick}"
                    )
                    .into());
                }
            }
        }
        for i in 0..2 {
            let q = sums[i];
            let r = ids[i];
            if let Some(e) = req {
                let pair = e
                    .interval
                    .pairs
                    .iter()
                    .find(|p| (p.a, p.b) == (r.foot.min(r.pad), r.foot.max(r.pad)));
                max_scalar_gap =
                    max_scalar_gap.max((pair.map_or(0., |p| p.normal_impulse) - q.scalar).abs());
            }
            if q.scalar > 0. && first[i].is_none() {
                first[i] = Some(tick);
                first_impact.push(
                    json!({"rig":i,"tick":tick,"delivered":q.wire(),"after":ledgers[i].sample(),"native":first_native[i]}),
                );
            }
            if [
                1531, 1532, 1540, 1612, 1672, 1732, 1800, 1920, 2120, 2160, 2300,
            ]
            .contains(&tick)
            {
                checkpoints.push(json!({"rig":i,"tick":tick,"sample":ledgers[i].sample()}));
            }
        }
        if mu.is_none() {
            let tick_q = sums[0];
            let s = ledgers[0].pre;
            detailed.push(json!({"tick":tick,"time_ms":tick as f64*0.5,"native_calls":frames.len(),"delivered":tick_q.wire(),"after":ledgers[0].sample()}));
            if (1522..=1570).contains(&tick) {
                view.push(json!({"tick":tick,"time_us":tick*500,"native_steps":frames.len(),"solver_iterations":4,
                    "selected":req.is_some_and(|e|e.requested_pairs.iter().any(|p|p.foot==ids[0].foot)),"weak_start":false,"secondary_start":false,
                    "foot":wire(s.0),"upper":wire(s.1.unwrap()),"pad":wire(s.2),"normal_impulse_ns":tick_q.scalar,
                    "gap_m":s.0.position[2]-s.2.position[2]-0.06-q_up(s.0).abs().dot(DVec3::new(0.18,0.1,0.06))}));
            }
        }
    }
    if max_scalar_gap > 1e-12 {
        return Err("native scalar point sum and integrated receipt disagree".into());
    }
    if !detailed.is_empty() {
        fs::write(
            out.join(format!("ledgers/{policy}.json")),
            serde_json::to_vec(&detailed)?,
        )?;
        let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]});
        fs::write(
            out.join(format!("traces/{policy}.json")),
            serde_json::to_vec(&json!({"frames":view,"geometry":geometry}))?,
        )?;
    }
    Ok(
        json!({"mu":mu,"policy":policy,"initial":start_state,"first_loaded":first_impact,"checkpoints":checkpoints,"max_scalar_point_sum_disagreement_ns":max_scalar_gap}),
    )
}

fn oracle(tilted: bool, policy: &str, rigid: bool) -> Result<Value> {
    // Choose the link height so the unique lowest corner or level face touches
    // at exactly 2.1875 ms. After this first tick the other inclined corners
    // must still clear the deck, leaving one independent normal constraint.
    let (roll, pitch) = if tilted { (0.14, -0.16) } else { (0., 0.) };
    let q = DQuat::from_rotation_y(pitch) * DQuat::from_rotation_x(roll);
    let mut corners = vec![];
    for x in [-0.18, 0.18] {
        for y in [-0.1, 0.1] {
            corners.push(DVec3::new(x, y, -0.06));
        }
    }
    corners.sort_by(|a, b| (q * *a).z.total_cmp(&(q * *b).z));
    let z = -(q * corners[0]).z + 1.4 * 0.0021875;
    let base = xml(false, 0., rigid, policy);
    let scene = base
        .replace("<gravity>0 0 -9.81", "<gravity>0 0 0")
        .replace("<linear_velocity>0 0 -0.55", "<linear_velocity>0 0 -1.1")
        .replace(
            "<pose>0 0 0.074 0 0 0",
            &format!("<pose>0 0 {z:.17} {roll} {pitch} 0"),
        );
    let mut sim = load_world_with_runtime(
        &scene,
        "angular-oracle.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: true,
            ..Default::default()
        },
    )?;
    let ids = Reading {
        foot: bind(&sim, "default::rocker::foot")?.1,
        upper: bind(&sim, "default::rocker::upper").ok().map(|p| p.1),
        pad: bind(&sim, "default::lift::pad")?.1,
    };
    let buf = instrument(&mut sim, &[ids], policy != "ordinary")?;
    let mut ledger = Ledger::new(departure::ingest(native(&sim), &ids));
    let mut positive = BTreeSet::new();
    for tick in 1..=5 {
        sim.step_exact(1)?;
        let frames = actual_frames(&sim, &[ids], tick, &buf)?;
        for frame in frames {
            for p in &frame.patches {
                for pt in p.points.iter().filter(|p| p.normal_impulse > 0.) {
                    let arm = if p.a == ids.foot {
                        DVec3::from(pt.lever_a)
                    } else if p.b == ids.foot {
                        DVec3::from(pt.lever_b)
                    } else {
                        continue;
                    };
                    let local = q.conjugate() * arm;
                    positive.insert((local.x > 0., local.y > 0.));
                }
            }
            ledger.advance(
                ids,
                frame.states[0],
                frame.dt,
                &frame.patches,
                DVec3::ZERO,
                true,
            )?;
        }
    }
    let lever = q * (corners[0] - DVec3::Z * 0.07);
    let j = if tilted {
        1.4 / (0.5 + lever.cross(DVec3::Z).length_squared() / 0.05)
    } else {
        2.8
    };
    let k = if tilted {
        lever.cross(DVec3::Z * j)
    } else {
        DVec3::ZERO
    };
    let (f, _, p) = ledger.pre;
    let fq = DQuat::from_array(f.orientation);
    let secondary_gap = if tilted {
        corners[1..]
            .iter()
            .map(|x| f.position[2] + (fq * *x).z - p.position[2] - 0.06)
            .fold(f64::INFINITY, f64::min)
    } else {
        0.
    };
    if tilted && (positive.len() != 1 || secondary_gap < 0.015) {
        return Err("tilted physical oracle admitted another corner".into());
    }
    let measured = momentum(ledger.pre).h - ledger.initial.h;
    Ok(
        json!({"tilted":tilted,"rigid":rigid,"policy":policy,"expected_normal_ns":j,"expected_angular_nms":k.to_array(),
        "actual_normal_ns":ledger.total.scalar,"actual_angular_nms":measured.to_array(),"signed_point_moment_nms":(ledger.total.kn+ledger.total.kt).to_array(),
        "moment_error_nms":(measured-k).length(),"normal_error_ns":(ledger.total.scalar-j).abs(),"other_corner_gap_m":secondary_gap,"positive_quadrants":positive.len(),
        "ledger":ledger.sample()}),
    )
}
pub(super) fn run_all(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("ledgers"))?;
    fs::create_dir_all(out.join("traces"))?;
    let mut study = vec![];
    for mu in [None, Some(0.2), Some(0.8)] {
        for policy in POLICIES {
            study.push(mix_run(out, mu, policy)?);
        }
    }
    let mut controls = vec![];
    for tilted in [false, true] {
        for rigid in [false, true] {
            for p in ["ordinary", "relax8", "relax32"] {
                controls.push(oracle(tilted, p, rigid)?);
            }
        }
    }
    let lanes = [
        ("ordinary", "Ordinary solve"),
        ("relax8", "8 relaxation passes"),
        ("refresh8", "8 passes + fresh geometry"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Matched lift return · first angular impulse","pair_totals_only":true,"lanes":lanes}),
        )?,
    )?;
    fs::write(
        out.join("angular.json"),
        serde_json::to_vec(&json!({"mixed":study,"closed_form":controls}))?,
    )?;
    Ok(())
}
