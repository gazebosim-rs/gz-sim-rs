// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Actual opted-in Bevy/SDF co-bearing arcs with measured transitions.
use super::super::super::super::{
    authored::xml,
    edge_view::{self, Replay},
    transaction::Frame,
};
use super::super::transaction::{donor, frame};
use super::*;
use gz_physics_multifidelity::{EdgeImpactReceipt, TwoBearingReceipt};

fn source(i: Input, shape: Shape, f: Frame, label: &str) -> String {
    let mut s = xml(
        i,
        f,
        false,
        f.q * shape.gravity,
        if label == "ordinary" {
            "ordinary"
        } else {
            "edge"
        },
    );
    if label == "edge" {
        s = s.replace(
            "</project_edge>",
            "</project_edge><follow_frictionless_edge>true</follow_frictionless_edge>",
        );
    }
    let intrinsic = (shape.inertia - DMat3::from_diagonal(DVec3::new(0.0098, 0.0098, 0.))) * 0.5;
    let tensor = format!(
        "<ixx>{:.17}</ixx><iyy>{:.17}</iyy><izz>{:.17}</izz><ixy>{:.17}</ixy><ixz>{:.17}</ixz><iyz>{:.17}</iyz>",
        intrinsic.x_axis.x,
        intrinsic.y_axis.y,
        intrinsic.z_axis.z,
        intrinsic.x_axis.y,
        intrinsic.x_axis.z,
        intrinsic.y_axis.z
    );
    s.replace("<ixx>0.0201</ixx><iyy>0.0201</iyy><izz>0.025</izz>",&tensor).replace("A translating deck meets two vertices of a tilted sliding sole at the initial boundary.","A second vertex of a frictionless locked sole reaches a translating pad. Both vertices can support the rolling and yawing assembly until two further vertices touch.")
}
fn site(id: u8) -> usize {
    usize::from(2 * (id & 1) + ((id >> 1) & 1) + 4 * (id >> 2))
}
pub(in super::super) fn native_state(s: &Simulation, id: &Reading, f: Frame) -> State {
    let (shoe, upper, pad) = departure::ingest(engine(s), id);
    let upper = upper.unwrap();
    let c = (DVec3::from(shoe.position) + DVec3::from(upper.position)) * 0.5;
    let v = (DVec3::from(shoe.velocity) + DVec3::from(upper.velocity)) * 0.5;
    State {
        c: f.q.conjugate() * (c - DVec3::from(pad.position)) - PAD,
        v: f.q.conjugate() * (v - DVec3::from(pad.velocity)),
        motion: Motion {
            q: f.q.conjugate() * DQuat::from_array(shoe.orientation),
            w: f.q.conjugate() * DVec3::from(shoe.angular_velocity),
        },
    }
}
#[derive(Default)]
struct Balance {
    impulse: f64,
    force: f64,
    point: f64,
    linear: f64,
    angular: f64,
    pose: f64,
    velocity: f64,
    segments: usize,
    steps: u32,
}
fn check_segment(
    h: &TwoBearingReceipt,
    arc: Arc,
    tr: &mut Motion,
    absolute: &mut f64,
    keys: [usize; 2],
    frame: Frame,
    b: &mut Balance,
) -> Result<()> {
    let start = *absolute;
    let hlen = h.duration.as_secs_f64();
    let mut integral = [0.; 2];
    let init = *tr;
    let count = (hlen / 0.0000625).ceil().max(1.) as u32;
    for _ in 0..count {
        let (m, loads) = arc.advance_loaded(*tr, hlen / f64::from(count))?;
        for (j, k) in integral.iter_mut().zip(loads) {
            *j += k;
        }
        *tr = m;
    }
    *absolute += hlen;
    let post = arc.state(*absolute, *tr);
    let pre = arc.state(start, init);
    let net = arc.shape.mass * (post.v - pre.v - hlen * arc.shape.gravity);
    b.linear = b
        .linear
        .max(net.distance(DVec3::Z * h.normal_impulses.iter().sum::<f64>()));
    let moment = |m: Motion| m.q * (arc.shape.inertia * (m.q.conjugate() * m.w));
    b.angular = b.angular.max(
        (frame.q * (moment(*tr) - moment(init))).distance(DVec3::from(h.angular_impulse_about_com)),
    );
    let first = arc.load(init)?;
    let last = arc.load(*tr)?;
    let pad_v = frame.q * DVec3::new(0.17, -0.09, 0.22);
    for k in 0..2 {
        let id = site(h.sole_vertices[k]);
        let idx = keys
            .iter()
            .position(|j| *j == id)
            .ok_or("incorrect live bearing vertex")?;
        b.impulse = b.impulse.max((h.normal_impulses[k] - integral[idx]).abs());
        for load in [first[idx], last[idx]] {
            let bounds = h.normal_force_bounds[k];
            b.force = b
                .force
                .max((bounds[0] - load).max(load - bounds[1]).max(0.));
            if bounds[0] <= 0. {
                return Err("unqualified positive reaction".into());
            }
        }
        let initial = frame.shift + pad_v * start + frame.q * arc.shape.point(id, pre).0;
        let end = frame.shift + pad_v * (*absolute) + frame.q * arc.shape.point(id, post).0;
        b.point = b
            .point
            .max(initial.distance(h.start_points[k].into()))
            .max(end.distance(h.end_points[k].into()));
    }
    b.segments += 1;
    b.steps += h.integration_steps;
    Ok(())
}
fn evaluate(i: Input, shape: Shape, e: Arrival, j: Jump, f: Frame) -> Result<Value> {
    let source = source(i, shape, f, "edge");
    let mut sim = load(&source, true)?;
    let ids = ids(&sim)?;
    let path = Arc {
        shape,
        start: j.after,
        sites: j.new_sites,
    };
    let mut bal = Balance::default();
    let mut t = 0.;
    let mut motion = j.after.motion;
    let mut projected = 0.;
    let mut boundary = None;
    let physical = if matches!(j.new_sites, Sites::Two(_)) {
        Some(path.next(0.000125, 0.35)?)
    } else {
        None
    };
    let max = physical.map_or(1, |end| (end.time / DT).ceil() as u64 + 3);
    for tick in 1..=max {
        sim.step_exact(1)?;
        let rec = last_refinement(&sim).filter(|r| r.iteration == tick);
        if tick == 1 {
            let [EdgeImpactReceipt::Recontact(h)] = rec
                .ok_or("first impact missing")?
                .interval
                .edge_impacts
                .as_slice()
            else {
                return Err("different active return in live SDF".into());
            };
            for (k, idx) in [h.stationary_site, 1 - h.stationary_site]
                .into_iter()
                .enumerate()
            {
                let actual = DVec3::from(h.sites[idx].impulse_on_foot).dot(f.q * DVec3::Z);
                projected += actual;
                if (actual - j.normal[k]).abs() > 2e-10 {
                    return Err(format!(
                        "assembled recontact mismatch {actual:e} vs {:e}",
                        j.normal[k]
                    )
                    .into());
                }
            }
        }
        let segments = rec.map_or(&[][..], |r| r.interval.two_bearing_edges.as_slice());
        let Sites::Two(keys) = j.new_sites else {
            if !segments.is_empty() {
                return Err("single-site transfer entered two-bearing mode".into());
            }
            break;
        };
        for h in segments {
            let absolute = (tick - 1) as f64 * DT + h.start.as_secs_f64();
            if (absolute - t).abs() > 1e-12 {
                return Err("bearing resumed after an unmanaged native interval".into());
            }
            check_segment(h, path, &mut motion, &mut t, keys, f, &mut bal)?;
        }
        if (t - tick as f64 * DT).abs() > 1e-12 {
            boundary = Some(tick);
            break;
        }
        let s = native_state(&sim, &ids, f);
        let target = path.state(t, motion);
        let dq = s.motion.q - target.motion.q;
        bal.pose = bal
            .pose
            .max(s.c.distance(target.c))
            .max(dq.length().min((s.motion.q + target.motion.q).length()));
        bal.velocity = bal
            .velocity
            .max(s.motion.w.distance(target.motion.w))
            .max(s.v.distance(target.v));
    }
    if let Some(limit) = physical
        && (bal.segments == 0
            || boundary.is_none()
            || t >= limit.time + 1e-11
            || (limit.time - t) > 0.0025)
    {
        return Err(format!("unexpected smooth handoff at {t:.9} vs physical {:.9}: {} intervals, stop {boundary:?}",limit.time,bal.segments).into());
    }
    let residual = [
        bal.impulse,
        bal.point,
        bal.linear,
        bal.angular,
        bal.pose,
        bal.velocity,
        bal.force,
    ];
    if residual.into_iter().any(|v| v > 3e-9 || !v.is_finite()) {
        return Err(format!("native co-bearing residuals {residual:?}").into());
    }
    Ok(
        json!({"second_impact_time_s":e.time,"jump_ns":projected,"smooth_seconds":t,"physical_end_seconds":physical.map(|q|q.time),"physical_limit":physical.map(|q|kind(q.end)),"first_ordinary_tick":boundary,"segments":bal.segments,"quadrature_steps":bal.steps,"max_integral_error_ns":bal.impulse,"max_site_error_m":bal.point,"max_linear_residual_ns":bal.linear,"max_angular_residual_nms":bal.angular,"max_state_error":bal.pose,"max_twist_error":bal.velocity,"max_force_envelope_violation_n":bal.force}),
    )
}
fn replay(out: &Path, i: Input, a: Arc, j: Jump) -> Result<()> {
    edge_view::record(
        out,
        Replay {
            title: "An authored foot rolls and yaws on two bearing vertices",
            refinement_name: "Impact projection only",
            registered_name: "Projection and sustained bearing",
            capture_name: "first_native_handoff",
            impact_tick: 1,
            capture_tick: 99,
            last_tick: 105,
            sdf: "two-bearing.sdf",
        },
        |label| source(i, a.shape, frame(false), label),
        |hit, loads| {
            let EdgeImpactReceipt::Recontact(h) = hit else {
                return Err("unexpected replay edge event".into());
            };
            let nr =
                [h.stationary_site, 1 - h.stationary_site].map(|i| h.sites[i].impulse_on_foot[2]);
            loads[j.old] += nr[0];
            loads[j.new] += nr[1];
            Ok((
                h.sites[0].elapsed,
                json!({"old_vertex":j.old,"new_vertex":j.new,"j_old_new_ns":nr}),
            ))
        },
    )
}
fn cost(out: &Path, i: Input, a: Arc) -> Result<()> {
    let mut timings = vec![];
    let modes = ["ordinary", "projection", "edge"];
    for repetition in 0..9 {
        for offset in 0..3 {
            let mode = modes[(repetition + offset) % 3];
            let mut s = load(&source(i, a.shape, frame(false), mode), true)?;
            let mut all = vec![];
            let mut ticks = 0;
            for _ in 0..100 {
                let start = std::time::Instant::now();
                s.step_exact(1)?;
                all.push(start.elapsed().as_nanos() as u64);
                if let Some(r) = last_refinement(&s).filter(|e| e.iteration == all.len() as u64) {
                    ticks += r.interval.two_bearing_edges.len();
                }
            }
            timings.push(json!({"repetition":repetition,"mode":mode,"step_cost_ns":all,"two_bearing_segments":ticks}));
        }
    }
    fs::write(out.join("cost.json"), serde_json::to_vec(&timings)?)?;
    Ok(())
}
pub(in super::super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut records = vec![];
    let mut demo = None;
    for active in [0, 1] {
        for pitch in [-0.18, 0.18] {
            for opening in [0.09, 0.3, 0.5] {
                for rate in [-3., -0.6, 0.6, 3.] {
                    for yaw in [0., 0.67] {
                        let (i, a, _) = fixture(Case {
                            active,
                            pitch,
                            yaw,
                            opening,
                            pitch_rate: rate,
                        });
                        if solve(i.edge).is_err() {
                            continue;
                        }
                        let e = a.next(0.0000625, 0.4)?;
                        let End::Contact(new) = e.end else {
                            continue;
                        };
                        let input = donor(i, e.state);
                        for variant in ["natural", "triaxial"] {
                            let mut shape = a.shape;
                            if variant == "triaxial" {
                                shape.inertia = DMat3::from_cols(
                                    DVec3::new(0.034, 0.004, -0.003),
                                    DVec3::new(0.004, 0.051, 0.006),
                                    DVec3::new(-0.003, 0.006, 0.055),
                                );
                            }
                            let j = shape.recollide(e.state, a.sites.first(), new)?;
                            for rotated in [false, true] {
                                let r = evaluate(input, shape, e, j, frame(rotated))?;
                                if demo.is_none()
                                    && variant == "natural"
                                    && yaw > 0.
                                    && !rotated
                                    && r["physical_end_seconds"]
                                        .as_f64()
                                        .is_some_and(|t| (0.0495..0.05).contains(&t))
                                {
                                    demo = Some((input, a, j));
                                }
                                records.push(json!({"active":active,"pitch":pitch,"opening":opening,"rate":rate,"yaw":yaw,"variant":variant,"inclined":rotated,"both_bear":matches!(j.new_sites,Sites::Two(_)),"result":r}));
                            }
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("native.json"), serde_json::to_vec(&records)?)?;
    let (i, a, j) = demo.ok_or("missing representative co-bearing")?;
    replay(out, i, a, j)?;
    cost(out, i, a)
}
