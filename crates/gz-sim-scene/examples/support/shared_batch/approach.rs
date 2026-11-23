// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the conservative shared-support handoff to a continuous exact-shape
//! translation oracle, and repeat with all four prescribed pads reversing.
use super::*;
use glam::DQuat;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::queue_link_world_velocity;

const LABELS: [&str; 4] = ["foot", "upper", "heel", "toe"];
const HALF: [[f64; 3]; 4] = [
    [0.18, 0.10, 0.06],
    [0.05, 0.06, 0.06],
    [0.121, 0.16, 0.06],
    [0.126, 0.15, 0.06],
];
fn direction(angle: f64) -> DVec3 {
    let n = -DVec3::new(1.7562801153073353, 2.200326579036272, -9.39734766315276).normalize();
    let x = (DVec3::X - n * n.x).normalize();
    x * angle.cos() + n.cross(x) * angle.sin()
}
fn moving_source(
    stations: &[usize],
    registered: bool,
    axis: DVec3,
    spacing: f64,
    speed: f64,
) -> String {
    let mut src = source(stations, registered, 0., true);
    for &i in stations {
        for model in ["rocker", "lift", "toe"] {
            let k = src.find(&format!("<model name=\"{model}_{i}\">")).unwrap();
            let a = k + src[k..].find("<pose>").unwrap() + 6;
            let b = a + src[a..].find("</pose>").unwrap();
            let mut vals: Vec<f64> = src[a..b]
                .split_whitespace()
                .map(|s| s.parse().unwrap())
                .collect();
            for j in 0..3 {
                vals[j] += axis[j] * spacing * i as f64;
            }
            src.replace_range(
                a..b,
                &vals
                    .iter()
                    .map(|v| format!("{v:.17}"))
                    .collect::<Vec<_>>()
                    .join(" "),
            );
            let v = axis * if i == 0 { speed } else { -speed };
            let a = k + src[k..].find("<linear_velocity>").unwrap() + 17;
            let b = a + src[a..].find("</linear_velocity>").unwrap();
            src.replace_range(a..b, &format!("{:.17} {:.17} {:.17}", v.x, v.y, v.z));
        }
    }
    src
}
fn ids(sim: &Simulation, i: usize) -> [(u64, BodyId); 4] {
    [
        format!("rocker_{i}::foot"),
        format!("rocker_{i}::upper"),
        format!("lift_{i}::pad"),
        format!("toe_{i}::pad"),
    ]
    .map(|name| {
        let (entity, body) = bind(sim, &format!("default::{name}"));
        (entity, body.unwrap())
    })
}
#[derive(Clone, Copy)]
struct BoxPose {
    center: DVec3,
    basis: [DVec3; 3],
    half: [f64; 3],
}
impl BoxPose {
    fn new(s: BodyState, i: usize) -> Self {
        let q = DQuat::from_array(s.orientation);
        Self {
            center: s.position.into(),
            basis: [q * DVec3::X, q * DVec3::Y, q * DVec3::Z],
            half: HALF[i],
        }
    }
    fn radius(self, n: DVec3) -> f64 {
        self.basis
            .into_iter()
            .zip(self.half)
            .map(|(b, h)| h * n.dot(b).abs())
            .sum()
    }
}
// Solve -r <= dot(n, delta + s*axis) <= r on all 15 box-separating axes.
fn overlap_window(a: BoxPose, b: BoxPose, axis: DVec3) -> Option<(f64, f64)> {
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    for raw in a.basis.into_iter().chain(b.basis).chain(
        a.basis
            .into_iter()
            .flat_map(|p| b.basis.map(|q| p.cross(q))),
    ) {
        if raw.length_squared() < 1e-22 {
            continue;
        }
        let n = raw.normalize();
        let r = a.radius(n) + b.radius(n) + 5e-14;
        let d = n.dot(b.center - a.center);
        let u = n.dot(axis);
        if u.abs() < 1e-14 {
            if d.abs() > r {
                return None;
            }
        } else {
            let (x, y) = ((-r - d) / u, (r - d) / u);
            lo = lo.max(x.min(y));
            hi = hi.min(x.max(y));
        }
        if hi < lo {
            return None;
        }
    }
    Some((lo, hi))
}
fn limit(poses: [BoxPose; 4], axis: DVec3) -> (f64, [usize; 2]) {
    let mut best = (f64::NEG_INFINITY, [0, 0]);
    for (i, a) in poses.iter().enumerate() {
        for (j, b) in poses.iter().enumerate() {
            if (i < 2 || j < 2)
                && let Some((_, hi)) = overlap_window(*a, *b, axis)
                && hi > best.0
            {
                best = (hi, [i, j]);
            }
        }
    }
    best
}
fn cross_impulse(r: &Rig, tick: u64, ids: &[[(u64, BodyId); 4]; 2]) -> f64 {
    let cross = |a, b| {
        ids[0].iter().any(|(_, p)| *p == a) && ids[1].iter().any(|(_, p)| *p == b)
            || ids[0].iter().any(|(_, p)| *p == b) && ids[1].iter().any(|(_, p)| *p == a)
    };
    if let Some(report) = last_refinement(&r.sim).filter(|x| x.iteration == tick) {
        report
            .interval
            .pairs
            .iter()
            .filter(|p| cross(p.a, p.b))
            .map(|p| p.normal_impulse)
            .sum()
    } else {
        r.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_observations()
            .into_iter()
            .filter(|p| cross(p.a, p.b))
            .map(|p| p.normal_impulse)
            .sum()
    }
}
#[derive(Clone, Copy)]
struct Trial {
    heading: f64,
    speed: f64,
    arrival: f64,
    reverse: bool,
}
fn trial(out: &Path, base: [BoxPose; 4], case: Trial, save: bool) -> Result<Value> {
    let Trial {
        heading,
        speed,
        arrival,
        reverse,
    } = case;
    let axis = direction(heading);
    let (s_limit, pair) = limit(base, axis);
    let spacing = s_limit + 2. * speed * arrival;
    let src = moving_source(&[0, 1], true, axis, spacing, speed);
    let mut batch = Rig::from_source(&[0, 1], &src)?;
    let mut singles = [0, 1]
        .map(|i| Rig::from_source(&[i], &moving_source(&[i], true, axis, spacing, speed)))
        .into_iter()
        .collect::<Result<Vec<_>>>()?;
    let links = [ids(&batch.sim, 0), ids(&batch.sim, 1)];
    let singles_links = [ids(&singles[0].sim, 0), ids(&singles[1].sim, 1)];
    let initial = batch.states();
    let deceleration = 0.31 * 9.81;
    let turn = if reverse {
        Some(
            ((arrival - (speed * speed / deceleration + 0.008) / (2. * speed)) / DT).floor() as u64
                + 1,
        )
    } else {
        None
    };
    let last = ((arrival
        + if reverse {
            2. * speed / deceleration + 0.075
        } else {
            0.028
        })
        / DT)
        .ceil() as u64;
    if save {
        fs::write(out.join("approach.sdf"), &src)?;
    }
    let mut first_handoff = None;
    let mut first_cross = None;
    let mut restored = None;
    let mut pre = [0f64; 4];
    let mut after = [0f64; 4];
    let mut max_cross = 0f64;
    let mut min_clearance = f64::INFINITY;
    let mut min_pair = [0, 0];
    let mut samples = vec![];
    for tick in 1..=last {
        if Some(tick) == turn {
            for k in 0..2 {
                let v = axis * if k == 0 { -speed } else { speed };
                for ((id, _), (other, _)) in links[k][2..].iter().zip(&singles_links[k][2..]) {
                    queue_link_world_velocity(&mut batch.sim, *id, v.to_array(), [0.; 3])?;
                    queue_link_world_velocity(&mut singles[k].sim, *other, v.to_array(), [0.; 3])?;
                }
            }
        }
        batch.sim.step_exact(1)?;
        for s in &mut singles {
            s.sim.step_exact(1)?;
        }
        let shared = batch.loads(tick).into_iter().flatten().count();
        if shared != 2 {
            first_handoff.get_or_insert(tick);
        }
        if shared == 2 && first_handoff.is_some() {
            restored.get_or_insert(tick);
        }
        let cross = cross_impulse(&batch, tick, &links);
        if cross > 1e-10 {
            first_cross.get_or_insert(tick);
        }
        max_cross = max_cross.max(cross);
        let t = tick as f64 * DT;
        let coast = turn.map_or(t, |turn| t.min((turn - 1) as f64 * DT));
        let post = t - coast;
        let brake = post.min(2. * speed / deceleration);
        let foot_travel =
            speed * (coast + brake) - 0.5 * deceleration * brake * brake - speed * (post - brake);
        let pad_travel = speed * (coast - post);
        let mut errors = [0f64; 4];
        for (k, (actual, station)) in batch.states().into_iter().zip(&singles).enumerate() {
            let s = station.states()[0];
            let expected = DVec3::from(initial[k].position)
                + axis * foot_travel * if k == 0 { 1. } else { -1. };
            errors[0] = errors[0].max(DVec3::from(s.position).distance(expected));
            for (j, e) in error(actual, s).into_iter().take(3).enumerate() {
                errors[j + 1] = errors[j + 1].max(e);
            }
        }
        let selected = if first_handoff.is_none() {
            &mut pre
        } else {
            &mut after
        };
        for i in 0..4 {
            selected[i] = selected[i].max(errors[i]);
        }
        let mut clear = f64::INFINITY;
        for (i, a) in base.iter().enumerate() {
            for (j, b) in base.iter().enumerate() {
                if (i < 2 || j < 2)
                    && let Some((_, hi)) = overlap_window(*a, *b, axis)
                {
                    let si = if i < 2 { foot_travel } else { pad_travel };
                    let sj = if j < 2 { foot_travel } else { pad_travel };
                    let d = spacing - si - sj - hi;
                    clear = clear.min(d);
                    if d < min_clearance {
                        min_clearance = d;
                        min_pair = [i, j];
                    }
                }
            }
        }
        if save {
            samples.push(json!({"tick":tick,"owners":shared,"cross_ns":cross,"physical_clearance_m":clear,"isolated_position_error_m":errors[0],"joint_position_difference_m":errors[1]}));
        }
    }
    if first_cross.is_some_and(|x| first_handoff.is_none_or(|h| h > x))
        || pre[0] > 2e-10
        || pre[1] > 2e-10
        || pre[2] > 2e-10
        || pre[3] > 2e-10
    {
        return Err(format!("nonconservative handoff or altered prior state: {heading} {speed} {first_handoff:?} {first_cross:?} {pre:?}").into());
    }
    if save {
        fs::write(
            out.join("representative.json"),
            serde_json::to_vec(&samples)?,
        )?;
    }
    Ok(
        json!({"heading_rad":heading,"speed_each_m_s":speed,"initial_separation_m":spacing,"sat_first_overlap_separation_m":s_limit,"sat_first_links":pair.map(|i|LABELS[i]),"straight_geometric_contact_s":arrival,"turn_tick":turn,"last_tick":last,"first_handoff_tick":first_handoff,"first_cross_impulse_tick":first_cross,"first_restored_tick":restored,"max_cross_impulse_ns":max_cross,"minimum_sampled_physical_clearance_m":min_clearance,"closest_links":min_pair.map(|i|LABELS[i]),"before_metrics_independent_position_then_batch_vs_isolated_position_velocity_spin":pre,"after_metrics_independent_position_then_batch_vs_isolated_position_velocity_spin":after}),
    )
}
pub(super) fn run(out: &Path) -> Result<()> {
    let r = Rig::new(&[0], true, 0., true)?;
    let b = r.sim.world().resource::<PhysicsBackend>();
    let p = ids(&r.sim, 0);
    let base = std::array::from_fn(|i| BoxPose::new(b.state(p[i].1).unwrap(), i));
    let mut cases = vec![];
    for (k, h) in [-1.3, -0.86, -0.42, 0.03, 0.41, 0.8, 1.17, 1.54]
        .into_iter()
        .enumerate()
    {
        for (j, v) in [0.12, 0.23, 0.49].into_iter().enumerate() {
            let arrival = [0.19321381976, 0.25349999975, 0.32100000025][(j + k) % 3];
            cases.push(trial(
                out,
                base,
                Trial {
                    heading: h,
                    speed: v,
                    arrival,
                    reverse: false,
                },
                k == 4 && j == 2,
            )?);
        }
        cases.push(trial(
            out,
            base,
            Trial {
                heading: h,
                speed: 0.12,
                arrival: 0.25349999975,
                reverse: true,
            },
            false,
        )?);
    }
    fs::write(out.join("field.json"), serde_json::to_vec_pretty(&cases)?)?;
    Ok(())
}

pub(super) fn cost(out: &Path) -> Result<()> {
    let r = Rig::new(&[0], true, 0., true)?;
    let b = r.sim.world().resource::<PhysicsBackend>();
    let p = ids(&r.sim, 0);
    let base = std::array::from_fn(|i| BoxPose::new(b.state(p[i].1).unwrap(), i));
    let mut rows = vec![];
    for rep in 0..11 {
        for j in 0..2 {
            let reverse = (j + rep) % 2 == 0;
            let heading = if reverse { 0.8 } else { 0.41 };
            let speed = if reverse { 0.12 } else { 0.49 };
            let t_geo = if reverse {
                0.25349999975
            } else {
                0.19321381976
            };
            let axis = direction(heading);
            let spacing = limit(base, axis).0 + 2. * speed * t_geo;
            for registered in [rep % 2 == 0, rep % 2 != 0] {
                let src = moving_source(&[0, 1], registered, axis, spacing, speed);
                let mut batch = Rig::from_source(&[0, 1], &src)?;
                let keys = [ids(&batch.sim, 0), ids(&batch.sim, 1)];
                let mut wall = vec![];
                let mut cpu = vec![];
                let mut owners = vec![];
                for tick in 1..=if reverse { 900 } else { 443 } {
                    if reverse && tick == 401 {
                        for (k, pads) in keys.iter().enumerate() {
                            let v = axis * if k == 0 { -speed } else { speed };
                            for (id, _) in &pads[2..] {
                                queue_link_world_velocity(
                                    &mut batch.sim,
                                    *id,
                                    v.to_array(),
                                    [0.; 3],
                                )?;
                            }
                        }
                    }
                    let c = thread_clock_ns();
                    let start = Instant::now();
                    batch.sim.step_exact(1)?;
                    wall.push(start.elapsed().as_nanos() as u64);
                    cpu.push(thread_clock_ns().zip(c).map(|(e, b)| e - b));
                    owners.push(batch.loads(tick).into_iter().flatten().count());
                }
                rows.push(json!({"rep":rep,"reverse":reverse,"registered":registered,"wall_ns":wall,"cpu_ns":cpu,"owners":owners}));
            }
        }
    }
    fs::write(out.join("cost.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
