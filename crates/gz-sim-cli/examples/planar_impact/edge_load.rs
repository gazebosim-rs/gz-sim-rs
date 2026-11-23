// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frictional reaction after impact where a fixed deck corner bears on a rotating sole.
use super::*;
use gz_physics::PlanarEdgeContact;
use gz_physics_multifidelity::BoxContactActions;
type State = [f64; 8]; // θ, s, tangent COM speed, ω, Jx, Jz, lost work, angular impulse of gravity.
const TICKS: usize = 40;
const CLOCKS: [usize; 4] = [0, 1, 9, 39];
const LABELS: [&str; 3] = ["two", "eight", "sixty_four"];
#[derive(Clone, Copy)]
struct Ref {
    state: State,
    com: DVec3,
    velocity: DVec3,
    angular_momentum: f64,
    energy: f64,
}
fn state(p: PlanarImpactResponse, y: State) -> Ref {
    let [angle, s, u, w, _, _, _, _] = y;
    let (sin, cos) = angle.sin_cos();
    let e = DVec3::new(cos, 0., -sin);
    let n = DVec3::new(sin, 0., cos);
    let com = DVec3::X * p.center_of_pressure - s * e + HEIGHT * n;
    let v = u * e + s * w * n;
    let lever = com - DVec3::X * p.center_of_pressure;
    let momentum = 2. * v;
    Ref {
        state: y,
        com,
        velocity: v,
        angular_momentum: lever.cross(momentum).y + 0.05 * w,
        energy: v.length_squared() + 0.025 * w * w,
    }
}
fn deriv(c: Case, p: PlanarImpactResponse, y: State, gravity: DVec3) -> Result<Option<State>> {
    let [angle, s, u, w, _, _, _, _] = y;
    if s.abs() >= FOOT[0] - 1e-10 || angle.abs() > 0.55 || angle * p.pitch_rate < -1e-14 {
        return Ok(None);
    }
    // Solve the normal acceleration row of the constrained Newton-Euler
    // equations directly. The impulse integrator uses angular momentum and
    // energy in an inertial frame and does not integrate these equations.
    let (sin, cos) = angle.sin_cos();
    let ae = gravity.x * cos - gravity.z * sin;
    let an = gravity.x * sin + gravity.z * cos;
    let slip = u - HEIGHT * w;
    let (t, n, travel, loss) = if p.sticking {
        // Enforce both accelerations of a stationary material point in the
        // deck frame. This reference integrates center and pitch momentum;
        // the pivot integrator instead uses torque about the corner.
        let a = 0.5 + HEIGHT * HEIGHT / 0.05;
        let b = HEIGHT * s / 0.05;
        let d = 0.5 + s * s / 0.05;
        let bt = ae - s * w * w;
        let bn = an + 2. * w * u - HEIGHT * w * w;
        let det = a * d - b * b;
        let t = (b * bn - d * bt) / det;
        let n = (b * bt - a * bn) / det;
        if n <= 0. || t.abs() > c.mu * n {
            return Ok(None);
        }
        (t, n, 0., 0.)
    } else {
        let sign = slip.signum();
        let effective = 0.5 + (s * s - sign * c.mu * s * HEIGHT) / 0.05;
        if effective <= 0. || 0.5 + s * s / 0.05 <= (c.mu * s * HEIGHT / 0.05).abs() {
            return Ok(None);
        }
        let n = (HEIGHT * w * w - 2. * w * u - an) / effective;
        if n <= 0. || slip * (p.com_velocity[0] - HEIGHT * p.pitch_rate) <= 0. {
            return Ok(None);
        }
        let t = -sign * c.mu * n;
        (t, n, -slip, -t * slip)
    };
    Ok(Some([
        w,
        travel,
        ae + 0.5 * t - s * w * w,
        (-s * n - HEIGHT * t) / 0.05,
        t * cos + n * sin,
        -t * sin + n * cos,
        loss,
        2. * ((s * sin + HEIGHT * cos) * gravity.x - (-s * cos + HEIGHT * sin) * gravity.z),
    ]))
}
fn plus(y: State, k: State, h: f64) -> State {
    std::array::from_fn(|i| y[i] + h * k[i])
}
fn oracle(
    c: Case,
    p: PlanarImpactResponse,
    steps: u32,
    gravity: DVec3,
) -> Result<(Vec<Ref>, [f64; 3])> {
    let mut y = [
        0.,
        p.center_of_pressure,
        p.com_velocity[0],
        p.pitch_rate,
        0.,
        0.,
        0.,
        0.,
    ];
    let initial = state(p, y);
    let mut output = vec![];
    let mut err = [0.0_f64; 3];
    let h = 0.0005 / f64::from(steps);
    for tick in 1..=TICKS {
        for _ in 0..steps {
            let Some(a) = deriv(c, p, y, gravity)? else {
                return Ok((output, err));
            };
            let Some(b) = deriv(c, p, plus(y, a, 0.5 * h), gravity)? else {
                return Ok((output, err));
            };
            let Some(d) = deriv(c, p, plus(y, b, 0.5 * h), gravity)? else {
                return Ok((output, err));
            };
            let Some(e) = deriv(c, p, plus(y, d, h), gravity)? else {
                return Ok((output, err));
            };
            let next =
                std::array::from_fn(|i| y[i] + h / 6. * (a[i] + 2. * b[i] + 2. * d[i] + e[i]));
            if deriv(c, p, next, gravity)?.is_none() {
                return Ok((output, err));
            }
            y = next;
        }
        let r = state(p, y);
        err[0] = err[0].max((r.angular_momentum - initial.angular_momentum - y[7]).abs());
        err[1] = err[1]
            .max((r.energy + y[6] - initial.energy - 2. * gravity.dot(r.com - initial.com)).abs());
        err[2] = err[2].max(
            (2. * (r.velocity - initial.velocity - gravity * (0.0005 * tick as f64)))
                .distance(DVec3::new(y[4], 0., y[5])),
        );
        output.push(r);
    }
    Ok((output, err))
}
fn measured(
    c: Case,
    p: PlanarImpactResponse,
    reference: &[Ref],
    n: u32,
    show: bool,
    gravity: DVec3,
) -> Result<(Vec<Value>, Vec<Value>)> {
    let Rig {
        mut native,
        foot,
        upper,
        pad,
        velocity: incoming,
    } = Rig::build(c, Model::Locked, Duration::ZERO, gravity)?;
    let mut rows = vec![];
    let mut frames = vec![];
    let mut native_cumulative = DVec3::ZERO;
    let mut after_initial = DVec3::ZERO;
    let deck_v = DVec3::new(0.31, 0., 0.25);
    let mut first_phase = None;
    let mut active = n == 1;
    let mut followed = 0;
    let mut total = 0;
    for tick in 0..=reference.len() {
        let mut impulse = 0.;
        let mut calls = 0;
        if tick > 0 {
            let pairs = [(foot, pad)];
            let receipt = if tick == 1 || active {
                native
                    .advance_box_contacts(
                        DT,
                        if n == 1 && tick == 1 { 2 } else { n },
                        4.into(),
                        BoxContactActions {
                            planar_impacts: if tick == 1 { &pairs } else { &[] },
                            sustained_edges: if active { &pairs } else { &[] },
                            ..Default::default()
                        },
                        None,
                    )?
                    .interval
            } else {
                native.step_contact_interval(DT, n, 4)?
            };
            calls = receipt.native_steps;
            total += calls;
            if active {
                let end = receipt
                    .sustained_edges
                    .last()
                    .is_some_and(|s| s.start + s.duration == DT);
                followed += usize::from(end);
                active = end;
            }
            if tick == 1 {
                let [hit] = receipt.planar_impacts.as_slice() else {
                    return Err("lost initial patch".into());
                };
                if hit.resultant.elapsed != Duration::ZERO {
                    return Err("unmatched initial patch time".into());
                }
                after_initial = DVec3::from(hit.resultant.impulse_on_foot);
                first_phase = Some(hit.bearing);
            }
            for pair in receipt.pairs {
                if (pair.a, pair.b) != (foot.min(pad), foot.max(pad)) {
                    if pair.normal_impulse > 1e-10 {
                        return Err("unmodeled competing reaction".into());
                    }
                    continue;
                }
                impulse += pair.normal_impulse;
                native_cumulative += if pair.a == foot { 1. } else { -1. }
                    * (DVec3::from(pair.normal_impulse_on_a)
                        + DVec3::from(pair.tangent_impulse_on_a.ok_or("missing tangent")?));
            }
        }
        let b = links(
            native.state(foot).unwrap(),
            upper.and_then(|id| native.state(id)),
        );
        if tick > 0 && CLOCKS.contains(&(tick - 1)) {
            let r = reference[tick - 1];
            let relative_center = 0.5 * (DVec3::from(b[0].position) + DVec3::from(b[1].position))
                - deck_v * (tick as f64 * 0.0005);
            let [_, _, _, w, jx, jz, _, _] = r.state;
            let contact = DVec3::new(jx, 0., jz);
            let momentum = 2. * (velocity(b) - incoming - gravity * (tick as f64 * 0.0005));
            rows.push(json!({"tick":tick,"followed_ticks":followed,"cumulative_native_calls":total,"reference_post_impulse_ns":contact.to_array(),"momentum_post_impulse_ns":(momentum-after_initial).to_array(),"receipt_post_impulse_ns":(native_cumulative-after_initial).to_array(),
                "momentum_error_ns":(momentum-after_initial).distance(contact),"receipt_error_ns":(native_cumulative-after_initial).distance(contact),"com_error_m":relative_center.distance(r.com),
                "spin_error_rad_s":DVec3::from(b[0].angular_velocity).distance(DVec3::Y*w),"post_impulse_magnitude_ns":contact.length(),
                "instantaneous_ns":p.impulse,"bearing":format!("{:?}",first_phase.unwrap())}));
        }
        if show {
            frames.push(json!({"tick":tick,"time_us":tick*500,"foot":as_json(b[0]),"upper":as_json(b[1]),"pad":as_json(native.state(pad).unwrap()),"native_steps":calls,"solver_iterations":4,"normal_impulse_ns":impulse}));
        }
    }
    Ok((rows, frames))
}
pub(super) fn study(out: &Path, follow: bool, gravity: DVec3, pivot: bool) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut lanes = vec![];
    let mut display = (!pivot).then_some(12257);
    let mut candidate = 0;
    let mut arc_steps = 0;
    let mut reference_steps = 0;
    let mut arc_errors = [0.0_f64; 7];
    let mut arc_cases = 0;
    let mut no_load = 0;
    let mut invariants = [0.0_f64; 3];
    let mut split = 0.0_f64;
    for k in 0..FIELD {
        let c = Case::at(k);
        let p = c.analytic()?;
        if p.bearing == Bearing::Face
            || (p.center_of_pressure.abs() - FOOT[0]).abs() < 1e-11
            || p.sticking != pivot
        {
            continue;
        }
        candidate += 1;
        let (fine, e) = oracle(c, p, 8, gravity)?;
        if fine.is_empty() {
            no_load += 1;
            continue;
        }
        let (coarse, _) = oracle(c, p, 4, gravity)?;
        if coarse.len() != fine.len() {
            return Err(format!("unresolved mode transition {k}").into());
        }
        for (a, b) in fine.iter().zip(&coarse) {
            for (x, y) in a.state.into_iter().zip(b.state) {
                split = split.max((x - y).abs());
            }
        }
        for (v, u) in invariants.iter_mut().zip(e) {
            *v = v.max(u);
        }
        let mut a = gz_physics::PlanarEdgeArc {
            contact: PlanarEdgeContact {
                mass: 2.,
                pitch_inertia: 0.05,
                height: HEIGHT,
                contact_x: p.center_of_pressure,
                tangent_center_speed: p.com_velocity[0],
                pitch_rate: p.pitch_rate,
                friction: c.mu,
                acceleration: [gravity.x, gravity.z],
                external_torque: 0.,
            },
            angle: 0.,
            sole_bounds: [-FOOT[0], FOOT[0]],
            deck: if p.bearing == Bearing::LowerXEdge {
                gz_physics::DeckDirection::Positive
            } else {
                gz_physics::DeckDirection::Negative
            },
        };
        let mut j = [0.; 2];
        let mut lost = 0.;
        let mut admitted = 0;
        reference_steps += fine.len();
        for r in &fine {
            let Some(step) = a.advance(DT)? else {
                break;
            };
            a = step.next;
            admitted += 1;
            j[0] += step.normal_impulse[0] + step.tangent_impulse[0];
            j[1] += step.normal_impulse[1] + step.tangent_impulse[1];
            lost += step.lost_energy;
            let [x, z] = step.relative_center;
            let values = [
                (x + p.center_of_pressure - r.com.x).hypot(z - r.com.z),
                (step.relative_velocity[0] - r.velocity.x)
                    .hypot(step.relative_velocity[1] - r.velocity.z),
                (a.contact.pitch_rate - r.state[3]).abs(),
                (a.angle - r.state[0]).abs(),
                (j[0] - r.state[4]).hypot(j[1] - r.state[5]),
                (lost - r.state[6]).abs(),
                (a.contact.contact_x - r.state[1]).abs(),
            ];
            for (err, v) in arc_errors.iter_mut().zip(values) {
                *err = err.max(v);
            }
        }
        if display.is_none() && admitted == TICKS && p.pitch_rate.abs() > 0.5 {
            display = Some(k);
        }
        arc_steps += admitted;
        arc_cases += usize::from(admitted > 0);
        let mut profiles = vec![];
        let modes = if follow {
            [("two", 2), ("sixty_four", 64), ("arc", 1)]
        } else {
            [(LABELS[0], 2), (LABELS[1], 8), (LABELS[2], 64)]
        };
        for (id, (label, n)) in modes.into_iter().enumerate() {
            let show = display == Some(k);
            let (samples, frames) = measured(c, p, &fine, n, show, gravity)?;
            profiles.push(json!({"profile":label,"calls":n,"samples":samples}));
            if show {
                write_json(
                    &out.join("traces").join(format!("{label}.json")),
                    &json!({"geometry":{"foot_half_extents":FOOT,"pad_half_extents":PAD,"upper_half_extents":TOP},"frames":frames}),
                )?;
                lanes.push(json!({"label":label,"name":if n==1{"Admitted continuous deck-edge reaction".into()}else{format!("{n} native calls after the same patch impact")},"file":format!("traces/{label}.json"),"profile":id}));
            }
        }
        rows.push(json!({"case":k,"fixture":c,"active_ticks":fine.len(),"arc_ticks":admitted,"profiles":profiles}));
    }
    if invariants.into_iter().any(|v| v > 1e-8) || split > 1e-8 {
        return Err(format!("edge convergence {invariants:?}, {split}").into());
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Sustained loading on a deck edge after touchdown","pair_totals_only":true,"lanes":lanes,"case":display}),
    )?;
    write_json(
        &out.join("study.json"),
        &json!({"sticking":pivot,"gravity_m_s2":gravity.to_array(),"inputs":FIELD,"candidate":candidate,"no_full_loaded_tick":no_load,"certified":rows.len(),"conserved_angular_momentum_energy_plus_loss_linear_impulse":invariants,"max_double_grid_difference":split,"arc_cases":arc_cases,"arc_accepted_ticks":arc_steps,"reference_ticks":reference_steps,"arc_max_com_speed_spin_angle_impulse_work_contact_x_errors":arc_errors,"rows":rows}),
    )
}
