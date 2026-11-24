// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Numerical alternatives for a closed, sliding edge. Two atoms of pressure
//! on the line obey Coulomb friction locally. Other distributions are possible.
use super::*;
use gz_physics::FiniteBoxFlight;

const MASS: f64 = 2.;
const INERTIA: f64 = 0.05;
const MU: f64 = 0.5;

fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}

#[derive(Clone, Copy)]
struct Response {
    linear: DVec3,
    angular: DVec3,
    impulses: [DVec3; 2],
    pressures: [f64; 2],
}

// With friction directions fixed, eliminate the two normal equations exactly.
// Use the specified material sites for both the normal and tangent resultants.
fn response(arms: [DVec3; 2], normal: DVec3, slips: [DVec3; 2], rhs: [f64; 2]) -> Option<Response> {
    let tangent = slips.map(|v| v.reject_from_normalized(normal).try_normalize());
    let [Some(t0), Some(t1)] = tangent else {
        return None;
    };
    let cone = [normal - MU * t0, normal - MU * t1];
    let entry = |i: usize, j: usize| {
        normal.dot(cone[j] / MASS + arms[j].cross(cone[j]).cross(arms[i]) / INERTIA)
    };
    let (a, b, c, d) = (entry(0, 0), entry(0, 1), entry(1, 0), entry(1, 1));
    let det = a * d - b * c;
    if det.abs() <= 1e-12 * (a.abs() + b.abs()) * (c.abs() + d.abs()) {
        return None;
    }
    let pressures = [
        (d * rhs[0] - b * rhs[1]) / det,
        (a * rhs[1] - c * rhs[0]) / det,
    ];
    if pressures.iter().any(|p| !p.is_finite() || *p <= 0.) {
        return None;
    }
    let impulses = [cone[0] * pressures[0], cone[1] * pressures[1]];
    Some(Response {
        linear: impulses[0] + impulses[1],
        angular: arms[0].cross(impulses[0]) + arms[1].cross(impulses[1]),
        impulses,
        pressures,
    })
}

fn impact(
    f: FiniteBoxFlight,
    points: [DVec3; 2],
    n: DVec3,
    fractions: [f64; 2],
) -> Option<(FiniteBoxFlight, Value)> {
    let sites = fractions.map(|t| points[0].lerp(points[1], t));
    let arms = sites.map(|p| p - f.center_of_mass);
    let before = arms.map(|r| f.relative_velocity + f.angular_velocity.cross(r));
    let rhs = before.map(|v| -v.dot(n));
    let mut slips = before;
    for iteration in 1..=128 {
        let r = response(arms, n, slips, rhs)?;
        let post = FiniteBoxFlight {
            relative_velocity: f.relative_velocity + r.linear / MASS,
            angular_velocity: f.angular_velocity + r.angular / INERTIA,
            ..f
        };
        let after = arms.map(|arm| post.relative_velocity + post.angular_velocity.cross(arm));
        let res = std::array::from_fn::<_, 2, _>(|i| {
            (r.impulses[i].reject_from_normalized(n)
                + MU * r.pressures[i] * after[i].reject_from_normalized(n).normalize())
            .length()
        });
        if res.into_iter().all(|x| x < 2e-15) {
            let vn = points.map(|p| {
                n.dot(post.relative_velocity + post.angular_velocity.cross(p - f.center_of_mass))
            });
            let power = std::array::from_fn::<_, 2, _>(|i| r.impulses[i].dot(after[i]));
            let linear = (MASS * (post.relative_velocity - f.relative_velocity)).distance(r.linear);
            let angular =
                (INERTIA * (post.angular_velocity - f.angular_velocity)).distance(r.angular);
            let energy = |u: FiniteBoxFlight| {
                0.5 * (MASS * u.relative_velocity.length_squared()
                    + INERTIA * u.angular_velocity.length_squared())
            };
            let loss = energy(f) - energy(post);
            let work = (0..2)
                .map(|i| -0.5 * r.impulses[i].dot(before[i] + after[i]))
                .sum::<f64>();
            if vn.iter().any(|v| v.abs() > 1e-12)
                || power.iter().any(|x| *x >= 0.)
                || loss <= 0.
                || (loss - work).abs() > 1e-12
                || linear > 1e-12
                || angular > 1e-12
            {
                return None;
            }
            return Some((
                post,
                json!({
                    "fractions":fractions,"iterations":iteration,"sites_m":sites.map(|p|p.to_array()),
                    "site_j_ns":r.impulses.map(|p|p.to_array()),"normal_ns":r.pressures,"j_ns":r.linear.to_array(),
                    "torque_nms":r.angular.to_array(),"post_velocity_m_s":post.relative_velocity.to_array(),"post_spin_rad_s":post.angular_velocity.to_array(),
                    "slip_speeds_m_s":after.map(|v|v.reject_from_normalized(n).length()),"line_endpoint_normal_speed_m_s":vn,
                    "friction_law_residual_ns":res,"post_friction_work_j":power,"lost_energy_j":loss,
                    "energy_balance_j":loss-work,"linear_balance_ns":linear,"angular_balance_nms":angular,
                }),
            ));
        }
        slips = after;
    }
    None
}

pub(super) fn study(
    original: FiniteBoxFlight,
    stopped: &Value,
    line: &Value,
    horizon_s: f64,
) -> Result<Value> {
    let f = FiniteBoxFlight {
        moving: gz_physics::FlightBox {
            center: vector(&stopped["foot"])?,
            orientation: DQuat::from_array(serde_json::from_value(stopped["orientation"].clone())?),
            ..original.moving
        },
        center_of_mass: vector(&stopped["com"])?,
        relative_velocity: vector(&stopped["velocity"])?,
        angular_velocity: vector(&stopped["omega"])?,
        ..original
    };
    let points = [
        vector(&line["line_points_m"][0])?,
        vector(&line["line_points_m"][1])?,
    ];
    let normal = vector(&line["normal"])?;
    let at = stopped["elapsed"]
        .as_f64()
        .ok_or("missing failed line clock")?;
    let mut alternatives = vec![];
    for (label, fractions) in [
        ("both endpoints", [0., 1.]),
        ("first endpoint and three-quarter point", [0., 0.75]),
        ("first endpoint and midpoint", [0., 0.5]),
    ] {
        if let Some((post, mut row)) = impact(f, points, normal, fractions) {
            row["pressure_locations"] = json!(label);
            row["continuation"] = continuation(post, normal, fractions, at, horizon_s);
            alternatives.push(row);
        }
    }
    Ok(
        json!({"start_s":at,"horizon_s":horizon_s,"reason":"Two compressive pressure atoms at the specified fractions of the clipped contact segment.","alternatives":alternatives}),
    )
}

#[derive(Clone, Copy)]
struct Load {
    force: Response,
    dissipation: f64,
    max_gap: f64,
    max_normal_speed: f64,
    min_slip: f64,
}

fn load(
    f: FiniteBoxFlight,
    feature: gz_physics::EdgeFaceFeature,
    normal: DVec3,
    fractions: [f64; 2],
) -> std::result::Result<Load, &'static str> {
    let (seen, line) = f
        .incident_segment_now(1e-9)
        .ok_or("lost finite edge/face geometry")?;
    if seen != feature || line.normal.distance(normal) > 1e-12 {
        return Err("changed incident edge");
    }
    let endpoint_velocity = line
        .moving_points
        .map(|p| f.relative_velocity + f.angular_velocity.cross(p - f.center_of_mass));
    let max_normal_speed = endpoint_velocity
        .map(|v| v.dot(normal).abs())
        .into_iter()
        .fold(0., f64::max);
    let max_gap = (0..2)
        .map(|i| line.moving_points[i].distance(line.support_points[i]))
        .fold(0., f64::max);
    if max_normal_speed > 1e-8 {
        return Err("lost endpoint normal constraint");
    }
    let arms =
        fractions.map(|x| line.moving_points[0].lerp(line.moving_points[1], x) - f.center_of_mass);
    let velocities = arms.map(|arm| f.relative_velocity + f.angular_velocity.cross(arm));
    let min_slip = velocities
        .map(|v| v.reject_from_normalized(normal).length())
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    if min_slip < 1e-10 {
        return Err("slip stopped");
    }
    // The face normal is constant, and n·(ω × edge_axis) = 0 on this
    // plane-tangent material edge. Migration of the clipped end along that axis
    // therefore contributes no normal acceleration.
    let rhs = arms.map(|r| {
        -normal.dot(f.acceleration + f.angular_velocity.cross(f.angular_velocity.cross(r)))
    });
    let force =
        response(arms, normal, velocities, rhs).ok_or("no two compressive sliding pressures")?;
    for (arm, v) in arms.into_iter().zip(velocities) {
        let acceleration = f.acceleration
            + force.linear / MASS
            + (force.angular / INERTIA).cross(arm)
            + f.angular_velocity.cross(f.angular_velocity.cross(arm));
        if acceleration.dot(normal).abs() > 1e-10 || !v.is_finite() {
            return Err("normal acceleration residual");
        }
    }
    let dissipation = -(0..2)
        .map(|i| force.impulses[i].dot(velocities[i]))
        .sum::<f64>();
    if !dissipation.is_finite() || dissipation < 0. {
        return Err("non-dissipative force");
    }
    Ok(Load {
        force,
        dissipation,
        max_gap,
        max_normal_speed,
        min_slip,
    })
}

fn advance(
    from: FiniteBoxFlight,
    stage: FiniteBoxFlight,
    force: Response,
    dt: f64,
) -> FiniteBoxFlight {
    let delta = DQuat::from_scaled_axis(stage.angular_velocity * dt);
    let center = from.center_of_mass + dt * stage.relative_velocity;
    FiniteBoxFlight {
        moving: gz_physics::FlightBox {
            center: center + delta * (from.moving.center - from.center_of_mass),
            orientation: (delta * from.moving.orientation).normalize(),
            ..from.moving
        },
        center_of_mass: center,
        relative_velocity: from.relative_velocity + dt * (from.acceleration + force.linear / MASS),
        angular_velocity: from.angular_velocity + dt * force.angular / INERTIA,
        ..from
    }
}

fn continuation(
    f: FiniteBoxFlight,
    normal: DVec3,
    fractions: [f64; 2],
    at: f64,
    horizon_s: f64,
) -> Value {
    let Some((feature @ gz_physics::EdgeFaceFeature::MovingEdge { .. }, _)) =
        f.incident_segment_now(1e-9)
    else {
        return json!({"end":"unsupported incident geometry"});
    };
    let mut lanes = vec![];
    for mesh in [32, 128] {
        let (mut now, mut state) = (at, f);
        let mut frames = vec![];
        let mut min_pressure = f64::INFINITY;
        let mut min_slip = f64::INFINITY;
        let mut max_gap = 0f64;
        let mut max_normal_speed = 0f64;
        let mut work = 0.;
        let mut impulse = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let mut end = "original_clock";
        let tick_s = DT.as_secs_f64();
        for at_tick in ((at / tick_s).floor() as u64 + 1)..=(horizon_s / tick_s).round() as u64 {
            let goal = tick_s * at_tick as f64;
            let dt = (goal - now) / f64::from(mesh);
            for _ in 0..mesh {
                let one = |s| load(s, feature, normal, fractions);
                let update = (|| {
                    let a = one(state)?;
                    let mid = advance(state, state, a.force, 0.5 * dt);
                    let b = one(mid)?;
                    let next = advance(state, mid, b.force, dt);
                    let c = one(next)?;
                    Ok::<_, &'static str>((next, [a, b, c]))
                })();
                let (next, loads) = match update {
                    Ok(x) => x,
                    Err(reason) => {
                        end = reason;
                        break;
                    }
                };
                for l in loads {
                    min_pressure = min_pressure
                        .min(l.force.pressures[0])
                        .min(l.force.pressures[1]);
                    min_slip = min_slip.min(l.min_slip);
                    max_gap = max_gap.max(l.max_gap);
                    max_normal_speed = max_normal_speed.max(l.max_normal_speed);
                }
                impulse += dt * loads[1].force.linear;
                torque += dt * loads[1].force.angular;
                work -= dt * loads[1].dissipation;
                now += dt;
                state = next;
            }
            if end != "original_clock" {
                break;
            }
            now = goal;
            frames.push(json!({"original_time_s":now,"com_m":state.center_of_mass.to_array(),"foot_m":state.moving.center.to_array(),"orientation_xyzw":state.moving.orientation.to_array(),"v_m_s":state.relative_velocity.to_array(),"spin_rad_s":state.angular_velocity.to_array(),"support_frame_j_ns":impulse.to_array()}));
        }
        let energy = |s: FiniteBoxFlight| {
            0.5 * (MASS * s.relative_velocity.length_squared()
                + INERTIA * s.angular_velocity.length_squared())
        };
        let balance = energy(state)
            - energy(f)
            - MASS * f.acceleration.dot(state.center_of_mass - f.center_of_mass)
            - work;
        lanes.push(json!({"subdivisions_per_output":mesh,"end":end,"last_time_s":now,"frames":frames,"min_pressure_n":min_pressure,"minimum_slip_m_s":min_slip,
            "max_normal_velocity_m_s":max_normal_speed,"max_gap_m":max_gap,"friction_work_j":work,"energy_balance_j":balance,
            "linear_balance_ns":impulse.distance(MASS*(state.relative_velocity-f.relative_velocity-(now-at)*f.acceleration)),
            "angular_balance_nms":torque.distance(INERTIA*(state.angular_velocity-f.angular_velocity))}));
    }
    json!(lanes)
}
