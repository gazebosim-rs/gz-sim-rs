// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A vertical elastic coating bonded to the whole lower foot face. The
//! finite deck only supplies a supporting top surface. All areas and energy
//! fluxes refer to the undeformed sole material, in the translating deck frame.
use super::*;
use gz_physics::{DampedAreaFoundation, ElasticAreaFoundation};
#[path = "damped_landing.rs"]
pub(in super::super) mod damped_landing;

const LAYER_K: f64 = 2e8; // N/m³ per layer, chosen for this sensitivity study.
const SLIP_SPEED: f64 = 0.005; // m/s in the smooth sliding friction law.

struct Sole {
    half: DVec3,
    deck: DVec3,
    deck_half: DVec3,
    foot_local: DVec3,
    material: ElasticAreaFoundation,
    damping: Option<(f64, DampedAreaFoundation)>,
    gravity: DVec3,
}
#[derive(Default)]
struct Patch {
    load: Load,
    polygon: Vec<DVec3>,
    area: f64,
    first_moment: DVec3,
    maximum_depth: f64,
    heat_power: f64,
}

fn clip(points: Vec<DVec3>, inside: impl Fn(DVec3) -> f64) -> Vec<DVec3> {
    let Some(&last) = points.last() else {
        return points;
    };
    let mut prev = last;
    let mut d = inside(prev);
    let mut out = Vec::with_capacity(points.len() + 1);
    for p in points {
        let dp = inside(p);
        if (d < 0. && dp > 0.) || (d > 0. && dp < 0.) {
            out.push(prev.lerp(p, d / (d - dp)));
        }
        if dp >= 0. {
            out.push(p);
        }
        prev = p;
        d = dp;
    }
    out
}

impl Sole {
    fn height(&self) -> f64 {
        self.deck.z + self.deck_half.z
    }
    fn corners(&self, state: State) -> [DVec3; 4] {
        [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)].map(|(x, y)| {
            state.center + state.orientation * (self.foot_local + self.half * DVec3::new(x, y, -1.))
        })
    }

    fn active(&self, state: State) -> Vec<DVec3> {
        let mut p = self.corners(state).to_vec();
        for a in 0..2 {
            p = clip(p, |x| x[a] - self.deck[a] + self.deck_half[a]);
            p = clip(p, |x| self.deck[a] + self.deck_half[a] - x[a]);
        }
        clip(p, |x| self.height() - x.z)
    }

    fn patch(&self, state: State, depth: usize) -> Result<Patch> {
        let height = self.height();
        let normal = state.orientation * DVec3::Z;
        let sole = self.corners(state);
        // The top of the foot and upper link must stay above the deck; the
        // finite coating is used only while every vertical indentation is small.
        let top_clear = sole
            .iter()
            .all(|p| p.z + 2. * self.half.z * normal.z > height);
        let upper_center = state.center - state.orientation * self.foot_local;
        let upper_extent = (state.orientation * DVec3::X).z.abs() * 0.05
            + (state.orientation * DVec3::Y).z.abs() * 0.06
            + normal.z.abs() * 0.06;
        if normal.z < 0.99 || !top_clear || upper_center.z - upper_extent <= height {
            return Err("motion outside sole coating geometry".into());
        }
        let polygon = self.active(state);
        let mut p = Patch {
            polygon,
            ..Default::default()
        };
        if p.polygon.len() < 3 {
            p.polygon.clear();
            return Ok(p);
        }
        for i in 1..p.polygon.len() - 1 {
            let tri = [p.polygon[0], p.polygon[i], p.polygon[i + 1]];
            let area = (tri[1] - tri[0]).cross(tri[2] - tri[0]).length() * 0.5;
            if area == 0. {
                continue;
            }
            let overlap = tri.map(|v| (height - v.z).max(0.));
            p.maximum_depth = p.maximum_depth.max(overlap.into_iter().fold(0., f64::max));
            let result = self
                .material
                .integrate_triangle(area, overlap)
                .ok_or("invalid sole triangle")?;
            p.area += area;
            p.load.elastic_energy += result.energy_j;
            if self.damping.is_some() {
                continue; // Pressure and friction depend on local normal velocity.
            }
            p.load.force.z += result.normal_force_n;
            let moment = tri
                .iter()
                .zip(result.nodal_forces_n)
                .map(|(w, f)| *w * f)
                .sum::<DVec3>();
            p.first_moment += moment;
            p.load.torque += (moment - state.center * result.normal_force_n).cross(DVec3::Z);
            friction(tri, area, depth, height, state, self.material, &mut p.load);
        }
        if p.maximum_depth > self.half.z {
            return Err("deformed through the sole layer".into());
        }
        if self.damping.is_none() && p.load.force.z <= 0. {
            p.polygon.clear();
            return Ok(p);
        }
        // Moving footprint boundary carries stored energy into or out of the
        // active material. Its speed is measured in the material surface.
        for (a, b) in p.polygon.iter().zip(p.polygon.iter().cycle().skip(1)) {
            for axis in 0..2 {
                for sign in [-1., 1.] {
                    let side = self.deck[axis] + sign * self.deck_half[axis];
                    if (a[axis] - side).abs() > 1e-12 || (b[axis] - side).abs() > 1e-12 {
                        continue;
                    }
                    let grad = (1. - normal[axis] * normal[axis]).sqrt();
                    for u in [0.5 - 0.5 / 3_f64.sqrt(), 0.5 + 0.5 / 3_f64.sqrt()] {
                        let pt = a.lerp(*b, u);
                        let v = state.velocity + state.spin.cross(pt - state.center);
                        let e = self
                            .material
                            .sample(height - pt.z)
                            .ok_or("invalid boundary energy")?
                            .energy_j_m2;
                        p.load.boundary_power +=
                            0.5 * a.distance(*b) * e * (-sign * v[axis]) / grad;
                    }
                }
            }
        }
        if let Some((alpha, material)) = self.damping {
            damped_landing::apply_pressure(&mut p, state, height, depth, alpha, material)?;
        }
        Ok(p)
    }
}

fn friction(
    t: [DVec3; 3],
    area: f64,
    depth: usize,
    height: f64,
    state: State,
    mat: ElasticAreaFoundation,
    load: &mut Load,
) {
    let mut shear = |pt: DVec3, weight: f64, area: f64| {
        let pressure = mat
            .sample(height - pt.z)
            .expect("finite clipped triangle")
            .pressure_pa;
        let arm = pt - state.center;
        let v = (state.velocity + state.spin.cross(arm)).reject_from_normalized(DVec3::Z);
        let force =
            -MU * weight * area * pressure * v / (v.length_squared() + SLIP_SPEED.powi(2)).sqrt();
        load.force += force;
        load.torque += arm.cross(force);
        load.friction_power += force.dot(v);
    };
    for_each_site(t, area, depth, &mut shear);
}

// Degree-five symmetric triangle rule with positive weights.
fn for_each_site(t: [DVec3; 3], area: f64, depth: usize, visit: &mut impl FnMut(DVec3, f64, f64)) {
    let [a, b, c] = t;
    if depth > 0 {
        let ab = 0.5 * (a + b);
        let bc = 0.5 * (b + c);
        let ca = 0.5 * (c + a);
        for sub in [[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]] {
            for_each_site(sub, 0.25 * area, depth - 1, visit);
        }
        return;
    }
    let mut site = |x: f64, y: f64, weight: f64| visit(a + x * (b - a) + y * (c - a), weight, area);
    site(1. / 3., 1. / 3., 0.225);
    for (u, w) in [
        (0.470142064105115, 0.132394152788506),
        (0.101286507323456, 0.125939180544827),
    ] {
        let v = 1. - 2. * u;
        site(u, u, w);
        site(u, v, w);
        site(v, u, w);
    }
}

// Hold one separating plane fixed in the deck frame. Along its normal, every
// ballistic vertex has a negative acceleration bound, so positive endpoint
// gaps certify the entire intervening path, even for a foot overhanging a rim.
fn separating_plane(sole: &Sole, departure: State, arrival: State) -> Option<Value> {
    let fixed = [DVec3::X, DVec3::Y, DVec3::Z];
    let moving = fixed.map(|a| departure.orientation * a);
    let mut axes = fixed.to_vec();
    axes.extend(moving);
    axes.extend(fixed.iter().flat_map(|a| moving.map(|b| a.cross(b))));
    let mut best = None;
    let mut best_gap = 0.;
    for axis in axes {
        let Some(direction) = axis.try_normalize() else {
            continue;
        };
        let n = if direction.z < 0. {
            -direction
        } else {
            direction
        };
        let accel = sole.gravity.dot(n)
            + departure.spin.length_squared() * (sole.foot_local.length() + sole.half.length());
        if accel >= 0. {
            continue;
        }
        let plane = sole.deck.dot(n) + sole.deck_half.dot(n.abs());
        let gap = |s: State| {
            let foot = s.center + s.orientation * sole.foot_local;
            (foot.dot(n) - sole.half.dot((s.orientation.conjugate() * n).abs())) - plane
        };
        let first = gap(departure);
        let last = gap(arrival);
        if first.min(last) > best_gap {
            best_gap = first.min(last);
            best = Some(
                json!({"normal":n.to_array(),"plane_offset_m":plane,"departure_gap_m":first,
                "horizon_gap_m":last,"max_projected_acceleration_m_s2":accel}),
            );
        }
    }
    best
}

struct Trajectory {
    frames: Vec<Value>,
    positions: Vec<DVec3>,
    impulses: Vec<DVec3>,
    normals: Vec<f64>,
    checks: Value,
}
fn integrate(
    initial: State,
    sole: &Sole,
    times: &[f64],
    depth: usize,
    substeps: usize,
    display: bool,
) -> Result<Trajectory> {
    integrate_history(initial, sole, times, depth, substeps, display, false)
}

fn integrate_history(
    initial: State,
    sole: &Sole,
    times: &[f64],
    depth: usize,
    substeps: usize,
    display: bool,
    loss_report: bool,
) -> Result<Trajectory> {
    let mut state = initial;
    let initial_energy = sole.patch(initial, depth)?.load.elastic_energy;
    let mut frames = vec![];
    let mut positions = vec![];
    let mut impulses = vec![];
    let mut normals = vec![];
    let (mut j, mut angular) = (DVec3::ZERO, DVec3::ZERO);
    let (mut work, mut flux, mut peak, mut energy_defect, mut max_depth) =
        (0., 0., 0f64, 0f64, 0f64);
    let mut heat = 0.;
    let mut was_loaded = false;
    let mut history = vec![];
    let mut last_observation = times[0];
    let mut released = None;
    let start = times[0];
    for (index, &at) in times.iter().enumerate() {
        if index > 0 {
            let prev = times[index - 1];
            let n = (((at - prev) / (CLOCK_S / substeps as f64)).ceil() as usize).max(1);
            let dt = (at - prev) / n as f64;
            for k in 0..n {
                let now = prev + k as f64 * dt;
                let a = sole.patch(state, depth)?;
                let loaded = a.load.force.z > 0.;
                if loaded != was_loaded {
                    history.push(json!({"time_bracket_s":[last_observation,now],"loaded":loaded}));
                    released = (!loaded).then_some((now, state));
                    was_loaded = loaded;
                }
                last_observation = now;
                peak = peak.max(a.load.force.z);
                max_depth = max_depth.max(a.maximum_depth);
                let mid = state.advance_with_gravity(state, &a.load, dt * 0.5, sole.gravity);
                let b = sole.patch(mid, depth)?;
                state = state.advance_with_gravity(mid, &b.load, dt, sole.gravity);
                j += dt * b.load.force;
                angular += dt * b.load.torque;
                work += dt * b.load.friction_power;
                heat += dt * b.heat_power;
                flux += dt * b.load.boundary_power;
                peak = peak.max(b.load.force.z);
                max_depth = max_depth.max(b.maximum_depth);
            }
        }
        let p = sole.patch(state, depth)?;
        if (p.load.force.z > 0.) != was_loaded {
            history
                .push(json!({"time_bracket_s":[last_observation,at],"loaded":p.load.force.z>0.}));
            released = (p.load.force.z == 0.).then_some((at, state));
            was_loaded = p.load.force.z > 0.;
        }
        last_observation = at;
        peak = peak.max(p.load.force.z);
        max_depth = max_depth.max(p.maximum_depth);
        let defect = state.kinetic() - initial.kinetic() + p.load.elastic_energy
            - initial_energy
            - MASS * sole.gravity.dot(state.center - initial.center)
            + heat
            - work
            - flux;
        energy_defect = energy_defect.max(defect.abs());
        positions.push(state.center);
        impulses.push(j);
        normals.push(p.load.force.z);
        if display {
            let cop = (p.load.force.z > 0.).then(|| (p.first_moment / p.load.force.z).to_array());
            let mut frame = json!({"original_time_s":at,"com_m":state.center.to_array(),"foot_m":(state.center+state.orientation*sole.foot_local).to_array(),"orientation_xyzw":state.orientation.to_array(),"v_m_s":state.velocity.to_array(),"spin_rad_s":state.spin.to_array(),"support_frame_j_ns":j.to_array(),"normal_force_n":p.load.force.z,"loaded_area_m2":if cop.is_some(){p.area}else{0.},"center_of_pressure_m":cop,"loaded_polygon_m":p.polygon.iter().map(|v|v.to_array()).collect::<Vec<_>>(),"strain_energy_j":p.load.elastic_energy});
            if loss_report {
                frame["normal_dissipation_j"] = json!(heat);
                frame["normal_dissipation_w"] = json!(p.heat_power);
                frame["sliding_work_j"] = json!(work);
                frame["boundary_energy_flux_j"] = json!(flux);
                frame["max_indentation_m"] = json!(p.maximum_depth);
            }
            frames.push(frame);
        }
    }
    let horizon = *times.last().ok_or("missing final clock")?;
    let free_flight = if let Some((t, s)) = released {
        let dt = horizon - t;
        let rotation = (DQuat::from_scaled_axis(dt * s.spin) * s.orientation).normalize();
        let predicted = State {
            center: s.center + dt * s.velocity + 0.5 * dt * dt * sole.gravity,
            velocity: s.velocity + dt * sole.gravity,
            orientation: rotation,
            spin: s.spin,
        };
        let departure = sole
            .corners(s)
            .iter()
            .map(|p| p.z - sole.height())
            .fold(f64::INFINITY, f64::min);
        let clearance = sole
            .corners(predicted)
            .iter()
            .map(|p| p.z - sole.height())
            .fold(f64::INFINITY, f64::min);
        let acceleration = sole.gravity.z
            + s.spin.length_squared() * (sole.foot_local.length() + sole.half.length());
        let defect = sole
            .corners(predicted)
            .into_iter()
            .zip(sole.corners(state))
            .map(|(x, y)| x.distance(y))
            .fold(0., f64::max);
        let separator = separating_plane(sole, s, predicted);
        json!({"from_s":t,"to_s":horizon,"separating_plane":separator,"min_departure_gap_m":departure,
        "min_horizon_gap_m":clearance,"vertical_acceleration_bound_m_s2":acceleration,
        "ballistic_endpoint_defect_m":defect,"all_vertices_clear":departure>=0. && clearance>0. && acceleration<0.})
    } else {
        Value::Null
    };
    let j_defect = (MASS * (state.velocity - initial.velocity - (horizon - start) * sole.gravity)
        - j)
        .length();
    let angular_defect = (INERTIA * (state.spin - initial.spin) - angular).length();
    if work > 0. || !energy_defect.is_finite() || j_defect > 1e-10 || angular_defect > 1e-10 {
        return Err("momentum/dissipation failure".into());
    }
    let mut checks = json!({"quadrature_depth":depth,"substeps_per_public_clock":substeps,"peak_normal_n":peak,"maximum_indentation_m":max_depth,"max_energy_defect_j":energy_defect,"linear_impulse_defect_ns":j_defect,"angular_impulse_defect_nms":angular_defect,"sliding_work_j":work,"boundary_energy_flux_j":flux,"contact_transitions":history,"final_ballistic_check":free_flight});
    if loss_report {
        checks["normal_dissipation_j"] = json!(heat);
        if let Some((t, state)) = released {
            checks["lift_off_time_s"] = json!(t);
            checks["rebound_speed_m_s"] = json!(state.velocity.z);
        }
    }
    Ok(Trajectory {
        frames,
        positions,
        impulses,
        normals,
        checks,
    })
}

fn parse_state(d: &Value) -> Result<State> {
    let orientation = DQuat::from_array(serde_json::from_value(d["orientation"].clone())?);
    if !orientation.is_normalized() {
        return Err("invalid orientation".into());
    }
    Ok(State {
        center: vector(&d["com"])?,
        velocity: vector(&d["velocity"])?,
        spin: vector(&d["omega"])?,
        orientation,
    })
}

fn delta(a: &Trajectory, b: &Trajectory) -> Value {
    let dist = |a: &[DVec3], b: &[DVec3]| {
        a.iter()
            .zip(b)
            .map(|(a, b)| a.distance(*b))
            .fold(0., f64::max)
    };
    json!({"max_com_delta_m":dist(&a.positions,&b.positions),"max_impulse_delta_ns":dist(&a.impulses,&b.impulses),"max_sampled_normal_force_delta_n":a.normals.iter().zip(&b.normals).map(|(x,y)|(x-y).abs()).fold(0.,f64::max)})
}

fn compare_histories(profiles: &[Value]) -> Result<Vec<Value>> {
    let mut pairs = vec![];
    for (i, early) in profiles.iter().enumerate() {
        for late in &profiles[i + 1..] {
            let mut rows = vec![];
            let before = vector(&early["rigid_impulse_before_start_ns"])?
                - vector(&late["rigid_impulse_before_start_ns"])?;
            for b in array(&late["frames"])? {
                let t = num(&b["original_time_s"])?;
                let a = array(&early["frames"])?
                    .iter()
                    .find(|v| v["original_time_s"] == b["original_time_s"])
                    .ok_or("missing matched public clock")?;
                let impulse =
                    before + vector(&a["support_frame_j_ns"])? - vector(&b["support_frame_j_ns"])?;
                if impulse.distance(MASS * (vector(&a["v_m_s"])? - vector(&b["v_m_s"])?)) > 1e-9 {
                    return Err("common-clock impulse differs from momentum".into());
                }
                rows.push(json!({"original_time_s":t,
                    "com_distance_m":vector(&a["com_m"])?.distance(vector(&b["com_m"])?),
                    "impulse_difference_ns":impulse.to_array(),"impulse_distance_ns":impulse.length(),
                    "earlier_normal_force_n":a["normal_force_n"],"later_normal_force_n":b["normal_force_n"]}));
            }
            pairs.push(json!({"earlier":early["name"],"later":late["name"],"frames":rows}));
        }
    }
    Ok(pairs)
}

pub(in super::super) fn study(
    donor_path: &Path,
    corner_path: &Path,
    rigid_path: &Path,
    out: &Path,
) -> Result<()> {
    let donor = read(donor_path)?;
    let corner_doc = read(corner_path)?;
    let rigid = read(rigid_path)?;
    if [&donor, &corner_doc, &rigid]
        .into_iter()
        .any(|d| d["case"] != 80 || d["stroke"] != 2)
    {
        return Err("fixture mismatch".into());
    }
    let original = &donor["initial"];
    let corner = &corner_doc["fine"];
    let line = &array(&rigid["references"])?.last().ok_or("no line donor")?["fine"];
    let initial = parse_state(original)?;
    let body_offset =
        initial.orientation.conjugate() * (vector(&original["foot"])? - initial.center);
    let sole = Sole {
        half: vector(&donor["foot_half_extents_m"])?,
        deck: vector(&donor["deck"]["center_m"])?,
        deck_half: vector(&donor["deck"]["half_extents_m"])?,
        foot_local: body_offset,
        material: ElasticAreaFoundation::new(LAYER_K, LAYER_K).ok_or("invalid material")?,
        gravity: GRAVITY,
        damping: None,
    };
    let start = num(&original["elapsed"])?;
    let horizon = num(&donor["horizon_s"])?;
    let corner_t = num(&corner["elapsed"])?;
    let line_t = num(&line["elapsed"])?;
    let touch = num(&donor["first_arrival"]["time_s"])?;
    if !(start == 0.
        && start < touch
        && touch < corner_t
        && corner_t < line_t
        && line_t < horizon
        && sole.patch(initial, 0)?.load.force.z == 0.)
    {
        return Err("free approach/start order not verified".into());
    }
    let mut clocks: Vec<_> = (0..=(horizon / CLOCK_S).round() as usize)
        .map(|n| n as f64 * CLOCK_S)
        .collect();
    clocks.extend([touch, corner_t, line_t]);
    clocks.sort_by(f64::total_cmp);
    clocks.dedup_by(|a, b| (*a - *b).abs() < 1e-13);
    let mut profiles = vec![];
    for (name, d) in [
        ("Free approach", original),
        ("Corner restart", corner),
        ("Line restart", line),
    ] {
        let initial = parse_state(d)?;
        let at = num(&d["elapsed"])?;
        let times: Vec<_> = clocks
            .iter()
            .copied()
            .filter(|t| *t >= at - 1e-13)
            .collect();
        let run = |depth, rate, show| integrate(initial, &sole, &times, depth, rate, show);
        let fine = run(3, 1024, true)?;
        let space = delta(&run(2, 1024, false)?, &fine);
        let time = delta(&run(3, 512, false)?, &fine);
        for (difference, position, impulse, force) in
            [(&space, 2e-9, 2e-6, 1e-4), (&time, 5e-9, 1e-5, 0.02)]
        {
            if num(&difference["max_com_delta_m"])? > position
                || num(&difference["max_impulse_delta_ns"])? > impulse
                || num(&difference["max_sampled_normal_force_delta_n"])? > force
            {
                return Err(format!("{name} did not converge: {difference}").into());
            }
        }
        if num(&fine.checks["max_energy_defect_j"])? > 5e-7 {
            return Err(format!("{name} failed the energy-balance check").into());
        }
        if name == "Free approach" {
            let flight = &fine.checks["final_ballistic_check"];
            if flight["separating_plane"].is_null()
                || num(&flight["ballistic_endpoint_defect_m"])? > 1e-9
            {
                return Err("the foot did not prove free flight above the finite deck".into());
            }
            let events = array(&fine.checks["contact_transitions"])?;
            if events.len() != 2 || events[0]["loaded"] != true || events[1]["loaded"] != false {
                return Err("free-approach landing does not show a single bounce".into());
            }
            let interval = &events[0]["time_bracket_s"];
            if touch < num(&interval[0])? - 1e-12 || touch > num(&interval[1])? + 1e-12 {
                return Err("elastic onset does not bracket independent first touch".into());
            }
        }
        profiles.push(json!({"name":name,"start_s":at,"rigid_impulse_before_start_ns":d["j"],"frames":fine.frames,"checks":fine.checks,"space_refinement":space,"time_refinement":time}));
    }
    let comparisons = compare_histories(&profiles)?;
    fs::create_dir_all(out)?;
    write(
        &out.join("reference.json"),
        &json!({"case":80,"stroke":2,"start_s":start,"public_restart_s":donor["public_restart_s"],"horizon_s":horizon,"profiles":profiles,"comparisons":comparisons,"material":{"foot_stiffness_n_m3":LAYER_K,"deck_stiffness_n_m3":LAYER_K,"mu":MU,"slip_regularization_m_s":SLIP_SPEED,"surface":"complete lower foot face, vertical compression over finite deck top; undamped, no deck-side reaction"},"coordinate_frame":"original support-translating frame"}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slab() -> Sole {
        Sole {
            half: DVec3::new(0.18, 0.1, 0.06),
            deck: DVec3::new(0., 0., -0.06),
            deck_half: DVec3::new(1., 1., 0.06),
            foot_local: DVec3::NEG_Z * 0.07,
            material: ElasticAreaFoundation::new(LAYER_K, LAYER_K).unwrap(),
            gravity: GRAVITY,
            damping: None,
        }
    }

    #[test]
    fn separation_at_a_finite_rim_survives_an_overhanging_corner() {
        let mut sole = slab();
        sole.deck_half.x = 0.2;
        let from = State {
            center: DVec3::new(-0.1, 0., 0.1313),
            orientation: DQuat::from_rotation_y(-0.01),
            velocity: DVec3::Z,
            spin: DVec3::ZERO,
        };
        let to = State {
            center: from.center + 0.01 * DVec3::Z,
            ..from
        };
        assert!(sole.corners(from).iter().any(|v| v.z < sole.height()));
        assert!(separating_plane(&sole, from, to).is_some());
        let reentered = State {
            center: from.center - 0.003 * DVec3::Z,
            ..from
        };
        assert!(separating_plane(&sole, from, reentered).is_none());
    }

    #[test]
    fn full_flat_undamped_drop_recovers_impact_speed_and_release_time() -> Result<()> {
        let sole = slab();
        let speed = 1.;
        let initial = State {
            center: DVec3::Z * 0.13,
            orientation: DQuat::IDENTITY,
            velocity: -speed * DVec3::Z,
            spin: DVec3::ZERO,
        };
        let k = 0.5 * LAYER_K * 4. * sole.half.x * sole.half.y;
        let w = (k / MASS).sqrt();
        let g = -GRAVITY.z;
        let release = (std::f64::consts::PI + 2. * (g / (speed * w)).atan()) / w;
        let dt = 0.0004;
        let test = integrate(initial, &sole, &[0., release + dt], 0, 512, true)?;
        let expected_peak = k * (g / w.powi(2) + (g / w.powi(2)).hypot(speed / w));
        assert!((num(&test.checks["peak_normal_n"])? / expected_peak - 1.).abs() < 4e-6);
        let steps = array(&test.checks["contact_transitions"])?;
        assert_eq!(steps.len(), 2);
        let window = &steps[1]["time_bracket_s"];
        assert!(num(&window[0])? <= release && release <= num(&window[1])?);
        let last = test.frames.last().unwrap();
        assert!((vector(&last["v_m_s"])?.z - (speed - g * dt)).abs() < 3e-6);
        assert!((vector(&last["com_m"])?.z - (0.13 + speed * dt - 0.5 * g * dt * dt)).abs() < 2e-9);
        Ok(())
    }

    #[test]
    fn moving_rim_flux_matches_the_change_in_stored_coating_energy() -> Result<()> {
        let mut sole = slab();
        sole.deck_half.x = 0.17;
        let state = State {
            center: DVec3::new(0.12, 0., 0.1299),
            orientation: DQuat::IDENTITY,
            velocity: DVec3::X * 0.03,
            spin: DVec3::ZERO,
        };
        let p = sole.patch(state, 0)?;
        assert!(p.load.boundary_power < 0.);
        let h = 1e-5;
        let e = |t: f64| -> Result<f64> {
            Ok(sole
                .patch(
                    State {
                        center: state.center + t * state.velocity,
                        ..state
                    },
                    0,
                )?
                .load
                .elastic_energy)
        };
        assert!(((e(h)? - e(-h)?) / (2. * h) - p.load.boundary_power).abs() < 1e-9);
        assert!(p.load.friction_power < 0.);
        Ok(())
    }
}
