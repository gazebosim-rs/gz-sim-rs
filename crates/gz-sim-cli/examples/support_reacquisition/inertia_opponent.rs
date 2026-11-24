// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A driven ceiling physically crosses the opposite side of the free sole.
use super::*;
#[path = "inertia_upper.rs"]
pub(crate) mod upper;

fn text(i: &Impact, g: &Gate, named: bool) -> String {
    let raw = insert(&policy(i, true), &g.xml);
    if !named {
        return raw;
    }
    raw.replace(
        "</project_rigid_corner>",
        "</project_rigid_corner><guard_opponent>default::opponent::gate</guard_opponent>",
    )
}

#[derive(Clone, Copy)]
struct Flight {
    com: DVec3,
    v: DVec3,
    q: DQuat,
    l: DVec3,
}
impl Flight {
    fn from_state(r: &Physical, state: BodyState) -> Self {
        let q = DQuat::from_array(state.orientation);
        Self {
            com: DVec3::from(state.position) + q * ARM,
            q,
            v: state.velocity.into(),
            l: r.body.momentum(q, state.angular_velocity.into()),
        }
    }
    fn advance(&mut self, r: &Physical, t: f64) {
        self.com += t * (self.v + 0.5 * t * r.gravity);
        self.v += t * r.gravity;
        self.q = advance_q(r.body, self.q, self.l, t);
    }
    fn face_gap(self, gate: BodyState, normal: DVec3, half: f64, t: f64) -> f64 {
        let origin = self.com - self.q * ARM;
        let center = DVec3::from(gate.position) + DVec3::from(gate.velocity) * t;
        corners()
            .into_iter()
            .flat_map(|p| [p, DVec3::new(p.x, p.y, -p.z)])
            .map(|p| (origin + self.q * p - center).dot(normal) - half)
            .fold(f64::INFINITY, f64::min)
    }
}

pub(super) fn inspect(
    i: &Impact,
    base: u64,
    lo: u64,
    s: &Observed,
    foot: BodyState,
    gate: BodyState,
    cert: Option<gz_physics_multifidelity::OpponentClearance>,
) -> Value {
    let h = (s.end - lo) as f64 * 1e-9;
    let detail = cert.map(|cert| {
        // Decode the selected plane, then independently enumerate real sole vertices.
        // Evolve the measured native incoming state using constant world angular momentum
        // and RK4. A measured primary impulse changes both momenta at its physical time.
        let axis = (cert.face/2) as usize;
        let signed = if cert.face%2==0 {1.} else {-1.};
        let normal = DQuat::from_array(gate.orientation) * [DVec3::X,DVec3::Y,DVec3::Z][axis] * signed;
        let mut flight = Flight::from_state(&i.r,foot);
        let mut minimum = flight.face_gap(gate,normal,HALF_GATE[axis],0.);
        let mut times:Vec<f64> = (1..=64).map(|n|h*n as f64/64.).collect();
        if let Some(p) = s.projection {
            assert_eq!(p.pulse_start.as_nanos() as u64,lo);
            assert!(p.after_cut_s>=0. && p.after_cut_s<=h);
            times.push(p.after_cut_s);
        }
        times.sort_by(f64::total_cmp);
        times.dedup();
        let mut last=0.;
        for t in times {
            flight.advance(&i.r,t-last);
            minimum=minimum.min(flight.face_gap(gate,normal,HALF_GATE[axis],t));
            if let Some(p)=s.projection && p.after_cut_s==t {
                let j=DVec3::from(p.impact.impulse_on_foot);
                flight.v+=j/M;
                flight.l+=(DVec3::from(p.impact.point)-flight.com).cross(j);
            }
            last=t;
        }
        let end=Flight::from_state(&i.r,s.foot);
        let certified=cert.minimum_gap_pm as f64*1e-12;
        assert!(minimum > 0. && minimum+1e-11 >= certified, "gap {minimum} bound {certified}");
        assert_eq!(s.opponent,0., "guard leaked opponent impulse");
        if s.projection.is_none(){assert_eq!(s.corners.iter().sum::<f64>(),0.);}
        json!({"face":cert.face,"certified_minimum_gap_m":certified,"independent_sampled_minimum_gap_m":minimum,"free_endpoint_com_m":flight.com.distance(end.com),"free_endpoint_attitude_rad":angular_distance(flight.q,end.q)})
    });
    let net = M * (DVec3::from(s.foot.velocity) - DVec3::from(foot.velocity) - i.r.gravity * h);
    json!({"start_ns":base+lo,"end_ns":base+s.end,"net_foot_impulse":net.to_array(),"upper_projection":s.upper.map(|r|upper::projected::hit_record(r,base,foot,s.returned.map_or(s.foot,|q|q.branch_start))),"return_projection":s.returned.map(|r|upper::projected::returns::record(r,base,s.foot)),"primary_ns":s.corners.iter().sum::<f64>(),"opponent_ns":s.opponent,"projected":s.projection.is_some(),"certificate":detail})
}

// Reconstruct the first physical ceiling contact from the loaded SDF gate,
// independently of the requested placement delay and the native contact rows.
fn finite_ceiling(i: &Impact, frame: &Value, requested: f64) -> Result<Value> {
    let position = DVec3::from(serde_json::from_value::<[f64; 3]>(
        frame["opponent"]["position"].clone(),
    )?);
    let velocity = DVec3::from(serde_json::from_value::<[f64; 3]>(
        frame["opponent"]["velocity"].clone(),
    )?);
    let gate = DQuat::from_array(serde_json::from_value(
        frame["opponent"]["orientation"].clone(),
    )?);
    let normal = gate * DVec3::Z;
    let r = &i.r;
    let points: Vec<DVec3> = corners()
        .into_iter()
        .flat_map(|p| [p, DVec3::new(p.x, p.y, -p.z)])
        .collect();
    let radius = points
        .iter()
        .map(|p| (*p - ARM).length())
        .fold(0., f64::max);
    let speed_bound = [r.pre, r.post]
        .into_iter()
        .map(|b| {
            let w = (b.l.dot(r.body.omega(r.q, b.l)) / r.body.lower).sqrt();
            (b.v - r.pad_v).length() + r.gravity.length() * WINDOW + radius * w
        })
        .fold(0., f64::max);
    // All vertices approach the gate monotonically on both conservative
    // pre/post branches. The loaded gate plane stays parallel to the pad.
    assert!(normal.distance(r.n) < 1e-12);
    let closing = -(velocity - r.pad_v).dot(normal) - speed_bound;
    assert!(closing > 0.);
    let gap = |t| {
        let s = r.state(t);
        let q = DQuat::from_array(s.foot.orientation);
        let center = position + t * velocity;
        let foot = DVec3::from(s.foot.position);
        let mut nearest = (f64::INFINITY, 0., 0);
        for (index, p) in points.iter().enumerate() {
            let local = gate.conjugate() * (foot + q * (*p) - center);
            let g = -HALF_GATE[2] - local.z;
            if g < nearest.0 {
                nearest = (
                    g,
                    (HALF_GATE[0] - local.x.abs()).min(HALF_GATE[1] - local.y.abs()),
                    index,
                );
            }
        }
        nearest
    };
    let (mut lo, mut hi) = (0., WINDOW);
    assert!(gap(lo).0 > 0. && gap(hi).0 < 0.);
    for _ in 0..42 {
        let mid = 0.5 * (lo + hi);
        if gap(mid).0 > 0. {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let at = (lo + hi) * 0.5;
    let closest = gap(at);
    assert!(closest.1 > 0. && (at - requested).abs() < 1e-12);
    Ok(
        json!({"time_ns":at*1e9,"bracket_width_ns":(hi-lo)*1e9,"finite_face_margin_m":closest.1,"corner":closest.2,"gap_at_root_m":closest.0,"closing_lower_m_s":closing}),
    )
}

fn inputs() -> Vec<Impact> {
    let mut input = vec![manufactured(1160, 3), manufactured(1353, 3)];
    // Both split ranges contribute sticking, sliding and near-transition controls.
    for (n, i) in cases().into_iter().enumerate() {
        if n % 4 == 0 || n % 4 == 3 {
            input.push(i);
        }
    }
    input
}
const DELAYS: [i64; 9] = [-180000, -20000, -8, 0, 8, 500, 20000, 750000, 1100000];

pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut all = vec![];
    for i in inputs() {
        let mut cases = vec![];
        for delay in DELAYS {
            let g = gate(&i, delay as f64 * 1e-9);
            let mut lanes = vec![];
            let mut ceiling = None;
            for named in [false, true] {
                let xml = text(&i, &g, named);
                let (run, frames) = measure_gate(&i, &xml, true)?;
                if !named {
                    ceiling = Some(finite_ceiling(&i, &frames[0], g.at)?);
                }
                if i.r.id == 1160 && i.mode == 3 && delay == 750000 {
                    let name = if named { "named" } else { "ordinary_neighbor" };
                    fs::write(path.join(format!("{name}.sdf")), &xml)?;
                    write_json(
                        &path.join("traces").join(format!("{name}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"pad_half_extents":PAD_HALF,"foot_com_local":ARM.to_array(),"opponent_half_extents":HALF_GATE},"frames":frames}),
                    )?;
                }
                lanes.push(json!({"named":named,"run":run}));
            }
            cases.push(
                json!({"delay_ns":delay,"gate_ns":g.at*1e9,"physical_gate":ceiling,"gate_speed_m_s":g.speed,"lanes":lanes}),
            );
        }
        all.push(json!({"id":i.r.id,"mode":MODES[i.mode],"reference":law(&i),"primary_ns":i.r.arrival*1e9,"cases":cases}));
    }
    write_json(&path.join("study.json"), &json!({"controls":all}))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Returning a named moving ceiling to native contact","capture_ticks":{"primary_landing":5,"first_ceiling_contact":6},"lanes":[{"label":"ordinary_neighbor","name":"Native speculative ceiling","file":"traces/ordinary_neighbor.json"},{"label":"named","name":"Measured two-pair clearance","file":"traces/named.json"}]}),
    )
}

pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for (index, i) in inputs().into_iter().enumerate() {
        for delay in [0, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            let mut lanes = vec![];
            for named in [false, true] {
                let xml = text(&i, &g, named);
                let (mut sim, ids) = load(&xml, false)?;
                sim.step_exact(8)?;
                let end = state(backend(&sim), ids)?;
                costed(&xml, end)?;
                lanes.push((xml, end));
            }
            for rep in 0..6 {
                for a in 0..2 {
                    let k = (a + index + rep) % 2;
                    let s = costed(&lanes[k].0, lanes[k].1)?;
                    rows.push(json!({"id":i.r.id,"mode":MODES[i.mode],"delay_ns":delay,"named":k==1,"primary_tick":(i.r.arrival/0.0005).ceil()as u64,"gate_tick":(g.at/0.0005).ceil()as u64,"repeat":rep,"wall_ns":s.wall,"cpu_ns":s.cpu}));
                }
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}
