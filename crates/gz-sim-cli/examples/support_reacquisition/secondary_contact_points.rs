// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve the native contact rows and angular balance at a measured secondary landing.
use super::super::super::points::Bodies;
use super::*;

#[path = "secondary_cache_cohort.rs"]
pub(crate) mod cohort;
#[path = "secondary_relaxation_cost.rs"]
pub(crate) mod cost;
#[path = "secondary_native_divergence.rs"]
pub(crate) mod divergence;
#[path = "secondary_relaxation.rs"]
pub(crate) mod relaxation;
use gz_physics_multifidelity::{
    BoxContactActions, ContactSubstep, LandingRowOrder, LandingSolverPoint,
};

#[derive(Clone, Copy)]
enum Graph {
    Retained,
    Cold,
    Fresh,
    WarmNoop,
    ColdNoop,
    WarmSorted,
    ColdSorted,
    WarmExchanged,
    ColdExchanged,
}
impl Graph {
    fn name(self) -> &'static str {
        match self {
            Self::Retained => "retained",
            Self::Cold => "cold",
            Self::Fresh => "fresh",
            Self::WarmNoop => "warm_noop",
            Self::ColdNoop => "cold_noop",
            Self::WarmSorted => "warm_sorted",
            Self::ColdSorted => "cold_sorted",
            Self::WarmExchanged => "warm_exchanged",
            Self::ColdExchanged => "cold_exchanged",
        }
    }
    fn cold(self) -> bool {
        matches!(
            self,
            Self::Cold | Self::ColdNoop | Self::ColdSorted | Self::ColdExchanged
        )
    }
    fn order(self) -> Option<LandingRowOrder> {
        match self {
            Self::WarmNoop | Self::ColdNoop => Some(LandingRowOrder::Unchanged),
            Self::WarmSorted | Self::ColdSorted => Some(LandingRowOrder::SoleQuadrants),
            Self::WarmExchanged | Self::ColdExchanged => Some(LandingRowOrder::ExchangePositiveX),
            _ => None,
        }
    }
}
#[derive(Default)]
struct Totals {
    normal: f64,
    quadrant: [f64; 4],
    vector: DVec3,
    tangent: DVec3,
    weighted_foot_m: DVec3,
    weighted_world_m: DVec3,
    torque_normal: DVec3,
    torque_tangent: DVec3,
    positive_points: usize,
}
impl Totals {
    fn add(&mut self, r: &Self) {
        self.normal += r.normal;
        self.vector += r.vector;
        self.tangent += r.tangent;
        self.weighted_foot_m += r.weighted_foot_m;
        self.weighted_world_m += r.weighted_world_m;
        self.torque_normal += r.torque_normal;
        self.torque_tangent += r.torque_tangent;
        self.positive_points += r.positive_points;
        for (a, b) in self.quadrant.iter_mut().zip(r.quadrant) {
            *a += b;
        }
    }
    fn json(&self) -> Value {
        json!({"normal_ns":self.normal,"quadrant_normal_ns":self.quadrant,
       "normal_vector_ns":self.vector.to_array(),"tangent_vector_ns":self.tangent.to_array(),
       "normal_moment_nms":self.torque_normal.to_array(),"tangent_moment_nms":self.torque_tangent.to_array(),
       "normal_weighted_point_local_m":(self.normal>0.).then(||(self.weighted_foot_m/self.normal).to_array()),
       "normal_weighted_point_world_m":(self.normal>0.).then(||(self.weighted_world_m/self.normal).to_array()),
       "positive_points":self.positive_points})
    }
}
fn solver_rows(rows: Vec<LandingSolverPoint>) -> Value {
    json!(rows.into_iter().map(|p|json!({"foot_is_first":p.foot_is_first,"dominance":p.relative_dominance,
       "normal":p.normal_on_foot,"native_gap_m":p.geometric_gap,"solver_gap_m":p.solver_gap_at_update,
       "local_foot":p.local_foot,"local_pad":p.local_pad,"solver_foot_anchor":p.solver_foot_anchor,
       "solver_pad_anchor":p.solver_pad_anchor,"normal_ns":p.impulse})).collect::<Vec<_>>())
}
fn states(native: &MultiFidelityBackend, ids: [BodyId; 3]) -> Result<[BodyState; 3]> {
    let [a, b, c] = ids.map(|id| native.state(id));
    Ok([
        a.ok_or("missing foot")?,
        b.ok_or("missing upper")?,
        c.ok_or("missing support")?,
    ])
}
fn sample_states(s: &ContactSubstep<'_>, ids: [BodyId; 3]) -> Result<[BodyState; 3]> {
    let [a, b, c] = ids.map(|id| s.state(id));
    Ok([
        a.ok_or("missing observed foot")?,
        b.ok_or("missing observed upper")?,
        c.ok_or("missing observed support")?,
    ])
}
struct Interval {
    entry: Bodies,
    previous: [BodyState; 3],
    sums: Totals,
    first: Option<Value>,
    first_after_gap: Option<Value>,
    last_paid: bool,
    peak: Option<Value>,
    peak_load: f64,
    calls: u32,
    positive_calls: u32,
    max_dp: f64,
    max_dh: f64,
    duration: u64,
}
impl Interval {
    fn new(initial: [BodyState; 3]) -> Self {
        Self {
            entry: Bodies::from_states(initial),
            previous: initial,
            sums: Totals::default(),
            first: None,
            first_after_gap: None,
            last_paid: false,
            peak: None,
            peak_load: 0.,
            calls: 0,
            positive_calls: 0,
            max_dp: 0.,
            max_dh: 0.,
            duration: 0,
        }
    }
    fn observe(&mut self, s: ContactSubstep<'_>, ids: [BodyId; 3]) -> Result<()> {
        let [foot, _, pad] = ids;
        let after = sample_states(&s, ids)?;
        let prior = Bodies::from_states(self.previous);
        let next = Bodies::from_states(after);
        let base = DVec3::from(self.previous[0].position);
        let inv = DQuat::from_array(after[0].orientation).conjugate();
        let mut j = Totals::default();
        let mut paid = vec![];
        for patch in s.contact_patches() {
            let sign = if (patch.a, patch.b) == (foot, pad) {
                1.
            } else if (patch.a, patch.b) == (pad, foot) {
                -1.
            } else {
                if patch.points.iter().any(|p| p.normal_impulse != 0.) {
                    return Err("unregistered body received impulse".into());
                }
                continue;
            };
            for p in patch.points {
                let normal = DVec3::from(patch.normal_on_a) * sign;
                let t =
                    DVec3::from(p.tangent_impulse_on_a.ok_or("unknown contact tangent")?) * sign;
                let n = normal * p.normal_impulse;
                let arm = DVec3::from(if sign > 0. { p.lever_a } else { p.lever_b });
                let local = inv * arm;
                let quadrant = 2 * usize::from(local.x > 0.) + usize::from(local.y > 0.);
                let lever = base + arm - prior.com();
                j.weighted_foot_m += local * p.normal_impulse;
                j.weighted_world_m += (base + arm) * p.normal_impulse;
                j.quadrant[quadrant] += p.normal_impulse;
                j.normal += p.normal_impulse;
                j.vector += n;
                j.tangent += t;
                j.torque_normal += lever.cross(n);
                j.torque_tangent += lever.cross(t);
                if p.normal_impulse > 0. {
                    j.positive_points += 1;
                    paid.push(json!({"quadrant":quadrant,"sole_arm_world_m":arm.to_array(),"arm_to_assembly_com_m":lever.to_array(),
                         "normal_vector_ns":n.to_array(),"tangent_vector_ns":t.to_array(),"normal_ns":p.normal_impulse}));
                }
            }
        }
        self.calls += 1;
        self.duration += s.duration.as_nanos() as u64;
        if self.duration != s.elapsed.as_nanos() as u64 {
            return Err("native observer clock mismatch".into());
        }
        if j.normal > 0. {
            self.positive_calls += 1;
            let after_gap =
                self.first.is_some() && !self.last_paid && self.first_after_gap.is_none();
            if self.first.is_none() || after_gap || j.normal > self.peak_load {
                let row = json!({"substep":self.calls,"elapsed_ns":self.duration,"native_dt_ns":s.duration.as_nanos() as u64,
                 "integrals":j.json(),"solver_rows":solver_rows(s.landing_solver_points(foot,pad)?),"paid_points":paid,
                 "before":self.previous.map(Body::from),"after":after.map(Body::from)});
                if self.first.is_none() {
                    self.first = Some(row.clone());
                }
                if after_gap {
                    self.first_after_gap = Some(row.clone());
                }
                if j.normal > self.peak_load {
                    self.peak_load = j.normal;
                    self.peak = Some(row);
                }
            }
        }
        self.last_paid = j.normal > 0.;
        self.max_dp = self.max_dp.max(
            (next.momentum() - prior.momentum()
                + DVec3::Z * (2. * 9.81 * s.duration.as_secs_f64())
                - j.vector
                - j.tangent)
                .length(),
        );
        self.max_dh = self.max_dh.max(
            (next.angular_momentum()
                - prior.angular_momentum()
                - j.torque_normal
                - j.torque_tangent)
                .length(),
        );
        self.previous = after;
        self.sums.add(&j);
        Ok(())
    }
}
fn verify_raw(row: &Value, reported: &Value) -> Result<()> {
    for link in ["foot", "upper", "pad"] {
        for key in ["position", "orientation", "velocity", "angular_velocity"] {
            let a = row[link][key].as_array().ok_or("missing observed body")?;
            let b = reported[link][key]
                .as_array()
                .ok_or("missing archived body")?;
            if a.len() != b.len()
                || a.iter().zip(b).any(|(x, y)| {
                    x.as_f64()
                        .zip(y.as_f64())
                        .is_none_or(|(a, b)| (a - b).abs() > 1e-11)
                })
            {
                return Err(format!("observed branch changed {link} {key}").into());
            }
        }
    }
    if row["tick"] != reported["tick"]
        || row["first_local_ns"] != reported["first_local_ns"]
        || (scalar(&row["normal_impulse_ns"])? - scalar(&reported["normal_impulse_ns"])?).abs()
            > 1e-10
        || vec3(&row["tangent_on_foot_ns"])?.distance(vec3(&reported["tangent_on_foot_ns"])?)
            > 1e-10
    {
        return Err("observer changed native reactions".into());
    }
    Ok(())
}
fn branch(
    archives: (&Path, Option<&Path>),
    ids: [BodyId; 3],
    source: Tick,
    mut native: MultiFidelityBackend,
    grids: (u32, u32),
    graph: Graph,
    f: Fixture,
) -> Result<Value> {
    let [foot, _, pad] = ids;
    let (dir, original) = archives;
    let (grid, prior_grid) = grids;
    let first_solver = solver_rows(native.landing_solver_points(foot, pad)?);
    let baseline = match graph {
        Graph::Retained if grid == prior_grid => Some("uninterrupted".to_string()),
        Graph::Retained => Some("warm_swapped".to_string()),
        Graph::Cold => Some(format!("cold{grid}")),
        _ => None,
    };
    let reference = baseline
        .zip(original)
        .map(|(tag, path)| load(&path.join(format!("traces/{tag}.json"))))
        .transpose()?;
    let mut history = vec![];
    let mut traces = vec![];
    let mut initial = source.as_json();
    initial["corner_ns"] = json!([0., 0., 0., 0.]);
    traces.push(initial);
    let mut speed = source.pad.velocity[2];
    for tick in source.tick + 1..=source.tick + 10 {
        let next = command(f, tick - 1);
        if next != speed {
            native.set_world_velocity(pad, [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let mut observed = Interval::new(states(&native, ids)?);
        let mut err = None;
        let mut accept = |s: ContactSubstep<'_>| {
            if err.is_none() {
                err = observed.observe(s, ids).err();
            }
        };
        let pair = [(foot, pad)];
        let fresh = matches!(graph, Graph::Fresh) && tick == source.tick + 1;
        let iterations = ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: Some(1),
        };
        let mut hooks = None;
        let interval = if let Some(order) = graph.order().filter(|_| tick == source.tick + 1) {
            let (interval, counts) = native.observe_landing_row_order(
                (foot, pad),
                DT,
                grid,
                iterations,
                order,
                &mut accept,
            )?;
            hooks = Some(
                json!({"nonempty_manifolds_visited":counts.nonempty_manifolds_visited,
                                 "ordered_manifolds_changed":counts.ordered_manifolds_changed}),
            );
            interval
        } else {
            native
                .advance_box_contacts(
                    DT,
                    grid,
                    iterations,
                    BoxContactActions {
                        fresh_features: if fresh { &pair } else { &[] },
                        ..Default::default()
                    },
                    Some(&mut accept),
                )?
                .interval
        };
        if let Some(e) = err {
            return Err(e);
        }
        if observed.calls != grid || observed.duration != CLOCK_NS {
            return Err("native observation clock or budget mismatch".into());
        }
        let end = states(&native, ids)?;
        let body = Bodies::from_states(end);
        let summed =
            body.momentum() - observed.entry.momentum() + DVec3::Z * (2. * 9.81 * DT.as_secs_f64());
        let dp = (summed - observed.sums.vector - observed.sums.tangent).length();
        let dh = (body.angular_momentum()
            - observed.entry.angular_momentum()
            - observed.sums.torque_normal
            - observed.sums.torque_tangent)
            .length();
        let mut first = None;
        let mut normal = 0.;
        let mut tangent = DVec3::ZERO;
        let mut normal_vec = DVec3::ZERO;
        for p in interval.pairs {
            if (p.a, p.b) != (foot.min(pad), foot.max(pad)) {
                if p.normal_impulse > 0. {
                    return Err("unexpected extra pair".into());
                }
                continue;
            }
            let sign = if foot == p.a { 1. } else { -1. };
            first = p.first_positive_end;
            normal += p.normal_impulse;
            tangent += sign
                * DVec3::from(
                    p.tangent_impulse_on_a
                        .ok_or("missing native interval tangent")?,
                );
            normal_vec += sign * DVec3::from(p.normal_impulse_on_a);
        }
        if (normal - observed.sums.normal).abs() > 1e-10
            || normal_vec.distance(observed.sums.vector) > 1e-10
            || tangent.distance(observed.sums.tangent) > 1e-10
            || first.map(|h| h.as_nanos() as u64)
                != observed
                    .first
                    .as_ref()
                    .and_then(|r| r["elapsed_ns"].as_u64())
        {
            return Err("per-point sums disagree with public native pair receipt".into());
        }
        let row = Tick {
            tick,
            foot: end[0].into(),
            upper: end[1].into(),
            pad: end[2].into(),
            normal,
            tangent,
            extra: 0.,
            calls: grid,
            first,
            gap: supported_gap(end[0], end[2]),
        };
        let mut frame = row.as_json();
        frame["corner_ns"] = json!(observed.sums.quadrant);
        traces.push(frame.clone());
        if let Some(r) = &reference {
            verify_raw(&frame, &r["frames"][(tick - source.tick) as usize])?;
        }
        history.push(json!({"tick":tick,"native_calls":observed.calls,"positive_native_calls":observed.positive_calls,"fresh_features":fresh,"row_hook":hooks,
          "totals":observed.sums.json(),"independent_delta_p_minus_gravity_ns":summed.to_array(),
          "dp_residual_ns":dp,"dh_residual_nms":dh,"max_native_dp_ns":observed.max_dp,"max_native_dh_nms":observed.max_dh,
          "first_positive":observed.first,"first_after_unloaded_native_call":observed.first_after_gap,"peak":observed.peak}));
    }
    let label = format!("{}{grid}", graph.name());
    write_json(
        &dir.join(format!("traces/{label}.json")),
        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":traces}),
    )?;
    Ok(
        json!({"graph":graph.name(),"grid":grid,"initial_rows":first_solver,"original_trace_validated":reference.is_some(),"public":history}),
    )
}
pub(crate) fn run(dir: &Path, index: usize) -> Result<()> {
    let root = dir
        .parent()
        .ok_or("output must be a sibling of the native sources")?;
    let source = root.join(format!(
        "multifidelity-secondary-restart-conditioned/case-{index:03}"
    ));
    let check = load(&source.join("restart.json"))?;
    let near = check["near_cut"].as_u64().ok_or("missing cut")?;
    let f = fixture(index);
    let (cuts, _) = survey::cuts(f)?;
    let cut = cuts[0].ok_or("no primary donor")?;
    if check["format"] != "matched-secondary-restart-v2"
        || check["case"] != index
        || check["primary_tick"] != cut.tick
    {
        return Err("unmatched previous restart study".into());
    }
    let case = dir.join(format!("case-{index:03}"));
    let mut all = vec![];
    for (profile, scope) in SOURCES {
        let folder = case.join(profile.label);
        fs::create_dir_all(folder.join("traces"))?;
        let old = source.join(profile.label).join(near.to_string());
        let audit = load(&old.join("audit.json"))?;
        let mut branches = vec![];
        observe_path_until(f, &cut, profile, scope, near, |native, ids, row| {
            if row.tick == near {
                let [foot, _, pad] = ids;
                let s = native.airborne_restart_state(foot, pad)?;
                if s.links.map(Body::from) != [row.foot, row.upper, row.pad] {
                    return Err("incoming donor changed".into());
                }
                let archived = load(&old.join("traces/uninterrupted.json"))?;
                verify_raw(&row.as_json(), &archived["frames"][0])?;
                for n in [512, 4096] {
                    for graph in [Graph::Retained, Graph::Cold, Graph::Fresh] {
                        let fork = native.copy_airborne_landing(
                            foot,
                            pad,
                            matches!(graph, Graph::Cold),
                        )?;
                        branches.push(branch(
                            (&folder, Some(&old)),
                            ids,
                            *row,
                            fork,
                            (n, profile.suffix.n),
                            graph,
                            f,
                        )?);
                    }
                }
            }
            Ok(())
        })?;
        let lanes=[("retained4096","Retained contact geometry"),("cold4096","Rebuilt collision graph"),("fresh4096","Fresh sole geometry on the landing tick")]
          .map(|(label,name)|json!({"label":label,"name":name,"file":format!("traces/{label}.json")}));
        write_json(
            &folder.join("comparison.json"),
            &json!({"lanes":lanes,"title":"Measured native point reactions at the second landing",
         "capture_ticks":{"second_impact":near+1,"after_five_ms":near+10}}),
        )?;
        let record =
            json!({"incoming":profile.label,"cut":audit["snapshot"].clone(),"branches":branches});
        write_json(&folder.join("points.json"), &record)?;
        all.push(record);
    }
    write_json(
        &case.join("points.json"),
        &json!({"format":"secondary-landing-point-reactions-v1","case":index,"near_tick":near,"donors":all}),
    )
}
