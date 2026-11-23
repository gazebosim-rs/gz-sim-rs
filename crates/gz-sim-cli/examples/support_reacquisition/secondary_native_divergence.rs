// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Locate the first material change of a matched second landing inside its public interval.
use super::*;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

struct Step {
    start: [BodyState; 3],
    end: [BodyState; 3],
    ns: u64,
    rows: Vec<LandingSolverPoint>,
    normal: DVec3,
    tangent: DVec3,
    load: f64,
    quadrant: [f64; 4],
}
impl Step {
    fn impulse(&self) -> DVec3 {
        self.normal + self.tangent
    }
    fn detail(&self) -> Value {
        let raw=self.rows.iter().map(|p|json!({"foot_first":p.foot_is_first,
            "foot_m":p.local_foot,"pad_m":p.local_pad,"normal_on_foot":p.normal_on_foot,
            "observed_gap_m":p.geometric_gap,"solver_gap_m":p.solver_gap_at_update,
            "solver_foot_anchor":p.solver_foot_anchor,"solver_pad_anchor":p.solver_pad_anchor,"normal_ns":p.impulse})).collect::<Vec<_>>();
        json!({"ns":self.ns,"normal_impulse_ns":self.load,"normal_vector_ns":self.normal.to_array(),"tangent_vector_ns":self.tangent.to_array(),
               "quadrant_normal_ns":self.quadrant,"before":self.start.map(Body::from),"after":self.end.map(Body::from),"solver_rows":raw})
    }
    fn from_native(
        s: &ContactSubstep<'_>,
        before: [BodyState; 3],
        ids: [BodyId; 3],
    ) -> Result<Self> {
        let [foot, _, pad] = ids;
        let end = sample_states(s, ids)?;
        let mut row = Self {
            start: before,
            end,
            ns: s.elapsed.as_nanos() as u64,
            rows: s.landing_solver_points(foot, pad)?,
            normal: DVec3::ZERO,
            tangent: DVec3::ZERO,
            load: 0.,
            quadrant: [0.; 4],
        };
        let rotation = DQuat::from_array(end[0].orientation).conjugate();
        for patch in s.contact_patches() {
            let sign = if (patch.a, patch.b) == (foot, pad) {
                1.
            } else if (patch.a, patch.b) == (pad, foot) {
                -1.
            } else {
                if patch.points.iter().any(|p| p.normal_impulse > 0.) {
                    return Err("unexpected opponent on native return".into());
                }
                continue;
            };
            for p in patch.points {
                row.load += p.normal_impulse;
                row.normal += DVec3::from(patch.normal_on_a) * (p.normal_impulse * sign);
                row.tangent +=
                    DVec3::from(p.tangent_impulse_on_a.ok_or("missing native friction")?) * sign;
                let local = rotation * DVec3::from(if sign > 0. { p.lever_a } else { p.lever_b });
                row.quadrant[2 * usize::from(local.x > 0.) + usize::from(local.y > 0.)] +=
                    p.normal_impulse;
            }
        }
        Ok(row)
    }
}
fn run_branch(
    mut native: MultiFidelityBackend,
    ids: [BodyId; 3],
    f: Fixture,
    incoming: &Tick,
    order: LandingRowOrder,
    grid: u32,
) -> Result<Vec<Step>> {
    let [foot, _, pad] = ids;
    let vz = command(f, incoming.tick);
    if vz != incoming.pad.velocity[2] {
        native.set_world_velocity(pad, [0., 0., vz], [0.; 3])?;
    }
    let mut before = states(&native, ids)?;
    let mut steps = Vec::with_capacity(grid as usize);
    let mut error = None;
    let (receipt, activity) = native.observe_landing_row_order(
        (foot, pad),
        DT,
        grid,
        ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: Some(1),
        },
        order,
        |s| {
            if error.is_none() {
                match Step::from_native(&s, before, ids) {
                    Ok(row) => {
                        before = row.end;
                        steps.push(row);
                    }
                    Err(e) => error = Some(e),
                }
            }
        },
    )?;
    if let Some(error) = error {
        return Err(error);
    }
    if activity.nonempty_manifolds_visited == 0
        || steps.len() != grid as usize
        || steps.last().is_none_or(|r| r.ns != CLOCK_NS)
    {
        return Err("missing committed native observations".into());
    }
    let n = receipt.pairs.iter().map(|p| p.normal_impulse).sum::<f64>();
    if (n - steps.iter().map(|r| r.load).sum::<f64>()).abs() > 1e-9 {
        return Err("native solver points disagree with independently integrated receipt".into());
    }
    Ok(steps)
}
fn total(x: &Value) -> Result<DVec3> {
    Ok(vec3(&x["normal_vector_ns"])? + vec3(&x["tangent_vector_ns"])?)
}
fn difference(a: &Step, b: &Step) -> Value {
    let dv = |states: [BodyState; 3]| {
        let p = Bodies::from_states(states);
        (p.momentum(), p.angular_momentum())
    };
    let (pw, hw) = dv(a.start);
    let (pc, hc) = dv(b.start);
    json!({"retained":a.detail(),"rebuilt":b.detail(),"delta_before_linear_momentum_ns":(pc-pw).to_array(),
           "delta_before_angular_momentum_nms":(hc-hw).to_array(),"delta_j_ns":(b.impulse()-a.impulse()).to_array()})
}
pub(crate) fn run(out: &Path, index: usize) -> Result<()> {
    let root = out.parent().ok_or("expected siblings")?;
    let src = load(&root.join(format!(
        "multifidelity-secondary-order-field/cases/case-{index:03}.json"
    )))?;
    if src["status"] != "measured" {
        return Err("source lacks matched second landing".into());
    }
    let tick = src["incoming"]["cut_tick"]
        .as_u64()
        .ok_or("missing matched cut")?;
    let f = fixture(index);
    let (cuts, _) = survey::cuts(f)?;
    let primary = cuts[0].ok_or("no original primary cut")?;
    let dir = out.join(format!("case-{index:03}"));
    fs::create_dir_all(dir.join("traces"))?;
    observe_path_until(
        f,
        &primary,
        PATHS[6],
        ProjectionSpan::Native,
        tick,
        |parent, ids, sample| {
            if sample.tick != tick {
                return Ok(());
            }
            let [foot, _, pad] = ids;
            let state = parent.airborne_restart_state(foot, pad)?;
            if motion(&state) != src["incoming"]["motion"]
                || cache(&state) != src["incoming"]["cache"]
            {
                return Err("incident native state changed".into());
            }
            let mut controls = Vec::new();
            for (graph, cold, order) in [
                ("warm_sorted", false, LandingRowOrder::SoleQuadrants),
                ("cold_sorted", true, LandingRowOrder::SoleQuadrants),
                ("cold_exchanged", true, LandingRowOrder::ExchangePositiveX),
            ] {
                let steps = run_branch(
                    parent.copy_airborne_landing(foot, pad, cold)?,
                    ids,
                    f,
                    sample,
                    order,
                    4096,
                )?;
                let expected = src["branches"]
                    .as_array()
                    .ok_or("missing branches")?
                    .iter()
                    .find(|r| r["graph"] == graph)
                    .ok_or("source treatment missing")?;
                let j = steps.iter().map(Step::impulse).sum::<DVec3>();
                if j.distance(total(&expected["first_public"])?) > 1e-9 {
                    return Err(
                        "native trace failed to reproduce field's integrated impulse".into(),
                    );
                }
                controls.push(steps);
            }
            let (a, b) = (&controls[0], &controls[1]);
            let mut jw = DVec3::ZERO;
            let mut jc = DVec3::ZERO;
            let mut marks = BTreeMap::<String, usize>::new();
            let mut cumulative = Vec::new();
            let mut max_incident = 0.;
            for (i, (warm, cold)) in a.iter().zip(b).enumerate() {
                if warm.ns != cold.ns || controls[2][i].ns != warm.ns {
                    return Err("unequal common grid".into());
                }
                let dj = cold.impulse() - warm.impulse();
                jw += warm.impulse();
                jc += cold.impulse();
                if warm.load > 0. {
                    marks.entry("first_retained".into()).or_insert(i);
                }
                if cold.load > 0. {
                    marks.entry("first_rebuilt".into()).or_insert(i);
                }
                for (j, n) in [
                    (1e-9, "first_nano_ns"),
                    (1e-6, "first_micro_ns"),
                    (1e-3, "first_milli_ns"),
                    (1e-2, "first_centins"),
                ] {
                    if jw.distance(jc) >= j {
                        marks.entry(n.into()).or_insert(i);
                    }
                }
                if dj.length() > max_incident {
                    max_incident = dj.length();
                    marks.insert("peak_single_solve".into(), i);
                }
                cumulative.push(json!([
                    warm.ns,
                    jw.to_array(),
                    jc.to_array(),
                    warm.impulse().to_array(),
                    cold.impulse().to_array()
                ]));
            }
            let mut details = serde_json::Map::new();
            let mut sample_at = BTreeSet::from([0usize, 4095usize]);
            for (name, &i) in &marks {
                let mut v = difference(&a[i], &b[i]);
                v["substep"] = json!(i + 1);
                v["previous"] = i
                    .checked_sub(1)
                    .map_or(Value::Null, |j| difference(&a[j], &b[j]));
                details.insert(name.clone(), v);
                for j in i.saturating_sub(4)..=(i + 6).min(4095) {
                    sample_at.insert(j);
                }
            }
            for j in (0..4096).step_by(128) {
                sample_at.insert(j);
            }
            for (label, steps) in ["retained", "reconstructed", "exchanged"]
                .into_iter()
                .zip(&controls)
            {
                let mut frames = vec![
                    json!({"tick":0,"time_ns":0,"time_us":0,"foot":sample.foot,"upper":sample.upper,"pad":sample.pad,"native_steps":0,"normal_impulse_ns":0.,"corner_ns":[0.,0.,0.,0.]}),
                ];
                for &i in &sample_at {
                    let r = &steps[i];
                    let [foot, upper, pad] = r.end.map(Body::from);
                    frames.push(json!({"tick":i+1,"time_ns":r.ns,"time_us":r.ns/1000,"foot":foot,"upper":upper,"pad":pad,"native_steps":1,"normal_impulse_ns":r.load,"corner_ns":r.quadrant}));
                }
                write_json(
                    &dir.join(format!("traces/{label}.json")),
                    &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
                )?;
            }
            let lanes=[("retained","Retained, ordered sole rows"),("reconstructed","Rebuilt, ordered sole rows"),("exchanged","Rebuilt, exchanged positive-x rows")].map(|(label,name)|json!({"label":label,"name":name,"file":format!("traces/{label}.json")}));
            write_json(
                &dir.join("comparison.json"),
                &json!({"title":"Inside the first differing secondary landing solve","lanes":lanes,
            "native_samples":true,"sampled_ticks":true,"public_dt_ns":CLOCK_NS,"capture_ticks":marks.iter().map(|(k,v)|(k.clone(),v+1)).collect::<BTreeMap<_,_>>()}),
            )?;
            write_json(
                &dir.join("steps.json"),
                &json!({"case":index,"cut":src["incoming"].clone(),"first_public_ns":CLOCK_NS,
             "native_calls":4096,"elapsed_ns_and_world_impulses":cumulative,"events":details,
             "columns":["elapsed_ns","retained_cumulative_ns","rebuilt_cumulative_ns","retained_increment_ns","rebuilt_increment_ns"]}),
            )
        },
    )?;
    Ok(())
}
