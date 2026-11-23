// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched step controls before the first certified geometric collision.
use super::*;
use gz_physics::BodyState;
#[derive(Clone, Copy, Serialize, PartialEq, Debug)]
#[serde(rename_all = "snake_case")]
enum Frame {
    World,
    PadRest,
    FixedPad,
    Centered,
    Reversed,
}
#[derive(Clone, Copy, Serialize)]
struct Work {
    calls: u32,
    temporal: usize,
    relax: usize,
    fresh: bool,
}
fn configuration(id: usize, tree: bool, frame: Frame) -> (Physical, String) {
    let mut r = Physical::new(id);
    if matches!(frame, Frame::PadRest | Frame::FixedPad) {
        let boost = -r.pad_v;
        r.pre.v += boost;
        r.post.v += boost;
        r.at += boost * r.arrival;
        r.pad_v = DVec3::ZERO;
    }
    let mut xml = scene::xml(&r, tree, PROFILES[3]);
    if frame == Frame::FixedPad {
        let i = xml
            .find(r#"<plugin name="gz::sim::systems::PrescribedMotion""#)
            .unwrap();
        let j = i + xml[i..].find("</plugin>").unwrap() + 9;
        xml.replace_range(i..j, "<static>true</static>");
    }
    if frame == Frame::Centered {
        assert!(!tree && r.pre.l.length() < 1e-12);
        xml = xml.replace("<inertial><pose>0 0 0.07 ", "<inertial><pose>0 0 0 ");
    }
    if frame == Frame::Reversed {
        let i = xml.find("<model ").unwrap();
        let j = i + xml[i..].find("</model>").unwrap() + 8;
        let k = xml.find("</world>").unwrap();
        let m = format!("{}{}", &xml[j..k], &xml[i..j]);
        xml.replace_range(i..k, &m);
    }
    (r, xml)
}
fn gap(foot: BodyState, pad: BodyState) -> f64 {
    let n = DQuat::from_array(pad.orientation) * DVec3::Z;
    let delta = DVec3::from(foot.position) - DVec3::from(pad.position);
    let q = DQuat::from_array(foot.orientation);
    corners()
        .iter()
        .map(|r| (delta + q * *r).dot(n) - PAD_HALF[2])
        .fold(f64::INFINITY, f64::min)
}
fn probe(id: usize, tree: bool, frame: Frame, w: Work) -> Result<Value> {
    let (r, xml) = configuration(id, tree, frame);
    let (mut sim, ids) = sdf_features::load_finite_box(&xml, tree)?;
    let mut rb = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = rb.downcast_mut::<MultiFidelityBackend>().unwrap();
    let pair = [(ids.foot, ids.pad)];
    let pairs = if w.fresh { &pair[..] } else { &[][..] };

    let exact = (r.arrival * 1e9).round() as u64;
    let mut first = None;
    let mut pre = 0.;
    let mut all = 0.;
    let mut previous = vec![];
    let mut first_rows = vec![];
    let mut first_gap = [0.; 2];
    let mut ref_gap = [0.; 2];
    for k in 0..8 * w.calls {
        let start = u64::from(k) * 500000 / u64::from(w.calls);
        let end = u64::from(k + 1) * 500000 / u64::from(w.calls);
        let d = end - start;
        let g = gap(e.state(ids.foot).unwrap(), e.state(ids.pad).unwrap());
        let (receipt, _) = e.step_contact_interval_with_box_features(
            Duration::from_nanos(d),
            1,
            ContactSolverIterations {
                temporal: w.temporal,
                pgs: Some(1),
                relaxation: Some(w.relax),
            },
            pairs,
        )?;
        let j = receipt.pairs.iter().map(|x| x.normal_impulse).sum::<f64>();
        all += j;
        if end < exact {
            pre += j;
        }
        if first.is_none() {
            let rows=e.landing_solver_points(ids.foot,ids.pad)?.into_iter().map(|p|json!({"geometric_gap_m":p.geometric_gap,"solver_gap_at_update_m":p.solver_gap_at_update,"local_foot":p.local_foot,"local_pad":p.local_pad,"solver_foot_anchor":p.solver_foot_anchor,"solver_pad_anchor":p.solver_pad_anchor,"relative_dominance":p.relative_dominance,"foot_first":p.foot_is_first,"normal_on_foot":p.normal_on_foot,"impulse_ns":p.impulse})).collect();
            if j > 1e-12 {
                first = Some(end);
                first_gap = [
                    g,
                    gap(e.state(ids.foot).unwrap(), e.state(ids.pad).unwrap()),
                ];
                let analytical_gap = |time: u64| {
                    let s = r.state(time as f64 * 1e-9);
                    let n = r.n;
                    corners()
                        .into_iter()
                        .map(|p| {
                            (DVec3::from(s.foot.position)
                                + DQuat::from_array(s.foot.orientation) * p
                                - DVec3::from(s.pad.position))
                            .dot(n)
                                - PAD_HALF[2]
                        })
                        .fold(f64::INFINITY, f64::min)
                };
                ref_gap = [analytical_gap(start), analytical_gap(end)];
                first_rows = rows;
            } else {
                previous = rows;
            }
        }
    }
    Ok(
        json!({"id":id,"locked":tree,"frame":frame,"work":w,"analytic_arrival_ns":exact,"analytic_impulse_ns":r.j,"first_positive_end_ns":first,"first_interval_native_geometry_m":first_gap,"first_interval_Euler_geometry_m":ref_gap,"pre_arrival_ns":pre,"total_normal_ns":all,"rows_before":previous,"rows_at_first_reaction":first_rows}),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for id in [10, 16, 57, 85, 125] {
        for tree in [false, true] {
            let mut frames = vec![
                Frame::World,
                Frame::PadRest,
                Frame::FixedPad,
                Frame::Reversed,
            ];
            if id == 125 && !tree {
                frames.push(Frame::Centered);
            }
            for frame in frames {
                for calls in [1, 4, 8, 16, 64] {
                    for temporal in [1, 4] {
                        for relax in [0, 1, 16] {
                            for fresh in [false, true] {
                                rows.push(probe(
                                    id,
                                    tree,
                                    frame,
                                    Work {
                                        calls,
                                        temporal,
                                        relax,
                                        fresh,
                                    },
                                )?);
                            }
                        }
                    }
                }
            }
        }
    }
    write_json(&path.join("early.json"), &json!({"cases":rows}))
}
