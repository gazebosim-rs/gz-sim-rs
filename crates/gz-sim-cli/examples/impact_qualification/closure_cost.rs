// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rebuild the local orthonormal two-port mobility from measured SI geometry.
//! Verify that a bounded pressure search excludes actual single-ended solutions.
use super::transfer::{fixture, matched, vector};
use super::*;
use gz_physics::{EdgeMobility, LineClosure, exclude_closed_line};
fn input(r: &Value) -> Result<(EdgeMobility, [[f64; 3]; 2])> {
    let s = &r["segment"];
    let p = [vector(&s["points_m"][0])?, vector(&s["points_m"][1])?];
    let n = vector(&s["normal"])?;
    let x = (p[1] - p[0]).reject_from_normalized(n).normalize();
    let axes = [x, n.cross(x), n];
    let c = vector(&r["arc_end"]["com_m"])?;
    let arms = p.map(|q| q - c);
    let w: EdgeMobility = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            std::array::from_fn(|a| {
                std::array::from_fn(|b| {
                    axes[a].dot(axes[b] / 2. + arms[j].cross(axes[b]).cross(arms[i]) / 0.05)
                })
            })
        })
    });
    let v = vector(&r["arc_end"]["velocity_m_s"])?;
    let omega = vector(&r["arc_end"]["spin_rad_s"])?;
    Ok((
        w,
        arms.map(|r| axes.map(|axis| axis.dot(v + omega.cross(r)))),
    ))
}
pub(crate) fn summarize(registered: &Path, old: &Path, out: &Path) -> Result<()> {
    let mut groups = BTreeMap::<(String, String), Series>::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut rows = vec![];
    for k in 0..1250 {
        let r = fixture(registered, k)?;
        let baseline = fixture(old, k)?;
        for t in array(&r["result"]["trials"])? {
            let field = &t["response"];
            if field["segment"].is_null() {
                continue;
            }
            let d = &matched(&baseline, t)?["response"];
            if d["arc_end"] != field["arc_end"] {
                return Err("different incoming line".into());
            }
            let h = &d["pressure"];
            let basic = !h["pressure_bound"].is_null();
            let key = k * 3 + uint(&t["stroke"])? as usize - 1;
            let (w, v) = input(field)?;
            let result = exclude_closed_line(w, v, 0.5, 128)?;
            let previous = array(&h["closure"])?
                .last()
                .ok_or("no previous pressure search")?;
            let old = uint(&previous["visited"])?;
            if previous["excluded"] != true {
                return Err("unverified previous search".into());
            }
            let status = if result.status == LineClosure::Excluded {
                "excluded"
            } else {
                "retained"
            };
            let class = if basic { "direct" } else { "needs_work" };
            *counts.entry(format!("{class}_{status}")).or_default() += 1;
            let now = result.visited;
            for (g, s) in [
                ("previous_boxes", old as f64),
                ("current_boxes", f64::from(now)),
                ("change", f64::from(now) - old as f64),
            ] {
                groups
                    .entry((class.into(), g.into()))
                    .or_default()
                    .push(s, key);
            }
            rows.push(json!({"case":k,"stroke":t["stroke"],"direct":basic,"before":old,"after":now,"status":status}));
        }
    }
    let groups = groups
        .into_iter()
        .map(|((class, metric), v)| json!({"class":class,"metric":metric,"statistics":v.stats()}))
        .collect::<Vec<_>>();
    fs::create_dir_all(out)?;
    write(
        &out.join("search.json"),
        &json!({"counts":counts,"summary":groups,"rows":rows}),
    )
}
