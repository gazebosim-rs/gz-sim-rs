// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct each isolated upper strike in a separate angular-momentum formulation.
#[path = "shared/impact_measurements.rs"]
mod measures;
use measures::*;
use serde_json::json;
use std::{fs, path::Path};
#[path = "shared/opposing_mechanics.rs"]
mod mechanics;
use mechanics::*;
fn main() {
    let input = std::env::args().nth(1).expect("archive");
    let path = Path::new(&input);
    let x = read(&path.join("projection.json"));
    let mut a = Audit::default();
    assert_eq!(arr(&x["donors"]).len(), 66);
    for d in arr(&x["donors"]) {
        for row in arr(&d["cases"]) {
            let delay = int(&row["delay_ns"]);
            let e = &row["reference"];
            let tied = e.is_null();
            let kind = if tied {
                "inconclusive"
            } else if num(&e["mu"]) == 0. {
                "frictionless"
            } else if e["stick"] == true {
                "sticking"
            } else {
                "sliding"
            };
            a.count(&format!("events/{kind}"));
            if tied {
                assert!(delay <= 8 && arr(&row["cold"]).len() == 1);
            } else {
                assert_eq!(arr(&row["cold"]).len(), 3);
            }
            for p in arr(&row["cold"]) {
                let h = &p["upper"];
                let admit = !h.is_null();
                let base = num(&p["cut_ns"]);
                a.count(&format!("matched/{kind}/{admit}"));
                if tied {
                    assert!(!admit);
                    continue;
                }
                if admit {
                    assert_eq!(int(&h["corner"]), int(&e["corner"]));
                    assert_eq!(num(&p["lower_ns"]), 0.);
                    assert!(vec(&h["impulse"]).distance(vec(&e["impulse"])) < 2e-7);
                    assert!((num(&h["physical_ns"]) + base - num(&e["at_ns"])).abs() < 0.11);
                    reaction(&mut a, "matched/geometry", d, row, h, base);
                    compare(&mut a, "matched/admitted", e, h, base);
                    compare(&mut a, &format!("matched/{kind}/admitted"), e, h, base);
                }
                for (key, obs) in [("projected", p), ("native", &p["unprojected"])] {
                    if num(&obs["lower_ns"]) == 0. {
                        let error = vec(&obs["momentum_impulse"]).distance(vec(&e["impulse"]));
                        a.sample(
                            &format!("matched/{kind}/{key}"),
                            "net_vector_error_ns",
                            error,
                        );
                        if admit {
                            a.sample(
                                &format!("paired/{kind}/{key}"),
                                "net_vector_error_ns",
                                error,
                            );
                        }
                    } else {
                        a.count(&format!("matched/{key}/also_loaded_lower"));
                    }
                }
            }
            let runs = arr(&row["public"]);
            assert_eq!(runs.len(), 2);
            // Preserve the already-qualified lower impact whenever the author enables the ceiling.
            let p = arr(&runs[0]["data"]["primary_projections"]);
            let q = arr(&runs[1]["data"]["primary_projections"]);
            assert_eq!(p.len(), q.len());
            for (a0, b) in p.iter().zip(q) {
                assert_eq!(a0["at_ns"], b["at_ns"]);
                let error = vec(&a0["world"]).distance(vec(&b["world"]));
                a.sample("primary", "two_policy_reaction_delta_ns", error);
                assert!(error < 1e-6);
            }
            for (k, r) in runs.iter().enumerate() {
                let run = &r["data"];
                let name = if k == 0 { "native" } else { "projected" };
                for h in arr(&run["projections"]) {
                    reaction(&mut a, "public/geometry", d, row, h, 0.);
                }
                let first = &run["first_upper"];
                let resolved = !first["upper_projection"].is_null();
                if k == 1 {
                    a.count(&format!("public/{delay}/first_projected/{resolved}"));
                }
                if tied {
                    assert!(!resolved);
                    continue;
                }
                if first.is_null() {
                    continue;
                }
                let at = num(&e["at_ns"]);
                let end = num(&first["end_ns"]);
                let t = if resolved {
                    num(&first["upper_projection"]["physical_ns"])
                } else {
                    end
                };
                let key = format!("public/{delay}/{name}");
                a.sample(&key, "calls", num(&run["native_calls"]));
                a.sample(&key, "first_delay_ns", t - at);
                if resolved {
                    compare(
                        &mut a,
                        &format!("public/{delay}/first_hit_reference"),
                        e,
                        &first["upper_projection"],
                        0.,
                    );
                }
                // A native grouped tangent is attributable from momentum only when
                // the lower pair had zero impulse and no second physical strike arrived.
                let limit = at + num(&row["qualified_duration_ns"]);
                if t >= at - 0.1 && t <= limit + 0.1 && num(&first["primary_ns"]) == 0. {
                    a.sample(
                        &key,
                        "qualified_first_vector_error_ns",
                        vec(&first["net_foot_impulse"]).distance(vec(&e["impulse"])),
                    );
                }
                let p = &run["qualified_endpoint"];
                if !p.is_null() {
                    a.sample(&key, "last_qualified_com_m", num(&p["com_m"]));
                    a.sample(&key, "last_qualified_spin_rad_s", num(&p["spin_rad_s"]));
                }
            }
        }
    }
    for pass in ["cpu6-a", "cpu6-b"] {
        cost(&mut a, pass, &read(&path.join(pass).join("cost.json")));
    }
    fs::write(
        path.join("readback.json"),
        serde_json::to_vec_pretty(&a.report()).unwrap(),
    )
    .unwrap();
    println!(
        "{}",
        json!({"checked_donors":66,"public_pairs":990,"matched_ordered_cuts":1782,"tied_cuts":396})
    );
}
