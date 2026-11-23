// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
#[ignore = "requires three recorded fold-bench trials"]
fn only_a_bounded_receiver_in_the_landing_path_has_an_impulse() -> Result<()> {
    let input = env::var_os("GZ_FOLD_BENCH").ok_or("set GZ_FOLD_BENCH")?;
    let root = Path::new(&input);
    let run = |name: &str| -> Result<(Archive, Vec<Value>, Value)> {
        let dir = root.join(name);
        let outcome = read(&dir.join("outcome.json"))?;
        let start = uint(&outcome["pickup_time_ns"]);
        let scene = fs::read_to_string(dir.join("world.sdf"))?;
        let xml = roxmltree::Document::parse(&scene)?;
        let boundaries: Vec<_> = xml
            .descendants()
            .filter(|n| n.has_tag_name("box") && n.attribute("coupling") == Some("one_way"))
            .collect();
        assert_eq!(boundaries.len(), if name == "plane" { 0 } else { 2 });
        let r = inspect(root, name, 5, start)?;
        let samples: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("bench-trace.json"))?)?;
        assert_eq!(samples.len(), r.trace.len() - r.baseline - 1);
        assert!(scalar(&outcome["pickup_clearance_m"]) > 0.01);
        near(scalar(&outcome["pickup_bench_up_n"]), 0.);
        let weight = scalar(&outcome["weight_n"]);
        let jaws = outcome["pickup_jaw_up_n"].as_array().unwrap();
        assert!(((jaws.iter().map(scalar).sum::<f64>() - weight) / weight).abs() < 0.025);
        assert_eq!(samples.last(), Some(&outcome["terminal"]));
        for (i, (raw, t)) in samples.iter().zip(&r.trace[r.baseline + 1..]).enumerate() {
            assert_eq!(uint(&raw["time_ns"]), start + 1_000_000 * (1 + i as u64));
            let support = scalar(&raw["source_or_plane_impulse_ns"][2]);
            let receiver = scalar(&raw["receiver_impulse_ns"][2]);
            near(support + receiver, -0.001 * scalar(&t["bench_up_n"]));
            if name != "plane" {
                near(support, 0.);
            }
            if i + 1 < samples.len() || name == "miss" {
                near(receiver, 0.);
                near(support, 0.);
            }
        }
        let row = samples.last().unwrap();
        let gap = row["horizontal_skin_clearance_m"].as_array().unwrap();
        let j = |key| -scalar(&row[key][2]);
        match name {
            "plane" => {
                assert!(
                    j("source_or_plane_impulse_ns") > 1e-5 && gap.iter().all(|g| scalar(g) > 0.)
                );
            }
            "receiver" => {
                assert!(
                    j("receiver_impulse_ns") > 1e-5
                        && scalar(&gap[0]) > 0.
                        && scalar(&gap[1]) <= 0.
                );
            }
            _ => {
                assert_eq!(name, "miss");
                assert!(
                    scalar(&row["node_bounds_m"][2][1]) + 0.01 < 0.12
                        && gap.iter().all(|g| scalar(g) > 0.)
                );
            }
        }
        Ok((r, samples, outcome))
    };
    let (caught, cs, co) = run("receiver")?;
    let (missed, ms, mo) = run("miss")?;
    let (plane, ps, po) = run("plane")?;
    let matched = caught
        .trace
        .iter()
        .zip(&missed.trace)
        .take_while(|(c, m)| c == m)
        .count();
    assert_eq!(caught.baseline, missed.baseline);
    assert_eq!(matched, caught.trace.len() - 1);
    let last = cs.last().unwrap();
    let measured = json!({"finite_touchdown":co,"finite_miss":mo,"reference":po,
        "first_divergent_contact_tick":matched-caught.baseline,
        "miss_skin_gap_to_benches_m":ms.last().unwrap()["horizontal_skin_clearance_m"],
        "miss_max_height_m":ms.last().unwrap()["node_bounds_m"][2][1],
        "plane_delta_pickup_ms":(uint(&co["pickup_time_ns"]) as f64-uint(&po["pickup_time_ns"]) as f64)*1e-6,
        "plane_delta_first_impulse_ns":scalar(&ps.last().unwrap()["source_or_plane_impulse_ns"][2])-scalar(&last["receiver_impulse_ns"][2]),
        "plane_minus_finite_touchdown_ticks":ps.len() as i64-cs.len() as i64,
        "native_pose_checks":[caught.summary,missed.summary,plane.summary]});
    if let Some(path) = env::var_os("GZ_FOLD_BENCH_AUDIT") {
        fs::write(path, serde_json::to_vec_pretty(&measured)?)?;
    }
    Ok(())
}
