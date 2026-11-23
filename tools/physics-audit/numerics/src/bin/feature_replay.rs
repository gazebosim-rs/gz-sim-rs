// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reexecute both native experiments at the preselected common donor and package their public receipts.
use serde_json::{Value, json};
use std::{env, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn tr(v: &Value, s: usize) -> Result<&Value> {
    v["result"]["trials"]
        .as_array()
        .and_then(|x| x.iter().find(|r| r["stroke"] == s))
        .ok_or_else(|| "missing selected donor".into())
}
fn main() -> Result<()> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() != 6 {
        return Err("feature_replay REPORT_DIR FINITE_DIR RELAX_DIR FINITE_DRIVER RELAX_DRIVER NEW_SCRATCH_DIR".into());
    }
    let out = Path::new(&args[0]);
    let analysis = read(&out.join("feature-relaxation.json"))?;
    if analysis["final"] != true {
        return Err("physical feature replay requires the final joined field".into());
    }
    let pick = &analysis["outlier"];
    let k = pick["site"]["index"]
        .as_u64()
        .ok_or("no qualifying feature outlier")? as usize;
    let stroke = pick["site"]["stroke"].as_u64().ok_or("no chosen stroke")? as usize;
    let scratch = Path::new(&args[5]);
    fs::create_dir(scratch)?;
    let file = format!("case-{k:04}.json");
    let mut cut = None;
    for (src, driver, name, flag) in [
        (&args[1], &args[3], "finite", "--finite-replay"),
        (&args[2], &args[4], "relax", "--relaxation-replay"),
    ] {
        let dir = scratch.join(name);
        fs::create_dir_all(dir.join("cases"))?;
        let index_file = Path::new(src).join("cases").join(&file);
        let case = read(&index_file)?;
        let donor = &tr(&case, stroke)?["cut"];
        if cut.as_ref().is_some_and(|v| v != donor) {
            return Err("replay donors differ".into());
        }
        cut = Some(donor.clone());
        fs::hard_link(&index_file, dir.join("cases").join(&file))
            .or_else(|_| fs::copy(&index_file, dir.join("cases").join(&file)).map(|_| ()))?;
        let output = Command::new(driver)
            .arg(flag)
            .arg(k.to_string())
            .arg("--survey-stroke")
            .arg(stroke.to_string())
            .arg(&dir)
            .output()?;
        if !output.status.success() {
            return Err(format!(
                "{name} re-execution failed: {}",
                String::from_utf8_lossy(&output.stderr)
            )
            .into());
        }
    }
    let out = out.join("visual");
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    for (path, label, title) in [
        (
            scratch.join("relax/traces/relaxed.json"),
            "eight",
            "Eight unbiased passes throughout",
        ),
        (
            scratch.join(format!(
                "finite/case-{k:04}-stroke-{stroke}/traces/finite8.json"
            )),
            "finite",
            "Finite edge or vertex response at onset",
        ),
        (
            scratch.join("relax/traces/fine.json"),
            "fine",
            "4,096 calls and eight passes throughout",
        ),
    ] {
        let mut trace = read(&path)?;
        let frames = trace["frames"]
            .as_array_mut()
            .ok_or("missing source frames")?;
        if frames.len() != 27 {
            return Err("expected the measured 13 ms horizon".into());
        }
        for f in frames {
            let object = f.as_object_mut().ok_or("non-object source frame")?;
            // The finite interval archive records pair reactions. Limit all lanes to this common measurement.
            object.remove("corner_ns");
        }
        let filename = format!("traces/{label}.json");
        fs::write(out.join(&filename), serde_json::to_vec(&trace)?)?;
        lanes.push(json!({"label":label,"name":title,"file":filename}));
    }
    let first = pick["site"]["cut_tick"]
        .as_u64()
        .ok_or("missing cut tick")?;
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":format!("Finite contact and relaxation on fixture {k}, stroke {stroke}"),"pair_totals_only":true,
 "capture_ticks":{"first_public_reaction":first+1,"at_five_ms":first+10,"at_thirteen_ms":first+26},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("selection.json"), serde_json::to_vec_pretty(pick)?)?;
    fs::remove_dir_all(scratch)?;
    Ok(())
}
