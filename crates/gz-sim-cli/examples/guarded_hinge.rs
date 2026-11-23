// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise real competing CLI clients on a loaded hinge, then export each durable outcome.
#[allow(dead_code)]
#[path = "../../gz-physics-distributed/examples/support/primitive_replay_view.rs"]
mod primitive_replay_view;

use clap::Parser;
use gz_sim_scene::{distributed_hinge::load_distributed_hinge_world, reset_loaded_world};
use primitive_replay_view::View;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output, Stdio},
    sync::Barrier,
    thread,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const ARCHIVE: &str = "barrier.gzmoving";
const XYZ0: [f64; 4] = [0., 0., 0., 1.];

#[derive(Parser)]
struct Args {
    /// Authored distributed_hinge_impact SDF corresponding to the saved cut.
    #[arg(long)]
    sdf: PathBuf,
    /// Directory containing a moving hinge cut and a pending torque.
    #[arg(long)]
    cut: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    operator: PathBuf,
    /// Existing filesystem to use for temporary private test sessions.
    #[arg(long)]
    storage: PathBuf,
    /// New output directory.
    output: PathBuf,
}

struct Run<'a> {
    args: &'a Args,
    live: PathBuf,
    reference: PathBuf,
    events: Vec<Value>,
}

impl Run<'_> {
    fn command(&self, dir: &Path) -> Command {
        let mut cmd = Command::new(&self.args.operator);
        cmd.arg(dir).arg("--worker").arg(&self.args.worker);
        cmd
    }

    fn invoke(&self, dir: &Path, args: &[String]) -> Result<Value> {
        let out = self.command(dir).args(args).output()?;
        if !out.status.success() {
            return Err(String::from_utf8_lossy(&out.stderr).into_owned().into());
        }
        Ok(serde_json::from_slice(&out.stdout)?)
    }

    fn record(
        &mut self,
        operation: &str,
        expected: Option<(u64, u64)>,
        result: &str,
    ) -> Result<()> {
        let actual = self.invoke(&self.live, &[])?;
        if actual != self.invoke(&self.reference, &[])?
            || self.archive()? != fs::read(self.reference.join(ARCHIVE))?
        {
            return Err(format!("reference mismatch after {operation}").into());
        }
        self.events
            .push(json!({"operation":operation,"expected":expected,"result":result,"cut":actual}));
        Ok(())
    }

    fn archive(&self) -> Result<Vec<u8>> {
        Ok(fs::read(self.live.join(ARCHIVE))?)
    }

    fn stale(&mut self, operation: &str, epoch: u64, tick: u64, torque: f64) -> Result<()> {
        let before = self.archive()?;
        let cut = self.invoke(&self.live, &[])?;
        let output = self
            .command(&self.live)
            .args(guard(epoch, tick, torque))
            .output()?;
        let error = String::from_utf8_lossy(&output.stderr);
        let actual_epoch = cut["epoch"].as_u64().ok_or("missing epoch")?;
        let actual_tick = cut["state"]["hinge"]["tick"]
            .as_u64()
            .ok_or("missing tick")?;
        if output.status.success() || !error.contains(&format!("StaleInterval {{ expected_epoch: {epoch}, expected_tick: {tick}, epoch: {actual_epoch}, tick: {actual_tick} }}"))
            || before != self.archive()?
        {
            return Err(format!("incorrect rejection for {operation}: {error}").into());
        }
        self.record(operation, Some((epoch, tick)), "rejected")
    }

    fn compare_advance(&self, torque: f64) -> Result<()> {
        self.invoke(
            &self.reference,
            &[format!("--effort={torque}"), "--steps=1".into()],
        )?;
        Ok(())
    }
}

fn guard(epoch: u64, tick: u64, torque: f64) -> [String; 3] {
    [
        format!("--step-with-effort={torque}"),
        format!("--expect-epoch={epoch}"),
        format!("--expect-tick={tick}"),
    ]
}

fn replay_frame(row: &Value, ordinal: u64) -> Result<(String, String)> {
    let cut = &row["cut"];
    let hinge = &cut["state"]["hinge"];
    let mut view = View::default();
    for (index, (label, radius, body, color)) in [
        ("base", 0.16, &hinge["links"][0], [0.04, 0.30, 0.72, 1.]),
        ("tool", 0.12, &hinge["links"][1], [0.95, 0.25, 0.04, 1.]),
        (
            "ball",
            0.15,
            &cut["state"]["sphere"],
            [0.70, 0.12, 0.55, 1.],
        ),
    ]
    .into_iter()
    .enumerate()
    {
        view.add(
            10 + index as u64 * 2,
            label,
            serde_json::from_value(body["position"].clone())?,
            serde_json::from_value(body["orientation"].clone())?,
            json!({"kind":"sphere","radius":radius}),
            color,
        );
    }
    // Rails stay in world coordinates. The enlarged markers show ordinal tick and
    // epoch; neither gauge participates in the simulation.
    let actual = [
        cut["epoch"].as_u64().ok_or("missing epoch")?,
        hinge["tick"].as_u64().ok_or("missing tick")?,
    ];
    let expected: Option<[u64; 2]> = serde_json::from_value(row["expected"].clone())?;
    for (i, (name, clock, color)) in [
        ("committed", Some(actual), [0.1, 0.7, 0.82, 1.]),
        ("expected", expected, [0.99, 0.65, 0.08, 1.]),
    ]
    .into_iter()
    .enumerate()
    {
        let y = -0.48 - 0.19 * i as f64;
        let id = 20 + 10 * i as u64;
        view.cuboid(
            id,
            &format!("{name}_tick_rail"),
            [0.6, y, 0.05],
            [1.22, 0.04, 0.055],
            [0.17, 0.23, 0.30, 1.],
        );
        if let Some([epoch, tick]) = clock {
            view.add(
                100 + ordinal * 16 + i as u64 * 4,
                &format!("{name}_tick_{tick}"),
                [tick as f64 * 0.005, y, 0.11],
                XYZ0,
                json!({"kind":"sphere","radius":0.065}),
                color,
            );
            view.cuboid(
                102 + ordinal * 16 + i as u64 * 4,
                &format!("{name}_epoch_{epoch}"),
                [-0.23, y, 0.05 + epoch as f64 * 0.09],
                [0.10, 0.095, 0.10 + epoch as f64 * 0.18],
                color,
            );
        }
    }
    let outcome = row["result"].as_str().ok_or("missing result")?;
    let color = match outcome {
        "accepted" => [0.16, 0.83, 0.44, 1.],
        "rejected" => [0.95, 0.22, 0.16, 1.],
        "reset" => [0.45, 0.31, 0.91, 1.],
        _ => [0.56, 0.66, 0.72, 1.],
    };
    view.cuboid(
        110 + ordinal * 16,
        &format!("outcome_{}_{}", outcome, row["operation"].as_str().unwrap()),
        [1.42, -0.56, 0.13],
        [0.22, 0.23, 0.24],
        color,
    );
    Ok(view.finish_at(ordinal, (ordinal - 1) * 500_000_000))
}

fn main() -> Result<()> {
    let args = Args::parse();
    let temp = tempfile::tempdir_in(&args.storage)?;
    let mut run = Run {
        args: &args,
        live: temp.path().join("live"),
        reference: temp.path().join("reference"),
        events: Vec::new(),
    };
    for dir in [&run.live, &run.reference] {
        fs::create_dir(dir)?;
        fs::copy(args.cut.join(ARCHIVE), dir.join(ARCHIVE))?;
    }
    let initial = run.invoke(&run.live, &[])?;
    let t = initial["state"]["hinge"]["tick"]
        .as_u64()
        .ok_or("missing tick")?;
    let epoch = initial["epoch"].as_u64().ok_or("missing epoch")?;
    if initial["pending_effort_nm"].as_f64().is_none() || t == 0 {
        return Err("use a warm cut with saved torque".into());
    }
    run.record("loaded_with_pending", None, "loaded")?;
    run.stale("future_tick", epoch, t + 1, -0.02)?;
    run.stale("future_epoch", epoch + 1, t, -0.02)?;
    let wrote = run
        .command(&run.live)
        .args(guard(epoch, t, -0.02))
        .stdout(Stdio::null())
        .status()?;
    if !wrote.success() {
        return Err("lost-response operation failed".into());
    }
    run.compare_advance(-0.02)?;
    run.record("response_discarded", Some((epoch, t)), "accepted")?;
    run.stale("resubmit_after_lost_response", epoch, t, -0.02)?;
    // Simultaneous clients propose opposite torques for one interval.
    let torques = [0.23, -0.37];
    let gate = Barrier::new(2);
    let results: Vec<Output> = thread::scope(|scope| {
        torques
            .map(|torque| {
                let mut cmd = run.command(&run.live);
                cmd.args(guard(epoch, t + 1, torque));
                let gate = &gate;
                scope.spawn(move || {
                    gate.wait();
                    cmd.output()
                })
            })
            .into_iter()
            .map(|h| h.join().expect("client thread"))
            .collect::<std::io::Result<Vec<_>>>()
    })?;
    let winners: Vec<_> = results
        .iter()
        .enumerate()
        .filter(|(_, r)| r.status.success())
        .collect();
    let [(winner, output)] = winners.as_slice() else {
        return Err("expected exactly one successful client".into());
    };
    if serde_json::from_slice::<Value>(&output.stdout)? != run.invoke(&run.live, &[])? {
        return Err("wrong race winner receipt".into());
    }
    let other = 1 - *winner;
    let loser_error = String::from_utf8_lossy(&results[other].stderr);
    if !loser_error.contains("StaleInterval")
        && !loser_error.contains("WouldBlock")
        && !loser_error.contains("locked")
    {
        return Err(format!("unexpected losing result {loser_error}").into());
    }
    run.compare_advance(torques[*winner])?;
    run.record("two_competing_clients", Some((epoch, t + 1)), "accepted")?;
    run.stale("losing_client_reopens", epoch, t + 1, torques[other])?;
    let xml = fs::read_to_string(&args.sdf)?;
    for dir in [&run.live, &run.reference] {
        let mut sim =
            load_distributed_hinge_world(&xml, "guarded.sdf", "default", &args.worker, dir, None)?;
        reset_loaded_world(&mut sim, |_| ())?;
    }
    run.record("bevy_reset", None, "reset")?;
    for dir in [&run.live, &run.reference] {
        run.invoke(dir, &["--effort=0.08".into()])?;
    }
    run.record("new_epoch_pending", None, "queued")?;
    run.stale("same_tick_old_epoch", epoch, 0, -0.075)?;
    run.invoke(&run.live, &guard(epoch + 1, 0, -0.075))?;
    run.compare_advance(-0.075)?;
    run.record("new_epoch_and_tick_match", Some((epoch + 1, 0)), "accepted")?;
    fs::create_dir(&args.output)?;
    let frames = run
        .events
        .iter()
        .enumerate()
        .map(|(i, row)| replay_frame(row, i as u64 + 1))
        .collect::<Result<Vec<_>>>()?;
    gz_sim_cli::export_archive(&args.output.join("replay"), Default::default(), frames)?;
    let report = json!({"initial_tick":t,"epoch":epoch,"winning_torque_nm":torques[*winner],"losing_torque_nm":torques[other],"loser_error":loser_error,"events":run.events});
    fs::write(
        args.output.join("events.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    println!(
        "{} real operation outcomes verified against separate archives",
        report["events"].as_array().unwrap().len()
    );
    Ok(())
}
