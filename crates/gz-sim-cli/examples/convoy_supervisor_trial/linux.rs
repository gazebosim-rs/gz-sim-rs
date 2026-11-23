// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Linux process qualification for operator-selected reconstruction after supervisor loss.
mod process;
mod wire;

use clap::{Parser, ValueEnum};
use process::{Supervisor, wait_workers_gone};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    net::TcpListener,
    path::{Path, PathBuf},
    process::Command,
    thread,
    time::{Duration, Instant},
};
use wire::{Connection, clock};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn check(condition: bool, why: impl Into<String>) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(why.into().into())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, ValueEnum)]
enum Case {
    Paused,
    Advancing,
    Migrated,
}
#[derive(Parser)]
struct Args {
    #[arg(long)]
    case: Case,
    /// Release directory containing the production binaries and qualification examples.
    #[arg(long)]
    release: PathBuf,
    /// Physics worker or the deployed convoy_ssh_worker.
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    zenoh: bool,
    #[arg(long)]
    port: u16,
    /// Fresh trial directory with an existing parent.
    #[arg(long)]
    run: PathBuf,
    /// Loaded 150 ms cut. Use the post-transfer cut for pipes and pre-transfer cut for the SSH case.
    #[arg(long)]
    source: PathBuf,
    /// For the SSH deployment, the equivalent independently migrated local cut.
    #[arg(long, required_if_eq("case", "migrated"))]
    transferred: Option<PathBuf>,
    #[arg(long)]
    sdf: PathBuf,
    #[arg(long)]
    program: PathBuf,
    /// Previously qualified, uninterrupted trajectory with independently checked per-cut SHA-256.
    #[arg(long)]
    reference: PathBuf,
}
fn saved_json(dir: &Path, name: &str, value: &Value) -> Result<()> {
    fs::write(dir.join(name), serde_json::to_vec_pretty(value)?)?;
    Ok(())
}
fn output(c: &mut Command) -> Result<Value> {
    let o = c.output()?;
    check(
        o.status.success(),
        format!("{c:?}: {} {}", o.status, String::from_utf8_lossy(&o.stderr)),
    )?;
    Ok(serde_json::from_slice(&o.stdout)?)
}
fn hash(p: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(fs::read(p)?)))
}
fn inspect(a: &Args, p: &Path) -> Result<Value> {
    output(Command::new(a.release.join("convoy_inspect")).arg(p))
}
fn tick(c: &Value) -> Result<u64> {
    c["cut"]["observation"]["hinge"]["tick"]
        .as_u64()
        .ok_or_else(|| "bad cut".into())
}

fn check_poses(scene: &Value, cut: &Value) -> Result<()> {
    check(
        clock(scene)? == tick(cut)?,
        "scene and archive clocks differ",
    )?;
    let visuals = scene["visuals"]
        .as_array()
        .ok_or("missing displayed bodies")?;
    let o = &cut["cut"]["observation"];
    let states = o["hinge"]["links"]
        .as_array()
        .ok_or("missing links")?
        .iter()
        .chain(o["spheres"].as_array().ok_or("missing spheres")?);
    check(visuals.len() == 4, "missing convoy bodies")?;
    for (body, visual) in states.zip(visuals) {
        check(
            body["position"] == visual["translation"] && body["orientation"] == visual["rotation"],
            format!(
                "archived pose differs from displayed pose for {}",
                visual["name"]
            ),
        )?;
    }
    Ok(())
}

fn known_cut(cut: &Value, reference: &Value) -> Result<()> {
    let t = tick(cut)?;
    let row = reference["samples"]
        .as_array()
        .ok_or("missing reference samples")?
        .iter()
        .find(|s| s["tick"] == t)
        .ok_or_else(|| format!("missing independent reference tick {t}"))?;
    check(
        row["sha256"] == cut["sha256"],
        format!("different complete archive at tick {t}"),
    )
}

struct Events {
    start: Instant,
    rows: Vec<Value>,
}
impl Events {
    fn record(&mut self, what: &str, detail: Value) {
        self.rows.push(json!({"elapsed_ms":self.start.elapsed().as_secs_f64()*1000.,"event":what,"detail":detail}));
    }
}

fn parked(
    a: &Args,
    session: &Path,
    package: &Path,
    label: &str,
    t: u64,
    eligible: bool,
    events: &mut Events,
) -> Result<(Supervisor, Connection)> {
    let sha = hash(package)?;
    events.record(
        "cold_start",
        json!({"supervisor":label,"expected_tick":t,"package_sha256":sha}),
    );
    let mut child = Supervisor::start(a, session, package, Some(&sha), label)?;
    let conn = child.connection(a.port)?;
    conn.offer(t, eligible)?;
    check(
        child.workers()?.is_empty(),
        "a parked cold supervisor started a worker",
    )?;
    events.record(
        "parked",
        json!({"supervisor":label,"pid":child.pid(),"offer":conn.fence,"worker_processes":0}),
    );
    Ok((child, conn))
}

fn run(a: &Args) -> Result<()> {
    // Refuse an occupied viewer port before starting any process for this trial.
    drop(TcpListener::bind((std::net::Ipv4Addr::LOCALHOST, a.port))?);
    fs::create_dir(&a.run)?;
    fs::create_dir(a.run.join("snapshots"))?;
    let session = a.run.join("session");
    fs::create_dir(&session)?;
    let live = session.join("barrier.gzmultisphere");
    fs::copy(&a.source, &live)?;
    let reference = serde_json::from_slice(&fs::read(&a.reference)?)?;
    let mut events = Events {
        start: Instant::now(),
        rows: vec![],
    };
    let mut transfer = Value::Null;
    if let Some(expected) = &a.transferred {
        transfer = output(
            Command::new(a.release.join("examples/convoy_operator_fixture"))
                .arg("--worker")
                .arg(&a.worker)
                .arg("transfer")
                .arg(&session),
        )?;
        check(
            fs::read(&live)? == fs::read(expected)?,
            "deployed transfer differs from independently migrated archive",
        )?;
        events.record("deployed_transfer", transfer.clone());
    }

    let launch = a.run.join("launch.gzconvoy");
    let capture = a.run.join("latest.gzconvoy");
    let mut original = Supervisor::start(a, &session, &launch, None, "original")?;
    let mut hot = original.connection(a.port)?;
    let warm = inspect(a, &live)?;
    check_poses(&hot.scene, &warm)?;
    known_cut(&warm, &reference)?;
    check(
        hot.scene["paused"] == true && tick(&warm)? == 150,
        "expected loaded paused source at 150 ms",
    )?;
    let children = original.workers()?;
    check(
        children.len() == 3,
        format!("expected 3 original workers, got {children:?}"),
    )?;
    let launch_sha = hash(&launch)?;
    events.record("original_paused",json!({"pid":original.pid(),"workers":children,"launch_sha256":launch_sha,"archive_sha256":warm["sha256"]}));
    let mut selected = original.snapshot(a, None)?;
    check(
        fs::read(&launch)? == fs::read(&selected)?,
        "paused signal changed captured cut or installed drive",
    )?;
    events.record(
        "paused_snapshot",
        json!({"package_sha256":launch_sha,"tick":150}),
    );
    let mut selected_tick = 150;
    let mut suspended = None;
    if a.case == Case::Advancing {
        check(
            !a.zenoh,
            "in-flight interruption requires the local pipe worker",
        )?;
        hot.play()?;
        let until_signal = Instant::now() + Duration::from_secs(5);
        while tick(&inspect(a, &live)?)? < 151 {
            check(Instant::now() < until_signal, "missing first advancing cut")?;
            thread::sleep(Duration::from_millis(4));
        }
        selected = original.snapshot(a, Some(&selected))?;
        let package = output(
            Command::new(a.release.join("gz-sim-convoy-checkpoint"))
                .arg("inspect")
                .arg(&selected)
                .arg("--package-sha256")
                .arg(hash(&selected)?),
        )?;
        selected_tick = package["tick"].as_u64().ok_or("snapshot missing tick")?;
        check(
            selected_tick > 150,
            "running snapshot did not cross first advance",
        )?;
        let expected = reference["samples"]
            .as_array()
            .ok_or("missing reference trajectory")?
            .iter()
            .find(|s| s["tick"] == selected_tick)
            .ok_or("snapshot outside reference")?;
        check(
            package["archive_sha256"] == expected["sha256"]
                && package["next_torque_nm"] == expected["next_effort_nm"],
            "moving snapshot differs from unbroken reference",
        )?;
        events.record("advancing_snapshot", package);
        let until = Instant::now() + Duration::from_secs(10);
        loop {
            if tick(&inspect(a, &live)?)? >= 153.max(selected_tick + 1) {
                break;
            }
            check(
                Instant::now() < until,
                "running supervisor failed to publish",
            )?;
            thread::sleep(Duration::from_millis(4));
        }
    }
    if a.case == Case::Advancing {
        let cut = inspect(a, &live)?;
        let slot = cut["cut"]["owners"][0]
            .as_u64()
            .ok_or("missing hinge owner")?;
        let (stopped, pid, unread) = process::interrupt_pipe(&children, &session, slot)?;
        let before = hash(&live)?;
        events.record(
            "interrupted_worker_rpc",
            json!({"slot":slot,"pid":pid,"unread_pipe_bytes":unread,"last_archive_sha256":before}),
        );
        suspended = Some((stopped, before));
    }
    drop(hot);
    original.kill()?;
    if let Some((stopped, before)) = suspended {
        check(
            hash(&live)? == before,
            "uncertain worker RPC crossed the published cut",
        )?;
        drop(stopped);
    }
    events.record(
        "supervisor_sigkill",
        json!({"pid":original.pid(),"was_running":a.case==Case::Advancing}),
    );
    wait_workers_gone(&children)?;
    events.record("old_worker_processes_exited", json!({"workers":children}));
    let source = inspect(a, &live)?;
    let t = tick(&source)?;
    known_cut(&source, &reference)?;
    let cut0 = a.run.join("source.gzmultisphere");
    fs::copy(&live, &cut0)?;
    let package = if a.case == Case::Advancing {
        check(t >= 153, "unexpected surviving cut after Play")?;
        let (mut stale, mut ws) = parked(
            a,
            &session,
            &selected,
            "stale",
            selected_tick,
            false,
            &mut events,
        )?;
        let token = ws.offer(selected_tick, false)?.to_owned();
        let denial = ws.recover(selected_tick, &token, false)?;
        events.record("old_package_denied", denial);
        check(
            fs::read(&live)? == fs::read(&cut0)?,
            "stale package changed the surviving archive",
        )?;
        drop(ws);
        stale.kill()?;
        let report = output(
            Command::new(a.release.join("gz-sim-convoy-checkpoint"))
                .arg("capture-latest")
                .arg("--from")
                .arg(&selected)
                .arg("--package-sha256")
                .arg(hash(&selected)?)
                .arg("--session")
                .arg(&session)
                .arg("--output")
                .arg(&capture),
        )?;
        events.record("selected_surviving_cut", report);
        &capture
    } else {
        &selected
    };
    let (mut child, mut conn) =
        parked(a, &session, package, "replacement-1", t, true, &mut events)?;
    check_poses(&conn.scene, &source)?;
    if a.case == Case::Paused {
        let old = conn.offer(t, true)?.to_owned();
        drop(conn);
        child.kill()?;
        events.record(
            "parked_supervisor_sigkill",
            json!({"pid":child.pid(),"before_admission":true}),
        );
        (child, conn) = parked(a, &session, package, "replacement-2", t, true, &mut events)?;
        let current = conn.offer(t, true)?.to_owned();
        check(old != current, "cold supervisor reused previous token")?;
        events.record("old_offer_denied", conn.recover(t, &old, false)?);
        let accepted = conn.recover(t, &current, true)?;
        child.kill()?;
        events.record(
            "sigkill_after_admission",
            json!({"pid":child.pid(),"admission":accepted}),
        );
        drop(conn);
        check(
            fs::read(&live)? == fs::read(&cut0)?,
            "admission consumed pending physics input",
        )?;
        (child, conn) = parked(a, &session, package, "replacement-3", t, true, &mut events)?;
        check(
            conn.offer(t, true)? != current,
            "admitted token survived a supervisor death",
        )?;
        events.record(
            "admitted_old_offer_denied",
            conn.recover(t, &current, false)?,
        );
    }

    let token = conn.offer(t, true)?.to_owned();
    let parked_scene = conn.scene.clone();
    let launch_start = Instant::now();
    let admitted = conn.recover(t, &token, true)?;
    let admitted_ms = launch_start.elapsed().as_secs_f64() * 1000.;
    events.record("admitted", admitted);
    drop(conn);
    let mut restored = child.connection(a.port)?;
    let proof = restored
        .restored
        .as_ref()
        .ok_or("missing restored identity")?;
    check(
        proof["token"] == token && proof["epoch"] == 0 && proof["tick"] == t,
        "installed wrong operator identity",
    )?;
    check(
        restored.scene["paused"] == true && restored.scene["visuals"] == parked_scene["visuals"],
        "different installed scene",
    )?;
    check_poses(&restored.scene, &source)?;
    let reinstated_ms = launch_start.elapsed().as_secs_f64() * 1000.;
    let installed = child.workers()?;
    check(
        installed.len() == 3 && children.is_disjoint(&installed),
        "expected three fresh worker transports",
    )?;
    check(
        fs::read(&live)? == fs::read(&cut0)?,
        "install changed the source archive",
    )?;
    events.record(
        "verified_restored",
        json!({"pid":child.pid(),"workers":installed,"proof":proof,"poses":4}),
    );
    let now = Instant::now();
    let step = restored.step(t, true)?;
    let step_ms = now.elapsed().as_secs_f64() * 1000.;
    events.record("checked_step", step);
    let repeat = restored.step(t, false)?;
    events.record("replayed_step_denied", repeat);
    let next = inspect(a, &live)?;
    known_cut(&next, &reference)?;
    check(
        tick(&next)? == t + 1 && next["cut"]["pending_effort"].is_null(),
        "wrong committed successor",
    )?;
    let cut1 = a.run.join("successor.gzmultisphere");
    fs::copy(&live, &cut1)?;
    drop(restored);
    let last = child.connection(a.port)?;
    check_poses(&last.scene, &next)?;
    drop(last);
    child.kill()?;
    wait_workers_gone(&installed)?;
    let oracle = output(
        Command::new(a.release.join("examples/convoy_drive_oracle"))
            .arg("--worker")
            .arg(a.release.join("physics_worker"))
            .arg("--sdf")
            .arg(&a.sdf)
            .arg("--program")
            .arg(&a.program)
            .arg(&cut0)
            .arg(&cut1),
    )?;
    saved_json(&a.run, "oracle.json", &oracle)?;
    check(
        oracle["exact_native_archives"] == 2 && oracle["ordinary_observations"] == 2,
        "incomplete oracle readback",
    )?;
    saved_json(&a.run, "events.json", &json!(events.rows))?;
    saved_json(
        &a.run,
        "result.json",
        &json!({"case":format!("{:?}",a.case),"zenoh":a.zenoh,"transfer":transfer,
        "source_tick":t,"source_sha256":source["sha256"],"source_impulse_ns":source["cut"]["observation"]["sphere_reaction"]["normal_impulse"],
        "durable_pending_nm":source["cut"]["pending_effort"],"successor_tick":t+1,"successor_sha256":next["sha256"],
        "restored_pose_matches":4,"successor_pose_matches":4,"ordinary_observations":2,"exact_native_archives":2,
        "admission_ms":admitted_ms,"verified_install_from_request_ms":reinstated_ms,"checked_step_ms":step_ms}),
    )?;
    Ok(())
}
pub(super) fn execute() -> Result<()> {
    run(&Args::parse())
}
