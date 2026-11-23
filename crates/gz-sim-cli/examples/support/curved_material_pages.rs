// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise bounded transfers of a real SDF cable through the ordinary simulator and worker.
use super::*;
use std::{
    fmt::Write as _,
    fs::OpenOptions,
    io::{self, Read},
    process::{Command, Stdio},
};
const PAGE: u32 = 96 * 1024;

pub(super) fn world(count: usize, seed: u64) -> Result<String> {
    if !(16..=4096).contains(&count) {
        return Err("expected 16 through 4096 cable particles".into());
    }
    let mut sdf = xml(seed);
    let start = sdf
        .find("<particle position")
        .ok_or("missing cable population")?;
    let end = sdf[start..].find("</cable>").ok_or("missing cable end")? + start;
    let mut rows = String::new();
    for i in 0..count {
        let u = i as f64 / (count - 1) as f64;
        // Small, continuous departures from a straight cable give different contact times.
        let x = 0.005 * (7. * u).sin();
        let y = -0.28 + 0.56 * u;
        let z = 0.595 + 0.0015 * (14. * u).cos();
        let mass = 0.2 / count as f64;
        writeln!(
            rows,
            "<particle position='{x:.17} {y:.17} {z:.17}' mass='{mass:.17}'/>"
        )?;
    }
    sdf.replace_range(start..end, &rows);
    for i in (0..16).rev() {
        let id = i * (count - 1) / 15;
        sdf = sdf.replace(
            &format!("particle=\"{i}\" visual="),
            &format!("particle=\"{id}\" visual="),
        );
    }
    Ok(sdf)
}

fn frame(r: &mut impl Read) -> io::Result<Vec<u8>> {
    let mut len = [0; 4];
    r.read_exact(&mut len)?;
    let n = u32::from_le_bytes(len) as usize;
    if n > 192 * 1024 {
        return Err(io::Error::other("oversized managed frame"));
    }
    let mut body = vec![0; n];
    r.read_exact(&mut body)?;
    Ok(body)
}
fn send(w: &mut impl Write, b: &[u8]) -> io::Result<()> {
    w.write_all(&(b.len() as u32).to_le_bytes())?;
    w.write_all(b)?;
    w.flush()
}
fn word(b: &[u8], i: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(i..i + 4)?.try_into().ok()?))
}

fn hold_packed(
    dir: &Path,
    tick: u64,
    site: &str,
    worker: u32,
    bytes: usize,
    sent: usize,
) -> Result<()> {
    let pid = std::process::id();
    let part = dir.join(format!("pending-{pid}"));
    let ready = dir.join(format!("ready-{pid}"));
    fs::write(
        &part,
        serde_json::to_vec(
            &json!({"relay":pid,"worker":worker,"tick":tick,"site":site,"payload_bytes":bytes,"forwarded_bytes":sent}),
        )?,
    )?;
    fs::rename(part, ready)?;
    loop {
        std::thread::park();
    }
}

/// Put a transparent parent on the private worker pipes. Only the example starts this relay.
/// The real physics child obtains the normal exclusive lease and computes every returned page.
pub(super) fn relay(flag: &str, args: impl Iterator<Item = String>) -> Result<()> {
    trajectory_workers::arm_launch_owner()?;
    let actual =
        std::env::var_os("GZ_EXAMPLE_REAL_WORKER").ok_or("missing supervised worker path")?;
    let dir =
        PathBuf::from(std::env::var_os("GZ_EXAMPLE_BREAKPOINTS").ok_or("missing test directory")?);
    let mut child = Command::new(actual)
        // This example relay owns the physical solver; both ends fence their direct child.
        .env("GZ_TRAJECTORY_LAUNCH_OWNER", std::process::id().to_string())
        .arg(flag)
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()?;
    let mut input = child.stdin.take().unwrap();
    let mut output = child.stdout.take().unwrap();
    let mut source = io::stdin().lock();
    let mut sink = io::stdout().lock();
    let boot = frame(&mut source)?;
    send(&mut input, &boot)?;
    send(&mut sink, &frame(&mut output)?)?;
    let mut blob = None;
    loop {
        let mut cmd = match frame(&mut source) {
            Ok(v) => v,
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e.into()),
        };
        let tick = u64::from_le_bytes(cmd.get(24..32).ok_or("truncated request")?.try_into()?);
        let op = word(&cmd, 32).ok_or("missing operation")?;
        if matches!(op, 3 | 5 | 7)
            && fs::read_to_string(dir.join("different-interval"))
                .ok()
                .as_deref()
                == Some(&format!("{} {tick}", std::process::id()))
        {
            // On this private lease, ask the real solver for 3 ms instead of 2 ms. Both its
            // native and material state are authentic but belong to a different successor.
            let dt = cmd.get_mut(68..76).ok_or("truncated managed step")?;
            if u64::from_le_bytes((*dt).try_into()?) != 2_000_000 {
                return Err("unexpected disagreement fixture time step".into());
            }
            dt.copy_from_slice(&3_000_000u64.to_le_bytes());
        }
        let marker = if op == 7 {
            fs::read_to_string(dir.join(format!("hold-{}", std::process::id()))).unwrap_or_default()
        } else {
            String::new()
        };
        if marker == format!("{tick} before") {
            hold_packed(&dir, tick, "before", child.id(), 0, 0)?;
        }
        send(&mut input, &cmd)?;
        let mut answer = frame(&mut output)?;
        if !marker.is_empty() && word(&answer, 44) == Some(6) {
            if marker == format!("{tick} solved") {
                hold_packed(&dir, tick, "solved", child.id(), answer.len(), 0)?;
            }
            if marker == format!("{tick} partial") {
                let half = answer.len() / 2;
                sink.write_all(&(answer.len() as u32).to_le_bytes())?;
                sink.write_all(&answer[..half])?;
                sink.flush()?;
                hold_packed(&dir, tick, "partial", child.id(), answer.len(), half)?;
            }
        }
        if op == 7 && word(&answer, 44) == Some(6) {
            let marker = fs::read_to_string(dir.join("packed-response")).unwrap_or_default();
            if let Some((target, mode)) = marker.split_once(':')
                && target == format!("{} {tick}", std::process::id())
            {
                match mode {
                    "digest" => {
                        *answer.get_mut(52).ok_or("truncated packed digest")? ^= 1;
                    }
                    "length" => answer
                        .get_mut(48..52)
                        .ok_or("truncated packed length")?
                        .copy_from_slice(&u32::MAX.to_le_bytes()),
                    "loss" => {
                        child.kill()?;
                        let status = child.wait()?;
                        #[cfg(unix)]
                        if std::os::unix::process::ExitStatusExt::signal(&status) != Some(9) {
                            return Err("expected SIGKILL after the solved packed reply".into());
                        }
                        fs::write(
                            dir.join("solved-packed-loss"),
                            serde_json::to_vec(
                                &json!({"worker":child.id(),"relay":std::process::id(),"tick":tick,"reply_bytes":answer.len(),"status":status.to_string()}),
                            )?,
                        )?;
                        return Ok(());
                    }
                    _ => return Err("unknown packed receipt check".into()),
                }
            }
        }
        let offset = word(&cmd, 36).unwrap_or_default();
        let result_kind = word(&answer, 44);
        let site = match op {
            0 => {
                blob = Some(offset);
                None
            }
            1 if offset == PAGE => match blob {
                Some(0) => Some("recipe-page"),
                Some(1) => Some("source-page"),
                _ => None,
            },
            2 => match blob.take() {
                Some(0) => Some("recipe-finish"),
                Some(1) => Some("source-finish"),
                _ => None,
            },
            3 if result_kind == Some(2) => Some("solved"),
            4 if offset == PAGE => Some("successor-page"),
            4 if offset == PAGE * 2 => Some("successor-tail"),
            _ => None,
        };
        if let Some(site) = site
            && fs::read_to_string(dir.join("armed")).ok().as_deref() == Some(site)
        {
            fs::remove_file(dir.join("armed"))?;
            child.kill()?;
            let status = child.wait()?;
            #[cfg(unix)]
            if std::os::unix::process::ExitStatusExt::signal(&status) != Some(9) {
                return Err("child did not terminate with SIGKILL".into());
            }
            let mut event = OpenOptions::new()
                .create(true)
                .append(true)
                .open(dir.join("events"))?;
            writeln!(
                event,
                "{}",
                json!({"phase":site,"tick":tick,"pid":child.id(),"offset":offset,
                "request_bytes":cmd.len(),"consumed_reply_bytes":answer.len(),"status":status.to_string()})
            )?;
            return Ok(());
        }
        send(&mut sink, &answer)?;
    }
    drop(input);
    let _ = child.wait()?;
    Ok(())
}

pub(super) fn qualify(worker: &Path, scratch: &Path, count: usize, ticks: u32) -> Result<()> {
    let dir = tempfile::Builder::new()
        .prefix("gz-material-pages-")
        .tempdir_in(scratch)?;
    let exe = std::env::current_exe()?;
    let status = Command::new(&exe)
        .arg("page-owner")
        .arg(&exe)
        .arg(dir.path())
        .arg(count.to_string())
        .arg(ticks.to_string())
        .env("TMPDIR", dir.path())
        .env("GZ_EXAMPLE_REAL_WORKER", worker.canonicalize()?)
        .env("GZ_EXAMPLE_BREAKPOINTS", dir.path())
        .status()?;
    if !status.success() {
        if let Ok(events) = fs::read_to_string(dir.path().join("events")) {
            eprint!("{events}");
        }
        return Err(format!("managed page witness failed: {status}").into());
    }
    Ok(())
}

pub(super) fn owner(
    worker: &Path,
    dir: &Path,
    count: usize,
    ticks: u32,
    faults: bool,
) -> Result<()> {
    if faults && (count < 4090 || ticks < 90) {
        return Err("full fault witness needs >=4090 particles and >=90 ticks".into());
    }
    if ticks == 0 {
        return Err("choose a positive measurement length".into());
    }
    let sdf = world(count, 31)?;
    let mut local = setup(&sdf, None)?;
    let mut remote = setup(&sdf, Some(worker))?;
    if !faults {
        remote
            .world_mut()
            .resource_mut::<TrajectoryProcessPool>()
            .set_profiling(true);
    }
    let mut phases = Vec::<[u64; 6]>::new();
    let mut bytes = 0;
    let mut source_bytes = 0;
    let mut max_bytes = 0;
    let mut walls = Vec::new();
    let mut loads = 0;
    let mut cuts = 0;
    let mut last_state = None;
    for t in 0..ticks {
        let position = match t {
            32 => Some("recipe-page"),
            38 => Some("source-page"),
            44 => Some("source-finish"),
            50 => Some("solved"),
            56 => Some("successor-page"),
            62 => Some("successor-tail"),
            70 => Some("recipe-finish"),
            _ => None,
        }
        .filter(|_| faults);
        if matches!(t, 32 | 38 | 44 | 70) {
            let info = remote.committed_update_info()?;
            let slot = remote.world().resource::<TrajectoryProcessPool>().owner();
            trajectory_workers::select(&mut remote, info.epoch, info.iteration, 1 - slot)?;
        }
        // The last cold construction fault needs a vacant destination. Resetting recreates it.
        if t == 69 && faults {
            reset_loaded_world(&mut remote, |_| ())?;
            reset_loaded_world(&mut local, |_| ())?;
            equal(&observation(&remote)?, &observation(&local)?)?;
        }
        if let Some(site) = position {
            fs::write(dir.join("armed"), site)?;
        }
        if let Err(e) = remote.step_exact(1) {
            let detail = remote
                .world()
                .resource::<gz_sim_core::StepFault>()
                .error()
                .map(ToString::to_string);
            return Err(
                format!("public tick {t}, staged loss {position:?}, {e}: {detail:?}").into(),
            );
        }
        local.step_exact(1)?;
        let state = observation(&remote)?;
        equal(&state, &observation(&local)?)?;
        let stats = remote
            .world()
            .resource::<TrajectoryProcessPool>()
            .last()
            .unwrap();
        if let Some(p) = stats.profile {
            let ns = |d: std::time::Duration| d.as_nanos() as u64;
            phases.push([
                ns(stats.wall),
                ns(p.capture),
                ns(p.exchange),
                ns(p.install),
                ns(p.child_solve),
                ns(p.child_snapshot),
            ]);
        }
        if stats.restarted != position.is_some() {
            return Err(format!("staged recovery mismatch at {t}, site {position:?}").into());
        }
        loads += usize::from(state["guide_impulse_ns"].as_f64().unwrap() > 0.);
        cuts += 1;
        bytes += stats.successor_bytes;
        source_bytes += stats.source_bytes;
        max_bytes = max_bytes.max(stats.successor_bytes);
        walls.push(stats.wall.as_nanos() as u64);
        last_state = Some(state);
    }
    let events = if faults {
        fs::read_to_string(dir.join("events"))?
            .lines()
            .map(serde_json::from_str::<Value>)
            .collect::<std::result::Result<Vec<_>, _>>()?
    } else {
        Vec::new()
    };
    if faults && (events.len() != 7 || max_bytes <= 2 * PAGE as usize || loads == 0) {
        return Err("incomplete page cuts or unloaded scene".into());
    }
    walls.sort_unstable();
    let q = |p: usize| walls[(walls.len() * p).div_ceil(100) - 1];
    let mut percentiles = Vec::new();
    if !phases.is_empty() {
        for i in 0..6 {
            let mut a = phases.iter().map(|p| p[i]).collect::<Vec<_>>();
            a.sort_unstable();
            percentiles.push([
                a[(a.len() * 50).div_ceil(100) - 1],
                a[(a.len() * 95).div_ceil(100) - 1],
                a[(a.len() * 99).div_ceil(100) - 1],
            ]);
        }
    }
    let worst = phases.iter().enumerate().max_by_key(|(_, row)| row[0]);
    println!(
        "{}",
        json!({"stage_order":["wall","capture","exchange","install","child_solve","child_snapshot"],"stage_p50_p95_p99_ns":percentiles,
        "slowest_public_tick":worst.map(|(i,p)|(i+1,p)),"particles":count,"ticks":ticks,"exact_complete_cuts":cuts,"loaded_ticks":loads,
        "source_bytes":source_bytes,"successor_bytes":bytes,"largest_successor":max_bytes,"p50_ns":q(50),"p95_ns":q(95),"p99_ns":q(99),
        "final_material_sha256":last_state.unwrap()["material_and_public"],"cuts":events})
    );
    Ok(())
}
