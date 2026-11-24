// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise a partially started pool, including a genuinely unresponsive bootstrap pipe.
#[cfg(target_os = "linux")]
mod linux {
    use clap::Parser;
    use gz_physics_distributed::{
        physics::{GroundPlane, SimulationStep, fixture},
        pool::{PoolNativeRecovery, WorkerPool},
    };
    use serde_json::{Value, json};
    use std::{
        env,
        error::Error,
        fs,
        os::unix::process::CommandExt,
        path::{Path, PathBuf},
        process::{self, Command},
        time::{Duration, Instant},
    };

    type Result<T> = std::result::Result<T, Box<dyn Error>>;
    #[derive(Parser)]
    struct Args {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, default_value_t=8, value_parser=clap::value_parser!(u8).range(3..=8))]
        width: u8,
        out: PathBuf,
    }

    fn worker_shim() -> Result<()> {
        let args: Vec<_> = env::args_os().skip(1).collect();
        let id = args
            .windows(2)
            .find(|p| p[0] == "--worker")
            .and_then(|p| p[1].to_str())
            .ok_or("missing worker ID")?
            .parse::<u8>()?;
        let records =
            PathBuf::from(env::var_os("GZ_BOOT_TEST_RECORDS").ok_or("missing PID directory")?);
        fs::write(records.join(id.to_string()), process::id().to_string())?;
        if id == env::var("GZ_BOOT_TEST_FAIL")?.parse::<u8>()? {
            match env::var("GZ_BOOT_TEST_MODE")?.as_str() {
                "exit" => process::exit(71),
                // Exceeds the real worker handshake timeout. The parent must kill this PID.
                "stall" => std::thread::sleep(Duration::from_secs(30)),
                _ => return Err("unknown bootstrap fault".into()),
            }
            return Err("stalled bootstrap escaped its parent timeout".into());
        }
        Err(
            Command::new(env::var_os("GZ_BOOT_TEST_WORKER").ok_or("missing real worker")?)
                .args(args)
                .exec()
                .into(),
        )
    }

    fn reject_and_check() -> Result<()> {
        let file = PathBuf::from(env::var_os("GZ_BOOT_TEST_ARCHIVE").ok_or("missing archive")?);
        let directory =
            PathBuf::from(env::var_os("GZ_BOOT_TEST_RECORDS").ok_or("missing PID records")?);
        let fail: u8 = env::var("GZ_BOOT_TEST_FAIL")?.parse()?;
        let width: u8 = env::var("GZ_BOOT_TEST_WIDTH")?.parse()?;
        let mode = env::var("GZ_BOOT_TEST_MODE")?;
        let bytes = fs::read(&file)?;
        let plan = PoolNativeRecovery::open(&file)?;
        let frame = plan.frame().clone();
        let at = Instant::now();
        let result = plan.resume(env::current_exe()?);
        let elapsed = at.elapsed();
        if result.is_ok() || fs::read(&file)? != bytes {
            return Err("failed partial restore published or altered its archive".into());
        }
        let error = result.err().unwrap().to_string();
        let mut starts = Vec::new();
        for record in fs::read_dir(directory)? {
            let record = record?;
            let owner: u8 = record
                .file_name()
                .to_str()
                .ok_or("invalid owner")?
                .parse()?;
            let pid: u32 = fs::read_to_string(record.path())?.parse()?;
            if Path::new("/proc").join(pid.to_string()).exists() {
                return Err(
                    format!("worker {owner}, PID {pid}, survived a failed bootstrap").into(),
                );
            }
            starts.push((owner, pid));
        }
        starts.sort_unstable();
        let count = (usize::from(fail / 2 + 1) * 2).min(usize::from(width));
        if starts.iter().map(|r| r.0).ne(0..count as u8)
            || (mode == "stall"
                && !(Duration::from_secs(4)..Duration::from_secs(15)).contains(&elapsed))
        {
            return Err(format!("unexpected bootstrap reach/time: {starts:?}, {elapsed:?}").into());
        }
        let real = PathBuf::from(env::var_os("GZ_BOOT_TEST_WORKER").ok_or("missing real worker")?);
        let mut resumed = PoolNativeRecovery::open(&file)?.resume(real)?;
        if resumed.frame() != &frame {
            return Err("clean retry changed the archived frame".into());
        }
        // Two separately restored pools must advance the accepted queued input exactly once.
        let real = PathBuf::from(env::var_os("GZ_BOOT_TEST_WORKER").ok_or("missing real worker")?);
        let mut reference = PoolNativeRecovery::open(&file)?.resume(real)?;
        for _ in 0..8 {
            if resumed.advance()? != reference.advance()? {
                return Err("successor diverged after failed startup".into());
            }
        }
        println!(
            "{}",
            json!({"owner":fail,"mode":mode,"children_reaped":starts,"elapsed_ms":elapsed.as_secs_f64()*1000.,
            "saved_tick":frame.tick,"retry_successors":8,"error":error})
        );
        Ok(())
    }

    pub fn main() -> Result<()> {
        if env::args_os().nth(1).is_some_and(|x| x == "--run") {
            return worker_shim();
        }
        if env::args_os().nth(1).is_some_and(|x| x == "check") {
            return reject_and_check();
        }
        let args = Args::parse();
        let worker = fs::canonicalize(args.worker)?;
        fs::create_dir(&args.out)?;
        let archive = args.out.join("committed.gzpool");
        let mut seeds = vec![vec![]; args.width.into()];
        let mut body = fixture()[1].clone();
        body.position = [0., 0., body.size[2] * 0.5];
        let id = body.id;
        seeds[0].push(body);
        let mut original = WorkerPool::spawn(
            &worker,
            2098,
            seeds,
            SimulationStep::Ms10,
            Some(GroundPlane::new(0.)?),
            32,
        )?;
        for _ in 0..12 {
            original.advance_in_field([0., 0., -9.81], &[])?;
        }
        original.transfer_component(id, args.width - 1)?;
        original.queue_impulse(id, [0.13, -0.23, 0.43])?;
        original.save_native_recovery(&archive)?;
        drop(original);
        let mut rows = Vec::new();
        for (id, mode) in (0..args.width)
            .map(|i| (i, "exit"))
            .chain([(args.width - 1, "stall")])
        {
            let directory = args.out.join(format!("{mode}-{id}"));
            fs::create_dir(&directory)?;
            let child = Command::new(env::current_exe()?)
                .arg("check")
                .env("GZ_BOOT_TEST_ARCHIVE", &archive)
                .env("GZ_BOOT_TEST_WIDTH", args.width.to_string())
                .env("GZ_BOOT_TEST_RECORDS", &directory)
                .env("GZ_BOOT_TEST_WORKER", &worker)
                .env("GZ_BOOT_TEST_FAIL", id.to_string())
                .env("GZ_BOOT_TEST_MODE", mode)
                .output()?;
            if !child.status.success() {
                return Err(format!(
                    "startup failure case {id}/{mode}: {}",
                    String::from_utf8_lossy(&child.stderr)
                )
                .into());
            }
            rows.push(serde_json::from_slice::<Value>(&child.stdout)?);
        }
        fs::write(
            args.out.join("summary.json"),
            serde_json::to_vec_pretty(&rows)?,
        )?;
        Ok(())
    }
}
#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    linux::main()
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("The child-reaping qualification needs Linux procfs.");
}
