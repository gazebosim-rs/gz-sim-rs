// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cargo runner for the headless coverage profile. Each process owns its LLVM counters.
use std::{
    collections::BTreeSet,
    env,
    ffi::OsStr,
    io,
    os::unix::process::CommandExt,
    path::{Path, PathBuf},
    process::{Command, ExitCode},
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    thread,
};

const FINE_PICKUP: &str = concat!(
    "sheet_setdown::impact_clock::tests::",
    "fine_material_on_an_underresolved_rigid_pickup_does_not_emit_a_comparison"
);

fn foundations(executable: &Path, target: &Path) -> bool {
    executable.parent() == Some(target.join("debug/examples").as_path())
        && executable
            .file_name()
            .and_then(OsStr::to_str)
            .is_some_and(|name| {
                name.strip_prefix("local_robot_foundations-")
                    .is_some_and(|hash| {
                        hash.len() == 16 && hash.bytes().all(|byte| byte.is_ascii_hexdigit())
                    })
            })
}

fn cases(executable: &Path) -> io::Result<Vec<String>> {
    let list = Command::new(executable)
        .args(["--list", "--format", "terse"])
        .output()?;
    if !list.status.success() {
        return Err(io::Error::other(format!(
            "listing {} failed: {}",
            executable.display(),
            String::from_utf8_lossy(&list.stderr)
        )));
    }
    let mut names = BTreeSet::new();
    for row in std::str::from_utf8(&list.stdout)
        .map_err(io::Error::other)?
        .lines()
    {
        let name = row
            .strip_suffix(": test")
            .or_else(|| row.strip_suffix(": benchmark"))
            .filter(|name| !name.is_empty())
            .ok_or_else(|| io::Error::other(format!("unexpected libtest listing: {row}")))?;
        if !names.insert(name.to_owned()) {
            return Err(io::Error::other(format!("duplicate test: {name}")));
        }
    }
    if names.is_empty() {
        return Err(io::Error::other(
            "local robot integration suite has no tests",
        ));
    }
    let mut names: Vec<_> = names.into_iter().collect();
    // The measured microsecond material pickup is the longest single case.
    // Start it alongside independent work instead of leaving it at the tail.
    if let Some(index) = names.iter().position(|name| name == FINE_PICKUP) {
        names.swap(0, index);
    }
    Ok(names)
}

fn run_suite(executable: &Path, workers: usize) -> io::Result<ExitCode> {
    let names = cases(executable)?;
    let workers = workers.min(names.len());
    eprintln!(
        "Headless local robot coverage: {} exact libtest cases, up to {workers} concurrent processes",
        names.len()
    );
    let next = AtomicUsize::new(0);
    let failed = AtomicBool::new(false);
    thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                while let Some(name) = names.get(next.fetch_add(1, Ordering::Relaxed)) {
                    // The default libtest ignore policy, environment and working directory
                    // are inherited. The existing capture and process-group deadline still
                    // enclose every worker. Forward child output directly to that capture.
                    match Command::new(executable)
                        .args(["--exact", name, "--test-threads=1"])
                        .status()
                    {
                        Ok(status) if status.success() => (),
                        result => {
                            eprintln!("local robot test {name} failed: {result:?}");
                            failed.store(true, Ordering::Relaxed);
                        }
                    }
                }
            });
        }
    });
    Ok(if failed.load(Ordering::Relaxed) {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    })
}

fn run() -> io::Result<ExitCode> {
    let mut args = env::args_os().skip(1);
    let executable = PathBuf::from(
        args.next()
            .ok_or_else(|| io::Error::other("missing Cargo executable"))?,
    );
    let rest: Vec<_> = args.collect();
    if rest.is_empty()
        && let Some(target) = env::var_os("CARGO_TARGET_DIR")
        && foundations(&executable, Path::new(&target))
    {
        let workers = match env::var("RUST_TEST_THREADS") {
            Ok(value) => value
                .parse::<std::num::NonZeroUsize>()
                .map_err(io::Error::other)?
                .get(),
            Err(_) => thread::available_parallelism()?.get(),
        };
        return run_suite(&executable, workers.min(4));
    }
    Err(Command::new(executable).args(rest).exec())
}

fn main() -> ExitCode {
    run().unwrap_or_else(|error| {
        eprintln!("headless test runner: {error}");
        ExitCode::FAILURE
    })
}
