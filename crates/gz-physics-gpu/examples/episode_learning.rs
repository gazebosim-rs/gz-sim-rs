// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Train and evaluate a tabular velocity-tracking policy with asynchronous mixed GPU episodes.
//! cargo run --release -p gz-physics-gpu --example episode_learning -- --adapter NAME --out NEW_DIRECTORY [--capture NEW_REPLAY] [--seed INTEGER]
#[path = "support/sphere_pair/mod.rs"]
pub mod pair;
#[path = "support/sphere_plane/mod.rs"]
pub mod plane;
pub use pair::reference;
#[path = "support/episodes/capture.rs"]
mod capture;
#[path = "support/episodes/native.rs"]
mod native;
#[path = "support/episodes/plot.rs"]
mod plot;
#[path = "support/episodes/runner.rs"]
mod runner;
#[path = "support/supported_pair/fixture.rs"]
pub mod support;
#[path = "support/episodes/task.rs"]
mod task;
use gz_physics_gpu::simulation::{batch_boundary, checkpoint_mixed};
use runner::Runner;
use std::{error::Error, path::PathBuf};
use task::{COUNT, EVALUATE, Mode, Policy, TRAIN};
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let (mut adapter, mut out, mut capture, mut seed) = (None, None, None, None);
    while let Some(k) = args.next() {
        let v = args.next().ok_or("missing argument")?;
        match k.as_str(){"--adapter" if adapter.is_none()=>adapter=Some(v),"--out" if out.is_none()=>out=Some(PathBuf::from(v)),"--capture" if capture.is_none()=>capture=Some(PathBuf::from(v)),"--seed" if seed.is_none()=>seed=Some(v.parse::<u64>()?),_=>return Err("expected --adapter NAME --out NEW_DIRECTORY [--capture NEW_REPLAY] [--seed INTEGER]".into())}
    }
    let adapter = adapter.ok_or("select hardware by name with --adapter")?;
    let out = out.ok_or("supply --out NEW_DIRECTORY")?;
    std::fs::create_dir(&out)?;
    let seed = seed.unwrap_or(20261119);
    let mut train = Runner::new(&adapter, seed, Mode::Train, Policy::default(), false)?;
    let mut replay = Some(Runner::new(
        &adapter,
        seed,
        Mode::Train,
        Policy::default(),
        true,
    )?);
    let holdout = seed ^ 0x7171_c3c3_8a8a_b4b4;
    let path = out.join("queued-episode.bin");
    let mut saved = false;
    for k in 0..TRAIN {
        let primary = train.stage(COUNT)?;
        if let Some(other) = &mut replay {
            let secondary = other.stage(73)?;
            assert_eq!(primary, secondary, "policy command mismatch at {k}");
        }
        if k >= 257 && !saved && !primary.actions.is_empty() && !primary.resets.is_empty() {
            let size = train.save(&path)?;
            let restored = Runner::load(&adapter, &path)?;
            assert_eq!(
                checkpoint_mixed(&restored.sim)?.to_bytes()?,
                checkpoint_mixed(&train.sim)?.to_bytes()?
            );
            train.same(&restored);
            println!(
                "durable mid-episode checkpoint {size} B at tick {k}: {} selected resets, {} queued impulses",
                primary.resets.len(),
                primary.actions.len()
            );
            replay = Some(restored);
            saved = true;
        }
        train.step(true)?;
        if let Some(other) = &mut replay {
            other.step(false)?;
            train.same(other);
        }
    }
    assert!(saved);
    train.report("training");
    println!(
        "All {TRAIN} full ID-ordered observations matched 73-ID minibatches, with reverse admission then fresh-device checkpoint recovery. Learned Q values {:?}",
        train.agent.policy.q
    );
    std::fs::write(
        out.join("policy.json"),
        serde_json::to_vec_pretty(&train.agent.policy)?,
    )?;
    drop(replay);
    let policy = train.agent.policy.clone();
    drop(train);
    let mut greedy = Runner::new(&adapter, holdout, Mode::Greedy, policy.clone(), true)?;
    let mut coast = Runner::new(&adapter, holdout, Mode::Coast, policy, false)?;
    let mut native_policy = native::NativeRun::new(greedy.states());
    let mut native_coast = native::NativeRun::new(coast.states());
    let mut series = plot::Trajectory::default();
    series.add(0, &greedy, &coast);
    let mut archive = capture.as_ref().map(|_| capture::Capture::default());
    if let Some(a) = &mut archive {
        a.sample(0, &greedy, &coast);
    }
    for k in 1..=EVALUATE {
        let policy = greedy.stage(COUNT)?;
        let idle = coast.stage(COUNT)?;
        assert_eq!(
            policy.resets, idle.resets,
            "paired evaluation populations diverged"
        );
        native_policy.edit(&policy);
        native_coast.edit(&idle);
        greedy.step(true)?;
        coast.step(true)?;
        native_policy.step(greedy.states(), &greedy.agent.episodes);
        native_coast.step(coast.states(), &coast.agent.episodes);
        series.add(k, &greedy, &coast);
        if let Some(a) = &mut archive {
            a.sample(k, &greedy, &coast);
        }
    }
    let native_learned = native_policy.report("held-out greedy");
    let native_idle = native_coast.report("matched held-out no-action");
    for (learned, idle) in native_learned.into_iter().zip(native_idle) {
        assert!(learned < 0.65 * idle);
    }
    series.write(&out)?;
    greedy.report("held-out greedy");
    coast.report("matched held-out no-action");
    for (learned, base) in greedy.agent.error().into_iter().zip(coast.agent.error()) {
        assert!(
            learned < 0.65 * base,
            "held-out tracking error must improve by at least 35% in each profile"
        );
    }
    assert_eq!(batch_boundary(&greedy.sim).tick, EVALUATE);
    println!(
        "train seed {seed}, held-out seed {holdout}; 2,048 environments, 27 signed-error/profile states, three physical actions, fixed frozen-policy evaluation."
    );
    if let (Some(a), Some(p)) = (archive, capture) {
        a.write(&p)?;
        println!("native held-out replay {}", p.display());
    }
    Ok(())
}
