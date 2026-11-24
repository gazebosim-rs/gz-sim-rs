// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;
use std::io::Write;
pub(super) fn need(ok: bool, context: &str) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(context.to_owned().into())
    }
}
pub(super) fn output(path: &Path, value: &impl Serialize) -> Result<()> {
    fs::File::create_new(path)?.write_all(&serde_json::to_vec(value)?)?;
    Ok(())
}
fn elapsed(start: Instant) -> u64 {
    start.elapsed().as_nanos() as u64
}
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
pub(super) fn expected_denial(c: &mut Client, op: Op, b: &[u8], d: Denial) -> Result<()> {
    let r = c.call(op, b)?.0;
    need(
        matches!(r,Reply::Denied(reason) if reason==d),
        &format!("expected {d:?}, got {r:?}"),
    )
}
pub(super) fn boot(src: &mut Client, dst: &mut Client) -> Result<()> {
    need(
        matches!(
            src.call(Op::Boot, &[])?.0,
            Reply::Status {
                phase: Phase::Active,
                epoch: 1,
                tick: CUT
            }
        ),
        "source boot",
    )?;
    need(
        matches!(
            dst.call(Op::Inspect, &[])?.0,
            Reply::Status {
                phase: Phase::Idle,
                ..
            }
        ),
        "target admission",
    )
}
#[derive(Debug, Clone, Copy, Serialize)]
pub(super) struct Times {
    freeze_ns: u64,
    prepare_ns: u64,
    retire_ns: u64,
    publish_ns: u64,
    activate_ns: u64,
}
impl Times {
    fn sum(self) -> u64 {
        self.freeze_ns + self.prepare_ns + self.retire_ns + self.publish_ns + self.activate_ns
    }
}
pub(super) fn transfer(
    src: &mut Client,
    dst: &mut Client,
    path: &Path,
    inject: bool,
    mut reached: impl FnMut(Stage) -> Result<()>,
) -> Result<(Ticket, Vec<u8>, Times)> {
    let freeze = Op::Freeze {
        epoch: 1,
        tick: CUT,
    };
    if inject {
        src.omit(freeze, &[])?;
    }
    let start = Instant::now();
    let (Reply::Frozen(ticket), image) = src.call(freeze, &[])? else {
        return Err("missing frozen image".into());
    };
    let freeze_ns = elapsed(start);
    reached(Stage::Frozen)?;
    need(ticket.check(&image), "source freeze receipt")?;
    if inject {
        let stale = Op::Advance(Step {
            epoch: 1,
            tick: CUT + 1,
            effort: None,
        });
        expected_denial(src, stale, &[], Denial::Fenced)?;
        expected_denial(
            dst,
            Op::Advance(Step::new(CUT + 1, None)),
            &[],
            Denial::Fenced,
        )?;
        let mut bad = image.clone();
        bad[70] ^= 1;
        expected_denial(dst, Op::Prepare(ticket), &bad, Denial::Invalid)?;
        expected_denial(
            dst,
            Op::Prepare(Ticket::new(ticket.session, &bad)),
            &bad,
            Denial::Invalid,
        )?;
        let mut foreign = ticket;
        foreign.session[0] ^= 1;
        expected_denial(dst, Op::Prepare(foreign), &image, Denial::Invalid)?;
        let mut stale_epoch = ticket;
        stale_epoch.target_epoch = 1;
        expected_denial(dst, Op::Prepare(stale_epoch), &image, Denial::Invalid)?;
        let mut wrong = ticket;
        wrong.bodies = [1, 0];
        expected_denial(dst, Op::Prepare(wrong), &image, Denial::Invalid)?;
        dst.omit(Op::Prepare(ticket), &image)?;
    }
    let start = Instant::now();
    need(
        matches!(dst.call(Op::Prepare(ticket),&image)?.0,Reply::Prepared(t)if t==ticket),
        "target prepare",
    )?;
    let prepare_ns = elapsed(start);
    reached(Stage::Prepared)?;
    if inject {
        expected_denial(dst, Op::Activate(ticket), &[], Denial::NoDecision)?;
        expected_denial(
            dst,
            Op::Advance(Step::new(CUT + 1, None)),
            &[],
            Denial::Fenced,
        )?;
        src.omit(Op::Retire(ticket), &[])?;
    }
    let start = Instant::now();
    let Reply::Retired(retired) = src.call(Op::Retire(ticket), &[])?.0 else {
        return Err("missing retirement".into());
    };
    let retire_ns = elapsed(start);
    reached(Stage::Retired)?;
    if inject {
        let mut obsolete = Client::start(0, path)?;
        expected_denial(&mut obsolete, Op::Boot, &[], Denial::Fenced)?;
    }
    let start = Instant::now();
    publish(path, Decision { ticket, retired }, &image)?;
    let publish_ns = elapsed(start);
    reached(Stage::Decided)?;
    if inject {
        dst.omit(Op::Activate(ticket), &[])?;
    }
    let start = Instant::now();
    need(
        matches!(dst.call(Op::Activate(ticket),&[])?.0,Reply::Activated(t) if t==ticket),
        "target activate",
    )?;
    let activate_ns = elapsed(start);
    reached(Stage::Activated)?;
    Ok((
        ticket,
        image,
        Times {
            freeze_ns,
            prepare_ns,
            retire_ns,
            publish_ns,
            activate_ns,
        },
    ))
}
#[derive(Serialize)]
pub(super) struct Frame {
    offset: u16,
    logical_tick: u16,
    time_ns: u64,
    worker_local_clock_ns: u64,
    slot: Option<u8>,
    owner_epoch: u32,
    pid: u32,
    native_sha256: String,
    state: Observation,
}
pub(super) fn reference(tree: &Tree, tick: u16) -> Result<Frame> {
    Ok(Frame {
        offset: tick - CUT,
        logical_tick: tick,
        time_ns: u64::from(tick) * 1_000_000,
        worker_local_clock_ns: tree.time_ns(),
        slot: None,
        owner_epoch: 0,
        pid: std::process::id(),
        native_sha256: hex(&Sha256::digest(tree.image()?)),
        state: tree.observe(),
    })
}
pub(super) fn actual(reply: Reply, pid: u32, slot: u8) -> Result<Frame> {
    let Reply::Observed {
        epoch,
        tick,
        local_time_ns,
        digest,
        state,
    } = reply
    else {
        return Err(format!("missing observation, received {reply:?}").into());
    };
    Ok(Frame {
        offset: tick.checked_sub(CUT).ok_or("old frame")?,
        logical_tick: tick,
        time_ns: u64::from(tick) * 1_000_000,
        worker_local_clock_ns: local_time_ns,
        slot: Some(slot),
        owner_epoch: epoch,
        pid,
        native_sha256: hex(&digest),
        state,
    })
}
pub(super) fn compare(a: &Frame, b: &Frame) -> Result<()> {
    need(
        a.logical_tick == b.logical_tick
            && a.native_sha256 == b.native_sha256
            && wire::encode(&a.state)? == wire::encode(&b.state)?,
        &format!(
            "source vs participant native divergence at {}",
            a.logical_tick
        ),
    )
}
pub(super) fn replay(dst: &mut Client, path: &Path, journal: &[(Step, Hash)]) -> Result<()> {
    let (commit, image) = durable::committed(path)?;
    let t = commit.ticket;
    need(
        matches!(dst.call(Op::Prepare(t),&image)?.0,Reply::Prepared(x) if x==t),
        "fresh target prepare",
    )?;
    need(
        matches!(dst.call(Op::Activate(t),&[])?.0,Reply::Activated(x) if x==t),
        "fresh target activation",
    )?;
    for (cmd, digest) in journal {
        let r = dst.call(Op::Advance(*cmd), &[])?.0;
        need(
            matches!(r,Reply::Observed{epoch:2,tick,digest:x,..} if tick==cmd.tick&&x==*digest),
            "replayed committed step receipt",
        )?;
    }
    Ok(())
}
pub(super) fn run(dir: &Path) -> Result<()> {
    fs::create_dir(dir)?;
    let decision = dir.join("decision.bin");
    durable::start_session(&decision)?;
    let _controller = durable::controller(&decision)?;
    let mut a = Client::start(0, &decision)?;
    let mut b = Client::start(1, &decision)?;
    let mut control = Tree::warm()?;
    boot(&mut a, &mut b)?;
    let mut duplicate_source = Client::start(0, &decision)?;
    expected_denial(&mut duplicate_source, Op::Boot, &[], Denial::Busy)?;
    need(
        durable::controller(&decision).is_err(),
        "duplicate controller acquired session",
    )?;
    drop(duplicate_source);
    let reference_cut = reference(&control, CUT)?;
    let old = actual(a.call(Op::Observe, &[])?.0, a.child.id(), 0)?;
    compare(&reference_cut, &old)?;
    let (ticket, image, timing) = transfer(&mut a, &mut b, &decision, true, |_| Ok(()))?;
    need(
        control.image()? == image,
        "native file vs uninterrupted control at transfer",
    )?;
    let mut duplicate_target = Client::start(1, &decision)?;
    need(
        matches!(duplicate_target.call(Op::Prepare(ticket),&image)?.0, Reply::Prepared(t) if t==ticket),
        "duplicate target prepare",
    )?;
    expected_denial(
        &mut duplicate_target,
        Op::Activate(ticket),
        &[],
        Denial::Busy,
    )?;
    expected_denial(
        &mut duplicate_target,
        Op::Advance(Step::new(CUT + 1, None)),
        &[],
        Denial::Fenced,
    )?;
    drop(duplicate_target);
    let run_session = b.session;
    b.session[0] ^= 1;
    expected_denial(&mut b, Op::Observe, &[], Denial::Fenced)?;
    b.session = run_session;
    let target_cut = actual(b.call(Op::Observe, &[])?.0, b.child.id(), 1)?;
    compare(&reference_cut, &target_cut)?;
    expected_denial(
        &mut a,
        Op::Advance(Step {
            epoch: 1,
            tick: CUT + 1,
            effort: None,
        }),
        &[],
        Denial::Fenced,
    )?;
    expected_denial(
        &mut b,
        Op::Advance(Step {
            epoch: 1,
            tick: CUT + 1,
            effort: None,
        }),
        &[],
        Denial::Fenced,
    )?;
    expected_denial(
        &mut b,
        Op::Advance(Step::new(CUT + 2, None)),
        &[],
        Denial::Fenced,
    )?;
    expected_denial(
        &mut b,
        Op::Advance(Step::new(CUT + 1, Some(f64::NAN))),
        &[],
        Denial::Invalid,
    )?;
    let source_pid = a.kill_wait()?;
    let mut stale_source = Client::start(0, &decision)?;
    expected_denial(&mut stale_source, Op::Boot, &[], Denial::Fenced)?;
    let stale_pid = stale_source.kill_wait()?;
    let mut journal = Vec::<(Step, Hash)>::with_capacity(usize::from(SPAN));
    let mut original = vec![reference_cut];
    let mut handed = vec![target_cut];
    let mut failures = Vec::new();
    let mut digest = Sha256::new();
    digest.update(wire::encode(&old.state)?);
    for offset in 1..=SPAN {
        let tick = CUT + offset;
        let input = (offset == 100).then_some(0.375);
        let step = Step::new(tick, input);
        durable::intent(&decision, ticket, step)?;
        control.step(input)?;
        let expected = reference(&control, tick)?;
        if matches!(offset, 1 | 137) {
            b.omit(Op::Advance(step), &[])?;
            // Only the diagnostic observation is delivered. No step receipt entered
            // the coordinator journal for this interval before the participant dies.
            let observed = actual(b.call(Op::Observe, &[])?.0, b.child.id(), 1)?;
            compare(&expected, &observed)?;
            let killed = b.kill_wait()?;
            b = Client::start(1, &decision)?;
            let durable = durable::journal(&decision, ticket)?;
            need(
                durable.confirmed == journal && durable.outstanding == Some(step),
                "persisted unknown step or committed history changed",
            )?;
            replay(&mut b, &decision, &durable.confirmed)?;
            failures.push(json!({
                "lost_reply_at_tick": tick,
                "killed_pid": killed,
                "fresh_pid": b.child.id(),
                "committed_steps_replayed": journal.len(),
                "uncommitted_native_receipt_observed": observed.native_sha256,
            }));
        }
        if offset == 100 {
            b.omit(Op::Advance(step), &[])?;
        }
        let reply = b.call(Op::Advance(step), &[])?.0;
        let Reply::Observed {
            digest: receipt, ..
        } = reply
        else {
            return Err("unconfirmed step".into());
        };
        let observed = actual(reply, b.child.id(), 1)?;
        compare(&expected, &observed)?;
        if offset == 100 {
            let duplicate = actual(b.call(Op::Advance(step), &[])?.0, b.child.id(), 1)?;
            compare(&expected, &duplicate)?;
            expected_denial(
                &mut b,
                Op::Advance(Step::new(tick, None)),
                &[],
                Denial::Conflict,
            )?;
        }
        durable::receipt(&decision, ticket, step, receipt)?;
        journal.push((step, receipt));
        digest.update(wire::encode(&observed.state)?);
        original.push(expected);
        handed.push(observed);
    }
    let durable = durable::journal(&decision, ticket)?;
    need(
        durable.confirmed == journal && durable.outstanding.is_none(),
        "final durable journal mismatch",
    )?;
    let (cut, _) = durable::committed(&decision)?;
    need(cut.ticket == ticket, "durable decision changed")?;
    let manifest = json!({
        "schema": "gz-hinge-transfer/1",
        "session": hex(&ticket.session),
        "exclusive_source_and_target_duplicate_refusals": true,
        "session_bound_durable_step_receipts": journal.len(),
        "native_profile": "GZART001",
        "native_cut_bytes": image.len(),
        "physical_origin_tick": CUT,
        "dt_ns": 1_000_000,
        "cut_time_ns": 120_000_000,
        "model_id": 2,
        "source_slot": 0,
        "target_slot": 1,
        "cut_owner_epochs": [1,2],
        "held_effort_nm": fixture::PENDING,
        "future_effort": {"from_offset": 99, "to_offset": 100, "nm": 0.375},
        "source_pid": source_pid,
        "refused_stale_source_pid": stale_pid,
        "initial_target_pid": handed[0].pid,
        "last_target_pid": b.child.id(),
        "hard_losses": failures,
        "exact_frames": original.len(),
        "committed_journal_entries": journal.len(),
        "observations_sha256": hex(&digest.finalize()),
        "native_cut_sha256": hex(&ticket.image),
        "durable_decision_bytes": fs::metadata(&decision)?.len(),
        "receipt_loss": ["freeze","prepare","retire","activate","step at offsets 1,100,137"],
        "faulted_phase_ns": timing,
    });
    output(&dir.join("source-cut.json"), &old)?;
    output(&dir.join("reference.json"), &original)?;
    output(&dir.join("handoff.json"), &handed)?;
    output(&dir.join("summary.json"), &manifest)?;
    println!("{}", serde_json::to_string_pretty(&manifest)?);
    Ok(())
}
pub(super) fn bench(dir: &Path, trials: usize, storage: Option<&Path>) -> Result<()> {
    if !(1..=256).contains(&trials) {
        return Err("use one to 256 independent transfer trials".into());
    }
    fs::create_dir(dir)?;
    let placement = storage.unwrap_or(dir);
    let scratch = tempfile::Builder::new()
        .prefix("gz-hinge-benchmark-")
        .tempdir_in(placement)?;
    let mut timings = Vec::new();
    let mut step_times = Vec::new();
    let mut durable_steps = Vec::new();
    let mut native = Vec::new();
    let matched = Tree::warm()?;
    let baseline = matched.image()?;
    let components = || -> Result<[u64; 4]> {
        let start = Instant::now();
        let image = matched.image()?;
        let capture = elapsed(start);
        let start = Instant::now();
        let mut dest = Tree::fresh()?;
        let construction = elapsed(start);
        let start = Instant::now();
        dest.install(&image)?;
        let restore = elapsed(start);
        let start = Instant::now();
        dest.step(None)?;
        let step = elapsed(start);
        Ok([capture, construction, restore, step])
    };
    for i in 0..trials {
        let path = scratch.path().join(format!("decision-{i}.bin"));
        durable::start_session(&path)?;
        let _controller = durable::controller(&path)?;
        let mut src = Client::start(0, &path)?;
        let mut dst = Client::start(1, &path)?;
        boot(&mut src, &mut dst)?;
        if i.is_multiple_of(2) {
            native.push(components()?);
        }
        let (ticket, image, times) = transfer(&mut src, &mut dst, &path, false, |_| Ok(()))?;
        need(
            image == baseline,
            "benchmark cut differs from numerical reference",
        )?;
        timings.push(times);
        let op = Step::new(CUT + 1, None);
        let full = Instant::now();
        durable::intent(&path, ticket, op)?;
        let start = Instant::now();
        let Reply::Observed { digest, tick, .. } = dst.call(Op::Advance(op), &[])?.0 else {
            return Err("worker did not commit benchmark interval".into());
        };
        step_times.push(elapsed(start));
        need(tick == CUT + 1, "benchmark advanced the wrong step")?;
        durable::receipt(&path, ticket, op, digest)?;
        durable_steps.push(elapsed(full));
        if !i.is_multiple_of(2) {
            native.push(components()?);
        }
    }
    fn pct(mut vals: Vec<u64>) -> [u64; 3] {
        vals.sort();
        let n = vals.len();
        [50, 95, 99].map(|p| vals[(p * n).div_ceil(100) - 1])
    }
    let stat = |f: fn(Times) -> u64| pct(timings.iter().copied().map(f).collect());
    let cpu = fs::read_to_string("/proc/self/status").ok().and_then(|s| {
        s.lines().find_map(|l| {
            l.strip_prefix("Cpus_allowed_list:")
                .map(|v| v.trim().to_owned())
        })
    });
    let component = |index| pct(native.iter().map(|row| row[index]).collect());
    let report = json!({
        "trials":trials,"units":"ns","percentiles":[50,95,99],"cpu_affinity":cpu,"storage_parent":placement,"warm_cut_bytes":377,
        "freeze":stat(|x|x.freeze_ns),"prepare":stat(|x|x.prepare_ns),"durable_retire":stat(|x|x.retire_ns),
        "durable_publish":stat(|x|x.publish_ns),"activate":stat(|x|x.activate_ns),"transfer_total":stat(Times::sum),
        "first_worker_step_reply":pct(step_times.clone()),"first_durable_interval":pct(durable_steps.clone()),
        "capture_native_image":component(0),"fresh_sdf_world":component(1),"native_restore_into_fresh":component(2),"local_bevy_first_step":component(3),
        "raw_phase_samples":timings,"raw_worker_steps":step_times,"raw_durable_steps":durable_steps,"raw_native_components":native,
    });
    output(&dir.join("timing.json"), &report)?;
    println!(
        "{} trials transfer p50/p95/p99 {:?} ns",
        trials,
        stat(Times::sum)
    );
    Ok(())
}
