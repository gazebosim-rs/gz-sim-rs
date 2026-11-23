// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the coordinator alive while actual participant processes disappear in flight.
use super::*;
use run::{actual, boot, compare, need, reference, replay};
use serde_json::json;
#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
enum Loss {
    SourceBeforeExportReply,
    TargetPrepared,
    BothPrepared,
    SourceAfterRetire,
    TargetAfterActivate,
}
fn frozen(src: &mut Client) -> Result<(Ticket, Vec<u8>)> {
    let (Reply::Frozen(t), img) = src.call(
        Op::Freeze {
            epoch: 1,
            tick: CUT,
        },
        &[],
    )?
    else {
        return Err("frozen source did not export its image".into());
    };
    need(t.check(&img), "wrong freeze ticket")?;
    Ok((t, img))
}
fn prepare(dst: &mut Client, ticket: Ticket, img: &[u8]) -> Result<()> {
    need(
        matches!(dst.call(Op::Prepare(ticket),img)?.0,Reply::Prepared(t) if t==ticket),
        "prepared target",
    )
}
pub(super) fn qualify(dir: &Path) -> Result<()> {
    fs::create_dir(dir)?;
    let mut reports = Vec::new();
    for loss in [
        Loss::SourceBeforeExportReply,
        Loss::TargetPrepared,
        Loss::BothPrepared,
        Loss::SourceAfterRetire,
        Loss::TargetAfterActivate,
    ] {
        let case = dir.join(format!("{loss:?}"));
        fs::create_dir(&case)?;
        let path = case.join("decision.bin");
        durable::start_session(&path)?;
        let _controller = durable::controller(&path)?;
        let mut source = Client::start(0, &path)?;
        let mut target = Client::start(1, &path)?;
        boot(&mut source, &mut target)?;
        let mut killed = Vec::new();
        let mut control = Tree::warm()?;
        if loss == Loss::SourceBeforeExportReply {
            source.omit(
                Op::Freeze {
                    epoch: 1,
                    tick: CUT,
                },
                &[],
            )?;
            need(
                matches!(
                    source.call(Op::Inspect, &[])?.0,
                    Reply::Status {
                        phase: Phase::Frozen,
                        ..
                    }
                ),
                "lost source export was not frozen",
            )?;
            killed.push(source.kill_wait()?);
            source = Client::start(0, &path)?;
            need(
                matches!(
                    source.call(Op::Boot, &[])?.0,
                    Reply::Status {
                        phase: Phase::Active,
                        ..
                    }
                ),
                "unpublished source could not rebuild",
            )?;
        }
        let (t, img) = frozen(&mut source)?;
        need(img == control.image()?, "regenerated source cut diverged")?;
        prepare(&mut target, t, &img)?;
        if loss == Loss::BothPrepared {
            killed.push(source.kill_wait()?);
            killed.push(target.kill_wait()?);
            source = Client::start(0, &path)?;
            need(
                matches!(
                    source.call(Op::Boot, &[])?.0,
                    Reply::Status {
                        phase: Phase::Active,
                        ..
                    }
                ),
                "lost prepared source did not rebuild",
            )?;
            let restored = frozen(&mut source)?;
            need(
                restored == (t, img.clone()),
                "lost frozen tree changed its native cut",
            )?;
        }
        if loss == Loss::TargetPrepared {
            killed.push(target.kill_wait()?);
        }
        if matches!(loss, Loss::TargetPrepared | Loss::BothPrepared) {
            target = Client::start(1, &path)?;
            prepare(&mut target, t, &img)?;
        }
        let d = if loss == Loss::SourceAfterRetire {
            source.omit(Op::Retire(t), &[])?;
            need(
                matches!(
                    source.call(Op::Inspect, &[])?.0,
                    Reply::Status {
                        phase: Phase::Retired,
                        ..
                    }
                ),
                "missing retirement before source death",
            )?;
            killed.push(source.kill_wait()?);
            let (d, from_disk) = load_decision(&path.with_extension("retired"))?;
            need(
                d.ticket == t && from_disk == img,
                "source retirement failed to outlive its PID",
            )?;
            d
        } else {
            let Reply::Retired(retired) = source.call(Op::Retire(t), &[])?.0 else {
                return Err("missing retirement".into());
            };
            Decision { ticket: t, retired }
        };
        publish(&path, d, &img)?;
        if loss == Loss::TargetAfterActivate {
            target.omit(Op::Activate(t), &[])?;
            need(
                matches!(
                    target.call(Op::Inspect, &[])?.0,
                    Reply::Status {
                        phase: Phase::Active,
                        ..
                    }
                ),
                "target never activated",
            )?;
            killed.push(target.kill_wait()?);
            target = Client::start(1, &path)?;
            replay(&mut target, &path, &[])?;
        } else {
            need(
                matches!(target.call(Op::Activate(t),&[])?.0, Reply::Activated(x)if x==t),
                "target activation",
            )?;
        }
        compare(
            &reference(&control, CUT)?,
            &actual(target.call(Op::Observe, &[])?.0, target.child.id(), 1)?,
        )?;
        let mut measured = Sha256::new();
        for offset in 1..=32 {
            let effort = (offset == 17).then_some(0.375);
            let step = Step::new(CUT + offset, effort);
            durable::intent(&path, t, step)?;
            control.step(effort)?;
            let reply = target.call(Op::Advance(step), &[])?.0;
            let Reply::Observed { digest, .. } = reply else {
                return Err("target refused future interval".into());
            };
            compare(
                &reference(&control, step.tick)?,
                &actual(reply, target.child.id(), 1)?,
            )?;
            durable::receipt(&path, t, step, digest)?;
            measured.update(digest);
        }
        let log = durable::journal(&path, t)?;
        need(
            log.confirmed.len() == 32 && log.outstanding.is_none(),
            "incomplete future after participant death",
        )?;
        reports.push(json!({
            "loss":loss,"coordinator":std::process::id(),"killed_participants":killed,
            "committed_target":target.child.id(),"cut":CUT,"exact_future_intervals":32,
            "native_receipt_sequence_sha256":format!("{:x}",measured.finalize()),
        }));
    }
    let first = &reports[0]["native_receipt_sequence_sha256"];
    need(
        reports
            .iter()
            .all(|v| &v["native_receipt_sequence_sha256"] == first),
        "participant loss trajectories differ",
    )?;
    run::output(&dir.join("results.json"), &reports)?;
    println!(
        "Five transfer windows, six hard participant losses, 160 exact future intervals. The live coordinator preserved one owner and identical ordered physics receipts."
    );
    Ok(())
}
