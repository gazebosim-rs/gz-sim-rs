// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn decision(session: durable::Session, img: &[u8]) -> Result<Decision> {
    let ticket = Ticket::new(session, img);
    Ok(Decision {
        ticket,
        retired: ticket.retirement()?,
    })
}
#[test]
fn framing_rejects_bad_sizes_variants_trailing_metadata_and_partial_data() {
    let request = Request {
        session: [7; 16],
        id: 5,
        op: Op::Observe,
        omit_reply: false,
    };
    let mut good = Vec::new();
    wire::frame(&mut good, &request, &[]).unwrap();
    assert!(wire::read::<Request>(&*good).is_ok());
    let mut cases = vec![vec![], good[..8].to_vec(), good[..good.len() - 1].to_vec()];
    for (index, value) in [(0, b'X'), (8, 0), (9, 3), (10, 11), (11, 1)] {
        let mut corrupt = good.clone();
        corrupt[index] = value;
        cases.push(corrupt);
    }
    let mut trailing = good.clone();
    let size = u16::from_le_bytes(trailing[8..10].try_into().unwrap());
    trailing[8..10].copy_from_slice(&(size + 1).to_le_bytes());
    trailing.push(0);
    cases.push(trailing);
    let mut unknown = good.clone();
    unknown[12 + 16 + 8..12 + 16 + 8 + 4].copy_from_slice(&u32::MAX.to_le_bytes());
    cases.push(unknown);
    for (n, v) in cases.into_iter().enumerate() {
        assert!(wire::read::<Request>(&*v).is_err(), "case {n}")
    }
    assert!(wire::frame(Vec::new(), &request, &[0; 378]).is_err());
}
#[test]
fn deadlines_bind_replies_to_requests_and_detect_late_or_foreign_frames() -> Result<()> {
    // The receiver can discard a delayed earlier ID and preserve the matching reply.
    // A pipe with no payload in the configured window never yields an acknowledgement.
    let (tx, rx) = mpsc::sync_channel(2);
    let sample = |id| {
        Ok((
            Answer {
                id,
                reply: Reply::Denied(Denial::Fenced),
            },
            Vec::new(),
        ))
    };
    tx.send(sample(1))?;
    tx.send(sample(2))?;
    let get = |id, timeout| Client::receive_on(&rx, id, timeout);
    assert!(matches!(
        get(2, Duration::from_secs(1))?,
        Some((Reply::Denied(Denial::Fenced), _))
    ));
    assert!(get(3, Duration::from_millis(1))?.is_none());
    tx.send(sample(4))?;
    assert!(get(3, Duration::from_secs(1)).is_err());
    Ok(())
}
#[test]
fn immutable_session_retirement_and_decision_reject_stale_and_conflicting_cuts() -> Result<()> {
    let root = tempfile::tempdir()?;
    let a = root.path().join("a.bin");
    let b = root.path().join("b.bin");
    durable::start_session(&a)?;
    durable::start_session(&b)?;
    let mut tree = Tree::warm()?;
    let image = tree.image()?;
    let d = decision(durable::session(&a)?, &image)?;
    assert!(publish(&a, d, &image).is_err());
    durable::write_record(&a.with_extension("retired"), d, &image)?;
    publish(&a, d, &image)?;
    publish(&a, d, &image)?;
    let bytes = fs::read(&a)?;
    assert!(durable::write_record(&b.with_extension("retired"), d, &image).is_err());
    fs::copy(&a, &b)?;
    assert!(load_decision(&b).is_err());
    tree.step(None)?;
    let different = tree.image()?;
    let other = decision(d.ticket.session, &different)?;
    assert!(publish(&a, other, &different).is_err());
    assert!(durable::write_record(&a.with_extension("retired"), other, &different).is_err());
    assert_eq!(fs::read(&a)?, bytes);
    Ok(())
}
#[test]
fn ordered_journal_checks_session_contiguity_receipt_and_pending_identity() -> Result<()> {
    let root = tempfile::tempdir()?;
    let path = root.path().join("decision.bin");
    durable::start_session(&path)?;
    let image = Tree::warm()?.image()?;
    let t = Ticket::new(durable::session(&path)?, &image);
    let first = Step::new(CUT + 1, Some(0.25));
    let second = Step::new(CUT + 2, None);
    durable::intent(&path, t, second)?;
    assert!(durable::journal(&path, t).is_err());
    durable::intent(&path, t, first)?;
    assert!(durable::journal(&path, t).is_err());
    assert!(durable::intent(&path, t, Step::new(CUT + 1, None)).is_err());
    durable::receipt(&path, t, first, [1; 32])?;
    let journal = durable::journal(&path, t)?;
    assert_eq!(journal.confirmed, vec![(first, [1; 32])]);
    assert_eq!(journal.outstanding, Some(second));
    assert!(durable::receipt(&path, t, first, [2; 32]).is_err());
    let mut foreign = t;
    foreign.session[0] ^= 1;
    assert!(durable::journal(&path, foreign).is_err());
    let row = path.with_extension(format!("{}.receipt", first.tick));
    let mut data = fs::read(&row)?;
    *data.last_mut().unwrap() ^= 1;
    fs::write(&row, data)?;
    assert!(durable::journal(&path, t).is_err());
    Ok(())
}

#[test]
fn normal_transfer_wire_byte_accounting_uses_the_actual_framed_messages() -> Result<()> {
    let image = Tree::warm()?.image()?;
    let ticket = Ticket::new([1; 16], &image);
    let mut sent = 0;
    let mut received = 0;
    for (i, (label, op, reply, body_out, body_in)) in [
        (
            "freeze",
            Op::Freeze {
                epoch: 1,
                tick: CUT,
            },
            Reply::Frozen(ticket),
            &[][..],
            &image[..],
        ),
        (
            "prepare",
            Op::Prepare(ticket),
            Reply::Prepared(ticket),
            &image[..],
            &[][..],
        ),
        (
            "retire",
            Op::Retire(ticket),
            Reply::Retired(ticket.retirement()?),
            &[][..],
            &[][..],
        ),
        (
            "activate",
            Op::Activate(ticket),
            Reply::Activated(ticket),
            &[][..],
            &[][..],
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let req = Request {
            session: ticket.session,
            id: i as u64 + 1,
            op,
            omit_reply: false,
        };
        let mut a = Vec::new();
        let mut b = Vec::new();
        wire::frame(&mut a, &req, body_out)?;
        wire::frame(&mut b, &Answer { id: req.id, reply }, body_in)?;
        println!(
            "{label}: {} request bytes, {} reply bytes",
            a.len(),
            b.len()
        );
        sent += a.len();
        received += b.len();
    }
    assert_eq!(image.len(), 377);
    println!(
        "four clean handoff exchanges: {sent} request bytes, {received} reply bytes, {} total, 754 native bytes",
        sent + received
    );
    Ok(())
}
