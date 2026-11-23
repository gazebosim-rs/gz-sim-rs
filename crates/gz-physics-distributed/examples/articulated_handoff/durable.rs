// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Single-host session, OS-backed exclusive leases and bounded no-clobber records.
use super::*;
use std::{
    io::{ErrorKind, Write},
    sync::atomic::{AtomicU64, Ordering},
};
pub(super) type Session = [u8; 16];
static TEMP: AtomicU64 = AtomicU64::new(0);

pub(super) fn session(path: &Path) -> Result<Session> {
    let mut f = fs::File::open(path.with_extension("run"))?;
    let mut v = [0; 16];
    f.read_exact(&mut v)?;
    if f.read(&mut [0; 1])? != 0 {
        return Err("invalid local run identity".into());
    }
    Ok(v)
}
pub(super) fn start_session(path: &Path) -> Result<()> {
    let mut v = [0; 16];
    fs::File::open("/dev/urandom")?.read_exact(&mut v)?;
    save(path.with_extension("run").as_path(), &v)
}
pub(super) fn try_lease(path: &Path, kind: &str) -> Result<Option<fs::File>> {
    let f = fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(path.with_extension(kind))?;
    match f.try_lock() {
        Ok(()) => Ok(Some(f)),
        Err(fs::TryLockError::WouldBlock) => Ok(None),
        Err(fs::TryLockError::Error(e)) => Err(e.into()),
    }
}
pub(super) fn controller(path: &Path) -> Result<fs::File> {
    try_lease(path, "controller")?
        .ok_or_else(|| "another local hinge controller already owns this run".into())
}
pub(super) fn save(path: &Path, bytes: &[u8]) -> Result<()> {
    // A partially written staging inode is never a decision. New processes use
    // fresh names; an existing final record is replayable only for identical bytes.
    let dir = path.parent().ok_or("missing hinge session directory")?;
    let (tmp, mut f) = loop {
        let n = TEMP.fetch_add(1, Ordering::Relaxed);
        let tmp = dir.join(format!(".hinge-{}-{n}.pending", std::process::id()));
        match fs::File::create_new(&tmp) {
            Ok(f) => break (tmp, f),
            Err(e) if e.kind() == ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(e.into()),
        }
    };
    let result = (|| -> Result<()> {
        f.write_all(bytes)?;
        f.sync_all()?;
        recovery::publication_edge(path, false)?;
        match fs::hard_link(&tmp, path) {
            Ok(()) => (),
            Err(e) if e.kind() == ErrorKind::AlreadyExists => {
                let mut actual = Vec::new();
                fs::File::open(path)?
                    .take(bytes.len() as u64 + 1)
                    .read_to_end(&mut actual)?;
                if actual != bytes {
                    return Err("conflicting immutable hinge record".into());
                }
            }
            Err(e) => return Err(e.into()),
        }
        recovery::publication_edge(path, true)?;
        fs::File::open(dir)?.sync_all()?;
        Ok(())
    })();
    fs::remove_file(tmp)?;
    result
}
pub(super) fn write_record(path: &Path, record: Decision, image: &[u8]) -> Result<()> {
    if record.ticket.session != session(path)?
        || !record.ticket.check(image)
        || record.retired != record.ticket.retirement()?
    {
        return Err("invalid retirement record".into());
    }
    let mut bytes = Vec::new();
    wire::frame(&mut bytes, &record, image)?;
    save(path, &bytes)
}
pub(super) fn committed(path: &Path) -> Result<(Decision, Vec<u8>)> {
    let d = load_decision(path)?;
    if load_decision(&path.with_extension("retired"))? != d {
        return Err("commit and source retirement disagree".into());
    }
    Ok(d)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
struct Entry {
    session: Session,
    cut: Hash,
    step: Step,
    receipt: Option<Hash>,
}
impl Entry {
    fn new(t: Ticket, step: Step, receipt: Option<Hash>) -> Self {
        Self {
            session: t.session,
            cut: t.image,
            step,
            receipt,
        }
    }
}
#[derive(Serialize, Deserialize)]
struct JournalHeader {
    value: Entry,
    checksum: Hash,
}
fn row_path(path: &Path, tick: u16, received: bool) -> PathBuf {
    let suffix = if received { "receipt" } else { "intent" };
    path.with_extension(format!("{tick}.{suffix}"))
}
fn put(path: &Path, row: Entry) -> Result<()> {
    let checksum = Sha256::digest(wire::encode(&row)?).into();
    let mut v = Vec::new();
    wire::frame(
        &mut v,
        &JournalHeader {
            value: row,
            checksum,
        },
        &[],
    )?;
    save(path, &v)
}
fn get(path: &Path) -> Result<Entry> {
    let mut f = fs::File::open(path)?;
    let (h, img): (JournalHeader, _) = wire::read(&mut f)?;
    if !img.is_empty()
        || h.checksum != Hash::from(Sha256::digest(wire::encode(&h.value)?))
        || f.read(&mut [0; 1])? != 0
    {
        return Err("invalid local command journal".into());
    }
    Ok(h.value)
}
pub(super) fn intent(path: &Path, t: Ticket, s: Step) -> Result<()> {
    if s.epoch != t.target_epoch
        || !(CUT + 1..=CUT + SPAN).contains(&s.tick)
        || s.effort
            .map(f64::from_bits)
            .is_some_and(|e| !e.is_finite() || e.abs() > 4.)
    {
        return Err("invalid proposed hinge interval".into());
    }
    put(&row_path(path, s.tick, false), Entry::new(t, s, None))
}
pub(super) fn receipt(path: &Path, t: Ticket, s: Step, digest: Hash) -> Result<()> {
    if get(&row_path(path, s.tick, false))? != Entry::new(t, s, None) {
        return Err("receipt does not match durable input".into());
    }
    put(
        &row_path(path, s.tick, true),
        Entry::new(t, s, Some(digest)),
    )
}
#[derive(Default)]
pub(super) struct Journal {
    pub confirmed: Vec<(Step, Hash)>,
    pub outstanding: Option<Step>,
}
pub(super) fn journal(path: &Path, t: Ticket) -> Result<Journal> {
    let mut result = Journal::default();
    let mut ended = false;
    for tick in CUT + 1..=CUT + SPAN {
        let ip = row_path(path, tick, false);
        let rp = row_path(path, tick, true);
        if !ip.try_exists()? {
            if rp.try_exists()? {
                return Err("receipt without accepted intention".into());
            }
            ended = true;
            continue;
        }
        if ended {
            return Err("gap or multiple outstanding intervals in hinge journal".into());
        }
        let input = get(&ip)?;
        if input.session != t.session
            || input.cut != t.image
            || input.step.epoch != t.target_epoch
            || input.step.tick != tick
            || input.receipt.is_some()
        {
            return Err("journal belongs to a different run, cut or interval".into());
        }
        if rp.try_exists()? {
            let received = get(&rp)?;
            let digest = received.receipt.ok_or("empty interval receipt")?;
            if received != Entry::new(t, input.step, Some(digest)) {
                return Err("receipt does not match journal input".into());
            }
            result.confirmed.push((input.step, digest));
        } else {
            result.outstanding = Some(input.step);
            ended = true;
        }
    }
    Ok(result)
}
