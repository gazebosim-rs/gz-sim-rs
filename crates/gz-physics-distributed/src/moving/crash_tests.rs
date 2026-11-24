// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{
    fs,
    process::Command,
    time::{Duration, Instant},
};
const CHILD: &str = "moving::crash_tests::coordinator_process";
const ARCHIVE: &str = "barrier.gzmoving";
pub(crate) fn gate(name: &str) {
    if std::env::var("GZ_MOVING_TEST_GATE").as_deref() == Ok(name) {
        let marker = std::env::var_os("GZ_MOVING_TEST_MARKER").expect("crash marker");
        fs::write(marker, name).expect("write crash marker");
        std::thread::sleep(Duration::from_secs(30));
        panic!("test coordinator was not terminated");
    }
}
pub(crate) fn await_boundary(
    child: &mut std::process::Child,
    marker: &Path,
    edge: &str,
) -> io::Result<()> {
    let deadline = Instant::now() + Duration::from_secs(20);
    while !marker.try_exists()? {
        assert!(
            child.try_wait()?.is_none(),
            "coordinator exited before {edge}"
        );
        if Instant::now() > deadline {
            child.kill()?;
            child.wait()?;
            panic!("timeout at {edge}")
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    Ok(())
}
#[derive(Clone, Copy)]
enum Mutation {
    Step,
    Transfer,
    Effort(f64),
    Atomic(f64),
}
impl Mutation {
    fn label(self) -> &'static str {
        match self {
            Self::Step => "step",
            Self::Transfer => "transfer",
            Self::Effort(_) => "effort",
            Self::Atomic(_) => "atomic",
        }
    }
    fn apply(self, p: &mut MovingPool) -> Result<()> {
        match self {
            Self::Step => p.advance(),
            Self::Transfer => p.transfer(1),
            Self::Effort(value) => p.queue_effort(p.spec().hinge.joint_id, value),
            Self::Atomic(value) => p.advance_with_effort(p.spec().hinge.joint_id, value),
        }
    }
}
#[test]
#[ignore = "private coordinator crash harness child"]
fn coordinator_process() {
    let exe = std::env::var_os("GZ_MOVING_TEST_WORKER").unwrap();
    let directory = std::env::var_os("GZ_MOVING_TEST_SESSION").unwrap();
    let mut pool = MovingPool::resume(Path::new(&exe), Path::new(&directory)).unwrap();
    let op = match std::env::var("GZ_MOVING_TEST_OPERATION").unwrap().as_str() {
        "step" => Mutation::Step,
        "transfer" => Mutation::Transfer,
        "atomic" => Mutation::Atomic(
            std::env::var("GZ_MOVING_TEST_EFFORT")
                .unwrap()
                .parse()
                .unwrap(),
        ),
        "effort" => Mutation::Effort(
            std::env::var("GZ_MOVING_TEST_EFFORT")
                .unwrap()
                .parse()
                .unwrap(),
        ),
        _ => unreachable!(),
    };
    op.apply(&mut pool).unwrap();
    panic!("crash edge not reached");
}

pub(super) fn spec() -> MovingSpec {
    const SDF: &str = include_str!("../../../../examples/distributed_hinge_impact.sdf");
    let recipe = gz_sim_scene::distributed_hinge::distributed_moving_hinge_spec(
        SDF,
        "distributed_hinge_impact.sdf",
        "default",
    )
    .unwrap();
    serde_json::from_value(serde_json::to_value(recipe).unwrap()).unwrap()
}
fn seed(dir: &Path, bytes: &[u8]) -> io::Result<()> {
    fs::create_dir(dir)?;
    fs::write(dir.join(ARCHIVE), bytes)
}

fn check_edge(
    exe: &Path,
    initial: &[u8],
    op: Mutation,
    edge: &str,
    continuing_drive: Option<f64>,
) -> Result<()> {
    let tmp = tempfile::tempdir()?;
    let session = tmp.path().join("session");
    let reference = tmp.path().join("reference");
    let marker = tmp.path().join("boundary");
    seed(&session, initial)?;
    seed(&reference, initial)?;
    let mut uninterrupted = MovingPool::resume(exe, &reference)?;
    let epoch = uninterrupted.epoch();
    let tick = uninterrupted.observation().hinge.tick;
    let published = matches!(edge, "after-rename" | "after-sync");
    if published {
        if let Mutation::Atomic(value) = op {
            uninterrupted.queue_effort(uninterrupted.spec().hinge.joint_id, value)?;
            uninterrupted.advance()?;
        } else {
            op.apply(&mut uninterrupted)?;
        }
    }
    let mut command = Command::new(std::env::current_exe()?);
    if let Mutation::Effort(value) | Mutation::Atomic(value) = op {
        command.env("GZ_MOVING_TEST_EFFORT", value.to_string());
    }
    let mut child = command
        .args(["--exact", CHILD, "--ignored"])
        .env("GZ_MOVING_TEST_SESSION", &session)
        .env("GZ_MOVING_TEST_GATE", edge)
        .env("GZ_MOVING_TEST_MARKER", &marker)
        .env("GZ_MOVING_TEST_OPERATION", op.label())
        .spawn()?;
    await_boundary(&mut child, &marker, edge)?;
    assert!(
        matches!(MovingPool::resume(exe,&session),Err(ArticulationError::Io(e)) if e.kind()==io::ErrorKind::WouldBlock)
    );
    child.kill()?;
    child.wait()?;
    let mut recovered = MovingPool::resume(exe, &session)?;
    assert!(!session.join("barrier.pending.gzhinge").exists());
    println!(
        "recovered {} {edge} at tick {}, owners {:?}, generation {}",
        op.label(),
        recovered.observation().hinge.tick,
        recovered.owners(),
        recovered.generation()
    );
    if let Mutation::Atomic(value) = op
        && published
    {
        let joint = recovered.spec().hinge.joint_id;
        assert!(matches!(
            recovered.advance_with_effort_at(epoch, tick, joint, value),
            Err(ArticulationError::StaleInterval { .. })
        ));
    }
    for step in 0..=4 {
        if let Mutation::Atomic(value) = op
            && step == 1
            && !published
        {
            recovered.advance_with_effort_at(
                epoch,
                tick,
                recovered.spec().hinge.joint_id,
                value,
            )?;
            uninterrupted.queue_effort(uninterrupted.spec().hinge.joint_id, value)?;
            uninterrupted.advance()?;
        } else if step > 0 {
            for world in [&mut recovered, &mut uninterrupted] {
                if let Some(torque) = continuing_drive {
                    match world.pending_joint_effort() {
                        Some(saved) => assert_eq!(saved, torque),
                        None => world.queue_effort(world.spec().hinge.joint_id, torque)?,
                    }
                }
                world.advance()?;
            }
        }
        assert_eq!(
            recovered.pending_joint_effort(),
            uninterrupted.pending_joint_effort()
        );
        assert_eq!(recovered.observation(), uninterrupted.observation());
        assert_eq!(recovered.image(), uninterrupted.image());
        assert_eq!(recovered.owners(), uninterrupted.owners());
        assert_eq!(
            fs::read(session.join(ARCHIVE))?,
            fs::read(reference.join(ARCHIVE))?
        );
    }
    Ok(())
}

#[test]
#[ignore = "requires GZ_MOVING_TEST_WORKER pointing to the stock physics_worker binary"]
fn moving_controller_sigkill_rolls_back_partial_steps_and_publishes_whole_transitions() -> Result<()>
{
    let exe = PathBuf::from(std::env::var_os("GZ_MOVING_TEST_WORKER").expect("stock worker path"));
    let tmp = tempfile::tempdir()?;
    let mut pool = MovingPool::create(&exe, tmp.path(), 2, spec())?;
    let path = tmp.path().join(ARCHIVE);
    let mut separate = None;
    let mut join = None;
    let mut loaded = None;
    let mut release = None;
    for _ in 0..2400 {
        let before = fs::read(&path)?;
        let old = pool.owners();
        pool.advance()?;
        let now = pool.owners();
        if pool.observation().hinge.tick == 4 {
            separate = Some(before.clone());
        }
        if old[0] != old[1] && now[0] == now[1] {
            join = Some(before.clone());
        }
        if loaded.is_none()
            && pool
                .observation()
                .reactions
                .iter()
                .any(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
        {
            pool.queue_effort(pool.spec().hinge.joint_id, 2.25)?;
            loaded = Some(fs::read(&path)?);
        }
        if old[0] == old[1] && now[0] != now[1] {
            release = Some(before);
            break;
        }
    }
    assert!(
        release.is_some(),
        "no release at tick {}: routing {:?}, contacts {:?}",
        pool.cut.tick,
        pool.cut.next_routing(),
        pool.cut.reactions
    );
    // At early release the free sphere is still within the all-angle bound.
    // A newly queued torque must be honored by both routing and the native solve
    // even if the controller dies while rejoining the two independent roles.
    let freshly_split = fs::read(&path)?;
    pool.queue_effort(pool.spec().hinge.joint_id, -10_000.)?;
    let pending_rejoin = fs::read(&path)?;
    drop(pool);
    let separate = separate.unwrap();
    let join = join.expect("initial join");
    let loaded = loaded.expect("loaded contact");
    let release = release.expect("contact withdrawal");
    for (cut, op, edge) in [
        (&separate, Mutation::Step, "split-after-hinge"),
        (&join, Mutation::Step, "regroup-fenced"),
        (&join, Mutation::Step, "before-rename"),
        (&join, Mutation::Step, "after-rename"),
        (&loaded, Mutation::Transfer, "regroup-fenced"),
        (&loaded, Mutation::Transfer, "before-rename"),
        (&loaded, Mutation::Transfer, "after-rename"),
        (&loaded, Mutation::Effort(-0.375), "before-rename"),
        (&loaded, Mutation::Effort(-0.375), "after-rename"),
        (&release, Mutation::Step, "regroup-fenced"),
        (&release, Mutation::Step, "split-after-hinge"),
        (&release, Mutation::Step, "after-rename"),
        (&freshly_split, Mutation::Effort(-0.375), "after-rename"),
        (&pending_rejoin, Mutation::Step, "regroup-fenced"),
        (&pending_rejoin, Mutation::Step, "before-rename"),
        (&pending_rejoin, Mutation::Step, "after-rename"),
        (
            &separate,
            Mutation::Atomic(-0.375),
            "atomic-effort-accepted",
        ),
        (&separate, Mutation::Atomic(-0.375), "split-after-hinge"),
        (&join, Mutation::Atomic(0.), "regroup-fenced"),
        (&loaded, Mutation::Atomic(-0.375), "atomic-effort-accepted"),
        (&loaded, Mutation::Atomic(-0.375), "before-rename"),
        (&loaded, Mutation::Atomic(-0.375), "after-rename"),
        (&loaded, Mutation::Atomic(-0.375), "after-sync"),
        (&release, Mutation::Atomic(0.), "regroup-fenced"),
        (&release, Mutation::Atomic(0.), "split-after-hinge"),
        (&release, Mutation::Atomic(0.), "before-rename"),
        (&release, Mutation::Atomic(0.), "after-rename"),
        (&pending_rejoin, Mutation::Atomic(0.), "regroup-fenced"),
        (&pending_rejoin, Mutation::Atomic(0.), "after-sync"),
    ] {
        check_edge(&exe, cut, op, edge, None)?
    }
    Ok(())
}

#[test]
#[ignore = "requires GZ_MOVING_TEST_WORKER pointing to the stock physics_worker binary"]
fn moving_controller_sigkill_with_continuous_drive() -> Result<()> {
    let exe = PathBuf::from(std::env::var_os("GZ_MOVING_TEST_WORKER").expect("stock worker path"));
    let tmp = tempfile::tempdir()?;
    let mut pool = MovingPool::create(&exe, tmp.path(), 3, spec())?;
    let ledger = tmp.path().join(ARCHIVE);
    // Mutation::Effort queues the same command. A post-publication retry must
    // consume the command already in the cut instead of adding it a second time.
    let drive = -0.125;
    let mut loaded = None;
    for _ in 0..1600 {
        let joined = pool.cut.joined();
        pool.queue_effort(pool.spec().hinge.joint_id, drive)?;
        let pending = fs::read(&ledger)?;
        if loaded.is_none()
            && pool
                .observation()
                .reactions
                .iter()
                .any(|r| r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.)
        {
            loaded = Some(pending.clone());
        }
        pool.advance()?;
        if joined
            && !pool.cut.joined()
            && let Some(loaded) = loaded.take()
        {
            let released = fs::read(&ledger)?;
            pool.queue_effort(pool.spec().hinge.joint_id, drive)?;
            let driven_split = fs::read(&ledger)?;
            drop(pool);
            for (cut, op, edge) in [
                (&loaded, Mutation::Transfer, "regroup-fenced"),
                (&loaded, Mutation::Transfer, "after-rename"),
                (&pending, Mutation::Step, "regroup-fenced"),
                (&pending, Mutation::Step, "split-after-hinge"),
                (&pending, Mutation::Step, "before-rename"),
                (&pending, Mutation::Step, "after-rename"),
                (&released, Mutation::Effort(drive), "before-rename"),
                (&released, Mutation::Effort(drive), "after-rename"),
                (&driven_split, Mutation::Step, "split-after-hinge"),
            ] {
                check_edge(&exe, cut, op, edge, Some(drive))?;
            }
            return Ok(());
        }
    }
    Err(invalid(
        "driven hinge did not release after a loaded contact",
    ))
}

#[test]
fn both_pool_profiles_reject_crossed_sessions_before_contacting_workers() -> Result<()> {
    use crate::articulation::ArticulationPool;
    let tmp = tempfile::tempdir()?;
    let dir = tmp.path();
    let exe = dir.join("unreachable-worker");
    let spec = spec();
    Disk::lock(dir, true)?.publish(&Cut::new(spec, 2)?)?;
    assert!(matches!(
        ArticulationPool::open_scene(&exe, dir, Some(2), spec.hinge),
        Err(ArticulationError::Invalid(_))
    ));
    assert!(matches!(
        ArticulationPool::create(&exe, dir, 2, spec.hinge),
        Err(ArticulationError::Invalid(_))
    ));
    assert!(matches!(
        ArticulationPool::resume(&exe, dir),
        Err(ArticulationError::Invalid(_))
    ));
    let bytes = fs::read(dir.join(ARCHIVE))?;
    fs::write(dir.join("barrier.gzhinge"), b"owned by hinge pool")?;
    assert!(matches!(
        MovingPool::open_scene(&exe, dir, Some(2), spec),
        Err(ArticulationError::Invalid(_))
    ));
    assert!(matches!(
        MovingPool::create(&exe, dir, 2, spec),
        Err(ArticulationError::Invalid(_))
    ));
    assert!(matches!(
        MovingPool::resume(&exe, dir),
        Err(ArticulationError::Invalid(_))
    ));
    assert_eq!(fs::read(dir.join(ARCHIVE))?, bytes);
    Ok(())
}

#[test]
fn routing_encloses_whole_tick_and_requires_outward_clearance_and_drained_contact() -> Result<()> {
    let mut cut = Cut::new(spec(), 2)?;
    cut.owners = [0, 1];
    cut.tick = 1;
    cut.sphere.position = [3., 0., 0.5];
    cut.sphere.velocity = [0.; 3];
    assert_eq!(cut.next_routing(), Routing::Keep);
    // Both endpoints of this 6 m motion clear the entire hinge. A test of
    // just those endpoints would miss its passage completely.
    cut.sphere.velocity = [-6000., 0., 0.];
    assert_eq!(cut.next_routing(), Routing::Join);
    cut.sphere.position[0] = -3.;
    cut.sphere.velocity = [0.; 3];
    assert_eq!(cut.next_routing(), Routing::Keep);

    cut.owners = [0, 0];
    cut.sphere.position = [3., 0., 0.5];
    cut.sphere.velocity = [1., 0., 0.];
    assert_eq!(cut.next_routing(), Routing::CheckRelease);
    cut.reactions[1].warmstart_normal_impulse = 1.;
    assert_eq!(cut.next_routing(), Routing::Keep);
    cut.reactions[1] = Default::default();
    cut.reactions[1].active = true;
    assert_eq!(cut.next_routing(), Routing::Keep);
    cut.reactions[1] = Default::default();
    cut.sphere.position = [0.65, 0.14, 0.5];
    cut.sphere.velocity = [0., 1., 0.];
    assert_eq!(cut.next_routing(), Routing::Keep);
    Ok(())
}
