// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{native::TreeWorld, *};
use std::{io, path::Path};

#[derive(Serialize, Deserialize)]
pub(super) struct Request {
    pub generation: u64,
    pub action: Action,
}
#[derive(Serialize, Deserialize)]
pub(super) enum Action {
    Stage(Box<Snapshot>),
    Activate([u8; 32]),
    Fence([u8; 32]),
    Queue {
        joint: u64,
        effort: f64,
    },
    Advance {
        effort: Option<(u64, f64)>,
    },
    AdvanceMany(Vec<(u64, f64)>),
    SetMotor {
        joint: u64,
        setting: Option<MotorSetting>,
    },
    AdvanceLoaded {
        efforts: Vec<(u64, f64)>,
        force: [f64; 3],
    },
    SphereGeometry(Vec<u64>),
    SamplePoints(Vec<ArticulatedPointSite>),
    CommitPoints {
        sites: Vec<ArticulatedPointSite>,
        certificate: [u8; 32],
        impulses: Vec<[f64; 3]>,
        advance: bool,
    },
}
#[derive(Serialize, Deserialize)]
pub(super) enum Answer {
    Acknowledged,
    State(Box<Snapshot>),
    Geometry(Vec<([f64; 3], f64)>),
    Basis(ContactPointBasis),
    Applied(Box<Snapshot>, ContactImpulseReceipt),
}
struct Resident {
    generation: u64,
    active: bool,
    digest: [u8; 32],
    world: TreeWorld,
}

/// Serve the negotiated v1 floating-tree profile on the managed child's private channel.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<FloatingTreeSpec>(slot, lease, FloatingTreeWorkerProfile::ScalarV1)
}
/// Serve the extended profile accepting multiple contributions in one native advance.
pub fn serve_vector(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<FloatingTreeSpec>(slot, lease, FloatingTreeWorkerProfile::VectorV2)
}
pub fn serve_stopped(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<StoppedTreeSpec>(slot, lease, FloatingTreeWorkerProfile::StoppedV1)
}
pub fn serve_contact(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<ContactTreeSpec>(slot, lease, FloatingTreeWorkerProfile::ContactV1)
}
pub fn serve_motor(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<MotorTreeSpec>(slot, lease, FloatingTreeWorkerProfile::MotorV1)
}
/// Serve one tree and its external fixed-floor contact graph.
pub fn serve_grounded(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<GroundedTreeSpec>(slot, lease, FloatingTreeWorkerProfile::GroundedV1)
}
pub fn serve_payload(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<PayloadTreeSpec>(slot, lease, FloatingTreeWorkerProfile::PayloadV2)
}
pub fn serve_exact_payload(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<PayloadTreeSpec>(slot, lease, FloatingTreeWorkerProfile::ExactPayloadV3)
}
pub fn serve_grasp(slot: u8, lease: &Path) -> io::Result<()> {
    serve_profile::<GraspTreeSpec>(slot, lease, FloatingTreeWorkerProfile::GraspV1)
}
fn serve_profile<S: TreeConstruction>(
    slot: u8,
    lease: &Path,
    profile: FloatingTreeWorkerProfile,
) -> io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(io::Error::other("invalid floating tree worker slot"));
    };
    crate::moving::worker::serve_profile(slot, lease, profile.magic(), |spec: S| {
        if spec
            .required_worker()
            .is_some_and(|required| required != profile)
        {
            return Err(invalid(
                "floating tree worker capability disagrees with its recipe",
            ));
        }
        TreeWorld::new(&spec)?;
        let mut resident = None::<Resident>;
        let mut highest_generation = None;
        Ok(move |tick, req: Request, steps: &mut u64| {
            let generation = req.generation;
            let result = (|| {
                if let Action::Stage(snapshot) = req.action {
                    if resident.is_some()
                        || highest_generation.is_some_and(|prev| prev >= generation)
                        || snapshot.state.tick != tick
                    {
                        return Err(invalid("floating tree staging lease or clock invalid"));
                    }
                    let world = TreeWorld::restore(&spec, &snapshot)?;
                    resident = Some(Resident {
                        generation,
                        active: false,
                        digest: snapshot.digest,
                        world,
                    });
                    highest_generation = Some(generation);
                    return Ok(Answer::Acknowledged);
                }
                let r = resident
                    .as_mut()
                    .ok_or(invalid("floating tree slot is empty"))?;
                if r.generation != generation || r.world.tick() != tick {
                    return Err(invalid("floating tree lease or clock mismatch"));
                }
                match req.action {
                    Action::Activate(d) if !r.active && r.digest == d => {
                        r.active = true;
                        Ok(Answer::Acknowledged)
                    }
                    Action::Fence(d) if r.active && r.digest == d => {
                        resident = None;
                        Ok(Answer::Acknowledged)
                    }
                    Action::SphereGeometry(ids)
                        if r.active && profile == FloatingTreeWorkerProfile::ContactV1 =>
                    {
                        Ok(Answer::Geometry(r.world.contact_spheres(&ids)?))
                    }
                    Action::SamplePoints(sites)
                        if r.active && profile == FloatingTreeWorkerProfile::ContactV1 =>
                    {
                        Ok(Answer::Basis(r.world.point_basis(&sites)?))
                    }
                    Action::CommitPoints {
                        sites,
                        certificate,
                        impulses,
                        advance,
                    } if r.active
                        && profile == FloatingTreeWorkerProfile::ContactV1
                        && (advance || tick > 0) =>
                    {
                        let report = r.world.commit_point_impulses(
                            &sites,
                            &impulses,
                            certificate,
                            r.digest,
                            generation,
                            tick,
                        )?;
                        if advance {
                            r.world.advance()?;
                            *steps = steps
                                .checked_add(1)
                                .ok_or(invalid("contact tree counter exhausted"))?;
                        }
                        let snapshot = r.world.capture()?;
                        r.digest = snapshot.digest;
                        Ok(Answer::Applied(Box::new(snapshot), report))
                    }
                    Action::SetMotor { joint, setting } if r.active && S::MOTORIZED => {
                        r.world.set_motor(joint, setting)?;
                        let snapshot = r.world.capture()?;
                        r.digest = snapshot.digest;
                        Ok(Answer::State(Box::new(snapshot)))
                    }
                    Action::Queue { joint, effort } if r.active => {
                        r.world.effort(joint, effort)?;
                        let snapshot = r.world.capture()?;
                        r.digest = snapshot.digest;
                        Ok(Answer::State(Box::new(snapshot)))
                    }
                    action @ (Action::Advance { .. }
                    | Action::AdvanceMany(_)
                    | Action::AdvanceLoaded { .. })
                        if r.active =>
                    {
                        match action {
                            Action::Advance {
                                effort: Some((joint, torque)),
                            } => r.world.effort(joint, torque)?,
                            Action::Advance { effort: None } => {}
                            Action::AdvanceLoaded { efforts, force }
                                if profile == FloatingTreeWorkerProfile::GraspV1 =>
                            {
                                r.world.apply_efforts(&efforts)?;
                                r.world.load_payload(force)?;
                            }
                            Action::AdvanceMany(efforts)
                                if profile != FloatingTreeWorkerProfile::ScalarV1 =>
                            {
                                r.world.apply_efforts(&efforts)?
                            }
                            _ => {
                                return Err(invalid(
                                    "floating tree worker does not support vector inputs",
                                ));
                            }
                        }
                        r.world.advance()?;
                        *steps = steps
                            .checked_add(1)
                            .ok_or(invalid("floating tree worker step counter exhausted"))?;
                        let snapshot = r.world.capture()?;
                        r.digest = snapshot.digest;
                        Ok(Answer::State(Box::new(snapshot)))
                    }
                    _ => Err(invalid("invalid floating tree ownership transition")),
                }
            })();
            if result.is_err() {
                resident = None;
            }
            result
        })
    })
}
