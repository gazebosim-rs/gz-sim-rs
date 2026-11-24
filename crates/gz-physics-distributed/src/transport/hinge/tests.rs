// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::{ArticulationLinkSpec, NativeArticulation};

fn spec() -> ArticulationSpec {
    let link = ArticulationLinkSpec {
        id: 41,
        radius: 0.1,
        mass: 1.5,
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
        inertia_center: [0.; 3],
        inertia_orientation: [0., 0., 0., 1.],
        inertia_tensor: [0.1, 0.1, 0.1, 0., 0., 0.],
    };
    ArticulationSpec {
        ground: None,
        model_id: 2,
        links: [
            link,
            ArticulationLinkSpec {
                id: 7,
                position: [1., 0., 0.],
                ..link
            },
        ],
        joint_id: 22,
        anchor: [0.5, 0., 0.],
        axis: [0., 0., 1.],
    }
}

fn request(sequence: u64, generation: u64, tick: u64, command: HingeCommand) -> HingeRequest {
    HingeRequest {
        run: 345,
        worker: 2,
        sequence,
        generation,
        tick,
        command,
    }
}

#[test]
fn retries_return_identical_images_and_rejected_commands_preserve_the_next_sequence()
-> Result<(), Box<dyn std::error::Error>> {
    let spec = spec();
    let mut reference = NativeArticulation::new(&spec)?;
    let mut slot = Participant::new(345, 2, spec);
    let image = reference.capture()?;
    let digest = Sha256::digest(&image).into();
    let stage = request(
        1,
        0,
        0,
        HingeCommand::Stage {
            image: image.clone(),
        },
    );
    for invalid in [
        HingeRequest {
            worker: 7,
            ..stage.clone()
        },
        HingeRequest {
            run: 9,
            ..stage.clone()
        },
        HingeRequest {
            sequence: 0,
            ..stage.clone()
        },
        HingeRequest {
            sequence: 2,
            ..stage.clone()
        },
        HingeRequest {
            tick: 1,
            ..stage.clone()
        },
    ] {
        assert!(slot.apply(invalid).is_err());
    }
    let mut other = spec;
    other.joint_id += 1;
    assert!(
        slot.apply(request(
            1,
            0,
            0,
            HingeCommand::Stage {
                image: NativeArticulation::new(&other)?.capture()?
            }
        ))
        .is_err()
    );
    let staged = slot.apply(stage.clone())?;
    let digest_of_stage = stage.digest()?;
    assert!(staged.matches(&stage, digest_of_stage, 0));
    let mut wrong_reply = staged.clone();
    wrong_reply.request_digest[0] ^= 1;
    assert!(!wrong_reply.matches(&stage, digest_of_stage, 0));
    let mut missing = staged.clone();
    missing.state = None;
    assert!(!missing.matches(&stage, digest_of_stage, 0));
    assert_eq!(slot.apply(stage)?.image, staged.image);
    for command in [
        HingeCommand::Observe,
        HingeCommand::Advance,
        HingeCommand::Activate { digest: [0; 32] },
    ] {
        assert!(slot.apply(request(2, 0, 0, command)).is_err());
    }
    let activate = request(2, 0, 0, HingeCommand::Activate { digest });
    slot.apply(activate.clone())?;
    assert_eq!(slot.apply(activate)?.image, image);
    let effort = request(
        3,
        0,
        0,
        HingeCommand::QueueEffort {
            joint_id: 22,
            effort: 7.25,
        },
    );
    assert!(
        slot.apply(request(
            3,
            0,
            0,
            HingeCommand::QueueEffort {
                joint_id: 41,
                effort: 9.
            }
        ))
        .is_err()
    );
    reference.effort(22, 7.25)?;
    let queued = slot.apply(effort.clone())?;
    assert_eq!(queued.image, reference.capture()?);
    assert_eq!(slot.apply(effort)?.image, queued.image);
    assert!(
        slot.apply(request(
            3,
            0,
            0,
            HingeCommand::QueueEffort {
                joint_id: 22,
                effort: 2.
            }
        ))
        .is_err()
    );
    reference.advance()?;
    let advance = request(4, 0, 0, HingeCommand::Advance);
    assert!(
        slot.apply(HingeRequest {
            generation: 1,
            ..advance.clone()
        })
        .is_err()
    );
    let advanced = slot.apply(advance.clone())?;
    assert_eq!(advanced.state, Some(reference.observe()));
    assert_eq!(advanced.image, reference.capture()?);
    assert_eq!(slot.apply(advance)?.image, advanced.image);
    let fence = request(
        5,
        0,
        1,
        HingeCommand::Fence {
            digest: Sha256::digest(&advanced.image).into(),
        },
    );
    assert!(
        slot.apply(request(5, 0, 1, HingeCommand::Fence { digest }))
            .is_err()
    );
    assert!(slot.apply(fence.clone())?.state.is_none());
    assert!(slot.apply(fence)?.image.is_empty());
    for (gen_id, tick, command) in [
        (0, 1, HingeCommand::Observe),
        (
            0,
            1,
            HingeCommand::Stage {
                image: advanced.image.clone(),
            },
        ),
        (1, 0, HingeCommand::Stage { image }),
        (0, 1, HingeCommand::Advance),
    ] {
        assert!(slot.apply(request(6, gen_id, tick, command)).is_err());
    }
    assert_eq!(
        slot.apply(request(
            6,
            1,
            1,
            HingeCommand::Stage {
                image: advanced.image.clone()
            }
        ))?
        .state,
        advanced.state
    );
    Ok(())
}

#[test]
fn ground_profile_frames_are_bounded_distinct_and_exact() -> Result<(), Box<dyn std::error::Error>>
{
    let isolated = spec();
    let mut grounded = isolated;
    grounded.ground = Some(crate::articulation::ArticulationGroundSpec {
        link_id: 1,
        size: [4., 6.],
        height: -0.1,
        gravity_z: -9.81,
    });
    let profile = Profile::for_spec(&grounded);
    let legacy = Profile::for_spec(&isolated);
    assert_ne!(profile.flag(), legacy.flag());
    assert_ne!(profile.offer(), legacy.offer());
    assert_ne!(scope(profile, 3, 0), scope(legacy, 3, 0));
    assert_ne!(spec_digest(&grounded)?, spec_digest(&isolated)?);
    assert!(!serde_json::to_string(&isolated)?.contains(r#""ground""#));
    let req = request(
        1,
        0,
        0,
        HingeCommand::Stage {
            image: vec![143; MAX_ARTICULATION_IMAGE],
        },
    );
    let op = Operation::Command(&req);
    let packed = profile.encode(&op)?;
    assert!(packed.len() < MAX_ARTICULATION_IMAGE + 128);
    assert!(legacy.decode::<Operation<HingeRequest>>(&packed).is_err());
    assert!(legacy.encode(&op).is_err());
    let Operation::Command(decoded) = profile.decode(&packed)? else {
        panic!("expected command")
    };
    let decoded: HingeRequest = decoded;
    assert_eq!(req.digest()?, decoded.digest()?);
    let mut invalid = packed;
    invalid.push(0);
    assert!(profile.decode::<Operation<HingeRequest>>(&invalid).is_err());
    assert!(
        profile
            .decode::<Operation<HingeRequest>>(&vec![0; profile.limit() + 1])
            .is_err()
    );
    let old = NativeArticulation::new(&isolated)?.capture()?;
    assert!(NativeArticulation::restore(&grounded, 0, &old).is_err());
    let mut w = NativeArticulation::new(&grounded)?;
    w.advance()?;
    let image = w.capture()?;
    assert!(grounded.verify_image(1, &image).is_ok());
    assert!(isolated.verify_image(1, &image).is_err());
    Ok(())
}
