// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "partition-checkpoint")]
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, JointState, RotationSpec, Shape,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;
const DT: Duration = Duration::from_millis(1);
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn build() -> Result<(MultiFidelityBackend, [BodyId; 2], JointId)> {
    build_with_axis([0., 0., 1.], false)
}
fn build_with_axis(
    axis: [f64; 3],
    spatial_inertia: bool,
) -> Result<(MultiFidelityBackend, [BodyId; 2], JointId)> {
    install_tree(MultiFidelityBackend::new([0.; 3])?, axis, spatial_inertia)
}
fn install_tree(
    mut b: MultiFidelityBackend,
    axis: [f64; 3],
    spatial_inertia: bool,
) -> Result<(MultiFidelityBackend, [BodyId; 2], JointId)> {
    let mut spawn = |position, mass, radius, offset, angle: f64, inertia| {
        b.spawn_articulation_link(
            BodySpec {
                position,
                velocity: [0.21, -0.04, 0.08],
                mass: Some(mass),
                shape: Shape::Sphere { radius },
            },
            RotationSpec {
                orientation: [0., 0., (angle / 2.).sin(), (angle / 2.).cos()],
                angular_velocity: [0.; 3],
            },
            Some(Inertia {
                center: offset,
                orientation: if spatial_inertia {
                    [0.35f64.sin(), 0., 0., 0.35f64.cos()]
                } else {
                    [0., 0., 0., 1.]
                },
                tensor: if spatial_inertia {
                    [
                        inertia,
                        1.1 * inertia,
                        0.85 * inertia,
                        0.03 * inertia,
                        0.,
                        -0.02 * inertia,
                    ]
                } else {
                    [inertia, inertia, inertia, 0., 0., 0.]
                },
            }),
        )
    };
    let a = spawn([1.5, -0.7, 2.], 2., 0.16, [0.07, -0.03, 0.], 0.4, 0.12)?;
    let c = spawn([2.15, -0.45, 2.], 0.8, 0.12, [-0.04, 0.02, 0.], 1.1, 0.036)?;
    let j = b.add_reduced_revolute(Some(a), c, [1.83, -0.64, 2.], axis)?;
    Ok((b, [a, c], j))
}
fn state(b: &MultiFidelityBackend, ids: [BodyId; 2], j: JointId) -> ([BodyState; 2], JointState) {
    (
        ids.map(|id| b.state(id).unwrap()),
        b.joint_state(j).unwrap(),
    )
}
#[test]
fn moving_offset_tree_restores_at_a_bent_hinge_with_one_pending_effort() -> Result<()> {
    let (mut source, ids, j) = build()?;
    let (mut restored, ri, rj) = build()?;
    let original_reset = restored.initial_state();
    assert_eq!((ids, j), (ri, rj));
    for i in 0..120 {
        if i < 43 {
            source.add_joint_effort(j, 1.75)?;
        }
        source.step(DT)?;
    }
    let no_effort = source.checkpoint_floating_revolute()?;
    assert_eq!(no_effort.len(), 369);
    let (mut unforced, _, _) = build()?;
    let (mut ordinary, _, _) = build()?;
    let (mut doubled, _, _) = build()?;
    unforced.restore_floating_revolute(&no_effort)?;
    ordinary.restore_floating_revolute(&no_effort)?;
    ordinary.add_joint_effort(j, -0.8125)?;
    doubled.restore_floating_revolute(&no_effort)?;
    doubled.add_joint_effort(j, -1.625)?;
    unforced.step(DT)?;
    doubled.step(DT)?;
    source.add_joint_effort(j, -0.8125)?;
    let image = source.checkpoint_floating_revolute()?;
    restored.restore_floating_revolute(&image)?;
    assert_eq!(image.len(), 377);
    assert_eq!(restored.checkpoint_floating_revolute()?, image);
    // Query the immediate link collision, before either backend advances.
    let center = source.state(ids[1]).unwrap().position;
    let ray = [center[0], center[1], center[2] + 0.5];
    let cast = |b: &MultiFidelityBackend| b.cast_ray(ray, [0., 0., -1.], 2., Some(ids[0]));
    let hit = cast(&source)?.unwrap();
    assert!((hit - 0.38).abs() < 1e-10);
    assert_eq!(hit.to_bits(), cast(&restored)?.unwrap().to_bits());
    assert_eq!(state(&source, ids, j), state(&restored, ids, j));
    for i in 0..256 {
        if i == 99 {
            for b in [&mut source, &mut restored, &mut ordinary] {
                b.add_joint_effort(j, 0.375)?;
            }
        }
        source.step(DT)?;
        restored.step(DT)?;
        ordinary.step(DT)?;
        let applied = restored.checkpoint_floating_revolute()?;
        assert_eq!(applied.len(), 369);
        assert_eq!(
            ordinary.checkpoint_floating_revolute()?,
            applied,
            "ordinary one-shot input at interval {i}"
        );
        assert_eq!(
            source.checkpoint_floating_revolute()?,
            applied,
            "native coordinate bits at tick {i}"
        );
        if i == 0 {
            let no = unforced.joint_state(j).unwrap().velocity;
            let one = restored.joint_state(j).unwrap().velocity;
            let two = doubled.joint_state(j).unwrap().velocity;
            assert!(one < no - 1e-3);
            assert!(two < one - 1e-3);
            println!(
                "first-step qdot, zero/one/two held efforts: {no:.15}, {one:.15}, {two:.15} rad/s"
            );
        }
        assert_eq!(
            state(&source, ids, j),
            state(&restored, ids, j),
            "successor tick {i}"
        );
    }
    let joint = source.joint_state(j).unwrap();
    println!(
        "{} byte moving hinge, 256 exact successor states, q={} qdot={}",
        image.len(),
        joint.position,
        joint.velocity
    );
    source.reset()?;
    assert!(restored.owns_initial_state(&original_reset));
    restored.apply_initial(restored.prepare_initial(&original_reset)?)?;
    assert_eq!(state(&source, ids, j), state(&restored, ids, j));
    for _ in 0..24 {
        source.step(DT)?;
        restored.step(DT)?;
        assert_eq!(state(&source, ids, j), state(&restored, ids, j));
    }
    Ok(())
}

#[test]
fn repeated_cold_restores_preserve_multi_turn_motion_and_reverse_torque() -> Result<()> {
    let (mut control, ids, joint) = build()?;
    let (mut journey, _, _) = build()?;
    let mut high = 0.0_f64;
    let mut low = 0.0_f64;
    let mut restarts = 0;
    for tick in 0..8192 {
        let effort = match tick {
            0..128 => Some(4.),
            3500..3760 => Some(-4.5),
            _ if tick % 131 == 0 => Some(0.0625),
            _ => None,
        };
        if let Some(e) = effort {
            control.add_joint_effort(joint, e)?;
            journey.add_joint_effort(joint, e)?;
        }
        if tick > 0 && tick % 64 == 0 {
            let bytes = journey.checkpoint_floating_revolute()?;
            let (mut next, fresh_ids, fresh_joint) = build()?;
            assert_eq!((ids, joint), (fresh_ids, fresh_joint));
            next.restore_floating_revolute(&bytes)?;
            assert_eq!(bytes, next.checkpoint_floating_revolute()?);
            journey = next;
            restarts += 1;
        }
        control.step(DT)?;
        journey.step(DT)?;
        assert_eq!(
            control.checkpoint_floating_revolute()?,
            journey.checkpoint_floating_revolute()?,
            "native bits at tick {tick}"
        );
        let state = journey.joint_state(joint).unwrap();
        high = high.max(state.position);
        low = low.min(state.position);
    }
    assert_eq!(restarts, 127);
    assert!(high > 2. * std::f64::consts::TAU && low < -std::f64::consts::TAU);
    println!(
        "{restarts} fresh native restorations over 8192 one-ms intervals, continuous hinge position range [{low}, {high}] rad"
    );
    Ok(())
}

#[test]
fn oblique_hinge_axis_and_rotated_full_inertia_bind_the_native_recipe() -> Result<()> {
    let scale = 14f64.sqrt().recip();
    let axis = [scale, 2. * scale, 3. * scale];
    let (mut src, ids, j) = build_with_axis(axis, true)?;
    for _ in 0..73 {
        src.add_joint_effort(j, 1.625)?;
        src.step(DT)?;
    }
    let angular = src.state(ids[1]).unwrap().angular_velocity;
    assert!(angular.into_iter().all(|w| w.abs() > 0.01));
    let image = src.checkpoint_floating_revolute()?;
    let (mut wrong_axis, _, _) = build_with_axis([0., 0., 1.], true)?;
    let (mut wrong_inertia, _, _) = build_with_axis(axis, false)?;
    assert!(wrong_axis.restore_floating_revolute(&image).is_err());
    assert!(wrong_inertia.restore_floating_revolute(&image).is_err());
    let (mut restored, _, _) = build_with_axis(axis, true)?;
    restored.restore_floating_revolute(&image)?;
    assert_eq!(restored.checkpoint_floating_revolute()?, image);
    for _ in 0..512 {
        src.step(DT)?;
        restored.step(DT)?;
        assert_eq!(
            src.checkpoint_floating_revolute()?,
            restored.checkpoint_floating_revolute()?
        );
    }
    Ok(())
}

#[test]
fn retired_native_ids_and_future_allocator_survive_fresh_arena_reconstruction() -> Result<()> {
    let with_history = || -> Result<_> {
        let (mut base, stale, hinge) = build()?;
        assert!(base.remove(stale[0]));
        assert!(base.remove(stale[1]));
        assert!(base.joint_state(hinge).is_none());
        let (world, ids, joint) = install_tree(base, [0., 0., 1.], true)?;
        assert!(ids[0].raw() > stale[1].raw() && joint.raw() > hinge.raw());
        Ok((world, ids, joint, stale, hinge))
    };
    let (mut src, ids, joint, stale, retired_joint) = with_history()?;
    let (mut dest, _, _, _, _) = with_history()?;
    for _ in 0..52 {
        src.add_joint_effort(joint, 1.25)?;
        src.step(DT)?;
    }
    let bytes = src.checkpoint_floating_revolute()?;
    let (mut wrong_history, _, _) = build_with_axis([0., 0., 1.], true)?;
    assert!(wrong_history.restore_floating_revolute(&bytes).is_err());
    dest.restore_floating_revolute(&bytes)?;
    assert!(
        stale.iter().all(|id| dest.state(*id).is_none())
            && dest.joint_state(retired_joint).is_none()
    );
    for _ in 0..384 {
        src.step(DT)?;
        dest.step(DT)?;
        assert_eq!(
            src.checkpoint_floating_revolute()?,
            dest.checkpoint_floating_revolute()?
        );
    }
    let extra = BodySpec {
        position: [8., 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Sphere { radius: 0.2 },
    };
    let a = src.spawn_link(extra, Default::default(), None)?;
    let b = dest.spawn_link(extra, Default::default(), None)?;
    assert_eq!(a, b);
    assert_eq!(b.raw(), ids[1].raw() + 1);
    Ok(())
}

#[cfg(unix)]
#[path = "articulation_checkpoint/process.rs"]
mod process;
