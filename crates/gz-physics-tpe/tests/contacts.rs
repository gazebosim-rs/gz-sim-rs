// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// Original assertion: gz-physics/tpe/lib/src/CollisionDetector_TEST.cc.
use gz_physics::BodyState;
use gz_physics_tpe::{ContactError, ContactLimit, Contacts, ModelBounds, Motion, install};
use gz_sim_core::{Simulation, StepError};
use std::time::Duration;

fn motion(position: [f64; 3]) -> Motion {
    Motion::new(BodyState {
        position,
        velocity: [0.0; 3],
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0; 3],
    })
    .unwrap()
}
fn bounds(half: f64, fixed: bool, mask: u16) -> ModelBounds {
    ModelBounds::new([-half; 3], [half; 3], fixed, mask).unwrap()
}
fn simulation() -> Simulation {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    install(&mut sim).unwrap();
    sim
}

#[test]
fn overlap_touch_separation_static_and_masks_follow_native_model_policy() {
    for mode in 0..7 {
        let mut sim = simulation();
        let a = sim
            .world_mut()
            .spawn((
                motion([0.0; 3]),
                bounds(
                    2.0,
                    mode == 3 || mode == 4,
                    if mode == 5 {
                        1
                    } else if mode == 6 {
                        3
                    } else {
                        u16::MAX
                    },
                ),
            ))
            .id();
        let x = match mode {
            1 => 3.0,
            2 => 3.001,
            _ => 1.0,
        };
        let b = sim
            .world_mut()
            .spawn((
                motion([x, 0.0, 0.0]),
                bounds(
                    1.0,
                    mode == 3,
                    if mode == 5 || mode == 6 { 2 } else { u16::MAX },
                ),
            ))
            .id();
        sim.step_exact(1).unwrap();
        let pairs = sim.world().resource::<Contacts>().as_slice();
        if mode == 2 || mode == 3 || mode == 5 {
            assert!(pairs.is_empty(), "mode{mode}");
        } else {
            assert_eq!(pairs.len(), 1, "mode{mode}");
            assert_eq!(pairs[0].models, [a, b]);
            assert_eq!(
                pairs[0].point,
                [if mode == 1 { 2.0 } else { 1.0 }, 0.0, 0.0]
            );
        }
    }
}

#[test]
fn shrinking_bounds_filter_old_tree_envelopes_and_removed_generations_retire() {
    let mut sim = simulation();
    let a = sim
        .world_mut()
        .spawn((motion([0.0; 3]), bounds(10.0, false, u16::MAX)))
        .id();
    let b = sim
        .world_mut()
        .spawn((motion([5.0, 0.0, 0.0]), bounds(1.0, false, u16::MAX)))
        .id();
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Contacts>().as_slice().len(), 1);
    sim.world_mut()
        .entity_mut(a)
        .insert(bounds(1.0, false, u16::MAX));
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
    sim.world_mut()
        .entity_mut(b)
        .insert(motion([1.0, 0.0, 0.0]));
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<Contacts>().as_slice()[0].point,
        [0.5, 0.0, 0.0]
    );
    // Reserve this index explicitly; Bevy batches ordinary freed indices.
    let replacement = sim.world_mut().entity_mut(b).despawn_no_free();
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
    sim.world_mut()
        .spawn_at(
            replacement,
            (motion([1.0, 0.0, 0.0]), bounds(1.0, false, u16::MAX)),
        )
        .unwrap();
    assert_ne!(replacement, b);
    assert_eq!(replacement.index_u32(), b.index_u32());
    sim.step_exact(1).unwrap();
    let pair = sim.world().resource::<Contacts>().as_slice()[0];
    assert!(pair.models.contains(&a));
    assert!(pair.models.contains(&replacement));
    assert!(!pair.models.contains(&b));
    sim.reset().unwrap();
    // Replacement's constructor is at x=1, while b's constructor was x=5.
    assert_eq!(sim.world().resource::<Contacts>().as_slice().len(), 1);
    sim.world_mut()
        .entity_mut(replacement)
        .remove::<ModelBounds>();
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
}

#[test]
fn invalid_bounds_and_derived_overflow_are_typed_failures() {
    for (min, max) in [
        ([f64::NAN; 3], [1.0; 3]),
        ([-1.0; 3], [f64::INFINITY; 3]),
        ([2.0; 3], [1.0; 3]),
    ] {
        assert!(matches!(
            ModelBounds::new(min, max, false, 1),
            Err(ContactError::InvalidBounds)
        ));
    }
    let mut sim = simulation();
    let entity = sim
        .world_mut()
        .spawn((motion([f64::MAX; 3]), bounds(f64::MAX, false, u16::MAX)))
        .id();
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(
        sim.fault().unwrap().downcast_ref::<ContactError>(),
        Some(&ContactError::Overflow(entity))
    );
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
}

#[test]
fn contact_budget_fails_observably_without_replacing_the_last_complete_frame() {
    let mut sim = simulation();
    sim.world_mut().insert_resource(ContactLimit(1));
    for _ in 0..2 {
        sim.world_mut()
            .spawn((motion([0.0; 3]), bounds(1.0, false, u16::MAX)));
    }
    sim.step_exact(1).unwrap();
    let last = sim.world().resource::<Contacts>().as_slice().to_vec();
    assert_eq!(last.len(), 1);
    sim.world_mut()
        .spawn((motion([0.0; 3]), bounds(1.0, false, u16::MAX)));
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
    assert_eq!(sim.elapsed(), Duration::from_millis(1));
    assert_eq!(sim.world().resource::<Contacts>().as_slice(), last);
    assert_eq!(
        sim.fault().unwrap().downcast_ref::<ContactError>(),
        Some(&ContactError::LimitExceeded(1))
    );
}

// Original CollisionDetector.CheckStaticCollisionFiltering assertions.
#[test]
fn original_two_static_overlapping_boxes_produce_no_contacts() {
    let mut sim = simulation();
    sim.world_mut()
        .spawn((motion([1.0; 3]), bounds(2.0, true, u16::MAX)));
    sim.world_mut()
        .spawn((motion([0.0; 3]), bounds(2.0, true, u16::MAX)));
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
}

#[test]
fn shrinking_higher_index_bounds_rejects_a_stale_broadphase_candidate() {
    let mut sim = simulation();
    sim.world_mut()
        .spawn((motion([0.0; 3]), bounds(1.0, false, u16::MAX)));
    let b = sim
        .world_mut()
        .spawn((motion([5.0, 0.0, 0.0]), bounds(10.0, false, u16::MAX)))
        .id();
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().resource::<Contacts>().as_slice().len(), 1);
    sim.world_mut()
        .entity_mut(b)
        .insert(bounds(1.0, false, u16::MAX));
    sim.step_exact(1).unwrap();
    assert!(sim.world().resource::<Contacts>().as_slice().is_empty());
}

#[test]
fn finite_large_world_coordinates_keep_contact_points_finite() {
    let mut sim = simulation();
    for _ in 0..2 {
        sim.world_mut()
            .spawn((motion([1e308, 0.0, 0.0]), bounds(1.0, false, u16::MAX)));
    }
    sim.step_exact(1).unwrap();
    let contacts = sim.world().resource::<Contacts>().as_slice();
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].point, [1e308, 0.0, 0.0]);
    assert_ne!(contacts[0].models[0], contacts[0].models[1]);
}
