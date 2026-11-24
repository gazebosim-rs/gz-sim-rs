// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Is a loaded moving support ever steady enough to reuse the previous solve exactly?
use super::observation;
use bevy_ecs::prelude::Entity;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, Velocity, WorldPose, queue_link_world_wrench};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const PULSE: usize = 121;
const END: usize = 220;

struct Pair {
    payload: Entity,
    deck: Entity,
    payload_body: BodyId,
    deck_body: BodyId,
    public: u64,
}

#[derive(Clone, Copy)]
struct Observation {
    payload: WorldPose,
    deck: WorldPose,
    velocity: Velocity,
    support: Option<BodyId>,
    impulse: f64,
}

#[derive(Default, serde::Serialize)]
struct Drift {
    loaded: usize,
    exact_pose: usize,
    exact_twist: usize,
    exact_impulse: usize,
    exact_all: usize,
    max_position_m: f64,
    max_quaternion_component: f64,
    max_linear_m_s: f64,
    max_angular_rad_s: f64,
    max_impulse_ns: f64,
}

fn difference<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0., f64::max)
}

impl Drift {
    fn observe(&mut self, before: Observation, after: Observation, deck: BodyId) {
        if before.support != Some(deck) || after.support != Some(deck) {
            return;
        }
        let predicted = std::array::from_fn(|i| {
            before.payload.translation[i] + (after.deck.translation[i] - before.deck.translation[i])
        });
        let dx = difference(predicted, after.payload.translation);
        let dq = difference(before.payload.rotation, after.payload.rotation);
        let dv = difference(before.velocity.linear, after.velocity.linear);
        let dw = difference(before.velocity.angular, after.velocity.angular);
        let dj = (before.impulse - after.impulse).abs();
        let pose = predicted.map(f64::to_bits) == after.payload.translation.map(f64::to_bits)
            && before.payload.rotation.map(f64::to_bits)
                == after.payload.rotation.map(f64::to_bits);
        let twist = before.velocity.linear.map(f64::to_bits)
            == after.velocity.linear.map(f64::to_bits)
            && before.velocity.angular.map(f64::to_bits)
                == after.velocity.angular.map(f64::to_bits);
        let reaction = before.impulse.to_bits() == after.impulse.to_bits();
        self.loaded += 1;
        self.exact_pose += usize::from(pose);
        self.exact_twist += usize::from(twist);
        self.exact_impulse += usize::from(reaction);
        self.exact_all += usize::from(pose && twist && reaction);
        self.max_position_m = self.max_position_m.max(dx);
        self.max_quaternion_component = self.max_quaternion_component.max(dq);
        self.max_linear_m_s = self.max_linear_m_s.max(dv);
        self.max_angular_rad_s = self.max_angular_rad_s.max(dw);
        self.max_impulse_ns = self.max_impulse_ns.max(dj);
    }
}

fn pairs(sim: &Simulation, count: usize) -> Vec<Pair> {
    let mut decks = BTreeMap::new();
    let mut payloads = BTreeMap::new();
    for entity in sim.world().iter_entities() {
        let (Some(handle), Some(name), Some(public)) = (
            entity.get::<BodyHandle>(),
            entity.get::<Name>(),
            entity.get::<EntityId>(),
        ) else {
            continue;
        };
        let model = name.0.split("::").nth(1).expect("loaded SDF model");
        if let Some(i) = model.strip_prefix("deck_") {
            let index: usize = i.parse().unwrap();
            if index.is_multiple_of(100) {
                decks.insert(index, (entity.id(), handle.id()));
            }
        } else if let Some(i) = model.strip_prefix("payload_") {
            payloads.insert(
                i.parse::<usize>().unwrap(),
                (entity.id(), handle.id(), public.get()),
            );
        }
    }
    assert_eq!(payloads.len(), count.div_ceil(100));
    payloads
        .into_iter()
        .map(|(i, (payload, payload_body, public))| {
            let (deck, deck_body) = decks[&i];
            Pair {
                payload,
                deck,
                payload_body,
                deck_body,
                public,
            }
        })
        .collect()
}

fn observe(sim: &Simulation, pairs: &[Pair]) -> Vec<Observation> {
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let mut loads = BTreeMap::new();
    for hit in backend
        .rigid_contact_observations()
        .into_iter()
        .filter(|h| h.active && h.normal_impulse > 0.)
    {
        for (body, other) in [(hit.a, hit.b), (hit.b, hit.a)] {
            loads
                .entry(body)
                .and_modify(|item: &mut (Option<BodyId>, f64)| {
                    // A second bearing surface disqualifies the single-support predictor.
                    item.0 = None;
                    item.1 += hit.normal_impulse;
                })
                .or_insert((Some(other), hit.normal_impulse));
        }
    }
    pairs
        .iter()
        .map(|pair| {
            let (support, impulse) = loads.get(&pair.payload_body).copied().unwrap_or((None, 0.));
            let world = sim.world();
            Observation {
                payload: *world.get::<WorldPose>(pair.payload).unwrap(),
                deck: *world.get::<WorldPose>(pair.deck).unwrap(),
                velocity: *world.get::<Velocity>(pair.payload).unwrap(),
                support,
                impulse,
            }
        })
        .collect()
}

pub(super) fn run(count: usize, pulse: bool) -> Result<Value> {
    if !(256..=100_000).contains(&count) {
        return Err("expected 256..=100000 decks".into());
    }
    let mut sim = observation::scene(count, true)?;
    let pairs = pairs(&sim, count);
    let mut previous = observe(&sim, &pairs);
    let mut drift = Drift::default();
    let mut first_lost = None;
    let mut first_new = None;
    let mut trace = Vec::new();
    for tick in 1..=END {
        if tick == PULSE && pulse {
            queue_link_world_wrench(&mut sim, pairs[0].public, [510., 0., 185.], [0.; 3])?;
        }
        sim.step_exact(1)?;
        let current = observe(&sim, &pairs);
        if (81..PULSE).contains(&tick) {
            for ((&before, &after), pair) in previous.iter().zip(&current).zip(&pairs) {
                drift.observe(before, after, pair.deck_body);
            }
        }
        let lane = current[0];
        if tick >= PULSE {
            if lane.impulse == 0. {
                first_lost.get_or_insert(tick);
            }
            if lane.support.is_some_and(|id| id != pairs[0].deck_body) {
                first_new.get_or_insert(tick);
            }
        }
        trace.push(json!({"tick":tick,"position_m":lane.payload.translation,"rotation":lane.payload.rotation,
            "linear_m_s":lane.velocity.linear,"angular_rad_s":lane.velocity.angular,"support":lane.support.map(BodyId::raw),"normal_ns":lane.impulse}));
        previous = current;
    }
    assert_eq!(observation::sample(&sim).time, sim.elapsed());
    Ok(
        json!({"decks":count,"payloads":pairs.len(),"pulse":pulse,"pulse_tick":PULSE,"initial_support":pairs[0].deck_body.raw(),
        "first_unloaded_tick":first_lost,"first_new_bearing_tick":first_new,"co_moving_hold":drift,"primary":trace}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_translation_counts_one_hold_and_a_new_support_disqualifies_it() {
        let deck = BodyId::from_raw(3);
        let pose = WorldPose {
            translation: [0., 0., 0.],
            rotation: [0., 0., 0., 1.],
        };
        let before = Observation {
            payload: pose,
            deck: pose,
            velocity: Velocity {
                linear: [1., 0., 0.],
                angular: [0.; 3],
            },
            support: Some(deck),
            impulse: 0.1,
        };
        let mut after = before;
        after.deck.translation[0] = 0.25;
        after.payload.translation[0] = 0.25;
        let mut drift = Drift::default();
        drift.observe(before, after, deck);
        after.support = Some(BodyId::from_raw(7));
        drift.observe(before, after, deck);
        drift.observe(after, before, deck);
        assert_eq!(
            (
                drift.loaded,
                drift.exact_pose,
                drift.exact_twist,
                drift.exact_impulse,
                drift.exact_all
            ),
            (1, 1, 1, 1, 1)
        );
    }

    #[test]
    fn a_settled_load_responds_to_one_wrench_and_lands_on_a_different_support() {
        assert!(run(255, false).is_err());
        let sham = run(256, false).unwrap();
        let loaded = run(256, true).unwrap();
        let first = sham["primary"].as_array().unwrap();
        let second = loaded["primary"].as_array().unwrap();
        assert_eq!(&first[..PULSE - 1], &second[..PULSE - 1]);
        assert_ne!(first[PULSE - 1], second[PULSE - 1]);
        assert_eq!(sham["co_moving_hold"], loaded["co_moving_hold"]);
        assert_eq!(sham["co_moving_hold"]["loaded"], 120);
        assert!(sham["co_moving_hold"]["exact_all"].as_u64().unwrap() < 120);
        assert!(sham["first_unloaded_tick"].is_null());
        assert!(sham["first_new_bearing_tick"].is_null());
        let flight = loaded["first_unloaded_tick"].as_u64().unwrap();
        let land = loaded["first_new_bearing_tick"].as_u64().unwrap();
        assert!(flight >= PULSE as u64 && land > flight);
        assert!(second[land as usize - 1]["normal_ns"].as_f64().unwrap() > 0.);
    }
}
