// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_core::assign_id;
use std::time::Duration;

fn model(sim: &mut Simulation, name: &str, parent: Option<u64>) -> u64 {
    let parent = parent.map(|id| entity_by_id(sim.world(), id).unwrap());
    let mut entity = sim
        .world_mut()
        .spawn((EntityKind::Model, LocalName(name.into())));
    if let Some(parent) = parent {
        entity.insert(ChildOf(parent));
    }
    let entity = entity.id();
    assign_id(sim.world_mut(), entity).unwrap()
}

#[test]
fn name_resolution_checks_hierarchy_ambiguity_type_and_identity_ownership() {
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let world = sim
        .world_mut()
        .spawn((EntityKind::World, LocalName("default".into())))
        .id();
    let world = assign_id(sim.world_mut(), world).unwrap();
    let ball = model(&mut sim, "ball", Some(world));
    let mut request = Entity {
        name: "ball".into(),
        r#type: Type::Model as i32,
        ..Default::default()
    };
    assert_eq!(target(&sim, &request), Some(ball));
    request.name = "default::ball".into();
    assert_eq!(target(&sim, &request), Some(ball));
    let outer = model(&mut sim, "outer", Some(world));
    let nested = model(&mut sim, "ball", Some(outer));
    assert_eq!(target(&sim, &request), Some(ball));
    request.name = "ball".into();
    assert_eq!(target(&sim, &request), None);
    request.name = "outer::ball".into();
    assert_eq!(target(&sim, &request), Some(nested));
    for name in ["all", "::ball", "outer::", "", "missing::default::ball"] {
        request.name = name.into();
        assert_eq!(target(&sim, &request), None);
    }
    request.name = "default::ball".into();
    request.r#type = 999;
    assert_eq!(target(&sim, &request), None);
    request.id = ball;
    assert_eq!(target(&sim, &request), Some(ball));
    let mut other = Simulation::new(Duration::from_millis(1)).unwrap();
    let foreign = model(&mut other, "ball", None);
    let foreign_entity = entity_by_id(other.world(), foreign).unwrap();
    let identity = other
        .world_mut()
        .entity_mut(foreign_entity)
        .take::<EntityId>()
        .unwrap();
    sim.world_mut()
        .spawn((EntityKind::Model, LocalName("rogue".into()), identity));
    request.id = 0;
    request.r#type = Type::Model as i32;
    request.name = "rogue".into();
    assert_eq!(target(&sim, &request), None);
    sim.world_mut()
        .spawn((EntityKind::Model, LocalName("unassigned".into())));
    request.name = "unassigned".into();
    assert_eq!(target(&sim, &request), None);
    sim.world_mut().spawn(EntityKind::Model);
    request.name = "missing".into();
    assert_eq!(target(&sim, &request), None);
    assert!(entity_by_id(sim.world(), ball).is_some());
}

#[test]
fn empty_and_disconnected_plain_inputs_preserve_counters_and_report_closed_ingress() {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    let prefix = format!("@/removal-input-{}@", uuid::Uuid::new_v4());
    let mut io = crate::world_io::WorldIo::new_with_config(
        format!("{prefix}/clock"),
        format!("{prefix}/control"),
        format!("{prefix}/poses"),
        1,
        1024,
        config,
    )
    .unwrap();
    let (first, asynchronous) = mpsc::sync_channel(1);
    let (_second, blocking) = mpsc::sync_channel(1);
    let mut removals = Removals::new([asynchronous, blocking]);
    let mut counters = Counters::default();
    let before = format!("{counters:?}");
    removals
        .poll(&io, NonZeroUsize::new(2).unwrap(), &mut counters)
        .unwrap();
    assert_eq!(format!("{counters:?}"), before);
    drop(first);
    assert!(!io.session.is_closed());
    assert!(matches!(
        removals.poll(&io, NonZeroUsize::new(2).unwrap(), &mut counters),
        Err(Error::WorldClosed)
    ));
    assert_eq!(format!("{counters:?}"), before);
    io.shutdown();
    assert!(matches!(
        removals.poll(&io, NonZeroUsize::new(2).unwrap(), &mut counters),
        Err(Error::WorldClosed)
    ));
    assert_eq!(format!("{counters:?}"), before);
}

#[test]
fn defensive_payload_and_boolean_reply_bounds_keep_plain_queue_acceptance_semantics() {
    use gz_sim_proto::msgs::Boolean;
    use zenoh::Wait;
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config
        .insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:0\"]")
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    let prefix = format!("@/removal-bound-{}@", uuid::Uuid::new_v4());
    let io = crate::world_io::WorldIo::new_with_config(
        format!("{prefix}/clock"),
        format!("{prefix}/control"),
        format!("{prefix}/poses"),
        1,
        1,
        config,
    )
    .unwrap();
    let key = format!("{prefix}/remove");
    let [native_input, _blocking] = io
        .removal_inputs([key.clone(), format!("{key}/blocking")])
        .unwrap();
    let (inject, asynchronous) = mpsc::sync_channel(1);
    let (_held, blocking) = mpsc::sync_channel(1);
    let mut removals = Removals::new([asynchronous, blocking]);
    let mut counters = Counters::default();
    let budget = NonZeroUsize::new(1).unwrap();
    let replies = io
        .session
        .get(key.clone())
        .payload(Vec::new())
        .timeout(Duration::from_secs(3))
        .wait()
        .unwrap();
    let mut plain = native_input.recv_timeout(Duration::from_secs(3)).unwrap();
    // Corrupt only the plain envelope after bounded native admission. This tests
    // the defensive CPU bound without fabricating a Query or response slot.
    plain.payload = Entity {
        name: "oversized".into(),
        ..Default::default()
    }
    .encode_to_vec();
    assert!(Entity::decode(plain.payload.as_slice()).is_ok());
    assert!(plain.payload.len() > io.max_message_bytes);
    assert!(inject.try_send(plain).is_ok());
    removals.poll(&io, budget, &mut counters).unwrap();
    let false_reply = replies
        .recv_timeout(Duration::from_secs(3))
        .unwrap()
        .unwrap()
        .into_result()
        .unwrap();
    assert!(
        !Boolean::decode(false_reply.payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
    assert_eq!(counters.removals_rejected, 1);
    assert_eq!(counters.removals_queued, 0);
    assert_eq!(counters.response_failures, 0);
    // An empty Entity is a valid queued asynchronous command. Its true Boolean
    // needs two bytes, exceeding this owner's one-byte response bound.
    let replies = io
        .session
        .get(key)
        .payload(Vec::new())
        .timeout(Duration::from_secs(3))
        .wait()
        .unwrap();
    assert!(
        inject
            .try_send(native_input.recv_timeout(Duration::from_secs(3)).unwrap())
            .is_ok()
    );
    removals.poll(&io, budget, &mut counters).unwrap();
    assert_eq!(counters.removals_queued, 1);
    assert_eq!(counters.response_failures, 1);
    assert!(replies.recv_timeout(Duration::from_secs(3)).is_err());
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    removals.execute(&io, &mut sim, &mut counters);
    assert_eq!(counters.removals_failed, 1);
    assert_eq!(counters.removals_completed, 0);
}
