// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Captured original Double bytes through real canonical ingress and loaded inputs.
//! Native ABS outputs and Rust servo trajectories have separate evidence scopes.
use super::*;
use gz_sim_proto::{
    msgs::{Double, Header, ParameterDeclarations},
    prost::{Message, Name as ProtoName},
};
use gz_sim_scene::{JointControllerMode, JointSample, PhysicsEngine, load_world_with_sdf_step};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};

mod native {
    include!("../../../gz-sim-scene/tests/fixtures/m58_native_isolated_joint_controllers.rs");
}
const XML: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
const INPUT_BYTES: usize = 64;

fn client_config(endpoint: Option<&str>) -> zenoh::Config {
    let mut result = zenoh::Config::default();
    for (key, value) in [
        ("mode", "\"peer\"".to_owned()),
        (
            "listen/endpoints",
            if endpoint.is_some() {
                "[]".to_owned()
            } else {
                "[\"tcp/127.0.0.1:0\"]".to_owned()
            },
        ),
        (
            "connect/endpoints",
            endpoint.map_or_else(|| "[]".to_owned(), |value| format!("[\"{value}\"]")),
        ),
        ("scouting/multicast/enabled", "false".to_owned()),
        ("scouting/gossip/enabled", "false".to_owned()),
    ] {
        result.insert_json5(key, &value).unwrap();
    }
    result
}
fn loaded_server() -> (Server, zenoh::Session) {
    let observer = zenoh::open(client_config(None)).wait().unwrap();
    let endpoints: Vec<_> = observer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|value| value.to_string())
        .collect();
    assert_eq!(endpoints.len(), 1);
    assert_ne!(endpoints[0], "tcp/127.0.0.1:0");
    let simulation = load_world_with_sdf_step(
        XML,
        "m58-observed-wires.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let server = Server::with_zenoh_config(
        simulation,
        "default",
        ServerOptions {
            partition: format!("m58-observed-{}", uuid::Uuid::new_v4()),
            capacity: 128,
            max_message_bytes: INPUT_BYTES,
            ..Default::default()
        },
        client_config(Some(&endpoints[0])),
        NonZeroUsize::new(4).unwrap(),
    )
    .unwrap();
    (server, observer)
}
fn wait(mut condition: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
}
fn finite(words: &[u64]) {
    assert!(words.iter().all(|word| f64::from_bits(*word).is_finite()));
}
fn header(captured: Option<&native::NativeHeader>, actual: Option<&Header>) {
    match (captured, actual) {
        (None, None) => {}
        (Some(captured), Some(actual)) => {
            assert!(captured.wire.len() <= 4096);
            assert_eq!(Header::decode(captured.wire).unwrap(), *actual);
            assert_eq!(
                captured.stamp.as_ref().map(|stamp| (stamp.sec, stamp.nsec)),
                actual.stamp.as_ref().map(|stamp| (stamp.sec, stamp.nsec))
            );
            if let Some(stamp) = captured.stamp {
                assert!((0..1_000_000_000).contains(&stamp.nsec));
            }
            assert_eq!(captured.data.len(), actual.data.len());
            for (saved, decoded) in captured.data.iter().zip(&actual.data) {
                assert_eq!(saved.key, decoded.key);
                assert_eq!(
                    saved.value,
                    decoded
                        .value
                        .iter()
                        .map(String::as_str)
                        .collect::<Vec<_>>()
                        .as_slice()
                );
                assert_eq!(
                    gz_sim_proto::msgs::header::Map::decode(saved.wire).unwrap(),
                    *decoded
                );
            }
        }
        _ => panic!("captured optional Header presence differs"),
    }
}
fn double(value: &native::NativeDouble) -> Double {
    finite(&[value.data_bits]);
    assert!(value.wire.len() <= 32);
    let decoded = Double::decode(value.wire).unwrap();
    assert_eq!(decoded.data.to_bits(), value.data_bits);
    header(value.header, decoded.header.as_ref());
    decoded
}
fn parameter_kind(value: native::NativeParameterResultType) -> u8 {
    let tags = [
        native::NativeParameterResultType::Success,
        native::NativeParameterResultType::AlreadyDeclared,
        native::NativeParameterResultType::InvalidType,
        native::NativeParameterResultType::NotDeclared,
        native::NativeParameterResultType::ClientTimeout,
        native::NativeParameterResultType::Unexpected,
    ];
    let index = tags
        .iter()
        .position(|tag| std::mem::discriminant(tag) == std::mem::discriminant(&value))
        .unwrap();
    u8::try_from(index).unwrap()
}
fn parameter_names(case: &native::NativeCase) -> Vec<String> {
    const SUFFIXES: [&str; 8] = [
        "p_gain",
        "i_gain",
        "d_gain",
        "i_max",
        "i_min",
        "cmd_max",
        "cmd_min",
        "cmd_offset",
    ];
    if matches!(case.family, native::NativeFamily::Velocity) {
        return Vec::new();
    }
    case.configurations
        .iter()
        .flat_map(|config| {
            let world = config
                .graph
                .iter()
                .find(|node| matches!(node.kind, native::NativeGraphKind::World))
                .unwrap();
            let prefix = format!(
                "JointPositionController.{}.{}.{}.",
                world.name, config.model_name, config.joint_name
            );
            assert_eq!(
                config.parameter_prefix_from_public_joint_scope,
                Some(prefix.as_str())
            );
            SUFFIXES
                .iter()
                .map(|suffix| format!("{prefix}{suffix}"))
                .collect::<Vec<_>>()
        })
        .collect()
}
fn parameter_values(values: &[native::NativeParameterValue], names: &[String]) {
    for (index, value) in values.iter().enumerate() {
        assert_eq!(usize::try_from(value.parameter_index).unwrap(), index);
        assert_eq!(value.requested_name, names[index]);
        assert!(value.requested_name.len() <= 128);
        assert_eq!(parameter_kind(value.result.kind), 0);
        assert!(
            value.result.parameter_name.len() <= 128 && value.result.parameter_type.len() <= 32
        );
        double(value.value);
    }
}
fn registry(record: &native::NativeRegistryRecord, case: &native::NativeCase) {
    let count = match record.stage {
        native::NativeRegistryStage::Empty => 0,
        native::NativeRegistryStage::AfterPosition0 => 8,
        native::NativeRegistryStage::AfterPosition2 => 16,
    };
    assert_eq!(record.values.len(), count);
    parameter_values(record.values, &parameter_names(case));
    let decoded = ParameterDeclarations::decode(record.declaration_list.wire).unwrap();
    assert_eq!(
        decoded.parameter_declarations.len(),
        record.declaration_list.declarations.len()
    );
    assert_eq!(decoded.parameter_declarations.len(), count);
    for (actual, saved) in decoded
        .parameter_declarations
        .iter()
        .zip(record.declaration_list.declarations)
    {
        assert_eq!(actual.name, saved.name);
        assert_eq!(actual.r#type, saved.kind);
        assert_eq!(
            gz_sim_proto::msgs::ParameterDeclaration::decode(saved.wire).unwrap(),
            *actual
        );
    }
    match record.configure_parameters_call {
        None => assert_eq!(count, 0),
        Some(call) => {
            let config = &case.configurations[usize::try_from(call.index).unwrap()];
            assert_eq!(call.slot, config.slot);
            assert_eq!(call.model_entity, config.model_entity);
            assert_eq!(call.joint_entity, config.joint_entity);
            assert!(matches!(
                call.interface,
                native::NativeInterface::ConfigureParameters
            ));
        }
    }
}
#[derive(Debug, PartialEq, Eq)]
struct InputWords {
    slot: u32,
    model: u64,
    joint: u64,
    position: Option<Vec<u64>>,
    velocity: Option<Vec<u64>>,
}
fn inputs(values: &[native::NativeInput; 4]) -> Vec<InputWords> {
    values
        .iter()
        .enumerate()
        .map(|(slot, value)| {
            assert_eq!(usize::try_from(value.slot).unwrap(), slot);
            assert_ne!(value.model_entity, 0);
            assert_ne!(value.joint_entity, 0);
            for words in [value.joint_position, value.joint_velocity] {
                let words = words.unwrap();
                assert_eq!(words.len(), 1);
                finite(words);
            }
            InputWords {
                slot: value.slot,
                model: value.model_entity,
                joint: value.joint_entity,
                position: value.joint_position.map(|words| words.to_vec()),
                velocity: value.joint_velocity.map(|words| words.to_vec()),
            }
        })
        .collect()
}
fn commands(values: &[native::NativeCommandState; 4], selected: &[u32; 2]) {
    for (slot, value) in values.iter().enumerate() {
        assert_eq!(usize::try_from(value.slot).unwrap(), slot);
        assert_ne!(value.joint_entity, 0);
        assert!(value.joint_force_command.is_none());
        if let Some(words) = value.joint_velocity_command {
            assert!(selected.contains(&value.slot));
            assert!(words.len() <= 1);
            finite(words);
        }
    }
}
type InfoWords = (u64, u64, i64, u64, bool);
fn info(value: &native::NativeInfo) -> InfoWords {
    assert_eq!(value.sim_time_ns, value.real_time_ns);
    assert!(value.dt_ns >= 0);
    (
        value.sim_time_ns,
        value.real_time_ns,
        value.dt_ns,
        value.iterations,
        value.paused,
    )
}
fn registry_snapshot(
    value: Option<&[native::NativeParameterValue; 16]>,
    case: &native::NativeCase,
) {
    if matches!(case.family, native::NativeFamily::PositionAbs) {
        parameter_values(value.unwrap(), &parameter_names(case));
    } else {
        assert!(value.is_none());
    }
}
fn ordinal(case: &native::NativeCase, kind: native::NativeKind, index: usize) -> (usize, u64) {
    match kind {
        native::NativeKind::Config => (0, case.configurations[index].ordinal),
        native::NativeKind::Registry => (1, case.registry_records[index].ordinal),
        native::NativeKind::RoundInput => (2, case.round_inputs[index].ordinal),
        native::NativeKind::Pre => (3, case.pre_calls[index].ordinal),
        native::NativeKind::RoundEnd => (4, case.round_ends[index].ordinal),
        native::NativeKind::Publication => (5, case.publications[index].ordinal),
        native::NativeKind::Tap => (6, case.taps[index].ordinal),
        native::NativeKind::Fence => (7, case.fences[index].ordinal),
        native::NativeKind::Summary => {
            assert_eq!(index, 0);
            (8, case.summary.ordinal)
        }
    }
}
fn inspect_case(case: &native::NativeCase) {
    assert_eq!(case.schema, "M58_ISOLATED_CONTROLLER_COMPONENT_LEDGER_V1");
    let position = matches!(case.family, native::NativeFamily::PositionAbs);
    assert_eq!(
        case.selected_slots,
        if position { &[0, 2] } else { &[1, 3] }
    );
    assert_eq!(case.configurations.len(), 2);
    for (index, config) in case.configurations.iter().enumerate() {
        assert_eq!(usize::try_from(config.configure_index).unwrap(), index);
        assert_eq!(config.slot, case.selected_slots[index]);
        assert_eq!(usize::try_from(config.owner).unwrap(), index);
        assert!(
            !config.model_name.is_empty()
                && !config.joint_name.is_empty()
                && !config.plugin_xml.is_empty()
        );
        assert!(config.topic.starts_with('/'));
        assert_ne!(config.model_entity, config.joint_entity);
        assert_eq!(config.graph.len(), 11);
        let mut graph = BTreeMap::new();
        let mut kinds = [0; 4];
        for node in config.graph {
            assert_ne!(node.entity, 0);
            assert!(!node.name.is_empty());
            assert!(graph.insert(node.entity, node.parent_entity).is_none());
            let index = match node.kind {
                native::NativeGraphKind::World => 0,
                native::NativeGraphKind::Model => 1,
                native::NativeGraphKind::Link => 2,
                native::NativeGraphKind::Joint => 3,
            };
            kinds[index] += 1;
        }
        assert_eq!(kinds, [1, 2, 4, 4]);
        assert_eq!(graph[&config.joint_entity], Some(config.model_entity));
        for parent in graph.values().flatten() {
            assert!(graph.contains_key(parent));
        }
        assert!(
            config
                .interfaces
                .iter()
                .any(|value| matches!(value, native::NativeInterface::Configure))
        );
        assert!(
            config
                .interfaces
                .iter()
                .any(|value| matches!(value, native::NativeInterface::PreUpdate))
        );
        assert_eq!(config.discovery.publisher_type_names, &["gz.msgs.Double"]);
        assert_eq!(
            config.discovery.subscriber_type_names,
            &["gz.msgs.Double", "gz.msgs.Double"]
        );
        assert!(config.discovery.has_connections);
        match config.settings {
            native::NativeSettings::Position(settings) => {
                assert!(position && settings.use_velocity_commands);
                assert_eq!(settings.joint_index, 0);
                finite(&[settings.cmd_max_bits, settings.initial_target_bits]);
                let prefix = config.parameter_prefix_from_public_joint_scope.unwrap();
                assert!(prefix.starts_with("JointPositionController.") && prefix.ends_with('.'));
                assert!(
                    config
                        .interfaces
                        .iter()
                        .any(|value| matches!(value, native::NativeInterface::ConfigureParameters))
                );
            }
            native::NativeSettings::Velocity(settings) => {
                assert!(!position && !settings.use_force_commands);
                finite(&[settings.initial_target_bits]);
                assert!(config.parameter_prefix_from_public_joint_scope.is_none());
            }
        }
    }
    for record in case.registry_records {
        registry(record, case);
    }
    assert_eq!(case.registry_records.len(), if position { 3 } else { 0 });
    for (index, row) in case.round_inputs.iter().enumerate() {
        assert_eq!(row.round_index, u64::try_from(index).unwrap());
        assert_eq!(row.info.iterations, row.round_index);
        info(&row.info);
        inputs(row.inputs);
        commands(row.commands_before, case.selected_slots);
        registry_snapshot(row.registry_before, case);
        match row.role {
            native::NativeRoundRole::Initial => assert!(
                row.phase.is_none() && row.probe_index.is_none() && row.measurement_index.is_none()
            ),
            native::NativeRoundRole::Baseline => assert!(
                row.phase.is_some() && row.probe_index.is_none() && row.measurement_index.is_none()
            ),
            native::NativeRoundRole::Probe => assert!(
                row.phase.is_some()
                    && row.probe_index.is_some_and(|value| value < 32)
                    && row.measurement_index.is_none()
            ),
            native::NativeRoundRole::Measurement => assert!(
                row.phase.is_some() && row.probe_index.is_none() && row.measurement_index.is_some()
            ),
        }
    }
    assert_eq!(case.pre_calls.len(), 2 * case.round_inputs.len());
    assert_eq!(case.round_ends.len(), case.round_inputs.len());
    for row in case.pre_calls {
        let original = &case.round_inputs[usize::try_from(row.round_index).unwrap()];
        assert_eq!(
            row.call_index,
            2 * row.round_index + u64::from(row.selected_index)
        );
        assert_eq!(
            row.slot,
            case.selected_slots[usize::try_from(row.selected_index).unwrap()]
        );
        assert_eq!(info(&row.info), info(&original.info));
        assert_eq!(inputs(row.inputs_after), inputs(original.inputs));
        commands(row.commands_after, case.selected_slots);
        registry_snapshot(row.registry_after, case);
    }
    for row in case.round_ends {
        let original = &case.round_inputs[usize::try_from(row.round_index).unwrap()];
        assert_eq!(info(&row.info), info(&original.info));
        assert_eq!(inputs(row.inputs_after), inputs(original.inputs));
        commands(row.commands_after, case.selected_slots);
        registry_snapshot(row.registry_after, case);
    }
    assert_eq!(
        (case.publications.len(), case.taps.len(), case.fences.len()),
        (4, 4, 4)
    );
    for (phase, publication) in case.publications.iter().enumerate() {
        assert_eq!(usize::try_from(publication.phase).unwrap(), phase);
        assert!(publication.publish_returned_true);
        let config = case
            .configurations
            .iter()
            .find(|config| config.slot == publication.slot)
            .unwrap();
        assert_eq!(publication.slot, case.selected_slots[phase % 2]);
        assert_eq!(publication.topic, config.topic);
        let decoded = double(publication.message);
        assert!(decoded.data.is_finite());
        let tap = &case.taps[phase];
        assert_eq!(tap.phase, publication.phase);
        assert_eq!(tap.slot, publication.slot);
        assert_eq!(tap.topic, publication.topic);
        assert_eq!(tap.arrival_ordinal, u64::try_from(phase).unwrap());
        if phase > 0 {
            assert!(tap.arrival_elapsed_ns >= case.taps[phase - 1].arrival_elapsed_ns);
        }
        assert_eq!(double(tap.message), decoded);
        assert_eq!(tap.message.wire, publication.message.wire);
        let fence = &case.fences[phase];
        assert_eq!(fence.phase, publication.phase);
        assert_eq!(fence.slot, publication.slot);
        assert_eq!(fence.publication_ordinal, publication.ordinal);
        assert_eq!(fence.tap_ordinal, tap.ordinal);
        assert!((1..=32).contains(&fence.probe_round_indices.len()));
        assert_eq!(
            fence.probe_round_indices.len(),
            usize::try_from(case.summary.probe_round_counts[phase]).unwrap()
        );
        assert!(fence.baseline_round_index < fence.probe_round_indices[0]);
        let baseline = &case.round_inputs[usize::try_from(fence.baseline_round_index).unwrap()];
        assert!(matches!(baseline.role, native::NativeRoundRole::Baseline));
        assert_eq!(baseline.phase, Some(publication.phase));
        for (probe, round) in fence.probe_round_indices.iter().enumerate() {
            let input = &case.round_inputs[usize::try_from(*round).unwrap()];
            assert!(matches!(input.role, native::NativeRoundRole::Probe));
            assert_eq!(input.phase, Some(publication.phase));
            assert_eq!(input.probe_index, Some(u32::try_from(probe).unwrap()));
            if probe > 0 {
                assert!(fence.probe_round_indices[probe - 1] < *round);
            }
        }
        assert_eq!(
            Some(&fence.matched_probe_round_index),
            fence.probe_round_indices.last()
        );
        assert!(matches!(
            fence.completion,
            native::NativeCompletion::ObservedOriginalPreUpdateComponentEffect
        ));
        assert!(matches!(
            fence.evidence_scope,
            native::NativeFenceScope::TapIsNotPrivateCallbackAck
        ));
    }
    let counts = &case.summary.counts;
    assert_eq!(
        counts.configurations,
        u64::try_from(case.configurations.len()).unwrap()
    );
    assert_eq!(
        counts.registry_records,
        u64::try_from(case.registry_records.len()).unwrap()
    );
    assert_eq!(
        counts.round_inputs,
        u64::try_from(case.round_inputs.len()).unwrap()
    );
    assert_eq!(
        counts.pre_update_calls,
        u64::try_from(case.pre_calls.len()).unwrap()
    );
    assert_eq!(
        counts.round_ends,
        u64::try_from(case.round_ends.len()).unwrap()
    );
    assert_eq!(
        counts.publications,
        u64::try_from(case.publications.len()).unwrap()
    );
    assert_eq!(counts.tap_records, u64::try_from(case.taps.len()).unwrap());
    assert_eq!(counts.fences, u64::try_from(case.fences.len()).unwrap());
    assert_eq!(counts.command_state_getbacks, 16 * counts.round_inputs);
    assert_eq!(
        counts.helper_registry_get_calls,
        if position {
            24 + 64 * counts.round_inputs
        } else {
            0
        }
    );
    assert_eq!(
        counts.raw_rows,
        u64::try_from(case.raw_order.len()).unwrap()
    );
    assert_eq!(
        case.summary.state,
        "ORIGINAL_M58_COMPONENT_CAPTURE_COMPLETED_PENDING_INDEPENDENT_DATA"
    );
    assert_eq!(case.summary.probe_round_counts.len(), 4);
    assert!(
        case.summary
            .probe_round_counts
            .iter()
            .all(|count| (1..=32).contains(count))
    );
    let mut seen = [0_usize; 9];
    for (index, row) in case.raw_order.iter().enumerate() {
        let (kind, captured_ordinal) = ordinal(case, row.kind, row.index);
        assert_eq!(seen[kind], row.index);
        seen[kind] += 1;
        assert_eq!(row.raw_ordinal, u64::try_from(index).unwrap());
        assert_eq!(captured_ordinal, row.raw_ordinal);
    }
    assert_eq!(
        seen,
        [
            case.configurations.len(),
            case.registry_records.len(),
            case.round_inputs.len(),
            case.pre_calls.len(),
            case.round_ends.len(),
            case.publications.len(),
            case.taps.len(),
            case.fences.len(),
            1
        ]
    );
}

#[derive(Debug, PartialEq, Eq)]
struct FrozenJoint {
    id: u64,
    name: String,
    metadata: JointControllerConfiguration,
    lease: usize,
    native: [u64; 2],
    sample: Option<(Duration, u64, [u64; 2])>,
}
#[derive(Debug, PartialEq, Eq)]
struct Frozen {
    epoch: u64,
    elapsed: Duration,
    paused: bool,
    step: Duration,
    joints: Vec<FrozenJoint>,
}
fn frozen(server: &Server) -> Frozen {
    Frozen {
        epoch: server.simulation.epoch(),
        elapsed: server.simulation.elapsed(),
        paused: server.simulation.is_paused(),
        step: server.simulation.step_duration(),
        joints: server
            .joint_controllers
            .as_ref()
            .unwrap()
            .streams
            .iter()
            .map(|stream| {
                assert!(gz_sim_scene::joint_controller_binding_matches(
                    &server.simulation,
                    stream.identity
                ));
                let sample = server.simulation.world().get::<JointSample>(stream.entity);
                let state = server
                    .simulation
                    .world()
                    .resource::<gz_physics::PhysicsBackend>()
                    .joint_state(stream.joint.id())
                    .unwrap();
                FrozenJoint {
                    id: stream.identity,
                    name: stream.name.clone(),
                    metadata: stream.metadata.clone(),
                    lease: Arc::as_ptr(&stream._lease) as usize,
                    native: [state.position.to_bits(), state.velocity.to_bits()],
                    sample: sample.map(|sample| {
                        (
                            sample.time,
                            sample.epoch,
                            [
                                sample.state.position.to_bits(),
                                sample.state.velocity.to_bits(),
                            ],
                        )
                    }),
                }
            })
            .collect(),
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Counts {
    received: u64,
    applied: u64,
    rejected: u64,
}
fn counters(server: &Server) -> Counts {
    Counts {
        received: server.counters.joint_controller_messages_received,
        applied: server.counters.joint_controller_commands_applied,
        rejected: server.counters.joint_controller_messages_rejected,
    }
}
fn empty_inputs(server: &Server) {
    for stream in &server.joint_controllers.as_ref().unwrap().streams {
        assert!(matches!(
            stream.receiver.try_recv(),
            Err(mpsc::TryRecvError::Empty)
        ));
    }
}
fn poll_one(server: &mut Server, before: Counts, valid: bool) {
    wait(|| {
        server.poll_joint_controllers().unwrap();
        assert!(server.counters.joint_controller_messages_received <= before.received + 1);
        server.counters.joint_controller_messages_received == before.received + 1
    });
    assert_eq!(
        counters(server),
        Counts {
            received: before.received + 1,
            applied: before.applied + u64::from(valid),
            rejected: before.rejected + u64::from(!valid)
        }
    );
    assert!(!server.reset_failed);
    empty_inputs(server);
}

#[test]
fn captured_original_double_wires_reach_only_matching_loaded_controller_inputs() {
    let mut replays = 0;
    let mut combined = Counts {
        received: 0,
        applied: 0,
        rejected: 0,
    };
    let mut partitions = BTreeSet::new();
    for case in [native::NATIVE_RUNS.position, native::NATIVE_RUNS.velocity] {
        let (mut server, observer) = loaded_server();
        assert!(partitions.insert(server.network_config.partition.clone()));
        assert_eq!(server.joint_controllers.as_ref().unwrap().streams.len(), 4);
        let identities: BTreeSet<_> = server
            .joint_controllers
            .as_ref()
            .unwrap()
            .streams
            .iter()
            .map(|stream| stream.identity)
            .collect();
        assert_eq!(identities.len(), 4);
        assert!(!identities.contains(&0));
        assert_eq!(
            counters(&server),
            Counts {
                received: 0,
                applied: 0,
                rejected: 0
            }
        );
        inspect_case(case);
        for publication in case.publications {
            let config = case
                .configurations
                .iter()
                .find(|value| value.slot == publication.slot)
                .unwrap();
            let position = matches!(case.family, native::NativeFamily::PositionAbs);
            let mode = if position {
                JointControllerMode::Position
            } else {
                JointControllerMode::Velocity
            };
            let index = server
                .joint_controllers
                .as_ref()
                .unwrap()
                .streams
                .iter()
                .position(|stream| {
                    stream.metadata.model_name == config.model_name
                        && stream.metadata.joint_name == config.joint_name
                        && stream.metadata.mode == mode
                })
                .unwrap();
            let stream = &server.joint_controllers.as_ref().unwrap().streams[index];
            assert_eq!(stream.metadata.topic, config.topic);
            assert_eq!(
                stream.name,
                format!("default::{}::{}", config.model_name, config.joint_name)
            );
            let key = stream.endpoint.clone();
            assert!(stream._lease.matches_endpoint(&key));
            let publisher = observer.declare_publisher(key.clone()).wait().unwrap();
            wait(|| publisher.matching_status().wait().unwrap().matching());
            let monitor = observer.declare_subscriber(key).wait().unwrap();
            let before = frozen(&server);
            let count = counters(&server);
            empty_inputs(&server);
            let decoded = double(publication.message);
            assert!(publication.message.header.is_none());
            publisher
                .put(publication.message.wire.to_vec())
                .attachment(Double::full_name())
                .wait()
                .unwrap();
            let seen = monitor
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(seen.payload().to_bytes().as_ref(), publication.message.wire);
            assert_eq!(
                seen.attachment().unwrap().to_bytes().as_ref(),
                Double::full_name().as_bytes()
            );
            let received = server.joint_controllers.as_ref().unwrap().streams[index]
                .receiver
                .recv_timeout(Duration::from_secs(5))
                .unwrap();
            assert_eq!(received.0, server.simulation.epoch());
            assert_eq!(received.1.map(f64::to_bits), Some(decoded.data.to_bits()));
            empty_inputs(&server);
            assert_eq!(counters(&server), count);
            assert_eq!(frozen(&server), before);
            // Replay the SAME immutable native bytes for the actual product poll.
            publisher
                .put(publication.message.wire.to_vec())
                .attachment(Double::full_name())
                .wait()
                .unwrap();
            let seen = monitor
                .recv_timeout(Duration::from_secs(5))
                .unwrap()
                .unwrap();
            assert_eq!(seen.payload().to_bytes().as_ref(), publication.message.wire);
            poll_one(&mut server, count, true);
            assert_eq!(frozen(&server), before);
            let mut truncated = publication.message.wire.to_vec();
            truncated.pop().unwrap();
            assert!(Double::decode(truncated.as_slice()).is_err());
            let mut oversized = publication.message.wire.to_vec();
            while oversized.len() <= INPUT_BYTES {
                oversized.extend_from_slice(&[0x18, 0]);
            }
            assert_eq!(
                Double::decode(oversized.as_slice()).unwrap().data.to_bits(),
                publication.message.data_bits
            );
            for (bytes, attachment) in [
                (
                    publication.message.wire.to_vec(),
                    "gz.msgs.Twist".to_owned(),
                ),
                (truncated, Double::full_name()),
                (oversized, Double::full_name()),
            ] {
                let count = counters(&server);
                publisher
                    .put(bytes.clone())
                    .attachment(attachment)
                    .wait()
                    .unwrap();
                let seen = monitor
                    .recv_timeout(Duration::from_secs(5))
                    .unwrap()
                    .unwrap();
                assert_eq!(seen.payload().to_bytes().as_ref(), bytes.as_slice());
                poll_one(&mut server, count, false);
                assert_eq!(frozen(&server), before);
            }
            replays += 1;
        }
        let actual = counters(&server);
        assert_eq!(
            actual,
            Counts {
                received: 16,
                applied: 4,
                rejected: 12
            }
        );
        combined.received += actual.received;
        combined.applied += actual.applied;
        combined.rejected += actual.rejected;
        drop(server);
        observer.close().wait().unwrap();
    }
    assert_eq!(replays, 8);
    assert_eq!(
        combined,
        Counts {
            received: 32,
            applied: 8,
            rejected: 24
        }
    );
}
