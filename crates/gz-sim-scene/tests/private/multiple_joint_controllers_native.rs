// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observed command INPUT admission and pending-target isolation only.
//! Native families run in separate processes. This test never installs their
//! q/v, poses or clocks, steps Physics, or compares native ABS output to a servo.
use super::*;
use std::collections::BTreeSet;

mod native {
    include!("../fixtures/m58_native_isolated_joint_controllers.rs");
}

const SOURCE: &str = include_str!("../../../../examples/m58_multiple_joint_controllers.sdf");
const MODELS: [&str; 2] = ["arm_left", "arm_right"];
const JOINTS: [&str; 2] = ["base_hinge", "tip_hinge"];
const TOPICS: [&str; 4] = [
    "/model/arm_left/joint/base_hinge/0/cmd_pos",
    "/model/arm_left/joint/tip_hinge/cmd_vel",
    "/model/arm_right/joint/base_hinge/0/cmd_pos",
    "/model/arm_right/joint/tip_hinge/cmd_vel",
];
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

#[derive(Default)]
struct Custody {
    words: usize,
    wire_bytes: usize,
    input_sets: usize,
    command_sets: usize,
    parameter_values: usize,
}

fn text(value: &str, cap: usize) {
    assert!(value.is_ascii() && value.len() <= cap);
}

fn word(value: u64, custody: &mut Custody) -> f64 {
    custody.words += 1;
    let value = f64::from_bits(value);
    assert!(value.is_finite());
    value
}

fn wire(value: &[u8], custody: &mut Custody) {
    assert!(value.len() <= 16384);
    custody.wire_bytes += value.len();
}

fn header(value: Option<&native::NativeHeader>, custody: &mut Custody) {
    if let Some(value) = value {
        if let Some(stamp) = value.stamp {
            assert!((0..1_000_000_000).contains(&stamp.nsec));
            let _signed_seconds = stamp.sec;
        }
        assert!(value.data.len() <= 16);
        for datum in value.data {
            text(datum.key, 1024);
            assert!(datum.value.len() <= 16);
            for item in datum.value {
                text(item, 1024);
            }
            wire(datum.wire, custody);
        }
        wire(value.wire, custody);
    }
}

fn double(value: &native::NativeDouble, custody: &mut Custody) {
    header(value.header, custody);
    word(value.data_bits, custody);
    wire(value.wire, custody);
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

fn parameters(values: &[native::NativeParameterValue], names: &[String], custody: &mut Custody) {
    assert!(values.len() <= 16);
    for (index, value) in values.iter().enumerate() {
        custody.parameter_values += 1;
        assert_eq!(usize::try_from(value.parameter_index).unwrap(), index);
        assert_eq!(value.requested_name, names[index]);
        assert_eq!(
            parameter_kind(value.result.kind),
            0,
            "captured successful Registry result required"
        );
        // Success ParamName/ParamType may genuinely be empty; retain them.
        text(value.result.parameter_name, 128);
        text(value.result.parameter_type, 32);
        double(value.value, custody);
    }
}

fn registry_snapshot(
    value: Option<&[native::NativeParameterValue; 16]>,
    position: bool,
    names: &[String],
    custody: &mut Custody,
) {
    if position {
        parameters(
            value.expect("position has its genuine native Registry"),
            names,
            custody,
        );
    } else {
        assert!(value.is_none(), "velocity has no native Registry");
    }
}

fn input_set(
    value: &[native::NativeInput; 4],
    ids: &[u64; 4],
    models: &[u64; 2],
    custody: &mut Custody,
) {
    custody.input_sets += 1;
    for (slot, value) in value.iter().enumerate() {
        assert_eq!(usize::try_from(value.slot).unwrap(), slot);
        assert_eq!(value.joint_entity, ids[slot]);
        assert_eq!(value.model_entity, models[slot / 2]);
        for component in [value.joint_position, value.joint_velocity] {
            let observed = component.expect("native input presence");
            assert_eq!(observed.len(), 1);
            word(observed[0], custody);
        }
    }
}

fn same_command(left: &native::NativeCommandState, right: &native::NativeCommandState) -> bool {
    (
        left.slot,
        left.joint_entity,
        left.joint_velocity_command,
        left.joint_force_command,
    ) == (
        right.slot,
        right.joint_entity,
        right.joint_velocity_command,
        right.joint_force_command,
    )
}

fn same_inputs(left: &[native::NativeInput; 4], right: &[native::NativeInput; 4]) {
    for (left, right) in left.iter().zip(right) {
        assert_eq!(
            (
                left.slot,
                left.model_entity,
                left.joint_entity,
                left.joint_position,
                left.joint_velocity
            ),
            (
                right.slot,
                right.model_entity,
                right.joint_entity,
                right.joint_position,
                right.joint_velocity
            )
        );
    }
}

fn command_set(
    value: &[native::NativeCommandState; 4],
    ids: &[u64; 4],
    selected: &[u32; 2],
    custody: &mut Custody,
) {
    custody.command_sets += 1;
    for (slot, value) in value.iter().enumerate() {
        assert_eq!(usize::try_from(value.slot).unwrap(), slot);
        assert_eq!(value.joint_entity, ids[slot]);
        assert!(value.joint_force_command.is_none());
        if let Some(values) = value.joint_velocity_command {
            assert!(values.len() <= 1);
            for value in values {
                word(*value, custody);
            }
        }
        if !selected.contains(&u32::try_from(slot).unwrap()) {
            assert!(value.joint_velocity_command.is_none());
        }
    }
}

fn info(value: native::NativeInfo) -> [u64; 5] {
    assert!(value.dt_ns >= 0);
    assert_eq!(value.real_time_ns, value.sim_time_ns);
    if value.paused {
        assert_eq!(value.dt_ns, 0);
    }
    [
        value.sim_time_ns,
        value.real_time_ns,
        u64::try_from(value.dt_ns).unwrap(),
        value.iterations,
        if value.paused { 1 } else { 0 },
    ]
}

fn phase(value: Option<u32>) {
    if let Some(value) = value {
        assert!(value < 4);
    }
}

fn graph(case: &native::NativeCase) -> ([u64; 4], [u64; 2], Vec<String>) {
    assert_eq!(case.configurations.len(), 2);
    let nodes = case.configurations[0].graph;
    assert_eq!(nodes.len(), 11);
    let mut seen = BTreeSet::new();
    let mut joints = [0; 4];
    let mut models = [0; 2];
    let mut worlds = Vec::new();
    for node in nodes {
        assert!(node.entity > 0 && seen.insert(node.entity));
        text(node.name, 112);
        match node.kind {
            native::NativeGraphKind::World => {
                assert!(node.parent_entity.is_none());
                assert!(!node.name.is_empty());
                worlds.push((node.entity, node.name));
            }
            native::NativeGraphKind::Model => {
                let owner = MODELS.iter().position(|name| *name == node.name).unwrap();
                assert_eq!(models[owner], 0);
                models[owner] = node.entity;
            }
            native::NativeGraphKind::Link | native::NativeGraphKind::Joint => {
                assert!(node.parent_entity.is_some());
            }
        }
    }
    assert_eq!(worlds.len(), 1);
    assert_eq!(
        worlds[0].1, "default",
        "same declared world name as the genuine supported SDF"
    );
    let mut links = BTreeSet::new();
    assert!(models.iter().all(|value| *value != 0));
    for node in nodes {
        match node.kind {
            native::NativeGraphKind::World => {}
            native::NativeGraphKind::Model => {
                assert_eq!(node.parent_entity, Some(worlds[0].0));
            }
            native::NativeGraphKind::Link => {
                assert!(models.contains(&node.parent_entity.unwrap()));
                assert!(matches!(node.name, "base" | "tip"));
                assert!(links.insert((node.parent_entity.unwrap(), node.name)));
            }
            native::NativeGraphKind::Joint => {
                let owner = models
                    .iter()
                    .position(|id| Some(*id) == node.parent_entity)
                    .unwrap();
                let local = JOINTS.iter().position(|name| *name == node.name).unwrap();
                assert_eq!(joints[owner * 2 + local], 0);
                joints[owner * 2 + local] = node.entity;
            }
        }
    }
    assert!(joints.iter().all(|value| *value != 0));
    assert_eq!(links.len(), 4);
    let names = case
        .configurations
        .iter()
        .flat_map(|config| {
            let prefix = config.parameter_prefix_from_public_joint_scope;
            if matches!(case.family, native::NativeFamily::PositionAbs) {
                let expected = format!(
                    "JointPositionController.{}.{}.{}.",
                    worlds[0].1, config.model_name, config.joint_name
                );
                assert_eq!(prefix, Some(expected.as_str()));
                SUFFIXES
                    .iter()
                    .map(|suffix| format!("{expected}{suffix}"))
                    .collect::<Vec<_>>()
            } else {
                assert!(prefix.is_none());
                Vec::new()
            }
        })
        .collect();
    (joints, models, names)
}

fn configuration(
    value: &native::NativeConfiguration,
    index: usize,
    case: &native::NativeCase,
    ids: &[u64; 4],
    models: &[u64; 2],
    custody: &mut Custody,
) {
    let slot = usize::try_from(value.slot).unwrap();
    assert!(slot < 4);
    assert_eq!(value.slot, case.selected_slots[index]);
    assert_eq!(usize::try_from(value.owner).unwrap(), slot / 2);
    assert_eq!(value.model_entity, models[slot / 2]);
    assert_eq!(value.joint_entity, ids[slot]);
    assert_eq!(value.model_name, MODELS[slot / 2]);
    assert_eq!(value.joint_name, JOINTS[slot % 2]);
    assert_eq!(value.topic, TOPICS[slot]);
    assert_eq!(usize::try_from(value.configure_index).unwrap(), index);
    assert_eq!(value.graph.len(), 11);
    for (first, other) in case.configurations[0].graph.iter().zip(value.graph) {
        assert_eq!(
            (first.entity, first.name, first.parent_entity),
            (other.entity, other.name, other.parent_entity)
        );
        assert_eq!(
            std::mem::discriminant(&first.kind),
            std::mem::discriminant(&other.kind)
        );
    }
    let plugin = roxmltree::Document::parse(value.plugin_xml).unwrap();
    let root = plugin.root_element();
    assert_eq!(root.tag_name().name(), "plugin");
    let field = |name| {
        root.children()
            .find(|node| node.has_tag_name(name))
            .unwrap()
            .text()
            .unwrap()
    };
    assert_eq!(field("joint_name"), value.joint_name);
    let position = matches!(case.family, native::NativeFamily::PositionAbs);
    assert_eq!(value.interfaces.len(), if position { 3 } else { 2 });
    assert!(matches!(
        value.interfaces[0],
        native::NativeInterface::Configure
    ));
    assert!(matches!(
        value.interfaces[1],
        native::NativeInterface::PreUpdate
    ));
    match value.settings {
        native::NativeSettings::Position(settings) => {
            assert!(position && slot % 2 == 0);
            assert_eq!(settings.joint_index, 0);
            assert!(settings.use_velocity_commands);
            assert_eq!(word(settings.cmd_max_bits, custody), 2.0);
            assert_eq!(word(settings.initial_target_bits, custody), 0.0);
            assert_eq!(
                root.attribute("name"),
                Some("gz::sim::systems::JointPositionController")
            );
            assert_eq!(
                root.attribute("filename"),
                Some("gz-sim-joint-position-controller-system")
            );
            assert_eq!(field("joint_index"), "0");
            assert!(matches!(field("use_velocity_commands"), "true" | "1"));
            assert_eq!(
                field("cmd_max").parse::<f64>().unwrap().to_bits(),
                settings.cmd_max_bits
            );
            assert_eq!(
                field("initial_position").parse::<f64>().unwrap().to_bits(),
                settings.initial_target_bits
            );
            assert!(matches!(
                value.interfaces[2],
                native::NativeInterface::ConfigureParameters
            ));
        }
        native::NativeSettings::Velocity(settings) => {
            assert!(!position && slot % 2 == 1);
            assert!(!settings.use_force_commands);
            assert_eq!(word(settings.initial_target_bits, custody), 0.0);
            assert_eq!(
                root.attribute("name"),
                Some("gz::sim::systems::JointController")
            );
            assert_eq!(
                root.attribute("filename"),
                Some("gz-sim-joint-controller-system")
            );
            assert!(matches!(field("use_force_commands"), "false" | "0"));
            assert_eq!(
                field("initial_velocity").parse::<f64>().unwrap().to_bits(),
                settings.initial_target_bits
            );
        }
    }
    let discovery = value.discovery;
    assert!(discovery.has_connections);
    assert!(discovery.publisher_type_names.contains(&"gz.msgs.Double"));
    assert!(discovery.subscriber_type_names.contains(&"gz.msgs.Double"));
    for name in discovery
        .publisher_type_names
        .iter()
        .chain(discovery.subscriber_type_names)
    {
        text(name, 128);
    }
}

fn visit(case: &native::NativeCase) {
    assert_eq!(case.schema, "M58_ISOLATED_CONTROLLER_COMPONENT_LEDGER_V1");
    let position = matches!(case.family, native::NativeFamily::PositionAbs);
    assert_eq!(
        case.selected_slots,
        if position { &[0, 2] } else { &[1, 3] }
    );
    let (ids, models, names) = graph(case);
    let mut custody = Custody::default();
    for (index, value) in case.configurations.iter().enumerate() {
        configuration(value, index, case, &ids, &models, &mut custody);
    }
    assert_eq!(case.registry_records.len(), if position { 3 } else { 0 });
    for (index, value) in case.registry_records.iter().enumerate() {
        let expected = match value.stage {
            native::NativeRegistryStage::Empty => {
                assert_eq!(index, 0);
                0
            }
            native::NativeRegistryStage::AfterPosition0 => {
                assert_eq!(index, 1);
                8
            }
            native::NativeRegistryStage::AfterPosition2 => {
                assert_eq!(index, 2);
                16
            }
        };
        if let Some(call) = value.configure_parameters_call {
            assert!(index > 0);
            assert_eq!(usize::try_from(call.index).unwrap(), index - 1);
            let slot = usize::try_from(case.selected_slots[index - 1]).unwrap();
            assert_eq!(usize::try_from(call.slot).unwrap(), slot);
            assert_eq!(call.model_entity, models[slot / 2]);
            assert_eq!(call.joint_entity, ids[slot]);
            assert!(matches!(
                call.interface,
                native::NativeInterface::ConfigureParameters
            ));
        } else {
            assert_eq!(index, 0);
        }
        assert_eq!(value.declaration_list.declarations.len(), expected);
        let mut declared = BTreeSet::new();
        for declaration in value.declaration_list.declarations {
            assert!(
                names[..expected]
                    .iter()
                    .any(|name| name == declaration.name)
            );
            assert!(declared.insert(declaration.name));
            assert_eq!(declaration.kind, "gz.msgs.Double");
            wire(declaration.wire, &mut custody);
        }
        wire(value.declaration_list.wire, &mut custody);
        assert_eq!(value.values.len(), expected);
        parameters(value.values, &names, &mut custody);
    }
    for (index, value) in case.round_inputs.iter().enumerate() {
        assert_eq!(usize::try_from(value.round_index).unwrap(), index);
        phase(value.phase);
        match value.role {
            native::NativeRoundRole::Initial => {
                assert_eq!(index, 0);
                assert!(value.phase.is_none());
            }
            native::NativeRoundRole::Baseline
            | native::NativeRoundRole::Probe
            | native::NativeRoundRole::Measurement => {
                assert!(value.phase.is_some());
            }
        }
        if let Some(probe) = value.probe_index {
            assert!(probe < 32 && matches!(value.role, native::NativeRoundRole::Probe));
        }
        if let Some(measurement) = value.measurement_index {
            assert!(
                measurement < if position { 5 } else { 3 }
                    && matches!(value.role, native::NativeRoundRole::Measurement)
            );
        }
        assert_eq!(value.info.iterations, u64::try_from(index).unwrap());
        info(value.info);
        input_set(value.inputs, &ids, &models, &mut custody);
        command_set(
            value.commands_before,
            &ids,
            case.selected_slots,
            &mut custody,
        );
        registry_snapshot(value.registry_before, position, &names, &mut custody);
    }
    assert_eq!(case.pre_calls.len(), case.round_inputs.len() * 2);
    for (index, value) in case.pre_calls.iter().enumerate() {
        assert_eq!(usize::try_from(value.call_index).unwrap(), index);
        assert_eq!(usize::try_from(value.round_index).unwrap(), index / 2);
        assert_eq!(usize::try_from(value.selected_index).unwrap(), index % 2);
        assert_eq!(value.slot, case.selected_slots[index % 2]);
        assert_eq!(info(value.info), info(case.round_inputs[index / 2].info));
        input_set(value.inputs_after, &ids, &models, &mut custody);
        same_inputs(value.inputs_after, case.round_inputs[index / 2].inputs);
        command_set(
            value.commands_after,
            &ids,
            case.selected_slots,
            &mut custody,
        );
        let previous = if index % 2 == 0 {
            case.round_inputs[index / 2].commands_before
        } else {
            case.pre_calls[index - 1].commands_after
        };
        for (slot, (after, before)) in value.commands_after.iter().zip(previous).enumerate() {
            if slot != usize::try_from(value.slot).unwrap() {
                assert!(same_command(after, before));
            }
        }
        registry_snapshot(value.registry_after, position, &names, &mut custody);
    }
    assert_eq!(case.round_ends.len(), case.round_inputs.len());
    for (index, value) in case.round_ends.iter().enumerate() {
        assert_eq!(usize::try_from(value.round_index).unwrap(), index);
        assert_eq!(info(value.info), info(case.round_inputs[index].info));
        input_set(value.inputs_after, &ids, &models, &mut custody);
        same_inputs(value.inputs_after, case.round_inputs[index].inputs);
        command_set(
            value.commands_after,
            &ids,
            case.selected_slots,
            &mut custody,
        );
        for (end, after) in value
            .commands_after
            .iter()
            .zip(case.pre_calls[index * 2 + 1].commands_after)
        {
            assert!(same_command(end, after));
        }
        registry_snapshot(value.registry_after, position, &names, &mut custody);
    }
    assert_eq!(
        (case.publications.len(), case.taps.len(), case.fences.len()),
        (4, 4, 4)
    );
    for (index, value) in case.publications.iter().enumerate() {
        assert_eq!(usize::try_from(value.phase).unwrap(), index);
        assert_eq!(value.slot, case.selected_slots[index % 2]);
        assert_eq!(value.topic, TOPICS[usize::try_from(value.slot).unwrap()]);
        assert!(value.publish_returned_true);
        double(value.message, &mut custody);
    }
    for (index, value) in case.taps.iter().enumerate() {
        let publication = &case.publications[index];
        assert_eq!(
            (value.phase, value.slot, value.topic),
            (publication.phase, publication.slot, publication.topic)
        );
        assert_eq!(usize::try_from(value.arrival_ordinal).unwrap(), index);
        if index > 0 {
            assert!(value.arrival_elapsed_ns >= case.taps[index - 1].arrival_elapsed_ns);
        }
        assert_eq!(value.message.data_bits, publication.message.data_bits);
        assert_eq!(value.message.wire, publication.message.wire);
        double(value.message, &mut custody);
    }
    for (index, value) in case.fences.iter().enumerate() {
        assert_eq!(usize::try_from(value.phase).unwrap(), index);
        assert_eq!(value.slot, case.publications[index].slot);
        assert_eq!(value.publication_ordinal, case.publications[index].ordinal);
        assert_eq!(value.tap_ordinal, case.taps[index].ordinal);
        assert!(matches!(
            value.completion,
            native::NativeCompletion::ObservedOriginalPreUpdateComponentEffect
        ));
        assert!(matches!(
            value.evidence_scope,
            native::NativeFenceScope::TapIsNotPrivateCallbackAck
        ));
        assert_eq!(
            value.probe_round_indices.len(),
            usize::try_from(case.summary.probe_round_counts[index]).unwrap()
        );
        assert!((1..=32).contains(&value.probe_round_indices.len()));
        let baseline = &case.round_inputs[usize::try_from(value.baseline_round_index).unwrap()];
        assert!(matches!(baseline.role, native::NativeRoundRole::Baseline));
        assert_eq!(baseline.phase, Some(value.phase));
        assert_eq!(
            value.matched_probe_round_index,
            *value.probe_round_indices.last().unwrap()
        );
        let baseline_commands =
            case.round_ends[usize::try_from(value.baseline_round_index).unwrap()].commands_after;
        let targeted = usize::try_from(value.slot).unwrap();
        let marker = case.publications[index].message.data_bits;
        assert_ne!(
            baseline_commands[targeted].joint_velocity_command,
            Some(std::slice::from_ref(&marker))
        );
        for (probe, round) in value.probe_round_indices.iter().enumerate() {
            let input = &case.round_inputs[usize::try_from(*round).unwrap()];
            assert!(matches!(input.role, native::NativeRoundRole::Probe));
            assert_eq!(input.phase, Some(value.phase));
            assert_eq!(input.probe_index, Some(u32::try_from(probe).unwrap()));
            let after = case.round_ends[usize::try_from(*round).unwrap()].commands_after;
            let peers_unchanged = after
                .iter()
                .zip(baseline_commands)
                .enumerate()
                .all(|(slot, (after, before))| slot == targeted || same_command(after, before));
            let matched = after[targeted].joint_velocity_command
                == Some(std::slice::from_ref(&marker))
                && peers_unchanged;
            assert_eq!(
                matched,
                probe + 1 == value.probe_round_indices.len(),
                "every unsuccessful probe is retained before the final observed effect"
            );
        }
    }
    let counts = case.summary.counts;
    assert_eq!(
        case.summary.state,
        "ORIGINAL_M58_COMPONENT_CAPTURE_COMPLETED_PENDING_INDEPENDENT_DATA"
    );
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
        (counts.publications, counts.tap_records, counts.fences),
        (4, 4, 4)
    );
    assert_eq!(
        counts.command_state_getbacks,
        u64::try_from(custody.command_sets * 4).unwrap()
    );
    assert_eq!(
        counts.helper_registry_get_calls,
        u64::try_from(custody.parameter_values).unwrap()
    );
    assert_eq!(custody.input_sets, case.round_inputs.len() * 4);
    assert!(custody.words > 0 && custody.wire_bytes > 0);
    assert_eq!(
        counts.raw_rows,
        u64::try_from(case.raw_order.len()).unwrap()
    );
    let mut indices = [0usize; 9];
    for (ordinal, row) in case.raw_order.iter().enumerate() {
        let (kind, actual) = match row.kind {
            native::NativeKind::Config => (0, case.configurations[row.index].ordinal),
            native::NativeKind::Registry => (1, case.registry_records[row.index].ordinal),
            native::NativeKind::RoundInput => (2, case.round_inputs[row.index].ordinal),
            native::NativeKind::Pre => (3, case.pre_calls[row.index].ordinal),
            native::NativeKind::RoundEnd => (4, case.round_ends[row.index].ordinal),
            native::NativeKind::Publication => (5, case.publications[row.index].ordinal),
            native::NativeKind::Tap => (6, case.taps[row.index].ordinal),
            native::NativeKind::Fence => (7, case.fences[row.index].ordinal),
            native::NativeKind::Summary => {
                assert_eq!(row.index, 0);
                (8, case.summary.ordinal)
            }
        };
        assert_eq!(row.index, indices[kind]);
        indices[kind] += 1;
        assert_eq!(row.raw_ordinal, u64::try_from(ordinal).unwrap());
        assert_eq!(actual, row.raw_ordinal);
    }
    assert_eq!(
        indices,
        [
            case.configurations.len(),
            case.registry_records.len(),
            case.round_inputs.len(),
            case.pre_calls.len(),
            case.round_ends.len(),
            4,
            4,
            4,
            1
        ]
    );
}

#[derive(Debug)]
struct LoadedSlot {
    entity: Entity,
    id: u64,
    configuration: JointControllerConfiguration,
}

fn loaded_slots(sim: &Simulation) -> [LoadedSlot; 4] {
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|node| node.contains::<Controller>())
            .count(),
        4
    );
    let initial: Vec<_> = initial_joint_controller_configurations_with_identity(sim)
        .unwrap()
        .collect();
    assert_eq!(initial.len(), 4);
    std::array::from_fn(|slot| {
        let name = format!("default::{}::{}", MODELS[slot / 2], JOINTS[slot % 2]);
        let original = initial.iter().find(|row| row.name == name).unwrap();
        assert_eq!(original.kind, EntityKind::Joint);
        let entity = gz_sim_core::entity_by_id(sim.world(), original.id).unwrap();
        assert_eq!(sim.world().get::<Name>(entity).unwrap().0, name);
        assert_eq!(
            sim.world().get::<EntityKind>(entity),
            Some(&EntityKind::Joint)
        );
        let parent = sim.world().get::<ChildOf>(entity).unwrap().parent();
        assert_eq!(
            sim.world().get::<Name>(parent).unwrap().0,
            format!("default::{}", MODELS[slot / 2])
        );
        assert_eq!(
            original.parent,
            Some(sim.world().get::<EntityId>(parent).unwrap().get())
        );
        let publication = sim
            .world()
            .get::<JointControllerConfiguration>(entity)
            .unwrap();
        assert_eq!(publication, original.configuration);
        assert_eq!(publication.model_name, MODELS[slot / 2]);
        assert_eq!(publication.joint_name, JOINTS[slot % 2]);
        assert_eq!(publication.topic, TOPICS[slot]);
        assert_eq!(
            publication.mode,
            if slot % 2 == 0 {
                JointControllerMode::Position
            } else {
                JointControllerMode::Velocity
            }
        );
        let controller = sim.world().get::<Controller>(entity).unwrap();
        assert_eq!(controller.owner, original.id);
        assert_eq!(controller.publication, *publication);
        assert_eq!(
            sim.world().get::<JointHandle>(entity).unwrap().id(),
            controller.joint
        );
        assert!(joint_controller_binding_matches(sim, original.id));
        LoadedSlot {
            entity,
            id: original.id,
            configuration: publication.clone(),
        }
    })
}

#[derive(Debug, PartialEq, Eq)]
struct IdentitySnapshot {
    id: u64,
    name: String,
    kind: EntityKind,
    parent: Option<u64>,
    configuration: JointControllerConfiguration,
}

#[derive(Debug, PartialEq, Eq)]
struct StableScene {
    elapsed: Duration,
    epoch: u64,
    paused: bool,
    step: Duration,
    info: UpdateInfo,
    joints: Vec<(u64, [u64; 2])>,
    bodies: Vec<(u64, [u64; 13])>,
    world_poses: Vec<(u64, [u64; 7])>,
    local_poses: Vec<(u64, [u64; 7])>,
    originals: Vec<IdentitySnapshot>,
    live: Vec<IdentitySnapshot>,
}

fn pose_bits(pose: WorldPose) -> [u64; 7] {
    [
        pose.translation[0].to_bits(),
        pose.translation[1].to_bits(),
        pose.translation[2].to_bits(),
        pose.rotation[0].to_bits(),
        pose.rotation[1].to_bits(),
        pose.rotation[2].to_bits(),
        pose.rotation[3].to_bits(),
    ]
}

fn stable_scene(sim: &Simulation, slots: &[LoadedSlot; 4]) -> StableScene {
    let world = sim.world();
    let backend = world.resource::<PhysicsBackend>();
    let joints = slots
        .iter()
        .map(|slot| {
            let controller = world.get::<Controller>(slot.entity).unwrap();
            let state = backend.joint_state(controller.joint).unwrap();
            (
                slot.id,
                [state.position.to_bits(), state.velocity.to_bits()],
            )
        })
        .collect();
    let mut bodies = Vec::new();
    let mut world_poses = Vec::new();
    let mut local_poses = Vec::new();
    for node in world.iter_entities() {
        let Some(id) = node.get::<EntityId>().map(EntityId::get) else {
            continue;
        };
        if let Some(handle) = node.get::<BodyHandle>() {
            let state = backend.state(handle.id()).unwrap();
            let values: Vec<_> = state
                .position
                .into_iter()
                .chain(state.velocity)
                .chain(state.orientation)
                .chain(state.angular_velocity)
                .map(f64::to_bits)
                .collect();
            bodies.push((id, <[u64; 13]>::try_from(values).unwrap()));
        }
        if let Some(pose) = node.get::<WorldPose>() {
            world_poses.push((id, pose_bits(*pose)));
        }
        if let Some(pose) = node.get::<LocalPose>() {
            local_poses.push((id, pose_bits(pose.0)));
        }
    }
    bodies.sort_by_key(|row| row.0);
    world_poses.sort_by_key(|row| row.0);
    local_poses.sort_by_key(|row| row.0);
    assert!(bodies.len() >= 4 && !world_poses.is_empty() && !local_poses.is_empty());
    let mut originals: Vec<_> = initial_joint_controller_configurations_with_identity(sim)
        .unwrap()
        .map(|value| IdentitySnapshot {
            id: value.id,
            name: value.name.to_owned(),
            kind: value.kind,
            parent: value.parent,
            configuration: value.configuration.clone(),
        })
        .collect();
    originals.sort_by_key(|row| row.id);
    let mut live: Vec<_> = slots
        .iter()
        .map(|slot| {
            assert!(joint_controller_binding_matches(sim, slot.id));
            let parent = world.get::<ChildOf>(slot.entity).unwrap().parent();
            IdentitySnapshot {
                id: slot.id,
                name: world.get::<Name>(slot.entity).unwrap().0.clone(),
                kind: *world.get::<EntityKind>(slot.entity).unwrap(),
                parent: Some(world.get::<EntityId>(parent).unwrap().get()),
                configuration: world
                    .get::<JointControllerConfiguration>(slot.entity)
                    .unwrap()
                    .clone(),
            }
        })
        .collect();
    live.sort_by_key(|row| row.id);
    assert_eq!(live, originals);
    StableScene {
        elapsed: sim.elapsed(),
        epoch: sim.epoch(),
        paused: sim.is_paused(),
        step: sim.step_duration(),
        info: *world.resource::<UpdateInfo>(),
        joints,
        bodies,
        world_poses,
        local_poses,
        originals,
        live,
    }
}

fn pending_targets(sim: &Simulation, slots: &[LoadedSlot; 4]) -> [u64; 4] {
    std::array::from_fn(|slot| {
        sim.world()
            .get::<Controller>(slots[slot].entity)
            .unwrap()
            .target
            .to_bits()
    })
}

fn check_loaded_configuration(config: &native::NativeConfiguration, slot: &LoadedSlot) {
    assert_eq!(slot.configuration.model_name, config.model_name);
    assert_eq!(slot.configuration.joint_name, config.joint_name);
    assert_eq!(slot.configuration.topic, config.topic);
    assert_eq!(
        slot.configuration.mode,
        match config.settings {
            native::NativeSettings::Position(_) => JointControllerMode::Position,
            native::NativeSettings::Velocity(_) => JointControllerMode::Velocity,
        }
    );
    // Native numeric entity IDs stay in their own process domain. The mapping
    // uses captured owner/local names and immutable genuine Rust identities.
}

#[test]
fn captured_isolated_controller_commands_preserve_loaded_owner_targets_without_backend_writes() {
    for case in [native::NATIVE_RUNS.position, native::NATIVE_RUNS.velocity] {
        visit(case);
        let mut sim = load_world_with_sdf_step(
            SOURCE,
            "m58-observed-inputs.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let slots = loaded_slots(&sim);
        let before = stable_scene(&sim, &slots);
        let mut expected = pending_targets(&sim, &slots);
        assert_eq!(expected, [0.0_f64.to_bits(); 4]);
        for config in case.configurations {
            check_loaded_configuration(config, &slots[usize::try_from(config.slot).unwrap()]);
        }
        // Read only the genuine captured publication INPUTS. Native q/v,
        // Info/paused/ABS outputs remain visited observations, never ECS writes.
        for publication in case.publications {
            let index = usize::try_from(publication.slot).unwrap();
            let config = case
                .configurations
                .iter()
                .find(|row| row.slot == publication.slot)
                .unwrap();
            assert_eq!(publication.topic, config.topic);
            check_loaded_configuration(config, &slots[index]);
            let command = f64::from_bits(publication.message.data_bits);
            assert!(command.is_finite());
            let previous = pending_targets(&sim, &slots);
            assert_eq!(previous, expected);
            assert_ne!(
                previous[index],
                command.to_bits(),
                "this captured input changes the pending target"
            );
            if matches!(case.family, native::NativeFamily::Velocity) {
                assert!(
                    command.abs()
                        <= sim
                            .world()
                            .get::<Controller>(slots[index].entity)
                            .unwrap()
                            .max_velocity
                );
            }
            set_joint_controller_command(&mut sim, slots[index].id, command).unwrap();
            expected[index] = command.to_bits();
            assert_eq!(pending_targets(&sim, &slots), expected);
            for (other, (after, before)) in expected.iter().zip(previous).enumerate() {
                if other != index {
                    assert_eq!(*after, before);
                }
            }
            if matches!(case.family, native::NativeFamily::Velocity) {
                // All captured velocity markers are within the common admitted
                // target range. Compare the native ROUTED command, not motion.
                let fence = case
                    .fences
                    .iter()
                    .find(|row| row.phase == publication.phase)
                    .unwrap();
                let actual = &case.round_ends
                    [usize::try_from(fence.matched_probe_round_index).unwrap()]
                .commands_after[index];
                assert_eq!(
                    actual.joint_velocity_command,
                    Some(std::slice::from_ref(&publication.message.data_bits))
                );
                assert_eq!(expected[index], publication.message.data_bits);
            }
            assert_eq!(stable_scene(&sim, &slots), before);
            assert!(!sim.is_faulted());
        }
        let valid = pending_targets(&sim, &slots);
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                set_joint_controller_command(&mut sim, slots[0].id, invalid),
                Err(JointControllerCommandError::InvalidCommand)
            ));
            assert_eq!(pending_targets(&sim, &slots), valid);
            assert_eq!(stable_scene(&sim, &slots), before);
        }
        // This is a deliberately invalid Rust admission request, never emitted
        // or labelled as an observation from the original native reference.
        assert!(matches!(
            set_joint_controller_command(&mut sim, 0, 0.25),
            Err(JointControllerCommandError::MissingController(0))
        ));
        assert_eq!(pending_targets(&sim, &slots), valid);
        assert_eq!(stable_scene(&sim, &slots), before);
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
}
