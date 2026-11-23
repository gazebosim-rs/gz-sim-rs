// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, LoadError, Name, PhysicsEngine, damped_sole::last_step, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use sdformat::NoResources;

const CONTROL: &str = r#"<plugin name="gz::sim::systems::DampedSole" filename="gz-sim-damped-sole-system">
    <foot>default::foot::body</foot><support>default::deck::body</support>
    <foot_stiffness>1000000</foot_stiffness><deck_stiffness>1000000</deck_stiffness>
    <damping>1</damping><friction>0.5</friction><slip_speed>0.002</slip_speed>
    <substeps>4</substeps><quadrature_depth>0</quadrature_depth></plugin>"#;
const ADAPT: &str = "<max_substeps>16</max_substeps><max_quadrature_depth>2</max_quadrature_depth><position_tolerance>1e-5</position_tolerance><rotation_tolerance>1e-5</rotation_tolerance><impulse_tolerance>1e-3</impulse_tolerance><angular_impulse_tolerance>1e-3</angular_impulse_tolerance><heat_tolerance>1e-5</heat_tolerance>";
fn scene(height: f64) -> String {
    format!(
        r#"<sdf version="1.12"><world name="default"><physics name="p" type="ode"><max_step_size>0.0005</max_step_size></physics><gravity>0 0 0</gravity>{CONTROL}
    <model name="foot"><pose>0 0 {height} 0 0 0</pose><link name="body"><inertial><pose>0 0 0.07 0 0 0</pose><mass>2</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial><collision name="c"><geometry><box><size>0.36 0.2 0.12</size></box></geometry></collision></link></model>
    <model name="deck"><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0.01 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name="body"><collision name="c"><geometry><box><size>1 1 0.12</size></box></geometry></collision></link></model></world></sdf>"#
    )
}
fn load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "coating.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn id(s: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn read(s: &Simulation, link: u64) -> BodyState {
    let e = entity_by_id(s.world(), link).unwrap();
    s.world()
        .resource::<PhysicsBackend>()
        .state(s.world().get::<BodyHandle>(e).unwrap().id())
        .unwrap()
}

#[test]
fn held_wrench_velocity_commands_pause_and_public_clock() {
    let mut sim = load(&scene(0.5)).unwrap();
    let f = id(&sim, "default::foot::body");
    let d = id(&sim, "default::deck::body");
    let start = read(&sim, f);
    sim.set_paused(true);
    queue_link_world_wrench(&mut sim, f, [4., 0., 0.], [0., 0.1, 0.]).unwrap();
    queue_link_world_velocity(&mut sim, d, [0.25, 0., 0.], [0.; 3]).unwrap();
    assert!(queue_link_world_velocity(&mut sim, d, [f64::NAN; 3], [0., 1., 0.]).is_err());
    sim.tick().unwrap();
    assert_eq!(start, read(&sim, f));
    assert!(last_step(&sim).is_none());
    sim.step_exact(1).unwrap();
    let row = *last_step(&sim).unwrap();
    assert_eq!((row.foot, row.support, row.time.iteration), (f, d, 1));
    assert_eq!(row.step.normal_impulse, 0.);
    assert!((row.deck.center.x - 0.25 * 0.0005).abs() < 1e-14);
    assert!((row.state.velocity.x - 0.001).abs() < 1e-14);
    assert!((row.state.spin.y - 0.001).abs() < 1e-14);
    sim.tick().unwrap();
    assert_eq!(last_step(&sim).unwrap().time, row.time);
    sim.step_exact(1).unwrap();
    assert!((last_step(&sim).unwrap().state.velocity.x - row.state.velocity.x).abs() < 1e-14);
    assert!((last_step(&sim).unwrap().state.spin.y - row.state.spin.y).abs() < 1e-14);
}

#[test]
fn losing_deck_and_loaded_reset_restore_the_registered_pair() {
    let mut s = load(&scene(0.119)).unwrap();
    s.step_exact(1).unwrap();
    let first = *last_step(&s).unwrap();
    assert!(first.step.normal_impulse > 0.);
    assert!((first.step.impulse.z - 2. * first.state.velocity.z).abs() < 1e-13);
    let deck_model = id(&s, "default::deck");
    remove_model(&mut s, deck_model).unwrap();
    s.step_exact(2).unwrap();
    assert!(last_step(&s).is_none());
    assert_eq!(read(&s, first.foot).velocity[2], first.state.velocity.z);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_step(&s).is_none());
    s.step_exact(1).unwrap();
    let replay = last_step(&s).unwrap();
    assert_eq!(replay.foot, first.foot);
    assert_eq!(replay.support, first.support);
    assert_eq!(replay.step.normal_impulse, first.step.normal_impulse);
    assert_eq!(replay.state.center, first.state.center);
    assert!(replay.time.epoch > first.time.epoch);
}

#[test]
fn optional_adaptive_policy_publishes_the_accepted_grid() {
    let mut s = load(&scene(0.119).replacen("</plugin>", &format!("{ADAPT}</plugin>"), 1)).unwrap();
    s.step_exact(1).unwrap();
    let row = last_step(&s).unwrap();
    let estimate = row.adaptive.unwrap();
    assert!(estimate.substeps >= 8 && estimate.depth >= 1);
    assert_eq!(estimate.step.impulse, row.step.impulse);
    assert_eq!(estimate.step.contact.force, row.step.contact.force);
}

#[test]
fn invalid_policy_is_rejected_and_unregistered_world_uses_native_contacts() {
    for (a, b) in [
        ("default::deck::body", "default::missing::body"),
        ("<substeps>4", "<substeps>0"),
        ("<slip_speed>0.002", "<slip_speed>NaN"),
        ("<quadrature_depth>0", "<quadrature_depth>6"),
        ("<damping>", "<unknown>1</unknown><damping>"),
        ("<damping>", "<damping>1</damping><damping>"),
        ("default::foot::body", "default::deck::body"),
        ("</plugin>", "<max_substeps>16</max_substeps></plugin>"),
    ] {
        assert!(load(&scene(0.119).replacen(a, b, 1)).is_err(), "{a} -> {b}");
    }
    let mut s = load(&scene(0.115).replace(CONTROL, "")).unwrap();
    let f = id(&s, "default::foot::body");
    s.step_exact(20).unwrap();
    assert!(last_step(&s).is_none());
    assert!(read(&s, f).position[2] > 0.115);
}

fn split_deck(stiffness: Option<f64>) -> String {
    let base = scene(0.119);
    let from = base.find("<model name=\"deck\">").unwrap();
    let to = from + base[from..].find("</model>").unwrap() + "</model>".len();
    let deck = base[from..to]
        .replace("1 1 0.12", "0.5 1 0.12")
        .replace(">0.01 0 0<", ">0 0 0<");
    let left = deck.replace(
        "<model name=\"deck\">",
        "<model name=\"left\"><pose>-0.25 0 0 0 0 0</pose>",
    );
    let right = deck.replace(
        "<model name=\"deck\">",
        "<model name=\"right\"><pose>0.25 0 0 0 0 0</pose>",
    );
    let attr = stiffness.map_or(String::new(), |k| format!(" deck_stiffness=\"{k}\""));
    format!("{}{left}{right}{}", &base[..from], &base[to..]).replace(
        "<support>default::deck::body</support>",
        &format!(
            "<support{attr}>default::right::body</support><support>default::left::body</support>"
        ),
    )
}

#[test]
fn disjoint_same_material_panels_share_load_and_stiffer_layer_moves_its_share() {
    use gz_sim_scene::damped_sole::panel_steps;
    let mut equal = load(&split_deck(None)).unwrap();
    equal.step_exact(1).unwrap();
    let row = last_step(&equal).unwrap();
    let both = panel_steps(&equal);
    let a = both[0].step.normal_impulse;
    let b = both[1].step.normal_impulse;
    assert_eq!(both.len(), 2);
    assert!(both[0].support < both[1].support);
    assert!((a - b).abs() < 1e-4 * a && a > 0.);
    assert!((a + b - row.state.velocity.z * 2.).abs() < 1e-13);
    let mut varied = load(&split_deck(Some(3e6))).unwrap();
    varied.step_exact(1).unwrap();
    let shifted = panel_steps(&varied);
    assert_eq!(
        [both[0].support, both[1].support],
        [shifted[0].support, shifted[1].support]
    );
    assert!(shifted[1].step.normal_impulse > 1.49 * shifted[0].step.normal_impulse);
    let left = id(&varied, "default::left");
    let keep = shifted[1].step.normal_impulse;
    varied.set_paused(true);
    remove_model(&mut varied, left).unwrap();
    assert_eq!(panel_steps(&varied).len(), 1);
    assert_eq!(last_step(&varied).unwrap().step.normal_impulse, keep);
    varied.tick().unwrap();
    assert_eq!(panel_steps(&varied)[0].step.normal_impulse, keep);
    varied.step_exact(1).unwrap();
    assert!(panel_steps(&varied)[0].step.normal_impulse > 0.);
    reset_loaded_world(&mut varied, |_| ()).unwrap();
    assert!(panel_steps(&varied).is_empty());
    varied.step_exact(1).unwrap();
    assert_eq!(panel_steps(&varied)[1].step.normal_impulse, keep);
}

#[test]
fn interpenetrating_panel_paths_fail_before_committing_any_endpoint() {
    use gz_sim_scene::damped_sole::panel_steps;
    let xml = split_deck(None);
    assert!(load(&xml.replace("-0.25 0 0 0 0 0", "-0.24 0 0 0 0 0")).is_err());
    let mut s = load(&xml).unwrap();
    s.step_exact(1).unwrap();
    let left = id(&s, "default::left::body");
    let right = id(&s, "default::right::body");
    let foot = id(&s, "default::foot::body");
    let before = [left, right, foot].map(|i| read(&s, i).position);
    let previous = panel_steps(&s)[0].step.impulse;
    queue_link_world_velocity(&mut s, left, [0.5, 0., 0.], [0.; 3]).unwrap();
    assert!(s.step_exact(1).is_err());
    assert_eq!(before, [left, right, foot].map(|i| read(&s, i).position));
    assert_eq!(panel_steps(&s)[0].step.impulse, previous);
}

#[test]
fn swapped_native_panel_bindings_are_rejected_before_motion() {
    let mut s = load(&split_deck(Some(3e6))).unwrap();
    let left = id(&s, "default::left::body");
    let right = id(&s, "default::right::body");
    let e = entity_by_id(s.world(), left).unwrap();
    let other = entity_by_id(s.world(), right).unwrap();
    let left_handle = s.world_mut().entity_mut(e).take::<BodyHandle>().unwrap();
    let right_handle = s
        .world_mut()
        .entity_mut(other)
        .take::<BodyHandle>()
        .unwrap();
    s.world_mut().entity_mut(e).insert(right_handle);
    s.world_mut().entity_mut(other).insert(left_handle);
    assert!(s.step_exact(1).is_err());
    assert!(last_step(&s).is_none());
}

#[test]
fn adaptive_seam_checks_work_on_each_panel_and_survives_removal() {
    let xml = split_deck(Some(3e6)).replacen(
        "</plugin>",
        &format!("{ADAPT}<work_tolerance>0.00001</work_tolerance></plugin>"),
        1,
    );
    let mut sim = load(&xml).unwrap();
    sim.step_exact(1).unwrap();
    let check = |s: &Simulation| {
        let grid = last_step(s).unwrap().panel_accuracy.unwrap();
        assert!(grid.trials >= 3 && grid.substeps >= 8 && grid.depth >= 1);
        for e in [grid.temporal, grid.spatial] {
            assert!(e.support_work_j <= 1e-5 && e.sliding_work_j <= 1e-5 && e.rim_work_j <= 1e-5);
        }
    };
    check(&sim);
    let panel = id(&sim, "default::right");
    remove_model(&mut sim, panel).unwrap();
    sim.step_exact(1).unwrap();
    check(&sim);
    assert_eq!(gz_sim_scene::damped_sole::panel_steps(&sim).len(), 1);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    check(&sim);
    assert_eq!(gz_sim_scene::damped_sole::panel_steps(&sim).len(), 2);
}

#[test]
fn spinning_support_publishes_rotation_and_spin_work_then_stops_on_command() {
    use glam::{DQuat, DVec3};
    use gz_sim_scene::damped_sole::panel_steps;
    let xml = scene(0.119)
        .replace(">0.01 0 0<", ">0 0 0<")
        .replace("<angular_velocity>0 0 0<", "<angular_velocity>0 0 2<")
        .replacen(
            "</plugin>",
            "<flat_elastic_fast_path>true</flat_elastic_fast_path></plugin>",
            1,
        );
    let mut sim = load(&xml).unwrap();
    let d = id(&sim, "default::deck::body");
    sim.step_exact(2).unwrap();
    let row = panel_steps(&sim)[0];
    assert!(
        (row.deck.orientation * DVec3::X - DQuat::from_rotation_z(0.002) * DVec3::X).length()
            < 1e-13
    );
    assert_eq!(row.angular_velocity, DVec3::Z * 2.);
    assert!(row.spin_work_j > 0. && row.step.angular_impulse.z > 0.);
    assert_eq!(row.spin_work_j, row.support_work_j);
    assert!(!last_step(&sim).unwrap().flat_elastic);
    assert_eq!(row.deck.orientation.to_array(), read(&sim, d).orientation);
    sim.set_paused(true);
    queue_link_world_velocity(&mut sim, d, [0.; 3], [0.; 3]).unwrap();
    sim.tick().unwrap();
    assert_eq!(panel_steps(&sim)[0].spin_work_j, row.spin_work_j);
    sim.step_exact(1).unwrap();
    assert_eq!(panel_steps(&sim)[0].deck.orientation, row.deck.orientation);
    assert_eq!(panel_steps(&sim)[0].spin_work_j, 0.);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2).unwrap();
    assert_eq!(
        panel_steps(&sim)[0].step.angular_impulse,
        row.step.angular_impulse
    );
    let mut adaptive = load(&xml.replacen("</plugin>", &format!("{ADAPT}</plugin>"), 1)).unwrap();
    adaptive.step_exact(1).unwrap();
    let turn = last_step(&adaptive).unwrap().turntable_accuracy.unwrap();
    assert!(turn.substeps >= 8 && turn.depth >= 1 && turn.trials >= 3);
    assert_eq!(
        turn.result.spin_work_j,
        panel_steps(&adaptive)[0].spin_work_j
    );
}

#[test]
fn turning_seam_motion_is_rejected_at_authoring_and_command_admission() {
    let xml = split_deck(None);
    assert!(
        load(&xml.replacen("<angular_velocity>0 0 0<", "<angular_velocity>0 0 1<", 1)).is_err()
    );
    let mut sim = load(&xml).unwrap();
    let d = id(&sim, "default::right::body");
    assert!(queue_link_world_velocity(&mut sim, d, [0.; 3], [0., 0., 1.]).is_err());
    sim.step_exact(1).unwrap();
    assert_eq!(read(&sim, d).angular_velocity, [0.; 3]);
}

#[test]
fn independent_surface_grip_and_damping_control_panel_traction_and_heat() {
    use gz_sim_scene::damped_sole::panel_steps;
    let xml = split_deck(None)
        .replace(
            "<support>default::left",
            "<support friction=\"0\" damping=\"0\">default::left",
        )
        .replace(
            "<support>default::right",
            "<support friction=\"0.8\" damping=\"4\" slip_speed=\"0.001\">default::right",
        );
    let mut s = load(&xml).unwrap();
    for name in ["left", "right"] {
        let d = id(&s, &format!("default::{name}::body"));
        queue_link_world_velocity(&mut s, d, [0.2, 0., 0.1], [0.; 3]).unwrap();
    }
    s.step_exact(1).unwrap();
    let rows = panel_steps(&s);
    assert_eq!(rows[0].step.impulse.x, 0.);
    assert_eq!(rows[0].step.dissipated_energy, 0.);
    assert!(rows[1].step.impulse.x > 0. && rows[1].step.dissipated_energy > 0.);
    assert!(rows[1].step.normal_impulse > rows[0].step.normal_impulse);
    assert!(rows[1].step.sliding_work < 0.);
    assert_eq!(
        rows[1].support_work_j,
        rows[1].step.impulse.dot(rows[1].deck.velocity)
    );
    for attr in [
        "friction=\"-0.1\"",
        "damping=\"NaN\"",
        "slip_speed=\"0\"",
        "foot_stiffness=\"1\"",
        "deck_stiffness=\"inf\"",
    ] {
        assert!(
            load(&split_deck(None).replace(
                "<support>default::left",
                &format!("<support {attr}>default::left")
            ))
            .is_err()
        );
    }
}

#[test]
fn semantic_frames_resolve_box_and_com_and_unsupported_shapes_fail_closed() {
    use glam::DVec3;
    let base = scene(0.5);
    let framed=base.replace("<link name=\"body\"><inertial>",r#"<frame name="datum"><pose>0 0 0.02 0 0 0</pose></frame><link name="body"><inertial>"#)
        .replace("<inertial><pose>0 0 0.07","<inertial><pose relative_to=\"datum\">0 0 0.05")
        .replacen("<collision name=\"c\"><geometry>",r#"<collision name="c"><pose relative_to="datum">0 0 -0.02 0 0 0</pose><geometry>"#,1);
    let mut a = load(&base).unwrap();
    let mut b = load(&framed).unwrap();
    a.step_exact(2).unwrap();
    b.step_exact(2).unwrap();
    assert!(
        last_step(&a)
            .unwrap()
            .state
            .center
            .distance(last_step(&b).unwrap().state.center)
            < 1e-14
    );
    let pose = last_step(&a).unwrap().state.center;
    assert!(pose.distance(DVec3::new(0., 0., 0.57)) < 1e-14);
    for (from, to) in [
        (
            "<collision name=\"c\"><geometry>",
            "<collision name=\"c\"><pose>0.01 0 0 0 0 0</pose><geometry>",
        ),
        (
            "<collision name=\"c\"><geometry>",
            "<collision name=\"c\"><pose>0 0 0 0 0 0.1</pose><geometry>",
        ),
        ("0 0 0.07 0 0 0", "0 0 0.07 0 0 0.2"),
        ("<ixx>0.05</ixx>", "<ixy>0.001</ixy><ixx>0.05</ixx>"),
        (
            "<box><size>0.36 0.2 0.12</size></box>",
            "<sphere><radius>0.1</radius></sphere>",
        ),
    ] {
        assert!(load(&base.replacen(from, to, 1)).is_err(), "{from} -> {to}");
    }
}

#[test]
fn reset_and_paused_removal_cancel_pending_inputs_without_changing_survivor_commands() {
    let xml = split_deck(None);
    let mut s = load(&xml).unwrap();
    let left = id(&s, "default::left::body");
    let right = id(&s, "default::right::body");
    let foot = id(&s, "default::foot::body");
    s.step_exact(1).unwrap();
    let reference = last_step(&s).unwrap().state;
    s.set_paused(true);
    queue_link_world_velocity(&mut s, left, [0., 0., 1.], [0.; 3]).unwrap();
    queue_link_world_velocity(&mut s, right, [0., 0., 0.04], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut s, foot, [3., 0., 0.], [0.; 3]).unwrap();
    s.tick().unwrap();
    let l = id(&s, "default::left");
    remove_model(&mut s, l).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(read(&s, right).velocity, [0., 0., 0.04]);
    queue_link_world_velocity(&mut s, right, [0., 0., 9.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut s, foot, [9.; 3], [8.; 3]).unwrap();
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(last_step(&s).unwrap().state.center, reference.center);
    assert_eq!(last_step(&s).unwrap().state.velocity, reference.velocity);
    assert_eq!(read(&s, right).velocity, [0.; 3]);
    assert_eq!(read(&s, left).velocity, [0.; 3]);
}

#[test]
fn static_collisionless_observer_sees_a_foot_without_perturbing_its_contact() {
    use gz_sim_scene::{LidarSample, VisualGeometry};
    let fixture = scene(0.119);
    let observer = r#"<model name="observer"><static>true</static><pose>-1 0 0.12 0 0 0</pose><link name="mount">
        <visual name="marker"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></visual>
        <sensor name="scanner" type="lidar"><always_on>true</always_on><update_rate>0</update_rate><lidar><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.01</min_angle><max_angle>0.01</max_angle></horizontal></scan><range><min>0</min><max>4</max><resolution>0.001</resolution></range></lidar></sensor>
        </link></model>"#;
    let xml = fixture.replace("</world>", &format!("{observer}</world>"));
    let mut s = load(&xml).unwrap();
    let mut baseline = load(&fixture).unwrap();
    baseline.step_exact(3).unwrap();
    s.step_exact(3).unwrap();
    let link = id(&s, "default::observer::mount");
    let sensor = id(&s, "default::observer::mount::scanner");
    let visual = id(&s, "default::observer::mount::marker");
    assert!(
        s.world()
            .get::<BodyHandle>(entity_by_id(s.world(), link).unwrap())
            .is_none()
    );
    assert!(
        s.world()
            .get::<VisualGeometry>(entity_by_id(s.world(), visual).unwrap())
            .is_some()
    );
    let scan = |s: &Simulation| {
        s.world()
            .get::<LidarSample>(entity_by_id(s.world(), sensor).unwrap())
            .unwrap()
            .ranges[1]
    };
    let first = scan(&s);
    assert!((first - 0.82).abs() < 0.001, "lidar range {first}");
    assert_eq!(
        last_step(&s).unwrap().state.center,
        last_step(&baseline).unwrap().state.center
    );
    assert_eq!(
        last_step(&s).unwrap().step.impulse,
        last_step(&baseline).unwrap().step.impulse
    );
    let model = id(&s, "default::observer");
    remove_model(&mut s, model).unwrap();
    s.step_exact(2).unwrap();
    baseline.step_exact(2).unwrap();
    assert_eq!(
        last_step(&s).unwrap().state.center,
        last_step(&baseline).unwrap().state.center
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(3).unwrap();
    assert_eq!(scan(&s), first);
    assert!(load(&xml.replace("<static>true</static>", "<static>false</static>")).is_err());
    let extra=observer.replace("<visual name=\"marker\">","<collision name=\"c\"><geometry><sphere><radius>0.01</radius></sphere></geometry></collision><visual name=\"marker\">");
    assert!(load(&fixture.replace("</world>", &format!("{extra}</world>"))).is_err());
}
