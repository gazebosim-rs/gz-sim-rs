// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{
    ModelState, ModelTemplate, prepare_model_template, remove_model, set_model_state,
    spawn_model_template,
};
use sdformat::NoResources;

#[test]
fn normal_scene_template_wakes_and_hits_the_committed_rotated_panel() {
    const TARGET: &str = r#"<model name="stock"><pose>20 0 0 0 0 0</pose><link name="ball">
<inertial><mass>0.8</mass><inertia><ixx>0.002048</ixx><iyy>0.002048</iyy><izz>0.002048</izz></inertia></inertial>
<collision name="ball"><geometry><sphere><radius>0.08</radius></sphere></geometry></collision>
<visual name="visible"><geometry><sphere><radius>0.08</radius></sphere></geometry></visual>
</link></model>"#;
    let xml = ARRAY.replace("</world>", &format!("{TARGET}</world>"));
    let setup = || -> (Simulation, Mechanism, ModelTemplate) {
        let mut sim = load(&xml).unwrap();
        let rig = Mechanism::bind(&sim, &xml, "satellite");
        let sdf = sdformat::load_str(&xml, "arrive.sdf", &NoResources).unwrap();
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[1]).unwrap();
        let stock = coasting::stable_id(&sim, "default::stock");
        remove_model(&mut sim, stock).unwrap();
        rig.prepare(&mut sim, 1., 1000, |_, _| {}).unwrap();
        (sim, rig, template)
    };
    let (mut coast, rig, ct) = setup();
    let (mut native, nrig, nt) = setup();
    assert_eq!(rig, nrig);
    begin_model_free_coast(&mut coast, rig.model).unwrap();
    let mut impulses = [0.; 2];
    let mut spawned = Vec::new();
    for (i, (s, template)) in [(&mut coast, ct), (&mut native, nt)]
        .into_iter()
        .enumerate()
    {
        s.step_exact(1400).unwrap();
        let panel = rig.sample(s).links[5];
        let p = DVec3::from_array(panel.position) + DVec3::Z * 0.104;
        let pose = WorldPose {
            translation: p.to_array(),
            rotation: [0., 0., 0., 1.],
        };
        let ball = spawn_model_template(s, &template, pose).unwrap();
        spawned.push(ball);
        let v = DVec3::from_array(panel.velocity) - 2. * DVec3::Z;
        set_model_state(
            s,
            ball,
            ModelState {
                translation: pose.translation,
                rotation: pose.rotation,
                linear_velocity: v.to_array(),
                angular_velocity: [0.; 3],
            },
        )
        .unwrap();
        for _ in 0..20 {
            s.step_exact(1).unwrap();
            assert_eq!(
                model_fidelity(s, rig.model).unwrap(),
                ArticulationTier::Dynamic
            );
            let b = s
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            impulses[i] += b
                .rigid_contact_observations()
                .iter()
                .filter(|p| p.a == rig.links[5].1 || p.b == rig.links[5].1)
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        }
    }
    eprintln!("Post-travel spawned impact impulses (fast, native): {impulses:?}");
    assert!(impulses[0] > 0.3 && (impulses[0] - impulses[1]).abs() < 1e-8);
    let (a, b) = (rig.sample(&coast), rig.sample(&native));
    assert_eq!(spawned[0], spawned[1]);
    let removed = remove_model(&mut coast, spawned[0]).unwrap();
    assert_eq!(removed, (spawned[0]..spawned[0] + 4).collect::<Vec<_>>());
    gz_sim_scene::reset_loaded_world(&mut native, |_| ()).unwrap();
    assert!(gz_sim_core::entity_by_id(native.world(), spawned[1]).is_none());
    assert!(
        gz_sim_core::entity_by_id(
            native.world(),
            coasting::stable_id(&native, "default::stock")
        )
        .is_some()
    );
    for (l, r) in a.links.iter().zip(b.links) {
        assert!(DVec3::from_array(l.velocity).distance(DVec3::from_array(r.velocity)) < 1e-8);
    }
}
