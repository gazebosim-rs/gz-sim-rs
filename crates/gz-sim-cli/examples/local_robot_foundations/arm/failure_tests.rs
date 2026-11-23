// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn missing_named_joint_or_missing_payload_prevents_qualification() -> Result<()> {
    for name in ["shoulder", "lift", "close_left", "close_right", "parcel"] {
        let xml = SCENE.replace(&format!("name=\"{name}\""), "name=\"other\"");
        let mut r = Recording::new(&xml, "renamed-arm.sdf", "default")?;
        assert!(
            metrics(&mut r, false, false)
                .unwrap_err()
                .to_string()
                .contains(name)
        );
    }
    Ok(())
}

#[test]
fn removed_joint_or_body_and_invalid_actuator_command_propagate_errors() -> Result<()> {
    let mut r = Recording::new(SCENE, "invalid-arm.sdf", "default")?;
    let drive = Controller::new(&r.sim)?;
    assert!(
        drive
            .actuate(&mut r.sim, [(f64::NAN, 0., 0.); 4], true)
            .is_err()
    );
    assert!(handle(&r.sim, u64::MAX).is_err());
    let id = entity_id(&r.sim, "default::parcel::box")?;
    let ent = entity_by_id(r.sim.world(), id).unwrap();
    r.sim.world_mut().entity_mut(ent).remove::<BodyHandle>();
    assert!(
        metrics(&mut r, false, false)
            .unwrap_err()
            .to_string()
            .contains("missing body")
    );
    let joint = entity_by_id(r.sim.world(), drive.ids[0]).unwrap();
    r.sim
        .world_mut()
        .entity_mut(joint)
        .remove::<gz_sim_scene::JointHandle>();
    assert!(drive.command(&mut r.sim, 0., false).is_err());
    r.sim.world_mut().entity_mut(joint).remove::<JointSample>();
    assert!(
        drive
            .command(&mut r.sim, 0., false)
            .unwrap_err()
            .to_string()
            .contains("missing joint measurement")
    );
    r.sim.world_mut().despawn(joint);
    assert!(drive.sample(&r.sim, 0).is_err());
    Ok(())
}

#[test]
fn route_controls_require_an_actual_collision_and_an_actual_clear_transfer() -> Result<()> {
    let moved = SCENE.replace(".22627 .22627 .1175", "2 2 .1175");
    let mut blocked = Recording::new(&moved, "absent-obstruction.sdf", "default")?;
    assert!(
        metrics(&mut blocked, true, false)
            .unwrap_err()
            .to_string()
            .contains("missed obstacle")
    );
    let raised = SCENE.replace(".22627 .22627 .1175", ".22627 .22627 .225");
    let mut clear = Recording::new(&raised, "raised-obstruction.sdf", "default")?;
    assert!(
        metrics(&mut clear, false, false)
            .unwrap_err()
            .to_string()
            .contains("arm transfer failed")
    );
    let mut damaged = Recording::new(SCENE, "missing-visual-world.sdf", "default")?;
    damaged.world = "nonexistent".to_owned();
    assert!(metrics(&mut damaged, false, true).is_err());
    Ok(())
}
