// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn measured_pad_work_pairs_translated_moments_and_world_rotation_for_both_quaternion_signs() {
    let start = BodyState {
        position: [0.1, 0.2, 0.3],
        orientation: DQuat::from_rotation_y(0.4).to_array(),
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let dx = DVec3::new(0.1, 0., 0.2);
    let impulse = DVec3::new(0., 0.03, -0.4);
    let moment_mid = DVec3::new(-0.01, 0.005, -0.03);
    let moment_start = moment_mid + 0.5 * dx.cross(impulse);
    let q = DQuat::from_rotation_z(0.1) * DQuat::from_array(start.orientation);
    for orientation in [q, -q] {
        let end = BodyState {
            position: (dx + DVec3::from(start.position)).to_array(),
            orientation: orientation.to_array(),
            ..start
        };
        let expected = -(-0.4 * 0.2 - 0.03 * 0.1) / 0.01;
        assert!((pad_work(start, end, impulse, moment_start, 0.01) - expected).abs() < 1e-12);
    }
}

#[test]
fn free_material_loses_the_semi_implicit_gravity_energy_and_balances_angular_impulse() -> Result<()>
{
    let plugin = r#"<plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">
    <box link="default::arm::left" coupling="articulated_wrench" normal_compliance="0.001"/>
    <box link="default::arm::right" coupling="articulated_wrench" normal_compliance="0.001"/>
    <plane point="0 0 0.12" normal="0 0 1"/>
    <cable name="probe" substeps="1" iterations="4" radius=".005" self_contact="false" friction=".7" stretch=".0001" bend=".001">
    <particle position="0.1 0.1 0.5" mass=".01"/><particle position=".12 .1 .5" mass=".01"/></cable></plugin>"#;
    let xml = arm::SCENE.replace("</world>", &format!("{plugin}</world>"));
    let mut r = Recording::new(&xml, "audit.sdf", "default")?;
    let jaws = [
        entity_id(&r.sim, "default::arm::left")?,
        entity_id(&r.sim, "default::arm::right")?,
    ];
    assert!(EnergyAudit::new(&r.sim, "absent", jaws, 0.12).is_err());
    assert!(EnergyAudit::new(&r.sim, "probe", [0, jaws[1]], 0.12).is_err());
    let mut audit = EnergyAudit::new(&r.sim, "probe", jaws, 0.12)?;
    for _ in 0..8 {
        r.advance()?;
        let sample = bodies(&r.sim).next().unwrap().1;
        audit.observe(&r.sim, sample)?;
        assert!(audit.observe(&r.sim, sample).is_err());
        assert!(EnergyAudit::default().observe(&r.sim, sample).is_err());
    }
    let expected = -8. * 0.5 * 0.02 * (9.81_f64 * 0.001).powi(2);
    assert!((audit.final_material_change_j - expected).abs() < 1e-12);
    assert_eq!(
        (audit.positive_pad_work_j, audit.negative_pad_work_j),
        (0., 0.)
    );
    assert!(audit.max_angular_impulse_residual_nms < 1e-12);
    Ok(())
}
