// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent native-clock simulation of every accepted 1 ms gripper sample.
use super::*;
use gz_sim_scene::deformable::{articulated_pad_wrench, shell_constraint_impulses};
use serde::Serialize;

#[derive(Default, Serialize)]
struct Audit {
    public_ticks: u32,
    fine_ticks: u32,
    max_particle_position_error_m: f64,
    max_particle_velocity_error_m_s: f64,
    max_pad_linear_impulse_error_ns: f64,
    max_pad_angular_impulse_error_nms: f64,
    max_end_com_torque_error_nm: f64,
    max_structural_node_impulse_error_ns: f64,
    max_torque_origin_shift_nm: f64,
}
pub(super) fn check(xml: &str, fine_xml: &str, n: usize, damping: f64, out: &Path) -> Result<()> {
    let mut outer = Recording::new(xml, "exchange.sdf", "default")?;
    let mut fine = Recording::new(fine_xml, "fine.sdf", "default")?;
    let outer_controller = arm::Controller::with_jaw_damping(&outer.sim, damping)?;
    let fine_controller = arm::Controller::with_jaw_damping(&fine.sim, damping)?;
    let jaws = [
        entity_id(&fine.sim, "default::arm::left")?,
        entity_id(&fine.sim, "default::arm::right")?,
    ];
    let h = fine.sim.step_duration().as_secs_f64();
    let mut audit = Audit::default();
    for tick in 0..3400 {
        let t = tick as f64 * 0.001;
        arm_cable::lift_with_jaws(&outer_controller, &mut outer.sim, t, arm_sheet::JAW_STROKE)?;
        arm_cable::lift_with_jaws(&fine_controller, &mut fine.sim, t, arm_sheet::JAW_STROKE)?;
        let mut sum = [(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO); 2];
        let mut nodes = [[DVec3::ZERO; 3]; arm_sheet::COUNT];
        for _ in 0..n {
            fine.advance()?;
            audit.fine_ticks += 1;
            for (i, &id) in jaws.iter().enumerate() {
                let pos = DVec3::from(state(&fine.sim, id)?.position);
                let w = articulated_pad_wrench(&fine.sim, id).ok_or("missing reference wrench")?;
                let j = DVec3::from(w.force_n) * h;
                let k = DVec3::from(w.torque_nm) * h;
                sum[i].0 += j;
                // Integrate the reference about the fixed world origin.
                sum[i].1 += k + pos.cross(j);
                sum[i].2 += k;
            }
            for (node, j) in nodes
                .iter_mut()
                .zip(shell_constraint_impulses(&fine.sim, "sheet").ok_or("missing fine material")?)
            {
                node[0] += j.distance;
                node[1] += j.bending;
                node[2] += j.bend_damping;
            }
        }
        outer.advance()?;
        audit.public_ticks += 1;
        let coarse = arm_sheet::cloth(&outer.sim)?;
        let reference = arm_sheet::cloth(&fine.sim)?;
        if coarse.rigid_material_exchanges() != n as u32
            || coarse.sim_time() != reference.sim_time()
        {
            return Err("public and fine exchange clocks disagree".into());
        }
        for (a, b) in coarse.particles().iter().zip(reference.particles()) {
            audit.max_particle_position_error_m = audit
                .max_particle_position_error_m
                .max(DVec3::from(a.position).distance(b.position.into()));
            audit.max_particle_velocity_error_m_s = audit
                .max_particle_velocity_error_m_s
                .max(DVec3::from(a.velocity).distance(b.velocity.into()));
        }
        for (i, &id) in jaws.iter().enumerate() {
            let pos = DVec3::from(state(&outer.sim, id)?.position);
            let w = articulated_pad_wrench(&outer.sim, id).ok_or("missing public wrench")?;
            let (j, k, uncorrected) = sum[i];
            let at_end = k - pos.cross(j);
            let error = (DVec3::from(w.torque_nm) * 0.001).distance(at_end);
            audit.max_pad_linear_impulse_error_ns = audit
                .max_pad_linear_impulse_error_ns
                .max((DVec3::from(w.force_n) * 0.001).distance(j));
            audit.max_pad_angular_impulse_error_nms =
                audit.max_pad_angular_impulse_error_nms.max(error);
            audit.max_end_com_torque_error_nm =
                audit.max_end_com_torque_error_nm.max(error / 0.001);
            audit.max_torque_origin_shift_nm = audit
                .max_torque_origin_shift_nm
                .max(uncorrected.distance(at_end) / 0.001);
        }
        for (expected, j) in nodes
            .iter()
            .zip(shell_constraint_impulses(&outer.sim, "sheet").ok_or("missing public material")?)
        {
            for (a, b) in expected.iter().zip([j.distance, j.bending, j.bend_damping]) {
                audit.max_structural_node_impulse_error_ns = audit
                    .max_structural_node_impulse_error_ns
                    .max(a.distance(b));
            }
        }
    }
    if audit.max_particle_position_error_m > 1e-9
        || audit.max_particle_velocity_error_m_s > 1e-7
        || audit.max_pad_linear_impulse_error_ns > 1e-9
        || audit.max_end_com_torque_error_nm > 1e-8
        || audit.max_structural_node_impulse_error_ns > 1e-9
    {
        return Err(format!(
            "public clock diverged from independent native reference: {}",
            serde_json::to_string(&audit)?
        )
        .into());
    }
    write_json(
        &out.join("exchange-audit.json"),
        &serde_json::to_value(audit)?,
    )
}
