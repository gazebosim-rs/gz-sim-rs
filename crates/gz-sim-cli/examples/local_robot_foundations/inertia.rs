// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::{DMat3, DQuat};
use gz_sim_scene::queue_link_force_at_point;

const WELD: &str = include_str!("../../../../examples/local_robot_inertia.sdf");
const BASE_MASS: f64 = 1.5;
const PAYLOAD_MASS: f64 = 3.5;
const BASE_COM: DVec3 = DVec3::new(0.03, -0.02, 0.015);
const CHILD_COM: DVec3 = DVec3::new(-0.02, 0.04, 0.08);
const CHILD_ORIGIN: DVec3 = DVec3::new(0.32, 0.12, 0.06);
const LOAD_POINT: DVec3 = DVec3::new(0.10, -0.11, 0.10);
const FORCE: DVec3 = DVec3::new(70., -25., 18.);

fn inertia(d: [f64; 3], off: [f64; 3]) -> DMat3 {
    let [xx, yy, zz] = d;
    let [xy, xz, yz] = off;
    DMat3::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz])
}
fn parallel_axis(mass: f64, d: DVec3) -> DMat3 {
    mass * (d.length_squared() * DMat3::IDENTITY - DMat3::from_cols(d.x * d, d.y * d, d.z * d))
}
struct Equivalent {
    center: DVec3,
    tensor: DMat3,
    load_point: DVec3,
}
impl Equivalent {
    fn from_fixture() -> Self {
        // Independently assemble the author-declared tensors in the base frame.
        let turn = std::f64::consts::FRAC_PI_2;
        let child = DMat3::from_rotation_z(turn);
        let r1 = DMat3::from_rotation_x(turn);
        let r2 = child * DMat3::from_rotation_y(turn);
        let c2 = CHILD_ORIGIN + child * CHILD_COM;
        let center = (BASE_MASS * BASE_COM + PAYLOAD_MASS * c2) / (BASE_MASS + PAYLOAD_MASS);
        let tensor = r1 * inertia([0.026, 0.031, 0.041], [0.002, -0.001, 0.003]) * r1.transpose()
            + r2 * inertia([0.072, 0.091, 0.110], [-0.006, 0.003, 0.002]) * r2.transpose()
            + parallel_axis(BASE_MASS, BASE_COM - center)
            + parallel_axis(PAYLOAD_MASS, c2 - center);
        Self {
            center,
            tensor,
            load_point: CHILD_ORIGIN + child * LOAD_POINT,
        }
    }
    fn single_sdf(&self) -> String {
        let [x, y, z] = self.center.to_array();
        let [xx, xy, xz, _, yy, yz, _, _, zz] = self.tensor.to_cols_array();
        format!(
            r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity><physics name="local" type="ignored"><max_step_size>0.0005</max_step_size></physics>
<model name="assembly"><pose>0 0 0.7 0.35 -0.4 0.55</pose><link name="base"><inertial><pose>{x} {y} {z} 0 0 0</pose><mass>5</mass>
<inertia><ixx>{xx}</ixx><iyy>{yy}</iyy><izz>{zz}</izz><ixy>{xy}</ixy><ixz>{xz}</ixz><iyz>{yz}</iyz></inertia></inertial>
<collision name="base"><geometry><box><size>0.48 0.19 0.11</size></box></geometry></collision>
<visual name="housing"><geometry><box><size>0.48 0.19 0.11</size></box></geometry><material><diffuse>0.12 0.70 0.72 1</diffuse></material></visual>
<visual name="tip"><pose>0.32 0.12 0.06 0 0 1.5707963267948966</pose><geometry><box><size>0.30 0.24 0.28</size></box></geometry><material><diffuse>0.97 0.57 0.15 1</diffuse></material></visual>
</link></model></world></sdf>"#
        )
    }
}
pub(super) fn run(output: &Path) -> Result<()> {
    fs::create_dir_all(output)?;
    let eq = Equivalent::from_fixture();
    let single_xml = eq.single_sdf();
    let mut single = Recording::new(&single_xml, "single.sdf", "default")?;
    let mut weld = Recording::new(WELD, "local_robot_inertia.sdf", "default")?;
    let base = entity_id(&weld.sim, "default::assembly::base")?;
    let child = entity_id(&weld.sim, "default::assembly::payload")?;
    let whole = entity_id(&single.sim, "default::assembly::base")?;
    let initial = state(&single.sim, whole)?;
    let rotation = DMat3::from_quat(DQuat::from_array(initial.orientation));
    let dt = single.sim.step_duration().as_secs_f64();
    let expected_v = FORCE * dt / (BASE_MASS + PAYLOAD_MASS);
    let tensor = rotation * eq.tensor * rotation.transpose();
    let expected_w = tensor.inverse() * (rotation * (eq.load_point - eq.center)).cross(FORCE) * dt;
    let mut first = Value::Null;
    let mut max_com: f64 = 0.;
    let mut max_speed: f64 = 0.;
    let mut max_spin: f64 = 0.;
    weld.capture()?;
    single.capture()?;
    for tick in 1..=500 {
        if tick <= 120 {
            queue_link_force_at_point(
                &mut weld.sim,
                child,
                LOAD_POINT.to_array(),
                FORCE.to_array(),
            )?;
            queue_link_force_at_point(
                &mut single.sim,
                whole,
                eq.load_point.to_array(),
                FORCE.to_array(),
            )?;
        }
        weld.advance()?;
        single.advance()?;
        let a = state(&weld.sim, base)?;
        let b = state(&weld.sim, child)?;
        let s = state(&single.sim, whole)?;
        let get_center =
            |s: BodyState, c| DVec3::from(s.position) + DQuat::from_array(s.orientation) * c;
        let center = (BASE_MASS * get_center(a, BASE_COM)
            + PAYLOAD_MASS * get_center(b, CHILD_COM))
            / (BASE_MASS + PAYLOAD_MASS);
        let v = (BASE_MASS * DVec3::from(a.velocity) + PAYLOAD_MASS * DVec3::from(b.velocity))
            / (BASE_MASS + PAYLOAD_MASS);
        let spin = DVec3::from(a.angular_velocity);
        max_com = max_com.max(center.distance(get_center(s, eq.center)));
        max_speed = max_speed.max(v.distance(s.velocity.into()));
        max_spin = max_spin.max(spin.distance(s.angular_velocity.into()));
        if tick == 1 {
            first = json!({"predicted_com_velocity_m_s":expected_v.to_array(), "weld_com_velocity_m_s":v.to_array(),
                "predicted_spin_rad_s":expected_w.to_array(), "weld_spin_rad_s":spin.to_array(),
                "linear_error_m_s": v.distance(expected_v), "angular_error_rad_s":spin.distance(expected_w)});
            if v.distance(expected_v) > 1e-10
                || spin.distance(expected_w) > expected_w.length() * 1e-5
            {
                return Err(format!("first-load mismatch: {first}").into());
            }
        }
        if tick == 1 || tick % 10 == 0 {
            weld.capture()?;
            single.capture()?;
        }
    }
    if max_com > 2e-7 || max_spin > 1e-5 || max_speed > 1e-6 {
        return Err(format!(
            "weld and single diverged: COM {max_com}, spin {max_spin}, speed {max_speed}"
        )
        .into());
    }
    // The angular-velocity prediction freezes the starting inertia orientation.
    // At half the timestep the body turns less during the force application.
    let finer_xml = WELD.replace("<max_step_size>0.0005", "<max_step_size>0.00025");
    let mut fine = Recording::new(&finer_xml, "inertia-half-step.sdf", "default")?;
    let fine_child = entity_id(&fine.sim, "default::assembly::payload")?;
    queue_link_force_at_point(&mut fine.sim, fine_child, LOAD_POINT.into(), FORCE.into())?;
    fine.advance()?;
    let finer_spin = DVec3::from(state(&fine.sim, fine_child)?.angular_velocity);
    let finer_error = finer_spin.distance(0.5 * expected_w);
    let coarse_error = first["angular_error_rad_s"]
        .as_f64()
        .ok_or("missing first-step spin")?;
    if finer_error > 0.25 * coarse_error {
        return Err(format!("half-step spin discrepancy did not shrink: {finer_error}").into());
    }
    first["half_step_angular_error_rad_s"] = json!(finer_error);
    fs::write(output.join("equivalent-single.sdf"), single_xml)?;
    write_json(
        &output.join("accuracy.json"),
        &json!({"first_load":first,"combined_com_m":eq.center.to_array(),"tensor_columns_kg_m2":eq.tensor.to_cols_array_2d(),"max_com_error_m":max_com,"max_speed_error_m_s":max_speed,"max_spin_error_rad_s":max_spin}),
    )?;
    single.save(&output.join("single"))?;
    weld.save(output)
}
