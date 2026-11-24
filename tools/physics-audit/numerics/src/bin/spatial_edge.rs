// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Reconstruct spatial momentum, energy and both Coulomb rays from raw measurements.
use glam::{DQuat, DVec3};
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn f(a: &Value) -> f64 {
    a.as_f64().unwrap()
}
fn v(a: &Value) -> DVec3 {
    DVec3::from_array(std::array::from_fn(|i| f(&a[i])))
}
fn main() -> Result<(), Box<dyn Error>> {
    let path = PathBuf::from(std::env::args_os().nth(1).ok_or("pass phase directory")?);
    let rows: Vec<Value> = serde_json::from_slice(&fs::read(path.join("edge.json"))?)?;
    let declines: Vec<Value> = serde_json::from_slice(&fs::read(path.join("declines.json"))?)?;
    let mut site = [0f64; 6];
    let mut total = [0f64; 6];
    let mut spins = [0f64; 6];
    let mut law = 0f64;
    let mut manufactured = 0f64;
    let mut angular = 0f64;
    let mut both = 0.;
    let mut min_angle = 180f64;
    let mut max_angle = 0f64;
    let mut max_iter = 0u64;
    let mut worst_seq = 0f64;
    let mut kinetic_error = 0f64;
    let mut max_l = 0f64;
    let mut max_yaw = 0f64;
    let mut data = vec![];
    for (k, r) in rows.iter().enumerate() {
        let mu = f(&r["mu"]);
        let arms = [v(&r["arms_m"][0]), v(&r["arms_m"][1])];
        let incoming = &r["before"];
        let v0 = v(&incoming["velocity_m_s"]);
        let w0 = v(&incoming["spin_rad_s"]);
        let j = [v(&r["impulses_ns"][0]), v(&r["impulses_ns"][1])];
        let v1 = v0 + (j[0] + j[1]) * 0.5;
        let w1 = w0 + (arms[0].cross(j[0]) + arms[1].cross(j[1])) / 0.05;
        assert!(
            v1.distance(v(&r["velocity_m_s"])) < 1e-13 && w1.distance(v(&r["spin_rad_s"])) < 1e-13
        );
        let mut directions = [DVec3::ZERO; 2];
        for i in 0..2 {
            let pre = v0 + w0.cross(arms[i]);
            let post = v1 + w1.cross(arms[i]);
            directions[i] = post.normalize();
            assert!(
                pre.z < 0.
                    && post.z.abs() < 1e-13
                    && post.distance(v(&r["point_velocities_m_s"][i])) < 1e-13
            );
            assert!(post.truncate().length() >= f(&r["slip_speed_floor"][i]) - 1e-13);
            assert!(f(&r["normal_speed_upper_if_other_alone"][i]) < 0.);
            let e = (j[i].truncate() + mu * j[i].z * post.truncate().normalize()).length();
            law = law.max(e);
            manufactured = manufactured.max(j[i].distance(v(&r["desired_ns"][i])));
            assert!((j[i].z - f(&r["target_loads_ns"][i])).abs() < 1e-12);
        }
        let angle = directions[0].angle_between(directions[1]).to_degrees();
        min_angle = min_angle.min(angle);
        max_angle = max_angle.max(angle);
        let spent = v0.length_squared() - v1.length_squared()
            + 0.025 * (w0.length_squared() - w1.length_squared());
        assert!(spent > 0.);
        kinetic_error = kinetic_error.max((spent - f(&r["dissipation_j"])).abs());
        max_l = max_l.max(f(&r["contraction_bound"]));
        max_iter = max_iter.max(r["fixed_point_iterations"].as_u64().unwrap());
        let rot = DQuat::from_rotation_z(f(&r["yaw_deg"]).to_radians());
        let base = rows
            .iter()
            .find(|m| m["mu"] == r["mu"] && m["case"] == r["case"] && m["yaw_deg"] == 0.)
            .unwrap();
        for i in 0..2 {
            max_yaw = max_yaw.max((rot * v(&base["impulses_ns"][i])).distance(j[i]));
        }
        for m in r["sequential"].as_array().unwrap() {
            for p in m["point_velocities_m_s"].as_array().unwrap() {
                worst_seq = worst_seq.max(v(p).z.abs());
            }
        }
        for (i, m) in r["native"].as_array().unwrap().iter().enumerate() {
            let measured = [v(&m["impulses_ns"][0]), v(&m["impulses_ns"][1])];
            let all = (measured[0] + measured[1]).distance(j[0] + j[1]);
            assert!((all - f(&m["total_error_ns"])).abs() < 1e-13 && f(&m["start_error"]) < 1e-12);
            let worst = measured[0].distance(j[0]).max(measured[1].distance(j[1]));
            site[i] = site[i].max(worst);
            total[i] = total[i].max(all);
            spins[i] = spins[i].max(f(&m["spin_error_rad_s"]));
            both += 1.;
            let h = arms[0].cross(measured[0]) + arms[1].cross(measured[1]);
            angular = angular.max(h.distance(0.05 * (v(&m["state"]["spin_rad_s"]) - w0)));
            if [1, 2, 4, 5].contains(&i) {
                data.push((k, i, worst));
            }
        }
    }
    assert_eq!((rows.len(), declines.len(), both as usize), (52, 12, 312));
    assert!(
        max_l < 0.89
            && manufactured < 1e-12
            && law < 1e-12
            && max_yaw < 1e-12
            && angular < 1e-12
            && kinetic_error < 1e-12
            && min_angle > 1.
    );
    let plot_path = path.join("edge-reactions.svg");
    let a = SVGBackend::new(&plot_path, (1080, 475)).into_drawing_area();
    a.fill(&WHITE)?;
    let mut plot = ChartBuilder::on(&a)
        .caption(
            "Largest error on either touching vertex",
            ("sans-serif", 24),
        )
        .margin(16)
        .x_label_area_size(45)
        .y_label_area_size(65)
        .build_cartesian_2d(-0.8f64..52., 0f64..0.061)?;
    plot.configure_mesh()
        .x_desc("Certified incoming state (load, slip, material and yaw)")
        .y_desc("Vector impulse error (N·s)")
        .light_line_style(RGBColor(228, 232, 235))
        .draw()?;
    for (id, label, color) in [
        (2, "Locked articulation · 32 passes", RGBColor(177, 84, 44)),
        (5, "Standalone rigid · 32 passes", RGBColor(41, 112, 159)),
    ] {
        plot.draw_series(
            data.iter()
                .filter(|r| r.1 == id)
                .map(|r| Circle::new((r.0 as f64, r.2), 3, color.filled())),
        )?
        .label(label)
        .legend(move |(x, y)| Circle::new((x + 12, y), 4, color.filled()));
    }
    plot.configure_series_labels()
        .border_style(BLACK)
        .background_style(WHITE.mix(0.95))
        .draw()?;
    a.present()?;
    println!(
        "{}",
        serde_json::to_string_pretty(
            &json!({"certified":rows.len(),"declined":declines.len(),"observations":both as usize,"max_contraction":max_l,"max_iterations":max_iter,"max_manufactured_error_ns":manufactured,"max_coulomb_residual_ns":law,"max_energy_residual_j":kinetic_error,"max_rotation_error_ns":max_yaw,"max_native_moment_residual_nms":angular,"slip_direction_separation_deg":[min_angle,max_angle],"max_sequential_remaining_normal_velocity_m_s":worst_seq,"native_order":["locked1","locked8","locked32","rigid1","rigid8","rigid32"],"max_individual_error_ns":site,"max_total_error_ns":total,"max_spin_error_rad_s":spins})
        )?
    );
    Ok(())
}
