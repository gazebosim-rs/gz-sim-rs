// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare normal material losses for the same previously unloaded foot.
use super::*;

pub(super) fn apply_pressure(
    patch: &mut Patch,
    state: State,
    height: f64,
    depth: usize,
    alpha: f64,
    material: DampedAreaFoundation,
) -> Result<()> {
    let velocity = |point: DVec3| state.velocity + state.spin.cross(point - state.center);
    let factor = |point: DVec3| alpha.mul_add(-velocity(point).z, 1.);
    // The unclamped pressure is depth times an affine rate factor. Cut its
    // zero crossing exactly. A compressed region with zero force can still
    // release stored energy into heat and must remain in the energy integral.
    let (active, unloaded) = if patch.polygon.iter().all(|&point| factor(point) >= 0.) {
        (std::mem::take(&mut patch.polygon), Vec::new())
    } else {
        (
            clip(patch.polygon.clone(), factor),
            clip(std::mem::take(&mut patch.polygon), |p| -factor(p)),
        )
    };
    patch.area = 0.;
    for (polygon, carries_pressure) in [(&active, true), (&unloaded, false)] {
        let Some(&first) = polygon.first() else {
            continue;
        };
        for pair in polygon[1..].windows(2) {
            let [a, b] = [pair[0], pair[1]];
            let area = 0.5 * (a - first).cross(b - first).length();
            if area == 0. {
                continue;
            }
            if carries_pressure {
                patch.area += area;
            }
            let mut invalid_sample = false;
            let mut integrate = |point: DVec3, w: f64, triangle_area: f64| {
                let v = velocity(point);
                let Some(sample) = material.sample(height - point.z, -v.z) else {
                    invalid_sample = true;
                    return;
                };
                let da = w * triangle_area;
                patch.heat_power += sample.dissipation_w_m2 * da;
                if carries_pressure {
                    let normal = da * sample.pressure_pa;
                    let slip = v.reject_from_normalized(DVec3::Z);
                    let friction =
                        -MU * normal * slip / (slip.length_squared() + SLIP_SPEED.powi(2)).sqrt();
                    let reaction = normal * DVec3::Z + friction;
                    patch.first_moment += point * normal;
                    patch.load.force += reaction;
                    patch.load.torque += (point - state.center).cross(reaction);
                    patch.load.friction_power += friction.dot(slip);
                }
            };
            // In the unloaded polygon, heat is only quadratic and requires no refinement.
            for_each_site(
                [first, a, b],
                area,
                if carries_pressure { depth } else { 0 },
                &mut integrate,
            );
            if invalid_sample {
                return Err("unrepresentable damped pressure during integration".into());
            }
        }
    }
    if patch.load.force.z > 0. {
        patch.polygon = active;
    } else {
        patch.area = 0.;
    }
    if !patch.heat_power.is_finite() || patch.heat_power < 0. || !patch.load.force.is_finite() {
        return Err("invalid damped material integral".into());
    }
    Ok(())
}

fn damping_delta(other: &Trajectory, fine: &Trajectory) -> Result<Value> {
    let mut d = delta(other, fine);
    for (source, metric) in [
        ("normal_dissipation_j", "normal_heat_delta_j"),
        ("rebound_speed_m_s", "rebound_speed_delta_m_s"),
        ("lift_off_time_s", "release_time_delta_s"),
        ("peak_normal_n", "peak_force_delta_n"),
    ] {
        d[metric] = json!((num(&other.checks[source])? - num(&fine.checks[source])?).abs());
    }
    Ok(d)
}

pub(in super::super::super) fn study(
    donor_file: &Path,
    previous_file: &Path,
    out: &Path,
    light: f64,
    stronger: f64,
) -> Result<()> {
    if !light.is_finite() || !stronger.is_finite() || light <= 0. || stronger <= light {
        return Err(
            "damped profiles require two finite positive coefficients in increasing order".into(),
        );
    }
    let donor = read(donor_file)?;
    let previous = read(previous_file)?;
    if [&donor, &previous]
        .iter()
        .any(|d| d["case"] != 80 || d["stroke"] != 2)
    {
        return Err("expected fixture 80, stroke 2".into());
    }
    let old = array(&previous["profiles"])?
        .iter()
        .find(|p| p["name"] == "Free approach")
        .ok_or("missing undamped control")?;
    let original = &donor["initial"];
    let initial = parse_state(original)?;
    let baseline = array(&old["frames"])?;
    let times = baseline
        .iter()
        .map(|f| num(&f["original_time_s"]))
        .collect::<Result<Vec<_>>>()?;
    let arrival = num(&donor["first_arrival"]["time_s"])?;
    let t0 = num(&original["elapsed"])?;
    if times.first() != Some(&t0)
        || times.last() != Some(&num(&donor["horizon_s"])?)
        || arrival <= t0
        || times.windows(2).any(|t| t[0] >= t[1])
        || !times.contains(&arrival)
    {
        return Err("mismatched free cut, first touch or public clocks".into());
    }
    let incoming = -(initial.velocity.z + GRAVITY.z * (arrival - t0));
    let springs = ElasticAreaFoundation::new(LAYER_K, LAYER_K).ok_or("invalid sole")?;
    let mut profiles = vec![];
    for (name, alpha) in [
        ("Undamped", 0.),
        ("Light damping", light),
        ("Stronger damping", stronger),
    ] {
        let sole = Sole {
            half: vector(&donor["foot_half_extents_m"])?,
            deck: vector(&donor["deck"]["center_m"])?,
            deck_half: vector(&donor["deck"]["half_extents_m"])?,
            foot_local: initial.orientation.conjugate()
                * (vector(&original["foot"])? - initial.center),
            material: springs,
            gravity: GRAVITY,
            damping: if alpha > 0. {
                Some((alpha, springs.with_damping(alpha).ok_or("invalid damping")?))
            } else {
                None
            },
        };
        let run = |depth, rate, display| {
            integrate_history(initial, &sole, &times, depth, rate, display, true)
        };
        let mut fine = run(3, 1024, true)?;
        let space = damping_delta(&run(2, 1024, false)?, &fine)?;
        let time = damping_delta(&run(3, 512, false)?, &fine)?;
        if alpha == 0. {
            for (a, b) in fine.frames.iter().zip(baseline) {
                for (k, value) in b.as_object().ok_or("invalid baseline")? {
                    if &a[k] != value {
                        return Err(format!(
                            "undamped baseline changed: {k} at {}",
                            a["original_time_s"]
                        )
                        .into());
                    }
                }
            }
        }
        let speed = num(&fine.checks["rebound_speed_m_s"])?;
        let flight = &fine.checks["final_ballistic_check"];
        let events = array(&fine.checks["contact_transitions"])?;
        if events.len() != 2
            || events[0]["loaded"] != true
            || events[1]["loaded"] != false
            || flight["separating_plane"].is_null()
            || num(&flight["ballistic_endpoint_defect_m"])? > 1e-9
        {
            return Err(format!("unproved release for {name}: {flight}").into());
        }
        for (d, position, impulse, force) in [(&space, 2e-9, 2e-6, 1e-4), (&time, 5e-9, 1e-5, 0.02)]
        {
            if num(&d["max_com_delta_m"])? > position
                || num(&d["max_impulse_delta_ns"])? > impulse
                || num(&d["max_sampled_normal_force_delta_n"])? > force
                || num(&d["normal_heat_delta_j"])? > 5e-6
                || num(&d["rebound_speed_delta_m_s"])? > 2e-5
                || num(&d["release_time_delta_s"])? > 1.1e-6
                || num(&d["peak_force_delta_n"])? > 0.03
            {
                return Err(format!("mesh disagreement for {name}: {d}").into());
            }
        }
        if num(&fine.checks["max_energy_defect_j"])? > 5e-7
            || speed <= 0.
            || (alpha > 0. && num(&fine.checks["normal_dissipation_j"])? <= 0.)
        {
            return Err(
                format!("energy or rebound check failed for {name}: {}", fine.checks).into(),
            );
        }
        fine.checks["incoming_speed_m_s"] = json!(incoming);
        fine.checks["restitution"] = json!(speed / incoming);
        profiles.push(json!({"name":name,"damping_s_m":alpha,"start_s":t0,"rigid_impulse_before_start_ns":original["j"],
            "checks":fine.checks,"frames":fine.frames,"space_refinement":space,"time_refinement":time}));
    }
    let comparisons = compare_histories(&profiles)?;
    fs::create_dir_all(out)?;
    write(
        &out.join("reference.json"),
        &json!({"case":80,"stroke":2,"start_s":t0,"public_restart_s":donor["public_restart_s"],
        "horizon_s":donor["horizon_s"],"first_touch_s":arrival,"material":{"foot_stiffness_n_m3":LAYER_K,"deck_stiffness_n_m3":LAYER_K,"mu":MU,
        "slip_regularization_m_s":SLIP_SPEED,"normal_response":"elastic pressure times max(0, 1 + damping * local compression rate); dissipation includes the force cap",
        "assumption":"prescribed sensitivity coefficients for the full sole on the deck top"},"profiles":profiles,"comparisons":comparisons,
        "coordinate_frame":previous["coordinate_frame"]}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_damped_drop_reproduces_closed_form_restitution_and_lost_energy() -> Result<()> {
        let alpha = 0.8;
        let material = ElasticAreaFoundation::new(LAYER_K, LAYER_K).unwrap();
        let sole = Sole {
            half: DVec3::new(0.18, 0.1, 0.06),
            deck: DVec3::new(0., 0., -0.06),
            deck_half: DVec3::new(1., 1., 0.06),
            foot_local: DVec3::NEG_Z * 0.07,
            material,
            damping: Some((alpha, material.with_damping(alpha).unwrap())),
            gravity: DVec3::ZERO,
        };
        let speed = 1.3;
        let initial = State {
            center: DVec3::Z * 0.13,
            orientation: DQuat::IDENTITY,
            velocity: -DVec3::Z * speed,
            spin: DVec3::ZERO,
        };
        let path = integrate_history(initial, &sole, &[0., 0.004], 0, 1024, true, true)?;
        let rebound = num(&path.checks["rebound_speed_m_s"])?;
        // On a complete compression/release cycle integrate v/(1+αv) dv:
        // F(-α v_out) = F(α v_in), F(s) = s - ln(1+s).
        let f = |s: f64| s - s.ln_1p();
        assert!((f(-alpha * rebound) - f(alpha * speed)).abs() < 1e-6);
        let recovered = DampedAreaFoundation::coefficient_from_flat_impact(speed, rebound)
            .ok_or("no flat-drop damping coefficient")?;
        assert!((recovered - alpha).abs() < 2e-6);
        let heat = num(&path.checks["normal_dissipation_j"])?;
        assert!((heat - 0.5 * MASS * (speed * speed - rebound * rebound)).abs() < 2e-6);
        assert!((0.0..speed).contains(&rebound));
        Ok(())
    }

    #[test]
    fn clamped_part_can_store_and_dissipate_energy_without_supporting_load() -> Result<()> {
        let material = ElasticAreaFoundation::new(20., 20.).unwrap();
        let sole = Sole {
            half: DVec3::new(0.18, 0.1, 0.06),
            deck: DVec3::new(0., 0., -0.06),
            deck_half: DVec3::new(1., 1., 0.06),
            foot_local: DVec3::NEG_Z * 0.07,
            material,
            damping: Some((1., material.with_damping(1.).unwrap())),
            gravity: DVec3::ZERO,
        };
        let mut state = State {
            center: DVec3::Z * 0.129,
            orientation: DQuat::IDENTITY,
            velocity: DVec3::Z * 2.,
            spin: DVec3::ZERO,
        };
        let p = sole.patch(state, 1)?;
        let area = 4. * sole.half.x * sole.half.y;
        assert!(p.polygon.is_empty() && p.load.force == DVec3::ZERO && p.load.elastic_energy > 0.);
        assert!((p.heat_power - 0.001 * 10. * 2. * area).abs() < 1e-12);
        state.velocity.z = 1.;
        state.spin.y = 1.;
        let half = sole.patch(state, 1)?;
        assert!((half.area - area * 0.5).abs() < 1e-12);
        assert!(half.first_moment.x > 0. && half.heat_power > 0.);
        Ok(())
    }
}
