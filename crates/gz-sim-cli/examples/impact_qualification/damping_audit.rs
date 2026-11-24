// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Readback of the damped landing. Integrate in material coordinates from the
//! saved body pose and twist, independently of the solver's polygon triangles.
use super::*;

const MASS: f64 = 2.;
const INERTIA: f64 = 0.05;
const G: f64 = 9.81;

#[derive(Clone, Copy)]
struct Motion {
    foot: DVec3,
    q: DQuat,
    com: DVec3,
    v: DVec3,
    w: DVec3,
}

impl Motion {
    fn from_frame(f: &Value) -> Result<Self> {
        let q = DQuat::from_array(serde_json::from_value(f["orientation_xyzw"].clone())?);
        if !q.is_normalized() {
            return Err("invalid sole orientation".into());
        }
        Ok(Self {
            foot: vector(&f["foot_m"])?,
            q,
            com: vector(&f["com_m"])?,
            v: vector(&f["v_m_s"])?,
            w: vector(&f["spin_rad_s"])?,
        })
    }

    fn rate(self, site: DVec3) -> f64 {
        -(self.v + self.w.cross(site - self.com)).z
    }
    fn mechanical_energy(self) -> f64 {
        0.5 * (MASS * self.v.length_squared() + INERTIA * self.w.length_squared())
            + MASS * G * self.com.z
    }
}

fn dynamic(sole: &Sole, state: Motion, alpha: f64) -> (Reaction, f64, f64) {
    let mut net = Reaction::default();
    let mut dissipation = 0.;
    let mut cut_area = 0.;
    // Start with zero traction over the entire compressed sole, including any
    // unloading region which the tensile cutoff has switched off. Its stored
    // elastic energy still decreases as the sole moves out of compression.
    sole.visit(state.foot, state.q, None, |site, d, da| {
        let fel = sole.stiffness * d * da;
        net.energy += 0.5 * fel * d;
        dissipation -= fel * state.rate(site);
    });
    let base = state.foot - state.q * DVec3::Z * sole.half.z;
    let factor = Limit {
        a: -alpha * state.w.cross(state.q * DVec3::X).z,
        b: -alpha * state.w.cross(state.q * DVec3::Y).z,
        c: 1. + alpha * state.rate(base),
    };
    sole.visit(state.foot, state.q, Some(factor), |site, d, da| {
        let rate = state.rate(site);
        let multiplier = 1. + alpha * rate;
        if multiplier > 0. {
            let force = sole.stiffness * d * da * multiplier;
            net.area += da;
            net.force += force;
            net.first_moment += site * force;
            dissipation += force * rate;
        }
    });
    let cutoff = Limit {
        a: -factor.a,
        b: -factor.b,
        c: -factor.c,
    };
    sole.visit(state.foot, state.q, Some(cutoff), |site, _, da| {
        if 1. + alpha * state.rate(site) < 0. {
            cut_area += da;
        }
    });
    (net, dissipation, cut_area)
}

fn release(sole: &Sole, p: &Value, donor: &Value, last: &Value) -> Result<Value> {
    let checks = &p["checks"];
    let transitions = array(&checks["contact_transitions"])?;
    if transitions.len() != 2
        || transitions[0]["loaded"] != true
        || transitions[1]["loaded"] != false
    {
        return Err("these free-drop profiles require one loading and one release".into());
    }
    let touch = num(&donor["first_arrival"]["time_s"])?;
    let first: [f64; 2] = serde_json::from_value(transitions[0]["time_bracket_s"].clone())?;
    let end: [f64; 2] = serde_json::from_value(transitions[1]["time_bracket_s"].clone())?;
    if first[0] > touch + 1e-12 || first[1] < touch - 1e-12 || !(touch < end[0] && end[0] <= end[1])
    {
        return Err("inconsistent free approach / release times".into());
    }
    let donor_t = num(&donor["initial"]["elapsed"])?;
    let incoming = G * (touch - donor_t) - vector(&donor["initial"]["velocity"])?.z;
    let t_final = num(&last["original_time_s"])?;
    let out = vector(&last["v_m_s"])?.z + G * (t_final - end[1]);
    if (num(&checks["lift_off_time_s"])? - end[1]).abs() > 1e-12
        || (num(&checks["rebound_speed_m_s"])? - out).abs() > 1e-9
        || (num(&checks["incoming_speed_m_s"])? - incoming).abs() > 1e-9
        || (num(&checks["restitution"])? - out / incoming).abs() > 1e-9
    {
        return Err("rebound metric differs from independent free-flight velocity".into());
    }
    // Verify the reported fixed separating plane using the terminal state
    // propagated backwards under gravity. Negative projected acceleration
    // puts each vertex above its endpoint chord throughout the free interval.
    let plane = &checks["final_ballistic_check"]["separating_plane"];
    let n = vector(&plane["normal"])?;
    let final_state = Motion::from_frame(last)?;
    let dt = t_final - end[1];
    let arm = final_state.q.conjugate() * (final_state.foot - final_state.com);
    let q0 = DQuat::from_scaled_axis(-dt * final_state.w) * final_state.q;
    let c0 = final_state.com - final_state.v * dt - 0.5 * G * dt * dt * DVec3::Z;
    let plane_offset = sole.deck.dot(n) + sole.deck_half.dot(n.abs());
    let gap = |c: DVec3, q: DQuat| {
        (c + q * arm).dot(n) - sole.half.dot((q.conjugate() * n).abs()) - plane_offset
    };
    let clearance = gap(c0, q0).min(gap(final_state.com, final_state.q));
    let acceleration =
        -G * n.z + final_state.w.length_squared() * (arm.length() + sole.half.length());
    if !(n.is_normalized()
        && (plane_offset - num(&plane["plane_offset_m"])?).abs() < 1e-12
        && dt > 0.
        && clearance > 0.
        && acceleration < 0.)
    {
        return Err("unproved separation between release and horizon".into());
    }
    let t0 = num(&donor["public_restart_s"])?;
    Ok(
        json!({"first_contact_s":touch,"release_bracket_s":end,"release_after_public_restart_s":end[1]-t0,
        "time_in_contact_s":[end[0]-touch,end[1]-touch],"incoming_com_speed_m_s":incoming,
        "outgoing_com_speed_m_s":out,"restitution":out/incoming,"separating_plane_clearance_lower_bound_m":clearance}),
    )
}

fn check_profile(sole: &Sole, p: &Value, donor: &Value, control: &[Value]) -> Result<Value> {
    let frames = array(&p["frames"])?;
    let first = frames.first().ok_or("missing drop")?;
    let start = Motion::from_frame(first)?;
    let t0 = num(&first["original_time_s"])?;
    let alpha = num(&p["damping_s_m"])?;
    if !(alpha >= 0.
        && t0 == num(&donor["initial"]["elapsed"])?
        && first["normal_force_n"].as_f64() == Some(0.))
        || start.com.distance(vector(&donor["initial"]["com"])?) > 1e-13
        || start.v.distance(vector(&donor["initial"]["velocity"])?) > 1e-13
        || start.w.distance(vector(&donor["initial"]["omega"])?) > 1e-13
        || start.foot.distance(vector(&donor["initial"]["foot"])?) > 1e-13
        || !start.q.abs_diff_eq(
            DQuat::from_array(serde_json::from_value(
                donor["initial"]["orientation"].clone(),
            )?),
            1e-13,
        )
    {
        return Err("not the same free flight".into());
    }
    let (mut max_force, mut max_moment, mut max_power, mut max_defect, mut max_j, mut max_cut) =
        (0_f64, 0_f64, 0_f64, 0_f64, 0_f64, 0_f64);
    let (mut max_base_pose, mut max_base_impulse) = (0_f64, 0_f64);
    let (mut prev_t, mut prev_heat, mut prev_friction) = (f64::NEG_INFINITY, 0., 0.);
    for f in frames {
        let t = num(&f["original_time_s"])?;
        if t <= prev_t {
            return Err("non-increasing clock".into());
        }
        let s = Motion::from_frame(f)?;
        let (r, dissipated, cut) = dynamic(sole, s, alpha);
        let force = num(&f["normal_force_n"])?;
        let moment = if force > 0. {
            force * vector(&f["center_of_pressure_m"])?
        } else {
            if !f["center_of_pressure_m"].is_null() {
                return Err("pressure center without load".into());
            }
            DVec3::ZERO
        };
        let moment_err = moment.distance(r.first_moment);
        let area = num(&f["loaded_area_m2"])?;
        let heat = num(&f["normal_dissipation_j"])?;
        let friction = num(&f["sliding_work_j"])?;
        let flux = num(&f["boundary_energy_flux_j"])?;
        let impulse = vector(&f["support_frame_j_ns"])?;
        let defect =
            s.mechanical_energy() - start.mechanical_energy() + r.energy + heat - friction - flux;
        let j = impulse.distance(MASS * (s.v - start.v + (t - t0) * G * DVec3::Z));
        let power_err = (dissipated - num(&f["normal_dissipation_w"])?).abs();
        if !((force - r.force).abs() < 2e-6
            && moment_err < 1e-6
            && (r.energy - num(&f["strain_energy_j"])?).abs() < 1e-8
            && (force.max(r.force) < 1e-6 || (area - r.area).abs() < 1e-8)
            && power_err < 1e-5
            && dissipated > -1e-6
            && heat >= prev_heat - 1e-12
            && friction <= prev_friction + 1e-12
            && defect.abs() < 3e-6
            && j < 1e-9)
        {
            return Err(format!("{} at {t} s: force error {} N, moment {moment_err} Nm, heat rate {dissipated} W (error {power_err}), stored {}, energy defect {defect} J, area error {}, impulse residual {j} Ns",p["name"],r.force-force,r.energy,area-r.area).into());
        }
        max_force = max_force.max((force - r.force).abs());
        max_moment = max_moment.max(moment_err);
        max_power = max_power.max(power_err);
        max_defect = max_defect.max(defect.abs());
        max_j = max_j.max(j);
        max_cut = max_cut.max(cut);
        prev_t = t;
        prev_heat = heat;
        prev_friction = friction;
    }
    for b in control {
        let clock = num(&b["original_time_s"])?;
        // The archive also contains optional earlier phase event clocks;
        // public boundaries are mandatory in every new run.
        let aligned = (clock / 0.0005 - (clock / 0.0005).round()).abs() < 1e-10;
        let a = frames.iter().find(|f| {
            f["original_time_s"]
                .as_f64()
                .is_some_and(|t| (t - clock).abs() < 1e-14)
        });
        if aligned && a.is_none() {
            return Err("missing public clock".into());
        }
        if alpha == 0.
            && let Some(a) = a
        {
            max_base_pose = max_base_pose.max(vector(&a["com_m"])?.distance(vector(&b["com_m"])?));
            max_base_impulse = max_base_impulse
                .max(vector(&a["support_frame_j_ns"])?.distance(vector(&b["support_frame_j_ns"])?));
        }
    }
    if max_base_pose > 1e-8 || max_base_impulse > 1e-5 {
        return Err("undamped control diverged from archive".into());
    }
    let last = frames.last().ok_or("no end")?;
    if (prev_heat - num(&p["checks"]["normal_dissipation_j"])?).abs() > 1e-10
        || prev_t != num(&donor["horizon_s"])?
    {
        return Err("incomplete final work or clock".into());
    }
    Ok(
        json!({"name":p["name"],"damping_s_m":alpha,"frames":frames.len(),"max_force_error_n":max_force,
        "max_first_moment_error_nm":max_moment,"max_heat_rate_error_w":max_power,
        "max_energy_defect_j":max_defect,"max_impulse_residual_ns":max_j,"max_clamped_area_m2_sampled":max_cut,
        "dissipated_normal_energy_j":prev_heat,"archived_undamped_com_distance_m":if alpha==0.{Some(max_base_pose)}else{None},
        "archived_undamped_impulse_distance_ns":if alpha==0.{Some(max_base_impulse)}else{None},"release":release(sole,p,donor,last)?}),
    )
}

pub(in super::super) fn run(free: &Path, directory: &Path) -> Result<()> {
    let donor = read(&free.join("donor.json"))?;
    let base = read(&free.join("reference.json"))?;
    let data = read(&directory.join("reference.json"))?;
    if [&donor, &base, &data]
        .iter()
        .any(|r| r["case"] != 80 || r["stroke"] != 2)
    {
        return Err("requires fixture 80, second stroke".into());
    }
    let control = array(&base["profiles"])?
        .iter()
        .find(|a| a["name"] == "Free approach")
        .ok_or("missing undamped free approach")?;
    let k = |key| num(&data["material"][key]);
    let ka = k("foot_stiffness_n_m3")?;
    let kb = k("deck_stiffness_n_m3")?;
    let sole = Sole {
        half: vector(&donor["foot_half_extents_m"])?,
        deck: vector(&donor["deck"]["center_m"])?,
        deck_half: vector(&donor["deck"]["half_extents_m"])?,
        stiffness: 1. / (1. / ka + 1. / kb),
    };
    let mut profiles = vec![];
    for p in array(&data["profiles"])? {
        profiles.push(check_profile(&sole, p, &donor, array(&control["frames"])?)?);
    }
    let mut levels = profiles
        .iter()
        .map(|p| num(&p["damping_s_m"]))
        .collect::<Result<Vec<_>>>()?;
    levels.sort_by(f64::total_cmp);
    if levels.len() != 3 || levels[0] != 0. || levels[1] <= 0. || levels[2] <= levels[1] {
        return Err("incomplete damping sweep".into());
    }
    write(
        &directory.join("audit.json"),
        &json!({"case":80,"stroke":2,"profiles":profiles}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unloading_across_a_rotating_flat_sole_counts_heat_in_the_tension_cutoff() {
        let sole = Sole {
            half: DVec3::new(1., 0.5, 0.1),
            deck: DVec3::new(0., 0., -1.),
            deck_half: DVec3::new(2., 2., 1.),
            stiffness: 1.,
        };
        let s = Motion {
            foot: DVec3::new(0., 0., -0.9),
            q: DQuat::IDENTITY,
            com: DVec3::ZERO,
            v: DVec3::ZERO,
            w: 2. * DVec3::Y,
        };
        let (r, heat, cut) = dynamic(&sole, s, 1.);
        // Unit compression throughout. Pressure max(0,1+2x) vanishes for x<=-1/2.
        for (got, want) in [
            (cut, 0.5),
            (r.area, 1.5),
            (r.force, 2.25),
            (r.first_moment.x, 1.125),
            (r.energy, 1.),
            (heat, 2.25),
        ] {
            assert!((got - want).abs() < 1e-13, "{got} vs {want}");
        }
    }
}
