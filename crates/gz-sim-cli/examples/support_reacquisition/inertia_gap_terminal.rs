// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Whole-sequence native controls and sensitivity to the two actual materials.
use super::*;
#[path = "inertia_gap_terminal_view.rs"]
mod replay;
pub(super) struct Terminal {
    pub(super) offset: f64,
    pub(super) cut: Flight,
    pub(super) finite: Vec<(f64, Crossing)>,
    pub(super) impulses: [DVec3; 2],
    pub(super) radius: [f64; 2],
    pub(super) corner: [usize; 2],
    pub(super) free: f64,
    pub(super) gap_time: f64,
    pub(super) speed_err: f64,
    pub(super) momentum_err: f64,
    pub(super) speed_cap: f64,
    pub(super) spin_cap: f64,
    position_error: f64,
    angle_error: f64,
}
fn value_vec(v: &Value) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
impl Terminal {
    pub(super) fn new(i: &Impact, lower: Crossing, s: &Sequence) -> Option<Self> {
        let proof = &s.limit["proof"];
        let extended = &s.limit["extended"];
        let free = extended["free_after_s"]
            .as_f64()
            .or_else(|| proof["free_after_s"].as_f64())?;
        let fixed = &proof["frozen"];
        let last = *s.finite.last()?;
        let side = last.surface;
        let corner = [lower.corner, s.finite[0].corner];
        let mut cut = last.after();
        let w0 = cut.l.length() / i.r.body.lower;
        cut.v = value_vec(&fixed["velocity"]);
        cut.l = value_vec(&fixed["momentum"]);
        let impulses = [0, 1].map(|k| value_vec(&fixed["impulses"][k]));
        let err = s
            .limit
            .get("propagated")
            .cloned()
            .unwrap_or_else(|| uncertainty::Error::default().data());
        let radius = [0, 1].map(|k| {
            err["impulses_ns"][k].as_f64().unwrap()
                + s.limit["radii"][k].as_f64().unwrap_or_else(|| {
                    proof["world_vector_radii"][k].as_f64().unwrap()
                        + impulses[k].distance(value_vec(&proof["world_vector_centers"][k]))
                        + 2e-12
                })
        });
        let gap_time = proof["remaining_time_cap_s"].as_f64()? + err["time_s"].as_f64()?;
        let position_error = err["position_m"].as_f64()?;
        let angle_error = err["angle_rad"].as_f64()?;
        let sum = proof["total_impulse_bound_ns"].as_f64()?;
        let rr = corner.map(|j| vertices()[j].length());
        let maxr = rr[0].max(rr[1]);
        let spin_cap = w0 + (maxr * sum + err["momentum_nms"].as_f64()?) / i.r.body.lower;
        let speed_cap = err["velocity_m_s"].as_f64()?
            + last.after().v.length()
            + i.r.gravity.length() * gap_time
            + sum / M;
        let speed_err = err["velocity_m_s"].as_f64()?
            + (radius[0] + radius[1]) / M
            + i.r.gravity.length() * gap_time;
        let momentum_err = err["momentum_nms"].as_f64()?
            + radius[0] * rr[0]
            + radius[1] * rr[1]
            + maxr * (angle_error + spin_cap * gap_time) * sum;
        let offset = s.records.last()?["offset_s"].as_f64()?
            + s.records.last()?["offset_correction_s"].as_f64()?;
        let finite = s
            .records
            .iter()
            .zip(&s.finite)
            .map(|(r, h)| {
                (
                    r["offset_s"].as_f64().unwrap() + r["offset_correction_s"].as_f64().unwrap(),
                    *h,
                )
            })
            .collect();
        debug_assert_eq!(corner[side], last.corner);
        Some(Self {
            offset,
            cut,
            finite,
            impulses,
            radius,
            corner,
            free,
            gap_time,
            speed_err,
            momentum_err,
            speed_cap,
            spin_cap,
            position_error,
            angle_error,
        })
    }
    pub(super) fn state(&self, r: &Physical, lower: Crossing, delta: f64) -> Flight {
        if delta >= self.offset {
            return advance(r, self.cut, delta - self.offset);
        }
        self.finite
            .iter()
            .rev()
            .find(|(at, _)| *at <= delta)
            .map_or_else(
                || advance(r, lower.after(), delta),
                |(at, h)| advance(r, h.after(), delta - at),
            )
    }
    pub(super) fn bin(&self, lo: f64, hi: f64) -> [DVec3; 2] {
        let mut result = [DVec3::ZERO; 2];
        for (t, h) in &self.finite {
            if *t > lo && *t <= hi {
                result[h.surface] += h.law.impulse;
            }
        }
        if self.offset > lo && self.offset <= hi {
            for (k, r) in result.iter_mut().enumerate() {
                *r += self.impulses[k];
            }
        }
        result
    }
    pub(super) fn bounds(&self, r: &Physical, after: f64) -> Value {
        assert!(after >= self.gap_time && after <= self.free);
        let dt = after;
        let q0 = self.angle_error + 2. * self.spin_cap * self.gap_time;
        let wcap = self
            .spin_cap
            .max((self.cut.l.length() + self.momentum_err) / r.body.lower);
        // Normed quaternion flow f(q)=1/2 q I^-1 R(q)^T L is Lipschitz
        // with constant <=2.5 |L|/Imin. Use a rotation-angle bound from
        // quaternion distance, with slack throughout this short horizon.
        let k = 2.5 * wcap;
        let g = (k * dt).exp();
        let qdist = g * (q0 + (self.momentum_err / r.body.lower) * dt);
        let angle = 4. * qdist;
        let com = self.position_error + 2. * self.speed_cap * self.gap_time + dt * self.speed_err;
        json!({"free_after_s":self.free,"remaining_time_cap_s":self.gap_time,"linear_speed_error_m_s":self.speed_err,"momentum_error_nms":self.momentum_err,"com_error_m":com,"angle_error_rad":angle,"spin_error_rad_s":self.momentum_err/r.body.lower+2.*wcap*angle})
    }
}
fn paired(
    i: &Impact,
    g: &Gate,
    gate_mu: f64,
    lower: Crossing,
    s: &Sequence,
    terminal: &Terminal,
) -> crate::Result<Value> {
    let after_ns = (0.5 * terminal.free * 1e9).floor().min(32.) as u64;
    let dt = (terminal.offset * 1e9).ceil() as u64 + after_ns;
    if dt as f64 * 1e-9 > terminal.offset + terminal.free {
        return Ok(Value::Null);
    }
    let raw = native::source(i, g, gate_mu);
    let controls = native::third_controls(i, g, &raw, lower.at, lower.after(), dt)?;
    let exact = terminal.state(&i.r, lower, dt as f64 * 1e-9);
    let js = terminal.bin(0., dt as f64 * 1e-9);
    let bounds = terminal.bounds(&i.r, dt as f64 * 1e-9 - terminal.offset);
    Ok(
        json!({"dt_ns":dt,"after_last_ns":after_ns,"finite_events":s.finite.len(),"finite_strikes":s.records,"conditional_proof":s.limit["proof"],"extended_proof":s.limit["extended"],"last_offset_s":terminal.offset,"normal_sums_ns":([0,1].map(|k|js[k].dot(if k==0{i.r.n}else{-i.r.n}))),"world_impulses":js.map(|j|j.to_array()),"world_vector_radii":terminal.radius,"frozen_tail_ns":terminal.impulses.map(|j|j.to_array()),"bounds":bounds,"after":body_view(native::at_foot(&i.r,exact)),"controls":controls}),
    )
}
fn material_test(i: &Impact, g: &Gate, upper: Crossing, lower: Crossing) -> Value {
    let mut cases = vec![];
    let planes = faces(&i.r, g);
    for lmu in [0., 0.15, 0.45, 0.85] {
        for umu in [0., 0.15, 0.45, 0.85] {
            let mu = [lmu, umu];
            let mut hit = lower;
            hit.mu = lmu;
            let law = coulomb(i.r.body, hit.before, hit.arm, planes[0], lmu);
            let row = match law {
                Err(reason) => json!({"mu":mu,"kind":"declined_lower","reason":reason}),
                Ok(law) => {
                    hit.law = law;
                    let s = sequence(i, g, upper, hit, mu);
                    let finite = s.finite.iter().fold([DVec3::ZERO; 2], |mut sum, h| {
                        sum[h.surface] += h.law.impulse;
                        sum
                    });
                    let terminal = Terminal::new(i, hit, &s);
                    let sums = terminal.as_ref().map(|x| x.bin(0., x.offset + 8e-9));
                    json!({"mu":mu,"lower_sticks":law.stick,"lower_impulse":law.impulse.to_array(),"third_dt_s":s.third.map(|(_,dt)|dt.data()),"finite_events":s.finite.len(),"finite_impulses":finite.map(|j|j.to_array()),"last_conditional_event":s.finite.last().map(|h|h.data()),"stop":s.reason,"kind":s.limit["kind"],"tail":s.limit.get("proof").map(|p|json!({"normal_speed_pair_ratio_cap":p["normal_speed_pair_ratio_cap"],"outward_acceleration":p["both_release_accel_floor"],"limit":p["normal_sum_bounds_ns"],"frozen":p["frozen"]["impulses"]})),"terminal_vector_bounds":terminal.as_ref().map(|x|x.radius),"integrated_world_impulses":sums.map(|v|v.map(|j|j.to_array()))})
                }
            };
            cases.push(row);
        }
    }
    json!(cases)
}
pub(crate) fn write(path: &Path, sensitivity: bool) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut donors = vec![];
    let mut display_done = false;
    for i in inputs() {
        let mut cases = vec![];
        for delay in [20000, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            let ps = faces(&i.r, &g);
            for gm in [0., 0.35, 0.8] {
                let mu = [i.mu, i.mu.min(gm)];
                let Ok(upper) = upper(&i, &g, mu[1]) else {
                    continue;
                };
                let Some(lower) = follow(&i, ps, mu, upper, WINDOW)
                    .next
                    .filter(|h| h.surface == 0)
                else {
                    continue;
                };
                let s = sequence(&i, &g, upper, lower, mu);
                let mut paired_data = Value::Null;
                if !sensitivity && let Some(term) = Terminal::new(&i, lower, &s) {
                    paired_data = paired(&i, &g, gm, lower, &s, &term)?;
                    if !display_done
                        && term.offset > 30e-9
                        && term.offset < 100e-6
                        && term.free > term.offset * 1.2
                        && term.radius[0] + term.radius[1] < 1e-4
                        && term.impulses[0].length() + term.impulses[1].length() > 1e-5
                    {
                        replay::write(path, &i, &g, gm, lower, &term, None)?;
                        display_done = true;
                    }
                }
                cases.push(json!({"delay_ns":delay,"gate_mu":gm,"gate_velocity":g.velocity.to_array(),"original_kind":s.limit["kind"],"upper":upper.data(),"lower":lower.data(),"actual":paired_data,"material_controls":sensitivity.then(||material_test(&i,&g,upper,lower))}));
            }
        }
        if !cases.is_empty() {
            donors.push(json!({"id":i.r.id,"mode":MODES[i.mode],"tensor":i.r.body.tensor.to_cols_array(),"primary":law(&i),"pad_velocity":i.r.pad_v.to_array(),"cases":cases}));
        }
    }
    if !sensitivity {
        assert!(display_done);
    }
    write_json(
        &path.join(if sensitivity {
            "materials.json"
        } else {
            "terminal.json"
        }),
        &json!({"donors":donors}),
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn opposing_limits_distinguish_accumulation_departure_and_inconclusive_cuts() {
        for (id, mode, gm, want) in [
            (1160, "slide", 0.35, "accumulation_then_free_release"),
            (16, "stick", 0.8, "departing"),
            (34, "stick", 0.35, "unresolved"),
        ] {
            let i = inputs()
                .into_iter()
                .find(|i| i.r.id == id && MODES[i.mode] == mode)
                .unwrap();
            let g = gate(&i, 750e-6);
            let mu = [i.mu, i.mu.min(gm)];
            let u = upper(&i, &g, mu[1]).unwrap();
            let l = follow(&i, faces(&i.r, &g), mu, u, WINDOW).next.unwrap();
            assert_eq!(l.surface, 0);
            let s = sequence(&i, &g, u, l, mu);
            assert_eq!(s.limit["kind"], want);
            if want == "accumulation_then_free_release" {
                let limit = Terminal::new(&i, l, &s).unwrap();
                assert!(limit.free >= 128e-9 && limit.speed_err > 0.);
                let endpoint = limit.offset + 64e-9;
                let js = limit.bin(0., endpoint);
                let x = limit.state(&i.r, l, endpoint);
                let net = M * (x.v - l.after().v - i.r.gravity * endpoint);
                assert!(net.distance(js[0] + js[1]) < 1e-10);
                assert!(
                    s.limit["proof"]["normal_speed_pair_ratio_cap"]
                        .as_f64()
                        .unwrap()
                        < 1.
                );
            }
        }
    }
}

pub(super) fn compact_view(
    path: &Path,
    i: &Impact,
    g: &Gate,
    mu: f64,
    l: Crossing,
    runs: (&Sequence, &Sequence),
) -> crate::Result<()> {
    let full = Terminal::new(i, l, runs.0).ok_or("missing complete terminal reference")?;
    let early = Terminal::new(i, l, runs.1).ok_or("missing early terminal reference")?;
    replay::write(path, i, g, mu, l, &full, Some(&early))
}
