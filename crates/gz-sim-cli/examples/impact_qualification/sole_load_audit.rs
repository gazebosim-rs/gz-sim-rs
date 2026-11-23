// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent cross-section integration of a flat elastic sole over a finite deck.
use super::{transfer::vector, *};
use glam::{DQuat, DVec3};

#[path = "damping_audit.rs"]
pub(super) mod damping;

/// Keep a*u + b*v + c >= 0 in material coordinates (metres).
#[derive(Clone, Copy)]
struct Limit {
    a: f64,
    b: f64,
    c: f64,
}

#[derive(Default)]
struct Reaction {
    area: f64,
    force: f64,
    first_moment: DVec3,
    energy: f64,
}

struct Sole {
    half: DVec3,
    deck: DVec3,
    deck_half: DVec3,
    stiffness: f64,
}

impl Sole {
    /// Integrate over the deck-clipped, compressed sole in material coordinates.
    /// An additional affine bound can select the positive-traction region.
    fn visit(
        &self,
        foot: DVec3,
        q: DQuat,
        bound: Option<Limit>,
        mut sample: impl FnMut(DVec3, f64, f64),
    ) {
        let u = q * DVec3::X;
        let v = q * DVec3::Y;
        let base = foot - q * DVec3::Z * self.half.z;
        let height = self.deck.z + self.deck_half.z;
        let mut limits = vec![
            Limit {
                a: 0.,
                b: 1.,
                c: self.half.y,
            },
            Limit {
                a: 0.,
                b: -1.,
                c: self.half.y,
            },
            Limit {
                a: -u.z,
                b: -v.z,
                c: height - base.z,
            },
        ];
        for axis in 0..2 {
            for sign in [-1., 1.] {
                limits.push(Limit {
                    a: sign * u[axis],
                    b: sign * v[axis],
                    c: self.deck_half[axis] + sign * (base[axis] - self.deck[axis]),
                });
            }
        }
        limits.extend(bound);
        // Within successive u cuts the upper and lower v limits are affine.
        // Include crossings with the sides of the sole and zero-pressure line.
        let (mut left, mut right) = (-self.half.x, self.half.x);
        for &p in &limits {
            if p.b == 0. {
                if p.a > 0. {
                    left = left.max(-p.c / p.a);
                } else if p.a < 0. {
                    right = right.min(-p.c / p.a);
                } else if p.c < 0. {
                    return;
                }
            }
        }
        if left >= right {
            return;
        }
        let mut cuts = vec![left, right];
        for (i, p) in limits.iter().enumerate() {
            for s in &limits[..i] {
                let determinant = p.a * s.b - p.b * s.a;
                if determinant != 0. {
                    let x = (p.b * s.c - p.c * s.b) / determinant;
                    if x > left && x < right {
                        cuts.push(x);
                    }
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        // For the damped law pressure is quadratic and power is cubic within
        // a strip. The affine integration width supplies one more degree.
        const GAUSS: [(f64, f64); 3] = [
            (-0.774_596_669_241_483_4, 5. / 9.),
            (0., 8. / 9.),
            (0.774_596_669_241_483_4, 5. / 9.),
        ];
        for interval in cuts.windows(2) {
            let dx = (interval[1] - interval[0]) * 0.5;
            for (sx, wx) in GAUSS {
                let x = interval[0] + dx * (1. + sx);
                let (mut lo, mut hi) = (-self.half.y, self.half.y);
                for p in &limits {
                    if p.b > 0. {
                        lo = lo.max(-(p.a * x + p.c) / p.b);
                    }
                    if p.b < 0. {
                        hi = hi.min(-(p.a * x + p.c) / p.b);
                    }
                }
                if lo >= hi {
                    continue;
                }
                let dy = (hi - lo) * 0.5;
                for (sy, wy) in GAUSS {
                    let y = lo + dy * (1. + sy);
                    let site = base + x * u + y * v;
                    let compression = height - site.z;
                    if compression <= 0. {
                        continue;
                    }
                    sample(site, compression, wx * wy * dx * dy);
                }
            }
        }
    }

    fn reaction(&self, foot: DVec3, q: DQuat) -> Reaction {
        let mut out = Reaction::default();
        self.visit(foot, q, None, |site, compression, area| {
            let force = area * self.stiffness * compression;
            out.area += area;
            out.force += force;
            out.energy += 0.5 * compression * force;
            out.first_moment += force * site;
        });
        out
    }
}

fn check_profile(sole: &Sole, p: &Value) -> Result<Value> {
    let frames = array(&p["frames"])?;
    let initial = frames.first().ok_or("missing initial material frame")?;
    let t0 = num(&initial["original_time_s"])?;
    let v0 = vector(&initial["v_m_s"])?;
    let mut rows = vec![];
    for f in frames {
        let q = DQuat::from_array(serde_json::from_value(f["orientation_xyzw"].clone())?);
        if !q.is_normalized() {
            return Err("invalid foot orientation".into());
        }
        let r = sole.reaction(vector(&f["foot_m"])?, q);
        let t = num(&f["original_time_s"])?;
        let force = num(&f["normal_force_n"])?;
        let area = num(&f["loaded_area_m2"])?;
        let first = if force > 0. {
            vector(&f["center_of_pressure_m"])? * force
        } else {
            if !f["center_of_pressure_m"].is_null() {
                return Err("zero-load pressure center".into());
            }
            DVec3::ZERO
        };
        let balance = 2. * (vector(&f["v_m_s"])? - v0 + (9.81 * (t - t0)) * DVec3::Z);
        let momentum_residual = balance.distance(vector(&f["support_frame_j_ns"])?);
        let df = (r.force - force).abs();
        let dm = r.first_moment.distance(first);
        let de = (r.energy - num(&f["strain_energy_j"])?).abs();
        // At a coplanar instant the positive-area set is ill-conditioned
        // (an arbitrarily small pose error can move its boundary across the sole).
        // Force, energy and moment still have well-defined limits there.
        let da = (r.area - area).abs();
        if df > 1e-6
            || dm > 1e-7
            || de > 1e-8
            || momentum_residual > 1e-9
            || (r.force.max(force) > 1e-6 && da > 1e-8)
        {
            return Err(format!("{} at {t} s: load {df} N, first moment {dm} Nm, area {da} m², strain energy {de} J, impulse residual {momentum_residual} N s", p["name"]).into());
        }
        rows.push(json!({"original_time_s":t,"force_difference_n":df,"area_difference_m2":da,"pressure_first_moment_difference_nm":dm,"energy_difference_j":de,"momentum_residual_ns":momentum_residual}));
    }
    Ok(json!({"name":p["name"],"frames":rows}))
}

fn state_mean(row: &Value, key: &str) -> Result<DVec3> {
    Ok((vector(&row["foot"][key])? + vector(&row["upper"][key])?) * 0.5)
}

fn native_comparison(edge_roll: &Path, donor: &Value, profiles: &[Value]) -> Result<Vec<Value>> {
    let source = read(&edge_roll.join("reference.json"))?;
    let start = num(&donor["public_restart_s"])?;
    if (num(&source["references"][0]["fine"]["elapsed"])? - start).abs() > 1e-12 {
        return Err("mismatched public restart".into());
    }
    let drive = vector(&donor["drive"])?;
    let rigid_j = vector(&source["references"][0]["fine"]["j"])?;
    let traces = edge_roll.join("raw/case-0080-stroke-2/traces");
    let mut controls = vec![];
    for (lane, name) in [
        (3, "Eight-call local solver"),
        (4, "Continuous 8192-call solver"),
    ] {
        let trace = read(&traces.join(format!("lane-{lane}.json")))?;
        let rows = array(&trace["frames"])?;
        let initial = rows.first().ok_or("missing native restart")?;
        if ["foot", "upper", "pad", "time_us", "tick"]
            .into_iter()
            .any(|k| initial[k] != source["frames"][0][k])
        {
            return Err("unmatched native restart pose or clock".into());
        }
        let time0 = uint(&initial["time_us"])?;
        let mut j = rigid_j;
        let mut records = vec![];
        for (index, row) in rows.iter().enumerate() {
            let us = uint(&row["time_us"])?
                .checked_sub(time0)
                .ok_or("clock reversal")?;
            let t = start + us as f64 * 1e-6;
            if index > 0 {
                if num(&row["drive"])? != drive.z {
                    return Err("actuator changed in native traces".into());
                }
                j += vector(&row["j"])?;
            }
            let mut comparisons = vec![];
            for p in profiles {
                if t + 1e-12 < num(&p["start_s"])? {
                    continue;
                }
                let f = array(&p["frames"])?
                    .iter()
                    .find(|r| {
                        r["original_time_s"]
                            .as_f64()
                            .is_some_and(|a| (a - t).abs() < 1e-12)
                    })
                    .ok_or("missing material clock")?;
                let delta = vector(&f["com_m"])? + drive * t - state_mean(row, "position")?;
                let dj = vector(&p["rigid_impulse_before_start_ns"])?
                    + vector(&f["support_frame_j_ns"])?
                    - j;
                comparisons.push(json!({"profile":p["name"],"com_distance_m":delta.length(),"com_shift_m":delta.to_array(),"contact_impulse_distance_ns":dj.length(),"contact_impulse_shift_ns":dj.to_array()}));
            }
            records.push(json!({"public_elapsed_us":us,"original_time_s":t,"material_comparisons":comparisons}));
        }
        controls.push(json!({"control":name,"frames":records}));
    }
    Ok(controls)
}

// At a common time each trajectory has received the same gravity impulse
// since the free cut. All differences in momentum come from contact.
fn compare_starts(profiles: &[Value]) -> Result<Vec<Value>> {
    let free = profiles
        .iter()
        .find(|p| p["name"] == "Free approach")
        .ok_or("missing free trajectory")?;
    let free_frames = array(&free["frames"])?;
    let mut reports = vec![];
    for p in profiles.iter().filter(|p| p["name"] != free["name"]) {
        let inherited = vector(&p["rigid_impulse_before_start_ns"])?
            - vector(&free["rigid_impulse_before_start_ns"])?;
        let mut samples = vec![];
        for f in array(&p["frames"])? {
            let t = num(&f["original_time_s"])?;
            let full = free_frames
                .iter()
                .find(|v| {
                    v["original_time_s"]
                        .as_f64()
                        .is_some_and(|x| (x - t).abs() < 1e-13)
                })
                .ok_or("missing common clock")?;
            let j = vector(&full["support_frame_j_ns"])?
                - vector(&f["support_frame_j_ns"])?
                - inherited;
            let dv = vector(&full["v_m_s"])? - vector(&f["v_m_s"])?;
            let residual = j.distance(2. * dv);
            if residual > 1e-8 {
                return Err(
                    "incorrect time origin or inherited impulse in restart comparison".into(),
                );
            }
            let delta = vector(&full["com_m"])? - vector(&f["com_m"])?;
            samples.push(json!({"original_time_s":t,"com_distance_m":delta.length(),"free_minus_restart_com_m":delta.to_array(),
                "impulse_distance_ns":j.length(),"free_minus_restart_j_ns":j.to_array(),"momentum_residual_ns":residual}));
        }
        reports.push(json!({"restart":p["name"],"frames":samples}));
    }
    Ok(reports)
}

pub(super) fn run(edge_roll: &Path, directory: &Path) -> Result<()> {
    let donor = read(&directory.join("donor.json"))?;
    let study = read(&directory.join("reference.json"))?;
    if [&donor, &study]
        .iter()
        .any(|d| d["case"] != 80 || d["stroke"] != 2)
    {
        return Err("requires fixture 80 stroke 2".into());
    }
    let k = |name| num(&study["material"][name]);
    let k1 = k("foot_stiffness_n_m3")?;
    let k2 = k("deck_stiffness_n_m3")?;
    if k1 <= 0. || k2 <= 0. {
        return Err("invalid layer stiffness".into());
    }
    let sole = Sole {
        half: vector(&donor["foot_half_extents_m"])?,
        deck: vector(&donor["deck"]["center_m"])?,
        deck_half: vector(&donor["deck"]["half_extents_m"])?,
        stiffness: 1. / (1. / k1 + 1. / k2),
    };
    let ps = array(&study["profiles"])?;
    if ps.len() != 3 {
        return Err("missing landing histories".into());
    }
    let checks = ps
        .iter()
        .map(|p| check_profile(&sole, p))
        .collect::<Result<Vec<_>>>()?;
    let restarts = compare_starts(ps)?;
    let controls = native_comparison(edge_roll, &donor, ps)?;
    write(
        &directory.join("load-audit.json"),
        &json!({"profiles":checks,"native_controls":controls,"restart_comparisons":restarts,
        "native_origin":"Both native controls resume at the archived public boundary after the primary rigid impact. The initial rigid impulse is included in the cumulative comparison."}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn deck_clips_loaded_half_of_rotated_sole() {
        let block = Sole {
            half: DVec3::new(1., 0.5, 0.1),
            deck: DVec3::new(0., 0.25, -1.),
            deck_half: DVec3::new(3., 0.25, 1.),
            stiffness: 10.,
        };
        let angle = 0.2_f64;
        let q = DQuat::from_rotation_y(angle);
        // Sole center lies on the plane. u > 0 and v in [0, 0.5] carry load.
        let f = block.reaction(q * DVec3::Z * block.half.z, q);
        assert!((f.area - 0.5).abs() < 1e-14);
        assert!((f.force - 2.5 * angle.sin()).abs() < 1e-14);
        assert!((f.energy - 5. / 6. * angle.sin().powi(2)).abs() < 1e-14);
        let center = f.first_moment / f.force;
        assert!((center.x - 2. / 3. * angle.cos()).abs() < 1e-14);
        assert!((center.y - 0.25).abs() < 1e-14);
        assert_eq!(block.reaction(DVec3::Z, q).force, 0.);
    }
}
