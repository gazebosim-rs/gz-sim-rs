// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A same-vertex return with an independent closed-form principal-axis trajectory.
use super::*;

struct Return {
    case: Impact,
    seconds: f64,
    speed: f64,
    sin_arm: f64,
    cos_arm: f64,
    lower_normal_jerk: f64,
}
fn sin_minus_x(x: f64) -> f64 {
    // For the admitted |x| < .03 the term after x^9 is below 5e-25.
    let x2 = x * x;
    x * x2 * (-1. / 6. + x2 * (1. / 120. + x2 * (-1. / 5040. + x2 / 362880.)))
}
impl Return {
    fn gap(&self, t: f64) -> f64 {
        let x = self.speed * t;
        self.sin_arm * sin_minus_x(x)
            + self.cos_arm * 2. * (0.5 * x).sin().powi(2)
            + 0.5 * self.case.r.gravity.dot(self.case.r.n) * t * t
    }
    fn normal_rate(&self, t: f64) -> f64 {
        let x = self.speed * t;
        self.speed * (-2. * self.sin_arm * (0.5 * x).sin().powi(2) + self.cos_arm * x.sin())
            + self.case.r.gravity.dot(self.case.r.n) * t
    }
    fn create(id: usize, mode: usize) -> Option<Self> {
        let body = ReferenceBody::new(DVec3::new(0.012, 0.019, 0.025), DQuat::IDENTITY);
        let mut r = Physical::with_body(id, body);
        let arrival = (1_500_013 + (id * 3137 % 150_000)) as f64 * 1e-9;
        r.at += r.pad_v * (arrival - r.arrival);
        r.arrival = arrival;
        let mut case = manufacture_for(r, mode);
        if case.unique_margin <= 1e-8 || case.r.closing < 0.05 || case.loss <= 0. {
            return None;
        }
        let r = &mut case.r;
        let arm = r.q * (corners()[r.corner] - ARM);
        let mut axis = r.q
            * if id.is_multiple_of(2) {
                DVec3::X
            } else {
                DVec3::Y
            };
        if axis.cross(arm).dot(r.n) < 0. {
            axis = -axis;
        }
        let a = axis.cross(arm).dot(r.n);
        let b = axis.cross(axis.cross(arm)).dot(r.n);
        let s = [8., 12., 18., 24.][(id / 5) % 4];
        let seconds = [7., 30., 75., 160., 280.][id % 5] * 1e-6;
        let x = s * seconds;
        let gn =
            -2. * (a * sin_minus_x(x) + b * 2. * (0.5 * x).sin().powi(2)) / (seconds * seconds);
        r.gravity = gn * r.n;
        let omega = s * axis;
        r.post.l = r.body.momentum(r.q, omega);
        r.pre.l = r.post.l - arm.cross(case.impulse);
        r.pre.v = r.pad_v + case.before - r.body.omega(r.q, r.pre.l).cross(arm);
        r.post.v = r.pre.v + case.impulse / M;
        let w = r.body.omega(r.q, r.post.l);
        assert!(r.body.accel(r.q, w).length() < 1e-10);
        assert!((r.post.v - r.pad_v + w.cross(arm)).distance(case.after) < 1e-12);
        let horizon = 0.0005;
        let phase = s * horizon;
        let lower = s.powi(3) * (a * phase.cos() - b.abs() * phase.sin());
        if lower <= 0.5 || (gn + b * s * s) <= 1e-5 || gn.abs() > 150. {
            return None;
        }
        // All other bottom corners stay above the top, all four remain inside
        // the finite face, and the upper face remains above the lower one.
        let drift = (r.post.v - r.pad_v).length() * horizon + 0.5 * gn.abs() * horizon * horizon;
        for (k, v) in corners().iter().enumerate() {
            let z = r.q * (*v - ARM);
            let local = r.pad_q.conjugate() * (r.at - r.pad_v * r.arrival + z);
            let travel = drift + z.length() * phase;
            if (k != r.corner && local.z <= travel + 1e-12)
                || PAD_HALF[0] - local.x.abs() <= travel + 1e-12
                || PAD_HALF[1] - local.y.abs() <= travel + 1e-12
            {
                return None;
            }
        }
        if (r.q * DVec3::Z).dot(r.n) <= phase + 1e-12
            || r.certify_branches(&[(0., r.pre)]).reason.is_some()
        {
            return None;
        }
        let ret = Self {
            case,
            seconds,
            speed: s,
            sin_arm: a,
            cos_arm: b,
            lower_normal_jerk: lower,
        };
        assert!(
            ret.gap(seconds).abs() < 1e-15
                && ret.gap(seconds * 0.5) > 0.
                && ret.gap(seconds * 1.1) < 0.
                && ret.normal_rate(seconds) < 0.
        );
        Some(ret)
    }
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut cases = Vec::new();
    for id in 0..512 {
        let mode = id % 4;
        let Some(ctrl) = Return::create(id, mode) else {
            continue;
        };
        let i = &ctrl.case;
        let r = &i.r;
        let first = (r.arrival * 1e9).round() as u64;
        let second = first + (ctrl.seconds * 1e9).round() as u64;
        let tick = first.div_ceil(500_000);
        let mut runs = vec![];
        for compact in [false, true] {
            for follow in [false, true] {
                let text = Study::Departure.text(i, 8, follow, compact);
                let (mut sim, ids) = load(&text, false)?;
                let stream = Arc::new(Mutex::new(Vec::new()));
                let seen = stream.clone();
                set_refined_observer(&mut sim, move |time, s| {
                    let Some(offset) = time.iteration.checked_sub(1) else {
                        return;
                    };
                    let end = offset * 500_000 + s.elapsed.as_nanos() as u64;
                    let cut = gyro::Cut {
                        foot: s.state(ids.foot).unwrap(),
                        pad: s.state(ids.pad).unwrap(),
                        ns: s.elapsed.as_nanos() as u64,
                    };
                    let measured = measured(cut, ids, s.contact_patches());
                    seen.lock()
                        .unwrap()
                        .push((end, s.rigid_point_impact, measured.corners));
                })?;
                sim.step_exact(tick)?;
                let log = last_box_timing(&sim).ok_or("return landing not selected")?;
                let spans = log.timing.departure_calls.as_deref().unwrap_or(&[]);
                let start = (tick - 1) * 500_000;
                let mut certified = vec![];
                for (t, h) in spans {
                    let lo = start + t.as_nanos() as u64;
                    let hi = lo + h.as_nanos() as u64;
                    assert!(
                        lo >= first && hi < second,
                        "covered physical return {id} {lo}:{hi}>{second}"
                    );
                    certified.push([lo, hi]);
                }
                let obs = stream.lock().unwrap();
                let applied=obs.iter().filter_map(|(end,hit,_)|hit.map(|h|json!({"application_ns":(*end-h.pulse_duration.as_nanos()as u64-h.pulse_start.as_nanos()as u64)+h.impact.elapsed.as_nanos()as u64,"end_ns":end,"world":h.impact.impulse_on_foot}))).collect::<Vec<_>>();
                let native = obs
                    .iter()
                    .filter(|(_, _, q)| q.iter().any(|&v| v > 0.))
                    .map(|(end, _, q)| json!({"end_ns":end,"corner_impulse":q}))
                    .collect::<Vec<_>>();
                runs.push(json!({"compact":compact,"follow":follow,"certified_ns":certified,"applications":applied,"native":native,"durations_ns":log.timing.durations.iter().map(|h|h.as_nanos()as u64).collect::<Vec<_>>()}));
            }
        }
        cases.push(json!({"id":id,"mode":MODES[mode],"first_ns":first,"second_ns":second,"return_after_s":ctrl.seconds,"speed_rad_s":ctrl.speed,"sin_arm_m":ctrl.sin_arm,"cos_arm_m":ctrl.cos_arm,"normal_acceleration":r.gravity.dot(r.n),"negative_jerk_min":ctrl.lower_normal_jerk,"mid_gap_m":ctrl.gap(ctrl.seconds/2.),"return_rate_m_s":ctrl.normal_rate(ctrl.seconds),"reference_impulse":i.impulse.to_array(),"trials":runs}));
        if cases.len() == 160 {
            break;
        }
    }
    assert_eq!(cases.len(), 160);
    write_json(&path.join("short_returns.json"), &json!({"cases":cases}))
}
