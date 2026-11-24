// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Contacts, Duration, RigidContactPatch, Serialize};
use glam::DVec3;

/// One jaw's uninterrupted positive reaction within a single controller interval.
#[derive(Debug, Clone, Serialize)]
pub(super) struct JawPulse {
    pub jaw: &'static str,
    pub start_ns: u128,
    pub end_ns: u128,
    pub loaded_steps: u32,
    pub positive_rows: u64,
    pub normal_impulse_ns: f64,
    pub linear_on_parcel_ns: [f64; 3],
    pub angular_normal_at_parcel_com_nms: [f64; 3],
}

pub(super) fn record(
    pulses: &mut Vec<JawPulse>,
    patches: &[RigidContactPatch],
    ids: Contacts,
    end: Duration,
    dt: Duration,
) {
    let start_ns = (end - dt).as_nanos();
    let end_ns = end.as_nanos();
    for patch in patches {
        let Some((peer, sign)) = ids.counterpart(patch.a, patch.b) else {
            continue;
        };
        let jaw = if peer == ids.left {
            "left"
        } else if peer == ids.right {
            "right"
        } else {
            continue;
        };
        let direction = DVec3::from_array(patch.normal_on_a) * sign;
        let mut rows = patch
            .points
            .iter()
            .filter(|p| p.normal_impulse > 0.)
            .peekable();
        if rows.peek().is_none() {
            continue;
        }
        let pulse = match pulses
            .iter_mut()
            .rev()
            .find(|p| p.jaw == jaw && p.end_ns >= start_ns)
        {
            Some(p) => p,
            None => {
                pulses.push(JawPulse {
                    jaw,
                    start_ns,
                    end_ns: start_ns,
                    loaded_steps: 0,
                    positive_rows: 0,
                    normal_impulse_ns: 0.,
                    linear_on_parcel_ns: [0.; 3],
                    angular_normal_at_parcel_com_nms: [0.; 3],
                });
                pulses.last_mut().unwrap()
            }
        };
        if pulse.end_ns != end_ns {
            pulse.loaded_steps += 1;
            pulse.end_ns = end_ns;
        }
        for row in rows {
            let impulse = direction * row.normal_impulse;
            let lever = if ids.parcel == patch.a {
                row.lever_a
            } else {
                row.lever_b
            };
            pulse.normal_impulse_ns += row.normal_impulse;
            pulse.positive_rows += 1;
            pulse.linear_on_parcel_ns =
                (DVec3::from_array(pulse.linear_on_parcel_ns) + impulse).to_array();
            pulse.angular_normal_at_parcel_com_nms =
                (DVec3::from_array(pulse.angular_normal_at_parcel_com_nms)
                    + DVec3::from_array(lever).cross(impulse))
                .to_array();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Args, run};
    use clap::Parser;
    use gz_physics::BodyId;
    use gz_physics_multifidelity::RigidPatchPoint;

    #[test]
    fn multiple_rows_share_one_time_bin_and_jaws_keep_separate_histories() {
        let ids = Contacts {
            left: BodyId::from_raw(1),
            right: BodyId::from_raw(2),
            parcel: BodyId::from_raw(3),
        };
        let patch = |a, b, j| RigidContactPatch {
            a,
            b,
            normal_on_a: [1., 0., 0.],
            points: vec![RigidPatchPoint {
                lever_a: [0., 2., 0.],
                lever_b: [0., -3., 0.],
                normal_impulse: j,
                tangent_impulse_on_a: None,
            }],
        };
        let left = patch(ids.parcel, ids.left, 2.);
        let right = patch(ids.right, ids.parcel, 1.);
        let dt = Duration::from_nanos(10);
        let mut pulses = vec![];
        record(
            &mut pulses,
            &[
                left.clone(),
                left.clone(),
                right.clone(),
                patch(ids.parcel, ids.right, 0.),
                patch(ids.left, ids.right, 8.),
                patch(ids.parcel, BodyId::from_raw(9), 3.),
            ],
            ids,
            dt,
            dt,
        );
        record(&mut pulses, &[right.clone(), left.clone()], ids, dt * 2, dt);
        record(&mut pulses, &[right], ids, dt * 3, dt);
        record(&mut pulses, &[left], ids, dt * 4, dt);
        assert_eq!(pulses.len(), 3);
        assert_eq!(
            (
                pulses[0].start_ns,
                pulses[0].end_ns,
                pulses[0].positive_rows,
                pulses[0].loaded_steps
            ),
            (0, 20, 3, 2)
        );
        assert_eq!(pulses[0].angular_normal_at_parcel_com_nms, [0., 0., -12.]);
        assert_eq!(pulses[1].linear_on_parcel_ns, [-3., 0., 0.]);
        assert_eq!(pulses[1].angular_normal_at_parcel_com_nms, [0., 0., -9.]);
        assert_eq!((pulses[2].start_ns, pulses[2].end_ns), (30, 40));
    }

    #[test]
    fn measuring_a_separated_strike_preserves_the_trajectory_and_total_reaction()
    -> crate::Result<()> {
        for steps in ["1", "500"] {
            let mut t = Args::try_parse_from([
                "study",
                "--right",
                "--gap-um",
                "350",
                "--divisor",
                "1.75",
                "--ticks",
                "2",
                "--substeps",
                steps,
            ])?
            .trial;
            let plain = run(t.clone())?;
            t.impact_pulses = true;
            let traced = run(t)?;
            for (ordinary, observed) in plain.samples.iter().zip(&traced.samples) {
                assert_eq!(ordinary.bodies, observed.bodies);
                let p = observed.impact.pulses.as_ref().unwrap();
                assert!(
                    (p.iter().map(|v| v.normal_impulse_ns).sum::<f64>() - ordinary.impact.right_ns)
                        .abs()
                        < 1e-12
                );
                assert!(p.iter().all(|v| v.jaw == "right"));
            }
        }
        Ok(())
    }
}
