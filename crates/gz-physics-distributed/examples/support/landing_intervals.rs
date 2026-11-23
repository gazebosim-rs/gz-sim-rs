// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resample complete A/B runs within each affinity and build-order group.
use super::{LAYOUTS, median};
use serde_json::{Value, json};
use std::collections::BTreeMap;

pub(super) struct Pair {
    pub touchdowns: [Vec<f64>; 2],
    pub window_change: f64,
}

pub(super) type Set = BTreeMap<(String, bool), Vec<Pair>>;
const DRAWS: usize = 4096;
const SEED: u64 = 0x35c1_ca11;

fn percentile(values: &mut [f64], numerator: usize, denominator: usize) -> f64 {
    let index = (values.len() * numerator).div_ceil(denominator) - 1;
    *values.select_nth_unstable_by(index, f64::total_cmp).1
}

fn intervals(groups: &[&[Pair]]) -> Option<Value> {
    if groups.is_empty() || groups.iter().any(|g| g.len() < 2) {
        return None;
    }
    let mut rng = fastrand::Rng::with_seed(SEED);
    let mut exact = Vec::with_capacity(DRAWS);
    let mut windows = Vec::with_capacity(DRAWS);
    for _ in 0..DRAWS {
        let mut samples = [vec![], vec![]];
        let mut changes = Vec::new();
        for group in groups {
            for _ in 0..group.len() {
                // Carry both arms, all touchdowns and the paired window statistic together.
                let pair = &group[rng.usize(..group.len())];
                for (arm, times) in samples.iter_mut().zip(&pair.touchdowns) {
                    arm.extend_from_slice(times);
                }
                changes.push(pair.window_change);
            }
        }
        let [prior, next] = samples.each_mut().map(|v| percentile(v, 95, 100));
        exact.push(100. * (next / prior - 1.));
        windows.push(median(changes));
    }
    let bounds = |v: &mut [f64]| [percentile(v, 1, 40), percentile(v, 39, 40)];
    Some(json!({"pooled_touchdown_p95_change_pct":bounds(&mut exact),
                "median_paired_window_p95_change_pct":bounds(&mut windows)}))
}

pub(super) fn report(sets: &[Set]) -> Value {
    let mut layouts = BTreeMap::new();
    for layout in LAYOUTS {
        let groups: Vec<_> = sets
            .iter()
            .flat_map(|set| {
                set.iter()
                    .filter(move |((name, _), _)| name == layout)
                    .map(|(_, pairs)| pairs.as_slice())
            })
            .collect();
        layouts.insert(
            layout,
            json!({
                "stratum_sizes":groups.iter().map(|g|g.len()).collect::<Vec<_>>(),
                "central_95_percent":intervals(&groups),
            }),
        );
    }
    json!({"draws":DRAWS,"seed":SEED,"change":"100 * (candidate / reference - 1)",
           "resampling_unit":"matched run within affinity and build order","layouts":layouts})
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resamples_matched_arms_together() {
        let trials = |value: f64| Pair {
            touchdowns: [vec![value, value * 2.], vec![value * 0.5, value]],
            window_change: -50.,
        };
        let groups = [
            vec![trials(1.), trials(2.)],
            vec![trials(10.), trials(400.)],
        ];
        assert!(intervals(&[&groups[0][..1]]).is_none());
        let result = intervals(&[&groups[0], &groups[1]]).unwrap();
        for metric in [
            "pooled_touchdown_p95_change_pct",
            "median_paired_window_p95_change_pct",
        ] {
            assert_eq!(result[metric], json!([-50., -50.]));
        }
    }

    #[test]
    fn keeps_stratum_sizes_when_treatment_effects_differ() {
        let pair = |next: f64| Pair {
            touchdowns: [vec![100.], vec![next]],
            window_change: next - 100.,
        };
        // Three slower pairs outweigh two faster ones in every resample.
        let slower = [pair(150.), pair(150.), pair(150.)];
        let faster = [pair(50.), pair(50.)];
        let result = intervals(&[&slower, &faster]).unwrap();
        assert_eq!(
            result["median_paired_window_p95_change_pct"],
            json!([50., 50.])
        );
    }
}
