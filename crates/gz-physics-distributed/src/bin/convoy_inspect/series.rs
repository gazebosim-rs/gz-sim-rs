// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    Result,
    compare::{Classification, Comparison, PhysicalErrors},
};
use gz_physics_distributed::multi_sphere::inspect_published_path;
use serde::{Deserialize, Serialize};
use std::{
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Pair {
    reference: PathBuf,
    candidate: PathBuf,
}

#[derive(Serialize)]
struct Difference {
    row: usize,
    reference: PathBuf,
    candidate: PathBuf,
    comparison: serde_json::Value,
}

#[derive(Serialize)]
struct Sample {
    row: usize,
    reference_clock: (u64, u64),
    candidate_clock: (u64, u64),
    classification: Classification,
    physical_errors: Option<PhysicalErrors>,
}

#[derive(Default, Serialize)]
pub(super) struct Series {
    rows_checked: usize,
    samples: Vec<Sample>,
    identical_archives: usize,
    equal_physical_observations: usize,
    incomparable_physical_observations: usize,
    first_archive_difference: Option<Difference>,
    first_logical_difference: Option<Difference>,
    first_physical_difference: Option<Difference>,
}

/// Manifest order breaks ties between multiple publications at the same cut.
/// Refuse a backwards clock independently on either lane. Validate all rows,
/// even once the first differences have been found.
pub(super) fn compare_series(path: &Path) -> Result<Series> {
    let base = path.parent().unwrap_or(Path::new("."));
    let pairs: Vec<Pair> = serde_json::from_reader(BufReader::new(File::open(path)?))?;
    if pairs.is_empty() {
        return Err("empty checkpoint comparison series".into());
    }
    let mut report = Series::default();
    let mut previous = None;
    for (i, pair) in pairs.into_iter().enumerate() {
        let row = i + 1;
        let inspect = |p: &Path| {
            inspect_published_path(&base.join(p))
                .map_err(|e| format!("row {row}, {}: {e}", p.display()))
        };
        let a = inspect(&pair.reference)?;
        let b = inspect(&pair.candidate)?;
        let c = Comparison::new(&a, &b)?;
        let left = (c.left.epoch, c.left.tick);
        let right = (c.right.epoch, c.right.tick);
        if previous.is_some_and(|(l, r)| left < l || right < r) {
            return Err(format!("backwards checkpoint clock at row {row}").into());
        }
        previous = Some((left, right));
        let finding = || -> Result<Difference> {
            Ok(Difference {
                row,
                reference: pair.reference.clone(),
                candidate: pair.candidate.clone(),
                comparison: serde_json::to_value(&c)?,
            })
        };
        report.rows_checked += 1;
        report.identical_archives += usize::from(c.equal.archive);
        report.equal_physical_observations +=
            usize::from(c.equal.physical_observations == Some(true));
        report.incomparable_physical_observations +=
            usize::from(c.equal.physical_observations.is_none());
        if !c.equal.archive && report.first_archive_difference.is_none() {
            report.first_archive_difference = Some(finding()?);
        }
        if (!c.equal.recipe
            || !c.equal.pending_input
            || c.equal.physical_observations != Some(true))
            && report.first_logical_difference.is_none()
        {
            report.first_logical_difference = Some(finding()?);
        }
        if c.equal.physical_observations == Some(false)
            && report.first_physical_difference.is_none()
        {
            report.first_physical_difference = Some(finding()?);
        }
        report.samples.push(Sample {
            row,
            reference_clock: left,
            candidate_clock: right,
            classification: c.classification,
            physical_errors: c.physical_errors,
        });
    }
    Ok(report)
}
