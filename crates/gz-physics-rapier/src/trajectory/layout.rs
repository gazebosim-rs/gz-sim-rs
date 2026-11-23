// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Share an evaluation only for tracks with identical integer times and floating-point control bits.
use super::*;
use std::{
    collections::{HashMap, hash_map::DefaultHasher},
    hash::{Hash, Hasher},
    sync::Arc,
};
pub(super) struct Row {
    pub id: BodyId,
    pub handle: RigidBodyHandle,
    pub origin: Vector,
    pub group: usize,
}
pub(super) struct Layout {
    pub paths: Vec<TranslationSpline>,
    pub rows: Vec<Row>,
}
fn key(path: &TranslationSpline) -> u64 {
    let mut h = DefaultHasher::new();
    (path.end_policy() as u8).hash(&mut h);
    for w in path.waypoints() {
        w.time.hash(&mut h);
        w.displacement.map(f64::to_bits).hash(&mut h);
        w.velocity.map(f64::to_bits).hash(&mut h);
    }
    h.finish()
}
fn same(a: &TranslationSpline, b: &TranslationSpline) -> bool {
    a.end_policy() == b.end_policy()
        && a.waypoints().len() == b.waypoints().len()
        && a.waypoints().zip(b.waypoints()).all(|(u, v)| {
            u.time == v.time
                && u.displacement.map(f64::to_bits) == v.displacement.map(f64::to_bits)
                && u.velocity.map(f64::to_bits) == v.velocity.map(f64::to_bits)
        })
}
impl Curves {
    pub(super) fn layout(&self) -> &Arc<Layout> {
        self.layout.get_or_init(|| {
            let mut paths = Vec::new();
            let mut rows = Vec::with_capacity(self.tracks.len());
            let mut buckets: HashMap<u64, Vec<usize>> = HashMap::new();
            for (&id, t) in self.tracks.iter().filter(|(_, t)| t.active) {
                let bucket = buckets.entry(key(&t.path)).or_default();
                let group = bucket
                    .iter()
                    .copied()
                    .find(|&g| same(&paths[g], &t.path))
                    .unwrap_or_else(|| {
                        let g = paths.len();
                        paths.push(t.path.clone());
                        bucket.push(g);
                        g
                    });
                rows.push(Row {
                    id,
                    handle: t.handle,
                    origin: t.origin,
                    group,
                });
            }
            Arc::new(Layout { paths, rows })
        })
    }
    pub(super) fn invalidate_layout(&mut self) {
        self.layout.take();
        self.samples.take();
    }
    pub(in crate::trajectory) fn sample_groups(&self) -> &[TranslationSample] {
        self.samples.get_or_init(|| {
            self.layout()
                .paths
                .iter()
                .map(|p| p.sample(self.time))
                .collect()
        })
    }
    pub(crate) fn cursor(&self) -> Option<Cursor<'_>> {
        self.authored().then(|| Cursor {
            rows: &self.layout().rows,
            samples: self.sample_groups(),
            next: 0,
        })
    }
}
pub(crate) struct Cursor<'a> {
    rows: &'a [Row],
    samples: &'a [TranslationSample],
    next: usize,
}
impl Cursor<'_> {
    pub(crate) fn get(&mut self, id: BodyId) -> Option<(Vector, Vector)> {
        if self.rows.get(self.next).is_none_or(|r| r.id != id) {
            if self.rows.get(self.next).is_none_or(|r| id < r.id)
                && self
                    .next
                    .checked_sub(1)
                    .is_none_or(|i| self.rows[i].id < id)
            {
                return None;
            }
            self.next = self.rows.partition_point(|r| r.id < id);
        }
        let row = self.rows.get(self.next).filter(|r| r.id == id)?;
        self.next += 1;
        let s = &self.samples[row.group];
        Some((
            row.origin + Vector::from(s.displacement),
            Vector::from(s.velocity),
        ))
    }
}
