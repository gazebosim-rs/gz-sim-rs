// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use serde_json::{Value as V, json};
use std::collections::BTreeMap;
pub(super) fn arr(v: &V) -> &[V] {
    v.as_array().unwrap()
}
pub(super) fn num(v: &V) -> f64 {
    v.as_f64().unwrap()
}
pub(super) fn int(v: &V) -> u64 {
    v.as_u64().unwrap()
}
pub(super) fn vec(v: &V) -> DVec3 {
    DVec3::from_array(serde_json::from_value(v.clone()).unwrap())
}
pub(super) fn quat(v: &V) -> DQuat {
    DQuat::from_array(serde_json::from_value(v.clone()).unwrap())
}
#[derive(Default)]
pub(super) struct Audit {
    counts: BTreeMap<String, usize>,
    values: BTreeMap<String, BTreeMap<String, Vec<f64>>>,
}
impl Audit {
    pub(super) fn count(&mut self, key: &str) {
        *self.counts.entry(key.into()).or_default() += 1;
    }
    pub(super) fn sample(&mut self, g: &str, key: &str, x: f64) {
        assert!(x.is_finite());
        self.values
            .entry(g.into())
            .or_default()
            .entry(key.into())
            .or_default()
            .push(x);
    }
    pub(super) fn report(self) -> V {
        let stats=self.values.into_iter().map(|(g,kv)|(g,kv.into_iter().map(|(k,mut v)|{
            v.sort_by(f64::total_cmp);let len=v.len();(k,json!({"n":len,"min":v[0],"p05":v[(5*len).div_ceil(100)-1],"median":v[len.div_ceil(2)-1],"p95":v[(95*len).div_ceil(100)-1],"max":v[len-1]}))
        }).collect::<BTreeMap<_,_>>())).collect::<BTreeMap<_,_>>();
        json!({"counts":self.counts,"statistics":stats})
    }
}
