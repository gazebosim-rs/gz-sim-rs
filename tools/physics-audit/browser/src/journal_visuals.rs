// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare aligned SDF visuals with a separate local CPU journal, including paused model removal.
use super::{Result, same, vector};
use serde_json::Value;
use std::{collections::BTreeSet, io::BufRead};

pub(super) fn verify(journal: impl BufRead, frames: &[Value]) -> Result<()> {
    let mut lines = journal.lines();
    let profile: Value = serde_json::from_str(&lines.next().ok_or("missing journal profile")??)?;
    let names = profile["observed_entities"]
        .as_object()
        .ok_or("missing journal link names")?;
    // Advance only on the journal clock. A paused edit can yield multiple physical states
    // at the same time, distinguished by the set of live links and their stable identities.
    let mut cursor = 0;
    for line in lines {
        let state: Value = serde_json::from_str(&line?)?;
        let t = state["time_ns"].as_u64().ok_or("missing journal time")?;
        let links = state["observed_links"]
            .as_array()
            .ok_or("missing journal links")?;
        while let Some(frame) = frames.get(cursor) {
            let clock = frame["ns"].as_u64().ok_or("missing archive time")?;
            if clock < t {
                return Err(format!(
                    "no journal body state matched frame {} at {clock} ns",
                    cursor + 1
                )
                .into());
            }
            if clock > t {
                break;
            }
            let visuals = frame["scene"]["visuals"]
                .as_array()
                .ok_or("missing visuals")?;
            if visuals.len() != links.len() {
                break;
            }
            let mut matched = BTreeSet::new();
            for visual in visuals {
                let (link, _) = visual["name"]
                    .as_str()
                    .and_then(|s| s.rsplit_once("::"))
                    .ok_or("unscoped visual name")?;
                let id = names
                    .get(link)
                    .and_then(Value::as_u64)
                    .ok_or("visual link not in journal profile")?;
                if let Some(body) = links.iter().find(|body| body["id"] == id)
                    && same(&visual["translation"], vector::<3>(&body["translation_m"])?)?
                    && same(&visual["rotation"], vector::<4>(&body["rotation_xyzw"])?)?
                {
                    matched.insert(id);
                }
            }
            if matched.len() != links.len() {
                break;
            }
            cursor += 1;
        }
        if cursor == frames.len() {
            return Ok(());
        }
    }
    Err("journal ended before a matching frame was found".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn same_time_removal_must_appear_in_order_in_the_independent_journal() {
        let pose = [0., 0., 0.];
        let q = [0., 0., 0., 1.];
        let visual = |name| json!({"name":name, "translation":pose, "rotation":q});
        let link = |id| json!({"id":id, "translation_m":pose,"rotation_xyzw":q});
        let frame = |visuals: Vec<Value>| json!({"ns":0,"scene":{"visuals":visuals}});
        let a = visual("w::left::pad::box");
        let b = visual("w::right::pad::box");
        let both = frame(vec![a.clone(), b]);
        let one = frame(vec![a]);
        let journal = format!(
            "{}\n{}\n{}\n",
            json!({"observed_entities":{"w::left::pad":1,"w::right::pad":2}}),
            json!({"time_ns":0,"observed_links":[link(1),link(2)]}),
            json!({"time_ns":0,"observed_links":[link(1)]})
        );
        verify(journal.as_bytes(), &[both.clone(), one.clone()]).unwrap();
        assert!(verify(journal.as_bytes(), &[one, both]).is_err());
        let wrong = frame(vec![
            json!({"name":"w::left::pad::box","translation":[0.1,0.,0.],"rotation":q}),
        ]);
        assert!(verify(journal.as_bytes(), &[wrong]).is_err());
    }
}
