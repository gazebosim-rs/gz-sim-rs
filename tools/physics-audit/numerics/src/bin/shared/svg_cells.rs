// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read the six-pane physical lattice directly from emitted SVG coordinates.
use super::{Key, Result};
use std::{collections::BTreeSet, fs, path::Path};
pub(super) fn attr<'a>(line: &'a str, key: &str) -> Result<&'a str> {
    let text = line
        .split_once(&format!(" {key}=\""))
        .ok_or("SVG attribute")?
        .1;
    Ok(text.split_once('"').ok_or("SVG attribute end")?.0)
}
pub(super) fn color(v: f64, limit: f64, candidate: bool) -> String {
    let start = [249., 248., 241.];
    let end = if !candidate {
        [32., 75., 135.]
    } else if v < 0. {
        [0., 117., 119.]
    } else {
        [174., 57., 16.]
    };
    let mix = (v.abs() / limit).min(1.0);
    let c: [u8; 3] = std::array::from_fn(|i| (start[i] * (1. - mix) + end[i] * mix).round() as u8);
    format!("#{:02X}{:02X}{:02X}", c[0], c[1], c[2])
}

pub(super) fn check(svg: &Path, expected_color: impl Fn(Key) -> String) -> Result<()> {
    let xml = fs::read_to_string(svg)?;
    let mut tiles = vec![];
    let mut x = BTreeSet::new();
    let mut y = BTreeSet::new();
    for l in xml.lines().filter(|l| l.starts_with("<rect ")) {
        let w = attr(l, "width")?.parse::<u16>()?;
        let h = attr(l, "height")?.parse::<u16>()?;
        if !(11..=13).contains(&w) || !(9..=14).contains(&h) {
            continue;
        }
        let px = attr(l, "x")?.parse::<u16>()?;
        let py = attr(l, "y")?.parse::<u16>()?;
        tiles.push((px, py, attr(l, "fill")?));
        x.insert(px);
        y.insert(py);
    }
    assert_eq!((tiles.len(), x.len(), y.len()), (3750, 75, 50));
    let x: Vec<_> = x.into_iter().collect();
    let y: Vec<_> = y.into_iter().collect();
    let mut emitted = BTreeSet::new();
    for (px, py, actual) in tiles {
        let col = x.binary_search(&px).unwrap();
        let row = y.binary_search(&py).unwrap();
        let offset = 165 + 5 * (col % 25);
        let speed = 76 + 2 * (24 - row % 25);
        let holds = row >= 25;
        let stroke = col / 25 + 1;
        let key = (offset, speed, holds, stroke);
        assert!(emitted.insert(key));
        let expected = expected_color(key);
        if actual != expected {
            return Err(format!("{svg:?} cell {key:?}: expected {expected}, got {actual}").into());
        }
    }
    Ok(())
}
