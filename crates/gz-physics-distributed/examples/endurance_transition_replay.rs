// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Present actual late contact transitions using a full-scene overview and a rigidly scaled detail.
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport, Primitive};
use primitive_replay_view::View;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::PathBuf,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// Verified sampled native frames and measured transition metadata (encounter.jsonl, cycles.jsonl).
    #[arg(long)]
    source: PathBuf,
    /// New directory for native replay and original clock/geometry mapping.
    directory: PathBuf,
}
#[derive(Deserialize)]
struct Frame {
    tick: u64,
    bodies: Vec<Body>,
    owners: Vec<u8>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
}
#[derive(Serialize, Deserialize)]
struct Contact {
    ids: Vec<u64>,
    first_tick: u64,
    first_impulse_n_s: f64,
    first_positive: (u64, f64),
    first_nonpositive_after_load: u64,
    first_absent: u64,
}
impl Contact {
    fn receipt(&self, f: &Frame) -> Result<Option<f64>> {
        match self.ids.as_slice() {
            [id] => Ok(f
                .floors
                .iter()
                .find(|s| s.body == *id)
                .map(|s| s.normal_impulse)),
            [a, b] => Ok(f
                .pairs
                .iter()
                .find(|p| (p.a, p.b) == (*a, *b))
                .map(|p| p.normal_impulse)),
            _ => Err("unexpected contact shape".into()),
        }
    }
    fn qualify(&self, frames: &[Frame]) -> Result<()> {
        let at = |tick| -> Result<&Frame> {
            frames
                .iter()
                .find(|f| f.tick == tick)
                .ok_or_else(|| format!("missing native contact landmark {tick}").into())
        };
        let prior = self.receipt(at(self.first_tick - 1)?)?;
        if (self.ids.len() == 2 && prior.is_some()) || prior.is_some_and(|v| v > 0.) {
            return Err("contact already loaded before its reported transition".into());
        }
        for (tick, expected) in [
            (self.first_tick, Some(self.first_impulse_n_s)),
            (self.first_positive.0, Some(self.first_positive.1)),
            (self.first_nonpositive_after_load, Some(0.)),
            (self.first_absent, None),
        ] {
            if self.receipt(at(tick)?)? != expected {
                return Err(format!("incorrect receipt at {tick}").into());
            }
        }
        Ok(())
    }
}
#[derive(Serialize)]
struct Panel {
    ids: Vec<u64>,
    center: [f64; 2],
    offset_y: f64,
    scale: f64,
}
impl Panel {
    fn fit(ids: Vec<u64>, frames: &[Frame], dy: f64, max_scale: f64) -> Result<Self> {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for f in frames {
            for &id in &ids {
                let b = f
                    .bodies
                    .get(id as usize - 1)
                    .ok_or("unknown panel member")?;
                for axis in 0..2 {
                    let radius = b.size[axis] / 2.;
                    lo[axis] = lo[axis].min(b.position[axis] - radius);
                    hi[axis] = hi[axis].max(b.position[axis] + radius);
                }
            }
        }
        let scale = (23. / (hi[0] - lo[0]))
            .min(8. / (hi[1] - lo[1]))
            .min(max_scale);
        if !scale.is_finite() || scale <= 0. {
            return Err("invalid viewport".into());
        }
        Ok(Self {
            ids,
            center: [(hi[0] + lo[0]) / 2., (hi[1] + lo[1]) / 2.],
            offset_y: dy,
            scale,
        })
    }
    fn pos(&self, p: [f64; 3]) -> [f64; 3] {
        [
            (p[0] - self.center[0]) * self.scale,
            (p[1] - self.center[1]) * self.scale + self.offset_y,
            p[2] * self.scale,
        ]
    }
    fn draw(
        &self,
        view: &mut View,
        f: &Frame,
        tag: &str,
        id_offset: u64,
        focus: Option<&Contact>,
        owner_generations: Option<&[u64]>,
    ) -> Result<()> {
        let palette = [
            [0.14, 0.48, 0.86, 1.],
            [0.98, 0.5, 0.11, 1.],
            [0.62, 0.30, 0.76, 1.],
            [0.09, 0.65, 0.45, 1.],
        ];
        for &id in &self.ids {
            let body = &f.bodies[id as usize - 1];
            let owner = f.owners[id as usize - 1];
            let color = if let Some(c) = focus {
                match c.ids.iter().position(|v| *v == id) {
                    Some(0) => [0.97, 0.54, 0.14, 1.],
                    Some(_) => [0.12, 0.68, 0.78, 1.],
                    None => [0.48, 0.54, 0.60, 1.],
                }
            } else {
                *palette.get(owner as usize).ok_or("bad owner")?
            };
            let size = body.size.map(|v| v * self.scale);
            let shape = match body.primitive {
                Primitive::Box => json!({"kind":"box","size":size}),
                Primitive::Sphere => json!({"kind":"sphere","radius":size[0]/2.}),
                _ => return Err("unexpected shape".into()),
            };
            view.add(
                id_offset + 2 * id + 128 * owner_generations.map_or(0, |g| g[id as usize - 1]),
                &format!("{tag}_body_{id}"),
                self.pos(body.position),
                body.orientation,
                shape,
                color,
            );
        }
        Ok(())
    }
}
fn anchor(f: &Frame, event: &Contact) -> [f64; 3] {
    let mut p = [0.; 3];
    for &id in &event.ids {
        let b = &f.bodies[id as usize - 1];
        for (a, c) in p.iter_mut().zip(b.position) {
            *a += c / event.ids.len() as f64
        }
    }
    p
}
fn detail(frames: &[Frame], event: &Contact) -> Result<Panel> {
    let hit = frames
        .iter()
        .find(|f| f.tick == event.first_tick)
        .ok_or("missing event")?;
    let p = anchor(hit, event);
    // Include nearby native shapes around the event once for the complete window, with a fixed
    // translation and uniform scale. Distances and motion remain coherent across assembly boundaries.
    let ids = hit
        .bodies
        .iter()
        .filter(|b| (0..2).all(|a| (b.position[a] - p[a]).abs() - b.size[a] / 2. <= 2.25))
        .map(|b| b.id)
        .collect();
    Panel::fit(ids, frames, -5.2, 2.5)
}
#[derive(Serialize)]
struct Index {
    source_tick: u64,
    playback_tick: u64,
    segment: usize,
    normal_impulse_n_s: Option<f64>,
}
fn main() -> Result<()> {
    let args = Args::parse();
    let path = args.source.join("encounter.jsonl");
    let source_hash = format!("{:x}", Sha256::digest(fs::read(&path)?));
    let frames: Vec<Frame> = BufReader::new(File::open(path)?)
        .lines()
        .map(|line| Ok(serde_json::from_str(&line?)?))
        .collect::<Result<_>>()?;
    let ledger_path = args.source.join("cycles.jsonl");
    let last = BufReader::new(File::open(&ledger_path)?)
        .lines()
        .last()
        .ok_or("missing event ledger")??;
    let row: Value = serde_json::from_str(&last)?;
    let ev: [Contact; 2] = ["ground", "cross_assembly"]
        .map(|k| serde_json::from_value(row["encounters"][k].clone()))
        .into_iter()
        .collect::<std::result::Result<Vec<_>, _>>()?
        .try_into()
        .map_err(|_| "missing event")?;
    if frames.iter().any(|f| {
        f.bodies.len() != 62
            || f.owners.len() != 62
            || f.bodies
                .iter()
                .enumerate()
                .any(|(i, b)| b.id as usize != i + 1)
    }) || frames.windows(2).any(|w| w[0].tick >= w[1].tick)
    {
        return Err("invalid native samples".into());
    }
    let gaps: Vec<_> = frames
        .windows(2)
        .enumerate()
        .filter(|(_, w)| w[1].tick - w[0].tick > 5)
        .map(|(i, _)| i + 1)
        .collect();
    let [split] = gaps[..] else {
        return Err("expected two contiguous sampled native windows".into());
    };
    let windows = [&frames[..split], &frames[split..]];
    let overview = Panel::fit((1..=62).collect(), &frames, 5.2, 1.)?;
    let mut close = vec![];
    for (window, event) in windows.iter().zip(&ev) {
        event.qualify(window)?;
        close.push(detail(window, event)?);
    }
    let mut stream = vec![];
    let mut index = vec![];
    let mut playback_start = 0;
    let mut colors = frames[0].owners.clone();
    let mut generations = vec![0u64; 62];
    for (seg, window) in windows.into_iter().enumerate() {
        let contact = &ev[seg];
        let tag = ["ground_detail", "pair_detail"][seg];
        let panel = &close[seg];
        for f in window {
            let playback = playback_start + f.tick - window[0].tick;
            let impulse = contact.receipt(f)?;
            let mut view = View::default();
            view.cuboid(
                2,
                "display_floor",
                [0., 0., -0.055],
                [26., 22., 0.11],
                [0.29, 0.34, 0.40, 1.],
            );
            for ((old, current), generation) in
                colors.iter_mut().zip(&f.owners).zip(&mut generations)
            {
                if old != current {
                    *old = *current;
                    *generation += 1;
                }
            }
            overview.draw(&mut view, f, "overview", 10_000, None, Some(&generations))?;
            panel.draw(
                &mut view,
                f,
                tag,
                300 + seg as u64 * 180,
                Some(contact),
                None,
            )?;
            let status = match impulse {
                None => "absent",
                Some(0.) => "zero",
                Some(_) => "loaded",
            };
            let mut at = panel.pos(anchor(f, contact));
            at[2] += 1.55;
            let ids = contact
                .ids
                .iter()
                .map(u64::to_string)
                .collect::<Vec<_>>()
                .join("_");
            let label = format!(
                "native_tick_{}_{}_{ids}_{status}_{:.6}_N_s",
                f.tick,
                tag,
                impulse.unwrap_or(0.)
            );
            let color = match impulse {
                None => [0.43, 0.47, 0.51, 1.],
                Some(0.) => [0.94, 0.74, 0.2, 1.],
                Some(_) => [0.1, 0.77, 0.42, 1.],
            };
            view.cuboid(
                2000 + 2 * index.len() as u64,
                &label,
                at,
                [0.36, 0.36, 0.18],
                color,
            );
            index.push(Index {
                source_tick: f.tick,
                playback_tick: playback,
                segment: seg,
                normal_impulse_n_s: impulse,
            });
            stream.push(view.finish(playback));
        }
        playback_start += window.last().unwrap().tick - window[0].tick + 1;
    }
    fs::create_dir(&args.directory)?;
    gz_sim_cli::export_archive(&args.directory.join("replay"), Default::default(), stream)?;
    fs::write(
        args.directory.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"source_sha256":source_hash,"ledger_row_sha256":format!("{:x}",Sha256::digest(last.as_bytes())),"overview":overview,"detail":close,"contacts":ev,"frames":index}),
        )?,
    )?;
    println!(
        "Captured {} native samples, {} ground-detail bodies and {} cross-assembly-detail bodies.",
        index.len(),
        close[0].ids.len(),
        close[1].ids.len()
    );
    Ok(())
}
