// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native gzweb archive of committed distributed states. The horizontal axis
//! is physical +X. The two colored strips and dividing line are display guides.
use gz_physics_distributed::{coordinator::Frame, physics::Body};
use serde_json::{Value, json};
use std::{error::Error, f64::consts::FRAC_1_SQRT_2 as S, path::Path};

const OWNER: [[f64; 3]; 2] = [[0.04, 0.80, 0.91], [1.0, 0.34, 0.04]];
pub const PHYSICAL_GAP: f64 = 12.;
const DISPLAY_GAP: f64 = 1.15;
const IDENTITY: [f64; 4] = [0., 0., 0., 1.];

fn visual(
    id: u64,
    name: &str,
    position: [f64; 3],
    rotation: [f64; 4],
    size: [f64; 3],
    rgb: [f64; 3],
) -> Value {
    json!({"id":id,"name":name,"translation":position,"rotation":rotation,
        "shape":{"kind":"box","size":size},"color":[rgb[0],rgb[1],rgb[2],1.]})
}

fn model(id: u64, name: &str, ids: &[u64]) -> Value {
    json!({"model_id":id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false})
}

fn view_position([x, y, z]: [f64; 3]) -> [f64; 3] {
    [y, -x, z]
}
fn view_rotation([x, y, z, w]: [f64; 4]) -> [f64; 4] {
    [S * (x + y), S * (y - x), S * (z - w), S * (w + z)]
}

fn local_point(body: &Body, offset: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = body.orientation;
    let [a, b, c] = offset;
    let rotated = [
        (1. - 2. * (y * y + z * z)) * a + 2. * (x * y - w * z) * b + 2. * (x * z + w * y) * c,
        2. * (x * y + w * z) * a + (1. - 2. * (x * x + z * z)) * b + 2. * (y * z - w * x) * c,
        2. * (x * z - w * y) * a + 2. * (y * z + w * x) * b + (1. - 2. * (x * x + y * y)) * c,
    ];
    view_position(std::array::from_fn(|i| body.position[i] + rotated[i]))
}

struct BodyVisual {
    id: u64,
    model: u64,
    name: String,
    vertical_shift: f64,
}

/// Fix the model IDs, materials and lane offsets from the first committed state.
/// Only poses change during replay, even when several bodies change owners.
fn body_visuals(frame: &Frame) -> Vec<BodyVisual> {
    let mut bodies: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.bodies)
        .collect();
    bodies.sort_unstable_by_key(|b| b.id);
    let pair = bodies.iter().filter(|b| b.position[2] == 0.).count() == 2;
    bodies
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let z = b.position[2];
            let (name, vertical_shift) = if z == PHYSICAL_GAP || z == -PHYSICAL_GAP {
                let lane = if z > 0. {
                    "source_bystander_at_plus12m"
                } else {
                    "destination_bystander_at_minus12m"
                };
                let unique_name =
                    if bodies.iter().filter(|other| other.position[2] == z).count() == 1 {
                        lane.to_owned()
                    } else {
                        format!("{lane}_{}", b.id)
                    };
                (unique_name, z.signum() * DISPLAY_GAP - z)
            } else {
                let name = match (pair, b.id) {
                    (true, 1) => "incoming".to_owned(),
                    (true, 2) => "resident".to_owned(),
                    _ => format!("body_{}", b.id),
                };
                (name, 0.)
            };
            BodyVisual {
                id: b.id,
                model: 6 + i as u64 * 16,
                name,
                vertical_shift,
            }
        })
        .collect()
}

fn make_frame(
    frame: &Frame,
    sequence: u64,
    partitions: bool,
    items: &[BodyVisual],
) -> Result<(Value, Value), Box<dyn Error>> {
    if frame
        .workers
        .iter()
        .map(|w| w.snapshot.bodies.len())
        .sum::<usize>()
        != items.len()
    {
        return Err("contact replay changed its body set".into());
    }
    let selective = items.iter().any(|v| v.vertical_shift != 0.);
    let (mut visuals, mut models) = if selective {
        // All bodies move along X. Translate entire bystander lanes for a readable
        // inset without changing trajectory shape, velocity, rotation or box size.
        let tracks = [
            (3, "contact_lane_z0m", 0.),
            (4, "source_lane_z_plus12m", DISPLAY_GAP),
            (5, "destination_lane_z_minus12m", -DISPLAY_GAP),
        ];
        let mut guides = Vec::new();
        for (id, name, z) in tracks {
            guides.push(visual(
                id,
                &format!("default::lanes::{name}"),
                [0., 0., z - 0.344],
                IDENTITY,
                [1.02, 5.6, 0.07],
                [0.27, 0.30, 0.32],
            ));
        }
        (guides, vec![model(2, "lanes", &[3, 4, 5])])
    } else if partitions {
        (
            vec![
                visual(
                    3,
                    "default::partitions::worker_0",
                    [0., 1.4, -0.344],
                    IDENTITY,
                    [1.15, 2.8, 0.07],
                    [0.14, 0.34, 0.39],
                ),
                visual(
                    4,
                    "default::partitions::worker_1",
                    [0., -1.4, -0.344],
                    IDENTITY,
                    [1.15, 2.8, 0.07],
                    [0.43, 0.27, 0.18],
                ),
                visual(
                    5,
                    "default::partitions::handoff_plane",
                    [0., 0., -0.304],
                    IDENTITY,
                    [1.15, 0.045, 0.010],
                    [0.92, 0.92, 0.92],
                ),
            ],
            vec![model(2, "partitions", &[3, 4, 5])],
        )
    } else {
        // The whole contact island changes owner in place, independently of position.
        (
            vec![visual(
                3,
                "default::datum::plate",
                [0., 0., -0.344],
                IDENTITY,
                [1.15, 5.6, 0.07],
                [0.27, 0.30, 0.32],
            )],
            vec![model(2, "datum", &[3])],
        )
    };
    for item in items {
        let body_id = item.id;
        let model_id = item.model;
        let name = &item.name;
        let (owner, body) = frame.body(body_id).ok_or("missing replay body")?;
        let mut display = body.clone();
        display.position[2] += item.vertical_shift;
        let body = &display;
        let q = view_rotation(body.orientation);
        let mut ids = Vec::new();
        let mut add = |part: &str, position: [f64; 3], size: [f64; 3], color: [f64; 3]| {
            let id = model_id + 1 + ids.len() as u64;
            visuals.push(visual(
                id,
                &format!("default::{name}::{part}"),
                position,
                q,
                size,
                color,
            ));
            ids.push(id);
        };
        add(
            "core",
            view_position(body.position),
            body.size,
            [0.28, 0.30, 0.32],
        );
        // Scene identity and material definitions stay immutable during playback.
        // The owner's six panels cover the solid core. The other six panels
        // remain enclosed inside it. Only poses change at the ownership barrier.
        for (partition, color) in OWNER.iter().enumerate() {
            // Dense scenes use one colored cap per owner to fit the archive's
            // 64 KiB frame size. The inactive cap stays inside the solid body.
            if items.len() > 12 {
                let z = if partition == usize::from(owner) {
                    body.size[2] * 0.5
                } else {
                    0.
                };
                add(
                    &format!("owner_{partition}_cap"),
                    local_point(body, [0., 0., z]),
                    [body.size[0] * 0.9, body.size[1] * 0.9, body.size[2] * 0.32],
                    *color,
                );
                continue;
            }
            for axis in 0..3 {
                let mut size = body.size.map(|s| s * 0.995);
                let thickness = 0.001;
                size[axis] = thickness;
                for (face, sign) in [("minus", -1.), ("plus", 1.)] {
                    let mut offset = [0.; 3];
                    if partition == usize::from(owner) {
                        offset[axis] = sign * (body.size[axis] / 2. + thickness);
                    }
                    add(
                        &format!("owner_{partition}_{axis}_{face}"),
                        local_point(body, offset),
                        size,
                        *color,
                    );
                }
            }
        }
        if body_id == 1 {
            add(
                "identity_stripe",
                local_point(body, [-body.size[0] * 0.2, 0., 0.]),
                [
                    body.size[0] * 0.15,
                    body.size[1] * 1.012,
                    body.size[2] * 1.012,
                ],
                [0.98, 0.98, 0.98],
            );
        }
        models.push(model(model_id, name, &ids));
    }
    let ns = u64::try_from(frame.step().duration().as_nanos())?
        .checked_mul(frame.tick)
        .ok_or("replay clock overflow")?;
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,
        "sequence":sequence,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,
        "paused":false,"visuals":visuals});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,
        "scene_sequence":sequence,"models":models,"templates":[]});
    Ok((scene, catalog))
}

/// A committed physical frame, optionally accompanied by measured display guides.
pub trait Sample {
    fn committed(&self) -> &Frame;
    fn guides(&self, _visuals: &mut Vec<Value>, _models: &mut Vec<Value>) {}
}

impl Sample for Frame {
    fn committed(&self) -> &Frame {
        self
    }
}

pub fn write(path: &Path, frames: &[impl Sample]) -> Result<(), Box<dyn Error>> {
    let first = frames
        .first()
        .ok_or("cannot record an empty handoff")?
        .committed();
    let items = body_visuals(first);
    let partitions = first.workers.iter().all(|w| !w.snapshot.bodies.is_empty());
    let samples = frames
        .iter()
        .enumerate()
        .map(|(i, sample)| {
            let (mut scene, mut catalog) =
                make_frame(sample.committed(), i as u64 + 1, partitions, &items)?;
            sample.guides(
                scene["visuals"]
                    .as_array_mut()
                    .expect("constructed visuals"),
                catalog["models"]
                    .as_array_mut()
                    .expect("constructed models"),
            );
            Ok((scene.to_string(), catalog.to_string()))
        })
        .collect::<Result<Vec<_>, Box<dyn Error>>>()?;
    gz_sim_cli::export_archive(path, Default::default(), samples)?;
    Ok(())
}
