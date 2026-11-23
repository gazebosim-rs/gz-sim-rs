// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frictionless full-sole touchdown after a two-vertex bearing.
#[path = "face/face_bearing.rs"]
pub(in super::super::super::super::super) mod bearing;
use super::*;
use gz_physics::{RectangularNormalContact, RectangularNormalResponse};
#[path = "face/law.rs"]
mod mechanics;
#[path = "face/native.rs"]
mod observations;
#[path = "face/registered.rs"]
mod registered;
#[path = "face/train.rs"]
pub(in super::super::super::super::super) mod train;
#[path = "face/transfer.rs"]
pub(in super::super::super::super::super) mod transfer;
use mechanics::{Law, Mode, Sole};
fn manufactured(mut a: Arc, yaw: f64, variant: &str, pressure: DVec2, load: f64) -> Result<Sole> {
    let q = DQuat::from_rotation_z(yaw);
    let r = a.shape.vertices;
    let cp = q * DVec3::new(0.18 * pressure.x, 0.10 * pressure.y, r[0].z);
    let (ux, uy, site) = match variant {
        "face" => (0., 0., 0),
        "edge_x" => (0., -0.75, 0),
        "edge_y" => (0.6, 0., 0),
        "vertex" => (0.6, -0.75, 0),
        _ => unreachable!(),
    };
    let w = q * DVec3::new(ux, uy, 1.4);
    a.start.motion = Motion { q, w };
    a.start.c = DVec3::new(0., 0., -r[0].z);
    a.start.v = DVec3::new(0.18, -0.08, -w.cross(q * r[site]).z);
    let iw = DMat3::from_quat(q) * a.shape.inertia * DMat3::from_quat(q).transpose();
    a.start.v -= load / a.shape.mass * DVec3::Z;
    a.start.motion.w -= iw.inverse() * cp.cross(load * DVec3::Z);
    Ok(Sole {
        shape: a.shape,
        incoming: a.start,
    })
}
fn compare(s: Sole) -> Result<(Value, Law, RectangularNormalResponse)> {
    let law = s.solve()?;
    let matrix = s.mobility();
    let u = s.incoming_normal();
    let response = RectangularNormalContact {
        inverse_mass: matrix,
        free_normal: u,
    }
    .solve()?;
    let post = (0..4)
        .map(|k| (response.normal[k] - law.post[k]).abs())
        .fold(0., f64::max);
    let center = match (response.normalized_center, law.center) {
        (Some(a), Some(b)) => DVec2::from(a).distance(b),
        (None, None) => 0.,
        _ => return Err("pressure center admission mismatch".into()),
    };
    let jerr = (response.resultant - law.total).abs();
    let range = response.equivalents.shift_range;
    let mut extremes = vec![];
    let mut err = 0f64;
    for shift in [range[0], 0.5 * range[0] + 0.5 * range[1], range[1]] {
        let load = response.equivalents.at(shift).unwrap();
        let net = load.iter().sum::<f64>() * DVec3::Z;
        let h = s
            .arms()
            .into_iter()
            .zip(load)
            .map(|(r, j)| r.cross(j * DVec3::Z))
            .sum::<DVec3>();
        let v = s.incoming.v + net / s.shape.mass;
        let w = s.incoming.motion.w + s.iw().inverse() * h;
        err = err
            .max(v.distance(law.after.v))
            .max(w.distance(law.after.motion.w));
        if load.iter().any(|j| *j < -1e-14) || (0..4).any(|k| load[k] * law.post[k] > 1e-10) {
            return Err("invalid normal corner representation".into());
        }
        extremes.push(json!(load));
    }
    if post.max(center).max(jerr).max(err) > 2e-10 {
        return Err(format!("pressure mismatch mode {:?} post {post:e} center {center:e}, net {jerr:e}, twist {err:e}",law.mode).into());
    }
    Ok((
        json!({"mode":format!("{:?}",law.mode),"resultant_ns":response.resultant,"geometry_resultant_ns":law.total,"center_normalized":response.normalized_center,"geometry_center_normalized":law.center.map(|p|p.to_array()),"free_m_s":u,"post_m_s":response.normal,"inverse_mass":matrix,"possible_corner_normals_ns":extremes,"null_width_ns":range[1]-range[0],"max_twist_error":err,"max_post_error_m_s":post,"kinetic_loss_j":law.loss,"state":state_wire(s.incoming),"after":state_wire(law.after)}),
        law,
        response,
    ))
}
pub(in super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    field(out, false)
}
pub(in super::super::super::super::super) fn run_registered(out: &Path) -> Result<()> {
    field(out, true)
}
fn field(out: &Path, production: bool) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut field = vec![];
    let mut controls = vec![];
    let mut demo = None;
    let mut interior = None;
    for active in [0, 1] {
        for pitch in [-0.18, 0.18] {
            for opening in [0.09, 0.3, 0.5] {
                for rate in [-3., -0.6, 0.6, 3.] {
                    for yaw in [0., 0.67] {
                        let (i, a, _) = fixture(Case {
                            active,
                            pitch,
                            yaw,
                            opening,
                            pitch_rate: rate,
                        });
                        if solve(i.edge).is_err() {
                            continue;
                        }
                        let e = a.next(0.000125, 0.4)?;
                        let End::Contact(new) = e.end else {
                            continue;
                        };
                        for variant in ["natural", "triaxial"] {
                            let mut shape = a.shape;
                            if variant == "triaxial" {
                                shape.inertia = DMat3::from_cols(
                                    DVec3::new(0.034, 0.004, -0.003),
                                    DVec3::new(0.004, 0.051, 0.006),
                                    DVec3::new(-0.003, 0.006, 0.055),
                                );
                            }
                            let j = shape.recollide(e.state, a.sites.first(), new)?;
                            let Sites::Two(keys) = j.new_sites else {
                                continue;
                            };
                            let branch = Arc {
                                start: j.after,
                                shape,
                                sites: j.new_sites,
                            };
                            let third = branch.next(0.0000625, 0.35)?;
                            if (third.state.motion.q * DVec3::Z - DVec3::Z).length() > 1e-9 {
                                continue;
                            }
                            let sole = Sole {
                                shape,
                                incoming: third.state,
                            };
                            let (mut r, l, _) = compare(sole)?;
                            r["native"] = if production {
                                registered::cases(i, sole)?
                            } else {
                                json!(observations::isolated(i, sole, l)?)
                            };
                            if demo.is_none()
                                && variant == "natural"
                                && yaw > 0.
                                && matches!(l.mode, Mode::Edge(_, _))
                            {
                                demo = Some((i, sole, l));
                            }
                            field.push(json!({"active":active,"pitch":pitch,"opening":opening,"rate":rate,"yaw":yaw,"inertia":variant,"original_bearing":keys,"second_time_s":e.time,"third_time_s":e.time+third.time,"result":r}));
                        }
                    }
                }
            }
        }
    }
    let (manufactured_input, base, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        yaw: 0.67,
        opening: 0.3,
        pitch_rate: 0.6,
    });
    for density in [0.4, 1., 2.3] {
        for yaw in [0., 0.38, 1.71] {
            for y in [-0.67, 0.17, 0.61] {
                for x in [-0.83, 0.28, 0.89] {
                    let mut shape = base.shape;
                    shape.mass *= density;
                    shape.inertia = DMat3::from_cols(
                        DVec3::new(0.034, 0.004, -0.003),
                        DVec3::new(0.004, 0.051, 0.006),
                        DVec3::new(-0.003, 0.006, 0.055),
                    ) * density;
                    for (mode, cp) in [
                        ("face", DVec2::new(x, y)),
                        ("edge_x", DVec2::new(-1., y)),
                        ("edge_y", DVec2::new(x, -1.)),
                        ("vertex", DVec2::new(-1., -1.)),
                    ] {
                        let input =
                            manufactured(Arc { shape, ..base }, yaw, mode, cp, 0.87 * density)?;
                        let (mut data, l, _) = compare(input)?;
                        data["native"] = if production {
                            registered::cases(manufactured_input, input)?
                        } else {
                            json!(observations::isolated(manufactured_input, input, l)?)
                        };
                        if production
                            && density == 1.
                            && yaw == 0.38
                            && x == 0.28
                            && y == 0.17
                            && mode == "face"
                        {
                            interior = Some((manufactured_input, input, l));
                        }
                        if l.center.unwrap().distance(cp) > 1e-8 {
                            return Err(
                                "independent normalization mismatch on manufactured COP".into()
                            );
                        }
                        controls.push(json!({"density":density,"yaw":yaw,"mode":mode,"prescribed_center":cp.to_array(),"result":data}));
                    }
                }
            }
        }
    }
    fs::write(out.join("physical.json"), serde_json::to_vec(&field)?)?;
    fs::write(
        out.join("manufactured.json"),
        serde_json::to_vec(&controls)?,
    )?;
    let (i, sole, law) = demo.ok_or("no observed co-bearing to opposite edge transfer")?;
    if production {
        let (i, sole, l) =
            interior.ok_or("missing experimentally underdetermined full-face state")?;
        registered::record(out, i, sole, l)?;
        registered::flights(out, i, sole)
    } else {
        observations::record(out, i, sole, law)
    }
}
