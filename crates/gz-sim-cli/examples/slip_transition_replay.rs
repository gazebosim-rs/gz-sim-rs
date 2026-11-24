// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched Bevy worlds with actual kinetic-transition timing and an independent reference.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/inclined_sheet.rs"]
mod sheet;
mod soft_replay;
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/slip_transition.rs"]
mod transition;
use clap::Parser;
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use sheet::Incline;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};
use transition::Run;
#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    moving: bool,
    #[arg(long)]
    measurements: PathBuf,
}
fn cut(g: &Incline) -> Value {
    let mut receipts = vec![];
    for r in g.material().support_this_step() {
        let sites = |a: &Option<
            gz_sim_scene::deformable::articulated_impact::SupportApplication,
        >| {
            a.as_ref().map(|a|a.material.iter().map(|l|{
                let friction=l.friction.expect("frictional supported sheet");
                json!({"source":l.site.sphere,"vertices":l.site.feature.vertices(),"point":l.site.point,"normal":l.site.normal,"on_link":l.on_rigid,"on_nodes":l.on_particles,"before":friction.initial_slip,"after":friction.final_slip})
            }).collect::<Vec<_>>())
        };
        receipts.push(json!({"start_ns":r.start.as_nanos()as u64,"duration_ns":r.duration.as_nanos()as u64,"refinements":r.slip_transition_refinements,"trials":r.trajectory_trials,"head":sites(&r.beginning),"tail":sites(&r.end)}));
    }
    json!(receipts)
}
struct Visual<'a> {
    frame: &'a mut Frame,
    name: &'a str,
    shift: f64,
    rot: DQuat,
}
impl Visual<'_> {
    fn transform(&self, p: DVec3) -> DVec3 {
        self.rot * p + self.shift * DVec3::Y
    }
    fn key(&self, s: &str) -> String {
        format!("{}_{s}", self.name)
    }
    fn ball(&mut self, s: &str, p: DVec3, r: f64, col: [f64; 4]) {
        self.frame.add(
            &self.key(s),
            self.transform(p),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":r}),
            col,
        );
    }
    fn bar(&mut self, s: &str, a: DVec3, b: DVec3, len: f64, width: f64, col: [f64; 4]) {
        self.frame.edge(
            &self.key(s),
            self.transform(a),
            self.transform(b),
            len,
            width,
            col,
        );
    }
}
fn draw(
    v: &mut Visual<'_>,
    g: &Incline,
    seed: &[DVec3],
    stand_lengths: [f64; 4],
    reference: (f64, f64),
    initial: DVec3,
    color: [f64; 4],
) -> Value {
    let along = sheet::downhill();
    let normal = sheet::normal();
    let (x, vel, m) = g.centroid();
    let x_error = (x - initial).dot(along) - reference.0;
    let vel_error = vel.dot(along) - reference.1;
    let pale = [0.31, 0.39, 0.51, 1.];
    let blue = [0.15, 0.43, 0.78, 1.];
    let rose = [0.94, 0.22, 0.39, 1.];
    for (i, b) in g.states().iter().enumerate() {
        let loc = DVec3::from(b.position);
        v.ball(&format!("pad_{i}"), loc, 0.085, blue);
        v.ball(
            &format!("rotor_marker_{i}"),
            loc + DQuat::from_array(b.orientation) * (0.08 * DVec3::X),
            0.011,
            [0.94, 0.73, 0.17, 1.],
        );
        let base = DVec3::new(loc.x, loc.y, 0.13);
        v.bar(
            &format!("stand_{i}"),
            base,
            loc,
            stand_lengths[i],
            0.022,
            pale,
        );
    }
    let material = g.material().particles();
    let nodes = material
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    for (i, p) in nodes.iter().enumerate() {
        v.ball(&format!("node_{i}"), *p, 0.014, color);
    }
    for (i, j) in [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)] {
        v.bar(
            &format!("edge_{i}_{j}"),
            nodes[i],
            nodes[j],
            seed[i].distance(seed[j]),
            0.009,
            color,
        );
    }
    for (face, ids) in [[0, 1, 3], [0, 3, 2]].iter().enumerate() {
        for (dir, [o, a, b]) in [[0, 1, 2], [1, 0, 2], [2, 0, 1]].iter().enumerate() {
            let (o, a, b) = (ids[*o], ids[*a], ids[*b]);
            for n in 1..=2 {
                let u = n as f64 / 3.;
                v.bar(
                    &format!("mesh_{face}_{dir}_{n}"),
                    nodes[o].lerp(nodes[a], u),
                    nodes[o].lerp(nodes[b], u),
                    u * seed[a].distance(seed[b]),
                    0.004,
                    color,
                );
            }
        }
    }
    // The white bead is the zero-error datum for each row. Error scales are
    // fixed in both panels. The physical cloth geometry is unscaled.
    let at = x + normal * 0.10;
    v.ball("velocity_error_zero", at, 0.013, [0.88, 0.91, 0.98, 1.]);
    let at_x = x + normal * 0.18;
    v.ball("position_error_zero", at_x, 0.013, [0.88, 0.91, 0.98, 1.]);
    for k in 1..=5 {
        let u = k as f64 / 5.;
        v.ball(
            &format!("velocity_error_{k}"),
            at + along * (vel_error * 4200. * u),
            if k == 5 { 0.016 } else { 0.007 },
            rose,
        );
        v.ball(
            &format!("position_error_{k}"),
            at_x + along * (x_error * 13000. * u),
            if k == 5 { 0.016 } else { 0.007 },
            [0.97, 0.78, 0.18, 1.],
        );
    }
    json!({"body":g.states().map(|b|json!({"x":b.position,"q":b.orientation,"v":b.velocity,"w":b.angular_velocity})), "nodes":material.iter().map(|p|json!({"x":p.position,"v":p.velocity,"m":1./p.inverse_mass})).collect::<Vec<_>>(),"mass":m,"x_error_m":x_error,"v_error_m_s":vel_error})
}
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let base = Run {
        mu: 0.15,
        speed: -0.42,
        width: 250000,
        transition: None,
        yaw: 0.,
        extra_mass: if a.moving { 0.002 } else { 0. },
    };
    let mut control = Incline::new(&base.sdf())?;
    let mut adaptive = Incline::new(
        &Run {
            transition: Some(2000),
            ..base
        }
        .sdf(),
    )?;
    let mut physical_ref = if a.moving {
        Some(Incline::new(
            &Run {
                width: 2000,
                ..base
            }
            .sdf(),
        )?)
    } else {
        None
    };
    let initial = control.centroid().0;
    let stand_lengths = control.states().map(|s| s.position[2] - 0.13);
    let seed = control
        .material()
        .particles()
        .iter()
        .map(|p| p.position.into())
        .collect::<Vec<_>>();
    let end = if a.moving { 300 } else { 480 };
    let mut out = BufWriter::new(File::create_new(a.measurements)?);
    let mut frames = vec![];
    for t in 0..=end {
        if t > 0 {
            control.step()?;
            adaptive.step()?;
            if let Some(r) = &mut physical_ref {
                r.step()?;
            }
        }
        let reference = if let Some(r) = &physical_ref {
            let (x, v, _) = r.centroid();
            ((x - initial).dot(base.down()), v.dot(base.down()))
        } else {
            sheet::planar_motion(t as f64 * 0.001, base.mu, base.speed)
        };
        let mut f = Frame::default();
        let mut raw =
            json!({"time_ns":t as u64*1_000_000,"reference":{"x":reference.0,"v":reference.1}});
        if let Some(r) = &physical_ref {
            raw["reference"]["nodes"] = json!(
                r.material()
                    .particles()
                    .iter()
                    .map(|p| json!({"x":p.position,"v":p.velocity,"m":1./p.inverse_mass}))
                    .collect::<Vec<_>>()
            );
        }
        for (lane, (name, w, color)) in [
            ("fixed", &control, [0.11, 0.79, 0.64, 1.]),
            ("adaptive", &adaptive, [0.96, 0.54, 0.18, 1.]),
        ]
        .into_iter()
        .enumerate()
        {
            let mut view = Visual {
                frame: &mut f,
                name,
                shift: [0.75, -0.75][lane],
                rot: DQuat::from_rotation_z(-0.81),
            };
            let mut stats = draw(
                &mut view,
                w,
                &seed,
                stand_lengths,
                reference,
                initial,
                color,
            );
            stats["intervals"] = cut(w);
            raw[name] = stats;
        }
        frames.push(f.finish(control.sim.elapsed(), t as usize + 1));
        serde_json::to_writer(&mut out, &raw)?;
        writeln!(out)?;
    }
    out.flush()?;
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), frames)?;
    println!(
        "{} native-frame state pairs, complete measured trial intervals, model-derived markers.",
        end + 1
    );
    Ok(())
}
