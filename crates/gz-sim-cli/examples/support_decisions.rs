// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise local support changes and plot committed tier and contact observations.
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, contact_promotion, load_world_with_sdf_step,
    queue_link_world_velocity, queue_link_world_wrench, set_model_fidelity,
};
use plotters::prelude::*;
use serde::Serialize;
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_sliding_payload.sdf");
const MISS: &str = "-1.1 1.15 0.17";

#[derive(Clone, Copy)]
enum Pass {
    Strike,
    Clear,
    Plane,
    Edge,
    Bumper,
    Wrench,
    Redirect,
}
impl Pass {
    fn label(self) -> &'static str {
        match self {
            Self::Strike => "Direct strike",
            Self::Clear => "Clear pass on box floor",
            Self::Plane => "Clear pass on plane",
            Self::Edge => "Pass near floor edge",
            Self::Bumper => "Pass near fixed bumper",
            Self::Wrench => "Force applied at 150 ms",
            Self::Redirect => "Redirected at 110 ms",
        }
    }
    fn source(self) -> String {
        let source = if matches!(self, Self::Strike) {
            SDF.to_owned()
        } else {
            SDF.replace("-1.1 0.71 0.17", MISS)
        };
        match self {
            // The upper floor edge at Y=1.45 m lies beyond the sliding box, within its one-step fallback.
            Self::Edge=>source.replace("0 0 -0.1 0 0 0","0 -3.55 -0.1 0 0 0"),
            Self::Bumper=>source.replace("</world>",r#"<model name="bumper"><static>true</static><pose>0 1.62 0.16 0 0 0</pose><link name="body"><collision name="stop"><geometry><sphere><radius>0.10</radius></sphere></geometry></collision></link></model></world>"#),
            Self::Plane=>source.replace("0 0 -0.1 0 0 0","0 0 0 0 0 0")
                .replace(r#"<collision name="surface"><geometry><box><size>10 10 0.2</size></box>"#,r#"<collision name="surface"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane>"#),
            _=>source,
        }
    }
}
#[derive(Serialize)]
struct Decision {
    case: &'static str,
    promotion_ms: Option<u64>,
    first_tip_impulse_ms: Option<u64>,
    peak_tip_impulse_ns: f64,
}
fn run(pass: Pass) -> Result<Decision> {
    let mut sim = load_world_with_sdf_step(
        &pass.source(),
        "support_decisions.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    sim.step_exact(2000)?;
    let id = |name: &str| -> Result<u64> {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .and_then(|e| e.get::<EntityId>().map(|v| v.get()))
            .ok_or_else(|| format!("missing {name}").into())
    };
    let arm = id("default::arm")?;
    let ram = id("default::ram::head")?;
    let tip = id("default::arm::tip")?;
    let [a, b] = [ram, tip].map(|i| {
        sim.world()
            .get::<BodyHandle>(entity_by_id(sim.world(), i).unwrap())
            .unwrap()
            .id()
    });
    set_model_fidelity(&mut sim, arm, ArticulationTier::Prescribed)?;
    contact_promotion::configure(&mut sim, arm, vec![ram])?;
    queue_link_world_velocity(&mut sim, ram, [4., 0., 0.], [0.; 3])?;
    let mut result = Decision {
        case: pass.label(),
        promotion_ms: None,
        first_tip_impulse_ms: None,
        peak_tip_impulse_ns: 0.,
    };
    for tick in 1..=500 {
        if matches!(pass, Pass::Wrench) && tick == 150 {
            queue_link_world_wrench(&mut sim, ram, [1., 0., 0.], [0.; 3])?;
        }
        if matches!(pass, Pass::Redirect) && tick == 111 {
            queue_link_world_velocity(&mut sim, ram, [4., -3.9, 0.], [0.; 3])?;
        }
        sim.step_exact(1)?;
        let impulse: f64 = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .rigid_contact_observations()
            .iter()
            .filter(|p| (p.a, p.b) == (a, b) || (p.a, p.b) == (b, a))
            .map(|p| p.normal_impulse)
            .sum();
        if impulse > 0. {
            result.first_tip_impulse_ms.get_or_insert(tick);
            result.peak_tip_impulse_ns = result.peak_tip_impulse_ns.max(impulse);
        }
    }
    result.promotion_ms = contact_promotion::last_promotion(&sim, arm).map(|p| p.iteration - 2000);
    match pass {
        Pass::Strike | Pass::Redirect => assert!(
            matches!((result.promotion_ms,result.first_tip_impulse_ms),(Some(p),Some(c)) if p<c)
        ),
        Pass::Clear | Pass::Plane => assert_eq!(
            (result.promotion_ms, result.first_tip_impulse_ms),
            (None, None)
        ),
        Pass::Wrench => assert_eq!(
            (result.promotion_ms, result.first_tip_impulse_ms),
            (Some(150), None)
        ),
        Pass::Edge | Pass::Bumper => {
            assert!(result.promotion_ms.is_some() && result.first_tip_impulse_ms.is_none())
        }
    }
    Ok(result)
}
fn plot(rows: &[Decision], path: &Path) -> Result<()> {
    let root = SVGBackend::new(path, (950, 425)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "When the supported arm switches to dynamic",
        (24, 15),
        ("sans-serif", 24),
    ))?;
    root.draw(&Text::new(
        "Gray = prescribed   ·   Teal = dynamic   ·   Orange dot = first payload–tip impulse",
        (24, 49),
        ("sans-serif", 16),
    ))?;
    let x = |ms: u64| 240 + (ms as i32 * 650 / 500);
    for i in 0..=5 {
        let t = i * 100;
        root.draw(&PathElement::new(
            [(x(t), 86), (x(t), 349)],
            RGBColor(233, 237, 240),
        ))?;
        root.draw(&Text::new(
            format!("{t}"),
            (x(t) - 8, 372),
            ("sans-serif", 15),
        ))?;
    }
    root.draw(&Text::new(
        "Milliseconds after launch",
        (480, 397),
        ("sans-serif", 16),
    ))?;
    for (i, r) in rows.iter().enumerate() {
        let y = 99 + i as i32 * 39;
        root.draw(&Text::new(r.case, (24, y - 9), ("sans-serif", 17)))?;
        root.draw(&Rectangle::new(
            [(x(0), y - 7), (x(500), y + 7)],
            RGBColor(197, 202, 207).filled(),
        ))?;
        if let Some(t) = r.promotion_ms {
            root.draw(&Rectangle::new(
                [(x(t), y - 7), (x(500), y + 7)],
                RGBColor(18, 111, 126).filled(),
            ))?;
        }
        if let Some(t) = r.first_tip_impulse_ms {
            root.draw(&Circle::new((x(t), y), 6, RGBColor(241, 156, 53).filled()))?;
        }
    }
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let dir: PathBuf = std::env::args_os()
        .nth(1)
        .ok_or("expected output directory")?
        .into();
    fs::create_dir_all(&dir)?;
    let rows = [
        Pass::Strike,
        Pass::Clear,
        Pass::Plane,
        Pass::Edge,
        Pass::Bumper,
        Pass::Wrench,
        Pass::Redirect,
    ]
    .map(run)
    .into_iter()
    .collect::<Result<Vec<_>>>()?;
    fs::write(
        dir.join("decisions.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    plot(&rows, &dir.join("decisions.svg"))
}
