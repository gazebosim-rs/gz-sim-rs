// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the sheet and 1 ms robot controller fixed while varying jaw density and pad stiffness.
#[path = "sheet_jaws/contact_windows.rs"]
pub(super) mod contact_windows;
#[path = "sheet_jaws/convergence.rs"]
pub(super) mod convergence;
#[path = "sheet_jaws/exchange_audit.rs"]
mod exchange_audit;
#[path = "sheet_jaws/force_view.rs"]
pub(super) mod force_view;
#[path = "sheet_jaws/interval.rs"]
mod interval;
use super::*;
use crate::arm_sheet::{Case, NOMINAL_FLARE, SheetStepFault};

#[derive(clap::Args, Clone, Copy, Debug)]
pub(crate) struct Settings {
    #[arg(long, default_value_t = 1.0)]
    jaw_mass_scale: f64,
    /// Scale derivative gain with sqrt(jaw mass) at fixed jaw position stiffness.
    #[arg(long)]
    tune_damping: bool,
    /// Number of native rigid/material exchanges per 1 ms actuator period.
    #[arg(long, default_value_t = 1)]
    exchanges_per_ms: usize,
    /// Keep the public Bevy step at 1 ms and exchange pad loads inside it.
    #[arg(long)]
    public_exchange_clock: bool,
    /// Measure the signed self-contact impulses delivered to each folded panel.
    #[arg(long)]
    panel_balance: bool,
    /// Normal viscous damping of each fingertip contact in N·s/m.
    #[arg(long, default_value_t = 0.)]
    pad_damping: f64,
    /// Pad compliance in m/N.
    #[arg(long, default_value_t = 0.001)]
    pad_compliance: f64,
    #[arg(long, default_value_t = 2)]
    substeps: usize,
    /// Cloth hinge damping in N·m·s/rad per edge.
    #[arg(long, default_value_t = 0.)]
    bend_damping: f64,
}

fn scene(
    scale: f64,
    compliance: f64,
    case: Case,
    panel_balance: bool,
    bend_damping: f64,
) -> Result<String> {
    if !(scale.is_finite() && scale > 0. && compliance.is_finite() && compliance >= 0.) {
        return Err(
            "expected a positive jaw density scale and a finite nonnegative pad compliance".into(),
        );
    }
    if !bend_damping.is_finite() || bend_damping < 0. {
        return Err("invalid cloth hinge damping".into());
    }
    let mut xml = arm_sheet::scene_with_loads(arm::SCENE, case, NOMINAL_FLARE, panel_balance)?;
    let doc = roxmltree::Document::parse(&xml)?;
    let mut edits = Vec::new();
    for node in doc.descendants().filter(|n| {
        n.has_tag_name("link")
            && matches!(n.attribute("name"), Some("left" | "right"))
            && n.parent().and_then(|m| m.attribute("name")) == Some("arm")
    }) {
        let inertial = node
            .children()
            .find(|n| n.has_tag_name("inertial"))
            .ok_or("missing jaw inertia")?;
        for field in inertial.descendants().filter(|n| {
            n.is_element()
                && matches!(
                    n.tag_name().name(),
                    "mass" | "ixx" | "iyy" | "izz" | "ixy" | "ixz" | "iyz"
                )
        }) {
            let text = field
                .first_child()
                .filter(|n| n.is_text())
                .ok_or("missing inertial coefficient")?;
            edits.push((
                text.range(),
                (field.text().unwrap_or_default().trim().parse::<f64>()? * scale).to_string(),
            ));
        }
    }
    for pad in doc.descendants().filter(|n| {
        n.has_tag_name("box")
            && matches!(
                n.attribute("link"),
                Some("default::arm::left" | "default::arm::right")
            )
    }) {
        let attr = pad
            .attribute_node("normal_compliance")
            .ok_or("missing articulated pad compliance")?;
        edits.push((attr.range_value(), compliance.to_string()));
    }
    if bend_damping > 0. {
        let cloth = doc
            .descendants()
            .find(|n| n.has_tag_name("cloth") && n.attribute("name") == Some("sheet"))
            .ok_or("missing folded sheet")?;
        let at = cloth.range().start + "<cloth".len();
        edits.push((at..at, format!(" bend_damping=\"{bend_damping}\"")));
    }
    edits.sort_by_key(|(r, _)| r.start);
    for (range, value) in edits.into_iter().rev() {
        xml.replace_range(range, &value);
    }
    Ok(xml)
}

fn damped_pads(xml: String, coefficient: f64) -> Result<String> {
    if !coefficient.is_finite() || coefficient < 0. {
        return Err("invalid pad damping".into());
    }
    if coefficient == 0. {
        return Ok(xml);
    }
    // This generator authors precisely two compliant box surfaces.
    Ok(xml.replace(
        "coupling=\"articulated_wrench\" normal_compliance=",
        &format!(
            "coupling=\"articulated_wrench\" normal_damping=\"{coefficient}\" normal_compliance="
        ),
    ))
}

fn timed_scene(
    scale: f64,
    compliance: f64,
    case: Case,
    exchanges_per_ms: usize,
    panel_balance: bool,
    bend_damping: f64,
) -> Result<String> {
    if exchanges_per_ms == 0 || !1_000_000_usize.is_multiple_of(exchanges_per_ms) {
        return Err("exchange count must divide one million nanoseconds exactly".into());
    }
    let native_dt = 0.001 / exchanges_per_ms as f64;
    Ok(
        scene(scale, compliance, case, panel_balance, bend_damping)?.replace(
            "<max_step_size>0.001</max_step_size>",
            &format!("<max_step_size>{native_dt}</max_step_size>"),
        ),
    )
}

#[cfg(test)]
fn run(out: &Path, settings: Settings) -> Result<()> {
    study(out, settings, None)
}

pub(super) fn study(out: &Path, settings: Settings, window: Option<(usize, usize)>) -> Result<()> {
    let Settings {
        jaw_mass_scale: scale,
        pad_compliance: compliance,
        pad_damping,
        substeps,
        tune_damping,
        public_exchange_clock,
        exchanges_per_ms,
        panel_balance,
        bend_damping,
    } = settings;
    let case = Case {
        substeps,
        ..Default::default()
    };
    let xml = if public_exchange_clock {
        if window.is_some()
            || !(2..=256).contains(&exchanges_per_ms)
            || !1_000_000_usize.is_multiple_of(exchanges_per_ms)
        {
            return Err(
                "public exchange study requires 2–256 exchanges and full public-step measurements"
                    .into(),
            );
        }
        scene(scale, compliance, case, panel_balance, bend_damping)?.replace(
            "filename=\"gz-sim-deformables-system\"",
            &format!(
                "filename=\"gz-sim-deformables-system\" rigid_exchanges=\"{exchanges_per_ms}\""
            ),
        )
    } else {
        timed_scene(
            scale,
            compliance,
            case,
            exchanges_per_ms,
            panel_balance,
            bend_damping,
        )?
    };
    let xml = damped_pads(xml, pad_damping)?;
    // With the same position stiffness, this preserves the continuous-time
    // damping ratio of an unloaded jaw. Both force and speed limits remain active.
    let damping = if tune_damping {
        12. * scale.sqrt()
    } else {
        12.
    };
    if let Some(window) = window {
        return interval::run(
            out,
            &xml,
            damping,
            [compliance, pad_damping],
            exchanges_per_ms,
            window,
        );
    }
    let result =
        arm_sheet::simulate_with_jaw_damping(&xml, case, NOMINAL_FLARE, Some(out), damping);
    let trial = match result {
        Ok(metrics) => {
            json!({"completed":true,"actuator_updates":metrics.actuator_updates,"meets_grasp_tolerances":metrics.qualify().is_ok(),"metrics":metrics})
        }
        Err(error) => {
            let fault = error.downcast::<SheetStepFault>()?;
            json!({"completed":false,"refused_at_tick":fault.tick,"diagnostic":fault.diagnostic})
        }
    };
    if public_exchange_clock && trial["completed"] == true {
        let fine = damped_pads(
            timed_scene(
                scale,
                compliance,
                case,
                exchanges_per_ms,
                panel_balance,
                bend_damping,
            )?,
            pad_damping,
        )?;
        exchange_audit::check(&xml, &fine, exchanges_per_ms, damping, out)?;
    }
    fs::write(out.join("world.sdf"), xml)?;
    write_json(
        &out.join("jaw-contact.json"),
        &json!({"native_step_ns":1_000_000/exchanges_per_ms,"public_step_ns":if public_exchange_clock {1_000_000}else{1_000_000/exchanges_per_ms},"actuator_period_ns":1_000_000,"jaw_density_scale":scale,"jaw_damping_ns_per_m":damping,"pad_compliance_m_per_n":compliance,"pad_damping_ns_per_m":pad_damping,"panel_balance":panel_balance,"bend_damping_nms_per_rad":bend_damping,"substeps":substeps,"trial":trial}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn public_and_reference_audits_refuse_inconsistent_clocks_and_missing_gripper() -> Result<()> {
        let case = Case {
            substeps: 1,
            ..Default::default()
        };
        let base = scene(1., 0.001, case, true, 0.001)?;
        let internal = base.replace(
            "filename=\"gz-sim-deformables-system\"",
            "filename=\"gz-sim-deformables-system\" rigid_exchanges=\"4\"",
        );
        let dir = tempfile::tempdir()?;
        let e = exchange_audit::check(&internal, &base, 4, 12., dir.path()).unwrap_err();
        assert!(e.to_string().contains("exchange clocks disagree"));
        assert!(exchange_audit::check("<broken/>", &base, 4, 12., dir.path()).is_err());
        assert!(exchange_audit::check(&internal, "<broken/>", 4, 12., dir.path()).is_err());
        let empty = "<sdf version=\"1.10\"><world name=\"default\"><physics name=\"p\" type=\"ignored\"><max_step_size>0.001</max_step_size></physics></world></sdf>";
        assert!(exchange_audit::check(empty, &base, 4, 12., dir.path()).is_err());
        assert!(exchange_audit::check(&internal, empty, 4, 12., dir.path()).is_err());
        let fine = timed_scene(1., 0.001, case, 4, true, 0.001)?;
        let different_payload = fine.replace("mass=\"0.0007\"", "mass=\"0.00075\"");
        assert_ne!(fine, different_payload);
        let wrong =
            exchange_audit::check(&internal, &different_payload, 4, 12., dir.path()).unwrap_err();
        assert!(
            wrong
                .to_string()
                .contains("diverged from independent native reference"),
            "{wrong}"
        );
        let mut settings = Settings {
            jaw_mass_scale: 1.,
            pad_compliance: 0.001,
            substeps: 1,
            tune_damping: false,
            exchanges_per_ms: 4,
            public_exchange_clock: true,
            panel_balance: true,
            bend_damping: 0.,
            pad_damping: 0.,
        };
        assert!(study(dir.path(), settings, Some((0, 2))).is_err());
        settings.exchanges_per_ms = 3;
        assert!(study(dir.path(), settings, None).is_err());
        settings.exchanges_per_ms = 4;
        settings.jaw_mass_scale = -1.;
        assert!(study(dir.path(), settings, None).is_err());
        assert!(!dir.path().join("exchange-audit.json").exists());
        Ok(())
    }
    #[test]
    fn density_changes_both_full_tensors_and_only_the_two_pads() -> Result<()> {
        let nominal =
            arm_sheet::scene_with_loads(arm::SCENE, Case::default(), NOMINAL_FLARE, false)?;
        let light = scene(0.125, 0.0001, Case::default(), false, 0.)?;
        let parse = |s: &str| sdformat::load_str(s, "test.sdf", &sdformat::NoResources);
        let (a, b) = (parse(&nominal)?, parse(&light)?);
        for (model, other) in a.worlds[0].models.iter().zip(&b.worlds[0].models) {
            for (link, changed) in model.links.iter().zip(&other.links) {
                let factor =
                    if model.name == "arm" && matches!(link.name.as_str(), "left" | "right") {
                        0.125
                    } else {
                        1.
                    };
                assert_eq!(changed.inertial.mass, link.inertial.mass * factor);
                assert_eq!(
                    changed.inertial.inertia,
                    link.inertial.inertia.map(|i| i * factor)
                );
            }
        }
        assert_eq!(light.matches("normal_compliance=\"0.0001\"").count(), 2);
        assert!(scene(0., 0.001, Case::default(), false, 0.).is_err());
        assert!(scene(1., f64::NAN, Case::default(), false, 0.).is_err());
        Ok(())
    }
    #[test]
    fn force_exchange_resolves_a_stiff_grasp_on_the_same_actuator_clock() -> Result<()> {
        let temp = tempfile::tempdir()?;
        let settings = Settings {
            jaw_mass_scale: 1. / 64.,
            pad_compliance: 0.00025,
            substeps: 1,
            tune_damping: true,
            exchanges_per_ms: 1,
            public_exchange_clock: false,
            panel_balance: false,
            bend_damping: 0.,
            pad_damping: 0.,
        };
        for (exchanges, soft_steps) in [(1, 4), (4, 1)] {
            let dir = temp.path().join(exchanges.to_string());
            fs::create_dir(&dir)?;
            run(
                &dir,
                Settings {
                    substeps: soft_steps,
                    exchanges_per_ms: exchanges,
                    ..settings
                },
            )?;
            let result: Value = serde_json::from_slice(&fs::read(dir.join("jaw-contact.json"))?)?;
            let trial = &result["trial"];
            if exchanges == 1 {
                // The refined shell endpoint now completes this coarse run.
                // Its jaws still miss the sheet and the bench carries its weight.
                assert_eq!(trial["completed"], true);
                assert_eq!(trial["meets_grasp_tolerances"], false);
                assert_eq!(trial["actuator_updates"], 3400);
                let m = &trial["metrics"];
                assert_eq!(m["mean_hold_jaw_up_n"], json!([0., 0.]));
                assert!(
                    (m["mean_hold_bench_up_n"].as_f64().unwrap() - m["weight_n"].as_f64().unwrap())
                        .abs()
                        < 0.001
                );
                assert!(dir.join("replay").is_dir());
            } else {
                assert_eq!(trial["actuator_updates"], 3400);
                assert_eq!(trial["meets_grasp_tolerances"], true);
                let recording: Value =
                    serde_json::from_slice(&fs::read(dir.join("recording.json"))?)?;
                assert_eq!(recording["steps"], 13_600);
                assert_eq!(recording["simulated_ns"], 3_400_000_000_u64);
                assert_eq!(recording["frames"], 137);
                let metrics = &trial["metrics"];
                for lift in metrics["mean_hold_lift_m"].as_array().unwrap() {
                    assert!((0.149..0.150).contains(&lift.as_f64().unwrap()));
                }
                let support: f64 = metrics["mean_hold_jaw_up_n"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_f64().unwrap())
                    .sum();
                assert!((support - metrics["weight_n"].as_f64().unwrap()).abs() < 0.0001);
                assert!(metrics["max_linear_impulse_residual_ns"].as_f64().unwrap() < 1e-11);
            }
        }
        assert!(
            run(
                temp.path(),
                Settings {
                    exchanges_per_ms: 0,
                    ..settings
                }
            )
            .is_err()
        );
        assert!(
            run(
                temp.path(),
                Settings {
                    exchanges_per_ms: 3,
                    ..settings
                }
            )
            .is_err()
        );
        Ok(())
    }
}
