// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Vary sheet mass and stiffness independently while replaying the same robot command.
use super::*;
use crate::arm_sheet::{Case, Metrics, SheetStepFault, run_case};
use serde::Serialize;

#[derive(Debug, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub(super) enum Trial {
    Completed {
        metrics: Box<Metrics>,
    },
    Refused {
        case: Case,
        tick: usize,
        diagnostic: String,
    },
}

impl Trial {
    pub(super) fn from_result(case: Case, result: Result<Metrics>) -> Result<Self> {
        match result {
            Ok(metrics) => Ok(Self::Completed {
                metrics: Box::new(metrics),
            }),
            Err(error) => {
                let refusal = error.downcast::<SheetStepFault>()?;
                Ok(Self::Refused {
                    case,
                    tick: refusal.tick,
                    diagnostic: refusal.diagnostic,
                })
            }
        }
    }
    fn measured(&self) -> Option<&Metrics> {
        match self {
            Self::Completed { metrics } => Some(metrics),
            Self::Refused { .. } => None,
        }
    }
}

#[derive(Debug, Serialize)]
struct Comparison {
    sheet_to_jaw_mass_ratio: f64,
    both_clocks_meet_grasp_tolerances: bool,
    max_panel_lift_change_m: f64,
    max_jaw_load_change_as_fraction_of_weight: f64,
    peak_material_gain_above_pad_work_as_fraction_of_lift_energy: [f64; 2],
}

fn compare(trials: &[Trial; 2], jaw_mass: f64) -> Option<Comparison> {
    let coarse = trials[0].measured()?;
    let fine = trials[1].measured()?;
    let largest_difference = |left: [f64; 2], right: [f64; 2]| {
        (left[0] - right[0]).abs().max((left[1] - right[1]).abs())
    };
    // mg times the commanded stroke is also well-defined if the sheet slips.
    let reference = fine.weight_n * arm_cable::LIFT_TRAVEL;
    Some(Comparison {
        sheet_to_jaw_mass_ratio: (fine.weight_n / 9.81) / jaw_mass,
        both_clocks_meet_grasp_tolerances: coarse.qualify().is_ok() && fine.qualify().is_ok(),
        max_panel_lift_change_m: largest_difference(coarse.mean_hold_lift_m, fine.mean_hold_lift_m),
        max_jaw_load_change_as_fraction_of_weight: largest_difference(
            coarse.mean_hold_jaw_up_n,
            fine.mean_hold_jaw_up_n,
        ) / fine.weight_n,
        peak_material_gain_above_pad_work_as_fraction_of_lift_energy: [coarse, fine]
            .map(|t| t.energy.max_material_gain_above_pad_work_j / reference),
    })
}

pub(super) fn run(out: &Path) -> Result<()> {
    let model = sdformat::load_str(arm::SCENE, "arm.sdf", &sdformat::NoResources)?;
    let jaw_mass = model
        .worlds
        .iter()
        .flat_map(|w| &w.models)
        .flat_map(|m| &m.links)
        .find(|l| l.scoped_name == "default::arm::left")
        .ok_or("missing reference jaw mass")?
        .inertial
        .mass;
    let mut rows = vec![];
    let mut qualified = 0;
    let mut replay_substeps = None;
    let mut candidate = Case::default();
    for mass_scale in [0.5, 1., 2.] {
        for stiffness_scale in [0.5, 1., 2.] {
            let case = Case {
                substeps: 2,
                mass_scale,
                stiffness_scale,
                closed: true,
            };
            let fine = Case {
                substeps: 4,
                ..case
            };
            let record = mass_scale == 2. && stiffness_scale == 0.5;
            let replay = out.join("heavy-soft");
            let first = Trial::from_result(case, run_case(case, record.then_some(&replay)))?;
            let record_fine = record && first.measured().is_none();
            let second = Trial::from_result(fine, run_case(fine, record_fine.then_some(&replay)))?;
            if record {
                candidate = case;
                replay_substeps = first
                    .measured()
                    .or(second.measured())
                    .map(|m| m.case.substeps);
            }
            let trials = [first, second];
            let comparison = compare(&trials, jaw_mass);
            qualified += usize::from(
                comparison
                    .as_ref()
                    .is_some_and(|c| c.both_clocks_meet_grasp_tolerances),
            );
            rows.push(json!({"mass_scale":mass_scale,"stiffness_scale":stiffness_scale,"comparison":comparison,"trials":trials}));
        }
    }
    // Measure the most compliant, heaviest specimen on a finer reference clock
    // even if both working clocks refused the contact.
    let reference = Case {
        substeps: 8,
        ..candidate
    };
    let replay = out.join("heavy-soft");
    let extra = Trial::from_result(
        reference,
        run_case(reference, replay_substeps.is_none().then_some(&replay)),
    )?;
    if replay_substeps.is_none() {
        replay_substeps = extra.measured().map(|m| m.case.substeps);
    }
    write_json(
        &out.join("sweep.json"),
        &json!({
            "jaw_mass_kg":jaw_mass,"material_substeps_per_arm_tick":[2,4],
            "comparisons_meeting_grasp_tolerances":qualified,"comparisons":rows,
            "heavy_soft_reference":extra,"heavy_soft_replay_substeps":replay_substeps
        }),
    )
}

#[cfg(test)]
#[path = "sheet_sweep_tests.rs"]
mod tests;
