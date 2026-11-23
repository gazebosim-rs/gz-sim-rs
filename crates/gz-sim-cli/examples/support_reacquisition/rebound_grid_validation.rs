// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent numerical controls at 62.5 and 31.25 µs after budget selection.
use super::*;

pub(super) fn write(out: &Path, selected: Budget) -> Result<()> {
    let mut cases = Vec::new();
    for (deck, switched, stroke) in [
        (Deck::Flat, false, 2),
        (Deck::Flat, true, 2),
        (Deck::Edge, false, 2),
        (Deck::Edge, true, 1),
    ] {
        let t = target(deck, switched, stroke)?.ok_or("missing validation contact")?;
        let baseline = continuation(t, Mode::Candidate)?;
        let candidate = continuation(t, Mode::Budget(selected))?;
        let usual = continuation(t, Mode::Fixed { onset: t.onset })?;
        let mut rows = Vec::new();
        for count in [4, 8, 16] {
            let other;
            let reference = if count == 4 {
                &usual
            } else {
                other = continuation(
                    t,
                    Mode::FinerGrid {
                        onset: t.onset,
                        substeps: count,
                    },
                )?;
                &other
            };
            for (name, trajectory) in [
                ("baseline", &baseline),
                ("selected", &candidate),
                ("four_substeps", &usual),
            ] {
                let (pose, velocity) = actuator_parity(t, trajectory, reference)?;
                let mut row = score(t, trajectory, reference)?;
                row["policy"] = name.into();
                row["reference_substeps"] = count.into();
                row["reference_step_ns"] = (STEP_US * 1000 / u64::from(count)).into();
                row["max_pad_position_difference_m"] = pose.into();
                row["max_pad_command_error_m_s"] = velocity.into();
                rows.push(row);
            }
        }
        cases.push(json!({"deck":deck.label(),"history":history(switched),"stroke":stroke+1,"onset":t.onset,"rows":rows}));
    }
    write_json(&out.join("grid-validation.json"), &json!({"cases":cases}))
}
