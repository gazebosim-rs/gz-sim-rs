// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-state empirical rank sensitivity across successive uniform contact grids.
use super::*;

#[derive(Default, Serialize)]
pub(super) struct Summary {
    counts_by_reference: [[usize; 3]; 4],
    successive_transitions: [[[usize; 3]; 3]; 3],
    same_at_all_four: usize,
}
impl Summary {
    pub(super) fn from_values(values: &[Metrics]) -> Self {
        let mut s = Self::default();
        for m in values {
            let ranks = m.change_by_reference_mm.map(|d| {
                if d < 0. {
                    0
                } else if d > 0. {
                    2
                } else {
                    1
                }
            });
            for (count, rank) in s.counts_by_reference.iter_mut().zip(ranks) {
                count[rank] += 1;
            }
            for (counts, pair) in s.successive_transitions.iter_mut().zip(ranks.windows(2)) {
                counts[pair[0]][pair[1]] += 1;
            }
            s.same_at_all_four += usize::from(ranks.iter().all(|r| *r == ranks[0]));
        }
        s
    }
}

pub(super) fn plot(path: &Path, cells: &[Cell], completed: usize) -> Result<()> {
    let root = SVGBackend::new(path, (1240, 740)).into_drawing_area();
    root.fill(&WHITE)?;
    let title = root.titled(
        "Does the closer policy depend on the uniform reference?",
        ("sans-serif", 25),
    )?;
    let (heading, body) = title.split_vertically(48);
    heading.draw(&Text::new(
        format!("{completed}/1,250 fixtures  ·  {} matched landings  ·  each cell counts a change from row to column", cells.len()),
        (26, 19), ("sans-serif", 17),
    ))?;
    let totals =
        [0, 1].map(|w| Summary::from_values(&cells.iter().map(|c| c.at(w)).collect::<Vec<_>>()));
    let center = |area: &_, font| {
        ("sans-serif", font)
            .into_text_style(area)
            .pos(Pos::new(HPos::Center, VPos::Center))
    };
    let labels = ["Onset", "Tie", "Adaptive"];
    for (i, panel) in body.split_evenly((2, 3)).into_iter().enumerate() {
        let (window, g) = (i / 3, i % 3);
        let counts = &totals[window].successive_transitions[g];
        let both = counts[0][0] + counts[1][1] + counts[2][2];
        let size = panel.dim_in_pixel();
        let title = format!(
            "{} ms   ·   {} → {} solves",
            WINDOWS[window] / 2,
            GRIDS[g],
            GRIDS[g + 1]
        );
        panel.draw(&Text::new(
            title,
            (size.0 as i32 / 2, 19),
            center(&panel, 19),
        ))?;
        panel.draw(&Text::new(
            format!("{} rankings changed", cells.len() - both),
            (size.0 as i32 / 2, 45),
            center(&panel, 15),
        ))?;
        for (row, counts) in counts.iter().enumerate() {
            let y = 97 + row as i32 * 59;
            panel.draw(&Text::new(labels[row], (24, y + 27), ("sans-serif", 14)))?;
            for (column, &count) in counts.iter().enumerate() {
                let x = 111 + column as i32 * 84;
                if row == 0 {
                    panel.draw(&Text::new(
                        labels[column],
                        (x + 40, y - 14),
                        center(&panel, 14),
                    ))?;
                }
                let color = match (row == column, count > 0) {
                    (_, false) => RGBColor(246, 247, 247),
                    (true, _) => RGBColor(189, 217, 215),
                    (false, _) => RGBColor(245, 203, 177),
                };
                panel.draw(&Rectangle::new([(x, y), (x + 80, y + 55)], color.filled()))?;
                panel.draw(&Text::new(
                    count.to_string(),
                    (x + 40, y + 27),
                    center(&panel, 19),
                ))?;
            }
        }
    }
    root.present()?;
    Ok(())
}
