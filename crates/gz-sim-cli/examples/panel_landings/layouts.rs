// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::sole_contact::{SoleDeck, SoleMaterial};
use gz_physics::sole_supports::SolePanel;

#[derive(Clone, Copy)]
pub struct Tile {
    pub xy: [f64; 2],
    pub half: [f64; 2],
    pub yaw: f64,
    /// Known covered area in square metres, computed from design dimensions.
    pub area: f64,
    pub stiffness_scale: f64,
}

fn tile(x: f64, y: f64, hx: f64, hy: f64, area: f64) -> Tile {
    Tile {
        xy: [x, y],
        half: [hx, hy],
        yaw: 0.,
        area,
        stiffness_scale: 1.,
    }
}

pub fn layouts() -> Vec<(&'static str, Vec<Tile>)> {
    // The footprint is 0.4 by 0.2 metres. Each arrangement has reflection
    // or central symmetry, so that a centered normal drop has zero net torque.
    vec![
        ("wide_deck", vec![tile(0., 0., 0.32, 0.18, 0.08)]),
        (
            "longitudinal_seam",
            [-0.125, 0.125]
                .map(|x| tile(x, 0., 0.125, 0.13, 0.04))
                .to_vec(),
        ),
        (
            "transverse_seam",
            [-0.075, 0.075]
                .map(|y| tile(0., y, 0.25, 0.075, 0.04))
                .to_vec(),
        ),
        (
            "rail_gap",
            [-0.12, 0.12]
                .map(|x| tile(x, 0., 0.08, 0.13, 0.032))
                .to_vec(),
        ),
        (
            "deck_slot",
            [-0.08, 0.08]
                .map(|y| tile(0., y, 0.04, 0.04, 0.0048))
                .to_vec(),
        ),
        (
            "corner_piers",
            [-0.14, 0.14]
                .into_iter()
                .flat_map(|x| [-0.075, 0.075].map(|y| tile(x, y, 0.035, 0.025, 0.0035)))
                .collect(),
        ),
        (
            "rotated_inset",
            vec![Tile {
                yaw: 0.5,
                ..tile(0., 0., 0.09, 0.04, 0.0144)
            }],
        ),
        ("narrow_crossbar", vec![tile(0., 0., 0.03, 0.17, 0.012)]),
        (
            "mixed_rails",
            vec![
                tile(-0.15, 0., 0.05, 0.13, 0.02),
                Tile {
                    stiffness_scale: 2.5,
                    ..tile(0., 0., 0.07, 0.13, 0.028)
                },
                tile(0.15, 0., 0.05, 0.13, 0.02),
            ],
        ),
        (
            "graded_tiles",
            [-1, 0, 1]
                .into_iter()
                .flat_map(|i| {
                    [-1, 0, 1].map(move |j| Tile {
                        stiffness_scale: 1. + f64::from(i * i + 2 * j * j) * 0.25,
                        ..tile(
                            f64::from(i) * 0.12,
                            f64::from(j) * 0.065,
                            0.05,
                            0.025,
                            0.005,
                        )
                    })
                })
                .collect(),
        ),
    ]
}

pub struct Trial {
    pub name: &'static str,
    pub mass: f64,
    pub incoming: f64,
    pub damping: f64,
    pub stiffness: f64,
    pub ratio: f64,
    pub rotation: DQuat,
    pub drift: DVec3,
}
pub fn trials() -> Vec<Trial> {
    let standard = || Trial {
        name: "standard",
        mass: 2.,
        incoming: 1.,
        damping: 0.3,
        stiffness: 1.8e7,
        ratio: 1.,
        rotation: DQuat::IDENTITY,
        drift: DVec3::ZERO,
    };
    vec![
        Trial {
            name: "elastic",
            damping: 0.,
            ..standard()
        },
        standard(),
        Trial {
            name: "lossy",
            damping: 2.,
            ..standard()
        },
        Trial {
            name: "heavy",
            mass: 7.,
            ..standard()
        },
        Trial {
            name: "fast",
            incoming: 2.5,
            ..standard()
        },
        Trial {
            name: "stiff_coating",
            stiffness: 1.8e8,
            ..standard()
        },
        Trial {
            name: "soft_deck",
            ratio: 0.08,
            ..standard()
        },
        Trial {
            name: "rising_deck",
            drift: DVec3::Z * 0.8,
            ..standard()
        },
        Trial {
            name: "translating_belt",
            drift: DVec3::new(1.1, -0.4, 0.),
            ..standard()
        },
        Trial {
            name: "inclined_moving_deck",
            rotation: DQuat::from_rotation_z(0.63) * DQuat::from_rotation_y(0.37),
            drift: DVec3::new(0.7, 0.2, -0.4),
            ..standard()
        },
    ]
}
impl Trial {
    pub fn panels(&self, tiles: &[Tile]) -> Vec<SolePanel> {
        tiles
            .iter()
            .enumerate()
            .map(|(i, t)| {
                let foot_k = self.stiffness * t.stiffness_scale * (1. + 1. / self.ratio);
                let deck_k = self.stiffness * t.stiffness_scale * (1. + self.ratio);
                SolePanel {
                    id: i as u64 + 1,
                    deck: SoleDeck {
                        center: self.rotation * DVec3::new(t.xy[0], t.xy[1], -0.04),
                        orientation: self.rotation * DQuat::from_rotation_z(t.yaw),
                        half_extents: DVec3::new(t.half[0], t.half[1], 0.04),
                        velocity: self.drift,
                    },
                    material: SoleMaterial::new(foot_k, deck_k, self.damping, 0.4, 0.01).unwrap(),
                }
            })
            .collect()
    }
}
