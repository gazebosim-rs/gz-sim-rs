// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Series stiffness of two linear layers. Stiffness and reaction use the same
//! reference measure (per metre for an edge, per square metre for a sole).
#[derive(Debug, Clone, Copy)]
pub(super) struct SeriesSprings {
    stiffness: f64,
    foot_fraction: f64,
    deck_fraction: f64,
}

pub(super) struct LayerResponse {
    pub load: f64,
    pub energy: f64,
    pub foot_compression: f64,
    pub deck_compression: f64,
}

impl SeriesSprings {
    pub fn new(foot: f64, deck: f64) -> Option<Self> {
        if !foot.is_finite() || !deck.is_finite() || foot <= 0.0 || deck <= 0.0 {
            return None;
        }
        let scale = foot.max(deck);
        let foot_scaled = foot / scale;
        let deck_scaled = deck / scale;
        let sum = foot_scaled + deck_scaled;
        let stiffness = foot.min(deck) / sum;
        (stiffness > 0.0).then_some(Self {
            stiffness,
            foot_fraction: deck_scaled / sum,
            deck_fraction: foot_scaled / sum,
        })
    }

    pub fn response(self, signed_overlap: f64) -> Option<LayerResponse> {
        if !signed_overlap.is_finite() {
            return None;
        }
        let compression = signed_overlap.max(0.0);
        let load = self.stiffness * compression;
        let energy = 0.5 * compression * load;
        energy.is_finite().then_some(LayerResponse {
            load,
            energy,
            foot_compression: compression * self.foot_fraction,
            deck_compression: compression * self.deck_fraction,
        })
    }
}
