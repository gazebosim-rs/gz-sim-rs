// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{BodyId, CouplingError, MultiFidelityBackend, PhysicsBackend};
use gz_physics_xpbd::TranslationBoundary;
use std::time::Duration;
pub(super) fn from_native(
    physics: &PhysicsBackend,
    body: BodyId,
    dt: Duration,
) -> Result<Option<TranslationBoundary>, CouplingError> {
    let Some((curve, origin, end)) = physics
        .downcast_ref::<MultiFidelityBackend>()
        .and_then(|p| p.translation_path(body))
    else {
        return Ok(None);
    };
    Ok(Some(
        TranslationBoundary::from_publication(curve, origin, end, dt)
            .ok_or(CouplingError::Binding)?,
    ))
}
