// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// The row-major cloth uses the upper-left to lower-right diagonal in every cell.
pub(crate) fn cell_faces(bottom_right: usize, columns: usize) -> [[usize; 3]; 2] {
    let upper_left = bottom_right - columns - 1;
    [
        [upper_left, bottom_right - columns, bottom_right],
        [upper_left, bottom_right, bottom_right - 1],
    ]
}

impl DistanceConstraint {
    /// Build a link with the current separation as its rest length.
    pub fn between(
        a: usize,
        b: usize,
        particles: &[Particle],
        compliance: f64,
    ) -> Result<Self, XpbdError> {
        let a_pos = particles
            .get(a)
            .ok_or(XpbdError::InvalidConstraint)?
            .position;
        let b_pos = particles
            .get(b)
            .ok_or(XpbdError::InvalidConstraint)?
            .position;
        let rest_length = (DVec3::from(a_pos) - DVec3::from(b_pos)).length();
        if !rest_length.is_finite()
            || rest_length <= 0.0
            || !compliance.is_finite()
            || compliance < 0.0
        {
            return Err(XpbdError::InvalidConstraint);
        }
        Ok(Self {
            a,
            b,
            rest_length,
            compliance,
        })
    }
}

impl SoftBody {
    /// Install a two-sided triangulated collision surface. Unreferenced particles
    /// remain independent point probes. One-ring material neighbours are excluded.
    /// This requires explicit self-contact and a positive particle radius.
    pub fn with_self_contact_surface(
        mut self,
        triangles: Vec<[usize; 3]>,
    ) -> Result<Self, XpbdError> {
        if !self.config.self_contact {
            return Err(XpbdError::InvalidConfig);
        }
        self.self_contact = None;
        self.pads = None;
        self.self_impulse = SelfContactImpulse::default();
        self.surface = Some(super::surface::Surface::new(triangles, &self.particles)?);
        Ok(self)
    }

    /// Apply Coulomb friction between the two physical skins at swept
    /// material contacts. Positive friction requires a fully triangulated
    /// shell with a well-defined angular velocity at every incident feature.
    pub fn with_self_contact_friction(mut self, coefficient: f64) -> Result<Self, XpbdError> {
        self.surface
            .as_mut()
            .ok_or(XpbdError::InvalidConfig)?
            .set_friction(coefficient)?;
        Ok(self)
    }

    /// Resolve finite spherical pads against the continuous shell skin. Requires
    /// self-contact triangles and a single shared soft body in the coupled solve.
    pub fn with_pad_surface_contact(mut self) -> Result<Self, XpbdError> {
        if self.capsule_material.is_some() {
            return Err(XpbdError::InvalidConfig);
        }
        let faces = self
            .surface
            .as_ref()
            .ok_or(XpbdError::InvalidConfig)?
            .pad_faces()?;
        self.pads = Some(super::pad::Pads::new(faces)?);
        Ok(self)
    }

    /// Consecutive particles resist stretch. Two-edge chords approximate bending;
    /// they do not constrain torsion or signed bending angle.
    pub fn cable(
        particles: Vec<Particle>,
        stretch_compliance: f64,
        bend_compliance: f64,
        config: SolverConfig,
    ) -> Result<Self, XpbdError> {
        if particles.len() < 2 || !bend_compliance.is_finite() || bend_compliance < 0.0 {
            return Err(XpbdError::InvalidConstraint);
        }
        let mut c = Vec::new();
        for i in 1..particles.len() {
            c.push(DistanceConstraint::between(
                i - 1,
                i,
                &particles,
                stretch_compliance,
            )?);
        }
        for i in 2..particles.len() {
            c.push(DistanceConstraint::between(
                i - 2,
                i,
                &particles,
                bend_compliance,
            )?);
        }
        let mut body = Self::new(particles, c, config)?;
        body.topology = MaterialTopology::Cable;
        Ok(body)
    }

    /// Row-major rectangular grid. Structural constraints follow rows and columns;
    /// a shared diagonal per cell resists shear. Every interior triangle edge
    /// resists changes in signed dihedral angle from its construction shape.
    /// Bending compliance is radians per newton metre per hinge.
    pub fn cloth(
        particles: Vec<Particle>,
        columns: usize,
        stretch_compliance: f64,
        shear_compliance: f64,
        bend_compliance: f64,
        config: SolverConfig,
    ) -> Result<Self, XpbdError> {
        if columns < 2
            || !particles.len().is_multiple_of(columns)
            || particles.len() / columns < 2
            || !bend_compliance.is_finite()
            || bend_compliance < 0.0
        {
            return Err(XpbdError::InvalidConstraint);
        }
        let rows = particles.len() / columns;
        if config.self_contact {
            super::surface::Surface::validate_size(
                particles.len(),
                2 * (rows - 1) * (columns - 1),
            )?;
        }
        let mut faces = Vec::new();
        let mut hinges = Vec::new();
        let mut open_edges = std::collections::HashMap::new();
        let mut c = Vec::new();
        let mut edge = |a, b, compliance| -> Result<(), XpbdError> {
            c.push(DistanceConstraint::between(a, b, &particles, compliance)?);
            Ok(())
        };
        for row in 0..rows {
            for column in 0..columns {
                let i = row * columns + column;
                if column >= 1 {
                    edge(i - 1, i, stretch_compliance)?;
                }
                if row >= 1 {
                    edge(i - columns, i, stretch_compliance)?;
                }
                if row >= 1 && column >= 1 {
                    edge(i - columns - 1, i, shear_compliance)?;
                    for [a, b, c] in cell_faces(i, columns) {
                        if config.self_contact {
                            faces.push([a, b, c]);
                        }
                        for (u, v, opposite) in [(a, b, c), (b, c, a), (c, a, b)] {
                            if let Some(other) = open_edges.remove(&(v, u)) {
                                hinges.push(DihedralConstraint::from_rest(
                                    [u, v, opposite, other],
                                    &particles,
                                    bend_compliance,
                                )?);
                            } else {
                                open_edges.insert((u, v), opposite);
                            }
                        }
                    }
                }
            }
        }
        let mut body = Self::new(particles, c, config)?;
        body.topology = MaterialTopology::Cloth(columns);
        body.bending_lambdas.resize(hinges.len(), 0.0);
        body.bending = hinges;
        if config.self_contact {
            body.with_self_contact_surface(faces)
        } else {
            Ok(body)
        }
    }
}
