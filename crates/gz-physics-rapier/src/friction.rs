// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Rigid-contact friction policy for one native solver graph.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(
    feature = "partition-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub enum NativeFrictionModel {
    #[default]
    Grouped,
    PerPoint,
}

impl NativeFrictionModel {
    pub(crate) const fn native(self) -> FrictionModel {
        match self {
            Self::Grouped => FrictionModel::Simplified,
            Self::PerPoint => FrictionModel::Coulomb,
        }
    }
}

/// Standalone rigid-contact policy. Selection at construction avoids reusing
/// a loaded manifold's impulses under a different constraint model.
impl RapierBackend {
    /// Construct the ordinary Rapier backend with one Coulomb friction
    /// constraint per contact point. Geometry, coefficients, sleeping and
    /// contact recycling retain their ordinary settings.
    pub fn new_per_point_coulomb(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        let mut backend = Self::new(gravity)?;
        backend.world.integration_parameters.friction_model =
            NativeFrictionModel::PerPoint.native();
        Ok(backend)
    }

    /// Construct the local multi-fidelity solver with a selected rigid friction
    /// model before any narrow-phase caches or links exist.
    pub fn new_multifidelity_with_friction(
        gravity: [f64; 3],
        model: NativeFrictionModel,
    ) -> Result<Self, PhysicsError> {
        let mut backend = Self::new_multifidelity(gravity)?;
        backend.world.integration_parameters.friction_model = model.native();
        Ok(backend)
    }

    /// Construct an empty transferable solver with its selected friction policy.
    #[cfg(feature = "partition-checkpoint")]
    pub fn new_multifidelity_with_profile(
        gravity: [f64; 3],
        profile: NativeSolverProfile,
    ) -> Result<Self, PhysicsError> {
        let mut backend = Self::new_multifidelity(gravity)?;
        backend.native_profile = Some(profile);
        backend.world.integration_parameters.friction_model = profile.friction().native();
        Ok(backend)
    }

    /// Report whether native rigid contacts use the per-point Coulomb model.
    pub fn per_point_friction_enabled(&self) -> bool {
        self.world.integration_parameters.friction_model == FrictionModel::Coulomb
    }
}

#[cfg(all(test, feature = "partition-checkpoint"))]
mod tests {
    use super::*;

    #[test]
    fn native_checkpoint_and_join_validate_the_model_for_every_source() -> Result<(), PhysicsError>
    {
        fn source(x: f64) -> Result<RapierBackend, PhysicsError> {
            let mut source = RapierBackend::new_multifidelity([0.; 3])?;
            source.spawn_partition_box(
                BodySpec {
                    position: [x, 0., 0.],
                    velocity: [0.; 3],
                    mass: Some(1.),
                    shape: Shape::Box {
                        half_extents: [0.25; 3],
                    },
                },
                Default::default(),
                false,
                0.35,
            )?;
            source.step(std::time::Duration::from_millis(10))?;
            Ok(source)
        }
        let mut a = source(0.)?;
        let mut b = source(4.)?;
        for profile in [
            NativeSolverProfile::Standard32,
            NativeSolverProfile::Extended64,
        ] {
            let before = a
                .checkpoint_solver_with_profile(profile)
                .expect("valid grouped resident");
            let image = b
                .checkpoint_solver_with_profile(profile)
                .expect("valid grouped donor");
            let original_world = bincode::serialize(&b.world).expect("bounded test fixture");
            let prefix = image
                .strip_suffix(original_world.as_slice())
                .expect("native world closes the image");
            a.join_box_solvers_with_profile(&[&b], profile)
                .expect("valid disjoint baseline join");
            // Everything else stays within the native protocol. Only the model differs.
            b.world.integration_parameters.friction_model = FrictionModel::Coulomb;
            assert!(b.checkpoint_solver_with_profile(profile).is_err());
            let mut foreign = prefix.to_vec();
            foreign.extend(bincode::serialize(&b.world).expect("bounded test fixture"));
            assert!(RapierBackend::restore_solver_with_profile(&foreign, profile).is_err());
            assert!(a.join_box_solvers_with_profile(&[&b], profile).is_err());
            assert!(b.join_box_solvers_with_profile(&[&a], profile).is_err());
            a.world.integration_parameters.friction_model = FrictionModel::Coulomb;
            assert!(a.join_box_solvers_with_profile(&[&b], profile).is_err());
            for source in [&mut a, &mut b] {
                source.world.integration_parameters.friction_model = FrictionModel::Simplified;
            }
            assert_eq!(a.checkpoint_solver_with_profile(profile)?, before);
        }
        Ok(())
    }
}

#[cfg(all(test, feature = "partition-checkpoint"))]
#[path = "../tests/private/friction_identity.rs"]
mod native_identity;
