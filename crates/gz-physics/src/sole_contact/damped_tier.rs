// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Uniform normal pressure permits one material evaluation per RK stage.
use super::*;
impl SoleBody {
    /// Same midpoint time grid as the fixed 3D solver, with a spatially uniform
    /// damped reaction. Compute the complete pressure polygon at the public endpoint.
    #[allow(clippy::too_many_arguments)]
    pub fn try_flat_damped_step(
        &self,
        state: &mut SoleState,
        deck: &SoleDeck,
        material: &SoleMaterial,
        gravity: DVec3,
        dt: f64,
        substeps: u32,
        force: DVec3,
        torque: DVec3,
    ) -> Result<Option<SoleStep>, SoleError> {
        if substeps == 0 || 0.5 * (dt / f64::from(substeps)) == 0. {
            return Err(SoleError::InvalidInput);
        }
        let Some((mut s, acc, touch_z)) =
            self.centered_normal_state(state, deck, gravity, force, torque, dt)?
        else {
            return Ok(None);
        };
        if material.alpha == 0. {
            return Ok(None);
        }
        let area = 4. * self.half_extents.x * self.half_extents.y;
        let eval = |z: f64, v: f64| {
            let depth = touch_z - z;
            if depth > self.half_extents.z {
                return Err(SoleError::OutsideCoating);
            }
            let point = material
                .damped
                .sample(depth, -v)
                .ok_or(SoleError::Nonfinite)?;
            Ok((point.pressure_pa * area, point.dissipation_w_m2 * area))
        };
        let mut out = SoleStep::default();
        let end = deck.center + dt * deck.velocity;
        if let Some(end) =
            self.free_world(*state, *deck, gravity + force / self.mass, dt, torque)?
        {
            *state = end;
            return Ok(Some(out));
        }
        {
            let h = dt / f64::from(substeps);
            for _ in 0..substeps {
                let (a, _) = eval(s.center.z, s.velocity.z)?;
                let z = s.center.z + 0.5 * h * s.velocity.z;
                let v = s.velocity.z + 0.5 * h * (acc.z + a / self.mass);
                let (b, power) = eval(z, v)?;
                let start_z = s.center.z;
                let start_v = s.velocity.z;
                s.center.z += h * v;
                s.velocity.z += h * (acc.z + b / self.mass);
                if start_z >= touch_z
                    && z >= touch_z
                    && s.center.z >= touch_z
                    && acc.z > 0.
                    && start_v < 0.
                {
                    let lowest_time = (-start_v / acc.z).clamp(0., h);
                    if start_z + lowest_time * (start_v + 0.5 * acc.z * lowest_time) < touch_z {
                        out.possible_missed_contacts += 1;
                    }
                }
                out.normal_impulse += h * b;
                out.dissipated_energy += h * power;
                out.peak_normal_force = out.peak_normal_force.max(a).max(b);
            }
        }
        out.impulse = deck.orientation * (DVec3::Z * out.normal_impulse);
        out.contact = self
            .patch(s, deck.half_extents, *material, 0)?
            .world(*deck, end);
        out.peak_normal_force = out.peak_normal_force.max(out.contact.normal_force);
        let world = s.world(*deck, end);
        if !world.finite()
            || !out.contact.finite()
            || !out.impulse.is_finite()
            || !out.peak_normal_force.is_finite()
            || !out.dissipated_energy.is_finite()
        {
            return Err(SoleError::Nonfinite);
        }
        *state = world;
        Ok(Some(out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scalar_and_spatial_grids_agree_during_unloading_and_elevator_landing() {
        let b = SoleBody {
            half_extents: DVec3::new(0.2, 0.1, 0.05),
            mass: 2.,
            foot_offset: DVec3::Z * -0.01,
            inertia: DVec3::splat(0.04),
        };
        let mat = SoleMaterial::new(2e7, 8e6, 1.2, 0.7, 0.002).unwrap();
        for (height, speed) in [(0.13, -1.2), (0.107, 2.), (0.108, -0.3)] {
            let mut d = SoleDeck {
                center: DVec3::ZERO,
                half_extents: DVec3::new(0.3, 0.2, 0.05),
                orientation: DQuat::IDENTITY,
                velocity: 0.4 * DVec3::Z,
            };
            let mut scalar = SoleState {
                center: height * DVec3::Z,
                orientation: DQuat::from_rotation_z(0.2),
                velocity: d.velocity + speed * DVec3::Z,
                spin: DVec3::ZERO,
            };
            let mut full = scalar;
            for _ in 0..70 {
                let h = 0.0005;
                let fast = b
                    .try_flat_damped_step(
                        &mut scalar,
                        &d,
                        &mat,
                        -9.81 * DVec3::Z,
                        h,
                        64,
                        DVec3::ZERO,
                        DVec3::ZERO,
                    )
                    .unwrap()
                    .unwrap();
                let fine = b
                    .step(&mut full, &d, &mat, -9.81 * DVec3::Z, h, 64, 1)
                    .unwrap();
                assert!(scalar.center.distance(full.center) < 1e-11);
                assert!(scalar.velocity.distance(full.velocity) < 1e-10);
                assert!((fast.normal_impulse - fine.normal_impulse).abs() < 1e-11);
                assert!((fast.dissipated_energy - fine.dissipated_energy).abs() < 1e-11);
                assert!((fast.contact.strain_energy - fine.contact.strain_energy).abs() < 1e-11);
                d.center += h * d.velocity;
            }
            let before = scalar;
            assert!(
                b.try_flat_damped_step(
                    &mut scalar,
                    &d,
                    &mat,
                    DVec3::ZERO,
                    0.01,
                    64,
                    DVec3::X,
                    DVec3::ZERO
                )
                .unwrap()
                .is_none()
            );
            assert_eq!(before.center, scalar.center);
        }
    }
}
