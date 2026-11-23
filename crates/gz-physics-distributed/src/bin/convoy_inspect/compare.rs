// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
use gz_physics_distributed::multi_sphere::{InspectedCut, MultiSphereObservation};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub(super) struct Header<'a> {
    sha256: &'a str,
    pub epoch: u64,
    pub tick: u64,
    generation: u64,
    width: u8,
    owners: [u8; 3],
    pending_effort_nm: Option<f64>,
    hinge_native_sha256: &'a str,
    pair_native_sha256: Option<&'a str>,
}

impl<'a> From<&'a InspectedCut> for Header<'a> {
    fn from(c: &'a InspectedCut) -> Self {
        Self {
            sha256: &c.sha256,
            epoch: c.cut.epoch,
            tick: c.cut.observation.hinge.tick,
            generation: c.cut.generation,
            width: c.cut.width,
            owners: c.cut.owners,
            pending_effort_nm: c.cut.pending_effort,
            hinge_native_sha256: &c.hinge_native_sha256,
            pair_native_sha256: c.pair_native_sha256.as_deref(),
        }
    }
}

#[derive(Debug, Default, Serialize)]
pub(super) struct PhysicalErrors {
    pub max_position_m: f64,
    pub max_rotation_rad: f64,
    pub max_linear_velocity_m_s: f64,
    pub max_angular_velocity_rad_s: f64,
    pub joint_position_rad: f64,
    pub joint_velocity_rad_s: f64,
    pub max_contact_normal_vector_error: f64,
    pub max_normal_impulse_n_s: f64,
    pub max_warmstart_normal_impulse_n_s: f64,
    pub max_tangent_impulse_n_s: f64,
    pub active_contact_changes: u8,
    pub loaded_contact_changes: u8,
}

fn finite(v: f64) -> Result<f64> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err("unrepresentable physical comparison".into())
    }
}

fn distance<const N: usize>(a: [f64; N], b: [f64; N]) -> Result<f64> {
    finite(a.into_iter().zip(b).fold(0f64, |d, (x, y)| d.hypot(x - y)))
}

fn unit(q: [f64; 4]) -> Result<[f64; 4]> {
    let length = distance(q, [0.; 4])?;
    if length == 0. {
        return Err("a zero quaternion has no physical orientation".into());
    }
    Ok(q.map(|x| x / length))
}

fn rotation(q: [f64; 4], r: [f64; 4]) -> Result<f64> {
    let q = unit(q)?;
    let r = unit(r)?;
    // Both quaternion signs describe the same pose. The two half-angle
    // chords retain small rotations that acos(abs(dot(q,r))) rounds away.
    let minus = distance(q, r)?;
    let plus = distance(q, r.map(|x| -x))?;
    Ok(4. * minus.min(plus).atan2(minus.max(plus)))
}

impl PhysicalErrors {
    fn between(a: &MultiSphereObservation, b: &MultiSphereObservation) -> Result<Option<Self>> {
        let bodies = |o: &MultiSphereObservation| {
            [
                o.hinge.links[0],
                o.hinge.links[1],
                o.spheres[0],
                o.spheres[1],
            ]
        };
        let left = bodies(a);
        let right = bodies(b);
        if a.hinge.joint_id != b.hinge.joint_id || left.map(|l| l.id) != right.map(|r| r.id) {
            return Ok(None);
        }
        let mut out = Self {
            joint_position_rad: finite((a.hinge.q - b.hinge.q).abs())?,
            joint_velocity_rad_s: finite((a.hinge.qdot - b.hinge.qdot).abs())?,
            ..Default::default()
        };
        for (l, r) in left.into_iter().zip(right) {
            out.max_position_m = out.max_position_m.max(distance(l.position, r.position)?);
            out.max_linear_velocity_m_s = out
                .max_linear_velocity_m_s
                .max(distance(l.velocity, r.velocity)?);
            out.max_angular_velocity_rad_s = out
                .max_angular_velocity_rad_s
                .max(distance(l.angular_velocity, r.angular_velocity)?);
            out.max_rotation_rad = out
                .max_rotation_rad
                .max(rotation(l.orientation, r.orientation)?);
        }
        // The four hinge-sphere contacts and the sphere pair use fixed roles
        // and signed world-space directions in both public observations.
        let reactions = |o: &MultiSphereObservation| {
            [
                o.reactions[0][0],
                o.reactions[0][1],
                o.reactions[1][0],
                o.reactions[1][1],
                o.sphere_reaction,
            ]
        };
        for (l, r) in reactions(a).into_iter().zip(reactions(b)) {
            out.active_contact_changes += u8::from(l.active != r.active);
            out.loaded_contact_changes +=
                u8::from((l.normal_impulse > 0.) != (r.normal_impulse > 0.));
            out.max_contact_normal_vector_error = out
                .max_contact_normal_vector_error
                .max(distance(l.normal, r.normal)?);
            out.max_normal_impulse_n_s = out
                .max_normal_impulse_n_s
                .max(finite((l.normal_impulse - r.normal_impulse).abs())?);
            out.max_warmstart_normal_impulse_n_s = out.max_warmstart_normal_impulse_n_s.max(
                finite((l.warmstart_normal_impulse - r.warmstart_normal_impulse).abs())?,
            );
            out.max_tangent_impulse_n_s = out
                .max_tangent_impulse_n_s
                .max(distance(l.tangent_impulse, r.tangent_impulse)?);
        }
        Ok(Some(out))
    }

    fn identical(&self) -> bool {
        self.max_position_m == 0.
            && self.max_rotation_rad == 0.
            && self.max_linear_velocity_m_s == 0.
            && self.max_angular_velocity_rad_s == 0.
            && self.joint_position_rad == 0.
            && self.joint_velocity_rad_s == 0.
            && self.max_contact_normal_vector_error == 0.
            && self.max_normal_impulse_n_s == 0.
            && self.max_warmstart_normal_impulse_n_s == 0.
            && self.max_tangent_impulse_n_s == 0.
            && self.active_contact_changes == 0
            && self.loaded_contact_changes == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum Classification {
    IdenticalArchive,
    ClockMismatch,
    RecipeMismatch,
    BodyOrJointIdentityMismatch,
    PhysicalDivergence,
    PendingInputChange,
    PlacementChange,
    GenerationChange,
    ObservationRepresentationChange,
    NativeStateChange,
    OtherArchiveDifference,
}

#[derive(Debug, Serialize)]
pub(super) struct Equality {
    pub archive: bool,
    pub recipe: bool,
    pub epoch: bool,
    pub tick: bool,
    pub generation: bool,
    pub width: bool,
    pub owners: bool,
    pub pending_input: bool,
    pub observation_representation: bool,
    pub physical_observations: Option<bool>,
    pub hinge_native_image: bool,
    pub pair_native_image: bool,
}

#[derive(Debug, Serialize)]
pub(super) struct Comparison<'a> {
    pub left: Header<'a>,
    pub right: Header<'a>,
    pub classification: Classification,
    pub equal: Equality,
    pub physical_errors: Option<PhysicalErrors>,
}

impl<'a> Comparison<'a> {
    pub fn new(a: &'a InspectedCut, b: &'a InspectedCut) -> Result<Self> {
        let left = Header::from(a);
        let right = Header::from(b);
        let same_clock = (left.epoch, left.tick) == (right.epoch, right.tick);
        let physical_errors = if same_clock {
            PhysicalErrors::between(&a.cut.observation, &b.cut.observation)?
        } else {
            None
        };
        let equal = Equality {
            archive: left.sha256 == right.sha256,
            recipe: serde_json::to_vec(&a.cut.spec)? == serde_json::to_vec(&b.cut.spec)?,
            epoch: left.epoch == right.epoch,
            tick: left.tick == right.tick,
            generation: left.generation == right.generation,
            width: left.width == right.width,
            owners: left.owners == right.owners,
            pending_input: left.pending_effort_nm.map(f64::to_bits)
                == right.pending_effort_nm.map(f64::to_bits),
            observation_representation: serde_json::to_vec(&a.cut.observation)?
                == serde_json::to_vec(&b.cut.observation)?,
            physical_observations: physical_errors.as_ref().map(PhysicalErrors::identical),
            hinge_native_image: left.hinge_native_sha256 == right.hinge_native_sha256,
            pair_native_image: left.pair_native_sha256 == right.pair_native_sha256,
        };
        use Classification as C;
        let classification = if equal.archive {
            C::IdenticalArchive
        } else if !same_clock {
            C::ClockMismatch
        } else if !equal.recipe {
            C::RecipeMismatch
        } else if physical_errors.is_none() {
            C::BodyOrJointIdentityMismatch
        } else if equal.physical_observations == Some(false) {
            C::PhysicalDivergence
        } else if !equal.pending_input {
            C::PendingInputChange
        } else if !equal.width || !equal.owners {
            C::PlacementChange
        } else if !equal.generation {
            C::GenerationChange
        } else if !equal.observation_representation {
            C::ObservationRepresentationChange
        } else if !equal.hinge_native_image || !equal.pair_native_image {
            C::NativeStateChange
        } else {
            C::OtherArchiveDifference
        };
        Ok(Self {
            left,
            right,
            classification,
            equal,
            physical_errors,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angles_handle_quaternion_sign_microrotations_and_half_turns() {
        let id = [0., 0., 0., 1.];
        assert_eq!(rotation(id, [0., 0., 0., -1.]).unwrap(), 0.);
        let tiny: f64 = 1e-12;
        let rotated = [0., 0., (tiny / 2.).sin(), (tiny / 2.).cos()];
        assert!((rotation(id, rotated).unwrap() - tiny).abs() < 1e-25);
        assert_eq!(
            rotation(id, [1., 0., 0., 0.]).unwrap(),
            std::f64::consts::PI
        );
        assert!(rotation(id, [0.; 4]).is_err());
        assert!(distance([f64::NAN], [1.]).is_err());
    }
}
