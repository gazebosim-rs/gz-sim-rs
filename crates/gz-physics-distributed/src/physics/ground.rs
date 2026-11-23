// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_multifidelity::NativeGroundPlane;

/// Immutable halfspace shared by every worker. The solid is on the inward side
/// of its unit world-space normal through its reference point.
#[derive(Clone, Copy, Debug)]
pub struct GroundPlane(NativeGroundPlane);

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Material {
    height: f64,
    friction: f64,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Oriented {
    point: [f64; 3],
    normal: [f64; 3],
    friction: f64,
}

// The canonical quiet NaN prefixes legacy horizontal material records.
// A distinct payload adds oriented records without changing finite elevations.
const ORIENTED_TAG: u64 = 0x7ff8_0000_0000_0001;
impl Serialize for GroundPlane {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let g = self.0;
        if self.horizontal() {
            if g.friction.to_bits() == 0 {
                return g.point[2].serialize(serializer);
            }
            return if serializer.is_human_readable() {
                Material {
                    height: g.point[2],
                    friction: g.friction,
                }
                .serialize(serializer)
            } else {
                (f64::NAN, g.point[2], g.friction).serialize(serializer)
            };
        }
        if serializer.is_human_readable() {
            Oriented {
                point: g.point,
                normal: g.normal,
                friction: g.friction,
            }
            .serialize(serializer)
        } else {
            (f64::from_bits(ORIENTED_TAG), g.point, g.normal, g.friction).serialize(serializer)
        }
    }
}
impl<'de> Deserialize<'de> for GroundPlane {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use serde::de::{Error, SeqAccess, Visitor};
        if deserializer.is_human_readable() {
            #[derive(Deserialize)]
            #[serde(untagged)]
            enum Wire {
                Height(f64),
                Surface(Material),
                Oriented(Oriented),
            }
            return match Wire::deserialize(deserializer)? {
                Wire::Height(h) => Self::new(h),
                Wire::Surface(m) => Self::with_friction(m.height, m.friction),
                Wire::Oriented(g) => Self::oriented(g.point, g.normal, g.friction),
            }
            .map_err(Error::custom);
        }
        struct Binary;
        impl<'de> Visitor<'de> for Binary {
            type Value = GroundPlane;
            fn expecting(&self, fmt: &mut std::fmt::Formatter) -> std::fmt::Result {
                fmt.write_str("a finite elevation or tagged plane")
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<GroundPlane, A::Error> {
                let mut number = || {
                    seq.next_element::<f64>()?
                        .ok_or_else(|| Error::custom("missing ground scalar"))
                };
                let tag = number()?;
                if tag.to_bits() == ORIENTED_TAG {
                    GroundPlane::oriented(
                        [number()?, number()?, number()?],
                        [number()?, number()?, number()?],
                        number()?,
                    )
                } else if tag.to_bits() == f64::NAN.to_bits() {
                    GroundPlane::with_friction(number()?, number()?)
                } else {
                    GroundPlane::new(tag)
                }
                .map_err(Error::custom)
            }
        }
        deserializer.deserialize_tuple(8, Binary)
    }
}
impl GroundPlane {
    pub fn new(height: f64) -> Result<Self, PhysicsError> {
        Self::with_friction(height, 0.)
    }
    pub fn with_friction(height: f64, friction: f64) -> Result<Self, PhysicsError> {
        Self::oriented([0., 0., height], [0., 0., 1.], friction)
    }
    pub fn oriented(
        point: [f64; 3],
        normal: [f64; 3],
        friction: f64,
    ) -> Result<Self, PhysicsError> {
        NativeGroundPlane::new(point, normal, friction).map(Self)
    }
    /// Elevation of the reference point, constant across a horizontal surface.
    pub fn height(self) -> f64 {
        self.0.point[2]
    }
    pub fn point(self) -> [f64; 3] {
        self.0.point
    }
    pub fn normal(self) -> [f64; 3] {
        self.0.normal
    }
    pub fn friction(self) -> f64 {
        self.0.friction
    }
    pub(crate) fn horizontal(self) -> bool {
        self.0.normal == [0., 0., 1.]
    }
    pub(crate) fn bits(self) -> [u64; 7] {
        self.0.bits()
    }
    pub(crate) fn admits(self, body: &Body) -> Result<(), WorldError> {
        if !body.bounds()?.above_plane(self.point(), self.normal()) {
            return Err(PhysicsError::TransitionDeferred {
                reason: "box intersects the shared ground",
            }
            .into());
        }
        Ok(())
    }
}
impl PartialEq for GroundPlane {
    fn eq(&self, other: &Self) -> bool {
        self.bits() == other.bits()
    }
}

/// Normal impulse (N·s) on a mobile box from the shared ground at a barrier.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundSupport {
    pub body: u64,
    pub normal_impulse: f64,
}
impl PartialEq for GroundSupport {
    fn eq(&self, other: &Self) -> bool {
        self.body == other.body && self.normal_impulse.to_bits() == other.normal_impulse.to_bits()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn legacy_ground_digest_bytes_are_preserved_and_material_encodings_are_unambiguous() {
        let legacy = GroundPlane::new(-0.).unwrap();
        let original = bincode::serialize(&(-0f64, 874_u64)).unwrap();
        assert_eq!(bincode::serialize(&(legacy, 874_u64)).unwrap(), original);
        assert_eq!(
            bincode::deserialize::<(GroundPlane, u64)>(&original).unwrap(),
            (legacy, 874)
        );
        assert_eq!(serde_json::to_string(&legacy).unwrap(), "-0.0");
        for friction in [0.25, 1., -0.] {
            let g = GroundPlane::with_friction(0.1, friction).unwrap();
            let binary = bincode::serialize(&(g, 31_u32)).unwrap();
            assert_eq!(binary.len(), 28);
            assert_eq!(
                bincode::deserialize::<(GroundPlane, u32)>(&binary).unwrap(),
                (g, 31)
            );
            let json = serde_json::to_string(&g).unwrap();
            assert_eq!(serde_json::from_str::<GroundPlane>(&json).unwrap(), g);
        }
        for invalid in [
            r#"{"height":0.1,"friction":-0.1}"#,
            r#"{"height":0.1,"friction":0.4,"mass":1}"#,
            "null",
        ] {
            assert!(serde_json::from_str::<GroundPlane>(invalid).is_err());
        }
        assert!(bincode::deserialize::<GroundPlane>(&f64::NAN.to_le_bytes()).is_err());
        assert!(bincode::deserialize::<GroundPlane>(&f64::INFINITY.to_le_bytes()).is_err());
    }
}
