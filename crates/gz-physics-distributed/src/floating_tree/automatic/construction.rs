// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

mod sealed {
    use super::*;
    pub trait Profile: Sized {
        const WIRE: [u8; 8];
        const MAGIC: [u8; 8];
        const FILE: &'static str;
        const FLAG: &'static str;
        fn tree(&self) -> Option<&ContactTreeSpec> {
            None
        }
        fn rotor(&self) -> Option<&RotorPadSpec> {
            None
        }
        fn validate_native(&self) -> Result<()>;
        fn sorted_joints(&self) -> BTreeSet<u64>;
        fn dimensions(&self) -> (usize, usize);
        fn image_admitted(&self, b: &[u8]) -> bool;
    }
}
pub(super) use sealed::Profile;
/// An admitted physical construction for an automatic, two-way material impact.
pub trait AutomaticConstruction:
    Profile + PartialEq + Clone + Serialize + serde::de::DeserializeOwned + Send + Sync + 'static
{
    fn world_supported(&self) -> bool {
        self.rotor().is_some()
    }
    fn interval_ns(&self) -> u64;
    fn receivers(&self) -> &[u64];
    fn coefficient(&self) -> f64;
    fn material(&self) -> &str;
    fn link_specs(&self) -> &[ArticulationLinkSpec];
    fn link_index(&self, id: u64) -> Result<usize> {
        self.link_specs()
            .binary_search_by_key(&id, |v| v.id)
            .map_err(|_| invalid("unknown automatic material link"))
    }
}
impl AutomaticConstruction for AutomaticMaterialSpec {
    fn interval_ns(&self) -> u64 {
        self.step_ns
    }
    fn receivers(&self) -> &[u64] {
        &self.links
    }
    fn coefficient(&self) -> f64 {
        self.restitution
    }
    fn material(&self) -> &str {
        &self.material_name
    }
    fn link_specs(&self) -> &[ArticulationLinkSpec] {
        &self.tree.tree.links
    }
}
impl Profile for AutomaticMaterialSpec {
    const WIRE: [u8; 8] = WIRE;
    const MAGIC: [u8; 8] = MAGIC;
    const FILE: &'static str = FILE;
    const FLAG: &'static str = "--managed-timed-material-v1";
    fn tree(&self) -> Option<&ContactTreeSpec> {
        Some(&self.tree)
    }
    fn sorted_joints(&self) -> BTreeSet<u64> {
        self.tree.tree.joints.iter().map(|v| v.joint_id).collect()
    }
    fn dimensions(&self) -> (usize, usize) {
        let n = self.tree.tree.links.len();
        (n, n + 5)
    }
    fn image_admitted(&self, b: &[u8]) -> bool {
        b.len() <= gz_physics_multifidelity::FLOATING_TREE_IMAGE_LIMIT + 48
            && b.starts_with(b"GZFTIM01")
    }
    fn validate_native(&self) -> Result<()> {
        self.validate_tree()
    }
}
impl AutomaticConstruction for AutomaticRotorMaterialSpec {
    fn interval_ns(&self) -> u64 {
        self.step_ns
    }
    fn receivers(&self) -> &[u64] {
        std::slice::from_ref(&self.pad.link.id)
    }
    fn coefficient(&self) -> f64 {
        self.restitution
    }
    fn material(&self) -> &str {
        &self.material_name
    }
    fn link_specs(&self) -> &[ArticulationLinkSpec] {
        std::slice::from_ref(&self.pad.link)
    }
}
impl Profile for AutomaticRotorMaterialSpec {
    const WIRE: [u8; 8] = *b"GZROTW01";
    const MAGIC: [u8; 8] = *b"GZROTC01";
    const FILE: &'static str = crate::articulation::ledger::ROTOR_MATERIAL_FILE;
    const FLAG: &'static str = "--managed-rotor-material-v1";
    fn rotor(&self) -> Option<&RotorPadSpec> {
        Some(&self.pad)
    }
    fn sorted_joints(&self) -> BTreeSet<u64> {
        [self.pad.joint_id].into_iter().collect()
    }
    // Native multibody link count includes the synthetic fixed world support.
    fn dimensions(&self) -> (usize, usize) {
        (2, 1)
    }
    fn image_admitted(&self, b: &[u8]) -> bool {
        b.len() <= 512 && b.starts_with(b"GZRTOI01")
    }
    fn validate_native(&self) -> Result<()> {
        let p = &self.pad;
        let l = &p.link;
        if !(1..=1_000_000_000).contains(&self.step_ns)
            || self.material_name.is_empty()
            || self.material_name.len() > 256
            || !(0.0..=1.).contains(&self.restitution)
            || [p.model_id, l.id, p.joint_id]
                .into_iter()
                .collect::<BTreeSet<_>>()
                .len()
                != 3
            || !p.anchor.iter().chain(&p.axis).all(|x| x.is_finite())
            || (glam::DVec3::from(p.axis).length_squared() - 1.).abs() > 1e-9
            || !p.lower.is_finite()
            || !p.upper.is_finite()
            || !(p.lower < 0. && 0. < p.upper)
            || !l.radius.is_finite()
            || l.radius <= 0.
            || !l.mass.is_finite()
            || l.mass <= 0.
        {
            return Err(invalid(
                "invalid finite world-revolute material construction",
            ));
        }
        Ok(())
    }
}
pub(super) fn validate<S: AutomaticConstruction>(s: &S, cold: &SoftBody) -> Result<()> {
    s.validate_native()?;
    if cold.particles().len() > 256
        || !cold.supports_articulated_impact()
        || cold.config().gravity != [0.; 3]
    {
        return Err(invalid("unsupported automatic material construction"));
    }
    let motion = native::TimedWorld::new(s)?.swept_spheres()?;
    cold.first_swept_sphere_impact(
        &motion.into_shapes(s.rotor().is_some())?,
        Duration::from_nanos(s.interval_ns()),
    )?;
    Ok(())
}
