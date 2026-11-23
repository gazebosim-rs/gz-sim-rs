// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A different support plane must disprove the cached separation before requesting a native refresh.
use super::*;
use rapier3d_f64::math::{Pose, Vector};
use std::sync::{OnceLock, atomic::AtomicBool};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReturnFeatureMode {
    /// Re-evaluate only the first unloaded manifold, independently of the retained normal.
    FirstUnloaded,
    /// Re-evaluate once if a current support plane improves the cached separation by at least the tolerance.
    FirstStale,
    /// Re-evaluate once if the plane deficit exceeds one native interval of incoming surface travel.
    FirstTravel,
    /// Repeat the geometric check while the enrolled pair is unloaded during this public interval.
    WhileStale,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReturnPlaneEvidence {
    pub other_origin_in_first: [f64; 3],
    pub other_rotation_in_first: [f64; 4],
    pub half_extents: [[f64; 3]; 2],
    pub cached_axis: [f64; 3],
    pub stronger_axis: [f64; 3],
    pub gap_m: [f64; 2],
    pub tolerance_m: f64,
    pub unloaded_points: usize,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReturnFeatureCounts {
    pub unloaded_tests: u64,
    pub stronger_plane: u64,
    pub requery: u64,
    pub first_witness: Option<ReturnPlaneEvidence>,
}
pub(super) struct Witness {
    pub(super) mode: ReturnFeatureMode,
    pub(super) tolerance: f64,
    pub(super) consumed: AtomicBool,
    pub(super) unloaded: AtomicU64,
    pub(super) stale: AtomicU64,
    pub(super) queried: AtomicU64,
    pub(super) first_witness: OnceLock<ReturnPlaneEvidence>,
}
impl Witness {
    pub(super) fn new(mode: ReturnFeatureMode, tolerance: f64) -> Self {
        Self {
            mode,
            tolerance,
            consumed: AtomicBool::new(false),
            unloaded: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            queried: AtomicU64::new(0),
            first_witness: OnceLock::new(),
        }
    }
    pub(super) fn counts(&self) -> ReturnFeatureCounts {
        ReturnFeatureCounts {
            unloaded_tests: self.unloaded.load(Ordering::Relaxed),
            stronger_plane: self.stale.load(Ordering::Relaxed),
            requery: self.queried.load(Ordering::Relaxed),
            first_witness: self.first_witness.get().copied(),
        }
    }
    pub(super) fn admit(
        &self,
        pose: &Pose,
        a: &dyn NativeShape,
        b: &dyn NativeShape,
        m: &ContactManifold,
    ) -> bool {
        if self.consumed.load(Ordering::Relaxed) {
            return false;
        }
        // Never request replacement geometry on a loaded patch or erase a retained multiplier.
        if m.points.iter().any(|p| {
            let c = &p.data;
            c.impulse != 0.
                || c.warmstart_impulse != 0.
                || c.tangent_impulse.iter().any(|v| *v != 0.)
                || c.warmstart_tangent_impulse.iter().any(|v| *v != 0.)
                || c.warmstart_tangent_world != Vector::ZERO
                || c.warmstart_twist_impulse != 0.
        }) {
            return false;
        }
        self.unloaded.fetch_add(1, Ordering::Relaxed);
        if self.mode != ReturnFeatureMode::FirstUnloaded {
            let (Some(a), Some(b)) = (a.as_cuboid(), b.as_cuboid()) else {
                return false;
            };
            let n = m.local_n1;
            if !n.is_finite() || (n.length_squared() - 1.).abs() > 1e-6 {
                return false;
            }
            let Some((axis, old, new)) =
                stronger_plane(pose, a.half_extents, b.half_extents, n, self.tolerance)
            else {
                return false;
            };
            self.first_witness.get_or_init(|| ReturnPlaneEvidence {
                other_origin_in_first: pose.translation.to_array(),
                other_rotation_in_first: pose.rotation.to_array(),
                half_extents: [a.half_extents.to_array(), b.half_extents.to_array()],
                cached_axis: n.to_array(),
                stronger_axis: axis.to_array(),
                gap_m: [old, new],
                tolerance_m: self.tolerance,
                unloaded_points: m.points.len(),
            });
            self.stale.fetch_add(1, Ordering::Relaxed);
        }
        self.mode == ReturnFeatureMode::WhileStale || !self.consumed.swap(true, Ordering::Relaxed)
    }
}
// Every projection gap is a lower bound on the separation of the two finite boxes.
// Return on the first axis that proves a worse-than-tolerance cached witness.
// No preference among ties is needed and a separating corner-to-corner normal can pass.
fn stronger_plane(
    p: &Pose,
    a: Vector,
    b: Vector,
    n: Vector,
    tol: f64,
) -> Option<(Vector, f64, f64)> {
    let local = p.rotation.inverse();
    let support = |d: Vector| a.dot(d.abs()) + b.dot((local * d).abs());
    let old = p.translation.dot(n) - support(n);
    let beats = |d: Vector| {
        let sign = if p.translation.dot(d) < 0. { -1. } else { 1. };
        let axis = sign * d;
        let gap = p.translation.dot(axis) - support(axis);
        (gap > old + tol).then_some((axis, old, gap))
    };
    let basis = [Vector::Z, Vector::X, Vector::Y];
    let other = basis.map(|d| p.rotation * d);
    if let Some(w) = basis.into_iter().chain(other).find_map(beats) {
        return Some(w);
    }
    for u in basis {
        for v in other {
            let axis = u.cross(v);
            let norm = axis.length_squared();
            if norm > 1e-20
                && let Some(w) = beats(axis / norm.sqrt())
            {
                return Some(w);
            }
        }
    }
    None
}
#[cfg(all(test, feature = "landing-audit"))]
mod tests {
    use super::*;
    #[test]
    fn cached_plane_must_be_disproved_by_a_real_support_axis() {
        let p = Pose::from_translation(Vector::Z * 2.00005);
        let half = Vector::ONE;
        assert!(stronger_plane(&p, half, half, Vector::Z, 1e-5).is_none());
        assert!(
            stronger_plane(
                &p,
                half,
                half,
                (Vector::Z + 0.012 * Vector::X).normalize(),
                1e-5
            )
            .is_some()
        );
        let diagonal = Pose::from_translation(Vector::splat(2.00005));
        assert!(stronger_plane(&diagonal, half, half, Vector::ONE.normalize(), 1e-5).is_none());
    }

    #[test]
    fn loaded_pair_and_foreign_rows_reproduce_the_unmodified_public_step() {
        fn setup() -> (RapierBackend, [BodyId; 3]) {
            let mut w = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
            let make = |x, z, half_extents| BodySpec {
                position: [x, 0., z],
                velocity: [0.; 3],
                mass: Some(1.),
                shape: Shape::Box { half_extents },
            };
            let mut feet = vec![];
            for x in [-0.23, 0.23] {
                let foot = w
                    .spawn_articulation_link(
                        make(x, 0.04, [0.12, 0.08, 0.04]),
                        RotationSpec::default(),
                        None,
                    )
                    .unwrap();
                let upper = w
                    .spawn_articulation_link(
                        make(x, 0.17, [0.04, 0.04, 0.05]),
                        RotationSpec::default(),
                        None,
                    )
                    .unwrap();
                w.add_limited_revolute(Some(foot), upper, [x, 0., 0.105], [1., 0., 0.], [0.; 2])
                    .unwrap();
                w.enable_exact_revolute_locks(foot).unwrap();
                // Use the same enrolled pair-filter path as the measured returning articulation.
                let collider = w.world.bodies[w.bodies[&foot].0].colliders()[0];
                w.world.colliders[collider].set_active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS);
                feet.push(foot);
            }
            let pad = w
                .spawn_velocity_boundary(
                    make(0., -0.06, [0.8, 0.4, 0.06]),
                    RotationSpec::default(),
                    None,
                )
                .unwrap();
            for _ in 0..100 {
                w.step(Duration::from_micros(500)).unwrap();
            }
            (w, [feet[0], feet[1], pad])
        }
        let (mut expected, ids) = setup();
        let (mut checked, b) = setup();
        assert_eq!(ids, b);
        let [foot, neighbor, pad] = ids;
        let rows = |w: &RapierBackend, f| {
            w.landing_solver_points(f, pad)
                .unwrap()
                .into_iter()
                .map(|r| (r.normal_on_foot, r.local_foot, r.local_pad, r.impulse))
                .collect::<Vec<_>>()
        };
        for f in [foot, neighbor] {
            let a = rows(&expected, f);
            assert!(a.iter().any(|r| r.3 > 0.));
            assert_eq!(a, rows(&checked, f));
        }
        let it = ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: Some(8),
        };
        let raw = expected
            .step_contact_interval(Duration::from_micros(500), 8, it)
            .unwrap();
        let (actual, count) = checked
            .step_returning_box_features(
                (foot, pad),
                Duration::from_micros(500),
                8,
                it,
                ReturnFeatureMode::WhileStale,
                None,
            )
            .unwrap();
        assert_eq!(raw, actual);
        assert_eq!(count.unloaded_tests, 0);
        assert_eq!(count.requery, 0);
        for f in [foot, neighbor] {
            assert_eq!(rows(&expected, f), rows(&checked, f));
            assert_eq!(expected.state(f), checked.state(f));
        }
    }
}
