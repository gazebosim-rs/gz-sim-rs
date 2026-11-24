// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Outward-rounded scalar intervals for finite local contact bounds.
use std::ops::{Add, Mul, Neg, Sub};
#[derive(Clone, Copy, Debug)]
pub(crate) struct Interval {
    pub(crate) lo: f64,
    pub(crate) hi: f64,
}
impl Interval {
    pub(crate) fn point(x: f64) -> Self {
        Self { lo: x, hi: x }
    }
    pub(crate) fn range(lo: f64, hi: f64) -> Self {
        Self {
            lo: lo.next_down(),
            hi: hi.next_up(),
        }
    }
    pub(crate) fn around(x: f64, r: f64) -> Self {
        Self::range(x - r, x + r)
    }
    pub(crate) fn maxabs(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }
    pub(crate) fn over(self, rhs: Self) -> Option<Self> {
        if rhs.lo <= 0. || !rhs.hi.is_finite() {
            return None;
        }
        Some(self * Interval::range(1. / rhs.hi, 1. / rhs.lo))
    }
    pub(crate) fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
    }
}
impl Add for Interval {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::range(self.lo + b.lo, self.hi + b.hi)
    }
}
impl Neg for Interval {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            lo: -self.hi,
            hi: -self.lo,
        }
    }
}
impl Sub for Interval {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        self + (-b)
    }
}
impl Mul for Interval {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        let c = [
            self.lo * b.lo,
            self.lo * b.hi,
            self.hi * b.lo,
            self.hi * b.hi,
        ];
        Self::range(
            c.into_iter().fold(f64::INFINITY, f64::min),
            c.into_iter().fold(f64::NEG_INFINITY, f64::max),
        )
    }
}
