// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Operators shared by the scalar CPU solver and the f64 Naga shader builder.
//! CPU predicates remain booleans. Shader predicates are boolean SSA values.
use crate::kernel::Value;
use naga::{BinaryOperator as B, MathFunction as M};
use std::ops::{Add, Div, Mul, Sub};
pub(super) trait Scalar:
    Copy + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> + Div<Output = Self>
{
    type Condition: Condition<Self>;
    fn less(self, rhs: Self) -> Self::Condition;
    fn greater(self, rhs: Self) -> Self::Condition;
    fn at_most(self, rhs: Self) -> Self::Condition;
    fn at_least(self, rhs: Self) -> Self::Condition;
    fn minimum(self, rhs: Self) -> Self;
    fn maximum(self, rhs: Self) -> Self;
    fn root(self) -> Self;
}
pub(super) trait Condition<T>: Copy {
    fn and(self, rhs: Self) -> Self;
    fn or(self, rhs: Self) -> Self;
    fn select(self, yes: T, no: T) -> T;
}
impl Scalar for f64 {
    type Condition = bool;
    fn less(self, rhs: Self) -> bool {
        self < rhs
    }
    fn greater(self, rhs: Self) -> bool {
        self > rhs
    }
    fn at_most(self, rhs: Self) -> bool {
        self <= rhs
    }
    fn at_least(self, rhs: Self) -> bool {
        self >= rhs
    }
    fn minimum(self, rhs: Self) -> Self {
        self.min(rhs)
    }
    fn maximum(self, rhs: Self) -> Self {
        self.max(rhs)
    }
    fn root(self) -> Self {
        self.sqrt()
    }
}
impl Condition<f64> for bool {
    fn and(self, rhs: Self) -> Self {
        self && rhs
    }
    fn or(self, rhs: Self) -> Self {
        self || rhs
    }
    fn select(self, yes: f64, no: f64) -> f64 {
        if self { yes } else { no }
    }
}
impl<'a> Scalar for Value<'a> {
    type Condition = Self;
    fn less(self, rhs: Self) -> Self {
        self.binary(B::Less, rhs)
    }
    fn greater(self, rhs: Self) -> Self {
        self.binary(B::Greater, rhs)
    }
    fn at_most(self, rhs: Self) -> Self {
        self.binary(B::LessEqual, rhs)
    }
    fn at_least(self, rhs: Self) -> Self {
        self.binary(B::GreaterEqual, rhs)
    }
    fn minimum(self, rhs: Self) -> Self {
        self.math(M::Min, Some(rhs))
    }
    fn maximum(self, rhs: Self) -> Self {
        self.math(M::Max, Some(rhs))
    }
    fn root(self) -> Self {
        self.math(M::Sqrt, None)
    }
}
impl<'a> Condition<Value<'a>> for Value<'a> {
    fn and(self, rhs: Self) -> Self {
        self.binary(B::LogicalAnd, rhs)
    }
    fn or(self, rhs: Self) -> Self {
        self.binary(B::LogicalOr, rhs)
    }
    fn select(self, yes: Self, no: Self) -> Self {
        self.choose(yes, no)
    }
}
