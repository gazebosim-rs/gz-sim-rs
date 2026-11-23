// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One f64 compute invocation per independent environment.
//! The graph is authored directly in Naga IR and compiled by the device backend.
use naga::{
    AddressSpace, Arena, ArraySize, BinaryOperator, Binding, Block, BuiltIn, EntryPoint,
    Expression, Function, FunctionArgument, GlobalVariable, Handle, Literal, LocalVariable,
    MathFunction, Module, ResourceBinding, Scalar, ShaderStage, Span, Statement, StorageAccess,
    Type, TypeInner, VectorSize,
};
use std::{
    cell::RefCell,
    ops::{Add, Div, Mul, Sub},
};

const SPAN: Span = Span::UNDEFINED;
const DT: f64 = 0.001;

// Each arithmetic operator appends one SSA node. In particular division and
// multiplication below preserve the operation order of the CPU projection.
#[derive(Clone, Copy)]
pub(super) struct Value<'a> {
    arena: &'a RefCell<Arena<Expression>>,
    id: Handle<Expression>,
}

impl<'a> Value<'a> {
    fn new(arena: &'a RefCell<Arena<Expression>>, expression: Expression) -> Self {
        Self {
            arena,
            id: arena.borrow_mut().append(expression, SPAN),
        }
    }

    pub(super) fn binary(self, op: BinaryOperator, other: Self) -> Self {
        Self::new(
            self.arena,
            Expression::Binary {
                op,
                left: self.id,
                right: other.id,
            },
        )
    }

    pub(super) fn math(self, fun: MathFunction, other: Option<Self>) -> Self {
        Self::new(
            self.arena,
            Expression::Math {
                fun,
                arg: self.id,
                arg1: other.map(|v| v.id),
                arg2: None,
                arg3: None,
            },
        )
    }

    pub(super) fn nonfinite(self) -> Self {
        let classify = |fun| {
            Self::new(
                self.arena,
                Expression::Relational {
                    fun,
                    argument: self.id,
                },
            )
        };
        classify(naga::RelationalFunction::IsNan).binary(
            BinaryOperator::LogicalOr,
            classify(naga::RelationalFunction::IsInf),
        )
    }

    fn sign_or_positive(self, zero: Self, one: Self, minus_one: Self) -> Self {
        self.binary(BinaryOperator::Less, zero)
            .choose(minus_one, one)
    }

    pub(super) fn choose(self, yes: Self, no: Self) -> Self {
        Self::new(
            self.arena,
            Expression::Select {
                condition: self.id,
                accept: yes.id,
                reject: no.id,
            },
        )
    }
}

macro_rules! arithmetic {
    ($trait:ident, $method:ident, $op:ident) => {
        impl $trait for Value<'_> {
            type Output = Self;
            fn $method(self, right: Self) -> Self {
                self.binary(BinaryOperator::$op, right)
            }
        }
    };
}
arithmetic!(Add, add, Add);
arithmetic!(Sub, sub, Subtract);
arithmetic!(Mul, mul, Multiply);
arithmetic!(Div, div, Divide);

/// A 64-thread compute entry named `step`, with one read/write storage buffer
/// at group zero binding zero. Its length determines the number of whole lanes.
pub(super) fn build<const WIDTH: usize, const STORES: usize>(
    constants: &[f64],
    calculate: impl for<'a> FnOnce([Value<'a>; WIDTH], &[Value<'a>]) -> [(usize, Value<'a>); STORES],
) -> Module {
    build_mode(constants, false, |values, constants, _| {
        calculate(values, constants)
    })
}

/// Each lane executes an exact, externally bounded number of one-millisecond
/// steps. Binding one is a read-only u32 count. The closure receives a one-based
/// tick offset as an exactly represented f64, alongside the previous observation.
pub(super) fn build_loop<const WIDTH: usize, const STORES: usize>(
    constants: &[f64],
    calculate: impl for<'a> FnOnce(
        [Value<'a>; WIDTH],
        &[Value<'a>],
        Value<'a>,
    ) -> [(usize, Value<'a>); STORES],
) -> Module {
    build_mode(constants, true, |values, constants, offset| {
        calculate(values, constants, offset.unwrap())
    })
}

fn build_mode<const WIDTH: usize, const STORES: usize>(
    constants: &[f64],
    repeated: bool,
    calculate: impl for<'a> FnOnce(
        [Value<'a>; WIDTH],
        &[Value<'a>],
        Option<Value<'a>>,
    ) -> [(usize, Value<'a>); STORES],
) -> Module {
    let mut module = Module::default();
    let mut ty = |inner| module.types.insert(Type { name: None, inner }, SPAN);
    let f64_ty = ty(TypeInner::Scalar(Scalar::F64));
    let storage_ty = ty(TypeInner::Array {
        base: f64_ty,
        size: ArraySize::Dynamic,
        stride: 8,
    });
    let id_ty = ty(TypeInner::Vector {
        size: VectorSize::Tri,
        scalar: Scalar::U32,
    });
    let u32_ty = ty(TypeInner::Scalar(Scalar::U32));
    let global = module.global_variables.append(
        GlobalVariable {
            name: Some("lanes".into()),
            space: AddressSpace::Storage {
                access: StorageAccess::LOAD | StorageAccess::STORE,
            },
            binding: Some(ResourceBinding {
                group: 0,
                binding: 0,
            }),
            ty: storage_ty,
            init: None,
            memory_decorations: Default::default(),
        },
        SPAN,
    );
    let control = repeated.then(|| {
        module.global_variables.append(
            GlobalVariable {
                name: Some("ticks".into()),
                space: AddressSpace::Uniform,
                binding: Some(ResourceBinding {
                    group: 0,
                    binding: 1,
                }),
                ty: u32_ty,
                init: None,
                memory_decorations: Default::default(),
            },
            SPAN,
        )
    });
    let mut function = Function::default();
    function.arguments.push(FunctionArgument {
        name: Some("invocation".into()),
        ty: id_ty,
        binding: Some(Binding::BuiltIn(BuiltIn::GlobalInvocationId)),
    });

    let arena = RefCell::new(Arena::new());
    let expr = |e| Value::new(&arena, e);
    let number = |x| expr(Expression::Literal(Literal::F64(x)));
    let uint = |i| expr(Expression::Literal(Literal::U32(i)));
    // Literals, arguments and globals are in scope at entry. Subsequent loads
    // and arithmetic are emitted only after the invocation bounds check.
    let buffer = expr(Expression::GlobalVariable(global));
    let invocation = expr(Expression::FunctionArgument(0));
    let offsets: [_; WIDTH] = std::array::from_fn(|i| uint(i as u32));
    let width = uint(WIDTH as u32);
    let numbers: Vec<_> = constants.iter().copied().map(number).collect();
    let control = control.map(|g| expr(Expression::GlobalVariable(g)));
    let iteration = control.map(|_| {
        let local = function.local_variables.append(
            LocalVariable {
                name: Some("iteration".into()),
                ty: u32_ty,
                init: Some(offsets[0].id),
            },
            SPAN,
        );
        expr(Expression::LocalVariable(local))
    });
    // A resident lane lives in function-local scalars for the whole loop.
    // The device storage buffer is touched only at admission and publication.
    let locals = iteration.map(|_| {
        std::array::from_fn(|_| {
            let v = function.local_variables.append(
                LocalVariable {
                    name: None,
                    ty: f64_ty,
                    init: None,
                },
                SPAN,
            );
            expr(Expression::LocalVariable(v))
        })
    });
    let mark = arena.borrow().len();
    let lane = expr(Expression::AccessIndex {
        base: invocation.id,
        index: 0,
    });
    let length = expr(Expression::ArrayLength(buffer.id));
    let outside = lane.binary(BinaryOperator::GreaterEqual, length / width);
    function
        .body
        .push(Statement::Emit(arena.borrow().range_from(mark)), SPAN);
    function.body.push(
        Statement::If {
            condition: outside.id,
            accept: Block::from_vec(vec![Statement::Return { value: None }]),
            reject: Block::new(),
        },
        SPAN,
    );

    let mark = arena.borrow().len();
    let base = lane * width;
    let pointers = offsets.map(|offset| {
        expr(Expression::Access {
            base: buffer.id,
            index: (base + offset).id,
        })
    });
    let control = control.map(|v| expr(Expression::Load { pointer: v.id }));
    function
        .body
        .push(Statement::Emit(arena.borrow().range_from(mark)), SPAN);

    if let Some(locals) = locals {
        let mark = arena.borrow().len();
        let initial = pointers.map(|p| expr(Expression::Load { pointer: p.id }));
        function
            .body
            .push(Statement::Emit(arena.borrow().range_from(mark)), SPAN);
        for (v, value) in locals.into_iter().zip(initial) {
            function.body.push(
                Statement::Store {
                    pointer: v.id,
                    value: value.id,
                },
                SPAN,
            );
        }
    }
    let state = locals.unwrap_or(pointers);
    let mut step_body = Block::new();
    let mark = arena.borrow().len();
    let next = iteration.map(|v| expr(Expression::Load { pointer: v.id }) + offsets[1]);
    let done = next
        .zip(control)
        .map(|(n, c)| n.binary(BinaryOperator::GreaterEqual, c));
    let offset = next.map(|v| {
        expr(Expression::As {
            expr: v.id,
            kind: naga::ScalarKind::Float,
            convert: Some(8),
        })
    });
    let values = state.map(|p| expr(Expression::Load { pointer: p.id }));
    let outputs = calculate(values, &numbers, offset);
    step_body.push(Statement::Emit(arena.borrow().range_from(mark)), SPAN);
    for &(slot, value) in &outputs {
        step_body.push(
            Statement::Store {
                pointer: state[slot].id,
                value: value.id,
            },
            SPAN,
        );
    }
    if let (Some(counter), Some(next), Some(done)) = (iteration, next, done) {
        step_body.push(
            Statement::If {
                condition: done.id,
                accept: Block::from_vec(vec![Statement::Break]),
                reject: Block::new(),
            },
            SPAN,
        );
        step_body.push(
            Statement::Store {
                pointer: counter.id,
                value: next.id,
            },
            SPAN,
        );
        function.body.push(
            Statement::Loop {
                body: step_body,
                continuing: Block::new(),
                break_if: None,
            },
            SPAN,
        );
        let mark = arena.borrow().len();
        let final_values = outputs.map(|(index, _)| {
            (
                index,
                expr(Expression::Load {
                    pointer: state[index].id,
                }),
            )
        });
        function
            .body
            .push(Statement::Emit(arena.borrow().range_from(mark)), SPAN);
        for (index, value) in final_values {
            function.body.push(
                Statement::Store {
                    pointer: pointers[index].id,
                    value: value.id,
                },
                SPAN,
            );
        }
    } else {
        function.body.append(&mut step_body);
    }
    function.body.push(Statement::Return { value: None }, SPAN);
    function.expressions = arena.into_inner();
    module.entry_points.push(EntryPoint {
        name: "step".into(),
        stage: ShaderStage::Compute,
        early_depth_test: None,
        workgroup_size: [64, 1, 1],
        workgroup_size_overrides: None,
        function,
        mesh_info: None,
        task_payload: None,
        incoming_ray_payload: None,
    });
    module
}

/// Original vertical strut and receiver graph.
pub fn module() -> Module {
    build::<12, 6>(
        &[DT, -9.81 * DT, 5. / 8. * (-9.81 * DT), 0., 1., -1.],
        |values, c| {
            let [h, gravity_step, rigid_gravity_chord, zero, one, minus_one] = *c else {
                unreachable!()
            };
            let load = |i: usize| values[i];
            let anchor_v = load(1);
            let anchor = load(0) + h * anchor_v;
            let bob_start = load(2);
            let bob_v = load(3) + gravity_step;
            let bob_predicted = bob_start + h * bob_v;
            let rigid_start = load(4);
            let rigid_v0 = load(5);
            let rigid_v = rigid_v0 + gravity_step;
            let rigid_predicted = rigid_start + h * (rigid_v0 + rigid_gravity_chord);
            let chord_v = (rigid_predicted - rigid_start) / h;
            let rest = load(6);
            let compliance = load(7);
            let bob_weight = load(8);
            let rigid_weight = load(9);
            let radius = load(10);

            // One bilateral distance projection; the constructor puts the bob above
            // the anchor, which also defines its rest direction at exact coincidence.
            let delta = bob_predicted - anchor;
            let lambda =
                (rest - delta.math(MathFunction::Abs, None)) / (bob_weight + compliance / (h * h));
            let dx = delta.sign_or_positive(zero, one, minus_one) * (bob_weight * lambda);
            let bob = bob_predicted + dx;
            let bob_v = bob_v + dx / h;

            // A relative segment that starts outside must hit the incident face, even
            // if its endpoint crosses the entire sphere. Starting inside uses the end
            // normal, falling back to the initial side for a coincident endpoint.
            let start_gap = rigid_start - bob_start;
            let end_gap = rigid_predicted - bob;
            let start_distance = start_gap.math(MathFunction::Abs, None);
            let end_side = end_gap.binary(BinaryOperator::Equal, zero).choose(
                start_gap.sign_or_positive(zero, one, minus_one),
                end_gap.sign_or_positive(zero, one, minus_one),
            );
            let side = start_distance
                .binary(BinaryOperator::GreaterEqual, radius)
                .choose(start_gap.sign_or_positive(zero, one, minus_one), end_side);
            let depth = radius - side * end_gap;
            let inverse_mass = bob_weight + rigid_weight;
            let j_pos = side * (depth.math(MathFunction::Max, Some(zero)) / inverse_mass / h);
            let bob_dz = (h * bob_weight) * j_pos;
            let rigid_dv = rigid_weight * j_pos;
            let bob_v = bob_v - bob_dz / h;
            let separating_v = side * ((chord_v + rigid_dv) - bob_v);
            let overlap_speed = (radius - start_distance).math(MathFunction::Max, Some(zero)) / h;
            // Remove the residual closing velocity, or only the part of separating
            // velocity attributable to an initial overlap. There is no restitution.
            let delta_v = (zero - separating_v).math(MathFunction::Max, Some(zero - overlap_speed));
            let j_vel = depth
                .binary(BinaryOperator::Greater, zero)
                .choose(side * (delta_v / inverse_mass), zero);
            [
                (0, anchor),
                (2, bob - bob_dz),
                (3, bob_v - bob_weight * j_vel),
                (4, rigid_predicted + h * rigid_dv),
                (5, rigid_v + (rigid_dv + rigid_weight * j_vel)),
                (11, j_pos + j_vel),
            ]
        },
    )
}

#[cfg(test)]
#[path = "../tests/private/kernel.rs"]
mod tests;
