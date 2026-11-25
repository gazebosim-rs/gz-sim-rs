<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Technical Design Document: Multi-Fidelity, Hybrid CPU/GPU Distributed Physics Engine in Rust

**Status:** Approved implementation direction, 2026-11-18

**Target Language:** Rust 2024, pinned gz-sim-rs workspace toolchain

**Target Ecosystem:** Next-Generation Robotics Simulation (Gazebo / ROS 2 / RL Training Pipelines)

---

The [implementation worklog](MULTIFIDELITY_WORKLOG.md) records executable examples, measured results, and outstanding milestones in the physics development worktree.

## 1. Executive Summary & Goals

Modern robotics simulation requires conflicting computational trade-offs: large-scale warehouse and factory logistics demand faster-than-real-time execution over thousands of entities, humanoid and dexterous manipulation tasks demand high-precision articulated contact dynamics, and emerging manipulation domains require realistic simulation of fabrics, cables, and deformable objects. Historically, physics engines force users to choose a single global fidelity level per simulation instance.

This document specifies the architecture for a **single, unified physics engine written in Rust** capable of simulating heterogeneous fidelity levels simultaneously within the same shared world state, scaling from deterministic single-node CPU testing to GPU-accelerated reinforcement learning (RL) batches and multi-node distributed worlds.

### 1.1 Core Design Goals

1. **Simultaneous Multi-Fidelity Execution:** Run fast kinematic motion, f64 articulated rigid-body dynamics, and contact-coupled deformables concurrently in a single scene.
2. **Dynamic Runtime Fidelity Promotion & Demotion:** Seamlessly transition entities between solver tiers at runtime (e.g., promoting an AMR from kinematic transport to full rigid-body dynamics upon entering a manipulation zone) without state discontinuities or energy spikes.
3. **Phased Two-Way Cross-Solver Coupling:** Support one-way coupling in V1 (kinematic/rigid boundaries driving soft bodies) while designing data structures from day one for full two-way momentum and wrench exchange in V2.
4. **Hybrid CPU/GPU Compute Model:** Default to CPU execution for low-latency robotics testing while using solver-owned data layouts with explicitly validated GPU transfer views for large-scale simulations and batched RL training.
5. **Toggleable Bit-Exact Determinism:** Provide a `Deterministic` execution mode for CI regression testing and lockstep ROS 2 simulation, and measure throughput-oriented profiles separately.
6. **Distributed Multi-Instance World Scaling:** Partition a single continuous simulation world across multiple compute nodes using spatial domain decomposition and low-latency halo (ghost) synchronization.

### 1.2 Non-Goals (Initial Scope)

* Photorealistic rendering or sensor raytracing (handled by external rendering/sensor pipelines consuming canonical state buffers).
* Full Navier-Stokes computational fluid dynamics (CFD) or aeroelasticity beyond simplified drag/buoyancy models.

---

## 2. State of the Art & Architectural Approach

Rather than forcing a single mathematical formulation (e.g., pure XPBD or pure Featherstone) to handle every physical regime—which inevitably compromises either humanoid contact accuracy or fleet-scale speed—this engine uses **Partitioned Multi-Solver Coupling** over a **Unified Spatial State**.

### 2.1 Existing Ecosystem Context

* **Dimforge (`rapier` & `nexus`):** Validates pure-Rust rigid-body multibody simulation (`rapier`) and GPU-accelerated multi-physics (`nexus` combining rigid bodies and MPM).
* **`phyz`:** Demonstrates pure-Rust $O(n)$ Featherstone dynamics alongside XPBD cloth/cables across CPU and GPU backends.
* **NVIDIA/DeepMind/Disney `Newton` & `Genesis`:** Validate the multi-solver architectural pattern where specialized solvers (MuJoCo-style articulated solvers, XPBD/VBD cloth, MPM) exchange forces over a shared spatial representation.
* **Gazebo (`gz-physics` + TPE):** Introduced the Trivial Physics Engine (TPE) for kinematic simulation alongside DART/Bullet, but historically isolated physics engines at the world/plugin boundary rather than coupling heterogeneous solvers within a single scene.

### 2.2 Core Architectural Paradigm

The existing Gazebo Bevy world owns public identities, lifecycle, simulation time, and committed observations. Install one multi-fidelity backend per selected world. Each pair of colliders has one contact owner. The initial CPU implementation uses the pinned Rapier f64 broad-phase, narrow-phase, and contact solver for both moving boundaries and rigid bodies. Introduce a new spatial index only when a measured workload needs it.

## 3. System architecture

### 3.1 Workspace and configuration

Begin with `gz-physics-multifidelity` in `gz-sim-rs/crates`. It implements the existing `gz-physics::DynamicBackend` contract and uses the Bevy installation, reset and commit phases. Keep standard Rapier and TPE independently selectable. Use the pinned sdformat-rs loader for SDF, frame resolution, and scoped names. Select fidelity explicitly at the model or assembly level. Record which model and capability caused a refused configuration. Additional import formats and external bindings are separate integrations.

Keep solver modules together initially. Separate GPU and distributed crates when they have an implementation and distinct dependency profiles. The headless CPU server must work independently of GPU, renderer, network workers, or external language runtimes. All new implementation and test code is Rust, including authored GPU kernels.

### 3.2 State, units, and time

Use Gazebo's public stable IDs and world epochs. Internal solver handles and recyclable dense slots remain private. Outside the numerical step Bevy consumers see one committed state. A backend may own contiguous buffers, constraint graphs, and caches. Allocate optional systems and their buffers when demand appears, and retire them when the last consumer disappears.

Use f64 SI values in a right-handed Z-up world, with XYZW rotations. Body state records link-origin position, world angular velocity, and world linear velocity at the center of mass. Convert to the reference point required by each algorithm. Use separate typed paths for continuous force/torque and instantaneous linear/angular impulse. Define accelerator packing, alignment, precision, transfers, and readback explicitly for each supported target.

Advance exactly the requested integer duration. Paused ticks consume no time and no pending input. Partition a public step into deterministic substeps that sum to that duration, including the remainder. Evaluate prescribed boundary motion at every contact substep. The SDF step, command semantics, sensor timestamps, and public iteration count retain their existing meanings.

## 4. Solver Specifications by Fidelity Tier

### 4.1 Tier 1: Fast Kinematic Solver (Process & Fleet Scale)

* **Target Workloads:** AMRs in transit, conveyor belts, automated doors, background traffic, sensor mounts.
* **Mathematical Formulation:**
  * Direct analytical integration: $\mathbf{x}_{t+\Delta t} = \mathbf{x}_t + \mathbf{v}_t \Delta t + \frac{1}{2}\mathbf{a}_t \Delta t^2$.
  * Spline/waypoint trajectory interpolation with exact velocity and acceleration derivatives.
  * Lightweight swept-AABB or discrete overlap queries for trigger volumes and proximity sensors without impulse resolution.
* **Step Rate:** Advance isolated trajectories analytically. Sample interacting trajectories at solver substeps. Qualify fleet throughput at the declared public timestep and scene density.

### 4.2 Tier 2: High-Accuracy Articulated Rigid-Body Solver (Humanoids & Manipulation)

* **Target Workloads:** Bipedal humanoids, legged quadrupeds, multi-DOF robotic arms, dexterous hands.
* **Mathematical Formulation:**
  * **Forward Dynamics:** Qualify the pinned Rapier f64 reduced-coordinate path first. Measure free-tree dynamics, contact errors, motor/limit behavior, and cost with increasing link counts. Implement a Rust Featherstone ABA if the required behavior or measured cost calls for it. The linear-time target concerns free-tree forward dynamics. Contact solving has a separate cost and convergence test.
  * **Contact & Friction:** Use the pinned f64 contact solver for the first implementation. Measure penetration, friction, stack stability, joint-limit error, and runtime at fixed solver settings. Use these results to decide whether a specialized contact solver is needed.
* **Step Rate:** Sub-stepped at $\Delta t = 0.5\text{–}2.0\text{ ms}$ with velocity-level and position-level stabilization.

### 4.3 Tier 3: Soft-Body & Fabric Solver

* **Target Workloads:** Garments, bagging/packaging, cables, hoses, deformable payloads.
* **Mathematical Formulation:**
  * **1D/2D Structures (Cables & Cloth):** **Extended Position-Based Dynamics (XPBD)** or **Vertex Block Descent (VBD)** supporting compliance-parameterized stretch, shear, and isometric bending constraints, plus spatial-hash self-collision.
  * **3D Volumetric Deformables (Optional Extension):** Moving Least Squares Material Point Method (**MLS-MPM**) for foam, soil, or granular media.
* **Step Rate:** Sub-stepped at $\Delta t = 0.25\text{–}1.0\text{ ms}$ inside the rigid-body micro-step loop.

---

## 5. Cross-Solver Coupling & Dynamic Fidelity Switching

### 5.1 Two-Way Coupling Architecture

Cross-tier contact uses boundary velocity constraints and a separate reaction-impulse accumulator.

```text
Prescribed boundaries ── surface motion ──► rigid and soft contacts
Rigid links ── geometry and surface motion ──► soft contact solver
Rigid links ◄── equal and opposite contact impulses ── soft contact solver (V2)
```

* **Kinematic $\leftrightarrow$ Rigid/Soft (One-Way):** Kinematic bodies act as infinite-mass moving boundaries. Their surface velocities $\mathbf{v}_{\text{surf}} = \mathbf{v}_{\text{lin}} + \boldsymbol{\omega} \times \mathbf{r}$ are injected directly into rigid contact constraints and soft-body SDF projections.
* **Rigid $\leftrightarrow$ Soft (Phased One-Way $\to$ Two-Way):**
  * **V1 (One-Way):** Soft-body vertices colliding against rigid link primitives/SDFs are projected out along the contact normal with friction damping; reaction forces are discarded.
  * **V2 (two-way)** — Record the contact solver's normal and tangential impulse on the particle. Apply the opposite impulse to the rigid link, with angular impulse about its center of mass. Sum impulse increments across soft substeps exactly once per coupling interval. Isolate contact multipliers from stretch, bending, and other internal constraints. If loads enter a solver as mean forces, divide the summed impulse by the actual receiving interval. Qualify the coupled update on light links and stiff materials before choosing a lagged or iterated scheme.

### 5.2 Dynamic promotion and demotion

Perform each switch as an assembly transaction at a defined tick boundary. Prepare the destination before modifying the active tier. Transfer public identities, current link poses and twists, joint coordinates and velocities, controller targets, and pending inputs. Retain joint configuration during prescribed motion. Reconstruct a base-origin velocity from center-of-mass velocity and angular velocity at the correct offset. Invalidate or remap contact caches when their geometry or solver ownership changes. A rejected switch leaves the source intact.

Promote on explicit request, zone entry or a predicted contact requiring dynamic response. Demote only with hysteresis, settled internal motion, and verified physical support or a compatible prescribed trajectory. A freely falling robot can have quiet joints. Compare pre/post pose, twist, momentum, and energy at the same instant using a documented numerical budget. Account separately for subsequent work by the prescribed trajectory.

## 6. Compute Backends & Determinism

### 6.1 Hybrid CPU / GPU Execution Model

Use the pinned f64 CPU solver for the reference trajectory. Profile island, ECS synchronization and broad-phase costs before selecting SIMD layout or a thread pool. Measure changes with the same contact workload, accuracy, public step, and total worker budget. Keep batched GPU execution behind an optional Rust interface. Validate target f64 support, layout and device limits, and measure packing, copies, fences and host observations alongside kernel time.

### 6.2 Replay and throughput

Pin the first deterministic domain to a build, execution profile, and CPU target. Specify construction, pair, feature, reduction and input order. Seed stochastic work by stable identity, step, stream, and draw counter. Include solver caches, pending input, and state that can affect the next step in replay validation. Test replay under different worker counts and after spawn, remove, and reset. Qualify GPU determinism per target. Compare CPU and GPU with quantity-specific error budgets. An accelerated execution profile must name its build features and numerical differences.

## 7. Distributed Multi-Instance Simulation

To simulate massive facilities (e.g., multi-building factories with thousands of robots) that exceed single-node compute capacity, the world is partitioned across a cluster of simulation workers.

### 7.1 Spatial Domain Decomposition & Halo Exchange

* **Spatial Partitioning:** The world is divided into 2D/3D spatial cells (using an adaptive octree or recursive coordinate bisection) mapped to worker nodes.
* **Ownership.** Give each entity one owner per epoch. Keep articulated trees and connected deformables whole. The owning worker may temporarily host a complete contacting island that spans multiple spatial cells.
* **Halo sizing.** Bound the swept geometry over the communication interval, including rotational extent, articulation and deformation.

### 7.2 Synchronization protocol over Zenoh

Co-locate potentially contacting dynamic islands for the first distributed implementation. Assign one solver owner for every interacting island and include the dynamic response of both participants. Commit migration at an epoch barrier with acknowledged ownership, sequence numbers and idempotent delivery. Transfer solver caches, pending commands, and impulses with the assembly. Define the behavior on a late or missing peer before enabling independent advancement. Use upstream Zenoh directly outside the numerical step, and qualify transports individually.

## 8. Delivery and acceptance

1. **Headless CPU scene.** Deliver a selected Rust backend, a moving prescribed primitive platform, and a dynamic payload through the normal SDF/CLI path. Observe friction-driven payload motion, a committed ray hit, exact time, pause, and reset. Refuse unsupported model configurations explicitly.
2. **Switching and fleet.** Implement assembly promotion/demotion, atomic refusal, stable IDs and exactly-once inputs. Round-trip a moving bent articulation with offset inertia. Exercise 10,000 switches and benchmark 100,000 kinematic entities against the 100× real-time target with fixed hardware, density, public timestep and collision workload.
3. **Articulated and deformable dynamics.** Qualify loaded joint topology, actuators, limits, pendulum, friction incline, and stack. Use XPBD for initial cables and cloth. Measure sag, stretch, self-contact, moving-boundary collision, and supported payload weight. Stage one-way coupling before two-way. Test linear and angular momentum at 1, 2, and 4 soft substeps per rigid step. Follow with the controlled 30-DOF humanoid and garment grasp demonstrations. Volumetric soft bodies and MLS-MPM are later extensions.
4. **GPU batches.** Compile Rust-authored kernels for an explicitly qualified device profile. Validate CPU comparisons before measuring 2,048 mixed environments and the 10,000 aggregate steps/s target. Report simulated time per wall second, accuracy, p99 step latency, memory and transfer costs.
5. **Distributed worlds.** Validate two-worker handover and a boundary collision against the single-node fixture. Exercise duplicate, stale, missing and delayed packets, migration during promotion, and owner recovery before adaptive load balancing.

Maintain the implementation and its worklog in the simulator development worktree. Treat targets as open until a recorded run names the workload, hardware, compiler and result.

## 9. Key Technical Risks & Mitigations

| Risk | Impact | Mitigation Strategy |
| :--- | :--- | :--- |
| **Energy Injection at Two-Way Rigid/Soft Boundaries** | Stiff soft bodies interacting with light rigid links can cause contact jitter or numerical explosion. | Measure stiffness and mass-ratio stability, momentum balance and timestep convergence. Apply any bounded contact impulse equally and oppositely. |
| **Fidelity Demotion Chattering** | Entities hovering near a promotion boundary repeatedly allocate/deallocate articulated state. | Implement spatial deadbands (promote at $R_{\text{in}}$, demote at $R_{\text{out}} > R_{\text{in}}$) combined with temporal cooldown timers and velocity thresholds. |
| **GPU/CPU Floating-Point Divergence** | GPU target precision and reduction order can differ from the f64 CPU reference. | Qualify repeatability on each supported build and device. Test quantity-specific error budgets against the CPU fixtures. |
| **Distributed Latency Spikes** | Network jitter delays halo packets across compute nodes, stalling the macro-step. | Validate epoch and sequence before committing a step. Benchmark overlapping communication with independent work and qualify optional transport profiles. |