// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A parked lift falls away from a supported foot and catches it on its return.
#[path = "support_reacquisition/admission_study.rs"]
mod admission_study;
#[cfg(feature = "landing-audit")]
#[path = "support_reacquisition/matched.rs"]
mod matched;
#[path = "support_reacquisition/rigid_control.rs"]
mod rigid_control;
#[path = "support_reacquisition/sweep.rs"]
mod sweep;
#[cfg(feature = "landing-audit")]
#[path = "support_reacquisition/touchdown.rs"]
mod touchdown;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier::{self, Dynamic, Prescribed},
    BodyHandle, FidelityError, Name, PhysicsEngine, load_world_with_engine, model_fidelity,
    queue_link_world_velocity, set_model_fidelity, support_loss_promotion,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::PathBuf,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SDF: &str = include_str!("../../../examples/multifidelity_support_reacquisition.sdf");
const SETTLE_STEPS: u64 = 2400;
const FOOT_HALF: [f64; 3] = [0.18, 0.10, 0.06];
const PAD_HALF: [f64; 3] = [0.33, 0.18, 0.06];
const PARK_STEPS: u64 = 80;
const WINDOW_STEPS: usize = 10;

#[derive(Parser)]
#[command(group(clap::ArgGroup::new("study").multiple(false)))]
#[cfg_attr(feature="landing-audit",command(group(clap::ArgGroup::new("survey_range").args(["secondary_material_policy","registered_return_field","secondary_travel_feature_field","secondary_return_feature_field","secondary_natural_field","secondary_relaxation_field","secondary_order_cohort","secondary_cache_cohort","persistence_prefix","physical_secondary_long","secondary_policy","vertex_policy","sticking_policy","internal_transfer_policy","launch_point_policy","spatial_point_policy","internal_transfer_long","retimed_long","secondary_long","clocked_sequence","contact_sequence","after_secondary","second_point","registered_second_point","precise_second_point","work_second_point","line_sticking","line_wrench","line_closure","finite_transfer_policy","finite_transfer","finite_load","finite_propagation","whole_primary_long","compact_primary_long","active_point_episode","whole_primary","subdivided_primary","compact_primary","deep_onset_controls","deep_onset","successor_onset","physical_handoff","cold_point_episode","cold_point_field","deep_convergence","finite_policy","finite_survey","grid_survey","relaxation_survey","propagation_survey","convergence_survey","continuation_survey"])),group(clap::ArgGroup::new("survey_playback").args(["primary_fixture_sdf","compact_primary_replay","whole_primary_replay","launch_interval_view","launch_point_replay","spatial_interval_view","internal_interval_view","internal_transfer_replay","spatial_point_replay","sticking_replay","vertex_replay","secondary_policy_replay","secondary_interval_view","second_replay","finite_replay","survey_replay","relaxation_replay","continuation_replay","span_replay"]))))]
struct Args {
    output: PathBuf,
    /// Record 13 ms on the identical 512-call continuation used by the three-stroke studies.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    persistence_prefix: bool,
    /// Replay a checked three-stroke finite-versus-ordinary first landing on the common native suffix.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    persistence_replay: Option<usize>,
    /// Restart an already airborne second landing and cross donor history with native resolution.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name = "INDEX")]
    secondary_restart: Option<usize>,
    /// Inspect actual native point rows and signed momentum at a qualified second landing.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name = "INDEX")]
    secondary_contact_points: Option<usize>,
    /// Compare three geometry histories at the second landing on 50 prespecified original fixtures.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_cache_cohort: bool,
    /// Change only selected solver row order on the same 50 actual returning feet.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_order_cohort: bool,
    /// Cross four native budgets and six relaxation counts on the same returning foot.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_relaxation_field: bool,
    /// Replay an archived second-landing relaxation treatment.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_relaxation_replay: Option<usize>,
    /// Requery an unloaded returning box pair only when present material planes disprove its cached normal.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_return_feature_field: bool,
    /// Exercise returning-box refresh in a real SDF world from cold start, including second landings.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    registered_return_sdf:bool,
    /// Reapply the production named-pair path to the complete matched secondary native field.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    registered_return_field:bool,
    /// Qualify registered finite-point and transfer policies on matched second-landing cuts.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_material_policy:bool,
    /// Load the original states as authored SDF and measure actual finite vertex projection.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",value_name="FEATURED_INDEX")]
    secondary_face_sdf:Option<usize>,
    /// Prove isolation of the incident vertex throughout the final crossing bracket.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    secondary_certify_vertices:bool,
    /// Cross ordinary subdivision and event-directed bounds at returning contact.
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="secondary_certify_vertices")]
    secondary_event_bounds:bool,
    /// Compare authored isolated-skew-edge contact with and without subnanosecond outgoing checks.
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="secondary_face_sdf")]
    secondary_skew_edge:bool,
    /// Stage the entire bounded material response between fractional first contact and its integer endpoint.
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="secondary_face_sdf")]
    secondary_fractional_points:bool,
    /// Compare only a verified change to the native grid after a fractional transaction.
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="secondary_fractional_points")]
    secondary_fractional_retime:bool,
    /// Time the archived ordinary and fractional Bevy public landing steps.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_fractional_sdf_cost:Option<u8>,
    /// Reproduce the complete archived policy response at a selected second landing.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",value_name="INDEX")]
    secondary_material_policy_replay:Option<usize>,
    /// Replay the physical and measured 62.5 µs intervals of one archived second return.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_material_replay:Option<usize>,
    /// Cross a single unloaded box requery with one native interval of incoming travel.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_travel_feature_field: bool,
    /// Replay the archived travel and fixed-length geometry controls.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_travel_feature_replay: Option<usize>,
    /// Time actual native returns under fixed and travel-scaled support-plane tests.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="REPEAT")]
    secondary_travel_feature_cost: Option<u8>,
    /// Pinned timing of native and registered second-landing treatments.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_material_cost:Option<u8>,
    /// Time midpoint versus event-directed material bounds at the same eight- and 64-call returns.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_event_bound_cost:Option<u8>,
    /// Replay ordinary, one-witness and repeated native box feature queries at a matched return.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_return_feature_replay: Option<usize>,
    /// Vary native relaxation at a support return in the original untouched row order.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    secondary_natural_field: bool,
    /// Replay a natural-order donor after comparing its first receipts with the archived factorial.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_natural_replay: Option<usize>,
    /// Time native feature refresh and conditional plane queries on matched returning pairs.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="REPEAT")]
    secondary_return_feature_cost: Option<u8>,
    /// Time public native intervals on matched secondary contact donors.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="REPEAT")]
    secondary_relaxation_cost: Option<u8>,
    /// Measure the ordering mechanism separately on an original discovery donor.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_order_pilot: Option<usize>,
    /// Resolve the first native force and geometry difference between ordered returning feet.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", value_name="INDEX")]
    secondary_native_divergence: Option<usize>,
    /// Compare identical physical lift commands on three solver grids.
    #[arg(long, group = "study")]
    landing_sweep: bool,
    /// Compare the settled-support admission dwell settings.
    #[arg(long, group = "study")]
    admission_study: bool,
    /// Run the identical lift protocol on the standalone rigid backend.
    #[arg(long, group = "study")]
    rigid_control: bool,
    /// Replay the third landing from controlled native restart states.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    matched: bool,
    /// Substep the third touchdown from a fixed native cut and retain the actuator clock.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    touchdown_window: bool,
    /// Resolve contact impulses, energy and lift work at each native touchdown step.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    landing_energy: bool,
    /// Run mass- and inertia-matched single-body energy controls.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    rigid_energy: bool,
    /// Sweep native contact solver settings on the same airborne cut.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    contact_controls: bool,
    /// Compare secondary contact selection for flat and partially supported soles.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    rebound_gate: bool,
    /// Qualify per-tick solver effort on matched secondary impacts.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    rebound_budget: bool,
    /// Isolate free-flight integration after a matched secondary impact.
    #[cfg(feature="landing-audit")]
    #[arg(long, group="study")]
    rebound_flight: bool,
    /// Exercise release-qualified weak recontact refinement on new support placements and drives.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    weak_recontact: bool,
    /// Time the same fixtures and solver policies at the actuator clock.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    weak_timing: bool,
    /// Attribute the first outer-support trajectory divergence from matched primary landings.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    outer_divergence: bool,
    /// Record contact-point impulses at native substep boundaries on matched primary impacts.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    native_impact: bool,
    /// Validate rigid and locked articulation impacts against a frictionless plastic solution.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    analytic_impact: bool,
    /// Diagnose the per-corner velocity response of the generic multibody contact solver.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_response: bool,
    /// Exercise authored pair-specific cuboid geometry refresh from a loaded local SDF world.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    sdf_box_features: bool,
    /// Establish certified event order and reactions for successive impacts of the same rigid sole.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_sequence: bool,
    /// Time equal-work and prospectively registered grids across certified consecutive impacts.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_sequence_cost: bool,
    /// Run causally timed production contact intervals against certified rocking-foot trajectories.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    prospective_box_timing: bool,
    /// Measure compiled production box timing on the same released actuator trajectory.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    prospective_box_timing_cost: bool,
    /// Validate rigid and locked non-spherical landings against torque-free Euler flight.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    inertia_timing: bool,
    /// Measure the loaded SDF paths for admitted and declined non-spherical landings.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    inertia_timing_cost: bool,
    /// Isolate triaxial free-flight integration error from native contact arrival error.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    gyroscopic_cut: bool,
    /// Replay the measured old and corrected rotated principal inertia against an Euler cut.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    inertia_frame_impact: bool,
    /// Audit every tick of exceptional isolated arrivals against free Euler corner trajectories.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    separated_crossings_diagnose: bool,
    /// Verify a measured isolated regular box corner impulse under Coulomb friction and gravity.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_point: bool,
    /// Time verified triaxial frictional corner solves on ordinary Bevy steps.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_point_cost: bool,
    /// Validate native fallback at a closing contact with inward unconstrained acceleration.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_point_refusals: bool,
    /// Check whether the first outgoing native call is collision-free from the actual point law.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_departure: bool,
    /// Profile the same-interval departure proof under full and compact native budgets.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_departure_cost: bool,
    /// Exercise the same corner returning during or shortly after an outgoing solve.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_short_returns: bool,
    /// Compare named rigid contacts in worlds with additional interacting and excluded bodies.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    rigid_point_neighbors: bool,
    /// Compare registered secondary clearance with independent contact and per-call handoff checks.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    named_opponent: bool,
    /// Check a second upper-vertex strike and its next physical contact from the same local state.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    upper_strike: bool,
    /// Compare measured upper-vertex projection with live SDF and matched impact references.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    projected_ceiling: bool,
    /// Measure local ceiling interaction costs.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    projected_ceiling_cost: bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    opposing_return:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    opposing_return_cost:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    opposing_return_view:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    opposing_gap:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_accumulation:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_materials:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_compact:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_compact_cost:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_compact_heldout:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    native_opposing_normals:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    native_opposing_normal_field:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    native_opposing_normal_cost:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_arrivals:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_arrival_field:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_arrival_cost:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_period:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_period_field:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_grids:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    opposing_normal_grid_field:bool,
    /// Time live finite-opponent guards over verified public actuator updates.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    named_opponent_cost: bool,
    /// Time verified public landings in empty and cluttered worlds.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    rigid_point_neighbors_cost: bool,
    /// Compare grouped and per-point friction to independent triaxial Coulomb impulses.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_friction: bool,
    /// Measure full public step cost across isolated triaxial sticking and sliding landings.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_friction_cost: bool,
    /// Compare true accelerated free-flight translation on finite rotating landings.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_gravity: bool,
    /// Replay accelerated landings on an unobserved public clock.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_gravity_cost: bool,
    /// Compare measured three-call landings and full solve budgets across isolated and repeated impacts.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    compact_impact: bool,
    /// Time selected, immediately following and ordinary public steps under compact landing.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    compact_impact_cost: bool,
    /// Nominate strictly separated spinning feet over the whole public interval.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    separated_crossings: bool,
    /// Measure separated crossing nomination across certified repeated strikes.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    separated_crossings_cost: bool,
    /// Measure complete certified multi-strike SDF prefixes without observation.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_returns_cost: bool,
    /// Follow multiple certified triaxial corner strikes and compare native reacquisition.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    triaxial_returns: bool,
    /// Verify certified rigid rotation during ordinary ticks after release.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    released_rotation: bool,
    /// Time release continuation from matched initial states on held-out donors.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    released_rotation_cost: bool,
    /// Test measured and fixed post-bracket native pulse durations on rigid impacts.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    rotation_impact_pulse: bool,
    /// Time the same qualified trajectories through ordinary Bevy stepping.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    rotation_impact_pulse_cost: bool,
    /// Bound fast spinning corner arrival from measured Euler derivatives.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    cubic_rotation_timing: bool,
    /// Measure counterbalanced cubic and frozen-spin timing at matched budgets.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    cubic_rotation_cost: bool,
    /// Compare named rigid-top free rotation against native gyroscopic integration.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    gyroscopic_flight: bool,
    /// Time counterbalanced ordinary SDF trajectories with and without rigid-top integration.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    gyroscopic_flight_cost: bool,
    /// Isolate premature speculative reactions before a certified free-rigid corner impact.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    rigid_early_activation: bool,
    /// Compare pair-specific precontact suppression against an unchanged registered native timeline.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    separated_flight: bool,
    /// Time opted-in separation certification and the identical unguarded policy.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    separated_flight_cost: bool,
    /// Qualify cuboid feature refresh and relaxation against closed-form spinning impacts.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_certification: bool,
    /// Time the landing policy across every independently certified moving-pad fixture.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_certification_cost: bool,
    /// Time unbiased post-integration sweeps at the matched analytic landing boundary.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_response_cost: bool,
    /// Time fixed-time PGS passes at the matched analytic impact boundary.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    analytic_pgs_cost: bool,
    /// Separate native contact refresh, solver work, and registration of impact within a tick.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    outer_cadence: bool,
    /// Qualify a live primary-only refresh schedule across held-out support placements.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    primary_policy: bool,
    /// Measure primary-onset and whole-run step latency for the qualified fixture.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    primary_policy_timing: bool,
    /// Compare nested native contact grids at a fixed actuator clock.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    impact_grids: bool,
    /// Verify whole-tick commands when contact is refined inside one public step.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    refined_interval: bool,
    /// Compare a free ankle and a locked ankle at a plastic corner impact.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    passive_hinge_impact: bool,
    /// Compare spatial friction at a single corner against a free-hinge impulse solution.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    spatial_hinge_impact: bool,
    /// Qualified free-hinge spatial impulses on an already registered native support.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    registered_corner:bool,
    /// Qualify isolated free-ankle landings across arrival phases, orientations and both body orders.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    registered_arrivals:bool,
    /// Measure the native call budget at an isolated arrival on the public clock.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    registered_arrival_cost:bool,
    /// Uniform-gravity free ankle arrivals measured from both release and the preceding public boundary.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    ballistic_arrivals:bool,
    /// Compare unbiased relaxation passes at matched frictional primary landings.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    frictional_relaxation: bool,
    /// Compare relaxation and native grids for a range of matched support fixtures.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    relaxation_survey: bool,
    /// Check finite rotating geometry and first reactions of matched primary landings.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_survey:bool,
    /// Advance the finite projected primary on the same public clock and compare common suffixes.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_policy:bool,
    /// Diagnose the contact load immediately after the finite primary impulse.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    finite_load:bool,
    /// Follow the initially loaded point until a finite feature changes or the public tick ends.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    finite_transfer:bool,
    /// Compare matched native suffix timing after the first projected impact.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    finite_transfer_policy:bool,
    /// Resolve a single carrying endpoint at the next line of contact and compare a cold native reload.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    second_point:bool,
    /// Qualify the SDF-scheduled endpoint projection at each cold secondary interval.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    registered_second_point:bool,
    /// Tighten the measured secondary time bracket before independently reloading the world.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    precise_second_point:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    work_second_point:bool,
    /// Follow the outgoing material point through the rest of the original actuator interval.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    after_secondary:bool,
    /// Trace successive certified pressure-bearing endpoints through an original actuator interval.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    contact_sequence:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    clocked_sequence:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_cost:Option<PathBuf>,
    /// Identify which incident finite lines attain the rigid-body minimum energy under sticking.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    line_sticking:bool,
    /// Measure the admissibility of native contact loads on an incident line.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    line_wrench:bool,
    /// Exclude simultaneous line closure over the whole distributed Coulomb pressure cone.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    line_closure:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    second_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_transfer_cost:Option<PathBuf>,
    /// Time the actual public native interval and verify it against an archived finite-policy survey.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_cost:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_propagation:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    retimed_long:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_long:bool,
    /// Compare finite secondary landing interventions at the original next actuator boundary.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    secondary_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    vertex_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    sticking_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    internal_transfer_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    spatial_point_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    launch_point_policy:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    internal_transfer_long:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    internal_long_replay:Option<usize>,
    /// Time checked Bevy updates against a completed internal-transfer field.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    bound_point_cost:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    slip_launch_cost:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",num_args=2,value_names=["SUBDIVIDED","ORDINARY"])]
    whole_primary_cost:Vec<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",num_args=2,value_names=["SUBDIVIDED","ORDINARY"])]
    compact_primary_cost:Vec<PathBuf>,
    /// Counterbalanced directed and dyadic bounds on identical original donor classes.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",num_args=2)]
    directed_primary_cost:Vec<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    primary_fixture_sdf:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    internal_transfer_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    spatial_point_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    launch_point_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    sticking_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    vertex_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    physical_secondary_long:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    physical_secondary_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_policy_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_interval_view:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    internal_interval_view:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    spatial_interval_view:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    launch_interval_view:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    secondary_long_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    retimed_long_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    deep_convergence:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    whole_primary:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    subdivided_primary:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    compact_primary:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    compact_primary_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    target_material_events:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    whole_primary_long:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    compact_primary_long:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    active_point_episode:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    active_point_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    active_episode_cost:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    whole_long_replay:Option<usize>,
    /// Run a fine-grid continuation from one shared, fully integrated first landing.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    matched_fine_suffix:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    whole_primary_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="whole_primary_replay")]
    whole_primary_control:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    cold_point_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="cold_point_replay")]
    cold_point_source:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="cold_point_replay",default_value_t=1)]
    cold_point_cut:usize,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    cold_field_cost:Option<PathBuf>,
    /// Time the complete first resumed Bevy step on one matched landing.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study", num_args = 2, value_names = ["CASE", "STROKE"], requires = "cold_fixture_source")]
    cold_fixture_cost: Vec<usize>,
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires = "cold_fixture_cost")]
    cold_fixture_source: Option<PathBuf>,
    /// Time this public update after advancing the same scene from its saved boundary.
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires = "cold_fixture_cost", default_value_t = 1, value_parser=clap::value_parser!(u32).range(1..=26))]
    cold_fixture_step: u32,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",num_args=2,value_names=["CASE","STROKE"])]
    cold_field_replay:Vec<usize>,
    /// Export the actual material transitions from the first public SDF step.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    observe_cold_interval:Option<PathBuf>,
    /// Record each of these successive public contact intervals.
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires = "observe_cold_interval", default_value_t = 1, value_parser=clap::value_parser!(u32).range(1..=26))]
    observe_cold_steps: u32,
    /// Save the separated foot and deck before the primary impact of fixture 80, stroke 2.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    free_landing_donor: bool,
    /// Save the incoming state and the carrying-edge force at fixture 80's rolling corner.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    corner_preload_donor: bool,
    /// Integrate two independent mechanical meshes from the preceding primary landing.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study",num_args=2,value_names=["CASE","STROKE"])]
    cold_field_reference:Vec<usize>,
    /// Reimport all three matched landings and audit the first loaded successor.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    cold_point_field:Option<PathBuf>,
    /// Allow contact transfers to redistribute the remaining native calls at a proved boundary.
    #[cfg(feature = "landing-audit")]
    #[arg(long,requires="cold_point_field")]
    cold_retime: bool,
    /// Pair cold starts with and without initial tangency detection at three public clocks.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    cold_point_episode:Option<PathBuf>,
    /// Restart an ordinary native suffix after 0.5, 1, 4, or 13 ms of qualified physical contact.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    physical_handoff:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,requires="physical_handoff",default_value_t=512)]
    handoff_grid:u32,
    /// Observe the next public tick after importing a shared physically integrated primary.
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    successor_onset:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    successor_onset_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    deep_onset:Option<PathBuf>,
    #[cfg(feature="landing-audit")]
    #[arg(long)]
    deep_onset_controls:bool,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    deep_onset_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    deep_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_long_replay:Option<usize>,
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    finite_replay:Option<usize>,
    /// Survey a deterministic matrix of local support offsets, speed and tier histories.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    grid_survey: bool,
    /// Follow a single changed primary through all remaining lift strokes on a common continuation.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    propagation_survey: bool,
    /// Compare three continuous grids over complete actuator strokes at slow, nominal and fast speed.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    convergence_survey:bool,
    /// Regenerate the continuous-grid native replay for a convergence case.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    convergence_replay:Option<usize>,
    /// Reproduce the native lanes of a completed long-horizon case.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    propagation_replay: Option<usize>,
    /// Regenerate a completed grid-survey fixture in gz-web-rs trace format.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    survey_replay: Option<usize>,
    /// Reproduce a completed fixed-time relaxation survey case.
    #[cfg(feature="landing-audit")]
    #[arg(long,group="study")]
    relaxation_replay: Option<usize>,
    /// Compare causal public-receipt and fixed-duration relaxation policies on a matched range.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    continuation_survey: bool,
    /// Reproduce the verified onset, weight-release and continuous relaxation lanes of a case.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    span_replay: Option<usize>,
    /// Time the six native continuation spans on the prespecified held-out donor cohort.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    span_cost: Option<PathBuf>,
    /// Time ordinary native intervals for held-out relaxation-survey donors from this directory.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    continuation_cost: Option<PathBuf>,
    /// Compare stopping and continuing relaxation after the same measured contact interval.
    #[cfg(feature = "landing-audit")]
    #[arg(long, group = "study")]
    continuation_replay: Option<usize>,
    /// One-based landing in the completed survey fixture.
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires = "survey_playback", default_value_t = 1)]
    survey_stroke: usize,
    /// Start at this zero-based position in the permuted fixture matrix.
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires="survey_range", default_value_t=0)]
    survey_start: usize,
    /// Run this many fixtures. Each has three independent matched primary landings.
    #[cfg(feature = "landing-audit")]
    #[arg(long, requires="survey_range", default_value_t=1)]
    survey_count: usize,
}

#[derive(Copy, Clone, Serialize, PartialEq)]
struct Body {
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    angular_velocity: [f64; 3],
}
impl From<BodyState> for Body {
    fn from(s: BodyState) -> Self {
        Self {
            position: s.position,
            orientation: s.orientation,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
        }
    }
}
#[derive(Copy, Clone, Serialize, PartialEq)]
struct Sample {
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    vertical_gap_m: f64,
}
/// Vertical clearance between the lowest point of the oriented sole and the
/// horizontal pad top. The sole stays over the pad throughout this fixture.
fn sole_gap(foot: Body, pad: Body) -> f64 {
    let up_local = DQuat::from_array(foot.orientation).conjugate() * DVec3::Z;
    let bottom = foot.position[2] - up_local.abs().dot(FOOT_HALF.into());
    bottom - pad.position[2] - PAD_HALF[2]
}

#[derive(Serialize)]
struct Frame {
    tick: u64,
    time_s: f64,
    cycle: u64,
    prescribed: bool,
    solved_prescribed: bool,
    switched: Sample,
    reference: Sample,
}

#[derive(Serialize)]
struct Recording {
    dt_s: f64,
    geometry: Value,
    frames: Vec<Frame>,
}

struct Lane {
    sim: Simulation,
    model: u64,
    pad_link: u64,
    foot: BodyId,
    upper: BodyId,
    pad: BodyId,
}
impl Lane {
    fn new() -> Result<Self> {
        Self::new_at(500)
    }
    fn new_at(step_us: u64) -> Result<Self> {
        Self::from_sdf_at(SDF, step_us)
    }
    fn from_sdf_at(source: &str, step_us: u64) -> Result<Self> {
        if step_us == 0 || !(SETTLE_STEPS * 500).is_multiple_of(step_us) {
            return Err("the native timestep must divide the initial 1.2 s settle".into());
        }
        let mut sim = load_world_with_engine(
            source,
            "reacquisition.sdf",
            "default",
            std::time::Duration::from_micros(step_us),
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        sim.step_exact(SETTLE_STEPS * 500 / step_us)?;
        Self::from_sim(sim)
    }
    fn from_sim(sim: Simulation)->Result<Self>{
        let id = |name: &str| -> Result<u64> {
            let scoped = format!("default::{name}");
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
                .and_then(|e| e.get::<EntityId>())
                .map(EntityId::get)
                .ok_or_else(|| format!("missing {scoped}").into())
        };
        let body = |name: &str| -> Result<BodyId> {
            let e = entity_by_id(sim.world(), id(name)?).ok_or("missing link entity")?;
            Ok(sim
                .world()
                .get::<BodyHandle>(e)
                .ok_or("missing native body")?
                .id())
        };
        Ok(Self {
            model: id("rocker")?,
            pad_link: id("lift::pad")?,
            foot: body("rocker::foot")?,
            upper: body("rocker::upper")?,
            pad: body("lift::pad")?,
            sim,
        })
    }
    fn sample(&self) -> Result<Sample> {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        let read = |id| {
            physics
                .state(id)
                .map(Body::from)
                .ok_or("missing native state")
        };
        let foot = read(self.foot)?;
        let pad = read(self.pad)?;
        let solver = physics
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local multifidelity backend")?;
        let normal_impulse_ns = solver
            .rigid_contact_observations()
            .iter()
            .filter(|p| (p.a, p.b) == (self.foot, self.pad) || (p.b, p.a) == (self.foot, self.pad))
            .map(|p| p.normal_impulse)
            .sum();
        Ok(Sample {
            foot,
            upper: read(self.upper)?,
            pad,
            normal_impulse_ns,
            vertical_gap_m: sole_gap(foot, pad),
        })
    }
    fn tier(&self) -> Result<ArticulationTier> {
        Ok(model_fidelity(&self.sim, self.model)?)
    }
    fn drive(&mut self, vz: f64) -> Result<()> {
        queue_link_world_velocity(&mut self.sim, self.pad_link, [0., 0., vz], [0.; 3])?;
        Ok(())
    }
    fn demote(&mut self) -> std::result::Result<(), FidelityError> {
        set_model_fidelity(&mut self.sim, self.model, Prescribed)
    }
}
/// Three stroke depths, with the same fast downward speed and three different return
/// velocities. The braking ramp uses the interval's mean velocity and integrates
/// exactly to the commanded height. Braking acceleration is 5 m/s² (< gravity).
struct Stroke {
    down: u64,
    plateau: u64,
    brake: u64,
    up: f64,
}
impl Stroke {
    fn start_park(&self) -> u64 {
        self.down + 24 + self.plateau + self.brake
    }
    fn velocity(&self, k: u64) -> f64 {
        let rise = self.down + 24;
        if k < self.down {
            -1.25
        } else if k < rise {
            0.
        } else if k < rise + self.plateau {
            self.up
        } else if k < self.start_park() {
            self.up * (1. - ((k - rise - self.plateau) as f64 + 0.5) / self.brake as f64)
        } else {
            0.
        }
    }
}
const STROKES: [Stroke; 3] = [
    Stroke {
        down: 48,
        plateau: 140,
        brake: 120,
        up: 0.30,
    },
    Stroke {
        down: 72,
        plateau: 110,
        brake: 180,
        up: 0.45,
    },
    Stroke {
        down: 96,
        plateau: 80,
        brake: 240,
        up: 0.60,
    },
];

fn deferred(result: std::result::Result<(), FidelityError>) -> Result<&'static str> {
    match result {
        Err(FidelityError::Physics(PhysicsError::TransitionDeferred { reason })) => Ok(reason),
        Err(e) => Err(e.into()),
        Ok(()) => Err("admitted a foot during flight or moving support".into()),
    }
}

#[derive(Serialize)]
struct Landing {
    tick: u64,
    time_s: f64,
    normal_impulse_ns: f64,
    momentum_balance_ns: f64,
    window_momentum_balance_ns: f64,
    window_normal_impulse_ns: f64,
    first_free_tick: u64,
    peak_free_gap_m: f64,
    incoming_foot_z_m_s: f64,
    landing_pad_z_m_s: f64,
    incoming_relative_z_m_s: f64,
}
fn landing(frames: &[Frame], dt: f64, lane: fn(&Frame) -> Sample) -> Result<Landing> {
    let mut airborne = false;
    let mut found = None;
    let mut first_free = None;
    let mut peak_gap = 0.0_f64;
    for (i, f) in frames.iter().enumerate() {
        let s = lane(f);
        // A positive rigid gap plus zero delivered impulse arms the event. The
        // lower bound keeps roundoff at a settled face out of the event detector.
        if s.normal_impulse_ns == 0. && s.vertical_gap_m > 1e-5 {
            first_free.get_or_insert(f.tick);
            airborne = true;
            peak_gap = peak_gap.max(s.vertical_gap_m);
        }
        if airborne && s.normal_impulse_ns > 0. {
            if found.replace(i).is_some() {
                return Err(format!("secondary landing at tick {}", f.tick).into());
            }
            airborne = false;
        }
    }
    let i = found.ok_or("no landing after a measured detached interval")?;
    let first = lane(&frames[i]);
    let prior = lane(&frames[i.checked_sub(1).ok_or("landing has no incoming sample")?]);
    let window = frames
        .get(i..i + WINDOW_STEPS)
        .ok_or("landing window incomplete")?;
    let window_sum: f64 = window.iter().map(|f| lane(f).normal_impulse_ns).sum();
    // Each articulated link is 1 kg. Vertical momentum change minus gravity
    // independently recovers the net pad impulse over the chosen interval.
    let foot_momentum = |s: Sample| s.foot.velocity[2] + s.upper.velocity[2];
    let impulse_balance = |end: Sample, ticks: usize| {
        foot_momentum(end) - foot_momentum(prior) + 2. * 9.81 * dt * ticks as f64
    };
    if first.pad.velocity[2] <= 0. || prior.foot.velocity[2] >= first.pad.velocity[2] {
        return Err("landing did not occur on the rising pad with a closing foot".into());
    }
    Ok(Landing {
        tick: frames[i].tick,
        time_s: frames[i].time_s,
        normal_impulse_ns: first.normal_impulse_ns,
        momentum_balance_ns: impulse_balance(first, 1),
        window_momentum_balance_ns: impulse_balance(lane(window.last().unwrap()), WINDOW_STEPS),
        window_normal_impulse_ns: window_sum,
        first_free_tick: first_free.ok_or("missing detached interval")?,
        peak_free_gap_m: peak_gap,
        incoming_foot_z_m_s: prior.foot.velocity[2],
        landing_pad_z_m_s: first.pad.velocity[2],
        incoming_relative_z_m_s: prior.foot.velocity[2] - first.pad.velocity[2],
    })
}

fn run() -> Result<(Value, Recording)> {
    let mut a = Lane::new()?;
    let mut b = Lane::new()?;
    let dt = a.sim.step_duration().as_secs_f64();
    let before = a.sample()?;
    if before != b.sample()? || before.normal_impulse_ns <= 0. {
        return Err("unmatched initial support".into());
    }
    a.demote()?;
    let mut frames = vec![Frame {
        tick: 0,
        time_s: 0.,
        cycle: 0,
        solved_prescribed: false,
        prescribed: true,
        switched: before,
        reference: b.sample()?,
    }];
    let mut cycles = Vec::new();
    let mut tick = 0;
    let mut last_v = 0.;
    for (index, stroke) in STROKES.iter().enumerate() {
        let cycle = index as u64 + 1;
        let depart = tick + PARK_STEPS + 1;
        let stop = depart + stroke.start_park();
        let mut promotion = None;
        let mut refusals = Vec::new();
        let mut settling_deferrals = 0;
        let start = frames.len();
        let mut admitted = None;
        while admitted.is_none() {
            tick += 1;
            let vz = if tick < depart {
                0.
            } else {
                stroke.velocity(tick - depart)
            };
            if vz != last_v {
                a.drive(vz)?;
                b.drive(vz)?;
                last_v = vz;
            }
            a.sim.step_exact(1)?;
            b.sim.step_exact(1)?;
            let solved = a.tier()?;
            let switched = a.sample()?;
            let reference = b.sample()?;
            if switched.pad != reference.pad {
                return Err("drive diverged between the native lanes".into());
            }
            if let Some(event) = support_loss_promotion::last_promotion(&a.sim, a.model)
                && event.iteration >= SETTLE_STEPS + depart
                && promotion.is_none()
            {
                if event.iteration != SETTLE_STEPS + tick
                    || event.link != a.foot
                    || event.support != a.pad
                    || solved != Dynamic
                {
                    return Err("receipt did not match the committed solve or pair".into());
                }
                promotion = Some((
                    tick,
                    json!({"tick":tick,"reason":format!("{:?}",event.reason),
                    "start_time_ns":event.start_time.as_nanos(),"end_time_ns":event.sim_time.as_nanos(),
                    "iteration":event.iteration,"model":event.model,"link":event.link.raw(),"support":event.support.raw()}),
                ));
            }
            // Challenge the actual admission gate in free flight and on the
            // still-rising lift after the impact. A deferred request is atomic.
            if tick == depart + stroke.down / 2 || tick == stop - stroke.brake / 2 {
                let reason = deferred(a.demote())?;
                refusals.push(json!({"tick":tick,"reason":reason,"vertical_gap_m":switched.vertical_gap_m,
                    "normal_impulse_ns":switched.normal_impulse_ns,"lift_velocity_z_m_s":switched.pad.velocity[2]}));
            }
            if tick >= stop {
                match a.demote() {
                    Ok(()) => admitted = Some(tick),
                    Err(FidelityError::Physics(PhysicsError::TransitionDeferred { .. })) => {
                        settling_deferrals += 1
                    }
                    Err(e) => return Err(e.into()),
                }
            }
            if tick > stop + 2400 {
                return Err(format!("cycle {cycle} failed to settle").into());
            }
            frames.push(Frame {
                tick,
                time_s: tick as f64 * dt,
                cycle,
                prescribed: a.tier()? == Prescribed,
                solved_prescribed: solved == Prescribed,
                switched,
                reference,
            });
        }
        let window = &frames[start..];
        let motion = &frames[depart as usize..];
        let sw = landing(motion, dt, |f| f.switched)?;
        let reference = landing(motion, dt, |f| f.reference)?;
        let comparison = json!({
            "landing_tick_offset":sw.tick as i64-reference.tick as i64,
            "first_normal_impulse_difference_ns":sw.normal_impulse_ns-reference.normal_impulse_ns,
            "window_normal_impulse_difference_ns":sw.window_normal_impulse_ns-reference.window_normal_impulse_ns,
            "incoming_relative_z_difference_m_s":sw.incoming_relative_z_m_s-reference.incoming_relative_z_m_s
        });
        let max_center_errors = window.iter().fold([0.0_f64; 2], |mut peak, f| {
            for (p, (x, y)) in peak.iter_mut().zip([
                (f.switched.foot, f.reference.foot),
                (f.switched.upper, f.reference.upper),
            ]) {
                *p = p.max(DVec3::from(x.position).distance(y.position.into()));
            }
            peak
        });
        let (promotion_tick, receipt) =
            promotion.ok_or("missing automatic promotion after departure")?;
        let settled = window.last().unwrap().switched;
        cycles.push(json!({"cycle":cycle,"departure_tick":depart,"promotion_tick":promotion_tick,"promotion":receipt,
            "stop_tick":stop,"demotion_tick":admitted,"parked_attempts_deferred":settling_deferrals,
            "stroke_depth_m":1.25*stroke.down as f64*dt,"return_speed_m_s":stroke.up,
            "switched_landing":sw,"reference_landing":reference,"comparison":comparison,"early_demotion_refusals":refusals,
            "redemotion":{"foot_velocity":settled.foot.velocity,"foot_angular_velocity":settled.foot.angular_velocity,
                "normal_impulse_ns":settled.normal_impulse_ns,"pad_velocity":settled.pad.velocity},
            "peak_center_error_m":{"foot":max_center_errors[0],"upper":max_center_errors[1]}}));
    }
    // Prove the final re-admission holds through more advancing steps too.
    for _ in 0..PARK_STEPS {
        tick += 1;
        a.sim.step_exact(1)?;
        b.sim.step_exact(1)?;
        let solved = a.tier()?;
        if solved != Prescribed {
            return Err("last parked hold did not persist".into());
        }
        frames.push(Frame {
            tick,
            time_s: tick as f64 * dt,
            cycle: 3,
            prescribed: true,
            solved_prescribed: true,
            switched: a.sample()?,
            reference: b.sample()?,
        });
    }
    let summary = json!({"dt_s":dt,"settle_s":SETTLE_STEPS as f64*dt,"initial_demotion_tick":0,"initial_normal_impulse_ns":before.normal_impulse_ns,
        "window_steps":WINDOW_STEPS,"window_s":WINDOW_STEPS as f64*dt,"end_tick":tick,
        "final_tier":"prescribed","model_id":a.model,"foot_body_id":a.foot.raw(),"pad_body_id":a.pad.raw(),"cycles":cycles});
    let data = Recording {
        dt_s: dt,
        geometry: json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}),
        frames,
    };
    Ok((summary, data))
}

#[cfg(feature="landing-audit")]
#[path="support_reacquisition/returning_box_sdf.rs"]
mod returning_box_sdf;
fn main() -> Result<()> {
    let args = Args::parse();
    #[cfg(feature = "landing-audit")]
    if args.registered_return_sdf {
        return returning_box_sdf::run(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if let Some(repeat)=args.secondary_fractional_sdf_cost{
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::material_policy::authored_cost(&args.output,repeat);
    }
    #[cfg(feature="landing-audit")]
    if let Some(id)=args.secondary_face_sdf {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::material_policy::authored(&args.output,id,args.secondary_event_bounds,args.secondary_skew_edge,args.secondary_fractional_points,args.secondary_fractional_retime);
    }
    #[cfg(feature="landing-audit")]
    if args.secondary_material_policy{
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::material_policy::field(&args.output,args.survey_start,args.survey_count,args.secondary_certify_vertices,args.secondary_event_bounds);
    }
    #[cfg(feature="landing-audit")]
    if let Some(id)=args.secondary_material_policy_replay{
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::material_policy::replay(&args.output,id,args.secondary_certify_vertices,args.secondary_event_bounds);
    }
    #[cfg(feature="landing-audit")]
    if let Some(index)=args.secondary_material_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::material_replay(&args.output,index);
    }
    #[cfg(feature="landing-audit")]
    if args.registered_return_field {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::registered_field(&args.output,args.survey_start,args.survey_count);
    }
    #[cfg(feature="landing-audit")]
    if args.secondary_return_feature_field || args.secondary_travel_feature_field {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::field(&args.output,args.survey_start,args.survey_count,args.secondary_travel_feature_field);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index)=args.secondary_return_feature_replay.or(args.secondary_travel_feature_replay) {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::features::replay(&args.output,index,args.secondary_travel_feature_replay.is_some());
    }
    #[cfg(feature = "landing-audit")]
    if let Some(repeat) = args.secondary_relaxation_cost.or(args.secondary_return_feature_cost).or(args.secondary_travel_feature_cost).or(args.secondary_material_cost).or(args.secondary_event_bound_cost) {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::cost::run(&args.output,repeat,args.secondary_return_feature_cost.is_some(),args.secondary_travel_feature_cost.is_some(),args.secondary_material_cost.is_some(),args.secondary_event_bound_cost.is_some());
    }
    #[cfg(feature = "landing-audit")]
    if args.secondary_relaxation_field || args.secondary_natural_field {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::field(&args.output,args.survey_start,args.survey_count,args.secondary_natural_field);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.secondary_relaxation_replay.or(args.secondary_natural_replay) {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::relaxation::replay(&args.output,index,args.secondary_natural_replay.is_some());
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.secondary_native_divergence {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::divergence::run(&args.output,index);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.secondary_order_pilot {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::cohort::order_pilot(&args.output, index);
    }
    #[cfg(feature = "landing-audit")]
    if args.secondary_cache_cohort || args.secondary_order_cohort {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::cohort::survey(
            &args.output, args.survey_start,args.survey_count,args.secondary_order_cohort);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.secondary_contact_points {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::points::run(&args.output, index);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.secondary_restart {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::landing_restart::run(&args.output, index);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.persistence_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::persistence::replay(&args.output, index);
    }
    #[cfg(feature = "landing-audit")]
    if args.persistence_prefix {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::persistence::survey(
            &args.output, args.survey_start, args.survey_count
        );
    }
    #[cfg(feature = "landing-audit")]
    if args.landing_energy {
        return touchdown::energy::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.rigid_energy {
        return touchdown::rigid_energy::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.refined_interval {
        return touchdown::contact_controls::rebound::weak_study::outer::interval::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.continuation_survey {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::span::survey(&args.output,args.survey_start,args.survey_count);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.span_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::span::replay(&args.output,index,args.survey_stroke);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(source) = args.span_cost {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::span::cost(&source,&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(source) = args.continuation_cost {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::cost::write(&source, &args.output);
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index) = args.continuation_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::replay_continuation(
            &args.output, index, args.survey_stroke,
        );
    }
    #[cfg(feature = "landing-audit")]
    if let Some(index)=args.relaxation_replay{
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::replay(&args.output,index,args.survey_stroke);
    }
    #[cfg(feature="landing-audit")]
    if let Some(index) = args.survey_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::survey::replay(&args.output,index,args.survey_stroke);
    }
    #[cfg(feature = "landing-audit")]
    if args.ballistic_arrivals {return touchdown::rigid_energy::hinge::friction::registered::arrivals::ballistic::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.registered_arrival_cost {return touchdown::rigid_energy::hinge::friction::registered::arrivals::cost(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.registered_arrivals {return touchdown::rigid_energy::hinge::friction::registered::arrivals::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.registered_corner {return touchdown::rigid_energy::hinge::friction::registered::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.spatial_hinge_impact {
        return touchdown::rigid_energy::hinge::friction::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.physical_secondary_long{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::physical::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.physical_secondary_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::physical::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.internal_interval_view{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::write(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::IntervalView::Transfer);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.spatial_interval_view{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::write(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::IntervalView::Spatial);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.launch_interval_view{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::write(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::IntervalView::Launch);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.secondary_interval_view{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::write(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::interval_view::IntervalView::Native);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.primary_fixture_sdf{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole_cost::fixture_sdf(&args.output,k,args.survey_stroke);}
    #[cfg(feature="landing-audit")]
    {
        use touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole_cost::{self,CostField};
        for (paths,study) in [(&args.whole_primary_cost,CostField::Primary),(&args.compact_primary_cost,CostField::Compact),(&args.directed_primary_cost,CostField::Directed)] {
            if let [source,control]=paths.as_slice(){return whole_cost::write(source,control,&args.output,study);}
        }
    }
    #[cfg(feature="landing-audit")]
    if let Some(source)=args.slip_launch_cost{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::cost::launch_cost(&source,&args.output);}
    #[cfg(feature="landing-audit")]
    if let Some(source)=args.bound_point_cost{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::cost::write(&source,&args.output);}
    #[cfg(feature="landing-audit")]
    if args.internal_transfer_long{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::long_transfer::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.internal_long_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::long_transfer::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if args.internal_transfer_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write_transfers(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.spatial_point_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write_spatial(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.launch_point_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write_launch(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.internal_transfer_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Transfers);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.spatial_point_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Spatial);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.launch_point_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Launch);}
    #[cfg(feature="landing-audit")]
    if args.sticking_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write_sticking(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.sticking_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Sticking);}
    #[cfg(feature="landing-audit")]
    if args.vertex_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write_vertices(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.vertex_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Sliding);}
    #[cfg(feature="landing-audit")]
    if args.secondary_policy{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.secondary_policy_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::replay(&args.output,k,args.survey_stroke,touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::policy::Panel::Landing);}
    #[cfg(feature="landing-audit")]
    if args.secondary_long{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::survey(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.secondary_long_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::secondary::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if args.retimed_long {return touchdown::contact_controls::rebound::weak_study::outer::propagation::retimed::survey(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.retimed_long_replay {return touchdown::contact_controls::rebound::weak_study::outer::propagation::retimed::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.cold_point_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point::replay(&args.output,args.cold_point_source.as_deref().ok_or("requires --cold-point-source")?,k,args.cold_point_cut);}
    #[cfg(feature="landing-audit")]
    if let [k,stroke]=args.cold_field_replay.as_slice(){return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point::field_replay(&args.output,*k,*stroke);}
    #[cfg(feature="landing-audit")]
    if let Some(sdf)=args.observe_cold_interval{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_reference::observe(&args.output,&sdf,args.observe_cold_steps);}
    #[cfg(feature="landing-audit")]
    if args.free_landing_donor {return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_reference::free_landing::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.corner_preload_donor {return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_reference::corner_arrival::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if let [k,stroke]=args.cold_field_reference.as_slice(){return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_reference::write(&args.output,*k,*stroke);}
    #[cfg(feature="landing-audit")]
    if let [case, stroke] = args.cold_fixture_cost.as_slice() {
        use touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point;
        return cold_point::fixture_cost(
            args.cold_fixture_source.as_deref().ok_or("missing fixture source")?,
            &args.output,
            *case,
            *stroke,
            args.cold_fixture_step,
        );
    }
    #[cfg(feature="landing-audit")]
    if let Some(source)=args.cold_field_cost{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point::cost(&source,&args.output);}
    #[cfg(feature="landing-audit")]
    if let Some(primary)=args.cold_point_field{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point::field(&args.output,&primary,args.survey_start,args.survey_count,args.cold_retime);}
    #[cfg(feature="landing-audit")]
    if let Some(episodes)=args.cold_point_episode{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::cold_point::write(&args.output,&episodes,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(episodes)=args.physical_handoff{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::handoff::write(&args.output,&episodes,args.survey_start,args.survey_count,args.handoff_grid);}
    #[cfg(feature="landing-audit")]
    if let Some(suffix)=args.successor_onset{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::onset::write_grid(&args.output,&suffix,args.survey_start,args.survey_count,true);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.successor_onset_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::onset::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if args.deep_onset_controls{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::onset::controls(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(deep)=args.deep_onset{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::onset::write(&args.output,&deep,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.deep_onset_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::onset::physical_replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.whole_primary_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole::replay(&args.output,k,args.survey_stroke,args.whole_primary_control.as_deref());}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.matched_fine_suffix{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::fine_suffix::write(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.whole_long_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole_long::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if let Some(source)=args.active_episode_cost{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::episode::cost(&source,&args.output);}
    #[cfg(feature="landing-audit")]
    if args.active_point_episode{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::episode::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.active_point_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::episode::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if args.whole_primary_long||args.compact_primary_long{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole_long::write(&args.output,args.survey_start,args.survey_count,args.compact_primary_long);}
    #[cfg(feature="landing-audit")]
    if args.target_material_events && !(args.compact_primary || args.subdivided_primary){return Err("--target-material-events requires a subdivided or compact primary field".into());}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.compact_primary_replay{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::compact::replay(&args.output,k,args.survey_stroke);}
    #[cfg(feature="landing-audit")]
    if args.compact_primary{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::compact::write(&args.output,args.survey_start,args.survey_count,args.target_material_events);}
    #[cfg(feature="landing-audit")]
    if args.whole_primary || args.subdivided_primary{return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::whole::write(&args.output,args.survey_start,args.survey_count,args.subdivided_primary,args.target_material_events);}
    #[cfg(feature="landing-audit")]
    if args.deep_convergence || args.finite_propagation {return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::survey(&args.output,args.survey_start,args.survey_count,args.deep_convergence);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.deep_replay.or(args.finite_long_replay) {return touchdown::contact_controls::rebound::weak_study::outer::propagation::finite::replay(&args.output,k,args.deep_replay.is_some());}
    #[cfg(feature="landing-audit")]
    if let Some(source)=args.finite_cost.as_deref().or(args.finite_transfer_cost.as_deref()){return touchdown::contact_controls::rebound::weak_study::outer::finite::policy::cost::write(source,&args.output,args.finite_transfer_cost.is_some());}
    #[cfg(feature="landing-audit")]
    if args.finite_policy {return touchdown::contact_controls::rebound::weak_study::outer::finite::policy::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.finite_replay {return touchdown::contact_controls::rebound::weak_study::outer::finite::policy::replay(&args.output,k,args.survey_stroke);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.second_replay{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::replay(&args.output,k,args.survey_stroke);}
    #[cfg(feature="landing-audit")]
    if let Some(dir)=args.secondary_cost.as_deref(){return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::cost::write(dir,&args.output);}
    #[cfg(feature="landing-audit")]
    if args.line_closure{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::closure::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.contact_sequence||args.clocked_sequence{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::after::sequence::write(&args.output,args.survey_start,args.survey_count,args.clocked_sequence);}
    #[cfg(feature="landing-audit")]
    if args.after_secondary{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::after::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.line_wrench{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::wrench::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.line_sticking{return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::line::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.second_point || args.registered_second_point || args.precise_second_point || args.work_second_point {return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::second::write(&args.output,args.survey_start,args.survey_count,args.registered_second_point || args.precise_second_point||args.work_second_point,args.precise_second_point||args.work_second_point,args.work_second_point);}
    #[cfg(feature="landing-audit")]
    if args.finite_transfer_policy {return touchdown::contact_controls::rebound::weak_study::outer::finite::policy::transfers(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.finite_transfer {return touchdown::contact_controls::rebound::weak_study::outer::finite::load::transfer::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.finite_load {return touchdown::contact_controls::rebound::weak_study::outer::finite::load::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.finite_survey {return touchdown::contact_controls::rebound::weak_study::outer::finite::write(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if args.convergence_survey {return touchdown::contact_controls::rebound::weak_study::outer::propagation::convergence::survey(&args.output,args.survey_start,args.survey_count);}
    #[cfg(feature="landing-audit")]
    if let Some(k)=args.convergence_replay {return touchdown::contact_controls::rebound::weak_study::outer::propagation::convergence::replay(&args.output,k);}
    #[cfg(feature="landing-audit")]
    if args.propagation_survey {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::survey(&args.output,args.survey_start,args.survey_count);
    }
    #[cfg(feature="landing-audit")]
    if let Some(i)=args.propagation_replay {
        return touchdown::contact_controls::rebound::weak_study::outer::propagation::replay(&args.output,i);
    }
    #[cfg(feature="landing-audit")]
    if args.passive_hinge_impact {
        return touchdown::rigid_energy::hinge::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.frictional_relaxation {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::study(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.relaxation_survey {
        return touchdown::contact_controls::rebound::weak_study::outer::relaxation::survey(&args.output,args.survey_start,args.survey_count);
    }
    #[cfg(feature = "landing-audit")]
    if args.grid_survey {
        return touchdown::contact_controls::rebound::weak_study::outer::survey::write(&args.output,args.survey_start,args.survey_count);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_grids {
        return touchdown::contact_controls::rebound::weak_study::outer::grids::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_response {
        return touchdown::rigid_energy::analytic::response::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.sdf_box_features {
        return touchdown::rigid_energy::analytic::sdf_features::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.inertia_frame_impact {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::gyro::frame_impact(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.separated_crossings_diagnose{
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::inspect(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_point || args.triaxial_departure {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::{self,Study};
        return projection::write(&args.output,if args.triaxial_departure {Study::Departure} else {Study::Projection});
    }
    #[cfg(feature="landing-audit")]
    if args.native_opposing_normals || args.native_opposing_normal_field || args.native_opposing_normal_cost || args.opposing_normal_arrivals || args.opposing_normal_arrival_field || args.opposing_normal_arrival_cost || args.opposing_normal_period || args.opposing_normal_period_field || args.opposing_normal_grids || args.opposing_normal_grid_field{use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::gaps::native_normal;return native_normal::write(&args.output,args.native_opposing_normal_field||args.opposing_normal_arrival_field||args.opposing_normal_period_field||args.opposing_normal_grid_field,args.native_opposing_normal_cost||args.opposing_normal_arrival_cost||args.opposing_normal_grids||args.opposing_normal_grid_field,args.opposing_normal_arrivals||args.opposing_normal_arrival_field||args.opposing_normal_arrival_cost,args.opposing_normal_period||args.opposing_normal_period_field||args.opposing_normal_grids||args.opposing_normal_grid_field);}
    #[cfg(feature="landing-audit")]
    if args.opposing_compact || args.opposing_compact_cost || args.opposing_compact_heldout{use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::gaps::compact;return compact::write(&args.output,args.opposing_compact_cost,args.opposing_compact_heldout);}
    #[cfg(feature="landing-audit")]
    if args.opposing_accumulation || args.opposing_materials {use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::gaps::terminal;return terminal::write(&args.output,args.opposing_materials);}
    #[cfg(feature="landing-audit")]
    if args.opposing_gap{use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::gaps;return gaps::write(&args.output);}
    #[cfg(feature="landing-audit")]
    if args.opposing_return || args.opposing_return_cost || args.opposing_return_view {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::projected::returns;
        return if args.opposing_return_cost {returns::cost(&args.output)} else if args.opposing_return_view{returns::write_view(&args.output)}else{returns::write(&args.output)?;returns::write_view(&args.output)};
    }
    #[cfg(feature="landing-audit")]
    if args.projected_ceiling || args.projected_ceiling_cost {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper::projected;
        return if args.projected_ceiling_cost {projected::cost(&args.output)} else {projected::write(&args.output)};
    }
    #[cfg(feature="landing-audit")]
    if args.upper_strike {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named::upper;
        return upper::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.named_opponent || args.named_opponent_cost {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors::named;
        return if args.named_opponent_cost {named::cost(&args.output)} else {named::write(&args.output)};
    }
    #[cfg(feature="landing-audit")]
    if args.rigid_point_neighbors || args.rigid_point_neighbors_cost {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::neighbors;
        return if args.rigid_point_neighbors_cost {neighbors::cost(&args.output)} else {neighbors::write(&args.output)};
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_short_returns {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::near_return::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_point_refusals {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::refusals(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_point_cost || args.triaxial_departure_cost {
        use touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::projection::{self,Study};
        return projection::cost(&args.output,if args.triaxial_departure_cost {Study::Departure} else {Study::Projection});
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_friction {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_friction_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::friction::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_gravity {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_gravity_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::gravity::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.compact_impact {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::compact::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.compact_impact_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::compact::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.separated_crossings {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.separated_crossings_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::sweep::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_returns_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.triaxial_returns {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::returns::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.released_rotation {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::released::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.released_rotation_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::released::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.rotation_impact_pulse {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::pulse::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.rotation_impact_pulse_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::pulse::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.cubic_rotation_timing {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::cubic::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.cubic_rotation_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::cubic::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.gyroscopic_flight {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::gyro::write(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.gyroscopic_flight_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::gyro::cost(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.gyroscopic_cut {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::gyro::diagnose(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.separated_flight {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::separated::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.separated_flight_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::separated::cost(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.rigid_early_activation {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::early::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.inertia_timing {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.inertia_timing_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::inertia_impact::cost(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.prospective_box_timing {
        return touchdown::rigid_energy::analytic::certification::sequence::production::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.prospective_box_timing_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::production::cost(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_sequence {
        return touchdown::rigid_energy::analytic::certification::sequence::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_sequence_cost {
        return touchdown::rigid_energy::analytic::certification::sequence::cost(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_certification_cost {
        return touchdown::rigid_energy::analytic::certification::cost::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_certification {
        return touchdown::rigid_energy::analytic::certification::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.impact_response_cost {
        return touchdown::rigid_energy::analytic::response::cost(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.analytic_pgs_cost {
        return touchdown::rigid_energy::analytic::cost::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.analytic_impact {
        return touchdown::rigid_energy::analytic::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.native_impact {
        return touchdown::contact_controls::rebound::weak_study::outer::points::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.primary_policy {
        return touchdown::contact_controls::rebound::weak_study::outer::primary_policy::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.primary_policy_timing {
        return touchdown::contact_controls::rebound::weak_study::outer::primary_policy::timing(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.outer_cadence {
        return touchdown::contact_controls::rebound::weak_study::outer::cadence::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.outer_divergence {
        return touchdown::contact_controls::rebound::weak_study::outer::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.weak_recontact {
        return touchdown::contact_controls::rebound::weak_study::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.weak_timing {
        return touchdown::contact_controls::rebound::weak_study::write_timing(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.rebound_flight {
        return touchdown::contact_controls::rebound::write_flight(&args.output);
    }
    #[cfg(feature="landing-audit")]
    if args.rebound_budget {
        return touchdown::contact_controls::rebound::write_budget(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.rebound_gate {
        return touchdown::contact_controls::rebound::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.contact_controls {
        return touchdown::contact_controls::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.touchdown_window {
        return touchdown::write(&args.output);
    }
    #[cfg(feature = "landing-audit")]
    if args.matched {
        return matched::write(&args.output);
    }
    if args.admission_study {
        return admission_study::write(&args.output);
    }
    if args.rigid_control {
        return rigid_control::write(&args.output);
    }
    if args.landing_sweep {
        return sweep::write(&args.output);
    }
    let (summary, data) = run()?;
    fs::create_dir_all(&args.output)?;
    let mut out = BufWriter::new(fs::File::create(args.output.join("trajectory.json"))?);
    serde_json::to_writer(&mut out, &data)?;
    out.flush()?;
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn three_rising_pad_landings_follow_measured_flight_and_rearm_the_same_foot() -> Result<()> {
        let (summary, recording) = run()?;
        let cycles = summary["cycles"].as_array().unwrap();
        assert_eq!(cycles.len(), STROKES.len());
        for c in cycles {
            let t = |key: &str| c[key].as_u64().unwrap() as usize;
            let departure = t("departure_tick");
            let parked = t("stop_tick");
            let demotion = t("demotion_tick");
            assert_eq!(t("promotion_tick"), departure);
            assert!(recording.frames[departure - 1].prescribed);
            assert!(!recording.frames[departure].solved_prescribed);
            assert_eq!(c["promotion"]["reason"], "MotionStarted");
            assert_eq!(c["promotion"]["support"], summary["pad_body_id"]);
            assert_eq!(c["promotion"]["link"], summary["foot_body_id"]);
            let impact = &c["switched_landing"];
            let reference = &c["reference_landing"];
            for landing in [impact, reference] {
                let free = landing["first_free_tick"].as_u64().unwrap() as usize;
                let hit = landing["tick"].as_u64().unwrap() as usize;
                assert!(departure <= free && free < hit && hit < parked && parked <= demotion);
                assert!(landing["peak_free_gap_m"].as_f64().unwrap() > 0.015);
                assert!(landing["incoming_relative_z_m_s"].as_f64().unwrap() < 0.);
                assert!(landing["landing_pad_z_m_s"].as_f64().unwrap() > 0.);
                for (contact, momentum) in [
                    ("normal_impulse_ns", "momentum_balance_ns"),
                    ("window_normal_impulse_ns", "window_momentum_balance_ns"),
                ] {
                    let impulse = landing[contact].as_f64().unwrap();
                    assert!(impulse > 0.0);
                    assert!(
                        (impulse - landing[momentum].as_f64().unwrap()).abs() < 1e-4,
                        "contact and whole-body momentum disagree on {contact}: {landing}"
                    );
                }
            }
            // Per-fixture guard on the three independently located impacts.
            assert_eq!(impact["tick"], reference["tick"]);
            for (key, bound) in [
                ("normal_impulse_ns", 0.04),
                ("window_normal_impulse_ns", 0.025),
            ] {
                let dynamic = reference[key].as_f64().unwrap();
                let delta = (impact[key].as_f64().unwrap() - dynamic).abs();
                assert!(
                    delta / dynamic < bound,
                    "cycle {} {key} relative deviation {}",
                    c["cycle"],
                    delta / dynamic
                );
            }
            let frame = &recording.frames[demotion];
            assert!(!frame.solved_prescribed && frame.prescribed);
            assert!(frame.switched.normal_impulse_ns > 0.);
            assert!(DVec3::from(frame.switched.foot.velocity).length() < 1e-3);
            assert_eq!(frame.switched.pad.velocity, [0.; 3]);
            assert!(recording.frames[demotion + 1].solved_prescribed);
            assert_eq!(c["early_demotion_refusals"].as_array().unwrap().len(), 2);
            assert!(
                recording.frames[demotion + 1..=demotion + PARK_STEPS as usize]
                    .iter()
                    .all(|f| f.solved_prescribed && f.switched.normal_impulse_ns == 0.)
            );
            assert!(
                recording.frames[departure..demotion]
                    .iter()
                    .all(|f| !f.prescribed)
            );
        }
        Ok(())
    }
}
