// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}
const HELD: PadReading = PadReading {
    normal_force_n: 2.,
    slip_speed_m_s: 0.,
};
#[test]
fn slip_is_continuous_advancing_time_and_pressure_can_recover() {
    let slipping = PadReading {
        slip_speed_m_s: -0.03,
        ..HELD
    };
    let mut guard = GraspGuard::default();
    assert_eq!(guard.observe(ms(3), [HELD, slipping]), GraspResponse::Hold);
    assert_eq!(guard.observe(ms(1), [HELD; 2]), GraspResponse::Hold);
    assert_eq!(guard.observe(ms(3), [slipping, HELD]), GraspResponse::Hold);
    assert_eq!(guard.observe(ms(0), [slipping; 2]), GraspResponse::Hold);
    assert_eq!(guard.observe(ms(1), [slipping; 2]), GraspResponse::Tighten);
    assert_eq!(guard.observe(ms(48), [slipping; 2]), GraspResponse::Tighten);
    assert_eq!(guard.observe(ms(20), [HELD; 2]), GraspResponse::Tighten);
    assert_eq!(
        guard.observe(ms(4), [HELD, PadReading::default()]),
        GraspResponse::SetDown
    );
    assert_eq!(guard.observe(ms(500), [HELD; 2]), GraspResponse::SetDown);
}
#[test]
fn invalid_readings_and_absent_pressure_are_fail_safe() {
    for p in [
        PadReading::default(),
        PadReading {
            normal_force_n: f64::INFINITY,
            ..HELD
        },
        PadReading {
            slip_speed_m_s: f64::NAN,
            ..HELD
        },
    ] {
        let mut guard = GraspGuard::default();
        assert_eq!(guard.observe(ms(4), [p, HELD]), GraspResponse::Tighten);
        assert_eq!(guard.observe(ms(49), [HELD, p]), GraspResponse::SetDown);
    }
}
