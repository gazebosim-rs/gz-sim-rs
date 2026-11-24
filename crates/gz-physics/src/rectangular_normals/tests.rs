// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const X: [f64; 4] = [-1., -1., 1., 1.];
const Y: [f64; 4] = [-1., 1., -1., 1.];
fn fixture() -> [[f64; 4]; 4] {
    let g =
        std::array::from_fn::<_, 4, _>(|i| DVec3::new(1., 0.17 * Y[i] - 0.02, 0.23 * X[i] + 0.08));
    let k = DMat3::from_cols(
        DVec3::new(0.7, 0.2, -0.3),
        DVec3::new(0.2, 12., -1.3),
        DVec3::new(-0.3, -1.3, 14.),
    );
    g.map(|a| g.map(|b| a.dot(k * b)))
}
fn moments(load: [f64; 4]) -> DVec3 {
    DVec3::new(
        load.iter().sum(),
        (load[2] + load[3]) - (load[0] + load[1]),
        (load[1] + load[3]) - (load[0] + load[2]),
    )
}
#[test]
fn equivalent_measures_preserve_pressure_moments_and_complementarity_under_scaling_and_symmetries()
{
    let w = fixture();
    for (j, v) in [
        ([0.11, 0.19, 0.27, 0.43], [0.; 4]),
        ([0.23, 0., 0., 0.77], [0.; 4]),
        ([0.31, 0.69, 0., 0.], [0., 0., 0.4, 0.4]),
        ([0., 0., 0.3, 0.], [0.5, 0.65, 0., 0.15]),
        ([0.; 4], [0.1, 0.13, 0.2, 0.23]),
    ] {
        let free =
            std::array::from_fn::<_, 4, _>(|i| v[i] - (0..4).map(|k| w[i][k] * j[k]).sum::<f64>());
        for perm in [
            [0, 1, 2, 3],
            [2, 3, 0, 1],
            [1, 0, 3, 2],
            [0, 2, 1, 3],
            [3, 1, 2, 0],
        ] {
            for mass in [1e-90, 1., 1e90] {
                for rate in [1e-65, 1., 1e65] {
                    let solve = RectangularNormalContact {
                        inverse_mass: perm.map(|i| perm.map(|k| w[i][k] / mass)),
                        free_normal: perm.map(|i| free[i] * rate),
                    }
                    .solve()
                    .unwrap();
                    let target = moments(perm.map(|i| j[i]));
                    assert!((solve.resultant / mass / rate - target.x).abs() < 3e-14);
                    if let Some([x, y]) = solve.normalized_center {
                        assert!(target.x > 0.);
                        assert!((x - target.y / target.x).abs() < 2e-13);
                        assert!((y - target.z / target.x).abs() < 2e-13);
                    } else {
                        assert_eq!(target.x, 0.);
                    }
                    let [a, b] = solve.equivalents.shift_range;
                    for s in [a, 0.5 * a + 0.5 * b, b] {
                        let loads = solve.equivalents.at(s).unwrap().map(|x| x / mass / rate);
                        assert!(loads.iter().all(|x| *x >= -1e-14));
                        assert!(moments(loads).distance(target) < 1e-13);
                        for (i, &k) in perm.iter().enumerate() {
                            let u = free[k] + (0..4).map(|p| w[k][perm[p]] * loads[p]).sum::<f64>();
                            assert!((u - v[k]).abs() < 3e-13);
                            assert!((solve.normal[i] / rate - u).abs() < 3e-13);
                            assert!((u * loads[i]).abs() < 3e-13);
                        }
                    }
                    if j[0] > 0. && j[3] > 0. {
                        assert!((b - a) / mass / rate > 0.1);
                    } else {
                        assert!((b - a) / mass / rate < 2e-14);
                    }
                }
            }
        }
    }
}
#[test]
fn nonaffine_inputs_indeterminate_rank_and_nonfinite_data_decline() {
    let w = fixture();
    let v = RectangularNormalContact {
        inverse_mass: w,
        free_normal: [0., 0.1, -0.3, -0.2],
    };
    let no = v.solve().unwrap();
    assert!(no.resultant > 0.);
    assert!(
        RectangularNormalContact {
            free_normal: [-1., -1., -1., -0.8],
            ..v
        }
        .solve()
        .is_err()
    );
    assert!(
        RectangularNormalContact {
            inverse_mass: [[1.; 4]; 4],
            ..v
        }
        .solve()
        .is_err()
    );
    let mut broken = v;
    broken.inverse_mass[0][2] += 0.1;
    assert!(broken.solve().is_err());
    assert!(
        RectangularNormalContact {
            free_normal: [f64::NAN; 4],
            ..v
        }
        .solve()
        .is_err()
    );
}
