// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare projected and retimed secondary landings at the first original
//! actuator boundary. All interventions get the same state and remainder.
use super::*;
const TREATMENTS: [(&str, u32, Action); 8] = [
    ("native2", 2, Action::Native),
    ("endpoint2", 2, Action::Endpoint),
    ("retimed2", 2, Action::Retime),
    ("native8", 8, Action::Native),
    ("endpoint8", 8, Action::Endpoint),
    ("retimed8", 8, Action::Retime),
    ("native4096", 4096, Action::Native),
    ("native8192", 8192, Action::Native),
];
fn row(branch: &Branch) -> Value {
    let tick = &branch.ticks[0];
    json!({"first":tick.as_json(),"impulse":branch.impulse,"world_impulse_ns":branch.first_world_impulse.to_array()})
}
const VERTICES: [(&str, u32, Action); 12] = [
    ("endpoint2", 2, Action::Endpoint),
    ("follow2", 2, Action::Follow),
    ("retimed2", 2, Action::Retime),
    ("follow_retimed2", 2, Action::FollowRetimed),
    ("endpoint8", 8, Action::Endpoint),
    ("follow8", 8, Action::Follow),
    ("retimed8", 8, Action::Retime),
    ("follow_retimed8", 8, Action::FollowRetimed),
    ("endpoint64", 64, Action::Endpoint),
    ("follow64", 64, Action::Follow),
    ("retimed64", 64, Action::Retime),
    ("follow_retimed64", 64, Action::FollowRetimed),
];
const STICKING: [(&str, u32, Action); 12] = [
    ("endpoint2", 2, Action::Endpoint),
    ("stick2", 2, Action::Stick),
    ("retimed2", 2, Action::Retime),
    ("stick_retimed2", 2, Action::StickRetimed),
    ("endpoint8", 8, Action::Endpoint),
    ("stick8", 8, Action::Stick),
    ("retimed8", 8, Action::Retime),
    ("stick_retimed8", 8, Action::StickRetimed),
    ("endpoint64", 64, Action::Endpoint),
    ("stick64", 64, Action::Stick),
    ("retimed64", 64, Action::Retime),
    ("stick_retimed64", 64, Action::StickRetimed),
];
const TRANSFERS: [(&str, u32, Action); 12] = [
    ("bound2", 2, Action::Both),
    ("transfer2", 2, Action::Transfer),
    ("bound_retimed2", 2, Action::BothRetimed),
    ("transfer_retimed2", 2, Action::TransferRetimed),
    ("bound8", 8, Action::Both),
    ("transfer8", 8, Action::Transfer),
    ("bound_retimed8", 8, Action::BothRetimed),
    ("transfer_retimed8", 8, Action::TransferRetimed),
    ("bound64", 64, Action::Both),
    ("transfer64", 64, Action::Transfer),
    ("bound_retimed64", 64, Action::BothRetimed),
    ("transfer_retimed64", 64, Action::TransferRetimed),
];
const SPATIAL: [(&str, u32, Action); 12] = [
    ("transfer2", 2, Action::Transfer),
    ("spatial2", 2, Action::Spatial),
    ("transfer_retimed2", 2, Action::TransferRetimed),
    ("spatial_retimed2", 2, Action::SpatialRetimed),
    ("transfer8", 8, Action::Transfer),
    ("spatial8", 8, Action::Spatial),
    ("transfer_retimed8", 8, Action::TransferRetimed),
    ("spatial_retimed8", 8, Action::SpatialRetimed),
    ("transfer64", 64, Action::Transfer),
    ("spatial64", 64, Action::Spatial),
    ("transfer_retimed64", 64, Action::TransferRetimed),
    ("spatial_retimed64", 64, Action::SpatialRetimed),
];
const LAUNCH: [(&str, u32, Action); 12] = [
    ("spatial2", 2, Action::Spatial),
    ("launch2", 2, Action::Launch),
    ("spatial_retimed2", 2, Action::SpatialRetimed),
    ("launch_retimed2", 2, Action::LaunchRetimed),
    ("spatial8", 8, Action::Spatial),
    ("launch8", 8, Action::Launch),
    ("spatial_retimed8", 8, Action::SpatialRetimed),
    ("launch_retimed8", 8, Action::LaunchRetimed),
    ("spatial64", 64, Action::Spatial),
    ("launch64", 64, Action::Launch),
    ("spatial_retimed64", 64, Action::SpatialRetimed),
    ("launch_retimed64", 64, Action::LaunchRetimed),
];
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Panel {
    Landing,
    Sliding,
    Sticking,
    Transfers,
    Spatial,
    Launch,
}
impl Panel {
    fn treatments(self) -> &'static [(&'static str, u32, Action)] {
        match self {
            Self::Landing => &TREATMENTS,
            Self::Sliding => &VERTICES,
            Self::Sticking => &STICKING,
            Self::Launch => &LAUNCH,
            Self::Spatial => &SPATIAL,
            Self::Transfers => &TRANSFERS,
        }
    }
}
fn collect(f: Fixture, panel: Panel) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (i, c) in cuts.into_iter().enumerate() {
        let Some(c) = c else { continue };
        let mut t = json!({"cut":snapshot(&c),"stroke":i+1});
        if let Some(lab) = laboratory(f, &c)? {
            let mut rows = vec![];
            for &(id, n, a) in panel.treatments() {
                // A native interval occupies at least one integer nanosecond.
                // Keep the actual work count in every row, including a saturated fine reference.
                let actual = n.min((DT - lab.clock_offset).as_nanos() as u32);
                let profile = p(id, b(actual, 1), b(8, 1));
                let branch = path(f, &c, &lab, profile, a, c.tick + 1)?;
                if rows.is_empty() {
                    t["initial"] = branch.initial.clone()
                } else if t["initial"] != branch.initial {
                    return Err("unequal cold initial state".into());
                }
                let mut result = row(&branch);
                result["profile"] = json!(id);
                if panel != Panel::Landing {
                    result["spans"] = json!(branch.vertices);
                }
                if matches!(panel, Panel::Transfers | Panel::Spatial | Panel::Launch) {
                    result["transfers"] = json!(branch.transfers);
                }
                rows.push(result);
            }
            t["paths"] = json!(rows);
            t["offset_ns"] = json!(lab.clock_offset.as_nanos() as u64);
            t["geometric_time_s"] = json!(lab.geometric_time_s);
            t["prehistory"] = lab.original;
        }
        trials.push(t)
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-interval-policy-v1",
        |f| collect(f, Panel::Landing),
    )
}
pub(crate) fn write_vertices(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-sliding-vertex-v1",
        |f| collect(f, Panel::Sliding),
    )
}
pub(crate) fn write_spatial(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-sliding-skew-edge-v1",
        |f| collect(f, Panel::Spatial),
    )
}
pub(crate) fn write_launch(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-skew-slip-launch-v1",
        |f| collect(f, Panel::Launch),
    )
}
pub(crate) fn write_transfers(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-internal-transfer-v1",
        |f| collect(f, Panel::Transfers),
    )
}
pub(crate) fn write_sticking(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(
        dir,
        first,
        count,
        "finite-secondary-sticking-point-v1",
        |f| collect(f, Panel::Sticking),
    )
}
pub(crate) fn replay(dir: &Path, k: usize, stroke: usize, panel: Panel) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("out of range secondary case".into());
    }
    let d: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing cut")?;
    let donor = laboratory(f, &c)?.ok_or("missing cold second impact")?;
    let t = d["result"]["trials"]
        .as_array()
        .ok_or("no trials")?
        .iter()
        .find(|t| t["stroke"] == stroke)
        .ok_or("missing matched stroke")?;
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    fs::create_dir_all(folder.join("traces"))?;
    if panel == Panel::Launch {
        let ns = (DT - donor.clock_offset).as_nanos();
        let xml=super::super::super::finite::load::transfer::second::substituted(&donor.xml,"max_step_size",&format!("{}.{:09}",ns/1_000_000_000,ns%1_000_000_000))?
            .replace("</pair>","<resolve_point_transfers>true</resolve_point_transfers><follow_sliding_skew_edge>true</follow_sliding_skew_edge><follow_skew_slip_launch>true</follow_skew_slip_launch></pair>");
        fs::write(folder.join("matched.sdf"), xml)?;
    }
    let mut lanes = vec![];
    let choices = if panel == Panel::Launch {
        [
            (4, "Bound spatial points, eight calls", 8),
            (5, "Certified slip launch, eight calls", 8),
            (9, "Certified slip launch, 64 calls", 64),
        ]
    } else if panel == Panel::Spatial {
        [
            (4, "Internal transfers, eight calls", 8),
            (5, "Sliding skew edges and transfers, eight calls", 8),
            (9, "Sliding skew edges and transfers, 64 calls", 64),
        ]
    } else if panel == Panel::Transfers {
        [
            (4, "Following individual material points, eight calls", 8),
            (5, "Resolving internal transfers, eight calls", 8),
            (9, "Resolving internal transfers, 64 calls", 64),
        ]
    } else if panel == Panel::Sticking {
        [
            (4, "Projected endpoint and eight native calls", 8),
            (5, "Fixed material point with eight calls", 8),
            (9, "Fixed material point with 64 calls", 64),
        ]
    } else if panel == Panel::Sliding {
        [
            (4, "Projected endpoint and eight native calls", 8),
            (5, "Sustained sliding vertex and eight calls", 8),
            (9, "Sustained sliding vertex and 64 calls", 64),
        ]
    } else {
        [
            (4, "Uniform eight after the certified endpoint", 8),
            (5, "Retimed eight after the same endpoint", 8),
            (6, "Continuous 4,096 ordinary native calls", 4096),
        ]
    };
    for (index, label, suffix) in choices {
        let (id, n, action) = panel.treatments()[index];
        let branch = path(
            f,
            &c,
            &donor,
            p(
                id,
                b(n.min((DT - donor.clock_offset).as_nanos() as u32), 1),
                b(suffix, 1),
            ),
            action,
            c.tick + 26,
        )?;
        let mut first = row(&branch);
        first["profile"] = json!(id);
        if panel != Panel::Landing {
            first["spans"] = json!(branch.vertices);
        }
        if matches!(panel, Panel::Transfers | Panel::Spatial | Panel::Launch) {
            first["transfers"] = json!(branch.transfers);
        }
        if serde_json::from_slice::<Value>(&serde_json::to_vec(&first)?)? != t["paths"][index] {
            return Err("replay changed archived first interval".into());
        }
        let mut frames = vec![branch.initial];
        frames.extend(branch.ticks.iter().map(Tick::as_json));
        let file = format!("traces/{id}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":id,"name":label,"file":file}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":if panel==Panel::Launch{"The foot starts slipping along a loaded skew edge"}else if panel==Panel::Spatial{"Following sliding contact between finite skew edges"}else if panel==Panel::Transfers{"Resolving changes of bearing inside a native call"}else if panel==Panel::Sticking{"Following the sticking point between skew edges"}else if panel==Panel::Sliding{"Following a sliding vertex after the moving support lands"}else{"Retiming the native call following the second impulse"},"pair_totals_only":true,"lanes":lanes}),
    )
}
