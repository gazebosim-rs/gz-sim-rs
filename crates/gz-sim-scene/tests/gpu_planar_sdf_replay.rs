// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently compose visual poses from the arm's authored SDF and measured link frames.
use glam::{DQuat, DVec3, EulerRot};
use roxmltree::Node;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, error::Error, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Clone)]
struct Pose {
    xyz: DVec3,
    rotation: DQuat,
}
impl Pose {
    fn compose(&self, child: &Pose) -> Self {
        Self {
            xyz: self.xyz + self.rotation * child.xyz,
            rotation: self.rotation * child.rotation,
        }
    }
}
struct Visual {
    link: String,
    local: Pose,
    shape: Value,
}
fn values<const N: usize>(text: &str) -> Result<[f64; N]> {
    let row = text
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<std::result::Result<Vec<_>, _>>()?;
    row.try_into()
        .map_err(|_| format!("expected {N} coordinates").into())
}
fn child<'a, 'b>(node: Node<'a, 'b>, name: &str) -> Result<Node<'a, 'b>> {
    node.children()
        .find(|c| c.has_tag_name(name))
        .ok_or_else(|| format!("missing SDF {name}").into())
}
fn visual_bindings(sdf: &str) -> Result<BTreeMap<String, Visual>> {
    let doc = roxmltree::Document::parse(sdf)?;
    let world = doc
        .descendants()
        .find(|n| n.has_tag_name("world"))
        .ok_or("missing SDF world")?;
    let mut visuals = BTreeMap::new();
    for model in world.children().filter(|n| n.has_tag_name("model")) {
        for link in model.children().filter(|n| n.has_tag_name("link")) {
            let scope = format!(
                "{}::{}::{}",
                world.attribute("name").unwrap(),
                model.attribute("name").unwrap(),
                link.attribute("name").unwrap()
            );
            for vis in link.children().filter(|n| n.has_tag_name("visual")) {
                let pose = vis.children().find(|c| c.has_tag_name("pose"));
                if let Some(node) = pose
                    && node.attribute("relative_to").is_some()
                {
                    return Err("expected link-relative visual in this SDF fixture".into());
                }
                let [x, y, z, r, p, h] =
                    values(pose.and_then(|v| v.text()).unwrap_or("0 0 0 0 0 0"))?;
                let geometry = child(vis, "geometry")?
                    .children()
                    .find(|g| g.is_element())
                    .ok_or("empty geometry")?;
                let shape = match geometry.tag_name().name() {
                    "box" => {
                        json!({"kind":"box","size":values::<3>(child(geometry,"size")?.text().unwrap())?})
                    }
                    "sphere" => {
                        json!({"kind":"sphere","radius":child(geometry,"radius")?.text().unwrap().trim().parse::<f64>()?})
                    }
                    other => {
                        return Err(format!(
                            "unsupported display geometry in arm fixture: {other}"
                        )
                        .into());
                    }
                };
                let local = Pose {
                    xyz: DVec3::new(x, y, z),
                    rotation: DQuat::from_euler(EulerRot::ZYX, h, p, r),
                };
                let name = format!("{scope}::{}", vis.attribute("name").unwrap());
                assert!(
                    visuals
                        .insert(
                            name,
                            Visual {
                                link: scope.clone(),
                                local,
                                shape
                            }
                        )
                        .is_none()
                );
            }
        }
    }
    Ok(visuals)
}
#[test]
fn visual_child_transform_uses_authored_rotation_and_shape() -> Result<()> {
    let sdf = r#"<sdf><world name="w"><model name="arm"><link name="l"><visual name="tip"><pose>1 0 0 0 0 1.5707963267948966</pose><geometry><sphere><radius>0.03</radius></sphere></geometry></visual></link></model></world></sdf>"#;
    let v = visual_bindings(sdf)?;
    let v = &v["w::arm::l::tip"];
    assert_eq!(v.link, "w::arm::l");
    assert_eq!(v.shape["radius"], 0.03);
    let parent = Pose {
        xyz: DVec3::new(1., 2., 3.),
        rotation: DQuat::from_rotation_y(-std::f64::consts::FRAC_PI_2),
    };
    let world = parent.compose(&v.local);
    assert!((world.xyz - DVec3::new(1., 2., 4.)).length() < 1e-12);
    assert!((world.rotation * DVec3::X - DVec3::Y).length() < 1e-12);
    Ok(())
}

#[test]
#[ignore = "requires a recorded GPU SDF run and its full-rate measured link trace"]
fn all_displayed_arm_poses_follow_the_measured_sdf_links() -> Result<()> {
    let root = env::var_os("GZ_GPU_SDF_REPLAY_AUDIT").ok_or("set GZ_GPU_SDF_REPLAY_AUDIT")?;
    let root = Path::new(&root);
    let read = |p: &str| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(root.join(p))?)?) };
    let binding = visual_bindings(include_str!("../../../examples/gpu_planar_tool_world.sdf"))?;
    let trace = read("qualified/frames.json")?;
    let trace = trace.as_array().unwrap();
    assert_eq!(trace.len(), 601);
    let mut samples = Vec::new();
    let mut max_error = [0f64; 2];
    let mut last = 0;
    let mut count = 0;
    for (tick, row) in trace.iter().enumerate() {
        assert_eq!(row["time_ns"], tick as u64 * 1_000_000);
        let mut poses = BTreeMap::new();
        for (n, visual) in &binding {
            let link = &row["links"][&visual.link];
            let pose = Pose {
                xyz: DVec3::from_array(serde_json::from_value(link["translation"].clone())?),
                rotation: DQuat::from_array(serde_json::from_value(link["rotation"].clone())?),
            }
            .compose(&visual.local);
            if n.ends_with("::tool") {
                assert!(
                    pose.xyz.distance(DVec3::from_array(serde_json::from_value(
                        row["gpu_tip_m"].clone()
                    )?)) < 1e-12
                );
                for (i, k) in ["cartesian_tip_m", "rapier_tip_m"].into_iter().enumerate() {
                    max_error[i] = max_error[i].max(
                        pose.xyz
                            .distance(DVec3::from_array(serde_json::from_value(row[k].clone())?)),
                    );
                }
            }
            poses.insert(
                n,
                json!({"translation":pose.xyz.to_array(),"rotation":pose.rotation.to_array()}),
            );
        }
        if row["gpu_normal_impulse_ns"].as_f64().unwrap() > 0. {
            last = tick;
            count += 1;
        }
        samples.push(json!({"time_ns":row["time_ns"],"poses":poses}));
    }
    for (err, expected) in max_error.into_iter().zip(
        read("qualified/comparison.json")?["max_tip_error_m"]
            .as_array()
            .unwrap(),
    ) {
        assert!((err - expected.as_f64().unwrap()).abs() < 1e-12);
    }
    assert_eq!(last, 314);
    assert!(count > 200);
    let mut ticks = Vec::new();
    for line in fs::read_to_string(root.join("qualified/replay/frames.jsonl"))?.lines() {
        let record: Value = serde_json::from_str(line)?;
        if record["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(record["scene"].as_str().unwrap())?;
        let tick = (scene["nanoseconds"].as_u64().unwrap() / 1_000_000) as usize;
        for v in scene["visuals"].as_array().unwrap() {
            let name = v["name"].as_str().unwrap();
            assert_eq!(v["shape"], binding[name].shape);
        }
        ticks.push(tick);
    }
    assert_eq!(ticks.first(), Some(&0));
    assert!(ticks.last().is_some_and(|last| (567..=600).contains(last)));
    assert!(ticks.windows(2).all(|p| p[1] > p[0] && p[1] - p[0] <= 36));
    let loaded = ticks
        .iter()
        .position(|&t| trace[t]["gpu_normal_impulse_ns"].as_f64().unwrap() > 0.)
        .unwrap();
    let out = root.join("viewer");
    fs::create_dir_all(&out)?;
    fs::write(out.join("expected.json"), serde_json::to_vec(&samples)?)?;
    let mapping: BTreeMap<_, _> = binding.keys().map(|n| (n, n)).collect();
    let index = json!({"kind":"authored-archive-v1","camera":"perspective","dt_ns":1,"source_poses":"../../viewer/expected.json","source_visual_poses":mapping,"captures":{"start":0,"loaded":loaded,"final":ticks.len()-1}});
    fs::write(out.join("index.json"), serde_json::to_vec_pretty(&index)?)?;
    Ok(())
}

#[test]
#[ignore = "requires an ordinary transport-command replay and subscribed robot joint states"]
fn command_replay_follows_subscribed_angles_and_measured_ground_contact() -> Result<()> {
    let root =
        env::var_os("GZ_GPU_COMMAND_REPLAY_AUDIT").ok_or("set GZ_GPU_COMMAND_REPLAY_AUDIT")?;
    let root = Path::new(&root);
    let sdf = include_str!("../../../examples/gpu_commanded_tool_world.sdf");
    let doc = roxmltree::Document::parse(sdf)?;
    let links = visual_bindings(sdf)?;
    let named = |tag, n| {
        doc.descendants()
            .find(|e| e.has_tag_name(tag) && e.attribute("name") == Some(n))
            .ok_or("missing robot element")
    };
    let pose = |node: Node<'_, '_>| -> Result<Pose> {
        let [x, y, z, r, p, h] = values(child(node, "pose")?.text().unwrap())?;
        Ok(Pose {
            xyz: DVec3::new(x, y, z),
            rotation: DQuat::from_euler(EulerRot::ZYX, h, p, r),
        })
    };
    let base = pose(named("model", "arm")?)?;
    let floor = pose(named("model", "floor")?)?;
    let shoulder = pose(named("link", "upper")?)?;
    let elbow = pose(named("link", "forearm")?)?;
    let axis = |name| -> Result<DVec3> {
        Ok(DVec3::from_array(values(
            child(child(named("joint", name)?, "axis")?, "xyz")?
                .text()
                .unwrap(),
        )?)
        .normalize())
    };
    let axes = [axis("shoulder")?, axis("elbow")?];
    let trace: Vec<Value> = serde_json::from_slice(&fs::read(root.join("qualified/trace.json"))?)?;
    assert_eq!(trace.len(), 600);
    let mut frames = vec![];
    let mut loaded = vec![];
    for tick in 0..=trace.len() {
        let mut a = [0.; 2];
        if tick > 0 {
            let row = &trace[tick - 1];
            assert_eq!(row["tick_ms"], tick);
            a = serde_json::from_value(row["position_rad"].clone())?;
        }
        let joint = |q, angle| Pose {
            xyz: DVec3::ZERO,
            rotation: DQuat::from_axis_angle(q, angle),
        };
        let top = base.compose(&shoulder).compose(&joint(axes[0], a[0]));
        let lower = top.compose(&elbow).compose(&joint(axes[1], a[1]));
        let mut poses = BTreeMap::new();
        for (n, v) in &links {
            let parent = match v.link.as_str() {
                "default::arm::upper" => &top,
                "default::arm::forearm" => &lower,
                "default::floor::base" => &floor,
                _ => return Err("unexpected SDF display link".into()),
            };
            let p = parent.compose(&v.local);
            if n.ends_with("::tool") && tick > 0 {
                let gap = p.xyz.z - v.shape["radius"].as_f64().unwrap() - floor.xyz.z;
                assert!((gap - trace[tick - 1]["gap_m"].as_f64().unwrap()).abs() < 1e-12);
            }
            poses.insert(
                n,
                json!({"translation":p.xyz.to_array(),"rotation":p.rotation.to_array()}),
            );
        }
        if tick > 0 && trace[tick - 1]["normal_impulse_ns"].as_f64().unwrap() > 0. {
            loaded.push(tick);
        }
        frames.push(json!({"time_ns":tick*1_000_000,"poses":poses}));
    }
    assert_eq!((loaded.first(), loaded.last()), (Some(&65), Some(&311)));
    let mut captures = vec![];
    for line in fs::read_to_string(root.join("qualified/replay/frames.jsonl"))?.lines() {
        let row: Value = serde_json::from_str(line)?;
        if row["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(row["scene"].as_str().unwrap())?;
        let tick = (scene["nanoseconds"].as_u64().unwrap() / 1_000_000) as usize;
        assert!(tick <= 600);
        for v in scene["visuals"].as_array().unwrap() {
            assert_eq!(v["shape"], links[v["name"].as_str().unwrap()].shape);
        }
        captures.push(tick);
    }
    let on = captures.iter().position(|t| loaded.contains(t)).unwrap();
    let off = captures.iter().position(|t| t > &311).unwrap();
    let out = root.join("viewer");
    fs::create_dir_all(&out)?;
    fs::write(out.join("expected.json"), serde_json::to_vec(&frames)?)?;
    let names: BTreeMap<_, _> = links.keys().map(|n| (n, n)).collect();
    fs::write(
        out.join("index.json"),
        serde_json::to_vec_pretty(
            &json!({"kind":"authored-archive-v1","camera":"perspective","dt_ns":1,"source_poses":"../../viewer/expected.json","source_visual_poses":names,"captures":{"start":0,"contact":on,"lift":off,"final":captures.len()-1}}),
        )?,
    )?;
    Ok(())
}
