// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify native replay clocks, poses, geometry and impulse indicators in Chromium.
mod authored;
mod journal_visuals;
mod gpu_arm_visuals;
mod gpu_tool_visuals;
mod particle_visuals;
mod corner_preload;
mod edge_compliance;
mod free_landing;
mod moving_seam;
mod sole_turntable;
mod welded_foot;
mod landing_audit;
mod promotion;
mod reacquisition;
mod touchdown;
mod two_supports;
use base64::{Engine, engine::general_purpose::STANDARD};
use serde_json::{Value, json};
use std::{
    collections::{BTreeMap, BTreeSet},
    error::Error,
    fs,
    net::TcpStream,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    thread,
    time::{Duration, Instant},
};
use tungstenite::{Message, WebSocket};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn sleep(ms: u64) {
    thread::sleep(Duration::from_millis(ms));
}
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}
#[derive(Default)]
struct Oracle {
    trace: Value,
    index: Value,
    scene_ticks: BTreeSet<u64>,
    ordinals: BTreeSet<u64>,
    last: Option<u64>,
    finished: bool,
    webgpu: bool,
    support_presence: BTreeMap<(u64, u64), bool>,
    support_catalogs: BTreeSet<bool>,
    pending_scene: Value,
    pending_catalog: Value,
}
fn vector<const N: usize>(v: &Value) -> Result<[f64; N]> {
    let row = v.as_array().ok_or("vector")?;
    if row.len() != N {
        return Err("invalid vector size".into());
    }
    let mut out = [0.; N];
    for (o, c) in out.iter_mut().zip(row) {
        *o = c.as_f64().ok_or("coordinate")?;
    }
    Ok(out)
}
fn same<const N: usize>(actual: &Value, expected: [f64; N]) -> Result<bool> {
    Ok(vector::<N>(actual)?
        .iter()
        .zip(expected)
        .all(|(v, e)| close(*v, e)))
}
impl Oracle {
    fn scene(&mut self, scene: &Value) -> Result<()> {
        if self.index["kind"] == "support-loss-promotion-v1"
            || self.index["kind"] == "authored-archive-v1"
        {
            self.pending_scene = scene.clone();
            return Ok(());
        }
        let ns = scene["seconds"].as_u64().ok_or("seconds")? * 1_000_000_000
            + scene["nanoseconds"].as_u64().ok_or("nanoseconds")?;
        let dt = self.index["dt_ns"].as_u64().ok_or("step size")?;
        if !ns.is_multiple_of(dt) {
            return Err("off-clock scene".into());
        }
        let tick = ns / dt;
        if matches!(
            self.index["kind"].as_str(),
            Some(
                "touchdown-window-v1"
                    | "contact-energy-v1"
                    | "rebound-gate-v1"
                    | "rebound-budget-v1"
                    | "rebound-flight-v1"
                    | "weak-recontact-v1"
                    | "outer-divergence-v1"
                    | "native-comparison-v1"
            )
        ) {
            touchdown::verify(scene, &self.trace, &self.index, tick)?;
            self.scene_ticks.insert(tick);
            self.last = Some(tick);
            return Ok(());
        }
        if self.index["kind"] == "landing-audit-v1" {
            landing_audit::verify(scene, &self.trace, &self.index, tick)?;
            self.scene_ticks.insert(tick);
            self.last = Some(tick);
            return Ok(());
        }
        let s = &self.trace["frames"][tick as usize];
        if s["tick"] != tick {
            return Err("unmeasured frame".into());
        }
        if self.index["kind"] == "support-reacquisition-v1" {
            reacquisition::scene(scene, &self.trace, &self.index, tick)?;
            self.last = Some(tick);
            self.scene_ticks.insert(tick);
            return Ok(());
        }
        if self.index["kind"] == "two-support-foot-v1" {
            two_supports::scene(scene, &self.trace, &self.index, tick)?;
            self.support_presence.insert(
                (
                    scene["epoch"].as_u64().ok_or("epoch")?,
                    scene["sequence"].as_u64().ok_or("sequence")?,
                ),
                s["support_b_present"].as_bool().ok_or("support presence")?,
            );
            self.last = Some(tick);
            self.scene_ticks.insert(tick);
            return Ok(());
        }
        let geometry = &self.trace["geometry"];
        let spacing = self.index["spacing_m"].as_f64().ok_or("spacing")?;
        let cx = self.index["center_x"].as_f64().ok_or("center x")?;
        let visuals = scene["visuals"].as_array().ok_or("visual list")?;
        if visuals.len() != 30 {
            return Err("unexpected visual count".into());
        }
        let first = self.index["promotion_tick"].as_u64().ok_or("promotion")?;
        let full = self.index["gauge_max_ns"].as_f64().ok_or("gauge scale")?;
        let width = self.index["gauge_width_m"].as_f64().ok_or("gauge width")?;
        let z = self.index["gauge_z"].as_f64().ok_or("gauge z")?;
        for (lane, name, shift) in [
            ("switched", "switched", spacing / 2.),
            ("reference", "always_dynamic", -spacing / 2.),
        ] {
            let find = |body: &str| {
                visuals
                    .iter()
                    .find(|v| v["name"] == format!("default::{name}_{body}::visual"))
                    .ok_or_else(|| format!("missing {name} {body} at {tick}"))
            };
            let point = |[x, y, z]: [f64; 3]| [y, shift - (x - cx), z];
            for (body, extent) in [
                ("foot", "foot_half_extents"),
                ("upper", "upper_half_extents"),
            ] {
                let g = find(body)?;
                let measured = &s[lane][body];
                let [x, y, z, w] = vector(&measured["orientation"])?;
                let r = std::f64::consts::FRAC_1_SQRT_2;
                let q = [r * (x + y), r * (y - x), r * (z - w), r * (w + z)];
                if g["shape"]["kind"] != "box"
                    || !same(
                        &g["shape"]["size"],
                        vector::<3>(&geometry[extent])?.map(|v| 2. * v),
                    )?
                    || !same(&g["rotation"], q)?
                    || !same(&g["translation"], point(vector(&measured["position"])?))?
                {
                    return Err(
                        format!("incorrect pose or shape at tick {tick} in {name}/{body}").into(),
                    );
                }
            }
            let floor = find("ledge")?;
            let center = vector::<3>(&geometry["ledge_center"])?;
            let size = vector::<3>(&geometry["ledge_half_extents"])?.map(|v| 2. * v);
            let geometry_edge = center[0] + size[0] / 2.;
            if !same(&floor["shape"]["size"], size)? || !same(&floor["translation"], point(center))?
            {
                return Err("incorrect physical support edge".into());
            }
            let impulse = self.trace["frames"][first as usize][lane]["normal_impulse_ns"]
                .as_f64()
                .ok_or("first ground reaction")?;
            let fraction = if tick < first { 0. } else { impulse / full };
            let y = vector::<3>(&geometry["ledge_center"])?[1];
            if !same(
                &find("first_reaction")?["translation"],
                point([cx + (fraction - 0.5) * width, y, z - 0.10 * width / 0.74]),
            )? {
                return Err(format!("incorrect ground impulse at {tick}").into());
            }
            let span = self.index["edge_ruler_half_span_m"]
                .as_f64()
                .ok_or("edge span")?;
            let [lx, ly, lz] = vector::<3>(&geometry["witness_local_m"])?;
            let [qx, qy, qz, qw] = vector::<4>(&s[lane]["foot"]["orientation"])?;
            let [px, py, pz] = vector::<3>(&s[lane]["foot"]["position"])?;
            let toe = [
                px + (1. - 2. * (qy * qy + qz * qz)) * lx
                    + 2. * (qx * qy - qw * qz) * ly
                    + 2. * (qx * qz + qw * qy) * lz,
                py + 2. * (qy * qx + qw * qz) * lx
                    + (1. - 2. * (qx * qx + qz * qz)) * ly
                    + 2. * (qy * qz - qw * qx) * lz,
                pz + 2. * (qz * qx - qw * qy) * lx
                    + 2. * (qz * qy + qw * qx) * ly
                    + (1. - 2. * (qx * qx + qy * qy)) * lz,
            ];
            if !same(&find("toe_witness")?["translation"], point(toe))? {
                return Err(format!("incorrect local toe witness in {lane} at {tick}").into());
            }
            let relative = s[lane]["edge_distance_m"]
                .as_f64()
                .ok_or("missing toe distance")?;
            // Check the two rotation formulas in physical meters before magnifying the signal.
            if !close(relative, toe[0] - geometry_edge) {
                return Err(
                    format!("toe distance disagrees with the native foot pose at {tick}").into(),
                );
            }
            let x = cx + 0.5 * width * (relative / span).clamp(-1., 1.);
            if !same(
                &find("toe_relative_to_edge")?["translation"],
                point([x, y, z + 0.03 * width / 0.74]),
            )? {
                return Err(format!("wrong edge position in {lane} at {tick}").into());
            }
        }
        self.last = Some(tick);
        self.scene_ticks.insert(tick);
        Ok(())
    }
}
#[cfg(test)]
mod focus_tests;
struct Chrome {
    socket: WebSocket<TcpStream>,
    session: Option<String>,
    next: u64,
    oracle: Oracle,
}
impl Chrome {
    fn recv(&mut self) -> Result<Value> {
        let v: Value = serde_json::from_slice(&self.socket.read()?.into_data())?;
        match v["method"].as_str() {
            Some("Network.webSocketFrameReceived") => {
                if let Some(data) = v["params"]["response"]["payloadData"].as_str()
                    && let Ok(frame) = serde_json::from_str::<Value>(data)
                {
                    match frame["kind"].as_str() {
                        Some("scene") => self.oracle.scene(&frame)?,
                        Some("lifecycle_catalog")
                            if self.oracle.index["kind"] == "support-loss-promotion-v1"
                                || self.oracle.index["kind"] == "authored-archive-v1" =>
                        {
                            self.oracle.pending_catalog = frame;
                        }
                        Some("lifecycle_catalog")
                            if self.oracle.index["kind"] == "two-support-foot-v1" =>
                        {
                            let key = (
                                frame["epoch"].as_u64().ok_or("catalog epoch")?,
                                frame["scene_sequence"].as_u64().ok_or("catalog sequence")?,
                            );
                            if let Some(&present) = self.oracle.support_presence.get(&key) {
                                two_supports::catalog(&frame, present)?;
                                self.oracle.support_catalogs.insert(present);
                            }
                        }
                        Some("replay_status") => {
                            if self.oracle.index["kind"] == "authored-archive-v1" {
                                let i = authored::verify(
                                    &self.oracle.pending_scene,
                                    &self.oracle.pending_catalog,
                                    &frame,
                                    &self.oracle.trace,
                                )?;
                                self.oracle.last = Some(i);
                                self.oracle.scene_ticks.insert(i);
                            }
                            if self.oracle.index["kind"] == "support-loss-promotion-v1" {
                                let id = promotion::check(
                                    &self.oracle.pending_scene,
                                    &self.oracle.pending_catalog,
                                    &frame,
                                    &self.oracle.trace,
                                    &self.oracle.index,
                                )?;
                                self.oracle.last = Some(id);
                                self.oracle.scene_ticks.insert(id);
                            }
                            if let Some(ordinal) = frame["frame"].as_u64() {
                                self.oracle.ordinals.insert(ordinal);
                            }
                            self.oracle.finished = frame["finished"] == true;
                        }
                        _ => {}
                    }
                }
            }
            Some("Network.responseReceived") => {
                let r = &v["params"]["response"];
                if r["url"]
                    .as_str()
                    .is_some_and(|u| u.ends_with("/webgpu/gzweb_bg.wasm"))
                    && r["status"] == 200
                {
                    self.oracle.webgpu = true;
                }
            }
            Some("Runtime.exceptionThrown") => {
                return Err(format!("viewer exception {}", v["params"]).into());
            }
            _ => {}
        }
        Ok(v)
    }
    fn call(&mut self, method: &str, params: Value) -> Result<Value> {
        self.next += 1;
        let id = self.next;
        let mut msg = json!({"id":id,"method":method,"params":params});
        if let Some(s) = &self.session {
            msg["sessionId"] = json!(s);
        }
        self.socket.send(Message::Text(msg.to_string().into()))?;
        loop {
            let v = self.recv()?;
            if v["id"] == id {
                if !v["error"].is_null() {
                    return Err(v.to_string().into());
                }
                return Ok(v["result"].clone());
            }
        }
    }
    fn until(&mut self, done: impl Fn(&Oracle) -> bool) -> Result<()> {
        let deadline = Instant::now() + Duration::from_secs(110);
        while !done(&self.oracle) {
            if Instant::now() > deadline {
                return Err(format!("timeout at {:?}", self.oracle.last).into());
            }
            self.recv()?;
        }
        Ok(())
    }
    fn paints(&mut self) -> Result<()> {
        self.call("Runtime.evaluate",json!({"expression":"new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(() => requestAnimationFrame(resolve))))","awaitPromise":true}))?;
        Ok(())
    }
    fn key_event(&mut self, kind: &str, key: &str, code: &str, vk: u32, mods: u32) -> Result<()> {
        self.call("Input.dispatchKeyEvent",json!({"type":kind,"key":key,"code":code,"windowsVirtualKeyCode":vk,"nativeVirtualKeyCode":vk,"modifiers":mods}))?;
        // Let Bevy sample the held key and modifier on separate paint cycles.
        self.paints()?;
        sleep(250);
        Ok(())
    }
    fn key(&mut self, key: &str, code: &str, vk: u32, mods: u32) -> Result<()> {
        self.key_event("keyDown", key, code, vk, mods)?;
        self.key_event("keyUp", key, code, vk, mods)?;
        sleep(180);
        Ok(())
    }
    fn zoom_out(&mut self, steps: u64) -> Result<()> {
        for _ in 0..steps.min(24) {
            self.call(
                "Input.dispatchMouseEvent",
                json!({"type":"mouseWheel","x":748,"y":458,"deltaX":0,"deltaY":200}),
            )?;
            self.paints()?;
        }
        Ok(())
    }
    fn focus_visual(&mut self, follow_sheet: bool, perspective: bool, row: u64) -> Result<()> {
        // In the folded-grasp scene the first material node follows the ten
        // robot and support visuals in the expanded outliner.
        if row > 23 {
            return Err("visual row is below the visible outliner".into());
        }
        let y = if follow_sheet { 460 } else { 200 + 28 * row as u32 };
        self.click_at(196, y)?;
        self.call(
            "Input.dispatchMouseEvent",
            json!({"type":"mouseMoved","x":743,"y":470}),
        )?;
        if perspective {
            self.key("h", "KeyH", 72, 0)?;
        }
        self.key("f", "KeyF", 70, 0)?;
        sleep(520);
        if follow_sheet {
            self.key_event("keyDown", "Shift", "ShiftLeft", 16, 8)?;
            self.key("F", "KeyF", 70, 8)?;
            self.key_event("keyUp", "Shift", "ShiftLeft", 16, 0)?;
            for _ in 0..10 {
                self.call(
                    "Input.dispatchMouseEvent",
                    json!({"type":"mouseWheel","x":745,"y":470,"deltaX":0,"deltaY":200}),
                )?;
                sleep(50);
            }
        } else if perspective {
            self.call(
                "Input.dispatchMouseEvent",
                json!({"type":"mouseWheel","x":745,"y":470,"deltaX":0,"deltaY":140}),
            )?;
            sleep(280);
        }
        Ok(())
    }
    fn click(&mut self, x: u32) -> Result<()> {
        self.click_at(x, 22)
    }
    fn click_at(&mut self, x: u32, y: u32) -> Result<()> {
        // Bevy samples pointer state on UI frames. Background tabs can receive
        // CDP mouse events without producing an active toolbar interaction.
        self.call("Page.bringToFront", json!({}))?;
        self.call(
            "Emulation.setFocusEmulationEnabled",
            json!({"enabled":true}),
        )?;
        self.call(
            "Input.dispatchMouseEvent",
            json!({"type":"mouseMoved","x":x,"y":y}),
        )?;
        self.paints()?;
        sleep(250);
        for (ty, buttons) in [("mousePressed", 1), ("mouseReleased", 0)] {
            self.call(
                "Input.dispatchMouseEvent",
                json!({"type":ty,"button":"left","buttons":buttons,"clickCount":1,"x":x,"y":y}),
            )?;
            self.paints()?;
            sleep(250);
        }
        Ok(())
    }
    // An offscreen browser can lose focus while the Bevy UI is first laid out.
    // Retry only while the credited replay clock still shows its old position.
    fn change_playback(&mut self, x: u32, started: bool) -> Result<()> {
        for _ in 0..3 {
            self.click(x)?;
            for _ in 0..4 {
                self.paints()?;
                if self.oracle.last.is_some_and(|t| (t != 0) == started) {
                    return Ok(());
                }
                sleep(200);
            }
        }
        Err(format!(
            "toolbar action at x={x} left replay clock at {:?}",
            self.oracle.last
        )
        .into())
    }
    fn capture(&mut self, p: &Path) -> Result<()> {
        let b = self.call(
            "Page.captureScreenshot",
            json!({"format":"jpeg","quality":90}),
        )?;
        fs::write(
            p,
            STANDARD.decode(b["data"].as_str().ok_or("missing screenshot")?)?,
        )?;
        Ok(())
    }
}

fn run(profile: &Path, source: &Path, index: &Path, output: &Path, port: u16, line_start: Option<&Path>) -> Result<()> {
    let view_dir = index.parent().ok_or("missing replay index directory")?;
    let index: Value = serde_json::from_slice(&fs::read(index)?)?;
    let ns_per_tick = index["dt_ns"].as_u64().ok_or("native timestep")?;
    let automatic = index["kind"] == "support-loss-promotion-v1";
    let authored = index["kind"] == "authored-archive-v1";
    let compliance = index["material_pressure_bins"].as_u64();
    let mut corner_readings = None;
    let mut surface_readings = None;
    let mut seam_readings = None;
    let mut turntable_readings = None;
    let mut weld_readings = None;
    let archival = if index["welded_foot"] == true {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        weld_readings = Some(welded_foot::verify(source, &archive,index["welded_impact_branch"]==true)?);
        Some(archive)
    } else if index["sole_turntable"] == true {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        turntable_readings = Some(sole_turntable::verify(source, &archive)?);
        Some(archive)
    } else if index["moving_seam"] == true {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        seam_readings = Some(moving_seam::verify(source, &archive)?);
        Some(archive)
    } else if index["free_landing"] == true {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        surface_readings = Some(free_landing::verify(source, &archive, index["normal_damping"] == true)?);
        Some(archive)
    } else if let Some(bins) = index["corner_pressure_bins"].as_u64() {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        corner_readings = Some(corner_preload::verify(source, line_start.ok_or("line-start reference path required")?, &archive, bins.try_into()?)?);
        Some(archive)
    } else if let Some(bins) = compliance {
        let archive = authored::read(&view_dir.join("replay"), &index)?;
        edge_compliance::verify(source, &archive, usize::try_from(bins)?)?;
        Some(archive)
    } else if authored {
        Some(authored::read(source, &index)?)
    } else {
        None
    };
    let physical: Vec<u64> = if let Some(raw) = archival.as_ref() {
        if ns_per_tick != 1 {
            return Err("archive oracle indexes physical time in nanoseconds".into());
        }
        raw.as_array()
            .ok_or("archived frames")?
            .iter()
            .map(|r| r["ns"].as_u64().ok_or("archive clock"))
            .collect::<std::result::Result<_, _>>()?
    } else if automatic {
        index["frames"]
            .as_array()
            .ok_or("frame index")?
            .iter()
            .map(|r| r["tick"].as_u64().ok_or("missing tick"))
            .collect::<std::result::Result<_, _>>()?
    } else {
        serde_json::from_value(index["ticks"].clone())?
    };
    let times: Vec<u64> = if automatic || authored {
        (0..physical.len() as u64).collect()
    } else {
        physical.clone()
    };
    let multi = index["kind"] == "two-support-foot-v1";
    let reacquisition = index["kind"] == "support-reacquisition-v1";
    let landing_audit = index["kind"] == "landing-audit-v1";
    let touchdown = index["kind"] == "touchdown-window-v1";
    let energy = index["kind"] == "contact-energy-v1";
    let policy_activity = index["activity_source"].is_string();
    let rebound = index["kind"] == "rebound-gate-v1";
    let budget = index["kind"] == "rebound-budget-v1";
    let flight = index["kind"] == "rebound-flight-v1";
    let weak = index["kind"] == "weak-recontact-v1";
    let outer = index["kind"] == "outer-divergence-v1";
    let native = index["kind"] == "native-comparison-v1";
    let has_reference_gauges = weak || outer || native;
    let paired_native = touchdown || energy || rebound || budget || flight || has_reference_gauges;
    let indexed_captures = reacquisition || landing_audit || paired_native || authored;
    let events: &[(&str, &str)] = if automatic || indexed_captures {
        &[]
    } else if multi {
        two_supports::EVENTS
    } else {
        &[
            ("demotion", "demotion_tick"),
            ("edge", "crossing_tick"),
            ("resumed", "promotion_tick"),
        ]
    };
    let mut marks: BTreeMap<u64, Vec<String>> = BTreeMap::new();
    if indexed_captures {
        for (label, tick) in index["captures"].as_object().ok_or("event capture index")? {
            marks
                .entry(tick.as_u64().ok_or("capture clock")?)
                .or_default()
                .push(label.clone());
        }
    }
    for &(name, key) in events {
        marks
            .entry(index[key].as_u64().ok_or("event clock")?)
            .or_default()
            .push(name.to_owned());
    }
    if automatic {
        for &label in promotion::LABELS {
            marks
                .entry(
                    index["captures"][label]
                        .as_u64()
                        .ok_or_else(|| format!("no event {label}"))?,
                )
                .or_default()
                .push(label.to_owned());
        }
    }
    let trace = if let Some(raw) = archival {
        raw
    } else if paired_native {
        touchdown::input(source, &index)?
    } else if landing_audit {
        serde_json::from_slice(&fs::read(
            source.join(
                index["trace_file"]
                    .as_str()
                    .ok_or("missing trace filename")?,
            ),
        )?)?
    } else if reacquisition {
        reacquisition::input(source)?
    } else if automatic {
        promotion::input(source)?
    } else {
        serde_json::from_slice(&fs::read(source.join("trajectory.json"))?)?
    };
    let addresses = profile.join("DevToolsActivePort");
    let start = Instant::now();
    while !addresses.exists() {
        if start.elapsed().as_secs() > 20 {
            return Err("Chrome did not start".into());
        }
        sleep(80);
    }
    let port_file = fs::read_to_string(addresses)?;
    let mut lines = port_file.lines();
    let cdp: u16 = lines.next().ok_or("missing debugging port")?.parse()?;
    let stream = TcpStream::connect(("127.0.0.1", cdp))?;
    stream.set_read_timeout(Some(Duration::from_secs(60)))?;
    let (socket, _) = tungstenite::client(
        format!(
            "ws://127.0.0.1:{cdp}{}",
            lines.next().ok_or("missing debugger address")?
        ),
        stream,
    )?;
    let mut chrome = Chrome {
        socket,
        session: None,
        next: 0,
        oracle: Oracle {
            trace,
            index,
            ..Oracle::default()
        },
    };
    let target = chrome.call("Target.createTarget", json!({"url":"about:blank"}))?;
    let attach = chrome.call(
        "Target.attachToTarget",
        json!({"targetId":target["targetId"],"flatten":true}),
    )?;
    chrome.session = Some(
        attach["sessionId"]
            .as_str()
            .ok_or("no page attachment")?
            .into(),
    );
    for method in ["Page.enable", "Network.enable", "Runtime.enable"] {
        chrome.call(method, json!({}))?;
    }
    chrome.call(
        "Emulation.setDeviceMetricsOverride",
        json!({"width":1440,"height":900,"deviceScaleFactor":1,"mobile":false}),
    )?;
    // Hold a single presentation credit at each event so the live canvas has time to paint.
    let event_clock = if automatic || authored {
        "if(m.kind==='replay_status')this.clock=m.frame-1".to_owned()
    } else {
        format!("if(m.kind==='scene')this.clock=(1e9*m.seconds+m.nanoseconds)/{ns_per_tick}")
    };
    let gate = format!(
        r#"const marks={:?};const released=new Set();const waiting=new Map();
    window.releaseCapture=t=>{{released.add(t);waiting.get(t)?.();waiting.delete(t);}};window.WebSocket=class extends WebSocket {{
    constructor(...args){{super(...args);this.addEventListener('message',e=>{{try{{const m=JSON.parse(e.data);{event_clock};}}catch{{}}}});}}
    send(s){{if(window.captureArmed && marks.includes(this.clock) && !released.has(this.clock) && JSON.parse(s).kind==='publication_ready'){{waiting.set(this.clock,()=>super.send(s));}}else super.send(s);}} }};"#,
        marks
            .keys()
            .copied()
            .filter(|t| *t != 0)
            .collect::<Vec<_>>()
    );
    chrome.call(
        "Page.addScriptToEvaluateOnNewDocument",
        json!({"source":gate}),
    )?;
    let frontend_port = std::env::var("GZWEB_FRONTEND_PORT")
        .ok()
        .map(|port| port.parse::<u16>())
        .transpose()?
        .unwrap_or(8082);
    let url = format!(
        "http://127.0.0.1:{frontend_port}/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A{port}%2F"
    );
    chrome.call("Page.navigate", json!({"url":url}))?;
    chrome.until(|o| o.last.is_some() && o.webgpu)?;
    sleep(1200);
    if chrome.oracle.last != Some(0) {
        chrome.oracle.scene_ticks.clear();
        chrome.oracle.ordinals.clear();
        chrome.click(655)?;
    }
    chrome.until(|o| o.last == Some(0) && o.ordinals.contains(&1))?;
    chrome.call(
        "Input.dispatchMouseEvent",
        json!({"type":"mouseMoved","x":720,"y":470}),
    )?;
    if authored && chrome.oracle.index["camera"] == "top" {
        chrome.key("7", "Digit7", 55, 0)?;
    } else if authored && chrome.oracle.index["camera"] == "perspective" {
        chrome.key("h", "KeyH", 72, 0)?;
    } else if authored && chrome.oracle.index["camera"] == "right" {
        chrome.key("3", "Digit3", 51, 0)?;
    } else if indexed_captures {
        chrome.key("1", "Digit1", 49, 0)?;
    } else {
        chrome.key_event("keyDown", "Shift", "ShiftLeft", 16, 8)?;
        chrome.key("!", "Digit1", 49, 8)?;
        chrome.key_event("keyUp", "Shift", "ShiftLeft", 16, 0)?;
    }
    chrome.call(
        "Input.dispatchMouseEvent",
        json!({"type":"mouseMoved","x":1000,"y":65}),
    )?;
    sleep(160);
    chrome.key("f", "KeyF", 70, 0)?;
    sleep(500);
    // Both complete lanes and their gauges fit between the full UI's side panels.
    chrome.call(
        "Input.dispatchMouseEvent",
        json!({"type":"mouseWheel","x":745,"y":470,"deltaX":0,"deltaY":if authored && matches!(chrome.oracle.index["camera"].as_str(),Some("top" | "perspective")){-80}else if chrome.oracle.index["free_landing"] == true {160} else if automatic{130}else if has_reference_gauges{-80}else{-250}}),
    )?;
    sleep(400);
    if authored && chrome.oracle.index["gpu_arm_frames"].is_string() {
        // View the arms obliquely so the X-Z joint planes and the three rows are visible.
        let (x, y, dx, dy) = if chrome.oracle.index["gpu_tool_contact"] == true {
            (735, 150, 6, 8)
        } else {
            (745, 450, 22, 3)
        };
        chrome.call("Input.dispatchMouseEvent", json!({"type":"mousePressed","button":"left","buttons":1,"x":x,"y":y,"clickCount":1}))?;
        sleep(100);
        for n in 1..=8 {
            chrome.call("Input.dispatchMouseEvent", json!({"type":"mouseMoved","button":"left","buttons":1,"x":x + dx*n,"y":y + dy*n}))?;
            sleep(30);
        }
        chrome.call("Input.dispatchMouseEvent", json!({"type":"mouseReleased","button":"left","buttons":0,"x":x + 8*dx,"y":y + 8*dy,"clickCount":1}))?;
        sleep(220);
        chrome.key("f", "KeyF", 70, 0)?;
        sleep(400);
    }
    let follow_sheet = chrome.oracle.index["follow_folded_sheet"] == true;
    let visual_row = chrome.oracle.index["focus_visual_row"].as_u64();
    if authored
        && (chrome.oracle.index["focus_first_visual"] == true || visual_row.is_some() || follow_sheet)
    {
        chrome.capture(&output.join("overview.jpg"))?;
        chrome.focus_visual(
            follow_sheet,
            chrome.oracle.index["focus_perspective"] == true,
            visual_row.unwrap_or(0),
        )?;
    }
    let zoom_steps = chrome.oracle.index["zoom_out_steps"].as_u64().unwrap_or(0);
    chrome.zoom_out(zoom_steps)?;
    chrome.capture(&output.join("initial.jpg"))?;
    for name in marks.remove(&0).into_iter().flatten() {
        if name == "initial" {
            continue;
        }
        fs::copy(
            output.join("initial.jpg"),
            output.join(format!("{name}.jpg")),
        )?;
    }
    chrome.call(
        "Runtime.evaluate",
        json!({"expression":"window.captureArmed=true"}),
    )?;
    if rebound || budget || flight || has_reference_gauges || authored {
        chrome.change_playback(688, true)?;
    } else {
        chrome.click(688)?;
    }
    for (tick, names) in marks {
        chrome.until(|o| o.last == Some(tick))?;
        chrome.paints()?;
        sleep(260);
        for name in names {
            chrome.capture(&output.join(format!("{name}.jpg")))?;
        }
        if chrome.oracle.last != Some(tick) {
            return Err("event capture drifted".into());
        }
        chrome.call(
            "Runtime.evaluate",
            json!({"expression":format!("window.releaseCapture({tick})")}),
        )?;
    }
    chrome.until(|o| o.finished)?;
    sleep(300);
    chrome.capture(&output.join("final.jpg"))?;
    let expected: BTreeSet<u64> = times.iter().copied().collect();
    if chrome.oracle.scene_ticks != expected
        || chrome.oracle.ordinals != (1..=times.len() as u64).collect()
    {
        return Err("browser skipped recorded frames".into());
    }
    if rebound || budget || flight || has_reference_gauges || authored {
        chrome.change_playback(655, false)?;
    } else {
        chrome.click(655)?;
    }
    chrome.until(|o| o.last == Some(0))?;
    let mut clocks = BTreeMap::new();
    if indexed_captures {
        for (label, tick) in chrome.oracle.index["captures"]
            .as_object()
            .ok_or("event capture index")?
        {
            clocks.insert(
                label.as_str(),
                if authored {
                    *physical
                        .get(tick.as_u64().ok_or("capture ordinal")? as usize)
                        .ok_or("capture exceeds recorded interval")? as f64
                        / 1e6
                } else {
                    tick.as_u64().ok_or("capture clock")? as f64 * ns_per_tick as f64 / 1e6
                },
            );
        }
    }
    for &(name, key) in events {
        clocks.insert(
            name,
            chrome.oracle.index[key].as_u64().unwrap() as f64 * ns_per_tick as f64 / 1e6,
        );
    }
    if automatic {
        for &label in promotion::LABELS {
            let id = chrome.oracle.index["captures"][label].as_u64().unwrap() as usize;
            clocks.insert(label, physical[id] as f64 * ns_per_tick as f64 / 1e6);
        }
    }
    let mut result = json!({
        "url":url,"renderer":"WebGPU","frames":times.len(),
        "frame_times_ms":physical.iter().map(|t|*t as f64*ns_per_tick as f64/1e6).collect::<Vec<_>>(),
        "body_poses_and_shapes_checked":4*times.len(),"coordinate_tolerance":1e-12,"source_trajectories":2,
        "reset_ms":chrome.oracle.last,"captured_ms":clocks,"indicator_positions_checked":4*times.len()
    });
    if authored {
        let expected = chrome
            .oracle
            .trace
            .as_array()
            .ok_or("recorded scenes")?
            .iter()
            .map(|v| {
                v["scene"]["visuals"]
                    .as_array()
                    .map(Vec::len)
                    .ok_or("visual count")
            })
            .collect::<std::result::Result<Vec<_>, _>>()?;
        result["source_trajectories"] = json!(1);
        result["body_poses_and_shapes_checked"] = json!(expected.iter().sum::<usize>());
        result["lifecycle_catalogs_checked"] = json!(expected.len());
        if chrome.oracle.index["gpu_arm_frames"].is_string() {
            result["independent_gpu_arm_states_checked"] = json!(9 * expected.len());
            if chrome.oracle.index["gpu_tool_contact"] == true {
                result["gpu_tool_audit"] = chrome.oracle.trace[0]["gpu_tool_audit"].clone();
            }
        }
        if let Some(names) = chrome.oracle.index["particle_visuals"].as_array() {
            result["independent_particle_positions_checked"] = json!(names.len() * expected.len());
        }
        if chrome.oracle.index["aligned_visual_links"].is_object() || chrome.oracle.index["aligned_journal_trace"].is_string() {
            result["independent_link_poses_checked"] = json!(expected.iter().sum::<usize>());
        }
        if chrome.oracle.index["source_visual_poses"].is_object() {
            result["independent_visual_poses_checked"] = json!(expected.iter().sum::<usize>());
        }
        result["indicator_positions_checked"] = json!(0);
        if let Some(bins) = compliance {
            result["source_trajectories"] = json!(3);
            result["material_pressure_bins_checked"] = json!(3 * bins * times.len() as u64);
            result["material_path_gauges_checked"] = json!(3 * times.len());
            result["indicator_positions_checked"] = json!(3 * (1 + bins) * times.len() as u64);
        }
        if let Some((links,impulses)) = weld_readings {
            let branch=chrome.oracle.index["welded_impact_branch"]==true;
            result["source_trajectories"] = json!(if branch{3}else{4});
            if branch{result["lateral_travel_gauges_checked"]=json!(impulses);}
            result["native_link_visuals_checked"] = json!(links);
            result["ground_contact_gauges_checked"] = json!(impulses);
        }
        if let Some((poses,work)) = turntable_readings {
            result["source_trajectories"] = json!(3);
            result["physics_rotations_checked"] = json!(poses);
            result["turntable_work_gauges_checked"] = json!(work);
        }
        if let Some(count) = seam_readings {
            result["source_trajectories"] = json!(2);
            result["panel_load_cursors_checked"] = json!(count);
        }
        if let Some((bins, gauges)) = corner_readings {
            result["source_trajectories"] = json!(3);
            result["material_pressure_bins_checked"] = json!(bins);
            result["material_path_gauges_checked"] = json!(gauges);
            result["indicator_positions_checked"] = json!(bins+gauges);
        }
        if let Some((forces, centers, edges, energies)) = surface_readings {
            result["source_trajectories"] = json!(3);
            result["normal_force_gauges_checked"] = json!(forces);
            result["centers_of_pressure_checked"] = json!(centers);
            result["loaded_patch_segments_checked"] = json!(edges);
            if energies>0 {result["normal_dissipation_gauges_checked"]=json!(energies);}
            result["indicator_positions_checked"] = json!(forces+centers+edges+energies);
        }
        result["reset_ms"] = json!(physical[0] as f64 / 1e6);
    } else if automatic {
        result["source_trajectories"] = json!(3);
        result["body_poses_and_shapes_checked"] = json!(6 * times.len());
        result["indicator_positions_checked"] = json!(6 * times.len());
        result["pad_presence_observations"] = json!(6 * times.len());
        result["pause_flags_checked"] = json!(3 * times.len());
        result["captured_ordinals"] = json!(
            chrome.oracle.index["captures"]
                .as_object()
                .unwrap()
                .iter()
                .filter(|(label, _)| promotion::LABELS.contains(&label.as_str()))
                .map(|(k, v)| (k, v.as_u64().unwrap() + 1))
                .collect::<BTreeMap<_, _>>()
        );
    } else if paired_native {
        result["source_cut_us"] = chrome.oracle.index["origin_us"].clone();
        if chrome.oracle.index["source_public_dt_ns"].is_u64() {
            result["source_public_dt_ns"] = chrome.oracle.index["source_public_dt_ns"].clone();
        }
        if chrome.oracle.index["native_samples"] == true || chrome.oracle.index["interval_samples"] == true {
            result["reaction_interval"] = json!(if chrome.oracle.index["native_samples"]==true {
                "each sampled native solve"
            } else { "each supplied integration interval" });
        }
        if chrome.oracle.index["load_face"].is_string() {
            result["material_face"] = chrome.oracle.index["load_face"].clone();
            result["material_corners_and_load_markers_checked"] = json!(4*times.len()*chrome.oracle.index["lanes"].as_array().map_or(0,Vec::len));
        }
        if chrome.oracle.index["origin_ns"].is_u64() {
            result["source_cut_ns"] = chrome.oracle.index["origin_ns"].clone();
        }
        result["body_poses_and_shapes_checked"] = json!(6 * times.len());
        result["corner_loads_checked"] = json!(8 * times.len());
        if has_reference_gauges {
            let n = chrome.oracle.index["lanes"]
                .as_array()
                .ok_or("missing lanes")?
                .len();
            result["source_trajectories"] = json!(n);
            let bodies_per_lane = if chrome.oracle.index["single_body"] == true {
                result["offset_com_gauge_roundoff"] = json!(
                    "For nonzero inertial COM offsets, the gauge axis also allows 32 f64 epsilons times (world COM coordinate magnitude plus offset L1 norm) times display gain."
                );
                2
            } else {
                3
            };
            let bodies_per_lane =
                bodies_per_lane + usize::from(chrome.oracle.index["has_opponent"] == true);
            result["body_poses_and_shapes_checked"] = json!(bodies_per_lane * n * times.len());
            let corners = if chrome.oracle.index["pair_totals_only"] == true {
                0
            } else {
                4 * n * times.len()
            };
            result["corner_loads_checked"] = json!(corners);
            result["pair_impulses_checked"] = json!(n * times.len());
            if chrome.oracle.index["support_impulses"] == true { result["individual_support_impulses_checked"] = json!(2*n*times.len()); }
            result["com_distances_checked"] = json!(n * times.len());
            result["native_step_counts_checked"] = json!(n * times.len());
            if let Some(n) = chrome.oracle.index["matched_step_count"].as_u64() {
                result["matched_impulse_error_and_wall_p95_readings_checked"] = json!(2*n);
            }
            if let Some(parts) = chrome.oracle.index["contact_interval_bars"].as_u64() {
                result["solver_interval_bars_checked"] = json!(parts);
            }
            if let Some(hits) = chrome.oracle.index["interval_impact_count"].as_u64() {
                result["interval_impact_points_and_vectors_checked"] = json!(hits);
            }
            if native {
                result["spin_differences_checked"] = json!(n * times.len());
            } else if weak {
                result["weak_trigger_markers_checked"] = json!(n * times.len());
            } else {
                result["matched_cut_com_distances_checked"] = json!(n * times.len());
            }
            result["indicator_positions_checked"] = json!((4 + 2 * usize::from(chrome.oracle.index["support_impulses"] == true)) * n * times.len() + corners);
        } else if flight {
            result["signed_height_error_readouts_checked"] = json!(2 * times.len());
            result["normal_impulse_gauges_checked"] = json!(2 * times.len());
            result["first_contact_markers_checked"] = json!(2 * times.len());
            result["indicator_positions_checked"] = json!(14 * times.len());
        } else if budget {
            result["pair_impulses_checked"] = json!(2 * times.len());
            result["native_step_counts_checked"] = json!(2 * times.len());
            result["solver_iteration_counts_checked"] = json!(2 * times.len());
            result["indicator_positions_checked"] = json!(14 * times.len());
        } else if rebound {
            result["pair_impulses_checked"] = json!(2 * times.len());
            result["supported_clearances_checked"] = json!(2 * times.len());
            result["solver_activity_gauges_checked"] = json!(2 * times.len());
            result["indicator_positions_checked"] = json!(14 * times.len());
        } else if energy {
            result["energy_gauges_checked"] = json!(4 * times.len());
            result["indicator_positions_checked"] = json!(12 * times.len());
            if policy_activity {
                result["solver_activity_gauges_checked"] = json!(2 * times.len());
                result["indicator_positions_checked"] = json!(14 * times.len());
            }
        } else {
            result["five_ms_integrals_checked"] = json!(2 * times.len());
            result["indicator_positions_checked"] = json!(10 * times.len());
        }
    } else if landing_audit {
        result["body_poses_and_shapes_checked"] = json!(6 * times.len());
        result["corner_loads_checked"] = json!(8 * times.len());
        result["measured_tier_markers_checked"] = json!(2 * times.len());
        result["indicator_positions_checked"] = json!(10 * times.len());
    } else if reacquisition {
        result["body_poses_and_shapes_checked"] = json!(6 * times.len());
        result["measured_landing_markers_checked"] = json!(2 * times.len());
        result["measured_tier_markers_checked"] = json!(2 * times.len());
    } else if multi {
        if chrome.oracle.support_catalogs != BTreeSet::from([false, true]) {
            return Err(format!(
                "missing support lifecycle states: {:?}",
                chrome.oracle.support_catalogs
            )
            .into());
        }
        result["support_catalog_states_checked"] = json!(["present", "retired"]);
        result["support_presence_checks"] = json!(4 * times.len());
    } else {
        result["ledge_edges_checked"] = json!(2 * times.len());
        result["toe_witnesses_checked"] = json!(2 * times.len());
    }
    fs::write(
        output.join("browser.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let source = PathBuf::from(args.next().ok_or("measurements directory")?);
    let index = PathBuf::from(args.next().ok_or("replay frame index")?);
    let output = PathBuf::from(args.next().ok_or("capture directory")?);
    let port = args.next().ok_or("replay port")?.parse()?;
    fs::create_dir_all(&output)?;
    let profile = output
        .canonicalize()?
        .join(format!(".browser-profile-{}", std::process::id()));
    fs::create_dir(&profile)?;
    let executable =
        std::env::var_os("GZWEB_CHROME_BIN").unwrap_or_else(|| "/opt/google/chrome/chrome".into());
    let mut chrome = Command::new(executable)
        .args([
            "--window-position=-4500,-4500",
            "--disable-backgrounding-occluded-windows",
            "--disable-renderer-backgrounding",
            "--no-sandbox",
            "--disable-dev-shm-usage",
            "--ignore-gpu-blocklist",
            "--ozone-platform=x11",
            "--enable-features=Vulkan",
            "--use-gl=angle",
            "--use-angle=gl",
            "--disable-vulkan-surface",
            "--enable-unsafe-webgpu",
            "--disable-background-networking",
            "--disable-component-update",
            "--no-first-run",
            "--disable-default-apps",
            "--window-size=1440,900",
            "--remote-debugging-port=0",
        ])
        .arg(format!("--user-data-dir={}", profile.display()))
        .arg("about:blank")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()?;
    let line_start = args.next().map(PathBuf::from);
    let outcome = run(&profile, &source, &index, &output, port, line_start.as_deref());
    let _ = chrome.kill();
    let _ = chrome.wait();
    sleep(450);
    for _ in 0..4 {
        if !profile.exists() {
            break;
        }
        let _ = fs::remove_dir_all(&profile);
        sleep(100);
    }
    outcome
}
