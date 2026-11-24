// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise the public gzweb lifecycle API of a running distributed stack example.
use clap::Parser;
use serde_json::{Value, json};
use std::{
    error::Error,
    net::{Ipv4Addr, TcpStream},
    time::Duration,
};
use tungstenite::{Message, WebSocket};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    /// Port of a freshly started distributed_stack.sdf server with --web-lifecycle.
    #[arg(long, default_value_t = 9071)]
    port: u16,
    /// Spawn, edit, retire and reset free cargo in a four-worker distributed_four_lanes.sdf world.
    #[arg(long)]
    four_lanes: bool,
}
struct Viewer {
    socket: WebSocket<TcpStream>,
    scene: Value,
    catalog: Value,
    request: u64,
}
impl Viewer {
    fn read(&mut self) -> Result<Value> {
        loop {
            match self.socket.read()? {
                Message::Text(s) => {
                    let v: Value = serde_json::from_str(&s)?;
                    match v["kind"].as_str() {
                        Some("scene") => self.scene = v.clone(),
                        Some("lifecycle_catalog") => self.catalog = v.clone(),
                        _ => {}
                    }
                    return Ok(v);
                }
                Message::Ping(_) | Message::Pong(_) => {}
                other => return Err(format!("unexpected message {other}").into()),
            }
        }
    }
    fn command(&mut self, action: &str, mut fields: Value, edit: bool) -> Result<Value> {
        self.request += 1;
        fields["kind"] = json!(if edit { "lifecycle" } else { "control" });
        fields["action"] = json!(action);
        fields["version"] = json!(1);
        fields["request_id"] = json!(self.request);
        if edit {
            fields["epoch"] = self.scene["epoch"].clone();
            fields["world_id"] = self.scene["world_id"].clone();
        }
        self.socket.send(Message::Text(fields.to_string().into()))?;
        loop {
            let v = self.read()?;
            if matches!(v["kind"].as_str(), Some("ack" | "lifecycle_ack"))
                && v["request_id"] == self.request
            {
                println!("{v}");
                if v["ok"] != true {
                    return Err("server refused operation".into());
                }
                return Ok(v);
            }
        }
    }
    fn run_to(&mut self, tick: u64) -> Result<()> {
        self.command("resume", json!({}), false)?;
        loop {
            let v = self.read()?;
            if v["kind"] == "scene"
                && v["seconds"].as_u64().ok_or("missing seconds")? * 100
                    + v["nanoseconds"].as_u64().ok_or("missing nanoseconds")? / 10_000_000
                    >= tick
            {
                break;
            }
        }
        self.command("pause", json!({}), false)?;
        Ok(())
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    let port = args.port;
    let stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port))?;
    stream.set_read_timeout(Some(Duration::from_secs(15)))?;
    let (socket, _) = tungstenite::client(
        format!("ws://127.0.0.1:{port}/?lifecycle=1").as_str(),
        stream,
    )?;
    let mut v = Viewer {
        socket,
        scene: Value::Null,
        catalog: Value::Null,
        request: 0,
    };
    while v.catalog.is_null() {
        v.read()?;
    }
    if v.scene["paused"] != true || v.scene["seconds"] != 0 || v.scene["nanoseconds"] != 0 {
        return Err("start the stack paused at time zero".into());
    }
    if args.four_lanes {
        exercise_pool(&mut v)?;
        v.socket.close(None)?;
        return Ok(());
    }
    let template = v.catalog["templates"]
        .as_array()
        .ok_or("missing startup templates")?
        .iter()
        .find(|t| t["label"] == "first")
        .ok_or("missing one-kilogram box template")?["template_id"]
        .clone();
    v.run_to(158)?;
    let added = v.command(
        "spawn",
        json!({"template_id":template,"translation":[0.,1.25,2.5],"rotation":[0.,0.,0.,1.]}),
        true,
    )?;
    let id = &added["model_id"];
    v.command(
        "set_state",
        json!({"model_id":id,"translation":[0.,1.22,1.75],"rotation":[0.,0.,0.,1.],
        "linear_velocity":[0.,-0.1,-0.6],"angular_velocity":[0.5,0.,0.]}),
        true,
    )?;
    v.run_to(230)?;
    v.command("get_state", json!({"model_id":id}), true)?;
    v.command(
        "set_state",
        json!({"model_id":id,"translation":[0.,-1.2,2.],"rotation":[0.,0.,0.,1.],
        "linear_velocity":[0.,0.,0.],"angular_velocity":[0.,0.,0.]}),
        true,
    )?;
    v.run_to(248)?;
    v.command("remove", json!({"model_id":id}), true)?;
    v.run_to(265)?;
    let reset = v.command("reset", json!({}), false)?;
    while v.scene["epoch"] != reset["epoch"] {
        v.read()?;
    }
    if v.scene["epoch"] != 1 || v.scene["visuals"].as_array().is_none_or(|s| s.len() != 3) {
        return Err("loaded reset did not restore the three startup visuals".into());
    }
    v.socket.close(None)?;
    Ok(())
}

fn current_tuple(v: &mut Viewer, epoch: u64, min_sequence: u64) -> Result<()> {
    while v.scene["epoch"] != epoch
        || v.catalog["epoch"] != epoch
        || v.catalog["scene_sequence"] != v.scene["sequence"]
        || v.scene["sequence"].as_u64().unwrap_or(0) < min_sequence
    {
        v.read()?;
    }
    Ok(())
}
fn pool_model(v: &Viewer, id: u64) -> Option<&Value> {
    v.catalog["models"]
        .as_array()?
        .iter()
        .find(|model| model["model_id"] == id)
}
fn pool_visual(v: &Viewer, id: u64) -> Option<&Value> {
    v.scene["visuals"]
        .as_array()?
        .iter()
        .find(|visual| visual["id"] == id)
}
fn spawn_cargo(
    v: &mut Viewer,
    template: u64,
    position: [f64; 3],
    shape: &Value,
) -> Result<(u64, u64)> {
    let ack = v.command(
        "spawn",
        json!({"template_id":template,"translation":position,"rotation":[0.,0.,0.,1.]}),
        true,
    )?;
    current_tuple(
        v,
        ack["epoch"].as_u64().ok_or("missing spawn epoch")?,
        ack["scene_sequence"]
            .as_u64()
            .ok_or("missing spawn barrier")?,
    )?;
    let id = ack["model_id"].as_u64().ok_or("missing spawn ID")?;
    let m = pool_model(v, id).ok_or("spawn omitted model")?;
    if m["can_remove"] != true || m["can_set_state"] != true {
        return Err("spawned cargo has incorrect capabilities".into());
    }
    let visual = m["visual_ids"][0].as_u64().ok_or("missing clone visual")?;
    let paint = pool_visual(v, visual).ok_or("spawn omitted visual")?;
    if paint["shape"] != *shape || paint["translation"] != json!(position) {
        return Err("spawn changed cargo geometry or pose".into());
    }
    Ok((id, visual))
}
fn remove_cargo(v: &mut Viewer, id: u64, visual: u64) -> Result<()> {
    let ack = v.command("remove", json!({"model_id":id}), true)?;
    current_tuple(
        v,
        ack["epoch"].as_u64().ok_or("missing remove epoch")?,
        ack["scene_sequence"]
            .as_u64()
            .ok_or("missing remove barrier")?,
    )?;
    if pool_model(v, id).is_some() || pool_visual(v, visual).is_some() {
        return Err("removed model or visual remains published".into());
    }
    Ok(())
}
fn exercise_pool(v: &mut Viewer) -> Result<()> {
    let startup_models = v.catalog["models"].clone();
    let startup_visuals = v.scene["visuals"].clone();
    let template = v.catalog["templates"]
        .as_array()
        .ok_or("missing startup templates")?
        .iter()
        .find(|t| t["label"] == "cargo_0")
        .and_then(|t| t["template_id"].as_u64())
        .ok_or("cargo_0 template unavailable")?;
    let cargo = startup_models
        .as_array()
        .unwrap()
        .iter()
        .find(|m| {
            m["name"] == "default::cargo_0" && m["can_set_state"] == true && m["can_remove"] == true
        })
        .ok_or("cargo_0 population editing unavailable")?;
    let cargo_id = cargo["model_id"].as_u64().ok_or("missing cargo model ID")?;
    let cargo_visual = cargo["visual_ids"][0]
        .as_u64()
        .ok_or("missing cargo visual ID")?;
    let shape =
        pool_visual(v, cargo_visual).ok_or("missing startup cargo geometry")?["shape"].clone();
    let original = v.command("get_state", json!({"model_id":cargo_id}), true)?["state"].clone();
    v.run_to(12)?;
    let (first, first_visual) = spawn_cargo(v, template, [-3., 5., 1.8], &shape)?;
    v.run_to(35)?;
    let changed=v.command("set_state",json!({"model_id":first,"translation":[-2.5,5.,2.],"rotation":[0.,0.,0.,1.],"linear_velocity":[1.4,0.,0.3],"angular_velocity":[0.,0.4,0.]}),true)?;
    let at = |row: &Value| -> Result<f64> {
        Ok(row["seconds"].as_f64().ok_or("missing seconds")?
            + row["nanoseconds"].as_f64().ok_or("missing nanoseconds")? * 1e-9)
    };
    v.run_to(80)?;
    let advanced = v.command("get_state", json!({"model_id":first}), true)?;
    let dt = at(&advanced)? - at(&changed)?;
    let pose: [f64; 3] = serde_json::from_value(advanced["state"]["translation"].clone())?;
    if dt <= 0.
        || (pose[0] - (-2.5 + 1.4 * dt)).abs() > 1e-10
        || (pose[2] - (2. + 0.3 * dt)).abs() > 1e-10
        || pool_model(v, first).is_none_or(|m| m["visual_ids"][0] != first_visual)
    {
        return Err("spawned cargo did not advance under its stable visual ID".into());
    }
    remove_cargo(v, first, first_visual)?;
    v.run_to(120)?;
    let (second, second_visual) = spawn_cargo(v, template, [2., 5., 2.2], &shape)?;
    if first == second || first_visual == second_visual {
        return Err("retired clone identity was reused".into());
    }
    v.run_to(155)?;
    remove_cargo(v, cargo_id, cargo_visual)?;
    v.run_to(175)?;
    let reset = v.command("reset", json!({}), false)?;
    let epoch = reset["epoch"].as_u64().ok_or("missing reset epoch")?;
    current_tuple(v, epoch, 0)?;
    let restored = v.command("get_state", json!({"model_id":cargo_id}), true)?;
    if epoch != 1
        || v.catalog["models"] != startup_models
        || v.scene["visuals"] != startup_visuals
        || restored["state"] != original
        || at(&restored)? != 0.
        || pool_model(v, second).is_some()
    {
        return Err(
            "reset did not restore startup identities, poses, capabilities and clock".into(),
        );
    }
    println!(
        "Verified four-worker pool cargo {cargo_id}/{cargo_visual}, clone {first}/{first_visual}, replacement clone {second}/{second_visual}, retired startup cargo and restored epoch {epoch} over the public WebSocket."
    );
    Ok(())
}
