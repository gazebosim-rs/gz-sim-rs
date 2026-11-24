// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, check};
use serde_json::{Value, json};
use std::{
    net::{Ipv4Addr, SocketAddr, TcpStream},
    time::Duration,
};
use tungstenite::{Message, WebSocket};

pub struct Connection {
    socket: WebSocket<TcpStream>,
    pub fence: Option<Value>,
    pub restored: Option<Value>,
    pub scene: Value,
    id: u64,
}

impl Connection {
    pub fn open(port: u16) -> Result<Self> {
        let addr = SocketAddr::from((Ipv4Addr::LOCALHOST, port));
        let tcp = TcpStream::connect_timeout(&addr, Duration::from_secs(1))?;
        tcp.set_read_timeout(Some(Duration::from_secs(4)))?;
        tcp.set_write_timeout(Some(Duration::from_secs(2)))?;
        tcp.set_nodelay(true)?;
        let (mut socket, _) =
            tungstenite::client(format!("ws://{addr}/?operator=1&recovery=1"), tcp)?;
        let (mut fence, mut restored) = (None, None);
        let scene = loop {
            let value = read(&mut socket)?;
            match value["kind"].as_str() {
                Some("operator_fence") => fence = Some(value),
                Some("operator_restored") => restored = Some(value),
                Some("scene") => break value,
                _ => (),
            }
        };
        Ok(Self {
            socket,
            fence,
            restored,
            scene,
            id: 0,
        })
    }

    pub fn offer(&self, tick: u64, eligible: bool) -> Result<&str> {
        let offer = self.fence.as_ref().ok_or("missing parked offer")?;
        check(
            offer["epoch"] == 0 && offer["tick"] == tick && offer["eligible"] == eligible,
            format!("wrong parked offer at {tick}: {offer}"),
        )?;
        offer["token"]
            .as_str()
            .ok_or_else(|| "missing offer token".into())
    }

    pub fn recover(&mut self, tick: u64, token: &str, accepted: bool) -> Result<Value> {
        let ack =
            self.request(json!({"kind":"recover_workers","token":token,"epoch":0,"tick":tick}))?;
        check(
            ack["kind"] == "operator_reply" && ack["accepted"] == accepted,
            format!("unexpected admission {ack}"),
        )?;
        Ok(ack)
    }

    pub fn step(&mut self, tick: u64, accepted: bool) -> Result<Value> {
        let ack = self.request(json!({"kind":"operator_step","epoch":0,"tick":tick}))?;
        check(
            ack["kind"] == "ack"
                && ack["ok"] == accepted
                && ack["paused"] == true
                && clock(&ack)? == tick + 1,
            format!("unexpected step {ack}"),
        )?;
        Ok(ack)
    }

    pub fn play(&mut self) -> Result<()> {
        let ack = self.request(json!({"kind":"control","action":"resume"}))?;
        check(
            ack["ok"] == true && ack["paused"] == false,
            format!("unexpected Play {ack}"),
        )
    }

    pub fn request(&mut self, mut value: Value) -> Result<Value> {
        self.id += 1;
        value["version"] = 1.into();
        value["request_id"] = self.id.into();
        self.socket.send(Message::Text(value.to_string().into()))?;
        loop {
            let v = read(&mut self.socket)?;
            if matches!(v["kind"].as_str(), Some("operator_reply" | "ack"))
                && v["request_id"] == self.id
            {
                return Ok(v);
            }
        }
    }
}
fn read(socket: &mut WebSocket<TcpStream>) -> Result<Value> {
    loop {
        match socket.read()? {
            Message::Text(s) => return Ok(serde_json::from_str(&s)?),
            Message::Ping(_) | Message::Pong(_) => socket.flush()?,
            _ => return Err("socket closed before required frame".into()),
        }
    }
}
pub fn clock(v: &Value) -> Result<u64> {
    let ns = v["nanoseconds"].as_u64().ok_or("missing nanoseconds")?;
    check(
        v["epoch"] == 0 && ns % 1_000_000 == 0,
        "expected epoch zero and millisecond clock",
    )?;
    Ok(v["seconds"].as_u64().ok_or("missing seconds")? * 1000 + ns / 1_000_000)
}
