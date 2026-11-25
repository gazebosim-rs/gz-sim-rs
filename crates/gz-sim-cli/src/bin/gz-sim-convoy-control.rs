// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared WebSocket operator for the local and legacy convoy command names.
use clap::{Parser, Subcommand};
use serde_json::{Value, json};
use std::{
    error::Error,
    io,
    net::{Ipv4Addr, SocketAddrV4, TcpStream},
    time::{Duration, Instant},
};
use tungstenite::{Message, WebSocket};

#[derive(Parser)]
struct Args {
    /// Loopback WebSocket port of a running Gazebo viewer server.
    #[arg(long)]
    port: u16,
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Print the received scene and available fence diagnosis without changing the session.
    Status,
    /// Request a verified rebuild at this fence. The reply acknowledges admission.
    Recover {
        epoch: u64,
        tick: u64,
        token: String,
    },
    /// Advance exactly one millisecond, only if the server is paused at this exact cut.
    Step { epoch: u64, tick: u64 },
    /// Start the live drive at its current cut.
    Play,
    /// Pause the live drive at its next boundary.
    Pause,
    /// Restore the initial world and pause at the beginning of the new epoch.
    Reset,
}
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn frame(ws: &mut WebSocket<TcpStream>, deadline: Instant) -> Result<Value> {
    loop {
        let timeout = deadline
            .checked_duration_since(Instant::now())
            .filter(|t| !t.is_zero())
            .ok_or_else(|| io::Error::from(io::ErrorKind::TimedOut))?;
        ws.get_ref().set_read_timeout(Some(timeout))?;
        match ws.read()? {
            Message::Text(s) => return Ok(serde_json::from_str(&s)?),
            Message::Ping(_) | Message::Pong(_) => ws.flush()?,
            _ => return Err("connection closed before receipt".into()),
        }
    }
}
fn send_and_receive(
    ws: &mut WebSocket<TcpStream>,
    request: &Value,
    deadline: Instant,
) -> Result<Value> {
    ws.send(Message::Text(request.to_string().into()))?;
    loop {
        let message = frame(ws, deadline)?;
        if matches!(message["kind"].as_str(), Some("operator_reply" | "ack"))
            && message["request_id"] == 1
        {
            return Ok(message);
        }
    }
}

pub(crate) fn main() -> Result<()> {
    let a = Args::parse();
    let addr = SocketAddrV4::new(Ipv4Addr::LOCALHOST, a.port);
    let stream = TcpStream::connect_timeout(&addr.into(), Duration::from_secs(5))?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
    let (mut ws, _) = tungstenite::client(format!("ws://{addr}/?operator=1&recovery=1"), stream)?;
    let until = Instant::now() + Duration::from_secs(60);
    let mut fence = None;
    let mut handoff = None;
    let scene = loop {
        let m = frame(&mut ws, until)?;
        match m["kind"].as_str() {
            Some("operator_fence") => fence = Some(m),
            Some("operator_restored") => handoff = Some(m),
            Some("scene") => break m,
            _ => {}
        }
    };
    let request = match a.command {
        Command::Status => {
            println!(
                "{}",
                json!({"scene":scene,"operator_fence":fence,"operator_restored":handoff})
            );
            return Ok(());
        }
        Command::Recover { epoch, tick, token } => {
            let f = fence
                .as_ref()
                .ok_or("server has no parked operator fence")?;
            if f["eligible"] != true
                || f["epoch"] != epoch
                || f["tick"] != tick
                || f["token"] != token
            {
                return Err("the observed fence does not authorize this request".into());
            }
            json!({"version":1,"kind":"recover_workers","request_id":1,"epoch":epoch,"tick":tick,"token":token})
        }
        _ if fence.is_some() => {
            return Err("parked workers cannot advance or change transport mode".into());
        }
        Command::Step { epoch, tick } => {
            json!({"version":1,"kind":"operator_step","request_id":1,"epoch":epoch,"tick":tick})
        }
        Command::Play => json!({"version":1,"kind":"control","request_id":1,"action":"resume"}),
        Command::Pause => json!({"version":1,"kind":"control","request_id":1,"action":"pause"}),
        Command::Reset => json!({"version":1,"kind":"control","request_id":1,"action":"reset"}),
    };
    let reply = send_and_receive(&mut ws, &request, until).map_err(|error| {
        format!(
            "Command outcome unknown. Reconnect with gz-sim-control --port {} status before issuing another command. ({error})",
            a.port
        )
    })?;
    println!("{reply}");
    if reply["accepted"] == true || reply["ok"] == true {
        Ok(())
    } else {
        Err("operator command refused".into())
    }
}
