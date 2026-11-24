// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exercise authenticated worker sessions through a silent, directional stream cut.
//! Data and EOF in a masked direction stop reaching the destination. Dropped
//! application bytes cannot be recovered by restoring the relay. Real TCP
//! retransmission, congestion control and packet timing are outside this fixture.
use clap::Parser;
use serde::Serialize;
use serde_json::json;
use std::{
    fs::{self, File},
    io::{self, Read, Write},
    net::{Shutdown, SocketAddr, TcpListener, TcpStream},
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering::SeqCst},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

#[derive(Parser)]
struct Args {
    /// Loopback socket for the dedicated test SSH client. Port zero selects a free port.
    #[arg(long)]
    listen: SocketAddr,
    #[arg(long)]
    upstream: SocketAddr,
    /// Empty existing directory. Atomically replace mode, or create stop to finish.
    #[arg(long)]
    control_dir: PathBuf,
}

const INTERVAL: Duration = Duration::from_millis(10);
const UP: u8 = 1;
const DOWN: u8 = 2;

fn mode(dir: &Path) -> io::Result<(&'static str, u8)> {
    let file = match File::open(dir.join("mode")) {
        Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(("pass", 0)),
        result => result?,
    };
    let mut s = String::new();
    file.take(17).read_to_string(&mut s)?;
    match s.trim() {
        "pass" if s.len() <= 16 => Ok(("pass", 0)),
        "drop-up" if s.len() <= 16 => Ok(("drop-up", UP)),
        "drop-down" if s.len() <= 16 => Ok(("drop-down", DOWN)),
        "drop-both" if s.len() <= 16 => Ok(("drop-both", UP | DOWN)),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid relay mode",
        )),
    }
}

fn publish(dir: &Path, name: &str, value: &impl Serialize) -> io::Result<()> {
    let next = dir.join(format!("{name}.next"));
    fs::write(&next, serde_json::to_vec(value)?)?;
    fs::rename(next, dir.join(name))
}

#[derive(Default, Serialize)]
struct Traffic {
    forwarded_bytes: AtomicU64,
    discarded_bytes: AtomicU64,
    source_ends: AtomicU64,
    forwarded_ends: AtomicU64,
    write_errors: AtomicU64,
}

#[derive(Default)]
struct Control {
    mask: AtomicU8,
    stop: AtomicBool,
    up: Traffic,
    down: Traffic,
}

#[derive(Default)]
struct End {
    seen: AtomicBool,
    forwarded: AtomicBool,
}

#[derive(Default)]
struct Ends {
    up: End,
    down: End,
}

fn pump(
    mut source: &TcpStream,
    mut dest: &TcpStream,
    control: &Control,
    direction: u8,
    ends: &Ends,
) {
    let (traffic, end) = if direction == UP {
        (&control.up, &ends.up)
    } else {
        (&control.down, &ends.down)
    };
    let mut buffer = [0; 16384];
    let mut dest_open = true;
    while !control.stop.load(SeqCst) {
        let n = match source.read(&mut buffer) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(_) => break,
        };
        let mut pending = &buffer[..n];
        while !pending.is_empty() {
            if !dest_open || control.mask.load(SeqCst) & direction != 0 {
                traffic
                    .discarded_bytes
                    .fetch_add(pending.len() as u64, SeqCst);
                break;
            }
            match dest.write(pending) {
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Ok(0) | Err(_) => {
                    // A dead sink cannot cause an artificial FIN in the other
                    // direction. Continue draining its online peer independently.
                    dest_open = false;
                    traffic.write_errors.fetch_add(1, SeqCst);
                }
                Ok(n) => {
                    traffic.forwarded_bytes.fetch_add(n as u64, SeqCst);
                    pending = &pending[n..];
                }
            }
        }
    }
    if !control.stop.load(SeqCst) {
        end.seen.store(true, SeqCst);
        traffic.source_ends.fetch_add(1, SeqCst);
        // A client can close after its own SSH timeout. A partition must
        // mask that EOF just as it masked the client's last request.
        while control.mask.load(SeqCst) & direction != 0 && !control.stop.load(SeqCst) {
            thread::sleep(INTERVAL);
        }
        if !control.stop.load(SeqCst) && dest.shutdown(Shutdown::Write).is_ok() {
            traffic.forwarded_ends.fetch_add(1, SeqCst);
            end.forwarded.store(true, SeqCst);
        }
    }
}

struct Pair {
    id: u64,
    client: Arc<TcpStream>,
    server: Arc<TcpStream>,
    ends: Arc<Ends>,
    threads: [JoinHandle<()>; 2],
}

impl Pair {
    fn start(
        id: u64,
        client: TcpStream,
        server: TcpStream,
        control: &Arc<Control>,
    ) -> io::Result<Self> {
        client.set_nodelay(true)?;
        server.set_nodelay(true)?;
        let client = Arc::new(client);
        let server = Arc::new(server);
        let ends = Arc::new(Ends::default());
        let spawn = |source: Arc<TcpStream>, dest: Arc<TcpStream>, direction| {
            let control = Arc::clone(control);
            let ends = Arc::clone(&ends);
            thread::spawn(move || pump(&source, &dest, &control, direction, &ends))
        };
        let threads = [
            spawn(Arc::clone(&client), Arc::clone(&server), UP),
            spawn(Arc::clone(&server), Arc::clone(&client), DOWN),
        ];
        Ok(Self {
            id,
            client,
            server,
            ends,
            threads,
        })
    }

    fn snapshot(&self) -> serde_json::Value {
        json!({
            "id": self.id,
            "up_ended": self.ends.up.seen.load(SeqCst),
            "down_ended": self.ends.down.seen.load(SeqCst),
            "up_end_forwarded": self.ends.up.forwarded.load(SeqCst),
            "down_end_forwarded": self.ends.down.forwarded.load(SeqCst),
        })
    }

    fn join(self) -> io::Result<()> {
        for worker in self.threads {
            worker
                .join()
                .map_err(|_| io::Error::other("relay pump panicked"))?;
        }
        Ok(())
    }
}

fn run(args: Args) -> io::Result<()> {
    if !args.listen.ip().is_loopback() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "listen requires loopback",
        ));
    }
    let dir = args.control_dir;
    if fs::read_dir(&dir)?.next().is_some() {
        return Err(io::Error::new(
            io::ErrorKind::AlreadyExists,
            "control directory must be empty",
        ));
    }
    let listener = TcpListener::bind(args.listen)?;
    listener.set_nonblocking(true)?;
    let control = Arc::new(Control::default());
    let mut pairs: Vec<Pair> = Vec::new();
    let mut accepted = 0u64;
    let mut closed = 0u64;
    let mut connect_errors = 0u64;
    let mut rejected = 0u64;
    let mut previous = serde_json::Value::Null;
    publish(
        &dir,
        "ready.json",
        &json!({"listen":listener.local_addr()?, "upstream":args.upstream}),
    )?;
    loop {
        let (label, mask) = mode(&dir)?;
        control.mask.store(mask, SeqCst);
        let stopped = dir.join("stop").try_exists()?;
        if stopped {
            control.stop.store(true, SeqCst);
            for p in &pairs {
                let _ = p.client.shutdown(Shutdown::Both);
                let _ = p.server.shutdown(Shutdown::Both);
            }
        }
        let mut i = 0;
        while i < pairs.len() {
            if stopped || pairs[i].threads.iter().all(JoinHandle::is_finished) {
                pairs.swap_remove(i).join()?;
                closed += 1;
            } else {
                i += 1;
            }
        }
        if !stopped {
            match listener.accept() {
                Ok((_, _)) if pairs.len() == 32 => rejected += 1,
                Ok((client, _)) => {
                    match TcpStream::connect_timeout(&args.upstream, Duration::from_secs(1)) {
                        Ok(server) => {
                            accepted += 1;
                            pairs.push(Pair::start(accepted, client, server, &control)?);
                        }
                        Err(_) => connect_errors += 1,
                    }
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
                    ) => {}
                Err(e) => return Err(e),
            }
        }
        let state = json!({
            "mode":label, "stopped":stopped, "accepted":accepted,
            "closed":closed, "active":pairs.len(), "connect_errors":connect_errors,
            "capacity_rejections":rejected, "up":control.up, "down":control.down,
            "connections":pairs.iter().map(Pair::snapshot).collect::<Vec<_>>(),
        });
        if state != previous {
            publish(&dir, "state.json", &state)?;
            previous = state;
        }
        if stopped {
            return Ok(());
        }
        thread::sleep(INTERVAL);
    }
}

fn main() -> io::Result<()> {
    run(Args::parse())
}

#[cfg(test)]
#[path = "convoy_link_relay/tests.rs"]
mod tests;
