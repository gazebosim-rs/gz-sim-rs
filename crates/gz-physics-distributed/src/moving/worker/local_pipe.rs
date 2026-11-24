// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The coordinator owns the native pipe descriptors. Different disjoint solvers
//! can execute outstanding requests without a blocking bridge thread per slot.
use super::LIMIT;
use rustix::{
    event::{PollFd, PollFlags, Timespec, poll},
    fs::{OFlags, fcntl_getfl, fcntl_setfl},
};
use std::{
    io::{self, BufReader, IoSlice, Read, Write},
    os::fd::AsFd,
    process::{ChildStdin, ChildStdout},
    time::Instant,
};

pub(super) struct LocalPipe {
    input: ChildStdin,
    output: BufReader<ChildStdout>,
}
fn nonblocking(fd: &impl AsFd) -> io::Result<()> {
    fcntl_setfl(fd, fcntl_getfl(fd)? | OFlags::NONBLOCK)?;
    Ok(())
}
fn wait(fd: &impl AsFd, interest: PollFlags, deadline: Instant) -> io::Result<()> {
    loop {
        let remaining = deadline
            .checked_duration_since(Instant::now())
            .ok_or(io::ErrorKind::TimedOut)?;
        let timeout = Timespec::try_from(remaining).map_err(io::Error::other)?;
        match poll(&mut [PollFd::new(fd, interest)], Some(&timeout)) {
            Ok(0) => return Err(io::ErrorKind::TimedOut.into()),
            // A hangup or error also wakes poll. The next I/O reports EOF,
            // broken pipe, or the real underlying error without losing data.
            Ok(_) => return Ok(()),
            Err(rustix::io::Errno::INTR) => continue,
            Err(e) => return Err(e.into()),
        }
    }
}
fn send_frame(input: &mut (impl Write + AsFd), bytes: &[u8], deadline: Instant) -> io::Result<()> {
    if bytes.len() > LIMIT {
        return Err(io::Error::other("oversized moving worker frame"));
    }
    let length = (bytes.len() as u32).to_le_bytes();
    let mut parts = [IoSlice::new(&length), IoSlice::new(bytes)];
    let mut rest = &mut parts[..];
    while !rest.is_empty() {
        match input.write_vectored(rest) {
            Ok(0) => return Err(io::ErrorKind::WriteZero.into()),
            Ok(n) => IoSlice::advance_slices(&mut rest, n),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                wait(input, PollFlags::OUT, deadline)?
            }
            Err(e) => return Err(e),
        }
    }
    Ok(())
}
fn read_exact(
    input: &mut BufReader<impl Read + AsFd>,
    mut buf: &mut [u8],
    deadline: Instant,
) -> io::Result<()> {
    while !buf.is_empty() {
        match input.read(buf) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(n) => buf = &mut buf[n..],
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                wait(input.get_ref(), PollFlags::IN, deadline)?
            }
            Err(e) => return Err(e),
        }
    }
    Ok(())
}
fn receive_frame(
    output: &mut BufReader<impl Read + AsFd>,
    deadline: Instant,
) -> io::Result<Vec<u8>> {
    let mut head = [0; 4];
    read_exact(output, &mut head, deadline)?;
    let n = u32::from_le_bytes(head) as usize;
    if n > LIMIT {
        return Err(io::Error::other("oversized moving worker frame"));
    }
    let mut bytes = vec![0; n];
    read_exact(output, &mut bytes, deadline)?;
    Ok(bytes)
}
impl LocalPipe {
    pub(super) fn new(input: ChildStdin, output: ChildStdout) -> io::Result<Self> {
        nonblocking(&input)?;
        nonblocking(&output)?;
        Ok(Self {
            input,
            output: BufReader::new(output),
        })
    }
    pub(super) fn send(&mut self, bytes: &[u8], deadline: Instant) -> io::Result<()> {
        send_frame(&mut self.input, bytes, deadline)
    }
    pub(super) fn receive(&mut self, deadline: Instant) -> io::Result<Vec<u8>> {
        receive_frame(&mut self.output, deadline)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::{io::pipe, thread, time::Duration};
    #[test]
    fn partial_frames_time_out_and_closed_overlarge_and_empty_frames_are_distinct() {
        let (r, mut w) = pipe().unwrap();
        nonblocking(&r).unwrap();
        let mut reader = BufReader::new(r);
        w.write_all(&[4, 0, 0, 0, 1]).unwrap();
        assert_eq!(
            receive_frame(&mut reader, Instant::now() + Duration::from_millis(20))
                .unwrap_err()
                .kind(),
            io::ErrorKind::TimedOut
        );
        drop(w);
        // A concurrent fork can keep the CLOEXEC writer alive until exec.
        // Establish hangup before testing the zero-budget EOF read.
        wait(
            reader.get_ref(),
            PollFlags::HUP,
            Instant::now() + Duration::from_secs(2),
        )
        .unwrap();
        assert_eq!(
            receive_frame(&mut reader, Instant::now())
                .unwrap_err()
                .kind(),
            io::ErrorKind::UnexpectedEof
        );
        let (r, mut w) = pipe().unwrap();
        w.write_all(&[0; 4]).unwrap();
        w.write_all(&u32::MAX.to_le_bytes()).unwrap();
        let mut reader = BufReader::new(r);
        assert!(
            receive_frame(&mut reader, Instant::now())
                .unwrap()
                .is_empty()
        );
        assert!(receive_frame(&mut reader, Instant::now()).is_err());
    }
    #[test]
    fn larger_than_pipe_capacity_and_saturated_write_obey_one_deadline() {
        let (r, mut w) = pipe().unwrap();
        nonblocking(&r).unwrap();
        nonblocking(&w).unwrap();
        let bytes = vec![0x85; LIMIT];
        let read = thread::spawn(move || {
            receive_frame(
                &mut BufReader::new(r),
                Instant::now() + Duration::from_secs(2),
            )
        });
        send_frame(&mut w, &bytes, Instant::now() + Duration::from_secs(2)).unwrap();
        assert_eq!(read.join().unwrap().unwrap(), bytes);
        let (_r, mut w) = pipe().unwrap();
        nonblocking(&w).unwrap();
        while matches!(w.write(&[0;4096]),Ok(n) if n>0) {}
        assert_eq!(
            send_frame(&mut w, &bytes, Instant::now() + Duration::from_millis(20))
                .unwrap_err()
                .kind(),
            io::ErrorKind::TimedOut
        );
    }
}
