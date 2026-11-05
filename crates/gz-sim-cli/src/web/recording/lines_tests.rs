// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde::Deserialize;
use std::io::Cursor;

#[derive(Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct Probe {
    ordinal: u64,
}

fn limits(line_bytes: usize, file_bytes: u64, lines: u32) -> Limits {
    Limits {
        line_bytes,
        file_bytes,
        lines,
    }
}

#[test]
fn exact_byte_and_record_boundaries_accept_complete_records_and_eof() {
    let first = encode(&Probe { ordinal: 1 }, 64).unwrap();
    let second = encode(&Probe { ordinal: 2 }, 64).unwrap();
    let mut input = first.clone();
    input.extend(&second);
    let mut reader = Lines::new(
        Cursor::new(input.clone()),
        limits(first.len(), input.len() as u64, 2),
    )
    .unwrap();
    assert_eq!(reader.offset(), 0);
    assert_eq!(
        reader.next_record::<Probe>().unwrap(),
        Some(Probe { ordinal: 1 })
    );
    assert_eq!(reader.offset(), first.len() as u64);
    assert_eq!(
        reader.next_record::<Probe>().unwrap(),
        Some(Probe { ordinal: 2 })
    );
    assert_eq!(reader.offset(), input.len() as u64);
    assert_eq!(reader.next_record::<Probe>().unwrap(), None);
    assert_eq!(reader.next_record::<Probe>().unwrap(), None);
    assert_eq!(encode(&Probe { ordinal: 1 }, first.len()).unwrap(), first);
    assert!(encode(&Probe { ordinal: 1 }, first.len() - 1).is_err());
}

#[test]
fn malformed_duplicate_unknown_overflow_and_unterminated_records_are_terminal() {
    for bad in [
        b"\n".as_slice(),
        b"{\"ordinal\":1,\"ordinal\":2}\n",
        b"{\"ordinal\":1,\"extra\":2}\n",
        b"{\"ordinal\":18446744073709551616}\n",
        b"{\"ordinal\":1}",
        b"{\"ordinal\":\xff}\n",
    ] {
        let mut input = bad.to_vec();
        // A following valid line must never rescue a previously refused stream.
        if bad.last() == Some(&b'\n') {
            input.extend(encode(&Probe { ordinal: 3 }, 64).unwrap());
        }
        let mut reader = Lines::new(Cursor::new(input), limits(64, 256, 8)).unwrap();
        assert!(reader.next_record::<Probe>().is_err());
        assert!(
            reader
                .next_record::<Probe>()
                .unwrap_err()
                .to_string()
                .contains("already refused")
        );
    }
}

#[test]
fn line_file_and_count_overflow_do_not_admit_partial_records() {
    let valid = encode(&Probe { ordinal: 1 }, 64).unwrap();
    let mut huge = Lines::new(Cursor::new(vec![b' '; 1024 * 1024]), limits(64, 1024, 8)).unwrap();
    assert!(
        huge.next_record::<Probe>()
            .unwrap_err()
            .to_string()
            .contains("line exceeds")
    );
    assert_eq!(huge.reader.position(), 65); // Bound plus one sentinel, not a MiB allocation/read.
    assert_eq!(huge.offset(), 0);
    let twice = [valid.clone(), valid.clone()].concat();
    for bound in [
        limits(64, 64, 1),
        limits(valid.len(), (twice.len() - 1) as u64, 2),
    ] {
        let mut reader = Lines::new(Cursor::new(twice.clone()), bound).unwrap();
        assert_eq!(
            reader.next_record::<Probe>().unwrap(),
            Some(Probe { ordinal: 1 })
        );
        assert!(reader.next_record::<Probe>().is_err());
        assert_eq!(reader.offset(), valid.len() as u64);
        assert!(reader.next_record::<Probe>().is_err());
    }
    assert!(encode(&"x".repeat(1024 * 1024), 64).is_err());
}

#[test]
fn invalid_configuration_and_real_io_failure_fail_closed() {
    for bound in [
        limits(0, 64, 1),
        limits(65, 64, 1),
        limits(LINE_BYTES + 1, FILE_BYTES, 1),
        limits(64, FILE_BYTES + 1, 1),
        limits(64, 64, 0),
        limits(64, 64, STREAM_LINES + 1),
    ] {
        assert!(Lines::new(Cursor::new(Vec::<u8>::new()), bound).is_err());
    }
    assert!(encode(&Probe { ordinal: 1 }, 0).is_err());
    assert!(encode(&Probe { ordinal: 1 }, LINE_BYTES + 1).is_err());
    struct Broken;
    impl Read for Broken {
        fn read(&mut self, _: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::other("fixture IO fault"))
        }
    }
    let mut reader = Lines::new(io::BufReader::new(Broken), limits(64, 64, 1)).unwrap();
    assert!(
        reader
            .next_record::<Probe>()
            .unwrap_err()
            .to_string()
            .contains("fixture IO fault")
    );
    assert!(
        reader
            .next_record::<Probe>()
            .unwrap_err()
            .to_string()
            .contains("already refused")
    );
}
