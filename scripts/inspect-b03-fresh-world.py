#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Read retained ONE paired diagnostic, refuse observer effects before comparison.

No process execution, physics reconstruction, expected-data materialization, or
acceptance decision. Native ECM contact order is a per-owner message/point order;
it is NOT the raw global CollisionResult or LCP constraint order.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex

BOUND = 1e-12
XML_SHA = '24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native_frames(path, expected_owners):
    assert len(expected_owners) == 15
    frames = {}
    contact_owners = {}
    pairs = {}
    points = {}
    active_owner = None
    active_pair = None
    for line in path.read_text().splitlines():
        row = shlex.split(line)
        if not row:
            continue
        tag = row[0]
        if tag == 'FRAME':
            tick, time = map(int, row[1:])
            assert tick not in frames
            frames[tick] = {'tick': tick, 'time_ns': time, 'bodies': [], 'joints': []}
        elif tag in ('BODY', 'JOINT'):
            tick, name, identity = int(row[1]), row[2], int(row[3])
            values = list(map(float, row[4:]))
            assert len(values) == (22 if tag == 'BODY' else 2)
            assert all(map(math.isfinite, values))
            frames[tick]['bodies' if tag == 'BODY' else 'joints'].append({'name': name, 'id': identity, 'values': values})
        elif tag == 'CONTACT_MESSAGES':
            assert len(row) == 4
            tick, owner, count = int(row[1]), row[2], int(row[3])
            assert tick in range(101) and owner in expected_owners
            assert count >= 0 and owner not in contact_owners.setdefault(tick, {})
            contact_owners[tick][owner] = count
            active_owner, active_pair = (tick, owner), None
            pairs[active_owner] = []
        elif tag == 'CONTACT_PAIR':
            assert len(row) == 10
            tick, owner, message = int(row[1]), row[2], int(row[3])
            assert active_owner == (tick, owner), 'dangling contact header'
            assert message == len(pairs[active_owner]), 'duplicate/unordered message index'
            assert message < contact_owners[tick][owner]
            first, second = row[4:6]
            assert first == owner and first in expected_owners and second in expected_owners
            counts = list(map(int, row[6:]))
            assert all(n >= 0 for n in counts)
            # This selected DART observer requires all four fields. Unavailable
            # extras must fail and be reported, never fabricated as zeroes.
            assert counts == [counts[0]] * 4, 'incomplete DART contact fields'
            active_pair = (tick, owner, message)
            assert active_pair not in points
            pairs[active_owner].append((message, counts[0]))
            points[active_pair] = []
        elif tag == 'CONTACT_POINT':
            assert len(row) == 18
            tick, owner, message, point = int(row[1]), row[2], int(row[3]), int(row[4])
            key = (tick, owner, message)
            assert active_pair == key and key in points, 'dangling contact point'
            assert point == len(points[key]), 'duplicate/unordered point index'
            assert point < pairs[(tick, owner)][message][1], 'point exceeds advertised cardinality'
            assert all(map(math.isfinite, map(float, row[5:])))
            points[key].append(point)
        elif tag.startswith('CONTACT_'):
            raise AssertionError('unrecognized contact record')
    assert set(contact_owners) == set(range(101))
    assert all(set(owners) == expected_owners for owners in contact_owners.values())
    for (tick, owner), headers in pairs.items():
        assert len(headers) == contact_owners[tick][owner], 'missing contact pair'
        assert [message for message, _ in headers] == list(range(contact_owners[tick][owner]))
        for message, count in headers:
            assert points[(tick, owner, message)] == list(range(count)), 'missing contact point'
    return list(frames.values()), contact_owners


def fresh_frames(path, cached):
    """Authoritative generic query frames, separate from all cached body rows."""
    names = {b['name'] for b in cached[0]['bodies']}
    native_ids = {b['name']: b['id'] for b in cached[0]['bodies']}
    physical_ids = set()
    declarations = {}
    query_ids = set()
    point_rows = {}
    bodies = {}
    for line in path.read_text().splitlines():
        row = shlex.split(line)
        if not row:
            continue
        if row[0] == 'ENTITY':
            physical_ids.add(int(row[3]))
        elif row[0] == 'QUERY_FRAME':
            assert len(row) == 12
            role, name, parent, entity = row[1], row[2], int(row[3]), int(row[4])
            assert role in ('origin', 'declared_com') and name in names
            assert parent == native_ids[name] and entity not in query_ids
            key = (name, role)
            assert key not in declarations
            local = list(map(float, row[5:]))
            expected_com = next(b['values'][19:22] for b in cached[0]['bodies'] if b['name'] == name)
            assert local == ([0., 0., 0.] if role == 'origin' else expected_com) + [0., 0., 0., 1.]
            declarations[key] = {'parent': parent, 'entity': entity, 'local': local}
            query_ids.add(entity)
        elif row[0] == 'FRESH_POINT':
            assert len(row) == 20
            tick, time, role, name, parent, entity = int(row[1]), int(row[2]), row[3], row[4], int(row[5]), int(row[6])
            assert tick in range(101) and time == tick * 1_000_000
            assert (name, role) in declarations
            declaration = declarations[(name, role)]
            assert parent == declaration['parent'] and entity == declaration['entity']
            key = (tick, name, role)
            assert key not in point_rows
            values = list(map(float, row[7:]))
            assert len(values) == 13 and all(map(math.isfinite, values))
            point_rows[key] = values
        elif row[0] == 'FRESH_BODY':
            assert len(row) == 29
            tick, time, name = int(row[1]), int(row[2]), row[3]
            parent, origin_id, com_id = map(int, row[4:7])
            assert tick in range(101) and time == tick * 1_000_000 and name in names
            assert parent == native_ids[name]
            assert origin_id == declarations[(name, 'origin')]['entity']
            assert com_id == declarations[(name, 'declared_com')]['entity']
            key = (tick, name)
            assert key not in bodies
            values = list(map(float, row[7:]))
            assert len(values) == 22 and all(map(math.isfinite, values))
            origin, com = point_rows[(tick, name, 'origin')], point_rows[(tick, name, 'declared_com')]
            expected = origin[:7] + origin[7:10] + com[7:10] + origin[10:13] + com[:3] + declarations[(name, 'declared_com')]['local'][:3]
            assert values == expected, 'fresh body fields detached from actual point rows'
            assert all(abs(a-b) <= BOUND for a,b in zip(origin[10:13], com[10:13])), 'two point frames disagree on rigid angular velocity'
            bodies[key] = {'name': name, 'id': parent, 'values': values}
        elif row[0].startswith(('QUERY_', 'FRESH_')):
            raise AssertionError('unknown authoritative query record')
    assert len(declarations) == 14 and len(query_ids) == 14 and not query_ids.intersection(physical_ids)
    assert set(declarations) == {(name, role) for name in names for role in ('origin', 'declared_com')}
    assert set(point_rows) == {(tick, name, role) for tick in range(101) for name in names for role in ('origin', 'declared_com')}
    assert set(bodies) == {(tick, name) for tick in range(101) for name in names}
    return {tick: {'tick': tick, 'time_ns': tick*1_000_000,
                   'bodies': [bodies[(tick,name)] for name in sorted(names)],
                   'joints': cached[tick]['joints']} for tick in range(101)}


def indexed(frames):
    result = {}
    for frame in frames:
        tick = frame['tick']
        assert tick not in result and frame['time_ns'] == tick * 1_000_000
        result[tick] = frame
        for family, count, fields in [('bodies', 7, 22), ('joints', 4, 2)]:
            rows = frame[family]
            assert len(rows) == count and len({r['name'] for r in rows}) == count
            assert all(len(r['values']) == fields and all(map(math.isfinite, r['values'])) for r in rows)
    return result


def fidelity(now, prior, role, ticks=(1, 10, 100)):
    prior_index = indexed(prior)
    assert len(prior) == len(ticks) and set(prior_index) == set(ticks)
    maximum = 0.0
    for frame in prior:
        tick = frame['tick']
        assert tick in ticks
        for family in ('bodies', 'joints'):
            current = {r['name']: r for r in now[tick][family]}
            assert set(current) == {r['name'] for r in frame[family]}
            for row in frame[family]:
                a, b = current[row['name']]['values'], row['values']
                assert len(a) == len(b)
                for field, (actual, old) in enumerate(zip(a, b)):
                    assert math.isfinite(old)
                    delta = abs(actual - old)
                    maximum = max(maximum, delta)
                    if delta > BOUND:
                        raise AssertionError(f'OBSERVER EFFECT {role} tick{tick} {row["name"]}[{field}]: {actual:.17e} versus {old:.17e}')
    assert {f['tick'] for f in prior} == set(ticks)
    return maximum


def inspect(args):
    baseline = json.loads(args.native_baseline.read_text())
    assert baseline['xml_sha256'] == XML_SHA and sha(args.native_prefix.with_suffix('.input.sdf')) == XML_SHA
    expected_owners = {e['name'] for e in baseline['entities'] if e['family'] == 'COLLISION'}
    assert sum(e['family'] == 'COLLISION' for e in baseline['entities']) == 15
    frames, contacts = native_frames(args.native_prefix.with_suffix('.state'), expected_owners)
    native = indexed(frames)
    assert set(native) == set(range(101))
    native_effect = fidelity(native, baseline['frames'], 'native')
    previous_frames, _ = native_frames(args.previous_native_prefix.with_suffix('.state'), expected_owners)
    previous = indexed(previous_frames)
    assert set(previous) == set(range(101))
    cached_effect = fidelity(native, previous_frames, 'native101 cached', tuple(range(101)))
    fresh = fresh_frames(args.native_prefix.with_suffix('.state'), native)
    # Tick zero consistency catches a query frame that stayed at its zero default.
    fidelity(fresh, [native[0]], 'initial fresh query frame', (0,))
    if args.native_only:
        return {'scope': 'native observer fidelity only; diagnostic, not qualification',
                'native_maximum_delta': native_effect, 'all101_cached_maximum_delta': cached_effect,
                'fresh_frames': list(fresh.values())}
    assert args.rust is not None and args.rust_baseline is not None
    rust = json.loads(args.rust.read_text())
    assert rust['xml_sha256'] == XML_SHA and hashlib.sha256(rust['xml'].encode()).hexdigest() == XML_SHA
    actual = indexed(rust['frames'])
    assert set(native) == set(actual) == set(range(101))
    assert [r['tick'] for r in rust['contacts']] == list(range(101))
    for snapshot in rust['contacts']:
        rows = [shlex.split(line) for line in snapshot['native_getters_and_last_contacts'].splitlines()]
        count = [int(r[1]) for r in rows if r[0] == 'CONTACT_COUNT']
        points = [r for r in rows if r[0] == 'CONTACT']
        assert len(count) == 1 and count[0] == len(points)
        assert [int(r[1]) for r in points] == list(range(count[0]))
        assert all(len(r) == 16 and all(map(math.isfinite, map(float, r[6:]))) for r in points)
    # STOP on any observer effect; reuse the existing Rust101 data, never rerun it.
    rust_effect = fidelity(actual, json.loads(args.rust_baseline.read_text()), 'Rust')
    first = None
    maximum = 0.0
    frame_deltas = []
    for tick in range(101):
        outside = 0
        largest = 0.0
        for family in ('bodies', 'joints'):
            expected = {r['name']: r for r in fresh[tick][family]}
            assert set(expected) == {r['name'] for r in actual[tick][family]}
            for row in actual[tick][family]:
                for field, (a, b) in enumerate(zip(row['values'], expected[row['name']]['values'])):
                    delta = abs(a - b)
                    largest = max(largest, delta)
                    if delta > BOUND:
                        outside += 1
                        if first is None:
                            first = {'tick': tick, 'name': row['name'], 'family': family, 'field': field, 'actual': a, 'native': b, 'delta': delta}
        maximum = max(maximum, largest)
        frame_deltas.append({'tick': tick, 'outside_bound': outside, 'maximum_absolute_delta': largest})
    return {'scope': 'diagnostic only; not qualification or cause proof', 'bound': BOUND,
            'observer_fidelity': {'native_maximum_delta': native_effect, 'rust_maximum_delta': rust_effect, 'all101_cached_maximum_delta': cached_effect},
            'first_divergence': first, 'maximum_absolute_delta': maximum, 'frames': frame_deltas,
            'native_contact_message_counts': contacts, 'fresh_frames': list(fresh.values()),
            'cached_frames': list(native.values()),
            'limitations': ['No global Dantzig row order observation', 'Native per-owner contact messages can duplicate pairs and retain cached data if Physics contactsEql considers them unchanged', 'Rust contact index is last CollisionResult order, not native ECM message order', 'Initial native body/shape/joint metadata are DOM/ECM unless explicitly reported as engine getter']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-prefix', type=Path, required=True)
    parser.add_argument('--rust', type=Path)
    parser.add_argument('--previous-native-prefix', type=Path, required=True, help='Completed prior101 cached capture, untouched')
    parser.add_argument('--native-baseline', type=Path, required=True)
    parser.add_argument('--rust-baseline', type=Path)
    parser.add_argument('--native-only', action='store_true', help='Stop before Rust execution if new native observation changes the old trajectory')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    inputs = [args.native_prefix.with_suffix(s) for s in ('.state', '.input.sdf', '.maps', '.metrics.json')]
    inputs += [args.native_baseline, args.previous_native_prefix.with_suffix('.state')]
    if not args.native_only:
        assert args.rust is not None and args.rust_baseline is not None
        inputs += [args.rust, args.rust.with_suffix('.maps'), args.rust_baseline]
    before = {str(p): sha(p) for p in inputs}
    result = {'inputs_sha256': before, 'state': 'unverified'}
    failure = None
    try:
        result.update(inspect(args))
        assert before == {str(p): sha(p) for p in inputs}, 'diagnostic input changed'
        result['state'] = 'Observer fidelity passed; diagnostic comparison only'
    except (AssertionError, KeyError, ValueError, OSError) as error:
        result['failure'] = str(error)
        failure = error
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    if failure is not None:
        raise failure


if __name__ == '__main__':
    main()
