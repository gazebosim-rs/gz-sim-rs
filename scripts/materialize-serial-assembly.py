#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Repackage actual selected-plugin serial-tree data; never derive a trajectory."""
import argparse
import hashlib
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'crates/gz-physics-dart/tests/fixtures/serial_assembly.cc'
OUTPUT=ROOT/'crates/gz-physics-dart/tests/fixtures/serial_assembly_native.json'

def require(ok,message):
    if not ok:
        raise ValueError(message)

def digest(data):
    return hashlib.sha256(data).hexdigest()

def parse(data):
    def unique(pairs):
        result={}
        for key,value in pairs:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    def reject(value):
        raise ValueError('nonfinite JSON: '+value)
    return json.loads(data,object_pairs_hook=unique,parse_constant=reject)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['generated','stdout','stderr','provenance']:
        parser.add_argument(name,type=Path)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    args=parser.parse_args()
    names={'0.sdf','1.sdf','before.maps','after.maps'}
    require(args.generated.is_dir() and not args.generated.is_symlink() and {p.name for p in args.generated.iterdir()}==names,'capture files differ')
    paths=[SOURCE,args.stdout,args.stderr,args.provenance]+[args.generated/name for name in sorted(names)]
    require(all(p.is_file() and not p.is_symlink() for p in paths),'nonregular capture input')
    before={p:p.read_bytes() for p in paths}
    require(not before[args.stderr],'unexpected native stderr')
    provenance=parse(before[args.provenance])
    require(provenance.get('state')=='Actual plugin observations collected; no Rust/native comparison claimed'
            and 'failure' not in provenance and 'traceback' not in provenance,'capture not successful')
    generator=digest(before[SOURCE]);require(provenance['generator_sha256']==generator,'generator differs')
    rows=[parse(line) for line in before[args.stdout].decode().splitlines()]
    require(len(rows)==8,'expected exactly8 rows')
    cases=[]
    for scenario in range(2):
        selected=rows[scenario*4:scenario*4+4]
        for row,tick in zip(selected,[0,1,10,100]):
            require(set(row)=={'scenario','tick','requested_time_ns','bodies','joints'},'row schema')
            require(type(row['scenario']) is int and row['scenario']==scenario and type(row['tick']) is int and row['tick']==tick
                    and type(row['requested_time_ns']) is int and row['requested_time_ns']==tick*1000000,'row clock/domain')
            require([b['name'] for b in row['bodies']]==['root','middle','leaf'],'body identity/order')
            for body in row['bodies']:
                require(set(body)=={'name','values'} and len(body['values'])==22
                        and all(type(v) in (int,float) and math.isfinite(v) for v in body['values']),'body values')
                require(abs(sum(v*v for v in body['values'][3:7])-1)<=1e-12,'nonunit orientation')
                require(body['values'][19:]==[0,0,0.1],'local COM changed')
            require(len(row['joints'])==2 and all(len(v)==2 and all(type(n) in (int,float) and math.isfinite(n) for n in v) for v in row['joints']),'joint values')
        if scenario==1:
            initial=selected[0]['bodies'][0]['values'][18]
            final=selected[-1]['bodies'][0]['values'][18]
            ballistic=initial-9.81*0.001**2*100*101/2
            require(final>ballistic+0.01,'floor case did not show actual contact response')
        xml=before[args.generated/f'{scenario}.sdf']
        # Exact XML is checked independently by the consuming Rust recipe.
        cases.append({'scenario':scenario,'xml':xml.decode(),'xml_sha256':digest(xml),'rows':selected})
    value={'schema_version':1,'copyright':'Copyright (C) 2026 Open Source Robotics Foundation','license':'Apache-2.0',
           'scope':'Selected native plugin final three-body tree; no B03 diagnosis or performance qualification',
           'time_scope':'requested dt times counted ForwardStep calls; not queried simulator clock',
           'generator_sha256':generator,'raw_stdout':before[args.stdout].decode(),
           'capture':{'provenance_sha256':digest(before[args.provenance]),'stdout_sha256':digest(before[args.stdout]),
                      'stderr_sha256':digest(before[args.stderr]),'maps_sha256':{name:digest(before[args.generated/f'{name}.maps']) for name in ['before','after']}},'cases':cases}
    require(all(p.read_bytes()==data for p,data in before.items()),'capture changed')
    with args.output.open('x') as output:
        output.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print('Materialized actual two-case serial-tree capture:',args.output)

if __name__=='__main__':
    main()
