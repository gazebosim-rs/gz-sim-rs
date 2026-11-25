use bevy_ecs::prelude::*;
use gz_sim_scene::{EntityKind, LocalPose, Velocity, WorldPose};
use serde_json::{json, Value};
use std::{collections::{BTreeMap,BTreeSet}, fs, os::unix::fs::FileExt};

#[derive(Clone)]
struct Row {
    values: [u64; 7],
    len: usize,
    address: usize,
    changed: u32,
    kind: EntityKind,
    table: &'static str,
}

pub struct Snapshot(Vec<Row>);
fn pose(p: &WorldPose) -> [u64;7] {
    let [x,y,z]=p.translation;
    let [a,b,c,d]=p.rotation;
    [x,y,z,a,b,c,d].map(f64::to_bits)
}
pub fn snapshot(world: &World) -> Snapshot {
    let mut rows=Vec::with_capacity(900_004);
    for entity in world.iter_entities() {
        let Some(&kind) = entity.get::<EntityKind>() else {assert!(entity.get::<WorldPose>().is_none() && entity.get::<LocalPose>().is_none() && entity.get::<Velocity>().is_none());continue};
        if let Some(p)=entity.get_ref::<WorldPose>() {
            rows.push(Row{values:pose(&p),len:7,address:std::ptr::from_ref(&*p).addr(),changed:p.last_changed().get(),kind,table:"world"});
        }
        if let Some(p)=entity.get_ref::<LocalPose>() {
            rows.push(Row{values:pose(&p.0),len:7,address:std::ptr::from_ref(&*p).addr(),changed:p.last_changed().get(),kind,table:"local"});
        }
        if let Some(p)=entity.get_ref::<Velocity>() {
            let [x,y,z]=p.linear;let [a,b,c]=p.angular;
            rows.push(Row{values:[x,y,z,a,b,c,0.].map(f64::to_bits),len:6,address:std::ptr::from_ref(&*p).addr(),changed:p.last_changed().get(),kind,table:"twist"});
        }
    }
    Snapshot(rows)
}
pub fn clear() {fs::write("/proc/self/clear_refs",b"4\n").unwrap();}
fn dirty_pages(pages:&BTreeSet<usize>)->usize {
    let map=fs::File::open("/proc/self/pagemap").unwrap();
    let mut n=0;
    for page in pages {
        let mut bytes=[0;8];
        map.read_exact_at(&mut bytes,*page as u64*8).unwrap();
        let pte=u64::from_ne_bytes(bytes);
        assert_ne!(pte & (1<<63),0);
        n+=usize::from(pte & (1<<55)!=0);
    }
    n
}
pub fn compare(before:&Snapshot,after:&Snapshot)->Value {
    assert_eq!(before.0.len(),after.0.len());
    let mut all_pages=BTreeSet::new();let mut changed_pages=BTreeSet::new();
    let mut changed_lines=BTreeSet::new();let mut table_lines=BTreeSet::new();let mut rows=BTreeMap::new();
    for (a,b) in before.0.iter().zip(&after.0) {
        assert_eq!((a.address,a.kind,a.table),(b.address,b.kind,b.table));
        let g=rows.entry((format!("{:?}",a.kind),a.table)).or_insert([0usize;6]);
        g[0]+=1;
        for p in a.address/4096..=(a.address+8*a.len-1)/4096 {all_pages.insert(p);}
        if a.changed!=b.changed {
            g[1]+=1;g[2]+=a.len*8;
            for line in a.address/64..=(a.address+8*a.len-1)/64 {table_lines.insert(line);}
        }
        for i in 0..a.len {
            if a.values[i]!=b.values[i] {
                g[3]+=1;g[4]+=a.values[i].to_ne_bytes().iter().zip(b.values[i].to_ne_bytes()).filter(|(l,r)|**l!=*r).count();
                let addr=a.address+8*i;
                changed_lines.insert(addr/64);changed_pages.insert(addr/4096);
            }
        }
        g[5]+=usize::from(a.values!=b.values);
    }
    json!({"table_rows":rows.into_iter().map(|((k,t),v)|json!([k,t,v])).collect::<Vec<_>>(),
    "row_columns":["count","changed_tick","changed_tick_value_bytes","different_f64","different_bytes","rows_with_different_value"],
    "all_component_pages":all_pages.len(),"dirty_component_pages":dirty_pages(&all_pages),
    "different_float_pages":changed_pages.len(),"dirty_different_float_pages":dirty_pages(&changed_pages),
    "different_float_cache_lines":changed_lines.len(),"changed_tick_value_cache_lines":table_lines.len()})
}
