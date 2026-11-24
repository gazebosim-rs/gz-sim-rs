// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId,PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId,Simulation};
use gz_sim_scene::{BodyHandle,ImuSample,Name,PhysicsEngine,WorldLoadOptions,load_world_with_runtime};
use serde_json::{Value,json};
use std::{error::Error,path::Path};
type Result<T>=std::result::Result<T,Box<dyn Error>>;
pub const SDF:&str=include_str!("../../../../examples/multifidelity_curved_shuttle.sdf");

/// Seed five reproduces the published SDF trial. Other seeds vary the physical path, cargo and phase.
pub fn vary(xml:String,seed:u64)->String{
    if seed==5{return xml;}
    let mut rng=seed;
    let mut u=||{rng=rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);(rng>>11) as f64 /9007199254740992.};
    xml.replace("0.55 0.2 0.25",&format!("{:.17} {:.17} {:.17}",0.3+0.30*u(),0.12+0.15*u(),0.13+0.14*u()))
       .replace("1400250000",&format!("{}",1300200000+(u()*200000000.) as u64))
       .replace("2200350000",&format!("{}",2100100000+(u()*200000000.) as u64))
       .replace("<mass>1.2</mass>",&format!("<mass>{:.17}</mass>",0.82+0.9*u()))
}
pub fn load(xml:&str,worker:Option<&Path>)->Result<Simulation>{
    Ok(load_world_with_runtime(xml,"shuttle.sdf","default",&sdformat::NoResources,WorldLoadOptions{engine:PhysicsEngine::MultiFidelity,step:None,noise_seed:[0;32],distributed_worker:worker},Default::default())?)
}
pub fn native(sim:&Simulation)->&MultiFidelityBackend{sim.world().resource::<PhysicsBackend>().downcast_ref().unwrap()}
pub fn body(sim:&Simulation,model:&str)->Result<(u64,BodyId)>{
    let name=format!("default::{model}::body");let e=sim.world().iter_entities().find(|e|e.get::<Name>().is_some_and(|n|n.0==name)).ok_or("missing native link")?;
    Ok((e.get::<EntityId>().unwrap().get(),e.get::<BodyHandle>().unwrap().id()))
}
pub fn imu(sim:&Simulation)->Result<Option<Value>>{
    sim.world().iter_entities().find_map(|e|e.get::<ImuSample>()).map(|s|{
        let r=s.reading.as_ref().map_err(|s|format!("{s:?}"))?;
        Ok(json!({"epoch":s.epoch,"time_ns":s.time.as_nanos(),"force":r.specific_force,"angular_velocity":r.angular_velocity,"orientation":r.orientation}))
    }).transpose()
}
