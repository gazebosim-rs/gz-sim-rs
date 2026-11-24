// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A bounded camera selection of live SDF visuals from the otherwise fully published fleet.
use bevy_ecs::prelude::*;
use gz_sim_core::{EntityId,Simulation};
use gz_sim_scene::{Name,WorldPose};
use serde_json::{Value,json};
use std::{collections::HashMap,error::Error,fs::File,path::Path};
type Result<T>=std::result::Result<T,Box<dyn Error>>;
pub struct Capture{geometry:Vec<Value>,models:Vec<Value>,entities:Vec<Entity>,frames:Vec<Value>}
impl Capture{
    pub fn new(sim:&Simulation,decks:usize)->Result<Self>{
        let chosen:Vec<_>=(0..3).flat_map(|y|(0..4).map(move|x|317*y+x)).filter(|i|*i<decks).collect();
        let mut named=HashMap::new();
        for &i in &chosen {
            named.insert(format!("default::shuttle_{i}"),None);
            named.insert(format!("default::shuttle_{i}::body::visual"),None);
        }
        for e in sim.world().iter_entities(){
            if let Some(n)=e.get::<Name>() && let Some(entry)=named.get_mut(&n.0){
                *entry=Some((e.id(),e.get::<EntityId>().ok_or("missing visual identity")?.get()));
            }
        }
        let mut c=Self{geometry:Vec::new(),models:Vec::new(),entities:Vec::new(),frames:Vec::new()};
        for i in chosen{
            let model=format!("default::shuttle_{i}");let vis=format!("{model}::body::visual");
            let (e,id)=named[&vis].ok_or("visual missing")?;let (_,m)=named[&model].ok_or("model missing")?;
            let a=(i%67) as f64/66.;
            c.geometry.push(json!({"id":id,"name":vis,"shape":{"kind":"box","size":[1.,1.,0.2]},"color":[0.12+0.72*a,0.47,0.84-0.64*a,1.]}));
            c.models.push(json!({"model_id":m,"name":model,"visual_ids":[id],"can_set_state":false,"can_remove":false}));
            c.entities.push(e);
        }
        Ok(c)
    }
    pub fn sample(&mut self,sim:&Simulation)->Result<()>{
        let mut poses=Vec::new();
        for &entity in &self.entities{
            let p=sim.world().get::<WorldPose>(entity).ok_or("published visual disappeared")?;
            poses.push((p.translation,p.rotation));
        }
        self.frames.push(json!({"time_ns":sim.elapsed().as_nanos(),"poses":poses}));Ok(())
    }
    pub fn write(self,path:&Path)->Result<()>{
        serde_json::to_writer(File::create_new(path)?,&json!({"geometry":self.geometry,"models":self.models,"frames":self.frames}))?;Ok(())
    }
}
