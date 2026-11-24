// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator,NativeRecoveryPlan,RecoveryPlan},
    physics::{Body,Island,Primitive,SimulationStep,Snapshot,fixture},
    transport::WorkerProcess
};
use std::{error::Error,path::Path};
fn conserved(s:&Snapshot)->([f64;3],[f64;3],f64){
    let mut linear=[0.;3];let mut angular=[0.;3];let mut energy=0.;
    for b in &s.bodies{
        let p=b.linear.map(|v|v*b.mass);let r=b.position;let radius=b.size[0]*0.5;
        let (it,iz)=match b.primitive{
            Primitive::Sphere=>(0.4*b.mass*radius*radius,0.4*b.mass*radius*radius),
            Primitive::Capsule=>{
                let h=b.size[2]*0.5;let caps=b.mass*(2.*radius)/(3.*h+2.*radius);let cylinder=b.mass-caps;
                (cylinder*(3.*radius*radius+4.*h*h)/12.+caps*(0.4*radius*radius+h*h+0.75*h*radius),
                 (cylinder*0.5+caps*0.4)*radius*radius)
            }
            _=>unreachable!()
        };
        let [x,y,z,w]=b.orientation;
        let axis=[2.*(x*z+w*y),2.*(y*z-w*x),1.-2.*(x*x+y*y)];
        let axial_rate=axis.into_iter().zip(b.angular).map(|(a,w)|a*w).sum::<f64>();
        for k in 0..3{
            let spin=it*b.angular[k]+(iz-it)*axis[k]*axial_rate;
            linear[k]+=p[k];
            angular[k]+=r[(k+1)%3]*p[(k+2)%3]-r[(k+2)%3]*p[(k+1)%3]+spin;
            energy+=0.5*(b.mass*b.linear[k].powi(2)+spin*b.angular[k]);
        }
    }
    (linear,angular,energy)
}
#[test]
fn oblique_spinning_spheres_transfer_under_load_and_resume_both_durable_formats()->Result<(),Box<dyn Error>>{
    run(Primitive::Sphere)
}
#[test]
fn oblique_capsule_sphere_transfers_under_load_and_recovers_all_future_frames()->Result<(),Box<dyn Error>>{
    run(Primitive::Capsule)
}
fn run(shape:Primitive)->Result<(),Box<dyn Error>>{
    let step=SimulationStep::Ms2;
    let base=Body{primitive:Primitive::Sphere,friction:0.37,..fixture()[0].promoted()};
    let bodies=[Body{id:4,primitive:shape,size:if shape==Primitive::Capsule{[0.6,0.6,0.9]}else{[0.6;3]},mass:1.,position:[-1.7,0.1,0.05],linear:[4.,0.,0.],angular:[0.3,-0.7,4.],..base.clone()},
        Body{id:7,size:[0.84;3],mass:2.,position:[1.4,-0.08,-0.03],linear:[-1.,0.,0.],angular:[-1.1,0.8,-2.],..base}];
    let executable=Path::new(env!("CARGO_BIN_EXE_physics_worker"));let run=if shape==Primitive::Capsule{0x400c}else{0x400b};
    let peers=[0,1].map(|i|WorkerProcess::spawn_with_step(executable,run,i as u8,&bodies[i..=i],step).unwrap());
    let mut world=Coordinator::new_recorded(run,peers,4096)?;
    let mut native=Island::restore(&Snapshot{step,bodies:bodies.to_vec(),..Default::default()})?;
    let start=conserved(&native.snapshot());let mut errors=[0f64;3];let mut impact=None;let mut migrated=None;
    let scratch=tempfile::tempdir()?;let mut recovered=None;
    for tick in 1..=430{
        if migrated.is_none() && impact.is_some(){
            let before=world.frame().clone();let src=usize::from(before.body(4).unwrap().0);
            assert_eq!(before.workers[src].components[0].ids,[4,7]);
            world.queue_impulse(4,[0.003,-0.,0.001])?;native.queue_impulse(4,[0.003,-0.,0.001])?;
            world.transfer_component(4)?;
            assert_ne!(world.frame().body(4).unwrap().0,src as u8);
            world.compact()?;
            let native_path=scratch.path().join("native");let seed_path=scratch.path().join("seed");
            world.save_native_recovery(&native_path)?;world.save_recovery(&seed_path)?;
            let a=NativeRecoveryPlan::open(&native_path)?.resume(executable)?;
            let b=RecoveryPlan::open(&seed_path)?.resume_with_ground(|run,owner,bodies,dt,ground|WorkerProcess::spawn_with_ground(executable,run,owner,bodies,dt,ground))?;
            assert_eq!(world.frame(),a.frame());assert_eq!(world.frame(),b.frame());
            recovered=Some([a,b]);world.recover(0)?;world.recover(1)?;migrated=Some(tick-1);
        }
        world.advance_routed_overlapped()?;native.step()?;
        let f=world.frame();let s=native.snapshot();
        for b in &s.bodies{assert_eq!(f.body(b.id).unwrap().1,b,"sphere {} tick {tick}",b.id);}
        let contacts:Vec<_>=f.workers.iter().flat_map(|w|w.snapshot.impulses.iter().cloned()).collect();assert_eq!(contacts,s.impulses);
        if let Some(ref mut replicas)=recovered{for c in replicas {assert_eq!(c.advance_routed_overlapped()?,f);}}
        if contacts.iter().any(|p|p.normal_impulse>0.){impact.get_or_insert(tick);}
        if migrated.is_none(){
            let (p,h,e)=conserved(&s);
            for k in 0..3{errors[0]=errors[0].max((p[k]-start.0[k]).abs());errors[1]=errors[1].max((h[k]-start.1[k]).abs());}
            errors[2]=errors[2].max(e-start.2);
        }
    }
    println!("oblique {shape:?} and sphere: first contact {impact:?}, warm transfer at {migrated:?}, 430 exact native frames and full native/seed resumes; through impact maximum component errors in linear/angular momentum {:.3e}/{:.3e}, peak excess kinetic energy {:.3e} J",errors[0],errors[1],errors[2]);
    assert!(impact.is_some_and(|i|i>150&&i<300));assert_eq!(impact,migrated);
    assert!(errors[0]<1e-10);
    assert!(errors[1]<0.005 && errors[2]<1e-10);
    for c in std::iter::once(world).chain(recovered.into_iter().flatten()){for mut p in c.into_peers(){p.shutdown()?;}}
    Ok(())
}
