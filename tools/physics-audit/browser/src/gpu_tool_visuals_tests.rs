// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn trial() -> (Vec<Value>, Vec<Value>) {
    let mut frames = vec![];
    let mut samples = vec![];
    for tick in 0..2 {
        let s = json!({"time_ns":tick*1_000_000,"arms":[{
            "lane_id":0,"base":[0.,0.,0.8],"tip":[0.,0.,0.07],
            "tool_radius_m":0.02,"ground_z_m":-0.75,"surface_gap_m":0.,
            "friction_mu":0.3,"normal_impulse_ns":0.2*f64::from(tick),"tangent_impulse_ns":-0.05*f64::from(tick),
            "first_contact_offset_s":if tick==0{-1.}else{0.00073},
        }]});
        frames.push(json!({"ns":tick*1_000_000,"scene":{"visuals":[
            {"name":"default::arm_0::tool","color":[0.94,0.46,0.1,1.],"shape":{"kind":"sphere","radius":0.02}},
            {"name":"default::arm_0::ground","translation":[0.,0.,0.03],"rotation":[0.,0.,0.,1.],"shape":{"kind":"box","size":[1.5,0.8,0.04]}},
            {"name":"default::arm_0::contact_marker","color":[1.,0.16,0.08,1.],"shape":{"kind":"sphere","radius":0.008},"translation":[0.,0.,if tick==0{0.03}else{0.103}]}
        ]}}));
        samples.push(s);
    }
    (frames, samples)
}

#[test]
fn validates_impulse_and_touchdown_time_even_when_the_tool_has_rebounded() -> Result<()> {
    let (mut f, mut s) = trial();
    f[1]["scene"]["visuals"][2]["translation"][2] = json!(0.104);
    s[1]["arms"][0]["surface_gap_m"] = json!(0.001);
    s[1]["arms"][0]["tip"][2] = json!(0.071);
    let report = verify(&f, &s)?;
    assert_eq!(report["positive_normal_samples"], 1);
    assert_eq!(report["first_positive_tick"], 1);
    assert!(close(
        report["first_contact_ms_by_lane"]["0"].as_f64().unwrap(),
        0.73
    ));
    assert_eq!(report["sum_signed_tangent_impulse_ns"], -0.05);
    Ok(())
}
#[test]
fn geometry_coulomb_bound_and_tick_convention_are_checked_independently() {
    assert!(verify(&[], &[]).is_err());
    for (path, v) in [
        ("/time_ns", json!(2)),
        ("/arms", Value::Null),
        ("/arms/0/lane_id", Value::Null),
        ("/arms/0/base", Value::Null),
        ("/arms/0/tip", Value::Null),
        ("/arms/0/ground_z_m", Value::Null),
        ("/arms/0/tool_radius_m", json!(-1.)),
        ("/arms/0/tool_radius_m", json!(0.03)),
        ("/arms/0/surface_gap_m", json!(0.02)),
        ("/arms/0/normal_impulse_ns", json!(-0.1)),
        ("/arms/0/tangent_impulse_ns", json!(0.1)),
        ("/arms/0/friction_mu", json!(-1.)),
        ("/arms/0/first_contact_offset_s", json!(0.002)),
        ("/arms/0/first_contact_offset_s", Value::Null),
    ] {
        let (f, mut s) = trial();
        *s[1].pointer_mut(path).unwrap() = v;
        assert!(verify(&f, &s).is_err(), "{path}");
    }
    for (path, v) in [
        ("/ns", json!(4)),
        ("/scene/visuals", Value::Null),
        ("/scene/visuals/0/color", json!([0., 0., 0., 1.])),
        ("/scene/visuals/0/shape/kind", json!("box")),
        ("/scene/visuals/0/name", json!("other-tip")),
        ("/scene/visuals/1/name", json!("other-floor")),
        ("/scene/visuals/1/shape/kind", json!("sphere")),
        ("/scene/visuals/1/shape/size/0", json!(2.)),
        ("/scene/visuals/1/rotation", json!([0., 0., 1., 0.])),
        ("/scene/visuals/1/translation/2", json!(2.)),
        ("/scene/visuals/1/translation/0", json!(1.)),
        ("/scene/visuals/2/translation", Value::Null),
        ("/scene/visuals/2/translation/2", json!(0.03)),
        ("/scene/visuals/2/shape/kind", json!("box")),
        ("/scene/visuals/2/shape/radius", json!(0.1)),
        ("/scene/visuals/2/color", Value::Null),
        ("/scene/visuals/2/color", json!([1., 0., 0., 1.])),
    ] {
        let (mut f, s) = trial();
        *f[1].pointer_mut(path).unwrap() = v;
        assert!(verify(&f, &s).is_err(), "{path}");
    }
    let (f, mut s) = trial();
    s[0]["arms"][0]["first_contact_offset_s"] = json!(0.);
    assert!(verify(&f, &s).is_err());
    s[0]["arms"][0]["normal_impulse_ns"] = json!(0.1);
    let mut colored = f.clone();
    colored[0]["scene"]["visuals"][2]["translation"][2] = json!(0.103);
    assert!(verify(&colored, &s).is_err());
    let (f, mut s) = trial();
    s[1]["arms"] = json!([]);
    assert!(verify(&f, &s).is_err());
}
