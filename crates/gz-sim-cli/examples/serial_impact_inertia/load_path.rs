// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
#[path = "load_path/momentum.rs"]
mod momentum;
use momentum::BalanceFrame;

#[derive(Clone, Serialize)]
pub(super) struct Transmission {
    parcel_on_finger_ns: [f64; 3],
    finger_on_palm_ns: [f64; 3],
    hand_on_wrist_ns: [f64; 3],
    parcel_on_finger_about_wrist_nms: [f64; 3],
    finger_on_palm_about_wrist_nms: [f64; 3],
    hand_on_wrist_about_wrist_nms: [f64; 3],
    palm_relative_to_nut_delta_rad_s: [f64; 3],
}

impl Transmission {
    fn from_motion(
        loaded: &[State],
        unloaded: &[State],
        finger: usize,
        frame: &BalanceFrame,
        t: f64,
    ) -> Self {
        let delta = |i: usize| frame.difference(i, &loaded[i], &unloaded[i], t);
        let (parcel_change, parcel_spin) = delta(5);
        let (finger_change, finger_spin) = delta(finger);
        let (palm_change, palm_spin) = delta(2);
        let (other_change, other_spin) = delta(if finger == 3 { 4 } else { 3 });
        let finger_on_palm = -parcel_change - finger_change;
        let finger_moment = -parcel_spin - finger_spin;
        let relative_w = |s: &[State]| DVec3::from(s[2].w) - DVec3::from(s[1].w);
        Self {
            parcel_on_finger_ns: (-parcel_change).to_array(),
            finger_on_palm_ns: finger_on_palm.to_array(),
            hand_on_wrist_ns: (finger_on_palm - palm_change - other_change).to_array(),
            parcel_on_finger_about_wrist_nms: (-parcel_spin).to_array(),
            finger_on_palm_about_wrist_nms: finger_moment.to_array(),
            hand_on_wrist_about_wrist_nms: (finger_moment - palm_spin - other_spin).to_array(),
            palm_relative_to_nut_delta_rad_s: (relative_w(loaded) - relative_w(unloaded))
                .to_array(),
        }
    }

    pub(super) fn gauge_height(&self, visual: &str) -> Option<f64> {
        let impulse = match visual {
            "default::load_path::readout::finger_to_palm" => self.finger_on_palm_ns,
            "default::load_path::readout::hand_to_wrist" => self.hand_on_wrist_ns,
            _ => return None,
        };
        Some(0.54 + 2. * DVec3::from(impulse).length())
    }
}

#[derive(Resource)]
struct Control {
    frames: Vec<Instant>,
    next: usize,
    contact: bool,
}

pub(super) fn measure(
    trial: &Trial,
    segments: Vec<ContactSegment>,
    audit: &mut Audit,
) -> Result<()> {
    if audit.unexpected_contact {
        return Err("load-path balance requires an isolated strike on the selected finger".into());
    }
    let mut miss = trial.clone();
    miss.miss = true;
    let xml = fixture(&miss);
    let (sim, bodies, _) = load_robot(&miss, &xml)?;
    let sdf = sdformat::load_str(&xml, "serial-impact.sdf", &sdformat::NoResources)?;
    let finger = if trial.right { 4 } else { 3 };
    let mut frames = std::mem::take(&mut audit.samples);
    let clear = states(&sim, &bodies);
    let basis = BalanceFrame::new(&sdf, &frames[0].states, &clear);
    frames[0].transmitted = Some(Transmission::from_motion(
        &frames[0].states,
        &clear,
        finger,
        &basis,
        0.,
    ));
    let initial = Control {
        frames,
        next: 1,
        contact: false,
    };
    let (_, control) = observe(sim, segments, initial, move |control, cut| {
        control.contact |= cut
            .contact_patches()
            .iter()
            .any(|patch| patch.points.iter().any(|p| p.normal_impulse > 0.));
        let frame = &mut control.frames[control.next];
        if u128::from(frame.end_ns) == cut.elapsed.as_nanos() {
            let unloaded: Vec<_> = bodies
                .iter()
                .map(|id| State::from(cut.state(*id).unwrap()))
                .collect();
            frame.transmitted = Some(Transmission::from_motion(
                &frame.states,
                &unloaded,
                finger,
                &basis,
                cut.elapsed.as_secs_f64(),
            ));
            control.next += 1;
        }
    })?;
    if control.contact || control.next != control.frames.len() {
        return Err(
            "unloaded control acquired contact or failed to match the native sample clock".into(),
        );
    }
    audit.samples = control.frames;
    Ok(())
}

pub(super) fn add_gauges(xml: String) -> String {
    let mut model = String::from(
        r#"<model name="load_path"><static>true</static><link name="readout"><collision name="stand"><geometry><sphere><radius>0.001</radius></sphere></geometry></collision>"#,
    );
    for (name, x, color) in [
        ("finger_to_palm", -0.24, "1 0.59 0.12 1"),
        ("hand_to_wrist", -0.29, "0.78 0.42 1 1"),
    ] {
        model.push_str(&format!(r#"<visual name="{name}_scale"><pose>{x} -0.12 0.59 0 0 0</pose><geometry><box><size>0.003 0.003 0.1</size></box></geometry><material><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
        <visual name="{name}"><pose>{x} -0.12 0.54 0 0 0</pose><geometry><sphere><radius>0.008</radius></sphere></geometry><material><diffuse>{color}</diffuse></material></visual>"#));
    }
    model.push_str("</link></model></world>");
    xml.replace("</world>", &model)
}

#[cfg(test)]
#[path = "load_path/tests.rs"]
mod tests;
