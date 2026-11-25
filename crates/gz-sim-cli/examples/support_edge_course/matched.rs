// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An independent always-dynamic edge continuation initialized at the wake boundary.
use super::*;

pub(super) struct Restart {
    lane: Lane,
    incoming_error: Value,
    first_impulse: Option<f64>,
    peak_center_error: [f64; 2],
    peak_angle_error: [f64; 2],
}

impl Restart {
    pub(super) fn new(source: &Lane, seed: &str) -> Result<Self> {
        let incoming = source.bodies();
        let foot = incoming[0];
        let upper = incoming[1];
        // The original exact-lock assembly has two equal masses with centered inertia.
        // The SDF initial velocity specifies the twist at their shared center of mass.
        let linear = (DVec3::from_array(foot.velocity) + DVec3::from_array(upper.velocity)) * 0.5;
        let [x, y, z] = foot.position;
        let [qx, qy, qz, qw] = foot.orientation;
        let [wx, wy, wz] = foot.angular_velocity;
        let doc = roxmltree::Document::parse(EDGE)?;
        let model = doc
            .descendants()
            .find(|n| n.has_tag_name("model") && n.attribute("name") == Some("rocker"))
            .ok_or("missing rocker")?;
        let pose = model
            .children()
            .find(|n| n.has_tag_name("pose"))
            .ok_or("missing rocker pose")?;
        let mut edits = vec![(
            pose.range(),
            format!(r#"<pose rotation_format="quat_xyzw">{x} {y} {z} {qx} {qy} {qz} {qw}</pose>"#),
        )];
        for (tag, text) in [
            (
                "linear_velocity",
                format!("{} {} {}", linear.x, linear.y, linear.z),
            ),
            ("angular_velocity", format!("{wx} {wy} {wz}")),
        ] {
            let node = model
                .descendants()
                .find(|n| n.has_tag_name(tag))
                .ok_or("missing initial twist")?;
            edits.push((node.range(), format!("<{tag}>{text}</{tag}>")));
        }
        let mut xml = EDGE.to_owned();
        edits.sort_by_key(|(range, _)| std::cmp::Reverse(range.start));
        for (range, text) in edits {
            xml.replace_range(range, &text);
        }
        let lane = Lane::new(&xml, seed, &["ledge::ground"])?;
        let reconstructed = lane.bodies();
        let (position, angle) = errors(incoming, reconstructed);
        let linear = std::array::from_fn::<_, 2, _>(|i| {
            DVec3::from_array(incoming[i].velocity)
                .distance(DVec3::from_array(reconstructed[i].velocity))
        });
        let angular = std::array::from_fn::<_, 2, _>(|i| {
            DVec3::from_array(incoming[i].angular_velocity)
                .distance(DVec3::from_array(reconstructed[i].angular_velocity))
        });
        if [position, angle, linear, angular]
            .into_iter()
            .flatten()
            .any(|e| e > 1e-12)
        {
            return Err(format!(
                "unmatched edge restart: {position:?} {angle:?} {linear:?} {angular:?}"
            )
            .into());
        }
        Ok(Self {
            lane,
            incoming_error: json!({"position_m":position,"rotation_rad":angle,"velocity_m_s":linear,"angular_velocity_rad_s":angular}),
            first_impulse: None,
            peak_center_error: [0.; 2],
            peak_angle_error: [0.; 2],
        })
    }
    pub(super) fn advance(&mut self, candidate: &Lane) -> Result<()> {
        self.lane.sim.step_exact(1)?;
        if self.first_impulse.is_none() {
            let impulse = self.lane.reactions()[0];
            if impulse <= 0. || (candidate.reactions()[0] / impulse - 1.).abs() > 0.05 {
                return Err(format!("matched edge reaction exceeds 5%: {impulse}").into());
            }
            self.first_impulse = Some(impulse);
        }
        let (dp, da) = errors(self.lane.bodies(), candidate.bodies());
        for i in 0..2 {
            self.peak_center_error[i] = self.peak_center_error[i].max(dp[i]);
            self.peak_angle_error[i] = self.peak_angle_error[i].max(da[i]);
        }
        Ok(())
    }
    pub(super) fn report(self) -> Result<Value> {
        if self.peak_center_error[0] > 1e-4 {
            return Err("matched edge restart exceeds the 0.1 mm center bound".into());
        }
        Ok(
            json!({"incoming_match_foot_upper":self.incoming_error,"first_dynamic_impulse_ns":self.first_impulse,
            "resumed_peak_center_error_m_foot_upper":self.peak_center_error,"resumed_peak_angle_error_rad_foot_upper":self.peak_angle_error}),
        )
    }
}
