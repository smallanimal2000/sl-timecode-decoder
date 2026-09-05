//! Signal-processing front-end: turns raw stereo samples into a stream of sliced
//! bits plus a continuous pitch/direction estimate.
//!
//! Two things happen per sample:
//!
//! * **Phase tracking.** We treat `(x, y) = (lead, lag)` channels as a point on a
//!   circle and track the angle `θ = atan2(y, x)`. Because amplitude modulation
//!   changes only the radius, not the angle, the angle is a clean pitch/direction
//!   signal. `dθ/dt` gives pitch; its sign gives direction.
//! * **Bit slicing.** At each positive→negative zero crossing of the *lag*
//!   channel, the *lead* channel is at a peak (quadrature). We compare that peak
//!   against an adaptive threshold (a fraction of the tracked envelope) to read
//!   one bit per carrier cycle.

use crate::format::{LeadChannel, TimecodeFormat};
use core::f32::consts::PI;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use crate::math::FloatExt;

/// One decoded bit, emitted once per carrier cycle.
#[derive(Clone, Copy, Debug)]
pub struct BitEvent {
    /// The decoded bit value (0 or 1).
    pub bit: u8,
    /// Confidence in `[0, ~1]`: distance of the measured peak from the slice
    /// threshold, normalised by the envelope.
    pub confidence: f32,
    /// Sample index (in frames) at which the bit was sliced.
    pub sample: u64,
    /// Signed pitch estimate at slice time (1.0 == nominal forward speed).
    pub pitch: f32,
    /// Carrier signal level (per-sample peak envelope, ~full-scale) at slice time.
    /// Decays toward the noise floor when the carrier stops.
    pub signal: f32,
    /// The measured lead-channel peak magnitude for this bit (diagnostics).
    pub peak: f32,
}

/// One-pole DC blocker (high-pass): `y = x - x1 + R*y1`.
#[derive(Clone, Copy, Debug)]
struct DcBlock {
    x1: f32,
    y1: f32,
    r: f32,
}

impl DcBlock {
    fn new() -> Self {
        DcBlock { x1: 0.0, y1: 0.0, r: 0.995 }
    }
    #[inline]
    fn process(&mut self, x: f32) -> f32 {
        let y = x - self.x1 + self.r * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }
}

/// Tunable front-end parameters. [`Default`] gives values validated against real
/// recordings and the stress suite; override for unusual inputs.
#[derive(Clone, Copy, Debug)]
pub struct SlicerConfig {
    /// Schmitt-trigger hysteresis, as a fraction of the high level.
    pub hysteresis: f32,
    /// Bi-level follower attack (toward a cluster) rate.
    pub attack: f32,
    /// Bi-level follower release (away from a cluster) rate.
    pub release: f32,
    /// Windowed-peak length = carrier period / this (larger = shorter window).
    pub peak_window_div: f32,
    /// Refractory period as a fraction of the carrier cycle.
    pub refractory_frac: f32,
    /// Initial low/high level ratio seeding the slicer.
    pub init_lo_ratio: f32,
    /// EMA rate for the pitch/velocity estimate.
    pub pitch_alpha: f32,
    /// Release time of the carrier-presence envelope, in carrier cycles. The
    /// envelope attacks instantly to each lead peak and decays with this time
    /// constant when the carrier is absent, so a stylus lift (silence) is seen
    /// as the level collapsing toward the noise floor within a few cycles.
    pub env_release_cycles: f32,
}

impl Default for SlicerConfig {
    fn default() -> Self {
        SlicerConfig {
            hysteresis: 0.12,
            attack: 0.10,
            release: 0.02,
            peak_window_div: 6.0,
            refractory_frac: 0.4,
            init_lo_ratio: 0.85,
            pitch_alpha: 0.05,
            env_release_cycles: 6.0,
        }
    }
}

/// Streaming front-end. Feed samples with [`Frontend::push`]; each call may
/// return a [`BitEvent`].
pub struct Frontend {
    lead: LeadChannel,
    /// Radians per sample of the carrier at nominal (1.0×) speed.
    nominal_w: f32,
    cfg: SlicerConfig,

    dc_x: DcBlock,
    dc_y: DcBlock,

    // phase tracking
    have_theta: bool,
    prev_theta: f32,
    vel_ema: f32,
    /// Cumulative unwrapped carrier phase (radians) — basis for sub-bit position.
    phase_total: f64,

    // slicing — adaptive bi-level data slicer
    /// Schmitt-trigger state for the lag channel: true when in the "high" region.
    y_hi_state: bool,
    /// Sample index of the last accepted crossing (for the refractory period).
    last_cross: u64,
    have_cross: bool,
    have_levels: bool,
    /// Ring buffer of recent |lead| samples, for windowed peak measurement.
    xhist: [f32; 8],
    xh_idx: usize,
    /// Follower tracking the "high" (bit=1) peak cluster; also the signal level.
    level_hi: f32,
    /// Follower tracking the "low" (bit=0) peak cluster.
    level_lo: f32,

    /// Per-sample peak envelope of |lead| for carrier-presence detection. Unlike
    /// `level_hi` (a bi-level cluster follower updated only at crossings), this is
    /// updated every sample and decays during silence, so it collapses when the
    /// stylus is lifted and the carrier disappears.
    env: f32,
    /// Per-sample decay factor applied to `env` when the input is below it.
    env_decay: f32,

    sample_idx: u64,
}

impl Frontend {
    pub fn new(fmt: &TimecodeFormat, sample_rate: f32) -> Self {
        Self::with_config(fmt, sample_rate, SlicerConfig::default())
    }

    pub fn with_config(fmt: &TimecodeFormat, sample_rate: f32, cfg: SlicerConfig) -> Self {
        // Envelope release: decay per sample so the level falls with a time
        // constant of `env_release_cycles` carrier cycles when the carrier stops.
        let base_period = sample_rate / fmt.carrier_hz; // samples per cycle at 1x
        let tau = (cfg.env_release_cycles.max(0.5)) * base_period;
        let env_decay = (-1.0 / tau).exp();
        Frontend {
            lead: fmt.lead,
            nominal_w: 2.0 * PI * fmt.carrier_hz / sample_rate,
            cfg,
            dc_x: DcBlock::new(),
            dc_y: DcBlock::new(),
            have_theta: false,
            prev_theta: 0.0,
            vel_ema: 0.0,
            phase_total: 0.0,
            y_hi_state: false,
            last_cross: 0,
            have_cross: false,
            have_levels: false,
            xhist: [0.0; 8],
            xh_idx: 0,
            level_hi: 1e-6,
            level_lo: 1e-6,
            env: 0.0,
            env_decay,
            sample_idx: 0,
        }
    }

    /// Current signed pitch estimate (1.0 == nominal forward speed).
    #[inline]
    pub fn pitch(&self) -> f32 {
        self.vel_ema / self.nominal_w
    }

    /// Cumulative unwrapped carrier phase in radians (2π per bit); used to
    /// interpolate sub-bit position between crossings.
    #[inline]
    pub fn phase_total(&self) -> f64 {
        self.phase_total
    }

    /// Current carrier signal level: a per-sample peak envelope of the lead
    /// channel (~full scale while the carrier is present). Decays toward the
    /// noise floor when the carrier stops — e.g. on a stylus lift — so it can be
    /// compared against a presence threshold even between/without crossings.
    #[inline]
    pub fn signal_level(&self) -> f32 {
        self.env
    }

    /// Map a raw `(l, r)` frame to `(x = lead, y = lag)`.
    #[inline]
    fn to_xy(&self, l: f32, r: f32) -> (f32, f32) {
        match self.lead {
            LeadChannel::Left => (l, r),
            LeadChannel::Right => (r, l),
        }
    }

    /// Feed one stereo frame. Returns a [`BitEvent`] on carrier-cycle boundaries.
    pub fn push(&mut self, l: f32, r: f32) -> Option<BitEvent> {
        let idx = self.sample_idx;
        self.sample_idx += 1;

        let (rx, ry) = self.to_xy(l, r);
        let x = self.dc_x.process(rx);
        let y = self.dc_y.process(ry);

        // Keep a short history of |lead| for windowed (noise-averaging) peak reads.
        let ax = x.abs();
        self.xhist[self.xh_idx] = ax;
        self.xh_idx = (self.xh_idx + 1) % self.xhist.len();

        // Carrier-presence envelope: instant attack to each lead peak, exponential
        // release when the input is lower. Tracked every sample (not just at
        // crossings) so it decays to the noise floor when the carrier disappears,
        // making a stylus lift observable via `signal_level()`.
        if ax > self.env {
            self.env = ax;
        } else {
            self.env *= self.env_decay;
        }

        // --- phase / pitch tracking ---
        let theta = y.atan2(x);
        if self.have_theta {
            let mut d = theta - self.prev_theta;
            // wrap to (-pi, pi]
            while d > PI {
                d -= 2.0 * PI;
            }
            while d <= -PI {
                d += 2.0 * PI;
            }
            // Reject wild jumps from near-silence / transients.
            if d.abs() < PI {
                self.vel_ema += self.cfg.pitch_alpha * (d - self.vel_ema);
                self.phase_total += d as f64;
            }
        }
        self.prev_theta = theta;
        self.have_theta = true;

        // --- bit slicing at a zero crossing of the lag channel ---
        // A Schmitt trigger with a ±h hysteresis band tracks the lag channel's
        // sign, so broadband noise near zero can't produce spurious crossings.
        // One crossing fires per carrier cycle, mid-cycle (carrier phase ~pi),
        // where the lead channel is at a clean peak. Forward playback slices on
        // the high→low transition, reverse on low→high, keeping the slice point
        // off the cycle boundary (where the bit amplitude is switching).
        let forwardish = self.pitch() >= 0.0;
        let base_period = 2.0 * PI / self.nominal_w; // samples/cycle at pitch 1
        let period = base_period / self.pitch().abs().max(0.1);
        let h = self.cfg.hysteresis * self.level_hi.max(1e-6);
        let mut crossing = false;
        if self.y_hi_state {
            if y < -h {
                self.y_hi_state = false;
                crossing = forwardish;
            }
        } else if y > h {
            self.y_hi_state = true;
            crossing = !forwardish;
        }

        // Refractory period: after an accepted crossing, ignore further ones for
        // ~0.4 of a carrier cycle at the current speed. Near a zero crossing the
        // signal is ~0, so under heavy noise it can swing across the hysteresis
        // band several times; this enforces one bit per cycle. Speed-scaled so
        // fast scratches (shorter cycles) still slice every cycle.
        if crossing && self.have_cross {
            let min_gap = (self.cfg.refractory_frac * period).clamp(3.0, 4.0 * base_period) as u64;
            if idx.saturating_sub(self.last_cross) < min_gap {
                crossing = false;
            }
        }

        let mut event = None;
        if crossing {
            self.last_cross = idx;
            self.have_cross = true;
            // Windowed peak: average |lead| over a fraction of a cycle around the
            // peak (where the cosine is flat, so signal is preserved) to average
            // down broadband noise by ~sqrt(K). Window shrinks at high speed to
            // avoid smearing across the shorter cycle.
            let k = ((period / self.cfg.peak_window_div) as usize).clamp(1, self.xhist.len());
            let mut sum = 0.0f32;
            for j in 0..k {
                let i = (self.xh_idx + self.xhist.len() - 1 - j) % self.xhist.len();
                sum += self.xhist[i];
            }
            let peak = sum / k as f32;

            // Adaptive bi-level data slicer. Two followers track the "high"
            // (bit=1) and "low" (bit=0) peak clusters; the threshold sits between
            // them. This adapts to any dynamic range — including normalized or
            // compressed recordings where the 1/0 peaks are close together — so
            // no fixed amplitude ratio is assumed.
            if !self.have_levels {
                self.level_hi = peak.max(1e-6);
                self.level_lo = self.level_hi * self.cfg.init_lo_ratio;
                self.have_levels = true;
            }
            // Smoothed asymmetric-EMA followers. Each tracks its cluster quickly
            // in the "toward the extreme" direction and slowly the other way, so
            // single noise spikes barely move the threshold, yet the levels still
            // self-adjust to any dynamic range (the low follower drifts up toward
            // the high one until low peaks pull it back down).
            let (a_fast, a_slow) = (self.cfg.attack, self.cfg.release);
            let a_hi = if peak > self.level_hi { a_fast } else { a_slow };
            self.level_hi += a_hi * (peak - self.level_hi);
            let a_lo = if peak < self.level_lo { a_fast } else { a_slow };
            self.level_lo += a_lo * (peak - self.level_lo);

            let threshold = 0.5 * (self.level_hi + self.level_lo);
            let bit = if peak > threshold { 1u8 } else { 0u8 };
            let gap = (self.level_hi - self.level_lo).max(1e-9);
            let confidence = ((peak - threshold).abs() / gap).min(1.0);
            event = Some(BitEvent {
                bit,
                confidence,
                sample: idx,
                pitch: self.pitch(),
                signal: self.env,
                peak,
            });
        }

        event
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::serato_cv02;
    use core::f32::consts::PI;

    #[test]
    fn tracks_forward_pitch_and_slices_bits() {
        // Render a real (amplitude-modulated) timecode signal and check the front
        // end recovers pitch ~1.0 and bits obeying the LFSR recurrence.
        let fmt = serato_cv02();
        let sr = 44_100.0;
        let mut enc = crate::synth::Encoder::new(fmt.clone(), sr);
        let mut buf = Vec::new();
        enc.render_const(1.0, 30_000, &mut buf);

        let mut fe = Frontend::new(&fmt, sr);
        let mut bits = Vec::new();
        for (l, r) in buf {
            if let Some(ev) = fe.push(l, r) {
                bits.push(ev.bit);
            }
        }
        // Pitch should be ~1.0 forward.
        assert!((fe.pitch() - 1.0).abs() < 0.05, "pitch={}", fe.pitch());

        // After the slicer settles, the recovered bits must satisfy the LFSR
        // recurrence b[k+N] = XOR of tapped bits (alignment-free correctness).
        let n = fmt.lfsr.bits as usize;
        let taps = fmt.lfsr.taps;
        let (mut ok, mut total) = (0usize, 0usize);
        for k in 64..bits.len().saturating_sub(n) {
            let mut pred = 0u8;
            for j in 0..n {
                if (taps >> j) & 1 == 1 {
                    pred ^= bits[k + j];
                }
            }
            total += 1;
            if pred == bits[k + n] {
                ok += 1;
            }
        }
        assert!(total > 100, "not enough bits");
        assert_eq!(ok, total, "sliced bits violate the LFSR recurrence ({ok}/{total})");
    }

    #[test]
    fn detects_reverse_direction() {
        let mut fmt = serato_cv02();
        fmt.lead = crate::format::LeadChannel::Left;
        let sr = 44_100.0;
        let mut fe = Frontend::new(&fmt, sr);
        let w = 2.0 * PI * fmt.carrier_hz / sr;
        let mut phi = 0.0f32;
        for _ in 0..3000 {
            let (x, y) = (fmt.amp_high * phi.cos(), fmt.amp_high * phi.sin());
            fe.push(x, y);
            phi -= w; // reverse
        }
        assert!(fe.pitch() < -0.5, "pitch={}", fe.pitch());
    }
}
