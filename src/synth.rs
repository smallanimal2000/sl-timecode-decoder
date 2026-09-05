//! Synthetic control-tone generator — the ground truth for testing the decoder.
//!
//! Given a [`TimecodeFormat`], the encoder renders stereo frames for an arbitrary
//! speed profile. Because we know the exact position/pitch at every frame, the
//! decoder's output can be checked against it to tolerance. It also stands in for
//! a real recording in examples until one is provided.
//!
//! Bit `c` (the `c`-th carrier cycle) occupies carrier phase `[2πc, 2π(c+1))`,
//! and its amplitude encodes `sequence[c mod period]`. Position on the record, in
//! "bits", is therefore `phase / 2π`.

use crate::format::{LeadChannel, TimecodeFormat};
use core::f64::consts::PI;

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use crate::math::FloatExt;

/// Stateful encoder producing stereo frames.
pub struct Encoder {
    fmt: TimecodeFormat,
    seq: Vec<u8>,
    /// Total carrier phase in radians.
    phi: f64,
    /// Nominal radians/sample at pitch 1.0.
    w: f64,
}

impl Encoder {
    pub fn new(fmt: TimecodeFormat, sample_rate: f32) -> Self {
        let seq = fmt.lfsr.sequence(fmt.seed);
        let w = 2.0 * PI * fmt.carrier_hz as f64 / sample_rate as f64;
        Encoder { fmt, seq, phi: 0.0, w }
    }

    /// Set the current position, in bits (carrier cycles from sequence start).
    pub fn seek_bits(&mut self, pos_bits: f64) {
        self.phi = pos_bits * 2.0 * PI;
    }

    /// Current position in bits (fractional).
    pub fn position_bits(&self) -> f64 {
        self.phi / (2.0 * PI)
    }

    #[inline]
    fn amp_at_cycle(&self, cycle: i64) -> f32 {
        let p = self.seq.len() as i64;
        let idx = ((cycle % p) + p) % p;
        if self.seq[idx as usize] == 1 {
            self.fmt.amp_high
        } else {
            self.fmt.amp_low
        }
    }

    /// The exact stereo sample the cartridge would produce at absolute groove
    /// position `pos_bits` (carrier cycles from sequence start). Pure function of
    /// position — the basis for arbitrary motion (scratch, skip, stop, reverse).
    #[inline]
    pub fn sample_at(&self, pos_bits: f64) -> (f32, f32) {
        let phi = pos_bits * 2.0 * PI;
        let cycle = pos_bits.floor() as i64;
        let amp = self.amp_at_cycle(cycle);
        let x = (amp as f64 * phi.cos()) as f32; // lead channel
        let y = (amp as f64 * phi.sin()) as f32; // lag channel
        match self.fmt.lead {
            LeadChannel::Left => (x, y),
            LeadChannel::Right => (y, x),
        }
    }

    /// Render a single frame at the given signed `pitch` (1.0 == nominal forward)
    /// and advance the phase.
    #[inline]
    pub fn render_frame(&mut self, pitch: f64) -> (f32, f32) {
        let cycle = self.phi.div_euclid(2.0 * PI) as i64;
        let amp = self.amp_at_cycle(cycle);
        let x = (amp as f64 * self.phi.cos()) as f32; // lead channel
        let y = (amp as f64 * self.phi.sin()) as f32; // lag channel
        self.phi += self.w * pitch;
        match self.fmt.lead {
            LeadChannel::Left => (x, y),
            LeadChannel::Right => (y, x),
        }
    }

    /// Render `n` frames at a constant `pitch`, appending to `out`.
    pub fn render_const(&mut self, pitch: f64, n: usize, out: &mut Vec<(f32, f32)>) {
        out.reserve(n);
        for _ in 0..n {
            out.push(self.render_frame(pitch));
        }
    }

    /// Render `n` frames, taking the pitch for frame `i` from `pitch(i)`.
    pub fn render_with<F: FnMut(usize) -> f64>(&mut self, n: usize, mut pitch: F, out: &mut Vec<(f32, f32)>) {
        out.reserve(n);
        for i in 0..n {
            let p = pitch(i);
            out.push(self.render_frame(p));
        }
    }

    /// Add white-ish noise to a rendered buffer (deterministic, seedable) to test
    /// robustness. `amp` is the noise peak relative to full scale.
    pub fn add_noise(buf: &mut [(f32, f32)], amp: f32, seed: u64) {
        let mut s = seed | 1;
        let mut next = || xorshift(&mut s);
        for f in buf.iter_mut() {
            f.0 += amp * next();
            f.1 += amp * next();
        }
    }
}

/// xorshift64 mapped to [-1, 1).
#[inline]
fn xorshift(s: &mut u64) -> f32 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    ((*s >> 11) as f32 / (1u64 << 53) as f32) * 2.0 - 1.0
}

/// High-level motion synthesizer: drives the [`Encoder`] along an arbitrary
/// position path to emulate real turntable handling — steady play, pitch ramps,
/// turntable braking, **scratching** (back-and-forth through reversals), needle
/// **skips** (position discontinuities), and gaps/lift.
///
/// Frames and per-frame ground-truth position accumulate in [`Self::frames`] /
/// [`Self::truth`]. During a skip's transient the truth is `NaN` (position
/// undefined while the stylus flies across grooves).
pub struct MotionSynth {
    enc: Encoder,
    sr: f64,
    /// Current absolute position in bits.
    pos: f64,
    /// RNG state for skip/gap noise.
    rng: u64,
    /// Rendered stereo frames.
    pub frames: Vec<(f32, f32)>,
    /// Ground-truth position (bits) for each frame; `NaN` during skip transients.
    pub truth: Vec<f64>,
}

impl MotionSynth {
    pub fn new(fmt: TimecodeFormat, sample_rate: f32, start_bits: f64) -> Self {
        let enc = Encoder::new(fmt, sample_rate);
        MotionSynth {
            enc,
            sr: sample_rate as f64,
            pos: start_bits,
            rng: 0x9E3779B97F4A7C15,
            frames: Vec::new(),
            truth: Vec::new(),
        }
    }

    /// Nominal bit rate (carrier Hz).
    fn carrier(&self) -> f64 {
        self.enc.fmt.carrier_hz as f64
    }

    pub fn position(&self) -> f64 {
        self.pos
    }

    /// Drive for `secs`, integrating a per-frame velocity in **bits/second**
    /// (`velocity(t_seconds)`), rendering one frame per sample.
    pub fn drive<F: FnMut(f64) -> f64>(&mut self, secs: f64, mut velocity: F) {
        let n = (secs * self.sr).round() as usize;
        for i in 0..n {
            let t = i as f64 / self.sr;
            self.frames.push(self.enc.sample_at(self.pos));
            self.truth.push(self.pos);
            self.pos += velocity(t) / self.sr;
        }
    }

    /// Steady playback at constant `pitch` (1.0 == nominal) for `secs`.
    pub fn play(&mut self, pitch: f64, secs: f64) {
        let v = pitch * self.carrier();
        self.drive(secs, |_| v);
    }

    /// Linear pitch ramp from `from` to `to` over `secs` (e.g. spin-up).
    pub fn ramp(&mut self, from: f64, to: f64, secs: f64) {
        let c = self.carrier();
        self.drive(secs, move |t| (from + (to - from) * (t / secs).min(1.0)) * c);
    }

    /// Turntable brake / power-off: pitch ramps from `from_pitch` to 0.
    pub fn brake(&mut self, from_pitch: f64, secs: f64) {
        self.ramp(from_pitch, 0.0, secs);
    }

    /// Sine scratch: the record swings back and forth, speed following
    /// `±peak_pitch·sin`, for `cycles` full swings at `freq_hz`. Passes through
    /// zero speed and reverses on every half-cycle.
    pub fn scratch_sine(&mut self, peak_pitch: f64, freq_hz: f64, cycles: f64) {
        let c = self.carrier();
        let secs = cycles / freq_hz;
        let w = 2.0 * PI * freq_hz;
        self.drive(secs, move |t| peak_pitch * c * (w * t).sin());
    }

    /// Baby scratch: constant-speed forward then backward strokes with sharp
    /// reversals. `strokes` counts single strokes (alternating direction),
    /// each `stroke_secs` long at `±pitch`.
    pub fn scratch_baby(&mut self, pitch: f64, stroke_secs: f64, strokes: usize) {
        for k in 0..strokes {
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            self.play(sign * pitch, stroke_secs);
        }
    }

    /// Instantaneous needle skip: jump the position by `delta_bits` with no
    /// intervening frames (idealised, no transient).
    pub fn skip(&mut self, delta_bits: f64) {
        self.pos += delta_bits;
    }

    /// Realistic needle skip: a short broadband click/gap of `gap_secs` (position
    /// undefined, truth = NaN) while the stylus crosses grooves, then the position
    /// jumps by `delta_bits`. `click_amp` is the transient level (full scale).
    pub fn skip_with_gap(&mut self, delta_bits: f64, gap_secs: f64, click_amp: f32) {
        let n = (gap_secs * self.sr).round() as usize;
        for i in 0..n {
            // decaying click envelope
            let env = click_amp * (1.0 - i as f32 / n.max(1) as f32);
            let l = env * xorshift(&mut self.rng);
            let r = env * xorshift(&mut self.rng);
            self.frames.push((l, r));
            self.truth.push(f64::NAN);
        }
        self.pos += delta_bits;
    }

    /// Signal dropout: the groove keeps advancing at `pitch` (position stays
    /// known) but the pickup delivers only noise for `secs` — e.g. dust, a cable
    /// glitch, or a damaged patch. The decoder should drop lock and re-acquire at
    /// the correct advanced position afterwards.
    pub fn dropout(&mut self, secs: f64, pitch: f64, noise_amp: f32) {
        let n = (secs * self.sr).round() as usize;
        let v = pitch * self.carrier();
        for _ in 0..n {
            let l = noise_amp * xorshift(&mut self.rng);
            let r = noise_amp * xorshift(&mut self.rng);
            self.frames.push((l, r));
            self.truth.push(self.pos);
            self.pos += v / self.sr;
        }
    }

    /// A gap with the stylus lifted / silent groove for `secs` (optionally low
    /// noise), position held.
    pub fn silence(&mut self, secs: f64, noise_amp: f32) {
        let n = (secs * self.sr).round() as usize;
        for _ in 0..n {
            let l = noise_amp * xorshift(&mut self.rng);
            let r = noise_amp * xorshift(&mut self.rng);
            self.frames.push((l, r));
            self.truth.push(f64::NAN);
        }
    }

    /// Add global additive noise to everything rendered so far.
    pub fn add_noise(&mut self, amp: f32, seed: u64) {
        Encoder::add_noise(&mut self.frames, amp, seed);
    }

    /// Add a constant per-channel DC offset to everything rendered so far. (Real
    /// turntable audio has no DC, but soundcard/ADC coupling or upstream DSP can
    /// introduce it; the decoder's DC blocker should absorb it.)
    pub fn add_dc(&mut self, dc_l: f32, dc_r: f32) {
        for f in self.frames.iter_mut() {
            f.0 += dc_l;
            f.1 += dc_r;
        }
    }
}
