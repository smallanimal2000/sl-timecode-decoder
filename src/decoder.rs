//! Public decoding API: streaming stereo frames in, position/pitch/direction out.
//!
//! The decoder wires together the [`Frontend`](crate::dsp::Frontend) (pitch +
//! bit slicing) and the [`PositionMap`](crate::lfsr::PositionMap) (bit window →
//! absolute position), adding a small sync/lock layer:
//!
//! * A rolling buffer of the most recent `window_bits` sliced bits is kept in
//!   time order.
//! * When full, the window is packed into an LFSR state (orientation depends on
//!   play direction) and looked up to get the absolute position of the newest
//!   bit.
//! * The lookup is cross-checked against the position predicted from the last
//!   result; agreement builds a lock, disagreement (a bit error) is held and
//!   re-synced after a few misses.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use core::f64::consts::PI;

use crate::dsp::{Frontend, SlicerConfig};
use crate::format::TimecodeFormat;
use crate::lfsr::{pack_state, PositionMap};

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use crate::math::FloatExt;

/// Play direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Reverse,
    Stopped,
}

/// A decoded state sample, emitted once per resolved carrier cycle.
#[derive(Clone, Copy, Debug)]
pub struct DecodeState {
    /// Absolute position on the record, in bits (carrier cycles from start).
    pub position_bits: f64,
    /// Absolute position on the record, in seconds (`position_bits / carrier_hz`).
    pub position_seconds: f64,
    /// Signed pitch (1.0 == nominal forward speed).
    pub pitch: f32,
    /// Play direction.
    pub direction: Direction,
    /// Whether the decoder currently has a confident position lock.
    pub locked: bool,
    /// Slice confidence of the bit that produced this state, in `[0, 1]`.
    pub confidence: f32,
    /// Carrier signal level (per-sample peak envelope, ~full scale) at this state.
    /// Decays toward the noise floor when the carrier stops (e.g. stylus lift).
    pub signal: f32,
    /// Sub-bit interpolated absolute position (bits), smooth between crossings.
    /// Same as `position_bits` at a resolved cycle, but continuously refined from
    /// carrier phase for smooth scrubbing. `NaN` until locked.
    pub fine_position: f64,
    /// Frame index at which this state was produced.
    pub sample: u64,
}

/// Tunable decoder parameters. [`Default`] values are validated against the real
/// recordings and the stress suite.
#[derive(Clone, Copy, Debug)]
pub struct DecoderConfig {
    /// Minimum carrier level (≈ −40 dBFS default) required to hold a lock; below
    /// it the input is noise floor and any lock would be spurious.
    pub min_signal: f32,
    /// Consecutive strong-signal bits required after onset/dropout before locking
    /// (lets the slicer converge and rides out needle-settling transients).
    pub warmup_bits: u32,
    /// Consecutive agreeing lookups needed to declare lock.
    pub lock_threshold: u32,
    /// Consecutive disagreeing lookups tolerated before re-syncing to the lookup.
    pub resync_after: u32,
    /// Speeds with |pitch| below this are reported as `Stopped`.
    pub stop_pitch: f32,
    /// Front-end / slicer tuning.
    pub slicer: SlicerConfig,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        DecoderConfig {
            min_signal: 0.01,
            warmup_bits: 48,
            lock_threshold: 6,
            resync_after: 3,
            stop_pitch: 0.02,
            slicer: SlicerConfig::default(),
        }
    }
}

pub struct Decoder {
    fmt: TimecodeFormat,
    frontend: Frontend,
    map: PositionMap,
    window_bits: usize,
    period: i64,
    cfg: DecoderConfig,

    bits: VecDeque<u8>,
    last_pos: Option<i64>,
    lock_count: u32,
    disagree_count: u32,
    warmup: u32,

    // sub-bit fine position: anchored to the last locked cycle, extrapolated by
    // carrier phase between events.
    anchor_pos: i64,
    anchor_phase: f64,
    have_anchor: bool,
    fine_pos: f64,
}

impl Decoder {
    pub fn new(fmt: TimecodeFormat, sample_rate: f32) -> Self {
        Self::with_config(fmt, sample_rate, DecoderConfig::default())
    }

    pub fn with_config(fmt: TimecodeFormat, sample_rate: f32, cfg: DecoderConfig) -> Self {
        let frontend = Frontend::with_config(&fmt, sample_rate, cfg.slicer);
        let map = PositionMap::build(fmt.lfsr, fmt.seed);
        let window_bits = fmt.window_bits();
        let period = (1i64 << fmt.lfsr.bits) - 1;
        Decoder {
            fmt,
            frontend,
            map,
            window_bits,
            period,
            cfg,
            bits: VecDeque::with_capacity(window_bits + 1),
            last_pos: None,
            lock_count: 0,
            disagree_count: 0,
            warmup: 0,
            anchor_pos: 0,
            anchor_phase: 0.0,
            have_anchor: false,
            fine_pos: f64::NAN,
        }
    }

    pub fn format(&self) -> &TimecodeFormat {
        &self.fmt
    }

    /// Minimum carrier level required to hold a lock (default ≈ −40 dBFS). Raise
    /// it for noisier inputs, lower it for very quiet recordings.
    pub fn set_min_signal(&mut self, level: f32) {
        self.cfg.min_signal = level;
    }

    pub fn min_signal(&self) -> f32 {
        self.cfg.min_signal
    }

    /// Current signed pitch estimate (1.0 == nominal forward speed).
    ///
    /// Continuous motion state: valid between bit events and through lock loss,
    /// so hosts can poll it on a fixed cadence (e.g. once per audio block)
    /// rather than only on the `Some(..)` return of [`push_frame`](Self::push_frame).
    /// Pure read; does not advance any state. Intentionally does not require a
    /// position lock — relative/scratch control needs motion, not position.
    #[inline]
    pub fn pitch(&self) -> f32 {
        self.frontend.pitch()
    }

    /// Current carrier signal level (per-sample peak envelope, ~full scale).
    ///
    /// Continuous motion state; compare against [`min_signal`](Self::min_signal)
    /// to decide carrier presence. Decays toward the noise floor when the carrier
    /// stops, so a stylus lift is observable here even though no bit events fire.
    /// Pure read; does not advance any state.
    #[inline]
    pub fn signal_level(&self) -> f32 {
        self.frontend.signal_level()
    }

    /// Cumulative unwrapped carrier phase in radians (2π per bit).
    ///
    /// Useful for host-side sub-bit interpolation. Pure read; does not advance
    /// any state.
    #[inline]
    pub fn phase_total(&self) -> f64 {
        self.frontend.phase_total()
    }

    /// True when the carrier is present and the record is moving fast enough to
    /// trust the signed [`pitch`](Self::pitch), independent of bit-level position
    /// lock. Uses the existing `min_signal` and `stop_pitch` thresholds.
    #[inline]
    pub fn moving(&self) -> bool {
        self.signal_level() >= self.cfg.min_signal && self.pitch().abs() >= self.cfg.stop_pitch
    }

    /// Current sub-bit interpolated absolute position (bits), or `None` if not
    /// locked. Updated every sample for smooth scrubbing between crossings.
    pub fn fine_position(&self) -> Option<f64> {
        if self.have_anchor && self.fine_pos.is_finite() {
            Some(self.fine_pos)
        } else {
            None
        }
    }

    #[inline]
    fn wrap(&self, v: i64) -> i64 {
        ((v % self.period) + self.period) % self.period
    }

    #[inline]
    fn wrap_f(&self, v: f64) -> f64 {
        let p = self.period as f64;
        ((v % p) + p) % p
    }

    /// Feed one stereo frame; returns a [`DecodeState`] when a bit is resolved
    /// (roughly once per carrier cycle).
    pub fn push_frame(&mut self, l: f32, r: f32) -> Option<DecodeState> {
        let ev = self.frontend.push(l, r);

        // Sub-bit fine position: extrapolate from the last locked anchor using
        // accumulated carrier phase. Updated every sample, even between crossings.
        if self.have_anchor {
            let dbits = (self.frontend.phase_total() - self.anchor_phase) / (2.0 * PI);
            self.fine_pos = self.wrap_f(self.anchor_pos as f64 + dbits);
        }

        let ev = ev?;

        // Signal-presence gate: below the floor the input is noise (e.g. the
        // pre-onset lead-in), where bit slices are random and any lock would be
        // spurious. Don't let noise accumulate toward a lock.
        let strong = ev.signal >= self.cfg.min_signal;
        if strong {
            self.warmup = (self.warmup + 1).min(self.cfg.warmup_bits);
        } else {
            self.lock_count = 0;
            self.warmup = 0;
        }

        let pitch = ev.pitch;
        let direction = if pitch.abs() < self.cfg.stop_pitch {
            Direction::Stopped
        } else if pitch > 0.0 {
            Direction::Forward
        } else {
            Direction::Reverse
        };

        // Maintain the rolling window in time order.
        self.bits.push_back(ev.bit);
        while self.bits.len() > self.window_bits {
            self.bits.pop_front();
        }
        if self.bits.len() < self.window_bits {
            return Some(DecodeState {
                position_bits: self.last_pos.map(|p| p as f64).unwrap_or(f64::NAN),
                position_seconds: f64::NAN,
                pitch,
                direction,
                locked: false,
                confidence: ev.confidence,
                signal: ev.signal,
                fine_position: self.fine_pos,
                sample: ev.sample,
            });
        }

        // Pack the window into an LFSR state. Forward: oldest bit is the LSB and
        // the lookup gives the oldest bit's index (+window-1 for the newest).
        // Reverse: the time-reversed window is the natural forward order, and the
        // lookup gives the newest bit's index directly.
        let looked = match direction {
            Direction::Reverse => {
                let rev: Vec<u8> = self.bits.iter().rev().copied().collect();
                self.map
                    .position_of_state(pack_state(&rev))
                    .map(|k| k as i64)
            }
            _ => {
                let win: Vec<u8> = self.bits.iter().copied().collect();
                self.map
                    .position_of_state(pack_state(&win))
                    .map(|k| self.wrap(k as i64 + self.window_bits as i64 - 1))
            }
        };

        // Predicted newest position from the previous result.
        let step: i64 = match direction {
            Direction::Forward => 1,
            Direction::Reverse => -1,
            Direction::Stopped => 0,
        };
        let predicted = self.last_pos.map(|p| self.wrap(p + step));

        let pos = match (looked, predicted) {
            (Some(l), Some(p)) => {
                if l == p {
                    self.lock_count = (self.lock_count + 1).min(self.cfg.lock_threshold * 2);
                    self.disagree_count = 0;
                    l
                } else {
                    // Bit error (or a real jump/scratch). Hold the prediction for
                    // a few cycles, then trust the lookup and re-sync.
                    self.lock_count = 0;
                    self.disagree_count += 1;
                    if self.disagree_count >= self.cfg.resync_after {
                        self.disagree_count = 0;
                        l
                    } else {
                        p
                    }
                }
            }
            (Some(l), None) => {
                // First fix.
                self.lock_count = 0;
                l
            }
            (None, Some(p)) => {
                // Unresolvable window (e.g. all-zero from silence); coast.
                self.lock_count = 0;
                p
            }
            (None, None) => {
                return Some(DecodeState {
                    position_bits: f64::NAN,
                    position_seconds: f64::NAN,
                    pitch,
                    direction,
                    locked: false,
                    confidence: ev.confidence,
                    signal: ev.signal,
                    fine_position: self.fine_pos,
                    sample: ev.sample,
                });
            }
        };

        self.last_pos = Some(pos);
        let locked =
            strong && self.warmup >= self.cfg.warmup_bits && self.lock_count >= self.cfg.lock_threshold;

        // Re-anchor the sub-bit interpolator to each locked cycle so it stays
        // exact and drift is bounded to a single bit between events.
        if locked {
            self.anchor_pos = pos;
            self.anchor_phase = self.frontend.phase_total();
            self.have_anchor = true;
            self.fine_pos = pos as f64;
        }

        Some(DecodeState {
            position_bits: pos as f64,
            position_seconds: pos as f64 / self.fmt.carrier_hz as f64,
            pitch,
            direction,
            locked,
            confidence: ev.confidence,
            signal: ev.signal,
            fine_position: if locked { pos as f64 } else { self.fine_pos },
            sample: ev.sample,
        })
    }

    /// Feed a block of stereo frames, returning one [`DecodeState`] per resolved
    /// carrier cycle.
    pub fn process(&mut self, frames: &[(f32, f32)]) -> Vec<DecodeState> {
        let mut out = Vec::new();
        for &(l, r) in frames {
            if let Some(s) = self.push_frame(l, r) {
                out.push(s);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::serato_cv02;
    use crate::synth::Encoder;

    fn render(start_bits: f64, pitch: f64, n: usize) -> (Vec<(f32, f32)>, TimecodeFormat, f32) {
        let fmt = serato_cv02();
        let sr = 44_100.0;
        let mut enc = Encoder::new(fmt.clone(), sr);
        enc.seek_bits(start_bits);
        let mut buf = Vec::new();
        enc.render_const(pitch, n, &mut buf);
        (buf, fmt, sr)
    }

    #[test]
    fn locks_and_reports_absolute_position_forward() {
        let start = 12_345.0;
        let (buf, fmt, sr) = render(start, 1.0, 60_000);
        let mut dec = Decoder::new(fmt, sr);
        let states = dec.process(&buf);
        let locked: Vec<_> = states.iter().filter(|s| s.locked).collect();
        assert!(!locked.is_empty(), "never locked");
        let first = locked[0];
        // First locked position should be near start + number of cycles elapsed.
        // Just assert it's a sane absolute value and advances forward.
        let last = locked.last().unwrap();
        assert_eq!(first.direction, Direction::Forward);
        assert!(last.position_bits > first.position_bits);
        // Reported absolute position should match ground truth for the newest bit.
        // The newest cycle at lock ~ start + (#events so far). Check monotonic +1.
        for w in locked.windows(2) {
            let d = w[1].position_bits - w[0].position_bits;
            assert!((d - 1.0).abs() < 1e-9 || d == 0.0, "delta={d}");
        }
    }

    #[test]
    fn absolute_position_matches_ground_truth() {
        // Render, then confirm a locked sample's position equals the encoder's
        // cycle index at that sample.
        let start = 500_000.0;
        let fmt = serato_cv02();
        let sr = 44_100.0;
        let mut enc = Encoder::new(fmt.clone(), sr);
        enc.seek_bits(start);
        let mut buf = Vec::new();
        // record ground-truth cycle index per frame
        let n = 40_000;
        let mut truth = Vec::with_capacity(n);
        for _ in 0..n {
            let cyc = enc.position_bits().floor();
            let f = enc.render_frame(1.0);
            buf.push(f);
            truth.push(cyc);
        }
        let mut dec = Decoder::new(fmt, sr);
        let mut checked = 0;
        for &(l, r) in &buf {
            if let Some(s) = dec.push_frame(l, r) {
                if s.locked {
                    // s.sample is the frame index the bit was sliced at.
                    let gt = truth[s.sample as usize];
                    // position_bits is the newest resolved cycle; allow small lag.
                    let diff = (s.position_bits - gt).abs();
                    assert!(diff <= 1.5, "decoded {} vs truth {}", s.position_bits, gt);
                    checked += 1;
                }
            }
        }
        assert!(checked > 100, "not enough locked samples: {checked}");
    }

    #[test]
    fn poll_pitch_matches_last_event() {
        // Polling `pitch()` between events returns the same value carried on the
        // most recent `DecodeState` (they read the same front end).
        let (buf, fmt, sr) = render(12_345.0, 1.0, 60_000);
        let mut dec = Decoder::new(fmt, sr);
        let mut checked = 0;
        for &(l, r) in &buf {
            if let Some(s) = dec.push_frame(l, r) {
                assert!((dec.pitch() - s.pitch).abs() < 1e-6, "poll {} vs event {}", dec.pitch(), s.pitch);
                assert!((dec.signal_level() - s.signal).abs() < 1e-6);
                checked += 1;
            }
        }
        assert!(checked > 100, "not enough events: {checked}");
    }

    #[test]
    fn pitch_follows_reverse_through_lock_state() {
        // Signed pitch tracks a reverse trajectory regardless of position lock.
        let (buf, fmt, sr) = render(500_000.0, -1.0, 60_000);
        let mut dec = Decoder::new(fmt, sr);
        for &(l, r) in &buf {
            dec.push_frame(l, r);
        }
        assert!(dec.pitch() < 0.0, "expected reverse pitch, got {}", dec.pitch());
        assert!(dec.moving(), "reverse at nominal speed should be moving");
    }

    #[test]
    fn stop_detection_after_silence() {
        // After the carrier goes silent, `moving()` becomes false: the front end's
        // signed pitch (updated every sample) decays to ~0 so the `stop_pitch`
        // gate trips, and the per-sample peak-envelope `signal_level()` decays
        // below `min_signal`. `moving()` must not report a stale "moving" state.
        let (buf, fmt, sr) = render(12_345.0, 1.0, 20_000);
        let mut dec = Decoder::new(fmt, sr);
        for &(l, r) in &buf {
            dec.push_frame(l, r);
        }
        assert!(dec.moving(), "should be moving while carrier present");
        // Feed silence; there are no crossings, so pitch relaxes toward zero.
        for _ in 0..40_000 {
            dec.push_frame(0.0, 0.0);
        }
        // Pitch relaxes to rest (well below any sane stop_pitch deadband); assert
        // near-zero rather than pinning to the exact config threshold.
        assert!(
            dec.pitch().abs() < 1e-3,
            "pitch {} did not relax toward rest",
            dec.pitch()
        );
        assert!(!dec.moving(), "silent carrier must not report as moving");
    }

    #[test]
    fn stylus_lift_drops_signal_level() {
        // A stylus lift removes the carrier entirely. Because bits are only sliced
        // at carrier crossings, no `DecodeState` is emitted once the signal stops —
        // so a host must be able to detect the lift by polling `signal_level()`.
        // Regression: the level used to be a bi-level follower updated only at
        // crossings, so it froze at its last value on silence and the lift was
        // never observable. It must now decay below `min_signal`.
        let (buf, fmt, sr) = render(12_345.0, 1.0, 20_000);
        let mut dec = Decoder::new(fmt, sr);
        for &(l, r) in &buf {
            dec.push_frame(l, r);
        }
        assert!(
            dec.signal_level() >= dec.min_signal(),
            "carrier present should read as signal, got {}",
            dec.signal_level()
        );
        // Lift the stylus: pure silence, no crossings.
        for _ in 0..20_000 {
            dec.push_frame(0.0, 0.0);
        }
        assert!(
            dec.signal_level() < dec.min_signal(),
            "signal_level {} did not decay below min_signal {} after stylus lift",
            dec.signal_level(),
            dec.min_signal()
        );
    }
}
