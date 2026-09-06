//! Pluggable timecode format descriptor.
//!
//! A [`TimecodeFormat`] captures everything the decoder (and the synthetic
//! encoder) needs to know about a particular control tone. Serato CV02 is
//! provided via [`serato_cv02`]; other DVS formats (Traktor, Final Scratch) can
//! be added as further constructors without touching the decoder.
//!
//! ## On the Serato parameters
//!
//! The carrier frequency (~1 kHz), quadrature stereo layout, amplitude-modulated
//! bits and 20-bit maximal-length LFSR are public facts about the format. The
//! *exact* LFSR taps, seed and amplitude thresholds are intended to be confirmed
//! empirically from a recording via the `analyze` example (Berlekamp–Massey);
//! the values here are a plausible working model good enough for the synthetic
//! encoder/decoder round-trip, and are clearly marked as provisional.

use crate::lfsr::Lfsr;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use crate::math::FloatExt;

/// Which stereo channel leads (by +90°) during forward playback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeadChannel {
    /// Left leads right during forward playback.
    Left,
    /// Right leads left during forward playback.
    Right,
}

/// A complete description of a control-tone format.
#[derive(Clone, Debug)]
pub struct TimecodeFormat {
    /// Human-readable name.
    pub name: &'static str,
    /// Carrier frequency in Hz at nominal (1.0×) playback speed. One bit is
    /// carried per carrier cycle, so this is also the nominal bit rate.
    pub carrier_hz: f32,
    /// LFSR geometry used for the position code.
    pub lfsr: Lfsr,
    /// LFSR seed: the register state at absolute position 0.
    pub seed: u32,
    /// Peak amplitude used to encode a `1` bit (linear, relative to full scale).
    pub amp_high: f32,
    /// Peak amplitude used to encode a `0` bit (linear, relative to full scale).
    pub amp_low: f32,
    /// Which channel leads during forward playback.
    pub lead: LeadChannel,
    /// Length, in bits (carrier cycles), of the coded **lead-in groove** that
    /// precedes the program origin (`seed` / position 0). The lead-in carries the
    /// same LFSR timecode but physically comes *before* the program, so the
    /// decoder reports it as **negative** positions: any resolved LFSR state whose
    /// raw cyclic index falls in the top `lead_in_bits` of the sequence (i.e.
    /// `>= 2^bits - 1 - lead_in_bits`) is reported as `raw - period`, placing the
    /// lead-in in `[-lead_in_bits, 0)`. Set to `0` for formats with no coded
    /// lead-in (or when it hasn't been measured), disabling the fold.
    pub lead_in_bits: u32,
    /// `true` if the parameters are confirmed against a real recording; `false`
    /// while still a provisional working model.
    pub confirmed: bool,
}

impl TimecodeFormat {
    /// Slice threshold (linear amplitude) separating `0` from `1`, expressed as a
    /// fraction of the tracked peak envelope. Uses the geometric mean of the two
    /// levels, which is robust to overall level scaling.
    #[inline]
    pub fn slice_ratio(&self) -> f32 {
        // envelope tracks the high peaks (~amp_high), so normalise by amp_high.
        (self.amp_high * self.amp_low).sqrt() / self.amp_high
    }

    /// Number of consecutive bits needed to uniquely resolve absolute position.
    /// For a maximal-length LFSR this is exactly the register width.
    #[inline]
    pub fn window_bits(&self) -> usize {
        self.lfsr.bits as usize
    }
}

// Bit peak levels approximating the documented −6 dB (`1`) / −9 dB (`0`) peaks.
// The slicer is adaptive, so only their ratio matters.
fn serato_amps() -> (f32, f32) {
    (10f32.powf(-6.0 / 20.0), 10f32.powf(-9.0 / 20.0)) // ~0.501, ~0.355
}

/// Serato Scratch Live "CV02", **side A** — parameters measured from a recording.
///
/// * 1 kHz carrier, quadrature stereo, **right channel leads** on forward play.
/// * 20-bit maximal-length LFSR, taps `0x361e5` (recovered via Berlekamp–Massey
///   from `serato-cv02-side-a.wav`; modal across 93% of windows).
///
/// The two sides use **different** LFSR polynomials (so software can tell them
/// apart) — see [`serato_cv02_side_b`]. `seed` is the register state at the first
/// **clean program-LFSR bit** of `serato-cv02-side-a-start.wav` (a recording that
/// begins right after the lead-in groove), so **position 0 = start of the program
/// groove**. The recording opens with a needle-landing transient / lead-in tone
/// (a near-constant AM pattern, not LFSR), so calibration anchors the origin at
/// the first bit where the program LFSR becomes self-consistent — rather than
/// stepping the register back through the noisy landing, which could miscount
/// carrier cycles. See `examples/calibrate.rs`.
///
/// Note: the CV carries timecode *before* this origin too, in the **lead-in
/// groove**; the decoder reports it as **negative** positions (see
/// [`TimecodeFormat::lead_in_bits`]). Earlier origins (`0x5e3e0`, then `0x2ebb`)
/// were derived from a needle-drop onset / the noisy first bit and are superseded.
pub fn serato_cv02_side_a() -> TimecodeFormat {
    let (amp_high, amp_low) = serato_amps();
    TimecodeFormat {
        name: "Serato CV02 side A (measured)",
        carrier_hz: 1000.0,
        lfsr: Lfsr::new(20, 0x361e5),
        seed: 0xafd8e,
        amp_high,
        amp_low,
        lead: LeadChannel::Right,
        // Lead-in decodes as negative positions down to ≈ −4783 (deepest lock in
        // `serato-cv02-side-a.wav`, whose needle-drop lands in the lead-in); a
        // small margin puts the fold threshold just deeper. The program runs to
        // ≈ +807k, ~236k bits below the threshold (period−4875), so no program
        // position is ever folded.
        lead_in_bits: 4875,
        confirmed: true,
    }
}

/// Serato Scratch Live "CV02", **side B** — parameters measured from a recording.
///
/// Same carrier/quadrature/geometry as [`serato_cv02_side_a`] but a distinct
/// 20-bit maximal-length LFSR, taps `0x4f0d9` (recovered from
/// `serato-cv02-side-b.wav`; modal across 97% of windows). `seed` is the state at
/// the first clean program-LFSR bit of `serato-cv02-side-b-start.wav` (position 0
/// = start of the program groove, after the lead-in), calibrated the same way as
/// side A (anchored past the needle-landing transient). See `examples/calibrate.rs`.
pub fn serato_cv02_side_b() -> TimecodeFormat {
    let (amp_high, amp_low) = serato_amps();
    TimecodeFormat {
        name: "Serato CV02 side B (measured)",
        carrier_hz: 1000.0,
        lfsr: Lfsr::new(20, 0x4f0d9),
        seed: 0x9a9a2,
        amp_high,
        amp_low,
        lead: LeadChannel::Right,
        // Lead-in decodes as negative positions down to ≈ −6051 (deepest lock in
        // `serato-cv02-side-b.wav`); a small margin puts the fold threshold just
        // deeper. The program runs to ≈ +913k, then a ~129k-bit unused gap
        // precedes the lead-in start (period−6175) — no program position is folded.
        lead_in_bits: 6175,
        confirmed: true,
    }
}

/// Default Serato CV02 format ([`serato_cv02_side_a`]). Prefer the explicit
/// per-side constructors when you know which side is playing.
pub fn serato_cv02() -> TimecodeFormat {
    serato_cv02_side_a()
}

/// Which pressed side of a Serato CV02 record is playing.
///
/// The two sides use distinct LFSR polynomials so software can tell them apart;
/// the [`Decoder`](crate::Decoder) infers this from the audio and reports it on
/// [`DecodeState::side`](crate::DecodeState::side).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    A,
    B,
}

impl Side {
    /// The measured [`TimecodeFormat`] for this side.
    pub fn format(self) -> TimecodeFormat {
        match self {
            Side::A => serato_cv02_side_a(),
            Side::B => serato_cv02_side_b(),
        }
    }

    /// Short human-readable label (`"A"` / `"B"`).
    pub fn label(self) -> &'static str {
        match self {
            Side::A => "A",
            Side::B => "B",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serato_lfsr_is_maximal() {
        let f = serato_cv02();
        assert!(f.lfsr.is_maximal_length(f.seed));
        assert_eq!(f.window_bits(), 20);
    }

    #[test]
    fn slice_ratio_between_levels() {
        let f = serato_cv02();
        let r = f.slice_ratio();
        assert!(r > f.amp_low / f.amp_high && r < 1.0);
    }
}
