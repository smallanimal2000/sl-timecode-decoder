//! Calibrate the absolute-position origin (seed) and lead-in length.
//!
//! Position 0 is the first **clean program-LFSR bit** — the start of the program
//! timecode that follows the record's coded **lead-in**. Two recordings per side
//! drive the calibration:
//!
//!  * `serato-cv02-side-?-start.wav` — begins right after the lead-in groove.
//!    Its first samples are a needle-landing transient / lead-in tone (a
//!    near-constant AM pattern, not LFSR), so we anchor the origin at the first
//!    bit where the program LFSR becomes self-consistent, and take the register
//!    state there as the seed. (Stepping *back* through the noisy landing to a
//!    nominal "bit 0" is avoided: the transient can add/drop carrier cycles and
//!    misplace the origin — the point of the user's note.)
//!  * `serato-cv02-side-?.wav` — the full-side reference. Its needle-drop lands in
//!    the lead-in, so under the program seed it decodes to **negative** positions;
//!    the deepest reliable lock gives the lead-in length (`lead_in_bits`).
//!
//! Run: `cargo run --example calibrate --features "wav synth"`
//! (defaults to the four repo recordings; or pass `start.wav ref.wav` pairs).

use sl_timecode_decoder::dsp::Frontend;
use sl_timecode_decoder::format::{serato_cv02_side_a, serato_cv02_side_b, TimecodeFormat};
use sl_timecode_decoder::lfsr::{pack_state, Lfsr};
use sl_timecode_decoder::Decoder;

/// Detect, to ~sample precision, where the carrier tone begins.
fn detect_onset(frames: &[(f32, f32)]) -> usize {
    let win = 441usize; // 10 ms @ 44.1 kHz
    let rms: Vec<f32> = frames
        .chunks(win)
        .map(|c| {
            let s: f32 = c.iter().map(|f| f.0 * f.0 + f.1 * f.1).sum();
            (s / (2 * c.len()) as f32).sqrt()
        })
        .collect();
    let peak_rms = rms.iter().cloned().fold(0.0f32, f32::max);
    let thr = 0.3 * peak_rms;
    let sustain = 5;
    let mut coarse = 0usize;
    for w in 0..rms.len().saturating_sub(sustain) {
        if rms[w..w + sustain].iter().all(|&v| v > thr) {
            coarse = w * win;
            break;
        }
    }
    let peak_amp = frames
        .iter()
        .map(|f| f.0.abs().max(f.1.abs()))
        .fold(0.0f32, f32::max);
    let lo = 0.1 * peak_amp;
    let mut i = coarse;
    while i > 0 && frames[i].0.abs().max(frames[i].1.abs()) > lo {
        i -= 1;
    }
    i
}

/// Slice bits from the tone onset of a recording.
fn slice_from_onset(path: &str, fmt: &TimecodeFormat, seconds: usize) -> Vec<u8> {
    let (frames, sr) =
        sl_timecode_decoder::wav::read_stereo_segment(path, 0, seconds * 44_100).expect("read wav");
    let onset = detect_onset(&frames);
    println!("  {path}: onset @ sample {onset} ({:.3}s)", onset as f32 / sr);
    let mut fe = Frontend::new(fmt, sr);
    let mut bits = Vec::new();
    for &(l, r) in &frames[onset..] {
        if let Some(ev) = fe.push(l, r) {
            bits.push(ev.bit);
        }
    }
    bits
}

/// First bit index that starts a fully-clean forward LFSR run of `horizon` bits,
/// with the register state there. Everything before it is landing noise / lead-in
/// tone. That state, taken as position 0, is the calibrated seed.
fn program_origin(bits: &[u8], lfsr: Lfsr, horizon: usize) -> Option<(usize, u32)> {
    let n = lfsr.bits as usize;
    'm: for m in 0..bits.len().saturating_sub(n + horizon) {
        let s0 = pack_state(&bits[m..m + n]);
        if s0 == 0 {
            continue;
        }
        let mut s = s0;
        for _ in 0..n {
            lfsr.step(&mut s);
        }
        for k in 0..horizon {
            if lfsr.step(&mut s) as u8 != bits[m + n + k] {
                continue 'm;
            }
        }
        return Some((m, s0));
    }
    None
}

/// Fraction of adjacent bits that differ — ~0.5 for LFSR/noise, ~0 for the
/// constant lead-in tone.
fn transition_rate(bits: &[u8]) -> f32 {
    if bits.len() < 2 {
        return 0.0;
    }
    let t = (1..bits.len()).filter(|&i| bits[i] != bits[i - 1]).count();
    t as f32 / (bits.len() - 1) as f32
}

fn calibrate_side(name: &str, start_path: &str, ref_path: &str, fmt: TimecodeFormat) {
    let period = (1i64 << fmt.lfsr.bits) - 1;
    println!("== side {name} (taps={:#x}, period={period}) ==", fmt.lfsr.taps);

    // --- program origin from the -start recording ---
    let sbits = slice_from_onset(start_path, &fmt, 30);
    let Some((m, seed)) = program_origin(&sbits, fmt.lfsr, 300) else {
        println!("  could not find a clean program origin");
        return;
    };
    println!(
        "  head[0..{m}] transition rate {:.0}% (lead-in tone ~0%, LFSR/noise ~50%)",
        transition_rate(&sbits[..m]) * 100.0
    );
    println!("  program origin (pos 0) = bit {m}, seed = {seed:#x}");
    if seed != fmt.seed {
        println!("  NOTE: differs from baked seed {:#x} — update format.rs", fmt.seed);
    }

    // --- lead-in length from the full reference (first 60 s) ---
    let (frames, sr) =
        sl_timecode_decoder::wav::read_stereo_segment(ref_path, 0, 60 * 44_100).expect("read");
    let mut dec = Decoder::new(fmt.clone(), sr);
    let states = dec.process(&frames);
    // Deepest reliable lead-in lock = most-negative reported position.
    let deepest = states
        .iter()
        .filter(|s| s.locked)
        .map(|s| s.position_bits as i64)
        .filter(|&p| p < 0 && p > -(period / 2)) // negative = lead-in; exclude mislocks
        .min();
    match deepest {
        Some(d) => {
            println!(
                "  ref lead-in: deepest lock {d} bits ({:.3}s before program)",
                (-d) as f32 / fmt.carrier_hz
            );
            // The baked value carries a small margin below the deepest lock so the
            // fold threshold sits just past the lead-in; flag only a large drift.
            let want = (-d) as u32;
            if fmt.lead_in_bits < want || fmt.lead_in_bits - want > 400 {
                println!(
                    "  NOTE: deepest lead-in lock ~{want}, baked lead_in_bits {} — check margin",
                    fmt.lead_in_bits
                );
            }
        }
        None => println!("  ref: no negative (lead-in) lock found in first 60s"),
    }

    // --- sanity: report first locks for both recordings ---
    for (label, path) in [("start", start_path), ("ref  ", ref_path)] {
        let (frames, sr) =
            sl_timecode_decoder::wav::read_stereo_segment(path, 0, 20 * 44_100).expect("read");
        let onset = detect_onset(&frames);
        let mut dec = Decoder::new(fmt.clone(), sr);
        let states = dec.process(&frames[onset..]);
        if let Some(f) = states.iter().find(|s| s.locked) {
            println!(
                "  decode {label}: first-lock pos={:+.0} bits (side {:?})",
                f.position_bits,
                f.side.map(|s| s.label())
            );
        }
    }
    println!();
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() >= 2 {
        for pair in args.chunks(2) {
            if pair.len() == 2 {
                calibrate_side("?", &pair[0], &pair[1], serato_cv02_side_a());
            }
        }
        return;
    }
    calibrate_side(
        "A",
        "serato-cv02-side-a-start.wav",
        "serato-cv02-side-a.wav",
        serato_cv02_side_a(),
    );
    calibrate_side(
        "B",
        "serato-cv02-side-b-start.wav",
        "serato-cv02-side-b.wav",
        serato_cv02_side_b(),
    );
}
