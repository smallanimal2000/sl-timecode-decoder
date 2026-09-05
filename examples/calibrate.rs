//! Calibrate the absolute-position origin (seed) from a start-to-end recording.
//!
//! Position 0 is defined as the first carrier cycle of the pressed timecode. We:
//!  1. detect tone onset (sustained carrier after the lead-in),
//!  2. slice bits from onset and find a clean 20-bit LFSR anchor a little later,
//!  3. step the register **backward** to the onset cycle — exact even if the
//!     first few bits were misread while the envelope settled,
//!  4. verify the seed reproduces the observed bits and that the decoder reports
//!     position ~0 at onset.
//!
//! Run: `cargo run --example calibrate --features "analysis wav synth" -- side.wav`

use sl_timecode_decoder::dsp::Frontend;
use sl_timecode_decoder::format::{serato_cv02_side_a, serato_cv02_side_b, TimecodeFormat};
use sl_timecode_decoder::lfsr::{pack_state, Lfsr, PositionMap};
use sl_timecode_decoder::Decoder;

/// Detect, to ~sample precision, where the carrier tone begins.
///
/// Two stages: (1) a coarse 10 ms-RMS scan finds a point safely inside the tone
/// (first window sustained above 30% of peak RMS for 50 ms); (2) walk backward
/// from there to the silence→tone edge (last sample below 10% of peak amplitude).
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
    let sustain = 5; // 50 ms
    let mut coarse = 0usize;
    for w in 0..rms.len().saturating_sub(sustain) {
        if rms[w..w + sustain].iter().all(|&v| v > thr) {
            coarse = w * win;
            break;
        }
    }
    // Fine edge: walk back to the last sample below 10% of peak amplitude.
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

/// Try to recover the seed (state at the first sliced bit) for a given format.
/// Returns `(seed, anchor_index, mismatches)` on success.
fn recover_seed(bits: &[u8], lfsr: Lfsr) -> Option<(u32, usize, usize)> {
    let n = lfsr.bits as usize;
    // Find a clean anchor: a window that the LFSR reproduces forward.
    for m in 64..bits.len().saturating_sub(n + 40) {
        let state_m = pack_state(&bits[m..m + n]);
        if state_m == 0 {
            continue;
        }
        // state_m emits bits[m], bits[m+1], … ; advance past the known window,
        // then check the recurrence predicts the following 40 bits.
        let mut s = state_m;
        for _ in 0..n {
            lfsr.step(&mut s);
        }
        let mut mism = 0;
        for k in 0..40 {
            let b = lfsr.step(&mut s);
            if b as u8 != bits[m + n + k] {
                mism += 1;
            }
        }
        if mism <= 1 {
            // Step back m times to the first sliced bit.
            let mut s0 = state_m;
            let mut ok = true;
            for _ in 0..m {
                match lfsr.step_back(s0) {
                    Some(p) => s0 = p,
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok && s0 != 0 {
                return Some((s0, m, mism));
            }
        }
    }
    None
}

fn calibrate(path: &str, fmt_a: TimecodeFormat, fmt_b: TimecodeFormat) {
    // Read the first 60 s from the very start of the file.
    let (frames, sr) =
        sl_timecode_decoder::wav::read_stereo_segment(path, 0, 60 * 44_100).expect("read wav");
    let onset = detect_onset(&frames);
    println!(
        "{path}: onset @ sample {onset} ({:.3}s)",
        onset as f32 / sr
    );

    // Slice bits from onset using each side's known lead channel (both Right).
    let mut fe = Frontend::new(&fmt_a, sr);
    let mut bits = Vec::new();
    for &(l, r) in &frames[onset..] {
        if let Some(ev) = fe.push(l, r) {
            bits.push(ev.bit);
        }
    }

    // Try both side polynomials; use whichever gives a clean anchor.
    let candidates = [("A", &fmt_a), ("B", &fmt_b)];
    let mut chosen = None;
    for (name, fmt) in candidates {
        if let Some((seed, m, mism)) = recover_seed(&bits, fmt.lfsr) {
            println!(
                "  matches side {name}: taps={:#x}, seed={seed:#x} (anchor @bit {m}, {mism} mismatch)",
                fmt.lfsr.taps
            );
            chosen = Some((name, fmt.clone(), seed, m));
            break;
        }
    }

    let Some((name, fmt, seed, anchor_m)) = chosen else {
        println!("  could not recover a clean seed");
        return;
    };

    // Verify against the CLEAN anchor (the first sliced bits are unreliable while
    // the envelope settles, which is why we stepped back from the anchor). Under
    // the recovered seed, the anchor's state must sit at position `anchor_m`.
    let pm = PositionMap::build(fmt.lfsr, seed);
    let n = fmt.lfsr.bits as usize;
    let pos_anchor = pm.position_of_state(pack_state(&bits[anchor_m..anchor_m + n]));
    println!("  anchor position under seed: {pos_anchor:?} (expect Some({anchor_m}))");

    // Run the decoder from onset with the calibrated seed and report position.
    let mut cal = fmt.clone();
    cal.seed = seed;
    let mut dec = Decoder::new(cal, sr);
    let states = dec.process(&frames[onset..]);
    let first_locked = states.iter().find(|s| s.locked);
    let last_locked = states.iter().rev().find(|s| s.locked);
    if let (Some(f), Some(l)) = (first_locked, last_locked) {
        println!(
            "  decoder: first-lock pos={:.0} bits (@{:.2}s of groove), last pos={:.0} bits ({:.1}s), pitch~{:.3}",
            f.position_bits, f.position_seconds, l.position_bits, l.position_seconds, l.pitch
        );
    }
    println!("  => side {name} seed = {seed:#x}");
}

fn main() {
    let paths: Vec<String> = std::env::args().skip(1).collect();
    let paths = if paths.is_empty() {
        vec![
            "serato-cv02-side-a.wav".to_string(),
            "serato-cv02-side-b.wav".to_string(),
        ]
    } else {
        paths
    };
    for p in &paths {
        calibrate(p, serato_cv02_side_a(), serato_cv02_side_b());
        println!();
    }
}
