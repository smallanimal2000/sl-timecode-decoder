//! Edge-case stress harness: exercise the decoder against realistic hard cases
//! and report lock %, position accuracy, false locks and re-locks.
//!
//! Scenarios: skip-during-scratch, fast reversals near zero speed, mid-play
//! dropout, heavy noise + skip, narrow dynamic range (normalized/compressed
//! bits), and DC offset.
//!
//! Run: `cargo run --release --example stress --features "synth wav"`

use sl_timecode_decoder::format::serato_cv02_side_a;
use sl_timecode_decoder::synth::MotionSynth;
use sl_timecode_decoder::{Decoder, TimecodeFormat};

const SR: f32 = 44_100.0;

fn main() {
    println!(
        "{:<24} {:>6} {:>8} {:>8} {:>9} {:>8}",
        "scenario", "lock%", "acc%", "maxerr", "falselk", "relocks"
    );

    // 1. Needle skip in the middle of a scratch.
    let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 200_000.0);
    m.play(1.0, 0.5);
    m.scratch_sine(3.0, 4.0, 1.5);
    m.skip_with_gap(60_000.0, 0.02, 0.6);
    m.scratch_sine(3.0, 4.0, 1.5);
    m.play(1.0, 0.5);
    eval("skip_during_scratch", &m, serato_cv02_side_a());

    // 2. Very fast reversals near zero speed (rapid baby scratch).
    let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 400_000.0);
    m.play(1.0, 0.4);
    m.scratch_baby(2.5, 0.04, 30); // 40ms strokes -> reversals ~12/s
    m.play(1.0, 0.4);
    eval("fast_reversals", &m, serato_cv02_side_a());

    // 3. Mid-play signal dropout (groove keeps moving under noise).
    let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 600_000.0);
    m.play(1.0, 1.0);
    m.dropout(0.3, 1.0, 0.02);
    m.play(1.0, 1.0);
    eval("mid_play_dropout", &m, serato_cv02_side_a());

    // 4. Broadband noise sweep (fraction of tone peak), plus a skip.
    for pct in [5, 10, 15, 20, 30] {
        let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 700_000.0);
        m.play(1.0, 1.2);
        m.skip_with_gap(-80_000.0, 0.02, 0.6);
        m.play(1.0, 1.2);
        m.add_noise(pct as f32 / 100.0 * serato_cv02_side_a().amp_high, 0xBEEF);
        eval(&format!("noise_{pct}pct_plus_skip"), &m, serato_cv02_side_a());
    }

    // 5. Narrow dynamic range: encode with 1/0 peaks close together (as after
    //    normalization/compression); decode with the STANDARD format (must adapt).
    let mut narrow = serato_cv02_side_a();
    narrow.amp_high = 0.9;
    narrow.amp_low = 0.81; // ratio 0.90 (vs ~0.71 nominal)
    let mut m = MotionSynth::new(narrow, SR, 800_000.0);
    m.play(1.0, 1.5);
    m.scratch_sine(2.0, 3.0, 2.0);
    m.play(1.0, 1.0);
    eval("narrow_dynamic_range", &m, serato_cv02_side_a());

    // 5b. Extreme narrow range (ratio 0.95).
    let mut narrow = serato_cv02_side_a();
    narrow.amp_high = 0.8;
    narrow.amp_low = 0.76; // ratio 0.95
    let mut m = MotionSynth::new(narrow, SR, 850_000.0);
    m.play(1.0, 2.0);
    eval("very_narrow_range_0.95", &m, serato_cv02_side_a());

    // 6. Large per-channel DC offset added on top of the signal.
    let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 900_000.0);
    m.play(1.0, 1.0);
    m.scratch_sine(2.0, 3.0, 2.0);
    m.play(1.0, 1.0);
    m.add_dc(0.3, -0.2); // asymmetric DC on L/R
    eval("dc_offset", &m, serato_cv02_side_a());

    // 7. Speed ceiling: steady playback at increasing pitch. The carrier is
    //    1 kHz·pitch; at 44.1 kHz it approaches Nyquist (folding) around ~20×,
    //    and samples-per-cycle drops below ~3 well before that.
    for pitch in [2.0, 4.0, 8.0, 12.0, 20.0] {
        let mut m = MotionSynth::new(serato_cv02_side_a(), SR, 500_000.0);
        m.play(pitch, 1.0);
        eval(&format!("steady_{pitch:.0}x"), &m, serato_cv02_side_a());
    }

    // 8. Everything at once: narrow DR + noise + spin-up + scratch + dropout +
    //    skip + fast baby scratch + DC offset.
    let mut narrow = serato_cv02_side_a();
    narrow.amp_high = 0.9;
    narrow.amp_low = 0.81; // ratio 0.90
    let mut m = MotionSynth::new(narrow, SR, 850_000.0);
    m.ramp(0.0, 1.0, 0.3);
    m.play(1.0, 0.6);
    m.scratch_sine(2.5, 3.0, 1.5);
    m.dropout(0.2, 1.0, 0.004);
    m.skip_with_gap(50_000.0, 0.02, 0.6);
    m.scratch_baby(2.0, 0.06, 10);
    m.play(1.0, 0.8);
    m.add_dc(0.15, -0.1);
    m.add_noise(0.10 * 0.9, 0x5EED); // 10% of the (narrow) tone peak
    eval("combined_worst_case", &m, serato_cv02_side_a());
}

fn eval(name: &str, m: &MotionSynth, decode_fmt: TimecodeFormat) {
    let mut dec = Decoder::new(decode_fmt, SR);
    let (mut events, mut locked) = (0usize, 0usize);
    let (mut acc, mut lt) = (0usize, 0usize);
    let (mut max_err, mut false_locks) = (0.0f64, 0usize);
    let (mut relocks, mut was) = (0usize, false);

    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(s) = dec.push_frame(l, r) {
            events += 1;
            if s.locked {
                locked += 1;
                if !was {
                    relocks += 1;
                }
                let gt = m.truth[i];
                if gt.is_finite() {
                    lt += 1;
                    let err = (s.position_bits - gt).abs();
                    if err <= 3.0 {
                        acc += 1;
                    } else {
                        false_locks += 1;
                    }
                    if err < 1000.0 {
                        max_err = max_err.max(err);
                    }
                }
            }
            was = s.locked;
        }
    }

    println!(
        "{:<24} {:>5.1}% {:>7.1}% {:>8.1} {:>9} {:>8}",
        name,
        100.0 * locked as f64 / events.max(1) as f64,
        100.0 * acc as f64 / lt.max(1) as f64,
        max_err,
        false_locks,
        relocks,
    );
}
