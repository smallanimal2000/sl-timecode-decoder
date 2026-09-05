//! Emulate real turntable handling (scratching + needle skips) with the motion
//! synthesizer, write the audio to WAV, and show how the decoder tracks it.
//!
//! Run: `cargo run --example scratch_demo --features "synth wav"`

use sl_timecode_decoder::format::serato_cv02_side_a;
use sl_timecode_decoder::synth::MotionSynth;
use sl_timecode_decoder::{Decoder, Direction};

fn main() {
    let fmt = serato_cv02_side_a();
    let sr = 44_100.0f32;

    // --- Scenario: spin-up, play, baby scratch, sine scratch, play ---
    let mut m = MotionSynth::new(fmt.clone(), sr, 100_000.0);
    m.ramp(0.0, 1.0, 0.4); // needle drop / spin-up
    m.play(1.0, 1.5); // steady groove
    m.scratch_baby(2.0, 0.18, 6); // 6 sharp forward/back strokes at 2x
    m.play(1.0, 0.5);
    m.scratch_sine(3.0, 4.0, 3.0); // 3 swings, ±3x, smooth reversals
    m.play(1.0, 1.5);
    sl_timecode_decoder::wav::write_stereo("scratch.wav", &m.frames, sr).unwrap();
    println!("wrote scratch.wav ({} frames)", m.frames.len());
    report("scratch", &fmt, sr, &m);

    // --- Scenario: play, needle skip (forward), play, skip (back), play ---
    let mut s = MotionSynth::new(fmt.clone(), sr, 300_000.0);
    s.play(1.0, 1.5);
    s.skip_with_gap(45_000.0, 0.02, 0.6); // skip ~45s forward with a click
    s.play(1.0, 1.5);
    s.skip_with_gap(-120_000.0, 0.02, 0.6); // skip back
    s.play(1.0, 1.5);
    sl_timecode_decoder::wav::write_stereo("skip.wav", &s.frames, sr).unwrap();
    println!("wrote skip.wav ({} frames)", s.frames.len());
    report("skip", &fmt, sr, &s);
}

fn report(name: &str, fmt: &sl_timecode_decoder::TimecodeFormat, sr: f32, m: &MotionSynth) {
    let mut dec = Decoder::new(fmt.clone(), sr);
    let mut locked = 0usize;
    let mut events = 0usize;
    let mut max_err = 0.0f64;
    let mut relocks = 0usize;
    let mut was_locked = false;

    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(st) = dec.push_frame(l, r) {
            events += 1;
            if st.locked {
                locked += 1;
                if !was_locked {
                    relocks += 1;
                }
                let gt = m.truth[i];
                if gt.is_finite() {
                    let err = (st.position_bits - gt).abs();
                    if err < 1000.0 {
                        max_err = max_err.max(err);
                    }
                }
            }
            was_locked = st.locked;
        }
    }

    // Final steady-state accuracy: compare decoder to truth at the very end.
    let mut dec2 = Decoder::new(fmt.clone(), sr);
    let mut last: Option<(f64, f64, Direction)> = None;
    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(st) = dec2.push_frame(l, r) {
            if st.locked && m.truth[i].is_finite() {
                last = Some((st.position_bits, m.truth[i], st.direction));
            }
        }
    }

    println!("--- {name} ---");
    println!(
        "  events={events} locked={locked} ({:.1}%) relocks={relocks} max|err|(locked,<1000)={max_err:.1} bits",
        100.0 * locked as f64 / events.max(1) as f64
    );
    if let Some((pos, gt, dir)) = last {
        println!(
            "  final: decoded pos={pos:.0}, truth={gt:.0}, err={:.1} bits, dir={dir:?}",
            (pos - gt).abs()
        );
    }
}
