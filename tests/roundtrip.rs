//! Integration tests: synthetic encoder -> decoder round-trips under varied
//! conditions, plus an (ignored) hook for a real recording.
//!
//! Run all: `cargo test --features synth`
//! Real recording: `SL_TC_WAV=/path/to/tone.wav cargo test --features "synth wav" -- --ignored`

use sl_timecode_decoder::format::serato_cv02;
use sl_timecode_decoder::synth::{Encoder, MotionSynth};
use sl_timecode_decoder::{DecodeState, Decoder, Direction, TimecodeFormat};

const SR: f32 = 44_100.0;

/// Decode a motion scenario with `fmt` and return
/// (lock fraction, accurate fraction among locked+known-truth, final error).
fn motion_metrics(m: &MotionSynth, fmt: TimecodeFormat) -> (f64, f64, f64) {
    let mut dec = Decoder::new(fmt, SR);
    let (mut events, mut locked, mut acc, mut lt) = (0usize, 0usize, 0usize, 0usize);
    let mut final_err = f64::INFINITY;
    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(s) = dec.push_frame(l, r) {
            events += 1;
            if s.locked && m.truth[i].is_finite() {
                locked += 1;
                lt += 1;
                let e = (s.position_bits - m.truth[i]).abs();
                if e <= 3.0 {
                    acc += 1;
                }
                final_err = e;
            } else if s.locked {
                locked += 1;
            }
        }
    }
    (
        locked as f64 / events.max(1) as f64,
        acc as f64 / lt.max(1) as f64,
        final_err,
    )
}

/// Decode a rendered buffer, returning the decoded states alongside the encoder's
/// ground-truth cycle index at each frame.
fn decode_with_truth(buf: &[(f32, f32)], truth: &[f64]) -> Vec<(DecodeState, f64)> {
    let mut dec = Decoder::new(serato_cv02(), SR);
    let mut out = Vec::new();
    for &(l, r) in buf {
        if let Some(s) = dec.push_frame(l, r) {
            let gt = truth[s.sample as usize];
            out.push((s, gt));
        }
    }
    out
}

fn render_const(start: f64, pitch: f64, n: usize) -> (Vec<(f32, f32)>, Vec<f64>) {
    let mut enc = Encoder::new(serato_cv02(), SR);
    enc.seek_bits(start);
    let (mut buf, mut truth) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for _ in 0..n {
        truth.push(enc.position_bits());
        buf.push(enc.render_frame(pitch));
    }
    (buf, truth)
}

/// Max absolute position error among locked states (ignoring rare resync spikes).
fn max_locked_error(states: &[(DecodeState, f64)]) -> (f64, usize) {
    let mut max_err = 0.0f64;
    let mut count = 0usize;
    for (s, gt) in states {
        if s.locked {
            count += 1;
            let e = (s.position_bits - gt).abs();
            if e < 50.0 {
                max_err = max_err.max(e);
            }
        }
    }
    (max_err, count)
}

#[test]
fn forward_nominal_speed() {
    let (buf, truth) = render_const(123_456.0, 1.0, 60_000);
    let states = decode_with_truth(&buf, &truth);
    let (err, locked) = max_locked_error(&states);
    assert!(locked > 500, "locked={locked}");
    assert!(err <= 1.5, "max err {err}");
    assert!(states.iter().rev().take(50).all(|(s, _)| s.direction == Direction::Forward));
}

#[test]
fn faster_and_slower_pitch() {
    for pitch in [0.5, 1.5, 2.0] {
        let (buf, truth) = render_const(10_000.0, pitch, 80_000);
        let states = decode_with_truth(&buf, &truth);
        let (err, locked) = max_locked_error(&states);
        assert!(locked > 200, "pitch {pitch}: locked={locked}");
        assert!(err <= 2.0, "pitch {pitch}: max err {err}");
    }
}

#[test]
fn reverse_playback() {
    let (buf, truth) = render_const(300_000.0, -1.0, 60_000);
    let states = decode_with_truth(&buf, &truth);
    let (err, locked) = max_locked_error(&states);
    // Direction-aware slicing keeps the reverse slice point mid-cycle, so reverse
    // now locks nearly as well as forward.
    assert!(locked > 800, "locked={locked}");
    assert!(err <= 2.0, "max err {err}");
    assert!(states.iter().rev().take(50).all(|(s, _)| s.direction == Direction::Reverse));
}

#[test]
fn robust_to_noise() {
    let (mut buf, truth) = render_const(77_777.0, 1.0, 80_000);
    // Noise at ~10% of the high-bit peak.
    let amp = 0.1 * serato_cv02().amp_high;
    Encoder::add_noise(&mut buf, amp, 0xC0FFEE);
    let states = decode_with_truth(&buf, &truth);
    let (err, locked) = max_locked_error(&states);
    assert!(locked > 300, "locked={locked}");
    assert!(err <= 3.0, "max err {err} under noise");
}

#[test]
fn variable_speed_profile() {
    let mut enc = Encoder::new(serato_cv02(), SR);
    enc.seek_bits(200_000.0);
    let n = 150_000;
    let (mut buf, mut truth) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for i in 0..n {
        let t = i as f64 / n as f64;
        let pitch = if t < 0.5 { 1.0 } else { 0.4 + 0.6 * (t * 20.0).sin().abs() };
        truth.push(enc.position_bits());
        buf.push(enc.render_frame(pitch));
    }
    let states = decode_with_truth(&buf, &truth);
    let (err, locked) = max_locked_error(&states);
    assert!(locked > 1000, "locked={locked}");
    assert!(err <= 3.0, "max err {err}");
}

#[test]
fn crosses_lfsr_wrap_continuously() {
    // Real recordings end before the LFSR period wraps (~17.5 min), so exercise
    // the wrap synthetically: start a few thousand bits before the end of the
    // period and play forward across it.
    let fmt = serato_cv02();
    let period: i64 = (1 << fmt.lfsr.bits) - 1;
    let start = (period - 3000) as f64;
    let (buf, truth) = render_const(start, 1.0, 220_000);
    let states = decode_with_truth(&buf, &truth);

    let locked: Vec<_> = states.iter().filter(|(s, _)| s.locked).map(|(s, _)| s).collect();
    assert!(locked.len() > 2000, "locked={}", locked.len());

    let mut wraps = 0usize;
    let mut discontinuities = 0usize;
    for w in locked.windows(2) {
        let (a, b) = (w[0].position_bits as i64, w[1].position_bits as i64);
        let step = ((b - a) % period + period) % period;
        let is_wrap = a > period * 9 / 10 && b < period / 10;
        if is_wrap {
            wraps += 1;
        }
        if step != 1 && step != 0 {
            discontinuities += 1;
        }
    }
    assert!(wraps >= 1, "never crossed the wrap");
    assert_eq!(discontinuities, 0, "position discontinuities across wrap");
}

#[test]
fn no_lock_on_noise_floor() {
    // Noise-floor lead-in (like a real record's pre-onset groove), then the tone
    // ramps in. The decoder must NOT lock on the noise, and must lock once the
    // real signal is present.
    let mut m = MotionSynth::new(serato_cv02(), SR, 50_000.0);
    m.silence(0.8, 0.0005); // ~ real intro level, below the -40 dBFS gate
    let silence_end = m.frames.len();
    m.ramp(0.0, 1.0, 0.3);
    m.play(1.0, 1.0);

    let mut dec = Decoder::new(serato_cv02(), SR);
    let (mut locked_in_silence, mut locked_after) = (0usize, 0usize);
    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(s) = dec.push_frame(l, r) {
            if s.locked {
                if i < silence_end {
                    locked_in_silence += 1;
                } else {
                    locked_after += 1;
                }
            }
        }
    }
    assert_eq!(locked_in_silence, 0, "locked on noise floor");
    assert!(locked_after > 100, "never locked after signal appeared");
}

#[test]
fn tracks_scratching() {
    // Spin-up, play, sharp baby scratches, smooth sine scratches, play.
    let mut m = MotionSynth::new(serato_cv02(), SR, 100_000.0);
    m.ramp(0.0, 1.0, 0.4);
    m.play(1.0, 1.0);
    m.scratch_baby(2.0, 0.18, 6);
    m.play(1.0, 0.4);
    m.scratch_sine(3.0, 4.0, 3.0);
    m.play(1.0, 1.0);

    let mut dec = Decoder::new(serato_cv02(), SR);
    let (mut locked, mut events, mut max_err) = (0usize, 0usize, 0.0f64);
    let mut last: Option<(f64, f64)> = None;
    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(s) = dec.push_frame(l, r) {
            events += 1;
            if s.locked && m.truth[i].is_finite() {
                locked += 1;
                let e = (s.position_bits - m.truth[i]).abs();
                if e < 1000.0 {
                    max_err = max_err.max(e);
                }
                last = Some((s.position_bits, m.truth[i]));
            }
        }
    }
    assert!(locked as f64 / events as f64 > 0.8, "lock {locked}/{events}");
    assert!(max_err <= 2.0, "max err {max_err} during scratch");
    let (p, gt) = last.unwrap();
    assert!((p - gt).abs() <= 2.0, "final pos {p} vs truth {gt}");
}

#[test]
fn recovers_after_needle_skip() {
    // Play, skip forward with a click, play, skip back, play. After each skip the
    // decoder must re-lock to the NEW absolute position (not coast).
    let mut m = MotionSynth::new(serato_cv02(), SR, 300_000.0);
    m.play(1.0, 1.5);
    m.skip_with_gap(45_000.0, 0.02, 0.6);
    m.play(1.0, 1.5);
    m.skip_with_gap(-120_000.0, 0.02, 0.6);
    m.play(1.0, 1.5);

    let mut dec = Decoder::new(serato_cv02(), SR);
    let (mut relocks, mut was) = (0usize, false);
    let (mut accurate, mut total_lt) = (0usize, 0usize);
    let mut last: Option<(f64, f64)> = None;
    for (i, &(l, r)) in m.frames.iter().enumerate() {
        if let Some(s) = dec.push_frame(l, r) {
            if s.locked {
                if !was {
                    relocks += 1;
                }
                if m.truth[i].is_finite() {
                    total_lt += 1;
                    // Absolute position must match truth — including the jumps.
                    // Coasting through a skip would leave this off by tens of
                    // thousands of bits.
                    if (s.position_bits - m.truth[i]).abs() <= 3.0 {
                        accurate += 1;
                    }
                    last = Some((s.position_bits, m.truth[i]));
                }
            }
            was = s.locked;
        }
    }
    let (p, gt) = last.unwrap();
    assert!((p - gt).abs() <= 2.0, "final pos {p} vs truth {gt}");
    // While locked, the decoder is at the correct absolute position almost always
    // (it unlocks during the skip transient rather than reporting a wrong lock).
    let frac = accurate as f64 / total_lt.max(1) as f64;
    assert!(frac > 0.95, "only {:.1}% of locked frames accurate", frac * 100.0);
    assert!(relocks <= 6, "too many re-locks: {relocks}");
}

#[test]
fn narrow_dynamic_range() {
    // Encode with 1/0 peaks close together (as after normalization/compression),
    // decode with the standard format — the adaptive slicer must adapt.
    for ratio in [0.90, 0.95] {
        let mut enc_fmt = serato_cv02();
        enc_fmt.amp_high = 0.8;
        enc_fmt.amp_low = 0.8 * ratio;
        let mut m = MotionSynth::new(enc_fmt, SR, 800_000.0);
        m.play(1.0, 2.0);
        let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
        assert!(lock > 0.8, "ratio {ratio}: lock {lock:.2}");
        assert!(acc > 0.98, "ratio {ratio}: acc {acc:.3}");
        assert!(final_err <= 2.0, "ratio {ratio}: final err {final_err}");
    }
}

#[test]
fn dc_offset_tolerated() {
    // A large per-channel DC bias must be absorbed by the DC blocker.
    let mut m = MotionSynth::new(serato_cv02(), SR, 900_000.0);
    m.play(1.0, 1.0);
    m.scratch_sine(2.0, 3.0, 2.0);
    m.play(1.0, 1.0);
    m.add_dc(0.3, -0.2);
    let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
    assert!(lock > 0.85, "lock {lock:.2}");
    assert!(acc > 0.98, "acc {acc:.3}");
    assert!(final_err <= 2.0, "final err {final_err}");
}

#[test]
fn recovers_after_dropout() {
    // A mid-play signal dropout (groove keeps moving under noise); the decoder
    // must re-acquire at the correct advanced position.
    let mut m = MotionSynth::new(serato_cv02(), SR, 600_000.0);
    m.play(1.0, 1.0);
    m.dropout(0.3, 1.0, 0.004); // below the signal gate
    m.play(1.0, 1.0);
    let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
    assert!(lock > 0.7, "lock {lock:.2}");
    assert!(acc > 0.98, "acc {acc:.3}");
    assert!(final_err <= 2.0, "final err {final_err}");
}

#[test]
fn tolerates_moderate_noise() {
    // Broadband noise at 12% of the tone peak (~18 dB SNR), plus a skip.
    let mut m = MotionSynth::new(serato_cv02(), SR, 700_000.0);
    m.play(1.0, 1.2);
    m.skip_with_gap(-80_000.0, 0.02, 0.6);
    m.play(1.0, 1.2);
    m.add_noise(0.12 * serato_cv02().amp_high, 0xBEEF);
    let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
    assert!(lock > 0.75, "lock {lock:.2}");
    assert!(acc > 0.97, "acc {acc:.3}");
    assert!(final_err <= 2.0, "final err {final_err}");
}

#[test]
fn sub_bit_position_is_smooth() {
    // fine_position updates every sample (not just per bit) and tracks the true
    // continuous position within ~1 bit, smoothly (sub-bit fractional values).
    let (buf, truth) = render_const(100_000.0, 1.0, 60_000);
    let mut dec = Decoder::new(serato_cv02(), SR);
    let (mut checked, mut fractional, mut max_err) = (0usize, 0usize, 0.0f64);
    for (i, &(l, r)) in buf.iter().enumerate() {
        dec.push_frame(l, r);
        if let Some(fp) = dec.fine_position() {
            let err = (fp - truth[i]).abs();
            if err < 10.0 {
                max_err = max_err.max(err);
            }
            let frac = fp.fract().abs();
            if frac > 1e-4 && (frac - 1.0).abs() > 1e-4 {
                fractional += 1;
            }
            checked += 1;
        }
    }
    assert!(checked > 20_000, "fine position rarely available: {checked}");
    assert!(fractional > 5_000, "fine position never sub-bit: {fractional}");
    assert!(max_err < 1.5, "fine position error {max_err}");
}

#[test]
fn speed_ceiling() {
    // Sustained playback tracks accurately up to ~4×; above that the carrier has
    // too few samples per cycle and lock degrades (documented limitation).
    for pitch in [2.0, 4.0] {
        let mut m = MotionSynth::new(serato_cv02(), SR, 500_000.0);
        m.play(pitch, 1.0);
        let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
        assert!(lock > 0.5, "pitch {pitch}: lock {lock:.2}");
        assert!(acc > 0.98, "pitch {pitch}: acc {acc:.3}");
        assert!(final_err <= 2.0, "pitch {pitch}: final err {final_err}");
    }
}

#[test]
fn combined_worst_case() {
    // Narrow DR + noise + spin-up + scratch + dropout + skip + fast baby scratch
    // + DC offset, all at once. Lock is modest under this pile-up, but the
    // decoder stays accurate when locked and recovers to the right position.
    let mut narrow = serato_cv02();
    narrow.amp_high = 0.9;
    narrow.amp_low = 0.81;
    let mut m = MotionSynth::new(narrow, SR, 850_000.0);
    m.ramp(0.0, 1.0, 0.3);
    m.play(1.0, 0.6);
    m.scratch_sine(2.5, 3.0, 1.5);
    m.dropout(0.2, 1.0, 0.004);
    m.skip_with_gap(50_000.0, 0.02, 0.6);
    m.scratch_baby(2.0, 0.06, 10);
    m.play(1.0, 0.8);
    m.add_dc(0.15, -0.1);
    m.add_noise(0.10 * 0.9, 0x5EED);

    let (lock, acc, final_err) = motion_metrics(&m, serato_cv02());
    assert!(lock > 0.10, "lock {lock:.2}");
    assert!(acc > 0.90, "acc {acc:.3}");
    assert!(final_err <= 2.0, "final err {final_err}");
}

/// Real-recording smoke test. Ignored by default; provide a WAV via SL_TC_WAV.
#[test]
#[ignore = "needs a real control-tone recording via SL_TC_WAV"]
#[cfg(feature = "wav")]
fn real_recording_monotonic() {
    let path = std::env::var("SL_TC_WAV").expect("set SL_TC_WAV to a recording path");
    let (frames, sr) = sl_timecode_decoder::wav::read_stereo(&path).expect("read wav");
    let period: i64 = (1 << serato_cv02().lfsr.bits) - 1;
    let mut dec = Decoder::new(serato_cv02(), sr);
    let states = dec.process(&frames);
    let locked: Vec<_> = states.iter().filter(|s| s.locked).collect();
    assert!(!locked.is_empty(), "never locked on real recording");

    // A real playthrough contains a long stretch of clean forward tracking
    // (position advancing +1 per event). The tail may hit a locked/skipping
    // groove, so assert on the longest clean run rather than a global average.
    let mut run = 0usize;
    let mut longest = 0usize;
    for w in locked.windows(2) {
        let step = ((w[1].position_bits as i64 - w[0].position_bits as i64) % period + period)
            % period;
        if step == 1 || step == 0 {
            run += 1;
            longest = longest.max(run);
        } else {
            run = 0;
        }
    }
    // ~10s of uninterrupted clean tracking at ~1kHz bit rate.
    assert!(longest > 10_000, "longest clean run only {longest} steps");
    println!(
        "locked {} / {} states, longest clean run {longest} steps on {path}",
        locked.len(),
        states.len()
    );
}
