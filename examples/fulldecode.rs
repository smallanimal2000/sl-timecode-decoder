//! Decode an entire recorded side start-to-end (streaming, flat memory) and
//! report lock quality, position continuity, and behaviour across the LFSR wrap.
//!
//! The LFSR period is 2^20-1 = 1,048,575 bits ≈ 17.48 min at 1×, so a ~37 min
//! side crosses the wrap ~twice. Position continuity is checked modulo the
//! period: each forward step should advance by exactly 1 (mod period), including
//! at the wrap where it goes period-1 -> 0.
//!
//! Run (release recommended):
//!   cargo run --release --example fulldecode --features wav -- serato-cv02-side-a.wav

use sl_timecode_decoder::format::{serato_cv02_side_a, serato_cv02_side_b};
use sl_timecode_decoder::{Decoder, Direction};

fn main() {
    let path = std::env::args().nth(1).expect("usage: fulldecode <wav>");
    let fmt = if path.contains("side-b") {
        serato_cv02_side_b()
    } else {
        serato_cv02_side_a()
    };
    println!("decoding {path} as {}", fmt.name);

    let mut reader = hound::WavReader::open(&path).expect("open wav");
    let spec = reader.spec();
    let sr = spec.sample_rate as f32;
    let ch = spec.channels as usize;
    let scale = (1i64 << (spec.bits_per_sample - 1)) as f32;
    let period: i64 = (1 << fmt.lfsr.bits) - 1;
    let carrier = fmt.carrier_hz as f64;

    let mut dec = Decoder::new(fmt, sr);

    // Streaming stats.
    let mut frames: u64 = 0;
    let mut events: u64 = 0;
    let mut locked_events: u64 = 0;
    let mut first_lock_sample: Option<u64> = None;
    let mut last_lock_sample: u64 = 0;
    let mut first_lock_pos: Option<i64> = None;
    let mut last_lock_pos: i64 = 0;
    let mut good_steps: u64 = 0; // +1 mod period while forward & locked
    let mut glitches: u64 = 0; // unexpected jumps while forward & locked
    let mut wraps: u64 = 0; // clean period-1 -> 0 transitions
    let mut wrap_glitches: u64 = 0; // discontinuities at a wrap
    let mut min_pitch = f32::INFINITY;
    let mut max_pitch = f32::NEG_INFINITY;
    let mut prev: Option<(i64, Direction, bool)> = None;

    // Sparse trajectory: print a locked state every ~30s of wall clock.
    let mark_step: u64 = 30 * sr as u64;
    let mut next_mark: u64 = 0;
    println!("--- trajectory (every ~30s) ---");
    println!("  wall_s   pos_bits   groove_s   pitch   dir");

    let mut samples = reader.samples::<i32>();
    loop {
        // Read one interleaved frame.
        let l = match samples.next() {
            Some(v) => v.expect("sample") as f32 / scale,
            None => break,
        };
        let mut r = l;
        for c in 1..ch {
            match samples.next() {
                Some(v) => {
                    let s = v.expect("sample") as f32 / scale;
                    if c == 1 {
                        r = s;
                    }
                }
                None => break,
            }
        }
        frames += 1;

        if let Some(s) = dec.push_frame(l, r) {
            events += 1;
            min_pitch = min_pitch.min(s.pitch);
            max_pitch = max_pitch.max(s.pitch);
            if s.locked {
                locked_events += 1;
                let pos = s.position_bits as i64;
                first_lock_sample.get_or_insert(s.sample);
                first_lock_pos.get_or_insert(pos);
                last_lock_sample = s.sample;
                last_lock_pos = pos;

                if s.sample >= next_mark {
                    println!(
                        "  {:>7.1} {:>10} {:>10.1} {:>7.3}  {:?}",
                        s.sample as f64 / sr as f64,
                        pos,
                        pos as f64 / carrier,
                        s.pitch,
                        s.direction,
                    );
                    next_mark = s.sample + mark_step;
                }

                if let Some((pprev, dprev, lprev)) = prev {
                    if lprev && dprev == Direction::Forward && s.direction == Direction::Forward {
                        let step = ((pos - pprev) % period + period) % period;
                        let is_wrap = pprev > (period * 9 / 10) && pos < (period / 10);
                        if step == 1 {
                            good_steps += 1;
                            if is_wrap {
                                wraps += 1;
                            }
                        } else if step == 0 {
                            // duplicate/held sample; ignore
                        } else {
                            glitches += 1;
                            if is_wrap {
                                wrap_glitches += 1;
                            }
                        }
                    }
                }
            }
            prev = Some((s.position_bits as i64, s.direction, s.locked));
        }
    }

    let dur = frames as f64 / sr as f64;
    println!("frames          : {frames}  ({dur:.1}s, {:.1} min)", dur / 60.0);
    println!("bit events      : {events}");
    println!(
        "locked events   : {locked_events} ({:.2}%)",
        100.0 * locked_events as f64 / events.max(1) as f64
    );
    if let (Some(fs), Some(fp)) = (first_lock_sample, first_lock_pos) {
        println!(
            "first lock      : sample {fs} ({:.2}s), pos {fp} bits ({:.2}s groove)",
            fs as f64 / sr as f64,
            fp as f64 / carrier
        );
        println!(
            "last lock       : sample {last_lock_sample} ({:.1}s), pos {last_lock_pos} bits ({:.1}s groove)",
            last_lock_sample as f64 / sr as f64,
            last_lock_pos as f64 / carrier
        );
    }
    println!("pitch range     : {min_pitch:.3} .. {max_pitch:.3}");
    println!("forward steps   : {good_steps} good (+1 mod period)");
    println!(
        "continuity      : {glitches} glitches ({:.4}% of forward steps)",
        100.0 * glitches as f64 / good_steps.max(1) as f64
    );
    println!("LFSR wraps      : {wraps} clean, {wrap_glitches} with a discontinuity");
    if wrap_glitches == 0 && wraps > 0 {
        println!("=> position is continuous across every LFSR wrap ✓");
    }
}
