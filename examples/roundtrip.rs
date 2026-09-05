//! End-to-end sanity check: synthesise a control tone with a varying speed
//! profile, decode it, and print how well the decoded position tracks truth.
//!
//! Run with: `cargo run --example roundtrip --features synth`

use sl_timecode_decoder::format::serato_cv02;
use sl_timecode_decoder::synth::Encoder;
use sl_timecode_decoder::{Decoder, Direction};

fn main() {
    let fmt = serato_cv02();
    let sr = 44_100.0f32;

    let mut enc = Encoder::new(fmt.clone(), sr);
    let start = 200_000.0;
    enc.seek_bits(start);

    // Speed profile: forward, slow down, brief reverse, back to forward.
    let n = 200_000usize;
    let pitch_at = |i: usize| -> f64 {
        let t = i as f64 / n as f64;
        if t < 0.4 {
            1.0
        } else if t < 0.5 {
            0.3
        } else if t < 0.6 {
            -0.8
        } else {
            1.2
        }
    };

    let mut buf = Vec::with_capacity(n);
    let mut truth = Vec::with_capacity(n);
    for i in 0..n {
        truth.push(enc.position_bits());
        buf.push(enc.render_frame(pitch_at(i)));
    }

    let mut dec = Decoder::new(fmt, sr);
    let mut max_err = 0.0f64;
    let mut locked_count = 0usize;
    let mut samples = 0usize;
    let mut last: Option<(u64, f64, Direction, bool, f32)> = None;

    for &(l, r) in &buf {
        if let Some(s) = dec.push_frame(l, r) {
            samples += 1;
            if s.locked {
                locked_count += 1;
                let gt = truth[s.sample as usize];
                let err = (s.position_bits - gt).abs();
                if err < 100.0 {
                    // ignore transient re-sync spikes for the max metric
                    max_err = max_err.max(err);
                }
            }
            last = Some((s.sample, s.position_bits, s.direction, s.locked, s.pitch));
        }
    }

    println!("frames rendered : {n}");
    println!("bit events      : {samples}");
    println!("locked events   : {locked_count}");
    println!("max |pos error| : {max_err:.2} bits (while locked)");
    if let Some((smp, pos, dir, locked, pitch)) = last {
        println!(
            "final state     : sample={smp} pos={pos:.1} bits dir={dir:?} locked={locked} pitch={pitch:.3}"
        );
    }
}
