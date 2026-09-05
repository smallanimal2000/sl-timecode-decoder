//! Render a synthetic Serato-style control tone to a WAV file. Stands in for a
//! real recording and lets you eyeball the signal.
//!
//! Run: `cargo run --example encode --features "synth wav" -- out.wav [seconds]`

use sl_timecode_decoder::format::serato_cv02;
use sl_timecode_decoder::synth::Encoder;
use sl_timecode_decoder::wav;

fn main() {
    let mut args = std::env::args().skip(1);
    let out = args.next().unwrap_or_else(|| "control_tone.wav".to_string());
    let seconds: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(5.0);

    let fmt = serato_cv02();
    let sr = 44_100.0f32;
    let n = (seconds * sr) as usize;

    let mut enc = Encoder::new(fmt, sr);
    enc.seek_bits(0.0);
    let mut buf = Vec::with_capacity(n);
    enc.render_const(1.0, n, &mut buf);

    wav::write_stereo(&out, &buf, sr).expect("write wav");
    println!("wrote {out}: {seconds}s, {n} frames @ {sr} Hz");
}
