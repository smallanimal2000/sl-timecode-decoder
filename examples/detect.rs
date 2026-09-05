//! Auto-detect which pressed side (A or B) a recording is, from the audio alone.
//!
//! The two Serato CV02 sides carry identical carriers and differ only in their
//! LFSR polynomial, so the side can't be read off level or pitch — you have to
//! find which polynomial the bit stream locks to. The `Decoder` tracks both sides
//! internally and reports the winner on `DecodeState::side` (and `Decoder::side`)
//! as soon as one locks; decoding then continues on that side with no extra work.
//!
//! Run:
//!   cargo run --release --example detect --features wav -- serato-cv02-side-a.wav
//!
//! Unlike `fulldecode`, this does not look at the filename — the answer comes
//! entirely from the signal.

use sl_timecode_decoder::format::serato_cv02;
use sl_timecode_decoder::Decoder;

fn main() {
    let path = std::env::args().nth(1).expect("usage: detect <wav>");

    let mut reader = hound::WavReader::open(&path).expect("open wav");
    let spec = reader.spec();
    let sr = spec.sample_rate as f32;
    let ch = spec.channels as usize;
    let scale = (1i64 << (spec.bits_per_sample - 1)) as f32;

    // A plain decoder already discriminates both Serato sides; the format arg only
    // supplies the (per-side-identical) front-end.
    let mut dec = Decoder::new(serato_cv02(), sr);

    let mut frames: u64 = 0;
    let mut detected = None;
    let mut samples = reader.samples::<i32>();
    'outer: loop {
        let l = match samples.next() {
            Some(v) => v.expect("sample") as f32 / scale,
            None => break,
        };
        let mut r = l;
        for c in 1..ch {
            match samples.next() {
                Some(v) => {
                    if c == 1 {
                        r = v.expect("sample") as f32 / scale;
                    }
                }
                None => break 'outer,
            }
        }
        frames += 1;

        if let Some(s) = dec.push_frame(l, r) {
            if let Some(side) = s.side {
                detected = Some(side);
                println!(
                    "detected side {} after {:.2}s ({} frames), first locked pos {:.0} bits",
                    side.label(),
                    frames as f64 / sr as f64,
                    frames,
                    s.position_bits,
                );
                break;
            }
        }
    }

    match detected {
        // A real host would keep feeding `dec` the rest of the stream; it stays on
        // the detected side.
        Some(side) => println!("side {} — continue decoding (pitch {:.3})", side.label(), dec.pitch()),
        None => {
            eprintln!(
                "could not determine side from {} frames ({:.1}s) — no control tone locked",
                frames,
                frames as f64 / sr as f64
            );
            std::process::exit(1);
        }
    }
}
