//! Clean-room parameter derivation from a recording.
//!
//! Reads a stereo WAV of a steady-speed control-tone passage and recovers the
//! carrier frequency, the L/R quadrature relationship, and the LFSR geometry
//! (via Berlekamp–Massey) — the parameters that populate a `TimecodeFormat`.
//! With no argument it analyses a synthetic tone so the pipeline can be seen
//! end-to-end before a real recording is available.
//!
//! Run: `cargo run --example analyze --features "analysis wav synth" -- recording.wav`

use sl_timecode_decoder::analysis;
use sl_timecode_decoder::dsp::Frontend;
use sl_timecode_decoder::format::{serato_cv02, LeadChannel};

fn main() {
    let path = std::env::args().nth(1);

    let fmt = serato_cv02();
    let (frames, sr, source) = match &path {
        Some(p) => {
            // Skip ~15s (needle-drop / lead-in), analyse a ~20s steady segment.
            let skip = 15 * 44_100;
            let take = 20 * 44_100;
            let (f, sr) = sl_timecode_decoder::wav::read_stereo_segment(p, skip, take)
                .expect("read wav");
            (f, sr, format!("recording {p} (segment @15s +20s)"))
        }
        None => {
            use sl_timecode_decoder::synth::Encoder;
            let sr = 44_100.0;
            let mut enc = Encoder::new(fmt.clone(), sr);
            let mut buf = Vec::new();
            enc.render_const(1.0, 1 << 16, &mut buf);
            (buf, sr, "synthetic tone (no file given)".to_string())
        }
    };

    println!("source          : {source}");
    println!("frames          : {}  @ {sr} Hz", frames.len());

    // 1) Carrier frequency + quadrature.
    let f0 = analysis::carrier_frequency(&frames, sr);
    let phase = analysis::channel_phase_offset(&frames, sr, f0);
    let lead = if phase < 0.0 { "Left leads Right" } else { "Right leads Left" };
    println!("carrier freq    : {f0:.1} Hz");
    println!(
        "L/R phase offset: {:.1}°  ({lead}, quadrature={})",
        phase.to_degrees(),
        (phase.abs() - std::f32::consts::FRAC_PI_2).abs() < 0.4
    );

    // 2) Slice bits from a steady passage using the DSP front-end. Use the lead
    // channel measured from the phase offset (positive => right leads).
    let mut fmt = fmt;
    fmt.lead = if phase > 0.0 { LeadChannel::Right } else { LeadChannel::Left };
    println!("using lead      : {:?}", fmt.lead);
    let mut fe = Frontend::new(&fmt, sr);
    let mut bits = Vec::new();
    for &(l, r) in &frames {
        if let Some(ev) = fe.push(l, r) {
            bits.push(ev.bit);
        }
    }
    // Drop the first few while the envelope settles, take a clean run.
    let start = 64.min(bits.len());
    let sample: Vec<u8> = bits[start..].iter().take(400).copied().collect();
    println!("sliced bits     : {} (using {} for BM)", bits.len(), sample.len());

    // 3) Recover the LFSR with Berlekamp–Massey.
    if sample.len() < 64 {
        println!("not enough bits to run Berlekamp–Massey");
        return;
    }
    let (l, c) = analysis::berlekamp_massey(&sample);
    let lfsr = analysis::connection_to_lfsr(l, &c);
    println!("LFSR length     : {l} bits");
    println!("polynomial      : {}", analysis::poly_string(l, &c));
    println!("tap mask        : {:#x}", lfsr.taps);
    println!("maximal-length  : {}", lfsr.is_maximal_length(1));

    println!();
    println!("--- provenance note ---");
    println!("Parameters above were measured from {source} by this crate's own");
    println!("analysis (FFT + Goertzel + Berlekamp–Massey), not copied from any");
    println!("existing decoder. Feed a real recording to confirm the working model.");
}
