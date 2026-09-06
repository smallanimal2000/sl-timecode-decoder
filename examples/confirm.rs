//! Confirm recovered parameters against a real recording:
//!  1. Slice bits (using the measured lead channel).
//!  2. Run Berlekamp–Massey over many windows and report the modal LFSR (and the
//!     fraction of error-free windows).
//!  3. Build a decoder from the recovered LFSR and check it locks and tracks
//!     position monotonically over the segment.
//!
//! Run: `cargo run --example confirm --features "analysis wav synth" -- side.wav`

use std::collections::HashMap;

use sl_timecode_decoder::analysis;
use sl_timecode_decoder::dsp::Frontend;
use sl_timecode_decoder::format::{serato_cv02, LeadChannel, TimecodeFormat};
use sl_timecode_decoder::lfsr::{pack_state, Lfsr};
use sl_timecode_decoder::Decoder;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: confirm <wav> [skip_secs] [take_secs]");
    let skip_s: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(15);
    let take_s: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(40);
    let skip = skip_s as u64 * 44_100;
    let take = take_s as usize * 44_100;
    let (frames, sr) =
        sl_timecode_decoder::wav::read_stereo_segment(&path, skip, take).expect("read wav");
    println!(
        "recording       : {path} (segment @{skip_s}s +{take_s}s, {} frames)",
        frames.len()
    );

    // --- carrier + lead ---
    let f0 = analysis::carrier_frequency(&frames, sr);
    let phase = analysis::channel_phase_offset(&frames, sr, f0);
    let lead = if phase > 0.0 { LeadChannel::Right } else { LeadChannel::Left };
    println!("carrier / lead  : {f0:.1} Hz, {lead:?} (phase {:.1}°)", phase.to_degrees());

    // --- slice bits ---
    let mut base = serato_cv02();
    base.lead = lead;
    let mut fe = Frontend::new(&base, sr);
    let mut bits = Vec::new();
    for &(l, r) in &frames {
        if let Some(ev) = fe.push(l, r) {
            bits.push(ev.bit);
        }
    }
    let bits = &bits[64.min(bits.len())..]; // drop envelope settle
    println!("sliced bits     : {}", bits.len());

    // --- Berlekamp–Massey over many windows; tally results ---
    let win = 300usize;
    let step = 250usize;
    let mut tally: HashMap<(usize, u32), usize> = HashMap::new();
    let mut windows = 0usize;
    let mut i = 0;
    while i + win <= bits.len() {
        let (l, c) = analysis::berlekamp_massey(&bits[i..i + win]);
        let taps = analysis::connection_to_lfsr(l, &c).taps;
        *tally.entry((l, taps)).or_insert(0) += 1;
        windows += 1;
        i += step;
    }
    let (&(best_l, best_taps), &best_n) = tally.iter().max_by_key(|(_, n)| **n).unwrap();
    let clean_frac = best_n as f64 / windows as f64;
    println!(
        "BM modal LFSR   : L={best_l}, taps={best_taps:#x}  ({best_n}/{windows} windows = {:.1}% clean)",
        clean_frac * 100.0
    );
    let recovered = Lfsr::new(best_l as u32, best_taps);
    println!("maximal-length  : {}", recovered.is_maximal_length(1));

    if best_l != 20 || !recovered.is_maximal_length(1) {
        println!("!! recovered LFSR is not a clean maximal-length 20-bit register");
        return;
    }

    // --- seed: state at the first clean bit window of the segment ---
    let seed = pack_state(&bits[0..best_l]);
    println!("seed @segment   : {seed:#x} (state at first sliced bit)");

    // --- build a format and run the full decoder ---
    let fmt = TimecodeFormat {
        name: "Serato CV02 (measured)",
        carrier_hz: f0,
        lfsr: recovered,
        seed,
        amp_high: base.amp_high,
        amp_low: base.amp_low,
        lead,
        lead_in_bits: 0, // unknown for an ad-hoc measured segment; no fold
        confirmed: true,
    };
    let mut dec = Decoder::new(fmt, sr);
    let states = dec.process(&frames);
    let locked: Vec<_> = states.iter().filter(|s| s.locked).collect();
    let lock_frac = locked.len() as f64 / states.len().max(1) as f64;

    // Position should advance by +1 per event once locked; count large jumps.
    let mut jumps = 0usize;
    let mut fwd = 0usize;
    for w in locked.windows(2) {
        let d = w[1].position_bits - w[0].position_bits;
        if d.abs() > 5.0 {
            jumps += 1;
        }
        if (d - 1.0).abs() < 1e-6 {
            fwd += 1;
        }
    }
    println!(
        "decoder lock    : {}/{} states locked ({:.1}%)",
        locked.len(),
        states.len(),
        lock_frac * 100.0
    );
    println!(
        "position track  : {fwd}/{} consecutive +1 steps, {jumps} jumps>5 bits",
        locked.len().saturating_sub(1)
    );
    if let (Some(first), Some(last)) = (locked.first(), locked.last()) {
        println!(
            "position range  : {:.0} .. {:.0} bits ({:.1}s .. {:.1}s of groove)",
            first.position_bits,
            last.position_bits,
            first.position_seconds,
            last.position_seconds,
        );
        println!("pitch (last)    : {:.4}", last.pitch);
    }
}
