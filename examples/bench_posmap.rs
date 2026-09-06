//! Micro-benchmark: `PositionMap` lookup throughput and build cost for the
//! `Fast` (bit-packed table) vs `Compact` (Pohlig–Hellman discrete-log) stores,
//! on the measured side-A 20-bit polynomial.
//!
//!   cargo run --release --example bench_posmap

use std::time::Instant;

use sl_timecode_decoder::lfsr::{pack_state, Lfsr, PositionMap, PositionMapKind};

fn main() {
    let lfsr = Lfsr::new(20, 0x361e5);
    let seed = 0xafd8e;
    let period = (1u64 << lfsr.bits) - 1;
    let bits = lfsr.bits as usize;

    // A pool of real windows/states to look up, spread across the sequence.
    // (Collect states directly from the walk so every lookup is a valid state.)
    let seq = lfsr.sequence(seed);
    let n_samples = 200_000usize;
    let stride = (seq.len() - bits) / n_samples;
    let states: Vec<u32> = (0..n_samples)
        .map(|i| pack_state(&seq[i * stride..i * stride + bits]))
        .collect();

    for kind in [
        PositionMapKind::Fast,
        PositionMapKind::Balanced,
        PositionMapKind::Compact,
    ] {
        let t0 = Instant::now();
        let pm = PositionMap::build_with_kind(lfsr, seed, kind);
        let build = t0.elapsed();

        // Warm + correctness spot-check, then timed loop.
        let mut acc = 0u64;
        let t1 = Instant::now();
        for &s in &states {
            acc = acc.wrapping_add(pm.position_of_state(s).unwrap() as u64);
        }
        let dt = t1.elapsed();

        let per = dt.as_nanos() as f64 / states.len() as f64;
        let thru = states.len() as f64 / dt.as_secs_f64();
        println!(
            "{kind:?}: build {:>7.2?}  |  lookup {:>7.1} ns/op  ({:.2} M lookups/s)  [acc={acc}]",
            build,
            per,
            thru / 1e6
        );
    }
    println!("(period = {period}, {n_samples} sampled lookups each)");
}
