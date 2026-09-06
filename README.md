# sl-timecode-decoder

A **clean-room** Rust decoder for DVS (Serato Scratch Live–style) control-tone
timecode. It reads recorded stereo audio of a control vinyl/CD and recovers the
stylus's **absolute position, pitch (speed), and direction**.

> **Clean-room:** every parameter and algorithm here was derived from public
> conceptual descriptions of the format and from empirical analysis of real
> recordings — *not* by copying any existing decoder's source (e.g. xwax, which
> is GPL). This keeps the provenance clean and lets the crate be licensed
> under MIT.

## Measured parameters (Serato CV02)

Recovered from real Serato CV02 recordings by this crate's own analysis
(`examples/confirm.rs`) and validated by decoding the same recordings (>99% lock,
near-perfect monotonic position tracking):

| parameter    | side A | side B |
|--------------|--------|--------|
| carrier      | ~1000 Hz | ~1000 Hz |
| quadrature   | 90°, **right channel leads** on forward playback | same |
| LFSR         | 20-bit, maximal-length | 20-bit, maximal-length |
| taps         | `0x361e5` | `0x4f0d9` |
| seed (origin)| `0xafd8e` | `0x9a9a2` |
| lead-in      | ≥ ~4783 bits (~4.8 s) | ≥ ~6051 bits (~6.1 s) |

The two sides use **different** LFSR polynomials so software can distinguish
them. The `seed` is the LFSR state at the **first clean program-timecode bit** of
the `serato-cv02-side-?-start.wav` recordings — which begin right after the
lead-in groove — so `position 0` = start of the program. Those recordings open
with a needle-landing transient / lead-in tone (a near-constant AM pattern, not
LFSR), so `examples/calibrate.rs` anchors the origin at the first bit where the
program LFSR becomes self-consistent, rather than stepping the register back
through the noisy landing (which could add or drop carrier cycles and misplace
the origin).

The CV also carries timecode in its **lead-in groove**, which physically
precedes the program. The decoder reports the lead-in as **negative** positions:
each side's `lead_in_bits` marks the top of the LFSR cycle, so a needle drop in
the lead-in decodes around `−4783` / `−6051` and climbs continuously through `0`
into the program. The lead-in length was measured against the full-side
references `serato-cv02-side-?.wav`, whose needle-drop onset lands in the lead-in
(the program itself reaches ≈ +807k / +913k, far below the fold threshold).

## How it works

The Serato control tone is a stereo signal:

* A steady **~1 kHz sine carrier**; its instantaneous frequency is proportional
  to playback speed (pitch).
* The two channels are in **quadrature** (~90° apart); which channel leads gives
  play direction.
* Each carrier cycle carries one **bit**, amplitude-modulated (high peak = `1`,
  low peak = `0`).
* The bit stream is a **maximal-length LFSR sequence** (20-bit register), so any
  window of 20 consecutive bits maps to a unique absolute position.

Decoding pipeline (`src/`):

| module     | role |
|------------|------|
| `format`   | `TimecodeFormat` descriptor (carrier, LFSR taps/seed, levels). Pluggable — `serato_cv02()` today; add Traktor/Final Scratch later. |
| `dsp`      | DC-block, quadrature phase tracker (pitch/direction), zero-crossing + peak **bit slicer**. |
| `lfsr`     | Fibonacci LFSR (step ±, sequence), and the **state→position** map. |
| `decoder`  | Public streaming API; rolling bit window → position, with a lock/re-sync layer. |
| `synth`    | Ground-truth **encoder** (feature `synth`) used by tests and examples. |
| `analysis` | **Berlekamp–Massey** + FFT/Goertzel to recover parameters from a recording (feature `analysis`). |
| `wav`      | Minimal WAV I/O helper (feature `wav`). |

## Usage

```rust
use sl_timecode_decoder::{Decoder, format};

let mut dec = Decoder::new(format::serato_cv02(), 44_100.0);
for state in dec.process(&stereo_frames) {
    println!(
        "{:.2}s  pos={:.0} bits  pitch={:.3}  {:?}  locked={}",
        state.position_seconds, state.position_bits, state.pitch,
        state.direction, state.locked,
    );
}
```

`DecodeState` also carries `fine_position` — a sub-bit interpolated absolute
position refined from carrier phase every sample (also via `dec.fine_position()`)
for smooth scrubbing between the ~1 kHz bit resolution. Tuning is exposed via
`DecoderConfig` / `SlicerConfig` (`Decoder::with_config`); the defaults are
validated against the real recordings and the stress suite.

### Side detection

The two sides carry identical carriers and differ only in their LFSR polynomial,
so the side can't be read off level, pitch, or direction — only from *which*
polynomial the bit stream follows. The decoder tracks **both** sides against the
same bit window and reports the winner on `DecodeState::side` (and `dec.side()`),
so a plain decoder auto-detects the side with no extra setup:

```rust
use sl_timecode_decoder::{Decoder, Side, format};

let mut dec = Decoder::new(format::serato_cv02(), 44_100.0);
for state in dec.process(&stereo_frames) {
    match state.side {
        Some(Side::A) => { /* side A */ }
        Some(Side::B) => { /* side B */ }
        None => { /* not resolved yet */ }
    }
}
```

Detection keys on the **sustained agreement rate** between each side's lookups
and its own predicted position: the correct polynomial agrees essentially every
cycle, while the wrong one — kept re-syncing to its own bogus lookups — only
agrees about half the time. Averaging over cycles separates the two cleanly. The
side latches on the first confident resolution and then persists through
dropouts.

## Examples

```sh
# End-to-end synthetic round-trip (varying speed incl. reverse):
cargo run --example roundtrip --features synth

# Render a synthetic control tone to WAV:
cargo run --example encode --features "synth wav" -- tone.wav 5

# Clean-room parameter recovery from a recording (or synthetic if no arg):
cargo run --example analyze --features "analysis wav synth" -- recording.wav

# Full confirmation: recover the LFSR over many windows AND check the decoder
# locks/tracks on the recording:
cargo run --example confirm --features "analysis wav synth" -- side.wav [skip_s] [take_s]

# Calibrate the program origin (seed) and lead-in length. With no args it uses
# the four repo recordings (per side: a -start file for the origin + the full
# file as a lead-in reference); or pass `start.wav ref.wav` pairs:
cargo run --example calibrate --features "analysis wav synth"

# Auto-detect which side (A/B) a recording is, from the audio alone (no filename):
cargo run --release --example detect --features wav -- side.wav
```

`analyze` prints the measured carrier frequency, the L/R quadrature offset, the
lead channel, and the LFSR recovered by Berlekamp–Massey — all derived, not
copied. `confirm` additionally tallies the LFSR across many windows (reporting
the clean-window rate) and reports decoder lock % and position-tracking quality.

## `no_std` (embedded / wasm)

The core decoder is `no_std`-compatible — it needs only `alloc` — so it runs on
bare-metal targets such as **ESP32** and on **`wasm32`**. Disable the default
`std` feature:

```toml
[dependencies]
sl-timecode-decoder = { version = "0.1", default-features = false }
# add "synth" too if you want the ground-truth encoder on-device.
```

Feature map:

| feature    | default | needs `std` | what it adds |
|------------|:-------:|:-----------:|--------------|
| `std`      | ✅      | —           | standard-library math; required by `wav`/`analysis` |
| `synth`    |         | no          | synthetic control-tone encoder (`no_std`-friendly) |
| `wav`      |         | yes         | WAV I/O helper (hound) |
| `analysis` |         | yes         | Berlekamp–Massey + FFT parameter recovery (rustfft) |

In a `no_std` build the transcendental float math (`sin`/`cos`/`atan2`/`sqrt`/…)
is provided by the pure-Rust [`libm`](https://docs.rs/libm) crate; under `std`
the standard-library math is used, so hosted numeric behavior is unchanged.

Verified to compile as:

```sh
cargo build --no-default-features                       # core, no_std + alloc
cargo build --no-default-features --features synth      # + encoder, no_std
cargo build --target wasm32-unknown-unknown --no-default-features
cargo build --target wasm32-unknown-unknown             # std on wasm32
```

> **Note on ESP32 RAM.** The position lookup for a 20-bit register is a ~4 MiB
> direct-indexed table, which does not fit in internal SRAM — target a part with
> PSRAM (e.g. ESP32-S3). The code is `no_std`-clean regardless; this is a
> deployment/memory-planning constraint, not a build one.

## Testing

```sh
cargo test --features "synth wav analysis"
```

Unit tests cover the LFSR bijection, Berlekamp–Massey recovery, and slicing;
integration tests (`tests/roundtrip.rs`) round-trip the synthetic encoder through
the decoder at various speeds, in reverse, and under noise, asserting position
error within a couple of bits. A real-recording smoke test is provided but
ignored by default:

```sh
SL_TC_WAV=/path/to/tone.wav cargo test --features "synth wav" -- --ignored
```

## Status & known limitations

* **Parameters confirmed** against real Serato CV02 recordings (both sides);
  `serato_cv02_side_a()` / `serato_cv02_side_b()` carry `confirmed: true`. The
  absolute-position **origin** is calibrated to the start of the program groove
  (position 0 ≈ start of the main timecode, right after the lead-in, ±1 bit).
* **Absolute position.** Reported modulo the LFSR period (2²⁰−1 bits ≈ 17.5 min
  at 1×), exact and continuous within a side, with 0 at the program start. The
  coded **lead-in groove** reads as **negative** positions (down to ≈ `−4783` /
  `−6051` on sides A/B) and joins the program continuously at 0.
* **Signal-presence gating.** The decoder won't lock on noise-floor input (e.g. a
  record's pre-onset lead-in); lock requires the carrier above `min_signal`
  (default ≈ −40 dBFS, tunable via `Decoder::set_min_signal`) plus a short warmup.
  `DecodeState` exposes `signal` and `confidence`.
* **Edge-case robustness.** The front end handles realistic turntable handling
  (see `examples/stress.rs`): scratching and fast reversals, needle skips
  (re-locks to the new absolute position), mid-play signal dropouts, DC offset (a
  DC blocker), and **narrow dynamic range** — normalized/compressed recordings
  where the 1/0 bit peaks are close together — via an *adaptive bi-level data
  slicer*. A Schmitt-trigger + refractory zero-crossing detector and a windowed
  (matched-filter) peak give ~20% broadband-noise tolerance (~14 dB SNR); it
  degrades gracefully beyond that. Real recordings decode at ~96–97% lock.
* **Speed ceiling.** Sustained playback tracks accurately up to ~4×; beyond that
  the carrier has too few samples per cycle (well below the ~20× Nyquist folding
  point) and lock degrades — above typical scratch speeds. See `examples/stress.rs`.
* **Offline only.** Real-time audio-device input (cpal) is out of scope for now;
  the core is a streaming block-processor, so it can be added later.

## Provenance & sources

This crate is **clean-room** and MIT-licensed, which is only defensible if its
provenance is clean. Concretely:

* **The format model came from public conceptual descriptions.** The general
  idea of a DVS control tone — a ~1 kHz quadrature sine carrier, one
  amplitude-modulated bit per carrier cycle, and a maximal-length LFSR whose
  sliding bit window yields absolute position — is publicly described and is not
  owned by any single implementation. That model is the *only* thing taken from
  external descriptions.
* **Every concrete parameter was measured, not copied.** All constants — carrier
  frequency, the quadrature lead channel, the two per-side 20-bit polynomials,
  and the calibrated seeds — were *recovered from real Serato CV02 recordings* by
  this crate's own analysis code (Berlekamp–Massey + FFT + calibration; see
  `examples/analyze.rs`, `confirm.rs`, `calibrate.rs` and `NOTE.md`). They are
  facts about the pressed vinyl, derived independently, not lifted from another
  program's source.
* **No code was taken from any GPL decoder — including [xwax] and [Mixxx].** Both
  are GPL-licensed (Mixxx's vinyl-control decoder is itself derived from xwax),
  and neither was copied, referenced for constants, or otherwise incorporated.
  This project and those share only the *format* being decoded — an
  interface/compatibility relationship, not a derived-work one. Decoding the same
  publicly-described signal is not a derivative work.

[xwax]: https://xwax.org/
[Mixxx]: https://mixxx.org/

## Disclaimer

This project is an independent, clean-room effort and is **not affiliated with,
endorsed, or sponsored by Serato, inMusic Brands, or any related party**.
"Serato", "Scratch Live", and any other trademarks are the property of their
respective owners; they are used here only for identification and descriptive
purposes.

## License

MIT
