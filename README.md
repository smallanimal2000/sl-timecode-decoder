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
| seed (origin)| `0x5e3e0` | `0x65b62` |

The two sides use **different** LFSR polynomials so software can distinguish
them. The `seed` is the LFSR state at the **first carrier cycle of the pressed
tone**, so `position 0` = start of the groove timecode. Seeds were calibrated
from start-to-end recordings by `examples/calibrate.rs` (detect onset → find a
clean LFSR anchor → step the register back to the onset cycle), accurate to
~±1 bit.

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

# Calibrate the absolute-position origin (seed) from a start-to-end recording:
cargo run --example calibrate --features "analysis wav synth" -- side-a.wav side-b.wav
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
  absolute-position **origin** is calibrated to the start of the pressed tone
  (position 0 ≈ groove start, ±1 bit).
* **Absolute position.** Reported modulo the LFSR period (2²⁰−1 bits ≈ 17.5 min
  at 1×), exact and continuous within a side, with 0 at the groove start.
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

## License

MIT
