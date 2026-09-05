//! Clean-room decoder for DVS control-tone timecode (Serato Scratch Live style).
//!
//! # What this is
//!
//! Digital vinyl systems press a *control tone* onto vinyl/CD: a stereo audio
//! signal that encodes the stylus's absolute position, speed and direction. This
//! crate decodes such a signal (offline, from recorded stereo PCM) back into
//! position / pitch / direction.
//!
//! The implementation is **clean-room**: it is written from public conceptual
//! descriptions of the format plus empirical analysis of a recording, without
//! copying any existing decoder's source.
//!
//! ## Signal model (Serato CV02)
//!
//! * A steady ~1 kHz sine **carrier**; its instantaneous frequency is
//!   proportional to playback speed (pitch).
//! * The two stereo channels are in **quadrature** (~90° apart); which channel
//!   leads gives play direction.
//! * Each carrier cycle carries one **bit**, amplitude-modulated: a high peak is
//!   `1`, a low peak is `0`.
//! * The bit stream is a **maximal-length LFSR sequence**, so any window of
//!   `lfsr_bits` consecutive bits maps to a unique absolute position.
//!
//! ## Usage
//!
//! ```
//! use sl_timecode_decoder::{Decoder, format};
//!
//! let mut dec = Decoder::new(format::serato_cv02(), 44_100.0);
//! # let frames: Vec<(f32, f32)> = Vec::new();
//! for state in dec.process(&frames) {
//!     // state.position_bits, state.pitch, state.direction, ...
//!     let _ = state;
//! }
//! ```
//!
//! See [`Decoder`] for the streaming API and [`format`] for the pluggable format
//! descriptor.
//!
//! ## `no_std`
//!
//! The core decoder is `no_std`-compatible (it only needs `alloc`), so it runs on
//! bare-metal / embedded targets such as ESP32 and on `wasm32`. Disable default
//! features to drop `std`:
//!
//! ```toml
//! sl-timecode-decoder = { version = "0.1", default-features = false }
//! ```
//!
//! In a `no_std` build the transcendental float math is provided by the pure-Rust
//! [`libm`](https://docs.rs/libm) crate. The `std` feature (on by default) enables
//! the standard-library math and is required by the `wav` and `analysis` helpers.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

pub mod decoder;
pub mod dsp;
pub mod format;
pub mod lfsr;

// The libm-backed float-math shim is always compiled (so `libm` is a genuinely
// used dependency in every configuration), but its [`math::FloatExt`] trait is
// only *imported* by the other modules under `not(feature = "std")`. Under `std`
// the inherent standard-library methods take priority, so this is dead there.
pub(crate) mod math;

#[cfg(any(test, feature = "synth"))]
pub mod synth;

#[cfg(feature = "analysis")]
pub mod analysis;

#[cfg(feature = "wav")]
pub mod wav;

pub use decoder::{DecodeState, Decoder, Direction};
pub use format::{Side, TimecodeFormat};
