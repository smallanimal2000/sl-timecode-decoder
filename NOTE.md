# NOTE.md — Findings, pitfalls, and lessons

Engineering notes for the clean-room Serato Scratch Live timecode decoder. Written
for whoever (human or agent) picks this up next. Complements the code comments and
README; this is the "why" and the "gotchas."

---

## 1. The Serato CV02 format (measured, clean-room)

All of these were **derived from real recordings** (`serato-cv02-side-a.wav`,
`serato-cv02-side-b.wav`) by this crate's own analysis (`examples/analyze`,
`confirm`, `calibrate`) — not copied from any existing decoder (xwax is GPL).

| property | value | how found |
|---|---|---|
| carrier | ~1000 Hz | FFT (`analysis::carrier_frequency`) |
| stereo | quadrature, **right channel leads** on forward | Goertzel phase offset (+~89°) |
| bits | 1 per carrier cycle, AM: high peak=`1`, low peak=`0` (~−6/−9 dB nominal) | — |
| position code | **20-bit maximal-length LFSR** | Berlekamp–Massey |
| taps side A | `0x361e5` | BM, modal over 93% of windows |
| taps side B | `0x4f0d9` | BM, modal over 97% of windows |
| seed side A | `0x5e3e0` (pos 0 = start of pressed tone) | `calibrate` |
| seed side B | `0x65b62` | `calibrate` |
| period | 2²⁰−1 = 1,048,575 bits ≈ **17.48 min** at 1× | — |

**Surprises worth remembering:**

- **Right channel leads, not left.** My initial working model guessed Left; the
  recordings are Right. Consistent across both sides and all segments. Always
  derive the lead channel from the measured phase sign; never assume.
- **The two sides use DIFFERENT LFSR polynomials** (A `0x361e5` vs B `0x4f0d9`),
  presumably so software can tell side A from side B. So parameters are
  **per-side**, not one global set. Don't assume symmetry.
- **The recordings are full playthroughs that then hit a locked/skipping groove.**
  Side A reaches ~688k bits (~11.5 min) then repeats one ~1.2 s revolution for
  ~25 min; side B climbs to ~1,047,800 bits (16.6 min, *775 short of the wrap*)
  then playback ends. **Neither crosses the LFSR wrap**, so wrap continuity is
  only verifiable synthetically (`crosses_lfsr_wrap_continuously`).

---

## 2. LFSR conventions (get these exactly right)

We use a **right-shifting Fibonacci** LFSR: `out = state & 1`, feedback =
`parity(state & taps)` shifted into the top bit.

- **Tap mapping (the #1 pitfall).** For polynomial `x^N + x^a + x^b + … + 1`, the
  recurrence in this convention is `b_{k+N} = b_{k+a} XOR b_{k+b} XOR …`, so the
  **tap bits are at positions {a, b, …} directly** (e.g. `x^20+x^3+1` → taps at
  bits {3,0} → `0x9`). I first wrote {19,2} (mirror image), which is *not* maximal
  and silently produced a degenerate sequence. Symptom: `is_maximal_length` fails
  and the position map has collisions. **Always assert maximal-length.**
- **Pack property (used everywhere).** In this convention the register at step k
  equals the next `N` output bits, **LSB-first**: `state_k = b_k | b_{k+1}<<1 | …`.
  This is why a window of N sliced bits packs straight back into a state
  (`lfsr::pack_state`) → position lookup. Verify with `pack_matches_register_state`.
- **Direction & the window orientation** (in `decoder`):
  - Forward: pack oldest→newest (oldest = LSB); lookup gives oldest bit's index →
    newest = `k + N − 1`.
  - Reverse: pack the time-**reversed** window; lookup gives the newest bit's index
    directly. (Derive it on paper before touching this — easy to get off-by-one.)
- **Berlekamp–Massey gotchas:**
  - BM needs an **error-free** run. One bit error → it "finds" a huge-order
    recurrence (L ≥ 32 → shift overflow if unguarded). Run BM over **many windows
    and take the modal (L, taps)**; report the clean-window fraction.
  - The connection polynomial → our tap mask: tap bit `L − i` set for each
    `C[i]=1` (see `analysis::connection_to_lfsr`).
  - When validating a recovered state forward, remember `state_m` emits bit `m`
    first — **advance N steps before comparing to bit `m+N`** (I had this bug in
    `calibrate`; it made every anchor look "dirty").

---

## 3. Signal processing / decoder design

Pipeline: DC-block → quadrature phase tracker (pitch/direction) → Schmitt-trigger
zero-crossing (bit timing) → windowed peak → adaptive bi-level slicer (bit value)
→ rolling N-bit window → LFSR position lookup → lock/resync + sub-bit interpolation.

Key design choices and *why*:

- **Position-addressable synthesis.** The control signal at groove position `p` is
  a pure function of `p` (`Encoder::sample_at`). Rendering motion = sampling along
  a position path. This makes scratch, skip, stop, reverse all trivial *and* gives
  exact per-frame ground truth. Best decision in the project — build this first.
- **Phase from a Lissajous angle.** `θ = atan2(lag, lead)`; AM changes radius not
  angle, so `dθ/dt` is a clean pitch/direction signal even as bit amplitude jumps.
- **Slice mid-cycle, direction-aware.** Sample the lead channel at the lag
  channel's zero-crossing (lead is at a peak there). Forward → falling crossing;
  **reverse → rising crossing**. If you use falling for both, reverse samples land
  on the cycle *boundary* where the bit amplitude is switching → ambiguous → poor
  reverse lock. This one cost a while to spot.
- **Adaptive bi-level slicer (crucial for dynamic range).** A *fixed* threshold
  ratio (assuming −6/−9 dB) reads every bit as `1` on a normalized/compressed
  recording where the two peak levels are close. Two smoothed asymmetric-EMA
  followers track the high/low clusters; threshold = midpoint. Handles ratios up
  to ~0.95. Seed the low follower *high* (0.85·hi) so the initial threshold is
  high and adapts *down* for wide DR (seeding low gets stuck for narrow DR).
- **Schmitt trigger + refractory + windowed peak (noise).** Broadband noise makes
  the signal ≈0 near the crossing, so it swings across zero repeatedly → spurious
  bits. Fixes, in order of impact:
  1. **Windowed (matched-filter) peak** — average `|lead|` over ~period/6 samples
     where the cosine is flat; noise ↓ by √K. Biggest single win (~10%→~20%
     noise tolerance). Window shrinks at high speed to avoid smearing.
  2. **Schmitt trigger** (±0.12·level_hi hysteresis) — ignores small wiggles.
  3. **Refractory period** (~0.4 cycle, speed-scaled) — one bit per cycle even if
     noise crosses the band multiple times.
- **Signal-presence gate, NOT confidence.** *Measure before designing:* intro
  noise floor ≈ 0.0001–0.0004 vs real tone ≈ 0.27 (≈1000×). Crucially,
  **confidence is higher on noise than signal** (random peaks vs a tiny envelope),
  so gating on confidence does the opposite of what you want. Gate on **signal
  level** (`min_signal`, default 0.01 ≈ −40 dBFS).
- **Warmup gate.** After onset/dropout, require ~48 strong bits before locking so
  the slicer levels converge and needle-settling transients don't cause a false
  lock. This fixed side B's bogus first lock (pos 825393 → pos 113 ≈ 0).
- **Lock / resync.** Lock after K agreeing lookups; on disagreement, hold the
  predicted position for a few cycles then trust the lookup (rides out isolated
  bit errors without unlocking; re-syncs after a real jump/skip).
- **Sub-bit position.** Integer bit position updates at the bit rate (~1 kHz);
  `fine_position` interpolates between crossings from accumulated carrier phase,
  re-anchored to each locked cycle. Note a ~0.5-bit offset (cycle index vs
  mid-cycle continuous position) — it's definitional, not a bug.

---

## 4. Robustness envelope (measured, `examples/stress.rs`)

| condition | result |
|---|---|
| real recordings | 95.6% / 97.1% lock, **0 continuity glitches** |
| broadband noise | solid to ~20% of tone peak (~14 dB SNR); graceful past that |
| narrow dynamic range | ratio up to ~0.95 (100% accurate) |
| scratching / fast reversals | 80–88% lock, 100% accurate |
| needle skip | re-locks to the correct new absolute position |
| signal dropout | coasts/re-acquires at the advanced position |
| DC offset | absorbed by the DC blocker |
| **speed ceiling** | accurate to ~4× sustained; degrades by 8× |
| combined worst-case | ~14% lock but 96.8% accurate, recovers |

**Speed ceiling is a front-end limit, not Nyquist.** At 8× there are only ~5.5
samples/cycle, so crossing timing + phase resolution degrade — well below the ~20×
Nyquist folding point. Above typical scratch speeds, so left as-is.

---

## 5. Practical pitfalls (operational)

- **Don't load the whole WAV.** The sides are 593 MB (24-bit/44.1k, ~37 min).
  Use `wav::read_stereo_segment` or stream frame-by-frame (`examples/fulldecode`).
- **Position is modulo the period.** Any continuity/monotonicity check must be
  mod `2²⁰−1` (a forward step of +1 at the wrap looks like a −1,048,574 jump).
- **`Date.now`/RNG-in-scripts caveats don't apply here** — but tests use a
  deterministic xorshift for noise so results are reproducible.
- **Test invalidated by a better algorithm.** The old "all-constant amplitude →
  all 1s" DSP test became meaningless once the slicer went adaptive (one level →
  no bits to distinguish). Replaced with "sliced bits obey the LFSR recurrence"
  (alignment-free correctness). Watch for tests encoding assumptions you later fix.
- **Seeds are calibrated to *these* pressings.** If Serato varies lead-in length
  across pressings, the absolute origin could shift a few bits on another copy.
  Re-run `calibrate` per recording; taps/lead are universal, the seed is an origin.

---

## 6. Meta-lessons

1. **Measure before you design.** Every good fix here came from a diagnostic:
   the confidence-vs-signal inversion, the noise-tolerance knee, the DR ratio, the
   speed samples/cycle limit. Guessing thresholds wastes time.
2. **Adaptive beats assumed constants.** Fixed slice ratio, fixed lead channel,
   fixed dynamic range — each assumption broke on real data. The adaptive versions
   are also simpler to reason about ("track the two clusters").
3. **Validate on real data, not just synthetic.** The synthetic round-trip was
   perfect long before the real recordings revealed the lead-channel and per-side
   polynomial facts. Synthetic tests confirm *your model*; real data corrects it.
4. **Clean-room means deriving the facts yourself.** Berlekamp–Massey + FFT +
   calibration reproduce the parameters from the tone, with documented provenance,
   rather than copying constants from GPL code.
5. **Trade-offs are real and local.** Hysteresis band: noise (want large) vs
   narrow DR (want small). Peak window: noise (want long) vs scratch speed (want
   short). Solve by making them *adaptive/scaled*, not by picking one compromise.

---

## 7. Not done / open

- **No real *scratched* recording** to validate against — only clean playthroughs
  exist, so scratch/skip robustness is synthetic-only. Needs data.
- Real-time input (cpal), other DVS formats (Traktor/Final Scratch), CI/publish.
- Speed ceiling >4× (would need sub-sample crossing interpolation / better phase).

**Done since first draft:**

- The `PositionMap` no longer uses a `HashMap`. For registers ≤24 bits it's a
  direct-indexed table (`table[state] = position`, ~4 MiB for 20-bit, O(1)
  lookup, no hashing); wider registers fall back to a sorted `(state, position)`
  array with binary search. Both are allocation-free at lookup time. The all-zero
  state is left `EMPTY`/absent since a maximal-length LFSR never visits it.
- **Full `no_std` support** (for ESP32 / `wasm32`). The core (`format`, `lfsr`,
  `dsp`, `decoder`, `synth`) is `#![no_std] + alloc`; only `wav` (hound) and
  `analysis` (rustfft) need `std`. Notes for the next person:
  - `std` is a **default feature**; `no_std` users set `default-features = false`.
    `wav`/`analysis` re-enable `std` transitively.
  - `core` has no `sin`/`cos`/`atan2`/`sqrt`/`powf`/`floor`/`round`, so a
    `math::FloatExt` shim supplies them via the pure-Rust `libm` crate. It's
    imported **only** under `not(feature = "std")` (via `#[cfg]` + an
    `#[allow(unused_imports)]`), so under `std` the inherent methods win and host
    numeric behavior is unchanged. `min`/`max`/`clamp`/`is_finite` *are* in `core`
    and are intentionally left out of the shim.
  - `libm` is a **non-optional** dep and the shim module is compiled
    unconditionally (though unused under `std`) — otherwise `cargo` flags `libm`
    as an unused dependency in `std` builds. Keep it that way.
  - Collections come from `alloc` (`use alloc::vec::Vec`,
    `alloc::collections::VecDeque`); `extern crate alloc;` lives in `lib.rs`.
    Avoid the `vec!` macro in core code (used `Vec::resize` in `lfsr`).
  - **Memory reality on ESP32:** the 20-bit direct table is ~4 MiB — it will not
    fit in internal SRAM (~520 KB on classic ESP32); it needs PSRAM (e.g.
    ESP32-S3 with 2–8 MiB). The build is `no_std`-clean regardless; the RAM
    footprint is the deployment constraint to plan for.
  - Verify with `cargo build --no-default-features [--features synth]` and
    `cargo build --target wasm32-unknown-unknown [--no-default-features]`.

---

## 8. Continuous motion-state API (`Decoder::pitch/signal_level/phase_total/moving`)

Purely additive getters on `Decoder` that delegate to the internal `Frontend`, so
a host can **poll** motion state on a fixed cadence (e.g. once per audio block)
instead of only reading it off `push_frame`'s `Some(..)` return (which fires only
~once per carrier cycle). Motivation: the wasm binding was running a *second*
parallel `Frontend` purely to poll `pitch()`, doubling the per-sample front-end
DSP and risking slicer-config drift between the two front ends. These getters let
that companion be deleted.

- `pitch() -> f32`, `signal_level() -> f32`, `phase_total() -> f64` — mirror the
  same-named `Frontend` methods. Pure reads; **do not advance state**.
- `moving() -> bool` — convenience: `signal_level() >= min_signal && |pitch| >=
  stop_pitch`. Intentionally does **not** require position lock — relative/scratch
  control needs *motion*, not *position*. `fine_position()` stays the position
  authority (still `None`/`NaN` until locked); these are the motion authority.

> **⚠️ SUPERSEDED by §9 (stylus-lift fix).** The freeze described in this block was
> the bug behind "fails to detect a stylus lift." `signal_level()` now returns a
> per-sample decaying peak envelope and *does* fall below `min_signal` on silence.
> The `moving()`/`stop_pitch` reasoning below still holds; the "signal_level stays
> frozen" claims do not. Left here for the archaeology.

**The pitfall that mattered here — `signal_level()` used to NOT decay on silence.**
The consumer's suggested test (prescription §5) assumed that after the carrier
goes silent, `signal_level()` "decays below `min_signal` within a bounded number
of samples." At the time it didn't: `signal_level()` returned `level_hi`, the
peak-cluster follower, which was **only updated inside the crossing branch** of
`Frontend::push` (`dsp.rs`, `if crossing { … self.level_hi += … }`). Hard silence
produces no zero-crossings → the follower was **frozen** at its last value, not
decayed. So (at the time):

  - **Do not use `signal_level()` alone to detect "needle lifted / carrier gone."**
    It will read stale-high indefinitely after an abrupt stop.
  - `moving()` still reports stopped correctly, but via the **pitch** term, not the
    signal term: `vel_ema` (hence `pitch()`) *is* updated every sample (outside the
    crossing branch), and on silence `atan2(0,0)=0 → dθ=0`, so it relaxes to ~0 and
    the `stop_pitch` gate trips. If `stop_pitch` were ever set to 0, `moving()`
    would get stuck true on silence — the deadband is load-bearing.
  - Consequence for hosts: treat `moving()` (or `pitch()`) as the stop signal.
    `signal_level()` is a carrier-*presence* indicator that updates only while the
    carrier is actually crossing.

**`stop_pitch = 0.02` has no documented derivation.** It arrived in the initial
commit with no provenance in code, git history, README, or this file — unlike the
measured per-side Serato params (§1), it is a hand-picked deadband meaning "under
2% of nominal speed = stopped." Plausible, but not tied to any measurement. If you
touch stop/deadband behavior, either derive it from data or leave a comment; don't
treat it as validated the way taps/seeds/lead are.

**Test lesson — don't duplicate a config constant into an assertion.** My first
stop-detection test hardcoded `0.02` as the threshold (copied from the config
default). That's a silent-staleness trap: retune `stop_pitch` and the test's
premise rots without failing for the right reason. Fixed by asserting the *intent*
("pitch relaxed to rest", `|pitch| < 1e-3`) plus `!moving()` (which reads the real
`cfg.stop_pitch`). There is no `stop_pitch()` getter on `Decoder` to reference
directly; add one if a test genuinely needs the live value. General rule: assert
behavior/intent, or read the live config — never re-encode a magic number.

**Tests added** (`decoder.rs`): poll-equivalence (`pitch()`/`signal_level()` match
the most recent `DecodeState`), pitch tracks a reverse trajectory through lock
state, and stop-after-silence (pitch → rest, `moving()` false). Build is clean
with and without the `synth`/`std` features; the getters add no state, no alloc,
and don't touch `push_frame` output — backwards compatible.

---

## 9. Stylus-lift detection — `signal_level()` now decays on silence

**Symptom reported:** "the library fails to detect a stylus lift." Lifting the
needle removes the carrier entirely; a host watching `signal_level()` against
`min_signal` to notice the record leaving the platter never saw the level drop.

**Root cause (the §8 freeze, now fixed).** `signal_level()` and `BitEvent.signal`
both returned `self.level_hi` — a bi-level *slicer* cluster follower updated **only
inside the `if crossing` branch** of `Frontend::push`, i.e. once per carrier cycle
at a zero-crossing. A stylus lift stops the crossings, so `level_hi` froze at its
last value and the reported level stayed pinned high forever. The mechanism is a
crossing-gated *slice threshold*, not a presence detector — using it as a presence
detector was the category error.

**Fix (`dsp.rs`).** Added a dedicated per-sample carrier-presence envelope,
separate from the slicer levels:

- New `Frontend.env` (+ precomputed `env_decay`): **instant attack** to each
  `|lead|` peak, **exponential release** when the sample is below it. Updated on
  **every** sample (outside the crossing branch), so it tracks ~full-scale while
  the carrier is present and decays toward the noise floor when it stops.
- `signal_level()` and `BitEvent.signal` now report `env`; `level_hi`/`level_lo`
  are untouched and still drive slicing/hysteresis. Clean separation:
  **`level_hi` = slice reference (crossing-gated); `env` = presence (per-sample).**
- Release is time-constant `env_release_cycles` (default **6** carrier cycles) ×
  base period, so it's **sample-rate independent** (computed from `sample_rate /
  carrier_hz` in the constructor). ~26 ms from full scale to −40 dBFS at 1 kHz/44.1k
  — fast enough for lift detection, slow enough that the ~15% intra-cycle ripple
  (`exp(-1/6)≈0.85` between peaks) stays far above `min_signal`.

**What did NOT change, and the residual gotcha.** Bits are still sliced only at
crossings, so a lift emits **no** `DecodeState` — `push_frame` returns `None` and
the last emitted state's `locked` flag goes **stale**. Detection is therefore a
**poll**: watch `signal_level()` (now decays) and/or `moving()`. Two distinct stop
kinds, if you need to tell them apart:

- **Stylus lift** (needle up): both channels → silence → `env` collapses →
  `signal_level() < min_signal`. This is the newly-detectable case.
- **Platter stop** (needle down, not spinning): the AC carrier also stops, so `env`
  decays too — physically similar. `moving()` reports stopped in both via the
  **pitch** term (`vel_ema → 0`), exactly as §8 describes.

If a host needs an event-driven "carrier lost" rather than a poll, add a poll-side
latch or a `Decoder` method that flips lock off when `signal_level()` stays below
`min_signal` for N samples — deliberately **not** added here (keeps the fix minimal
and `push_frame`-output-compatible).

**Test added** (`decoder.rs::stylus_lift_drops_signal_level`): carrier present →
`signal_level() ≥ min_signal`; then feed pure silence → assert it decays **below**
`min_signal`. This is the regression that the old frozen-follower behaviour failed.
All 16 lib tests + 16 stress/integration tests (incl. noise-floor, dropout,
needle-skip — same signal path) + doctest pass, with and without `synth`/`std`.

**Durable lessons:**

1. **A crossing-gated value is not a continuous sensor.** Anything updated only
   inside an event branch (here, per zero-crossing) is undefined between events and
   *frozen* when events stop. If a consumer polls it as a live signal, it will read
   stale exactly when the interesting thing (silence/stop) happens. Give presence
   its own per-sample follower; don't overload the slicer's threshold state.
2. **"Attack fast, release slow" peak envelope is the right primitive for presence.**
   Instant attack tracks the true peak; exponential release makes absence *decay*
   rather than freeze. Tie the release to the signal's own period so it's
   sample-rate independent.
3. **Separate the two roles a "level" plays.** One number was doing double duty as
   slice reference *and* presence indicator; they have opposite update cadences
   (per-crossing vs per-sample). Splitting them fixed the bug without perturbing
   slicing.
4. **When you document a limitation (§8), a later fix must go back and supersede it.**
   §8 codified the freeze as permanent guidance ("do not use `signal_level()` to
   detect needle-lifted"). Leaving that standing after fixing it would mislead the
   next reader worse than never having written it. Mark, don't silently delete.

---

## 10. `no_std` regression — `.exp()` from the §9 fix broke the shim build

**Symptom.** The §9 stylus-lift fix precomputes an exponential release constant in
`Frontend::with_config` (`dsp.rs`):

```rust
let env_decay = (-1.0 / tau).exp();
```

`cargo build` (default, `std`) was green, so this shipped in commit `3c0f812`. But
`cargo build --no-default-features` fails:

```
error[E0599]: no method named `exp` found for type `f32` in the current scope
   --> src/dsp.rs:156:38
```

**Root cause.** `f32::exp`/`f64::exp` are **`std` inherent methods** — `core` does
not provide them. On `no_std` targets the transcendental math comes from the
`math::FloatExt` libm shim (§7), which had `sin`/`cos`/`atan2`/`sqrt`/`powf`/
`floor`/`round`/`trunc`/`div_euclid` but **no `exp`**. So under `std` the inherent
method resolved the call and hid the gap; under `no_std` there was nothing to
resolve it. Classic "green on the default feature set, broken on the one nobody
built locally."

**Fix (`math.rs`).** Added `exp` to `FloatExt` for both float types, backed by
`libm::expf` (f32) and `libm::exp` (f64), following the existing pattern. Under
`std` the inherent method still wins, so hosted numeric behavior is unchanged;
under `no_std` the shim now fills the gap. Both `cargo build` and
`cargo build --no-default-features` are green again.

**Regression test (`math.rs::tests::exp_shim_matches_std`).** The subtle part:
a normal `#[test]` runs under `std`, where the inherent `exp` wins — so a naive
`x.exp()` test would pass *even if the shim method were missing* and never catch
the regression. Two properties make the test actually guard the shim:

- The `math` module is compiled **unconditionally** (§7 — it's kept non-optional so
  `libm` isn't flagged unused under `std`), so `FloatExt` is reachable from a
  standard `std` test build. No `no_std` target needed to exercise it.
- The test calls **`FloatExt::exp(x)` via fully-qualified trait syntax**, not
  `x.exp()`. That forces resolution to the *trait* method, so the inherent `std`
  method can't shadow it. If `exp` ever leaves `FloatExt` again, the test **fails
  to compile** — a compile-time tripwire on the trait's surface. It also asserts
  the libm result agrees with `std`'s inherent `exp` (the shim's whole job is to
  match `std` numeric behavior): tol `1e-5` for f32, `1e-12` for f64.

**Durable lessons:**

1. **A feature flag you don't build is a feature flag you break.** `std` is the
   default; every plain `cargo build`/`test` exercises it and *only* it. Any new
   transcendental/rounding float call is invisible to the default build and only
   bites `--no-default-features`. **Always run `cargo build --no-default-features`
   (and the `wasm32` target, §7) after touching numeric core code** — ideally in
   CI, which this repo still lacks (open item).
2. **Adding a float method at a call site is also a shim change.** The `FloatExt`
   trait must stay a superset of every float method used in `no_std`-compiled
   modules (`format`, `lfsr`, `dsp`, `decoder`, `synth`). Before using a new one
   (`exp`, `tan`, `ln`, `hypot`, …), check it's in the shim; if not, add it with its
   `libm::*f`/`libm::*` backing in the same commit.
3. **Test the trait surface via fully-qualified syntax so inherent methods can't
   mask a gap.** When a shim exists precisely because inherent methods sometimes
   win, a plain method-call test proves nothing about the shim. `Trait::method(x)`
   pins resolution to the trait and turns a missing method into a compile error.
4. **Green-on-default ≠ green.** The `std`-only pass in §9 is exactly why this
   slipped through. Match the guard to the surface you actually ship — here, all
   feature permutations, not just the convenient one.
