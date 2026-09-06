//! Generic Fibonacci LFSR and the state→position map used to resolve absolute
//! position from a window of decoded bits.
//!
//! # Conventions
//!
//! We use a **Fibonacci** LFSR of `bits` width with a feedback tap mask `taps`.
//! One step emits the current LSB and shifts right, feeding the parity of
//! `state & taps` into the top bit:
//!
//! ```text
//! out       = state & 1
//! feedback  = parity(state & taps)
//! state'    = (state >> 1) | (feedback << (bits - 1))
//! ```
//!
//! A convenient property of this convention: the register at a given step equals
//! the next `bits` output bits, LSB-first. That is, if the output stream is
//! `b0, b1, b2, …`, then
//!
//! ```text
//! state_k = b_k | (b_{k+1} << 1) | … | (b_{k+bits-1} << (bits-1))
//! ```
//!
//! So a window of `bits` consecutive output bits can be packed directly back
//! into the register state (see [`pack_state`]), then looked up in a
//! [`PositionMap`] to recover the absolute index `k`.

use alloc::vec::Vec;

#[cfg(not(any(
    feature = "map-fast",
    feature = "map-balanced",
    feature = "map-compact"
)))]
compile_error!(
    "enable at least one position-map store feature: `map-fast`, `map-balanced`, or `map-compact`"
);

/// Parity (XOR of all bits) of `v`.
#[inline]
pub fn parity(v: u32) -> u32 {
    v.count_ones() & 1
}

/// A Fibonacci LFSR configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lfsr {
    /// Register width in bits (e.g. 20).
    pub bits: u32,
    /// Feedback tap mask (feedback polynomial), width `bits`.
    pub taps: u32,
}

impl Lfsr {
    pub const fn new(bits: u32, taps: u32) -> Self {
        Lfsr { bits, taps }
    }

    /// Mask of `bits` set low bits.
    #[inline]
    pub fn mask(&self) -> u32 {
        if self.bits >= 32 {
            u32::MAX
        } else {
            (1u32 << self.bits) - 1
        }
    }

    /// Advance one step, returning the emitted bit and updating `state`.
    #[inline]
    pub fn step(&self, state: &mut u32) -> u32 {
        let out = *state & 1;
        let fb = parity(*state & self.taps);
        *state = ((*state >> 1) | (fb << (self.bits - 1))) & self.mask();
        out
    }

    /// Reverse one step: given the state *after* a forward step, recover the
    /// state *before* it. Returns `None` if no consistent predecessor exists
    /// (should not happen for a well-formed maximal-length LFSR).
    #[inline]
    pub fn step_back(&self, state: u32) -> Option<u32> {
        // Forward: state' = (prev >> 1) | (fb << (bits-1)), fb = parity(prev & taps).
        // The low `bits-1` bits of state' are `prev >> 1`, so prev's high bits are
        // known; only prev's LSB is free. Try both, and require the feedback bit to
        // match state's top bit.
        let top = (state >> (self.bits - 1)) & 1;
        let low = (state & self.mask()) & !(1 << (self.bits - 1)); // prev >> 1
        for lsb in 0..2u32 {
            let prev = ((low << 1) | lsb) & self.mask();
            if parity(prev & self.taps) == top {
                return Some(prev);
            }
        }
        None
    }

    /// Generate the full output sequence starting from `seed`, length equal to
    /// the period. For a maximal-length polynomial with a nonzero seed this is
    /// `2^bits - 1` bits long and visits every nonzero state exactly once.
    pub fn sequence(&self, seed: u32) -> Vec<u8> {
        let period = (1u64 << self.bits) - 1;
        let mut state = seed & self.mask();
        let mut out = Vec::with_capacity(period as usize);
        for _ in 0..period {
            out.push(self.step(&mut state) as u8);
        }
        out
    }

    /// True if `(bits, taps)` generate a maximal-length sequence from `seed`
    /// (period `2^bits - 1`, every nonzero state visited once). O(period).
    pub fn is_maximal_length(&self, seed: u32) -> bool {
        let period = (1u64 << self.bits) - 1;
        let mask = self.mask();
        let start = seed & mask;
        if start == 0 {
            return false;
        }
        let mut state = start;
        for i in 0..period {
            self.step(&mut state);
            if state == start {
                return i + 1 == period;
            }
        }
        false
    }
}

/// Pack `bits` output bits (LSB-first, `window[0]` is the earliest bit) into the
/// LFSR register state at the time of `window[0]`. `window.len()` must equal
/// `bits`.
#[inline]
pub fn pack_state(window: &[u8]) -> u32 {
    let mut s = 0u32;
    for (i, &b) in window.iter().enumerate() {
        s |= ((b & 1) as u32) << i;
    }
    s
}

/// Widest register for which a per-state table (the [`Store::Packed`] fast store,
/// or the sorted fallback build) is materialised. At 24 bits a bit-packed table
/// is `2^24 * 24 bits = 48 MiB`; wider registers only ever use the sorted store.
const DIRECT_TABLE_MAX_BITS: u32 = 24;

/// Which backing store [`PositionMap::build`] materialises. Each variant is
/// gated by its cargo feature (`map-fast` / `map-balanced` / `map-compact`), so
/// only the stores you compile in exist. All give identical lookups; they trade
/// memory against per-lookup CPU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PositionMapKind {
    /// Bit-packed per-state table: a single indexed read + shift/mask, O(1). Uses
    /// `2^bits * bits` bits (~2.5 MiB for a 20-bit register — 37.5% less than a
    /// naive `u32` table). The fast default on hosts with memory to spare.
    /// Requires the `map-fast` feature.
    #[cfg(feature = "map-fast")]
    Fast,
    /// Baby-step giant-step discrete-log store: a mid-size table (~256 KiB) with
    /// ~1 µs lookups (several× faster than [`Compact`](Self::Compact)) — the
    /// middle ground between `Fast` and `Compact`. Suited to embedded targets
    /// with a few hundred KB to spare (far less than the packed table, far faster
    /// than the Pohlig–Hellman store — e.g. for hard scratching on an ESP32).
    /// Requires the `map-balanced` feature.
    #[cfg(feature = "map-balanced")]
    Balanced,
    /// Pohlig–Hellman discrete-log store: a few dozen bytes regardless of period
    /// (~2000× smaller than the table), at the cost of a handful of GF(2^bits)
    /// field operations per lookup. The default on bare `no_std`/embedded builds,
    /// where the table would not fit. Requires the `map-compact` feature.
    #[cfg(feature = "map-compact")]
    Compact,
}

impl PositionMapKind {
    /// Default store for the current build — the best *enabled* store for the
    /// target. Hosts (`std`) prefer speed (`Fast` → `Balanced` → `Compact`);
    /// bare `no_std` targets prefer small RAM (`Compact` → `Balanced` → `Fast`).
    ///
    /// To keep the fast bit-packed table on an embedded target (e.g. an ESP32
    /// whose global allocator is backed by PSRAM, so the ~2.5 MiB table lands in
    /// PSRAM), build `no_std` with only the `map-fast` feature.
    pub const fn target_default() -> Self {
        // Exactly one of these arms is compiled for any valid feature set (the
        // `compile_error!` above rules out "no store enabled").
        #[cfg(all(feature = "std", feature = "map-fast"))]
        return PositionMapKind::Fast;
        #[cfg(all(feature = "std", not(feature = "map-fast"), feature = "map-balanced"))]
        return PositionMapKind::Balanced;
        #[cfg(all(
            feature = "std",
            not(feature = "map-fast"),
            not(feature = "map-balanced"),
            feature = "map-compact"
        ))]
        return PositionMapKind::Compact;

        #[cfg(all(not(feature = "std"), feature = "map-compact"))]
        return PositionMapKind::Compact;
        #[cfg(all(
            not(feature = "std"),
            not(feature = "map-compact"),
            feature = "map-balanced"
        ))]
        return PositionMapKind::Balanced;
        #[cfg(all(
            not(feature = "std"),
            not(feature = "map-compact"),
            not(feature = "map-balanced"),
            feature = "map-fast"
        ))]
        return PositionMapKind::Fast;
    }
}

/// Lookup backing for [`PositionMap`].
enum Store {
    /// Bit-packed `table[state] = position` in `bits`-wide little-endian fields.
    /// O(1), allocation-free. Used for registers up to [`DIRECT_TABLE_MAX_BITS`].
    #[cfg(feature = "map-fast")]
    Packed(Packed),
    /// Baby-step giant-step discrete-log store (~256 KiB, ~1 µs). See [`Bsgs`].
    #[cfg(feature = "map-balanced")]
    Bsgs(Bsgs),
    /// Discrete-log store (Pohlig–Hellman + CRT). Tiny and O(1)-memory in the
    /// period, at a few-thousand-op-per-lookup cost. See [`Dlog`].
    #[cfg(feature = "map-compact")]
    Dlog(Dlog),
    /// `(state, position)` pairs sorted by state, searched with `binary_search`.
    /// Fallback for registers too wide to pack, or when the discrete-log store
    /// cannot be built (non-primitive polynomial, unfavourable period).
    Sorted(Vec<(u32, u32)>),
}

/// Maps every reachable LFSR state to its absolute index in the output sequence.
///
/// Index `k` is the position such that `state_k = pack(b_k … b_{k+bits-1})`.
///
/// The backing store is chosen by [`PositionMapKind`]: a bit-packed table
/// ([`Fast`](PositionMapKind::Fast)) or a Pohlig–Hellman discrete-log store
/// ([`Compact`](PositionMapKind::Compact)). Both are allocation-free at lookup
/// time and give identical results; they trade memory against per-lookup CPU.
pub struct PositionMap {
    lfsr: Lfsr,
    seed: u32,
    store: Store,
}

impl PositionMap {
    /// Build the map using the [`target default`](PositionMapKind::target_default)
    /// store for the current build.
    pub fn build(lfsr: Lfsr, seed: u32) -> Self {
        Self::build_with_kind(lfsr, seed, PositionMapKind::target_default())
    }

    /// Build the map with an explicit [`PositionMapKind`]. `Compact` silently
    /// falls back to the sorted store if the discrete-log store cannot be built
    /// for this `(lfsr, seed)`; `Fast` falls back for registers wider than
    /// [`DIRECT_TABLE_MAX_BITS`].
    pub fn build_with_kind(lfsr: Lfsr, seed: u32, kind: PositionMapKind) -> Self {
        let seed = seed & lfsr.mask();
        let store = match kind {
            #[cfg(feature = "map-compact")]
            PositionMapKind::Compact => match Dlog::build(lfsr, seed) {
                Some(d) => Store::Dlog(d),
                None => Self::build_sorted_store(lfsr, seed),
            },
            #[cfg(feature = "map-balanced")]
            PositionMapKind::Balanced => match Bsgs::build(lfsr, seed) {
                Some(b) => Store::Bsgs(b),
                None => Self::build_sorted_store(lfsr, seed),
            },
            #[cfg(feature = "map-fast")]
            PositionMapKind::Fast => {
                if lfsr.bits <= DIRECT_TABLE_MAX_BITS {
                    Store::Packed(Packed::build(lfsr, seed))
                } else {
                    Self::build_sorted_store(lfsr, seed)
                }
            }
        };
        PositionMap { lfsr, seed, store }
    }

    /// Sorted `(state, position)` store, built by walking the whole sequence.
    /// The walk visits states in position order, so collect then sort by state.
    fn build_sorted_store(lfsr: Lfsr, seed: u32) -> Store {
        let period = (1u64 << lfsr.bits) - 1;
        let mut pairs = Vec::with_capacity(period as usize);
        let mut state = seed & lfsr.mask();
        for k in 0..period {
            pairs.push((state, k as u32));
            lfsr.step(&mut state);
        }
        pairs.sort_unstable_by_key(|&(s, _)| s);
        // A maximal-length LFSR visits each nonzero state once, so there are no
        // duplicate keys to dedup.
        Store::Sorted(pairs)
    }

    /// Test-only: build using the sorted-array store regardless of register
    /// width, so the binary-search path can be exercised on a small register.
    #[cfg(test)]
    fn build_sorted(lfsr: Lfsr, seed: u32) -> Self {
        let seed = seed & lfsr.mask();
        PositionMap {
            lfsr,
            seed,
            store: Self::build_sorted_store(lfsr, seed),
        }
    }

    pub fn lfsr(&self) -> Lfsr {
        self.lfsr
    }

    pub fn seed(&self) -> u32 {
        self.seed
    }

    /// Look up the absolute index for a raw register `state`.
    #[inline]
    pub fn position_of_state(&self, state: u32) -> Option<u32> {
        let state = state & self.lfsr.mask();
        match &self.store {
            #[cfg(feature = "map-fast")]
            Store::Packed(p) => p.get(state),
            #[cfg(feature = "map-balanced")]
            Store::Bsgs(b) => b.position_of_state(state),
            #[cfg(feature = "map-compact")]
            Store::Dlog(d) => d.position_of_state(state),
            Store::Sorted(pairs) => pairs
                .binary_search_by_key(&state, |&(s, _)| s)
                .ok()
                .map(|i| pairs[i].1),
        }
    }

    /// Resolve the absolute index of the earliest bit in `window`, where
    /// `window` holds exactly `bits` consecutive output bits, `window[0]`
    /// earliest. Returns `None` if the window doesn't correspond to a valid
    /// state (e.g. all-zero, or bit errors produced an unreachable state — every
    /// nonzero state is reachable for a maximal-length LFSR, so this mainly
    /// guards the zero state).
    #[inline]
    pub fn position_of_window(&self, window: &[u8]) -> Option<u32> {
        if window.len() != self.lfsr.bits as usize {
            return None;
        }
        self.position_of_state(pack_state(window))
    }
}

// ---------------------------------------------------------------------------
// Fast store: bit-packed per-state table.
// ---------------------------------------------------------------------------

/// `state → position` as `bits`-wide little-endian fields packed back-to-back.
/// A 20-bit register needs `2^20 * 20 bits = 2.5 MiB` (vs 4 MiB for a naive
/// `Vec<u32>`). Lookup reads a `u32` at the field's byte offset and shifts —
/// O(1), allocation-free. The backing buffer is padded so the last field's
/// 4-byte read stays in bounds.
#[cfg(feature = "map-fast")]
struct Packed {
    /// Field width in bits (= register width); position values are `< 2^bits`.
    bits: u32,
    /// Low-`bits` mask, applied to the read word to isolate the field.
    field_mask: u32,
    data: Vec<u8>,
}

#[cfg(feature = "map-fast")]
impl Packed {
    fn build(lfsr: Lfsr, seed: u32) -> Self {
        let bits = lfsr.bits as usize;
        let slots = 1usize << bits; // one field per state value, incl. never-used 0
        let field_mask = ((1u64 << bits) - 1) as u32;
        // +4 pad so the top field's `u32` read never runs past the end.
        let mut data = alloc::vec![0u8; (slots * bits).div_ceil(8) + 4];
        let period = (1u64 << lfsr.bits) - 1;
        let mut state = seed & lfsr.mask();
        for k in 0..period {
            // Read-modify-write the field: adjacent fields share these bytes, so
            // clear only this field's bits before OR-ing the value in.
            let off = state as usize * bits;
            let byte = off >> 3;
            let shift = (off & 7) as u32;
            let mut w =
                u32::from_le_bytes([data[byte], data[byte + 1], data[byte + 2], data[byte + 3]]);
            w &= !(field_mask << shift);
            w |= (k as u32) << shift;
            data[byte..byte + 4].copy_from_slice(&w.to_le_bytes());
            lfsr.step(&mut state);
        }
        Packed {
            bits: lfsr.bits,
            field_mask,
            data,
        }
    }

    #[inline]
    fn get(&self, state: u32) -> Option<u32> {
        // The all-zero state is never visited (its field reads 0, but so does
        // the seed's — position 0 — hence the explicit guard here).
        if state == 0 {
            return None;
        }
        let off = state as usize * self.bits as usize;
        let byte = off >> 3;
        let shift = (off & 7) as u32;
        let w = u32::from_le_bytes([
            self.data[byte],
            self.data[byte + 1],
            self.data[byte + 2],
            self.data[byte + 3],
        ]);
        Some((w >> shift) & self.field_mask)
    }
}

// ---------------------------------------------------------------------------
// Compact store: position by discrete logarithm (Pohlig–Hellman).
// ---------------------------------------------------------------------------
//
// A maximal-length LFSR's nonzero states form a cyclic group; the step map is
// multiplication by a fixed generator. Interpreting the register as an element
// of GF(2^bits), `state_k = x^k · seed`, so the position `k` is the discrete
// logarithm of `state · seed^-1` base `x` — recoverable without any per-state
// table. We compute it by Pohlig–Hellman: reduce the log modulo each prime-power
// factor of the group order `2^bits - 1`, then recombine by CRT.
//
// One subtlety recovered empirically: the register bit-order is *not* the field
// power basis, so we build the power basis explicitly (columns `T^i(1)`, `T` =
// step) and carry the change-of-basis matrix `B^-1`.

/// Multiply in GF(2^bits) in the power basis, reducing mod `x^bits ≡ red`.
#[cfg(any(feature = "map-balanced", feature = "map-compact"))]
#[inline]
fn gf_mul(mut a: u32, mut b: u32, bits: u32, red: u32) -> u32 {
    let hibit = 1u32 << (bits - 1);
    let mask = if bits >= 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    let mut r = 0u32;
    while b != 0 {
        if b & 1 != 0 {
            r ^= a;
        }
        b >>= 1;
        let carry = a & hibit;
        a = (a << 1) & mask;
        if carry != 0 {
            a ^= red;
        }
    }
    r
}

/// `a^e` in GF(2^bits) by square-and-multiply. The field one is `1` (`= x^0`).
#[cfg(any(feature = "map-balanced", feature = "map-compact"))]
#[inline]
fn gf_pow(mut a: u32, mut e: u64, bits: u32, red: u32) -> u32 {
    let mut r = 1u32;
    while e != 0 {
        if e & 1 != 0 {
            r = gf_mul(r, a, bits, red);
        }
        a = gf_mul(a, a, bits, red);
        e >>= 1;
    }
    r
}

/// Prime-power factorisation of `n` as `(p^e, p)` pairs, by trial division.
#[cfg(any(feature = "map-balanced", feature = "map-compact"))]
fn factor_prime_powers(mut n: u64) -> Vec<(u64, u64)> {
    let mut out = Vec::new();
    let mut d = 2u64;
    while d * d <= n {
        if n.is_multiple_of(d) {
            let mut pe = 1u64;
            while n.is_multiple_of(d) {
                pe *= d;
                n /= d;
            }
            out.push((pe, d));
        }
        d += 1;
    }
    if n > 1 {
        out.push((n, n));
    }
    out
}

/// Modular inverse of `a` mod `m` (assumes `gcd(a, m) = 1`), by extended Euclid.
#[cfg(feature = "map-compact")]
fn mod_inv(a: u128, m: u128) -> u128 {
    let (mut old_r, mut r) = (a as i128, m as i128);
    let (mut old_s, mut s) = (1i128, 0i128);
    while r != 0 {
        let q = old_r / r;
        let t = old_r - q * r;
        old_r = r;
        r = t;
        let t = old_s - q * s;
        old_s = s;
        s = t;
    }
    old_s.rem_euclid(m as i128) as u128
}

/// Chinese Remainder Theorem over pairwise-coprime `(residue, modulus)` pairs.
#[cfg(feature = "map-compact")]
fn crt(residues: &[(u128, u128)]) -> u128 {
    let mut modulus = 1u128;
    for &(_, m) in residues {
        modulus *= m;
    }
    let mut x = 0u128;
    for &(rr, m) in residues {
        let ni = modulus / m;
        let inv = mod_inv(ni % m, m);
        let term = ((rr % m) * (ni % modulus) % modulus) * inv % modulus;
        x = (x + term) % modulus;
    }
    x % modulus
}

/// Largest prime-power subgroup for which the compact store brute-forces a
/// discrete log; beyond this the search would be too slow and we fall back.
#[cfg(feature = "map-compact")]
const DLOG_MAX_SUBGROUP: u64 = 1 << 16;

/// GF(2^bits) in the LFSR step map's power basis, plus the register→power-basis
/// change of basis. Shared machinery for the discrete-log stores [`Dlog`] and
/// [`Bsgs`]: `state_k = x^k · seed`, so a position is a discrete log base `x`.
#[cfg(any(feature = "map-balanced", feature = "map-compact"))]
struct Field {
    bits: u32,
    mask: u32,
    period: u64,
    /// Reduction mask: `x^bits ≡ red` (power basis).
    red: u32,
    /// Rows of `B^-1`: `phi(s)_i = parity(binv[i] & s)`.
    binv: Vec<u32>,
}

#[cfg(any(feature = "map-balanced", feature = "map-compact"))]
impl Field {
    /// Build the field for `lfsr`, or `None` if the discrete-log approach does
    /// not apply (register too wide, basis singular, or a non-primitive
    /// polynomial — `x` must generate the whole group for a position to be its
    /// discrete log).
    fn build(lfsr: Lfsr) -> Option<Field> {
        let bits = lfsr.bits;
        if !(2..=DIRECT_TABLE_MAX_BITS).contains(&bits) {
            return None;
        }
        let mask = lfsr.mask();
        let period = (1u64 << bits) - 1;
        let n = bits as usize;

        // Power-basis columns col[i] = T^i(1) (T = step); w = T^bits(1).
        let mut cols = [0u32; 24];
        let mut v = 1u32;
        for c in cols.iter_mut().take(n) {
            *c = v;
            lfsr.step(&mut v);
        }
        let w = v;

        // Matrix B (rows over the state's bit index) augmented with I (→ B^-1)
        // and w's bits (→ reduction mask, since x^bits = B·red in this basis).
        let mut rows = [0u32; 24];
        for (r, row) in rows.iter_mut().enumerate().take(n) {
            for (c, &col) in cols.iter().enumerate().take(n) {
                if (col >> r) & 1 != 0 {
                    *row |= 1 << c;
                }
            }
        }
        let mut inv = [0u32; 24];
        for (r, e) in inv.iter_mut().enumerate().take(n) {
            *e = 1u32 << r;
        }
        let mut wb = [0u8; 24];
        for (r, e) in wb.iter_mut().enumerate().take(n) {
            *e = ((w >> r) & 1) as u8;
        }

        // Gauss–Jordan over GF(2): drive `rows` to the identity.
        for col in 0..n {
            let sel = (col..n).find(|&r| (rows[r] >> col) & 1 != 0)?;
            rows.swap(col, sel);
            inv.swap(col, sel);
            wb.swap(col, sel);
            for r in 0..n {
                if r != col && (rows[r] >> col) & 1 != 0 {
                    rows[r] ^= rows[col];
                    inv[r] ^= inv[col];
                    wb[r] ^= wb[col];
                }
            }
        }
        let mut red = 0u32;
        for (i, &b) in wb.iter().enumerate().take(n) {
            if b != 0 {
                red |= 1 << i;
            }
        }
        let binv: Vec<u32> = inv[..n].to_vec();

        let f = Field {
            bits,
            mask,
            period,
            red,
            binv,
        };

        // Primitivity: x (value 2) must generate the whole group, i.e.
        // x^(period/p) != 1 for every distinct prime p | period.
        for &(_, p) in &factor_prime_powers(period) {
            if f.pow(2, period / p) == 1 {
                return None;
            }
        }
        Some(f)
    }

    /// Map a register value into power-basis coordinates (`= B^-1 · state`).
    #[inline]
    fn phi(&self, state: u32) -> u32 {
        let mut out = 0u32;
        for (i, &row) in self.binv.iter().enumerate() {
            out |= parity(row & state) << i;
        }
        out
    }

    #[inline]
    fn mul(&self, a: u32, b: u32) -> u32 {
        gf_mul(a, b, self.bits, self.red)
    }

    #[inline]
    fn pow(&self, a: u32, e: u64) -> u32 {
        gf_pow(a, e, self.bits, self.red)
    }

    /// Turn a raw discrete log (base `x`) of `phi(state)` into an absolute
    /// position, given the log of `phi(seed)`.
    #[inline]
    fn position_from_log(&self, log: u32, dseed: u32) -> u32 {
        (log as i64 - dseed as i64).rem_euclid(self.period as i64) as u32
    }
}

/// Compact discrete-log store (Pohlig–Hellman + CRT): a few dozen bytes, at a
/// few-thousand-field-op cost per lookup. See [`PositionMapKind::Compact`].
#[cfg(feature = "map-compact")]
struct Dlog {
    field: Field,
    /// Discrete log of `phi(seed)`; subtracted to turn a raw log into `k`.
    dseed: u32,
    /// Per prime-power factor `pe` of the period: `(pe, x^(period/pe))` (subgroup gen).
    factors: Vec<(u32, u32)>,
}

#[cfg(feature = "map-compact")]
impl Dlog {
    /// Build the store, or `None` if the field cannot be built (see
    /// [`Field::build`]) or the period has an inconveniently large prime-power
    /// factor (the per-subgroup brute-force search would be too slow).
    fn build(lfsr: Lfsr, seed: u32) -> Option<Self> {
        let field = Field::build(lfsr)?;
        let mut factors = Vec::new();
        for &(pe, _) in &factor_prime_powers(field.period) {
            if pe > DLOG_MAX_SUBGROUP {
                return None;
            }
            factors.push((pe as u32, field.pow(2, field.period / pe)));
        }
        let mut d = Dlog {
            field,
            dseed: 0,
            factors,
        };
        d.dseed = d.raw_dlog(d.field.phi(seed & d.field.mask))?;
        Some(d)
    }

    /// Discrete log of field element `h` base `x`, by Pohlig–Hellman + CRT.
    fn raw_dlog(&self, h: u32) -> Option<u32> {
        let f = &self.field;
        let mut residues: Vec<(u128, u128)> = Vec::with_capacity(self.factors.len());
        for &(pe, gamma) in &self.factors {
            let target = f.pow(h, f.period / pe as u64);
            // Brute the log within the order-`pe` subgroup (pe is small).
            let mut acc = 1u32;
            let mut found = None;
            for e in 0..pe {
                if acc == target {
                    found = Some(e);
                    break;
                }
                acc = f.mul(acc, gamma);
            }
            residues.push((found? as u128, pe as u128));
        }
        Some(crt(&residues) as u32)
    }

    #[inline]
    fn position_of_state(&self, state: u32) -> Option<u32> {
        let state = state & self.field.mask;
        if state == 0 {
            return None;
        }
        let log = self.raw_dlog(self.field.phi(state))?;
        Some(self.field.position_from_log(log, self.dseed))
    }
}

/// Baby-step table size (entries) for the [`Bsgs`] store. 2^15 entries × 8 bytes
/// = 256 KiB, giving `ceil(period / 2^15)` giant steps per lookup (≈ 32 for a
/// 20-bit register) — a middle point between [`Dlog`] (tiny, ~µs) and the packed
/// table (~2.5 MiB, ~ns). See [`PositionMapKind::Balanced`].
#[cfg(feature = "map-balanced")]
const BSGS_TABLE_ENTRIES: u64 = 1 << 15;

/// Balanced discrete-log store (baby-step giant-step): a ~256 KiB table with
/// ~1 µs lookups. See [`PositionMapKind::Balanced`].
#[cfg(feature = "map-balanced")]
struct Bsgs {
    field: Field,
    dseed: u32,
    /// Baby-step count (= table size); giant steps advance by `x^b`.
    b: u64,
    /// Giant-step multiplier `x^-b = x^(period - b)` (so `h · giant^i = x^(log - i·b)`).
    giant: u32,
    /// `(x^j, j)` for `j` in `0..b`, sorted by field element for binary search.
    baby: Vec<(u32, u32)>,
}

#[cfg(feature = "map-balanced")]
impl Bsgs {
    fn build(lfsr: Lfsr, seed: u32) -> Option<Self> {
        let field = Field::build(lfsr)?;
        let period = field.period;
        let b = BSGS_TABLE_ENTRIES.min(period);
        // Baby steps: x^j for j in 0..b, sorted by element for lookup.
        let mut baby = Vec::with_capacity(b as usize);
        let mut e = 1u32; // x^0
        for j in 0..b {
            baby.push((e, j as u32));
            e = field.mul(e, 2); // × x
        }
        baby.sort_unstable_by_key(|&(elt, _)| elt);
        let giant = field.pow(2, period - b);
        let mut s = Bsgs {
            field,
            dseed: 0,
            b,
            giant,
            baby,
        };
        s.dseed = s.raw_dlog(s.field.phi(seed & s.field.mask))?;
        Some(s)
    }

    /// Discrete log of `h` base `x` by baby-step giant-step: find `i, j` with
    /// `h · x^(-i·b) = x^j` (`j < b`); then `log = i·b + j`.
    fn raw_dlog(&self, h: u32) -> Option<u32> {
        let f = &self.field;
        let mut cur = h;
        let giant_steps = f.period.div_ceil(self.b);
        for i in 0..giant_steps {
            if let Ok(idx) = self.baby.binary_search_by_key(&cur, |&(elt, _)| elt) {
                let log = i * self.b + self.baby[idx].1 as u64;
                return Some((log % f.period) as u32);
            }
            cur = f.mul(cur, self.giant);
        }
        None
    }

    #[inline]
    fn position_of_state(&self, state: u32) -> Option<u32> {
        let state = state & self.field.mask;
        if state == 0 {
            return None;
        }
        let log = self.raw_dlog(self.field.phi(state))?;
        Some(self.field.position_from_log(log, self.dseed))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // A known maximal-length 20-bit polynomial: x^20 + x^3 + 1.
    // Recurrence b_{k+20} = b_{k+3} XOR b_{k} -> taps at bit positions {3, 0}.
    fn lfsr20() -> Lfsr {
        Lfsr::new(20, (1 << 3) | (1 << 0))
    }

    #[test]
    fn small_lfsr_is_maximal() {
        // x^3 + x^2 + 1 -> taps at bit2 and bit1? Use a known 3-bit maximal one.
        // For 3-bit, x^3+x+1 is maximal. In right-shift Fibonacci, taps = bit2 | bit0.
        let l = Lfsr::new(3, (1 << 2) | (1 << 0));
        assert!(l.is_maximal_length(1));
        let seq = l.sequence(1);
        assert_eq!(seq.len(), 7);
    }

    #[test]
    fn step_back_inverts_step() {
        let l = lfsr20();
        let mut state = 0xABCDE & l.mask();
        let before = state;
        l.step(&mut state);
        assert_eq!(l.step_back(state), Some(before));
    }

    #[test]
    fn twenty_bit_is_maximal() {
        assert!(lfsr20().is_maximal_length(1));
    }

    #[test]
    fn pack_matches_register_state() {
        // The register at step k must equal the packed next `bits` output bits.
        let l = lfsr20();
        let seed = 1u32;
        let seq = l.sequence(seed);
        let bits = l.bits as usize;
        // Re-run capturing states.
        let mut state = seed;
        for k in 0..1000 {
            let window = &seq[k..k + bits];
            assert_eq!(pack_state(window), state, "mismatch at k={k}");
            l.step(&mut state);
        }
    }

    #[test]
    fn position_map_roundtrip() {
        let l = lfsr20();
        let seed = 1u32;
        let seq = l.sequence(seed);
        let bits = l.bits as usize;
        // Every store must round-trip every sampled window to its position.
        for kind in [
            PositionMapKind::Fast,
            PositionMapKind::Balanced,
            PositionMapKind::Compact,
        ] {
            let pm = PositionMap::build_with_kind(l, seed, kind);
            for &k in &[0usize, 1, 42, 1000, 500_000, (seq.len() - bits)] {
                let window = &seq[k..k + bits];
                assert_eq!(
                    pm.position_of_window(window),
                    Some(k as u32),
                    "kind {kind:?} k={k}"
                );
            }
        }
    }

    #[test]
    fn all_stores_agree() {
        // All four backing stores must give identical lookups. Use a small
        // register so the forced-sorted build is cheap.
        let l = Lfsr::new(15, (1 << 14) | (1 << 0)); // x^15 + x + 1, maximal
        assert!(l.is_maximal_length(1));
        let seed = 1u32;
        let fast = PositionMap::build_with_kind(l, seed, PositionMapKind::Fast);
        let balanced = PositionMap::build_with_kind(l, seed, PositionMapKind::Balanced);
        let compact = PositionMap::build_with_kind(l, seed, PositionMapKind::Compact);
        let sorted = PositionMap::build_sorted(l, seed);
        let seq = l.sequence(seed);
        let bits = l.bits as usize;
        for k in (0..=seq.len() - bits).step_by(97) {
            let w = &seq[k..k + bits];
            let f = fast.position_of_window(w);
            assert_eq!(f, Some(k as u32));
            assert_eq!(f, sorted.position_of_window(w));
            assert_eq!(f, balanced.position_of_window(w));
            assert_eq!(f, compact.position_of_window(w));
        }
        // The all-zero state is never visited by any store.
        assert_eq!(fast.position_of_state(0), None);
        assert_eq!(sorted.position_of_state(0), None);
        assert_eq!(balanced.position_of_state(0), None);
        assert_eq!(compact.position_of_state(0), None);
    }

    #[test]
    fn dlog_stores_match_fast_on_measured_serato() {
        // The discrete-log stores must be bit-exact against the fast table on the
        // real 20-bit side-A and side-B polynomials (the confirmed params).
        for (taps, seed) in [(0x361e5u32, 0xafd8eu32), (0x4f0d9, 0x9a9a2)] {
            let l = Lfsr::new(20, taps);
            assert!(l.is_maximal_length(seed));
            let fast = PositionMap::build_with_kind(l, seed, PositionMapKind::Fast);
            let balanced = PositionMap::build_with_kind(l, seed, PositionMapKind::Balanced);
            let compact = PositionMap::build_with_kind(l, seed, PositionMapKind::Compact);
            // Confirm each really built its intended store, not a fallback.
            assert!(matches!(balanced.store, Store::Bsgs(_)));
            assert!(matches!(compact.store, Store::Dlog(_)));
            let seq = l.sequence(seed);
            let bits = l.bits as usize;
            for k in (0..=seq.len() - bits).step_by(2011) {
                let w = &seq[k..k + bits];
                let f = fast.position_of_window(w);
                assert_eq!(f, Some(k as u32), "taps={taps:#x} k={k}");
                assert_eq!(
                    balanced.position_of_window(w),
                    f,
                    "balanced taps={taps:#x} k={k}"
                );
                assert_eq!(
                    compact.position_of_window(w),
                    f,
                    "compact taps={taps:#x} k={k}"
                );
            }
            assert_eq!(balanced.position_of_state(0), None);
            assert_eq!(compact.position_of_state(0), None);
        }
    }
}
