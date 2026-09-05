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

/// Widest register for which the direct-indexed table is used. At 24 bits the
/// table is `2^24 * 4 bytes = 64 MiB`; wider registers fall back to a compact
/// sorted array with binary-search lookup.
const DIRECT_TABLE_MAX_BITS: u32 = 24;

/// Sentinel marking an unpopulated slot in the direct-indexed table (the LFSR
/// never visits the all-zero state, and no valid position equals `u32::MAX`).
const EMPTY: u32 = u32::MAX;

/// Lookup backing for [`PositionMap`], chosen by register width.
enum Store {
    /// `table[state] = position`, or [`EMPTY`]. O(1), no hashing. Used for
    /// registers up to [`DIRECT_TABLE_MAX_BITS`] wide.
    Direct(Vec<u32>),
    /// `(state, position)` pairs sorted by state, searched with `binary_search`.
    /// Used for wider registers where a direct table would be too large.
    Sorted(Vec<(u32, u32)>),
}

/// Maps every reachable LFSR state to its absolute index in the output sequence.
///
/// Index `k` is the position such that `state_k = pack(b_k … b_{k+bits-1})`.
///
/// The backing store is chosen by register width: a direct-indexed table
/// (`state → position`) for registers up to 24 bits — O(1) lookup, no hashing,
/// ~4 MiB for a 20-bit register — or a compact sorted array with binary search
/// for wider registers. Either way memory is O(period) and lookup is allocation-
/// free.
pub struct PositionMap {
    lfsr: Lfsr,
    seed: u32,
    store: Store,
}

impl PositionMap {
    /// Build the map by walking the whole sequence from `seed`. O(period) time
    /// and memory (~4 MiB for a 20-bit register).
    pub fn build(lfsr: Lfsr, seed: u32) -> Self {
        let period = (1u64 << lfsr.bits) - 1;
        let mask = lfsr.mask();
        let seed = seed & mask;

        let store = if lfsr.bits <= DIRECT_TABLE_MAX_BITS {
            // Direct-indexed: table[state] = position. The all-zero state is
            // never visited, so leaving it EMPTY is correct.
            let mut table = Vec::new();
            table.resize(1usize << lfsr.bits, EMPTY);
            let mut state = seed;
            for k in 0..period {
                if table[state as usize] == EMPTY {
                    table[state as usize] = k as u32;
                }
                lfsr.step(&mut state);
            }
            Store::Direct(table)
        } else {
            // Sorted array: the walk visits states in position order, so collect
            // (state, position) then sort by state for binary search.
            let mut pairs = Vec::with_capacity(period as usize);
            let mut state = seed;
            for k in 0..period {
                pairs.push((state, k as u32));
                lfsr.step(&mut state);
            }
            pairs.sort_unstable_by_key(|&(s, _)| s);
            // A maximal-length LFSR visits each nonzero state once, so there are
            // no duplicate keys to dedup.
            Store::Sorted(pairs)
        };

        PositionMap { lfsr, seed, store }
    }

    /// Test-only: build using the sorted-array store regardless of register
    /// width, so the binary-search path can be exercised on a small register.
    #[cfg(test)]
    fn build_sorted(lfsr: Lfsr, seed: u32) -> Self {
        let period = (1u64 << lfsr.bits) - 1;
        let seed = seed & lfsr.mask();
        let mut pairs = Vec::with_capacity(period as usize);
        let mut state = seed;
        for k in 0..period {
            pairs.push((state, k as u32));
            lfsr.step(&mut state);
        }
        pairs.sort_unstable_by_key(|&(s, _)| s);
        PositionMap { lfsr, seed, store: Store::Sorted(pairs) }
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
            Store::Direct(table) => match table[state as usize] {
                EMPTY => None,
                pos => Some(pos),
            },
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
        let pm = PositionMap::build(l, seed);
        let seq = l.sequence(seed);
        let bits = l.bits as usize;
        for &k in &[0usize, 1, 42, 1000, 500_000, (seq.len() - bits)] {
            let window = &seq[k..k + bits];
            assert_eq!(pm.position_of_window(window), Some(k as u32));
        }
    }

    #[test]
    fn direct_and_sorted_stores_agree() {
        // The two backing stores must give identical lookups. Use a small
        // register so the forced-sorted build is cheap.
        let l = Lfsr::new(15, (1 << 14) | (1 << 0)); // x^15 + x + 1, maximal
        assert!(l.is_maximal_length(1));
        let seed = 1u32;
        let direct = PositionMap::build(l, seed);
        let sorted = PositionMap::build_sorted(l, seed);
        let seq = l.sequence(seed);
        let bits = l.bits as usize;
        for k in (0..=seq.len() - bits).step_by(97) {
            let w = &seq[k..k + bits];
            let d = direct.position_of_window(w);
            assert_eq!(d, Some(k as u32));
            assert_eq!(d, sorted.position_of_window(w));
        }
        // The all-zero state is never visited by either store.
        assert_eq!(direct.position_of_state(0), None);
        assert_eq!(sorted.position_of_state(0), None);
    }
}
