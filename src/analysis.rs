//! Clean-room parameter recovery (feature `analysis`).
//!
//! These tools derive a format's parameters from a *recording* rather than from
//! any existing implementation:
//!
//! * [`carrier_frequency`] — dominant tone via FFT.
//! * [`channel_phase_offset`] — L/R phase difference (quadrature / lead channel)
//!   via the Goertzel algorithm.
//! * [`berlekamp_massey`] — the minimal linear recurrence (LFSR length + taps)
//!   behind a decoded bit stream.
//!
//! Feed the decoder's sliced bits (from a steady-speed passage) into
//! [`berlekamp_massey`], convert with [`connection_to_lfsr`], and you have the
//! LFSR geometry — measured, not copied.

use crate::lfsr::Lfsr;
use rustfft::{num_complex::Complex, FftPlanner};

/// Estimate the carrier frequency (Hz) from the left channel via FFT. `frames`
/// should contain a steady-speed passage. Returns the frequency of the dominant
/// bin above ~50 Hz.
pub fn carrier_frequency(frames: &[(f32, f32)], sample_rate: f32) -> f32 {
    let n = frames.len().next_power_of_two().min(1 << 20).max(2);
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    let mut buf: Vec<Complex<f32>> = (0..n)
        .map(|i| Complex::new(frames.get(i).map(|f| f.0).unwrap_or(0.0), 0.0))
        .collect();
    fft.process(&mut buf);

    let min_bin = ((50.0 * n as f32) / sample_rate).ceil() as usize;
    let (mut best_bin, mut best_mag) = (min_bin, 0.0f32);
    for (k, c) in buf.iter().enumerate().take(n / 2).skip(min_bin) {
        let m = c.norm_sqr();
        if m > best_mag {
            best_mag = m;
            best_bin = k;
        }
    }
    best_bin as f32 * sample_rate / n as f32
}

/// Goertzel magnitude+phase of a real signal at frequency `f0`.
fn goertzel(signal: impl Iterator<Item = f32>, f0: f32, sample_rate: f32) -> (f32, f32) {
    let w = 2.0 * std::f32::consts::PI * f0 / sample_rate;
    let (cw, sw) = (w.cos(), w.sin());
    let coeff = 2.0 * cw;
    let (mut s1, mut s2) = (0.0f32, 0.0f32);
    for x in signal {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let real = s1 - s2 * cw;
    let imag = s2 * sw;
    ((real * real + imag * imag).sqrt(), imag.atan2(real))
}

/// Phase offset (radians) of the right channel relative to the left at `f0`. A
/// value near +π/2 or −π/2 confirms quadrature; the sign indicates which channel
/// leads.
pub fn channel_phase_offset(frames: &[(f32, f32)], sample_rate: f32, f0: f32) -> f32 {
    let (_, pl) = goertzel(frames.iter().map(|f| f.0), f0, sample_rate);
    let (_, pr) = goertzel(frames.iter().map(|f| f.1), f0, sample_rate);
    let mut d = pr - pl;
    while d > std::f32::consts::PI {
        d -= 2.0 * std::f32::consts::PI;
    }
    while d <= -std::f32::consts::PI {
        d += 2.0 * std::f32::consts::PI;
    }
    d
}

/// Berlekamp–Massey over GF(2). Returns `(L, C)` where `L` is the minimal LFSR
/// length and `C` is the connection polynomial (`C[0] == 1`), satisfying
/// `s[n] = XOR_{i=1..=L, C[i]=1} s[n-i]` for the maximal run it explains.
pub fn berlekamp_massey(s: &[u8]) -> (usize, Vec<u8>) {
    let n = s.len();
    let mut c = vec![0u8; n + 1];
    let mut b = vec![0u8; n + 1];
    c[0] = 1;
    b[0] = 1;
    let mut l = 0usize;
    let mut m = 1usize;

    for i in 0..n {
        // discrepancy
        let mut d = s[i] & 1;
        for j in 1..=l {
            d ^= c[j] & s[i - j];
        }
        if d == 0 {
            m += 1;
        } else if 2 * l <= i {
            let t = c.clone();
            for j in 0..=(n - m) {
                c[j + m] ^= b[j];
            }
            l = i + 1 - l;
            b = t;
            m = 1;
        } else {
            for j in 0..=(n - m) {
                c[j + m] ^= b[j];
            }
            m += 1;
        }
    }
    c.truncate(l + 1);
    (l, c)
}

/// Convert a Berlekamp–Massey connection polynomial to this crate's right-shift
/// Fibonacci [`Lfsr`]. Tap bit `L - i` is set for each `i` with `C[i] == 1`.
pub fn connection_to_lfsr(l: usize, c: &[u8]) -> Lfsr {
    let mut taps = 0u32;
    for i in 1..=l {
        if c.get(i).copied().unwrap_or(0) == 1 {
            let pos = l - i;
            if pos < 32 {
                taps |= 1 << pos;
            }
        }
    }
    Lfsr::new(l as u32, taps)
}

/// Human-readable polynomial string, e.g. `x^20 + x^3 + 1`.
pub fn poly_string(l: usize, c: &[u8]) -> String {
    let mut terms = vec![format!("x^{l}")];
    for i in 1..l {
        if c.get(i).copied().unwrap_or(0) == 1 {
            if i == 1 {
                terms.push("x".to_string());
            } else {
                terms.push(format!("x^{}", l - i));
            }
        }
    }
    terms.push("1".to_string());
    terms.join(" + ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::serato_cv02;
    use crate::synth::Encoder;

    #[test]
    fn bm_recovers_lfsr_from_sequence() {
        let fmt = serato_cv02();
        // Take a chunk of the true output sequence (> 2*L bits).
        let seq = fmt.lfsr.sequence(fmt.seed);
        let sample = &seq[1000..1000 + 200];
        let (l, c) = berlekamp_massey(sample);
        assert_eq!(l, fmt.lfsr.bits as usize, "recovered wrong length");
        let recovered = connection_to_lfsr(l, &c);
        assert_eq!(recovered.taps, fmt.lfsr.taps, "recovered wrong taps");
        assert!(recovered.is_maximal_length(1));
    }

    #[test]
    fn carrier_and_phase_from_synth() {
        let fmt = serato_cv02();
        let sr = 44_100.0;
        let mut enc = Encoder::new(fmt.clone(), sr);
        let mut buf = Vec::new();
        enc.render_const(1.0, 1 << 15, &mut buf);
        let f0 = carrier_frequency(&buf, sr);
        assert!((f0 - fmt.carrier_hz).abs() < 5.0, "f0={f0}");
        // Quadrature: |offset| ~ pi/2. Sign follows the format's lead channel
        // (Right lead => right is ahead => positive offset).
        let d = channel_phase_offset(&buf, sr, fmt.carrier_hz);
        assert!((d.abs() - std::f32::consts::FRAC_PI_2).abs() < 0.3, "d={d}");
        let expect_positive = matches!(fmt.lead, crate::format::LeadChannel::Right);
        assert_eq!(d > 0.0, expect_positive, "phase sign vs lead: d={d}");
    }
}
