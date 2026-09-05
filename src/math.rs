//! `no_std` float-math shim.
//!
//! `core` does not provide the transcendental / rounding float methods
//! (`sin`, `cos`, `atan2`, `sqrt`, `powf`, `floor`, …) that live in `std` — in a
//! hosted build those link against the platform libm. On a `no_std` target (e.g.
//! an ESP32) there is no libm to link, so we supply the same methods via the
//! pure-Rust [`libm`] crate through the [`FloatExt`] extension trait.
//!
//! This module is compiled **only** when the `std` feature is off. Under `std`
//! the inherent methods win and this shim is absent, so hosted numeric behavior
//! is exactly as before (and can use hardware/std math). Call sites import the
//! trait under the same `cfg`, so the source is identical either way.
//!
//! Methods that `core` already provides without libm — `min`, `max`, `clamp`,
//! `is_finite`, `is_nan` — are intentionally *not* in the trait; they resolve to
//! the inherent `core` methods in every build.

/// `std` float methods reimplemented via `libm`, for `no_std` builds.
///
/// Implemented for both `f32` and `f64`. Where an inherent `core` method of the
/// same name exists on a given toolchain it takes priority (inherent methods
/// always beat trait methods), so importing this trait never changes behavior on
/// targets where `core` already has the method — it only fills the gaps.
///
/// Some methods (notably `abs`) are inherent in `core` on current toolchains, so
/// the trait version goes unused there; others are only exercised with the
/// `synth` feature. The trait stays complete regardless, so it still fills the
/// gap on toolchains that lack a given method — hence `allow(dead_code)`.
#[allow(dead_code)]
pub trait FloatExt {
    fn abs(self) -> Self;
    fn sqrt(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn atan2(self, other: Self) -> Self;
    fn powf(self, n: Self) -> Self;
    fn floor(self) -> Self;
    fn round(self) -> Self;
    fn trunc(self) -> Self;
    fn div_euclid(self, rhs: Self) -> Self;
}

impl FloatExt for f32 {
    #[inline]
    fn abs(self) -> f32 {
        libm::fabsf(self)
    }
    #[inline]
    fn sqrt(self) -> f32 {
        libm::sqrtf(self)
    }
    #[inline]
    fn sin(self) -> f32 {
        libm::sinf(self)
    }
    #[inline]
    fn cos(self) -> f32 {
        libm::cosf(self)
    }
    #[inline]
    fn atan2(self, other: f32) -> f32 {
        libm::atan2f(self, other)
    }
    #[inline]
    fn powf(self, n: f32) -> f32 {
        libm::powf(self, n)
    }
    #[inline]
    fn floor(self) -> f32 {
        libm::floorf(self)
    }
    #[inline]
    fn round(self) -> f32 {
        libm::roundf(self)
    }
    #[inline]
    fn trunc(self) -> f32 {
        libm::truncf(self)
    }
    #[inline]
    fn div_euclid(self, rhs: f32) -> f32 {
        // Matches the semantics of the std `f32::div_euclid`.
        let q = libm::truncf(self / rhs);
        if self - q * rhs < 0.0 {
            if rhs > 0.0 {
                q - 1.0
            } else {
                q + 1.0
            }
        } else {
            q
        }
    }
}

impl FloatExt for f64 {
    #[inline]
    fn abs(self) -> f64 {
        libm::fabs(self)
    }
    #[inline]
    fn sqrt(self) -> f64 {
        libm::sqrt(self)
    }
    #[inline]
    fn sin(self) -> f64 {
        libm::sin(self)
    }
    #[inline]
    fn cos(self) -> f64 {
        libm::cos(self)
    }
    #[inline]
    fn atan2(self, other: f64) -> f64 {
        libm::atan2(self, other)
    }
    #[inline]
    fn powf(self, n: f64) -> f64 {
        libm::pow(self, n)
    }
    #[inline]
    fn floor(self) -> f64 {
        libm::floor(self)
    }
    #[inline]
    fn round(self) -> f64 {
        libm::round(self)
    }
    #[inline]
    fn trunc(self) -> f64 {
        libm::trunc(self)
    }
    #[inline]
    fn div_euclid(self, rhs: f64) -> f64 {
        // Matches the semantics of the std `f64::div_euclid`.
        let q = libm::trunc(self / rhs);
        if self - q * rhs < 0.0 {
            if rhs > 0.0 {
                q - 1.0
            } else {
                q + 1.0
            }
        } else {
            q
        }
    }
}
