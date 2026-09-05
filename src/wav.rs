//! Minimal WAV I/O helper (feature `wav`), used by tests and examples. Kept out
//! of the core codec so the decoder itself stays I/O-free.

use std::path::Path;

/// Read a stereo WAV file into `f32` frames in `[-1, 1]` plus its sample rate.
/// Mono files are duplicated to both channels; >2 channels use the first two.
pub fn read_stereo<P: AsRef<Path>>(path: P) -> Result<(Vec<(f32, f32)>, f32), hound::Error> {
    let mut reader = hound::WavReader::open(path)?;
    let spec = reader.spec();
    let ch = spec.channels as usize;
    let sr = spec.sample_rate as f32;

    let samples: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Float => reader.samples::<f32>().collect::<Result<_, _>>()?,
        hound::SampleFormat::Int => {
            let max = (1i64 << (spec.bits_per_sample - 1)) as f32;
            reader
                .samples::<i32>()
                .map(|s| s.map(|v| v as f32 / max))
                .collect::<Result<_, _>>()?
        }
    };

    let mut frames = Vec::with_capacity(samples.len() / ch.max(1));
    match ch {
        0 => {}
        1 => {
            for &s in &samples {
                frames.push((s, s));
            }
        }
        _ => {
            for chunk in samples.chunks(ch) {
                frames.push((chunk[0], chunk[1]));
            }
        }
    }
    Ok((frames, sr))
}

/// Read a segment of a stereo WAV: skip `skip_frames` frames, then read up to
/// `max_frames`. Avoids loading huge files entirely. Returns frames + sample rate.
pub fn read_stereo_segment<P: AsRef<Path>>(
    path: P,
    skip_frames: u64,
    max_frames: usize,
) -> Result<(Vec<(f32, f32)>, f32), hound::Error> {
    let mut reader = hound::WavReader::open(path)?;
    let spec = reader.spec();
    let ch = spec.channels.max(1) as usize;
    let sr = spec.sample_rate as f32;
    // hound seeks in frames (time), not interleaved samples.
    reader.seek(skip_frames.min(u32::MAX as u64) as u32)?;

    let want = max_frames.saturating_mul(ch);
    let mut frames = Vec::with_capacity(max_frames);
    match spec.sample_format {
        hound::SampleFormat::Float => {
            let it = reader.samples::<f32>().take(want);
            let vals: Vec<f32> = it.collect::<Result<_, _>>()?;
            for c in vals.chunks(ch) {
                frames.push((c[0], *c.get(1).unwrap_or(&c[0])));
            }
        }
        hound::SampleFormat::Int => {
            let max = (1i64 << (spec.bits_per_sample - 1)) as f32;
            let it = reader.samples::<i32>().take(want);
            let vals: Vec<f32> = it
                .map(|s| s.map(|v| v as f32 / max))
                .collect::<Result<_, _>>()?;
            for c in vals.chunks(ch) {
                frames.push((c[0], *c.get(1).unwrap_or(&c[0])));
            }
        }
    }
    Ok((frames, sr))
}

/// Write stereo `f32` frames to a 16-bit PCM WAV file.
pub fn write_stereo<P: AsRef<Path>>(
    path: P,
    frames: &[(f32, f32)],
    sample_rate: f32,
) -> Result<(), hound::Error> {
    let spec = hound::WavSpec {
        channels: 2,
        sample_rate: sample_rate as u32,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut writer = hound::WavWriter::create(path, spec)?;
    for &(l, r) in frames {
        let li = (l.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
        let ri = (r.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
        writer.write_sample(li)?;
        writer.write_sample(ri)?;
    }
    writer.finalize()
}
