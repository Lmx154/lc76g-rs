//! Conversions from the integer units of the decoders to floating point. Pure functions, kept apart so
//! the decode path stays integer-only.

/// 1e-7 degree to degrees. `f64` because `f32` cannot hold 1e-7 degree at 180 degrees.
pub fn degrees(e7: i32) -> f64 {
    f64::from(e7) * 1e-7
}

/// Millimetres to metres.
pub fn metres(mm: i32) -> f32 {
    mm as f32 * 1e-3
}

/// Millimetres per second to metres per second.
pub fn metres_per_second(mm_s: u32) -> f32 {
    mm_s as f32 * 1e-3
}

/// 0.01 degree to degrees.
pub fn course_degrees(cdeg: u16) -> f32 {
    f32::from(cdeg) * 0.01
}

/// DOP x100 to DOP.
pub fn dop(x100: u16) -> f32 {
    f32::from(x100) * 0.01
}
