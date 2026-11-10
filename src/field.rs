//! Field parsers. Integers only, exact arithmetic, round to nearest (halves away from zero).

use crate::sentence::{Date, Time};

/// A non-empty field that does not parse.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Bad;

pub(crate) type Parsed<T> = Result<T, Bad>;

/// Longest digit string accepted in a decimal number; keeps the mantissa inside `u64`.
const MAX_DIGITS: usize = 18;

/// A decimal number as `mant / 10^dec`.
#[derive(Clone, Copy)]
pub(crate) struct Decimal {
    neg: bool,
    mant: u64,
    dec: u32,
}

impl Decimal {
    pub(crate) fn parse(s: &str) -> Option<Self> {
        let (neg, digits) = match s.as_bytes().first()? {
            b'-' => (true, &s[1..]),
            b'+' => (false, &s[1..]),
            _ => (false, s),
        };
        let (int, frac) = digits.split_once('.').unwrap_or((digits, ""));
        if int.len() + frac.len() == 0 || int.len() + frac.len() > MAX_DIGITS {
            return None;
        }
        let mant = digits_value(int)?
            .checked_mul(10u64.pow(frac.len() as u32))?
            .checked_add(digits_value(frac)?)?;
        Some(Self {
            neg,
            mant,
            dec: frac.len() as u32,
        })
    }

    /// `self * num / den`, rounded to the nearest integer, halves away from zero.
    ///
    /// Common powers of ten of `num` and `10^dec` cancel first. The quotient is then computed in 32
    /// bits when numerator and denominator fit, else in 64 bits, and in 128 bits only when a product
    /// overflows 64 bits (128-bit division is a slow software routine on Cortex-M). All paths give the
    /// same result: `floor(n / d + 1/2)` of the same fraction.
    pub(crate) fn scaled(self, mut num: u64, den: u64) -> Option<i64> {
        let mut dec = self.dec;
        while dec > 0 && num.is_multiple_of(10) {
            num /= 10;
            dec -= 1;
        }
        let pow = 10u64.pow(dec);
        let q = match (self.mant.checked_mul(num), den.checked_mul(pow)) {
            (Some(n), Some(d)) => match (u32::try_from(n), u32::try_from(d)) {
                (Ok(n), Ok(d)) => {
                    let (q, r) = (n / d, n % d);
                    u64::from(q) + u64::from(r >= d - r)
                }
                _ => {
                    let (q, r) = (n / d, n % d);
                    q + u64::from(r >= d - r)
                }
            },
            _ => {
                let n = u128::from(self.mant) * u128::from(num);
                let d = u128::from(den) * u128::from(pow);
                u64::try_from((2 * n + d) / (2 * d)).ok()?
            }
        };
        let q = i64::try_from(q).ok()?;
        Some(if self.neg { -q } else { q })
    }
}

/// Value of an all-digit string (empty is 0); `None` on any other character or overflow.
fn digits_value(s: &str) -> Option<u64> {
    s.bytes().try_fold(0u64, |acc, b| {
        if b.is_ascii_digit() {
            acc.checked_mul(10)?.checked_add(u64::from(b - b'0'))
        } else {
            None
        }
    })
}

fn opt<T>(s: &str, f: impl FnOnce(&str) -> Option<T>) -> Parsed<Option<T>> {
    if s.is_empty() {
        Ok(None)
    } else {
        f(s).map(Some).ok_or(Bad)
    }
}

/// Plain unsigned integer.
pub(crate) fn uint<T: TryFrom<u64>>(s: &str) -> Parsed<Option<T>> {
    opt(s, |s| {
        if s.is_empty() {
            return None;
        }
        T::try_from(digits_value(s)?).ok()
    })
}

/// Unsigned integer that must be present.
pub(crate) fn uint_req<T: TryFrom<u64>>(s: &str) -> Parsed<T> {
    uint(s)?.ok_or(Bad)
}

/// Decimal number times `num / den`, as a signed integer of type `T`.
pub(crate) fn scaled<T: TryFrom<i64>>(s: &str, num: u64, den: u64) -> Parsed<Option<T>> {
    opt(s, |s| {
        let v = Decimal::parse(s)?.scaled(num, den)?;
        T::try_from(v).ok()
    })
}

/// Metres to millimetres.
pub(crate) fn metres_mm(s: &str) -> Parsed<Option<i32>> {
    scaled(s, 1000, 1)
}

/// Dimensionless value to hundredths (DOP).
pub(crate) fn centi_u16(s: &str) -> Parsed<Option<u16>> {
    scaled(s, 100, 1)
}

/// Knots to mm/s: 1 kn = 1852/3600 m/s = 4630/9 mm/s.
pub(crate) fn knots_mm_s(s: &str) -> Parsed<Option<u32>> {
    scaled(s, 4630, 9)
}

/// km/h to mm/s: 1 km/h = 1000000/3600 mm/s = 2500/9 mm/s.
pub(crate) fn kmh_mm_s(s: &str) -> Parsed<Option<u32>> {
    scaled(s, 2500, 9)
}

/// Course in degrees to hundredths of a degree, 0 to 360.
pub(crate) fn course_cdeg(s: &str) -> Parsed<Option<u16>> {
    let v: Option<u16> = scaled(s, 100, 1)?;
    match v {
        Some(v) if v > 36_000 => Err(Bad),
        v => Ok(v),
    }
}

/// `ddmm.mmmm` / `dddmm.mmmm` plus hemisphere letter to 1e-7 degree.
fn coordinate(
    value: &str,
    hemisphere: &str,
    deg_digits: usize,
    max_deg: u64,
    positive: u8,
    negative: u8,
) -> Parsed<Option<i32>> {
    if value.is_empty() || hemisphere.is_empty() {
        return Ok(None);
    }
    let south_west = match hemisphere.as_bytes() {
        [h] if *h == positive => false,
        [h] if *h == negative => true,
        _ => return Err(Bad),
    };
    let (int, frac) = value.split_once('.').unwrap_or((value, ""));
    if int.len() != deg_digits + 2 || frac.len() > 9 {
        return Err(Bad);
    }
    let deg = digits_value(&int[..deg_digits]).ok_or(Bad)?;
    let min = digits_value(&int[deg_digits..]).ok_or(Bad)?;
    let frac_value = digits_value(frac).ok_or(Bad)?;
    if min >= 60 || deg > max_deg || (deg == max_deg && (min != 0 || frac_value != 0)) {
        return Err(Bad);
    }
    let dec = frac.len() as u32;
    let total = Decimal {
        neg: false,
        mant: (deg * 60 + min) * 10u64.pow(dec) + frac_value,
        dec,
    };
    let e7 = total.scaled(10_000_000, 60).ok_or(Bad)?;
    let e7 = i32::try_from(e7).map_err(|_| Bad)?;
    Ok(Some(if south_west { -e7 } else { e7 }))
}

/// Latitude `ddmm.mmmmmm` and `N`/`S` to 1e-7 degree.
pub(crate) fn latitude(value: &str, hemisphere: &str) -> Parsed<Option<i32>> {
    coordinate(value, hemisphere, 2, 90, b'N', b'S')
}

/// Longitude `dddmm.mmmmmm` and `E`/`W` to 1e-7 degree.
pub(crate) fn longitude(value: &str, hemisphere: &str) -> Parsed<Option<i32>> {
    coordinate(value, hemisphere, 3, 180, b'E', b'W')
}

/// `hhmmss.sss`. Fractions are truncated to milliseconds.
pub(crate) fn time(s: &str) -> Parsed<Option<Time>> {
    opt(s, |s| {
        let (int, frac) = s.split_once('.').unwrap_or((s, ""));
        if int.len() != 6 || frac.len() > 9 {
            return None;
        }
        let hour = digits_value(&int[0..2])?;
        let minute = digits_value(&int[2..4])?;
        let second = digits_value(&int[4..6])?;
        let mut millis = digits_value(frac.get(..frac.len().min(3))?)?;
        for _ in frac.len().min(3)..3 {
            millis *= 10;
        }
        digits_value(frac)?;
        if hour > 23 || minute > 59 || second > 60 {
            return None;
        }
        Some(Time {
            hour: hour as u8,
            minute: minute as u8,
            second: second as u8,
            millis: millis as u16,
        })
    })
}

/// `ddmmyy`, year 2000 + yy.
pub(crate) fn date(s: &str) -> Parsed<Option<Date>> {
    opt(s, |s| {
        if s.len() != 6 {
            return None;
        }
        let day = digits_value(&s[0..2])?;
        let month = digits_value(&s[2..4])?;
        let year = digits_value(&s[4..6])?;
        if !(1..=31).contains(&day) || !(1..=12).contains(&month) {
            return None;
        }
        Some(Date {
            year: 2000 + year as u16,
            month: month as u8,
            day: day as u8,
        })
    })
}

#[cfg(test)]
mod tests {
    use super::Decimal;

    /// The straightforward computation: everything in 128 bits.
    fn reference(d: Decimal, num: u64, den: u64) -> Option<i64> {
        let n = u128::from(d.mant) * u128::from(num);
        let m = u128::from(den) * 10u128.pow(d.dec);
        let q = i64::try_from((2 * n + m) / (2 * m)).ok()?;
        Some(if d.neg { -q } else { q })
    }

    #[test]
    fn scaled_matches_the_128_bit_reference() {
        let scales = [
            (1000, 1),
            (100, 1),
            (4630, 9),
            (2500, 9),
            (10_000_000, 60),
            (1, 1),
            (7, 3),
            (1_000_000_000_000, 1),
        ];
        let mut mants = vec![0u64, 1, 2, 5, 9, 10, 99, 100, 4_294_967_295, 4_294_967_296];
        mants.extend(0..3000);
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        for bits in 1..=60 {
            for _ in 0..40 {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                mants.push(x >> (64 - bits));
            }
        }
        mants.push(999_999_999_999_999_999);
        let mut checked = 0u32;
        for dec in 0..=18u32 {
            let half_marks =
                [5u64, 25, 15, 125, 375].map(|k| k.wrapping_mul(10u64.pow(dec.saturating_sub(1))));
            for &(num, den) in &scales {
                for &mant in mants.iter().chain(&half_marks) {
                    let mant = mant % 1_000_000_000_000_000_000;
                    for neg in [false, true] {
                        let d = Decimal { neg, mant, dec };
                        assert_eq!(
                            d.scaled(num, den),
                            reference(d, num, den),
                            "mant {mant} dec {dec} num {num} den {den} neg {neg}"
                        );
                        checked += 1;
                    }
                }
            }
        }
        assert!(checked > 100_000);
    }
}
