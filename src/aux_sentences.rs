//! Decoders for ZDA, GNS, GST and GRS (specification sections 2.2.7 to 2.2.10, printed pages 24 to 31).
//!
//! Same conventions as [`crate::sentence`]: integers only, a field the module leaves empty decodes to
//! `None`, the talker is kept. Metres become millimetres, degrees of orientation become hundredths of a
//! degree, DOP becomes DOP x100.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

use crate::field::{self, Bad, Parsed};
use crate::sentence::{Date, Message, PositionMode, Talker, Time};

/// Number of constellation slots in the GNS mode indicator (section 2.2.8 note 1, page 28: GPS,
/// GLONASS, Galileo, BDS, QZSS, NavIC).
pub const GNS_MODE_SLOTS: usize = 6;

/// Number of residuals in a GRS sentence (section 2.2.10, page 30: "Repeat times: 12").
pub const GRS_RESIDUALS: usize = 12;

/// Largest absolute residual of GRS, metres (section 2.2.10, page 30: "Range: -999 to 999").
const GRS_MAX_RESIDUAL_MM: u32 = 999_000;

/// Largest absolute local-zone hour of ZDA (section 2.2.7, page 25: "00 to +-13 hours").
const ZDA_MAX_LOCAL_HOUR: i8 = 13;

/// Largest local-zone minute of ZDA (section 2.2.7, page 25: "00 to +59 minutes").
const ZDA_MAX_LOCAL_MIN: u8 = 59;

/// ZDA: time and date (section 2.2.7, pages 24 and 25).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Zda {
    /// Talker.
    pub talker: Talker,
    /// `<UTC>`: UTC time.
    pub time: Option<Time>,
    /// `<Day>`, `<Month>`, `<Year>` (page 25): UTC date, full year as printed. The three fields are
    /// all present or all empty; a partly filled date is malformed.
    pub date: Option<Date>,
    /// `<LocalHour>` (page 25): local zone hours, -13 to 13. The module does not support it and leaves
    /// it empty. A `-00` cannot be told from `00`.
    pub local_hour: Option<i8>,
    /// `<LocalMin>` (page 25): local zone minutes, 0 to 59. Not supported by the module either.
    pub local_min: Option<u8>,
}

impl Zda {
    /// UTC date and time as milliseconds since 1970-01-01 00:00:00, or `None` when either is empty or
    /// the day does not exist in the month. A leap second (`second` 60) rolls into the next day.
    pub fn utc_unix_ms(&self) -> Option<i64> {
        let (date, time) = (self.date?, self.time?);
        if date.day > days_in_month(date.year, date.month) {
            return None;
        }
        let days = days_from_civil(
            i32::from(date.year),
            i32::from(date.month),
            i32::from(date.day),
        );
        Some(i64::from(days) * 86_400_000 + i64::from(time.millis_of_day()))
    }

    /// Local zone offset in minutes, the sign of the hour applying to the minutes; `None` when the
    /// hour is empty, a missing minute counting as 0.
    pub fn local_offset_minutes(&self) -> Option<i16> {
        let hour = i16::from(self.local_hour?);
        let minute = i16::from(self.local_min.unwrap_or(0));
        Some(if hour < 0 {
            hour * 60 - minute
        } else {
            hour * 60 + minute
        })
    }
}

fn days_in_month(year: u16, month: u8) -> u8 {
    match month {
        2 if year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400)) => {
            29
        }
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// Days since 1970-01-01 of a proleptic Gregorian date (month 1 to 12, year 0 or more).
fn days_from_civil(year: i32, month: i32, day: i32) -> i32 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let month_from_march = (month + 9) % 12;
    let day_of_year = (153 * month_from_march + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

/// GNS mode indicator (section 2.2.8, `<ModeInd>`, pages 27 and 28 note 1): one character per
/// constellation, a variable number of them. Slot `i` is `None` when the sentence has fewer than
/// `i + 1` characters, so the field is kept without loss up to [`GNS_MODE_SLOTS`] characters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ConstellationModes {
    /// The characters in order: GPS, GLONASS, Galileo, BDS, QZSS, NavIC.
    pub modes: [Option<PositionMode>; GNS_MODE_SLOTS],
}

impl ConstellationModes {
    fn slot(&self, index: usize) -> Option<PositionMode> {
        self.modes.get(index).copied().flatten()
    }

    /// Mode of the GPS satellites (first character).
    pub fn gps(&self) -> Option<PositionMode> {
        self.slot(0)
    }

    /// Mode of the GLONASS satellites (second character).
    pub fn glonass(&self) -> Option<PositionMode> {
        self.slot(1)
    }

    /// Mode of the Galileo satellites (third character).
    pub fn galileo(&self) -> Option<PositionMode> {
        self.slot(2)
    }

    /// Mode of the BDS satellites (fourth character).
    pub fn bds(&self) -> Option<PositionMode> {
        self.slot(3)
    }

    /// Mode of the QZSS satellites (fifth character).
    pub fn qzss(&self) -> Option<PositionMode> {
        self.slot(4)
    }

    /// Mode of the NavIC satellites (sixth character).
    pub fn navic(&self) -> Option<PositionMode> {
        self.slot(5)
    }

    /// Number of characters the sentence carried.
    pub fn reported(&self) -> usize {
        self.modes.iter().take_while(|m| m.is_some()).count()
    }
}

/// GNS: GNSS fix data (section 2.2.8, pages 25 to 28).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gns {
    /// Talker.
    pub talker: Talker,
    /// `<UTC>` (page 26): UTC time of the fix.
    pub time: Option<Time>,
    /// `<Lat>`, `<N/S>` (page 26): latitude, 1e-7 degree, south negative.
    pub lat_e7: Option<i32>,
    /// `<Lon>`, `<E/W>` (page 26): longitude, 1e-7 degree, west negative.
    pub lon_e7: Option<i32>,
    /// `<ModeInd>` (page 27): one mode per constellation; `None` when the field is empty.
    pub modes: Option<ConstellationModes>,
    /// `<NumSatUsed>` (page 27): satellites in use, 0 to 99.
    pub sats_used: Option<u8>,
    /// `<HDOP>` (page 27): horizontal DOP x100.
    pub hdop_x100: Option<u16>,
    /// `<Alt>` (page 27): altitude above mean sea level, mm.
    pub alt_mm: Option<i32>,
    /// `<Sep>` (page 27): geoid separation, mm.
    pub geoid_sep_mm: Option<i32>,
    /// `<DiffAge>` (page 28): age of the differential data in milliseconds. Not supported by the
    /// module, which leaves it empty.
    pub diff_age_ms: Option<u32>,
    /// `<DiffStation>` (page 28): differential reference station ID. Not supported, empty.
    pub diff_station: Option<u16>,
    /// `<NavStatus>` (page 28): navigational status character, always `V` on this module.
    pub nav_status: Option<u8>,
}

/// GST: pseudorange error statistics (section 2.2.9, pages 28 and 29). Standard deviations are in
/// millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gst {
    /// Talker.
    pub talker: Talker,
    /// `<UTC>` (page 29): UTC time of the GGA or GNS fix this belongs to.
    pub time: Option<Time>,
    /// `<RMS_D>` (page 29): RMS of the standard deviation of the range inputs, mm.
    pub rms_mm: Option<u32>,
    /// `<MajorD>` (page 29): standard deviation of the semi-major axis of the error ellipse, mm.
    pub semi_major_mm: Option<u32>,
    /// `<MinorD>` (page 29): standard deviation of the semi-minor axis of the error ellipse, mm.
    pub semi_minor_mm: Option<u32>,
    /// `<Orient>` (page 29): orientation of the semi-major axis, 0.01 degree. The specification gives
    /// no range.
    pub orientation_cdeg: Option<i32>,
    /// `<LatD>` (page 29): standard deviation of the latitude error, mm.
    pub lat_std_mm: Option<u32>,
    /// `<LonD>` (page 29): standard deviation of the longitude error, mm.
    pub lon_std_mm: Option<u32>,
    /// `<AltD>` (page 29): standard deviation of the altitude error, mm.
    pub alt_std_mm: Option<u32>,
}

/// GRS residual calculation mode (section 2.2.10, `<Mode>`, page 30).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResidualMode {
    /// 0: the residuals were used to calculate the position in the matching GGA or GNS.
    UsedForFix,
    /// 1: the residuals were recomputed after that position was computed.
    Recomputed,
}

/// GRS: range residuals (section 2.2.10, pages 29 to 31).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Grs {
    /// Talker.
    pub talker: Talker,
    /// `<UTC>` (page 30): UTC time of the fix.
    pub time: Option<Time>,
    /// `<Mode>` (page 30).
    pub mode: ResidualMode,
    /// `<Resi>` (page 30): the twelve range residuals, mm, -999 m to 999 m; `None` for an empty slot.
    pub residuals_mm: [Option<i32>; GRS_RESIDUALS],
    /// `<SystemID>` (page 30): GNSS system ID, NMEA 4.10 and later; `None` when absent or empty.
    pub system_id: Option<u8>,
    /// `<SignalID>` (page 30): GNSS signal ID, NMEA 4.10 and later; `None` when absent or empty.
    pub signal_id: Option<u8>,
}

impl Grs {
    /// Number of residual slots that carry a value.
    pub fn residual_count(&self) -> usize {
        self.residuals_mm.iter().flatten().count()
    }
}

fn at<'a>(f: &[&'a str], index: usize) -> &'a str {
    f.get(index).copied().unwrap_or("")
}

fn unit_metre(s: &str) -> Parsed<()> {
    if s.is_empty() || s == "M" {
        Ok(())
    } else {
        Err(Bad)
    }
}

fn std_dev_mm(s: &str) -> Parsed<Option<u32>> {
    field::scaled(s, 1000, 1)
}

fn local_hour(s: &str) -> Parsed<Option<i8>> {
    if s.is_empty() {
        return Ok(None);
    }
    let (negative, digits) = match s.strip_prefix('-') {
        Some(digits) => (true, digits),
        None => (false, s.strip_prefix('+').unwrap_or(s)),
    };
    let hour = field::uint::<i8>(digits)?.ok_or(Bad)?;
    if hour > ZDA_MAX_LOCAL_HOUR {
        return Err(Bad);
    }
    Ok(Some(if negative { -hour } else { hour }))
}

fn local_min(s: &str) -> Parsed<Option<u8>> {
    match field::uint::<u8>(s)? {
        Some(minute) if minute > ZDA_MAX_LOCAL_MIN => Err(Bad),
        minute => Ok(minute),
    }
}

fn date_fields(day: &str, month: &str, year: &str) -> Parsed<Option<Date>> {
    match (
        field::uint::<u8>(day)?,
        field::uint::<u8>(month)?,
        field::uint::<u16>(year)?,
    ) {
        (None, None, None) => Ok(None),
        (Some(day), Some(month), Some(year))
            if (1..=31).contains(&day) && (1..=12).contains(&month) && year <= 9999 =>
        {
            Ok(Some(Date { year, month, day }))
        }
        _ => Err(Bad),
    }
}

fn mode_indicators(s: &str) -> Parsed<Option<ConstellationModes>> {
    if s.is_empty() {
        return Ok(None);
    }
    if s.len() > GNS_MODE_SLOTS || !s.is_ascii() {
        return Err(Bad);
    }
    let mut modes = [None; GNS_MODE_SLOTS];
    for (slot, index) in modes.iter_mut().zip(0..s.len()) {
        *slot = PositionMode::parse(s.get(index..=index).ok_or(Bad)?)?;
    }
    Ok(Some(ConstellationModes { modes }))
}

pub(crate) fn zda(talker: Talker, f: &[&str]) -> Parsed<Message> {
    if f.len() != 6 {
        return Err(Bad);
    }
    Ok(Message::Zda(Zda {
        talker,
        time: field::time(at(f, 0))?,
        date: date_fields(at(f, 1), at(f, 2), at(f, 3))?,
        local_hour: local_hour(at(f, 4))?,
        local_min: local_min(at(f, 5))?,
    }))
}

pub(crate) fn gns(talker: Talker, f: &[&str]) -> Parsed<Message> {
    if !(14..=15).contains(&f.len()) {
        return Err(Bad);
    }
    unit_metre(at(f, 9))?;
    unit_metre(at(f, 11))?;
    Ok(Message::Gns(Gns {
        talker,
        time: field::time(at(f, 0))?,
        lat_e7: field::latitude(at(f, 1), at(f, 2))?,
        lon_e7: field::longitude(at(f, 3), at(f, 4))?,
        modes: mode_indicators(at(f, 5))?,
        sats_used: field::uint(at(f, 6))?,
        hdop_x100: field::centi_u16(at(f, 7))?,
        alt_mm: field::metres_mm(at(f, 8))?,
        geoid_sep_mm: field::metres_mm(at(f, 10))?,
        diff_age_ms: field::scaled(at(f, 12), 1000, 1)?,
        diff_station: field::uint(at(f, 13))?,
        nav_status: match at(f, 14).as_bytes() {
            [] => None,
            [c] => Some(*c),
            _ => return Err(Bad),
        },
    }))
}

pub(crate) fn gst(talker: Talker, f: &[&str]) -> Parsed<Message> {
    if f.len() != 8 {
        return Err(Bad);
    }
    Ok(Message::Gst(Gst {
        talker,
        time: field::time(at(f, 0))?,
        rms_mm: std_dev_mm(at(f, 1))?,
        semi_major_mm: std_dev_mm(at(f, 2))?,
        semi_minor_mm: std_dev_mm(at(f, 3))?,
        orientation_cdeg: field::scaled(at(f, 4), 100, 1)?,
        lat_std_mm: std_dev_mm(at(f, 5))?,
        lon_std_mm: std_dev_mm(at(f, 6))?,
        alt_std_mm: std_dev_mm(at(f, 7))?,
    }))
}

pub(crate) fn grs(talker: Talker, f: &[&str]) -> Parsed<Message> {
    let with_ids = match f.len() {
        n if n == 2 + GRS_RESIDUALS => false,
        n if n == 4 + GRS_RESIDUALS => true,
        _ => return Err(Bad),
    };
    let mode = match field::uint_req::<u8>(at(f, 1))? {
        0 => ResidualMode::UsedForFix,
        1 => ResidualMode::Recomputed,
        _ => return Err(Bad),
    };
    let mut residuals_mm = [None; GRS_RESIDUALS];
    for (slot, index) in residuals_mm.iter_mut().zip(2..) {
        *slot = match field::metres_mm(at(f, index))? {
            Some(mm) if mm.unsigned_abs() > GRS_MAX_RESIDUAL_MM => return Err(Bad),
            mm => mm,
        };
    }
    let (system_id, signal_id) = if with_ids {
        (
            field::uint(at(f, 2 + GRS_RESIDUALS))?,
            field::uint(at(f, 3 + GRS_RESIDUALS))?,
        )
    } else {
        (None, None)
    };
    Ok(Message::Grs(Grs {
        talker,
        time: field::time(at(f, 0))?,
        mode,
        residuals_mm,
        system_id,
        signal_id,
    }))
}
