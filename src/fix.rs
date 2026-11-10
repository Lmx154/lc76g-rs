//! [`FixBuilder`]: assembles one [`Fix`] from the GGA and RMC of the same epoch plus the latest GSA.

use crate::sentence::{Date, FixType, Gga, Message, Quality, Rmc, Time};
use crate::units;

/// One position solution.
///
/// Time, date, validity, quality, coordinates, altitude, geoid separation, speed, course, satellite
/// count and HDOP come from the GGA and RMC that carry the same UTC time. `fix_type`, `pdop_x100` and
/// `vdop_x100` come from the most recent GSA seen before the fix completed. Above 1 Hz a valid
/// [`Config`](crate::Config) sends GSA at most once per second, so these three can be up to a second
/// older than the rest; they are `None` until a GSA has been seen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fix {
    /// UTC time of the epoch.
    pub time: Option<Time>,
    /// UTC date (RMC).
    pub date: Option<Date>,
    /// RMC status: true for `A`.
    pub valid: bool,
    /// GGA quality indicator.
    pub quality: Quality,
    /// Latitude, 1e-7 degree, south negative.
    pub lat_e7: Option<i32>,
    /// Longitude, 1e-7 degree, west negative.
    pub lon_e7: Option<i32>,
    /// Altitude above mean sea level, mm.
    pub alt_mm: Option<i32>,
    /// Geoid separation, mm.
    pub geoid_sep_mm: Option<i32>,
    /// Ground speed, mm/s.
    pub speed_mm_s: Option<u32>,
    /// Course over ground, 0.01 degree.
    pub course_cdeg: Option<u16>,
    /// Satellites used.
    pub sats_used: Option<u8>,
    /// Horizontal DOP x100.
    pub hdop_x100: Option<u16>,
    /// 2D/3D fix type of the latest GSA.
    pub fix_type: Option<FixType>,
    /// Position DOP x100 of the latest GSA.
    pub pdop_x100: Option<u16>,
    /// Vertical DOP x100 of the latest GSA.
    pub vdop_x100: Option<u16>,
}

impl Fix {
    /// Latitude in degrees, see [`units::degrees`].
    pub fn lat_deg(&self) -> Option<f64> {
        self.lat_e7.map(units::degrees)
    }

    /// Longitude in degrees.
    pub fn lon_deg(&self) -> Option<f64> {
        self.lon_e7.map(units::degrees)
    }

    /// Altitude above mean sea level in metres.
    pub fn alt_m(&self) -> Option<f32> {
        self.alt_mm.map(units::metres)
    }

    /// Ground speed in m/s.
    pub fn speed_m_s(&self) -> Option<f32> {
        self.speed_mm_s.map(units::metres_per_second)
    }

    /// Course over ground in degrees.
    pub fn course_deg(&self) -> Option<f32> {
        self.course_cdeg.map(units::course_degrees)
    }
}

#[derive(Clone, Copy)]
struct Pending {
    time: Option<Time>,
    gga: Option<Gga>,
    rmc: Option<Rmc>,
}

#[derive(Clone, Copy)]
struct Dop {
    fix_type: FixType,
    pdop_x100: Option<u16>,
    vdop_x100: Option<u16>,
}

/// Collects decoded messages and emits a [`Fix`] when both the GGA and the RMC of one epoch have
/// arrived, in either order, at the moment the second of the two arrives.
///
/// An epoch is identified by the UTC time of day. A GGA or RMC whose time differs from the pending one
/// starts a new epoch; the incomplete one is dropped and counted in [`FixBuilder::dropped`]. Two
/// messages without a time (receiver not yet synchronised) count as the same epoch.
pub struct FixBuilder {
    pending: Option<Pending>,
    dop: Option<Dop>,
    dropped: u32,
}

impl FixBuilder {
    /// Empty builder.
    pub const fn new() -> Self {
        Self {
            pending: None,
            dop: None,
            dropped: 0,
        }
    }

    /// Epochs started but replaced before both GGA and RMC arrived.
    pub const fn dropped(&self) -> u32 {
        self.dropped
    }

    /// Feeds one message. Messages other than GGA, RMC and GSA are ignored.
    pub fn push(&mut self, message: &Message) -> Option<Fix> {
        match message {
            Message::Gsa(gsa) => {
                self.dop = Some(Dop {
                    fix_type: gsa.fix_type,
                    pdop_x100: gsa.pdop_x100,
                    vdop_x100: gsa.vdop_x100,
                });
                None
            }
            Message::Gga(gga) => {
                self.epoch(gga.time).gga = Some(*gga);
                self.complete()
            }
            Message::Rmc(rmc) => {
                self.epoch(rmc.time).rmc = Some(*rmc);
                self.complete()
            }
            _ => None,
        }
    }

    fn epoch(&mut self, time: Option<Time>) -> &mut Pending {
        if self.pending.is_some_and(|p| p.time != time) {
            self.pending = None;
            self.dropped = self.dropped.saturating_add(1);
        }
        self.pending.get_or_insert(Pending {
            time,
            gga: None,
            rmc: None,
        })
    }

    fn complete(&mut self) -> Option<Fix> {
        let p = self.pending?;
        let (gga, rmc) = (p.gga?, p.rmc?);
        self.pending = None;
        let dop = self.dop;
        Some(Fix {
            time: gga.time,
            date: rmc.date,
            valid: rmc.valid,
            quality: gga.quality,
            lat_e7: gga.lat_e7,
            lon_e7: gga.lon_e7,
            alt_mm: gga.alt_mm,
            geoid_sep_mm: gga.geoid_sep_mm,
            speed_mm_s: rmc.speed_mm_s,
            course_cdeg: rmc.course_cdeg,
            sats_used: gga.sats_used,
            hdop_x100: gga.hdop_x100,
            fix_type: dop.map(|d| d.fix_type),
            pdop_x100: dop.and_then(|d| d.pdop_x100),
            vdop_x100: dop.and_then(|d| d.vdop_x100),
        })
    }
}

impl Default for FixBuilder {
    fn default() -> Self {
        Self::new()
    }
}
