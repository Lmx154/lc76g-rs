//! Typed decoding of the sentences the LC76G sends: [`Message`] and the structs it holds.
//!
//! Integers only. Units: coordinates 1e-7 degree, altitude and EPE millimetres, speed mm/s, course
//! 0.01 degree, DOP and HDOP x100. A field the module leaves empty (no fix) decodes to `None`.
//! Conversions to floating point are separate functions in [`crate::units`].

use crate::field::{self, Bad, Parsed};
use crate::nmea::Sentence;

/// Time of day, UTC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Time {
    /// 0 to 23.
    pub hour: u8,
    /// 0 to 59.
    pub minute: u8,
    /// 0 to 60 (leap second).
    pub second: u8,
    /// 0 to 999. Finer fractions are truncated.
    pub millis: u16,
}

impl Time {
    /// Milliseconds since 00:00:00.
    pub const fn millis_of_day(&self) -> u32 {
        ((self.hour as u32 * 60 + self.minute as u32) * 60 + self.second as u32) * 1000
            + self.millis as u32
    }
}

/// Calendar date, UTC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Date {
    /// Full year, 2000 + the two-digit year of the sentence.
    pub year: u16,
    /// 1 to 12.
    pub month: u8,
    /// 1 to 31.
    pub day: u8,
}

/// Talker ID (Table 4 of the specification).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Talker {
    /// `GP`: GPS.
    Gp,
    /// `GL`: GLONASS.
    Gl,
    /// `GA`: Galileo.
    Ga,
    /// `GB`: BDS.
    Gb,
    /// `GQ`: QZSS (the module itself reports QZSS as `GP`).
    Gq,
    /// `GN`: several systems combined.
    Gn,
}

impl Talker {
    fn from_prefix(address: &str) -> Option<Self> {
        match address.get(..2)? {
            "GP" => Some(Self::Gp),
            "GL" => Some(Self::Gl),
            "GA" => Some(Self::Ga),
            "GB" => Some(Self::Gb),
            "GQ" => Some(Self::Gq),
            "GN" => Some(Self::Gn),
            _ => None,
        }
    }
}

/// GGA quality indicator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Quality {
    /// 0: fix not available or invalid.
    NoFix,
    /// 1: GPS SPS mode, fix valid.
    Gps,
    /// 2: differential GPS or SBAS, fix valid.
    Differential,
    /// 3: GPS PPS mode, fix valid.
    Pps,
    /// 4: RTK, fixed integers.
    RtkFixed,
    /// 5: RTK, floating integers.
    RtkFloat,
    /// 6: estimated (dead reckoning).
    Estimated,
}

impl Quality {
    fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => Self::NoFix,
            1 => Self::Gps,
            2 => Self::Differential,
            3 => Self::Pps,
            4 => Self::RtkFixed,
            5 => Self::RtkFloat,
            6 => Self::Estimated,
            _ => return None,
        })
    }
}

/// Mode indicator of RMC, VTG and GLL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PositionMode {
    /// `A`.
    Autonomous,
    /// `D`: differential.
    Differential,
    /// `E`: estimated (dead reckoning).
    Estimated,
    /// `F`: float RTK.
    FloatRtk,
    /// `M`: manual input.
    Manual,
    /// `N`: no fix.
    NoFix,
    /// `R`: RTK, fixed integers.
    Rtk,
}

impl PositionMode {
    fn parse(s: &str) -> Parsed<Option<Self>> {
        Ok(Some(match s {
            "" => return Ok(None),
            "A" => Self::Autonomous,
            "D" => Self::Differential,
            "E" => Self::Estimated,
            "F" => Self::FloatRtk,
            "M" => Self::Manual,
            "N" => Self::NoFix,
            "R" => Self::Rtk,
            _ => return Err(Bad),
        }))
    }
}

/// GSA fix mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FixType {
    /// 1: fix not available.
    NoFix,
    /// 2: 2D.
    Fix2d,
    /// 3: 3D.
    Fix3d,
}

/// GGA: fix data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gga {
    /// Talker (`GN` for the multi-constellation solution).
    pub talker: Talker,
    /// UTC time of the fix.
    pub time: Option<Time>,
    /// Latitude, 1e-7 degree, south negative.
    pub lat_e7: Option<i32>,
    /// Longitude, 1e-7 degree, west negative.
    pub lon_e7: Option<i32>,
    /// Quality indicator.
    pub quality: Quality,
    /// Satellites in use.
    pub sats_used: Option<u8>,
    /// Horizontal DOP x100.
    pub hdop_x100: Option<u16>,
    /// Altitude above mean sea level, mm.
    pub alt_mm: Option<i32>,
    /// Geoid separation, mm.
    pub geoid_sep_mm: Option<i32>,
}

/// RMC: recommended minimum data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rmc {
    /// Talker.
    pub talker: Talker,
    /// UTC time of the fix.
    pub time: Option<Time>,
    /// Status: `A` (data valid) is true, `V` (receiver warning) is false.
    pub valid: bool,
    /// Latitude, 1e-7 degree, south negative.
    pub lat_e7: Option<i32>,
    /// Longitude, 1e-7 degree, west negative.
    pub lon_e7: Option<i32>,
    /// Speed over ground, mm/s.
    pub speed_mm_s: Option<u32>,
    /// Course over ground, 0.01 degree.
    pub course_cdeg: Option<u16>,
    /// UTC date.
    pub date: Option<Date>,
    /// Mode indicator.
    pub mode: Option<PositionMode>,
    /// NMEA 4.10 navigational status character (the module sends `V`, not supported).
    pub nav_status: Option<u8>,
}

/// GSA: DOP and active satellites. One sentence per satellite system.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gsa {
    /// Talker.
    pub talker: Talker,
    /// True for `A` (automatic 2D/3D), false for `M` (manual).
    pub automatic: bool,
    /// Fix mode.
    pub fix_type: FixType,
    /// IDs of the satellites used (the first 12), `None` for unused slots.
    pub sats: [Option<u16>; 12],
    /// Position DOP x100.
    pub pdop_x100: Option<u16>,
    /// Horizontal DOP x100.
    pub hdop_x100: Option<u16>,
    /// Vertical DOP x100.
    pub vdop_x100: Option<u16>,
    /// NMEA 4.10 GNSS system ID (1 GPS, 2 GLONASS, 3 Galileo, 4 BDS).
    pub system_id: Option<u8>,
}

/// One satellite of a GSV sentence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SatInfo {
    /// Satellite ID.
    pub id: u16,
    /// Elevation, degrees.
    pub elevation: Option<u8>,
    /// Azimuth, degrees.
    pub azimuth: Option<u16>,
    /// C/N0, dB-Hz; `None` when not tracked.
    pub cn0: Option<u8>,
}

/// GSV: satellites in view, up to four per sentence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gsv {
    /// Talker of the constellation (`GN` is not used for GSV).
    pub talker: Talker,
    /// Number of GSV sentences in this group.
    pub total_sentences: u8,
    /// Number of this sentence, from 1.
    pub sentence: u8,
    /// Satellites in view.
    pub sats_in_view: u8,
    /// The satellites in this sentence.
    pub sats: [Option<SatInfo>; 4],
    /// NMEA 4.10 signal ID.
    pub signal_id: Option<u8>,
}

/// VTG: course and speed over ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Vtg {
    /// Talker.
    pub talker: Talker,
    /// True course, 0.01 degree.
    pub course_cdeg: Option<u16>,
    /// Speed, mm/s (from the km/h field, else the knots field).
    pub speed_mm_s: Option<u32>,
    /// Mode indicator.
    pub mode: Option<PositionMode>,
}

/// GLL: geographic position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Gll {
    /// Talker.
    pub talker: Talker,
    /// Latitude, 1e-7 degree, south negative.
    pub lat_e7: Option<i32>,
    /// Longitude, 1e-7 degree, west negative.
    pub lon_e7: Option<i32>,
    /// UTC time.
    pub time: Option<Time>,
    /// Status `A` is true, `V` is false.
    pub valid: bool,
    /// Mode indicator.
    pub mode: Option<PositionMode>,
}

/// `$PAIR001`: acknowledgement of a PAIR command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ack {
    /// Command ID acknowledged (the number after `PAIR`).
    pub command: u16,
    /// 0 success, 1 being processed (another ack follows), 2 failed, 3 unsupported, 4 parameter
    /// error, 5 module busy.
    pub result: u8,
}

/// Capacity of the version string in [`Version`].
pub const VERSION_LEN: usize = 32;

/// `$PQTMVERNO`: firmware version.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Version {
    text: [u8; VERSION_LEN],
    len: u8,
    /// Firmware build date.
    pub build_date: Date,
    /// Firmware build time.
    pub build_time: Time,
}

impl Version {
    /// Version string, for example `LC76GABNR12A05S`.
    pub fn version(&self) -> &str {
        core::str::from_utf8(&self.text[..usize::from(self.len)]).unwrap_or("")
    }
}

/// `$PQTMEPE` (version 2): estimated position error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Epe {
    /// North, mm.
    pub north_mm: Option<u32>,
    /// East, mm.
    pub east_mm: Option<u32>,
    /// Down, mm.
    pub down_mm: Option<u32>,
    /// Horizontal (2D), mm.
    pub epe_2d_mm: Option<u32>,
    /// 3D, mm.
    pub epe_3d_mm: Option<u32>,
}

/// PQTM commands whose reply is of the `OK` / `ERROR` form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PqtmCommand {
    /// `PQTMCFGMSGRATE`.
    CfgMsgRate,
    /// `PQTMSAVEPAR`.
    SavePar,
    /// `PQTMVERNO` (its error reply).
    VerNo,
}

/// `$PQTM<name>,OK...` or `$PQTM<name>,ERROR,<code>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PqtmReply {
    /// The command answered.
    pub command: PqtmCommand,
    /// `None` for `OK`; the error code otherwise (1 invalid parameters, 2 execution failed).
    pub error: Option<u8>,
}

/// A decoded sentence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Message {
    /// GGA.
    Gga(Gga),
    /// RMC.
    Rmc(Rmc),
    /// GSA.
    Gsa(Gsa),
    /// GSV.
    Gsv(Gsv),
    /// VTG.
    Vtg(Vtg),
    /// GLL.
    Gll(Gll),
    /// PAIR001 acknowledgement.
    Ack(Ack),
    /// PQTMVERNO answer.
    Version(Version),
    /// PQTMEPE.
    Epe(Epe),
    /// PQTM OK/ERROR reply.
    PqtmReply(PqtmReply),
    /// Valid sentence of a type this crate does not decode (ZDA, GNS, GST, GRS, other PAIR and PQTM).
    Other,
    /// Valid checksum but the fields do not match the layout of the specification.
    Malformed,
}

/// GSV with four satellites and a signal ID has 20 fields; GSA has 18.
const MAX_FIELDS: usize = 24;

fn collect_fields<'a>(sentence: &Sentence<'a>, out: &mut [&'a str; MAX_FIELDS]) -> Parsed<usize> {
    let mut n = 0;
    for f in sentence.fields() {
        *out.get_mut(n).ok_or(Bad)? = f;
        n += 1;
    }
    Ok(n)
}

pub(crate) fn decode(sentence: &Sentence<'_>) -> Message {
    let mut fields = [""; MAX_FIELDS];
    let Ok(n) = collect_fields(sentence, &mut fields) else {
        return Message::Malformed;
    };
    decode_fields(sentence.address(), &fields[..n]).unwrap_or(Message::Malformed)
}

fn decode_fields(address: &str, f: &[&str]) -> Parsed<Message> {
    match address {
        "PAIR001" => return ack(f),
        "PQTMVERNO" => return version(f),
        "PQTMEPE" => return epe(f),
        "PQTMCFGMSGRATE" => return pqtm_reply(PqtmCommand::CfgMsgRate, f),
        "PQTMSAVEPAR" => return pqtm_reply(PqtmCommand::SavePar, f),
        _ => {}
    }
    let (Some(talker), true) = (Talker::from_prefix(address), address.len() == 5) else {
        return Ok(Message::Other);
    };
    match &address[2..] {
        "GGA" => gga(talker, f),
        "RMC" => rmc(talker, f),
        "GSA" => gsa(talker, f),
        "GSV" => gsv(talker, f),
        "VTG" => vtg(talker, f),
        "GLL" => gll(talker, f),
        _ => Ok(Message::Other),
    }
}

fn need(f: &[&str], n: usize) -> Parsed<()> {
    if f.len() >= n { Ok(()) } else { Err(Bad) }
}

fn status(s: &str) -> Parsed<bool> {
    match s {
        "A" => Ok(true),
        "V" => Ok(false),
        _ => Err(Bad),
    }
}

fn gga(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 11)?;
    let quality = field::uint_req::<u8>(f[5]).and_then(|c| Quality::from_code(c).ok_or(Bad))?;
    Ok(Message::Gga(Gga {
        talker,
        time: field::time(f[0])?,
        lat_e7: field::latitude(f[1], f[2])?,
        lon_e7: field::longitude(f[3], f[4])?,
        quality,
        sats_used: field::uint(f[6])?,
        hdop_x100: field::centi_u16(f[7])?,
        alt_mm: field::metres_mm(f[8])?,
        geoid_sep_mm: field::metres_mm(f[10])?,
    }))
}

fn rmc(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 9)?;
    Ok(Message::Rmc(Rmc {
        talker,
        time: field::time(f[0])?,
        valid: status(f[1])?,
        lat_e7: field::latitude(f[2], f[3])?,
        lon_e7: field::longitude(f[4], f[5])?,
        speed_mm_s: field::knots_mm_s(f[6])?,
        course_cdeg: field::course_cdeg(f[7])?,
        date: field::date(f[8])?,
        mode: PositionMode::parse(f.get(11).copied().unwrap_or(""))?,
        nav_status: match f.get(12).copied().unwrap_or("").as_bytes() {
            [] => None,
            [c] => Some(*c),
            _ => return Err(Bad),
        },
    }))
}

fn gsa(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 17)?;
    let automatic = match f[0] {
        "A" => true,
        "M" => false,
        _ => return Err(Bad),
    };
    let fix_type = match field::uint_req::<u8>(f[1])? {
        1 => FixType::NoFix,
        2 => FixType::Fix2d,
        3 => FixType::Fix3d,
        _ => return Err(Bad),
    };
    let mut sats = [None; 12];
    for (slot, s) in sats.iter_mut().zip(&f[2..14]) {
        *slot = field::uint(s)?;
    }
    Ok(Message::Gsa(Gsa {
        talker,
        automatic,
        fix_type,
        sats,
        pdop_x100: field::centi_u16(f[14])?,
        hdop_x100: field::centi_u16(f[15])?,
        vdop_x100: field::centi_u16(f[16])?,
        system_id: field::uint(f.get(17).copied().unwrap_or(""))?,
    }))
}

fn gsv(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 3)?;
    let rest = &f[3..];
    let (blocks, signal_id) = match rest.len() % 4 {
        0 => (rest, None),
        1 => (&rest[..rest.len() - 1], field::uint(rest[rest.len() - 1])?),
        _ => return Err(Bad),
    };
    if blocks.len() > 16 {
        return Err(Bad);
    }
    let mut sats = [None; 4];
    for (slot, b) in sats.iter_mut().zip(blocks.as_chunks::<4>().0) {
        *slot = match (
            field::uint::<u16>(b[0])?,
            b[1..].iter().all(|s| s.is_empty()),
        ) {
            (None, true) => None,
            (None, false) => return Err(Bad),
            (Some(id), _) => Some(SatInfo {
                id,
                elevation: field::uint(b[1])?,
                azimuth: field::uint(b[2])?,
                cn0: field::uint(b[3])?,
            }),
        };
    }
    Ok(Message::Gsv(Gsv {
        talker,
        total_sentences: field::uint_req(f[0])?,
        sentence: field::uint_req(f[1])?,
        sats_in_view: field::uint_req(f[2])?,
        sats,
        signal_id,
    }))
}

fn vtg(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 8)?;
    let speed_mm_s = match field::kmh_mm_s(f[6])? {
        Some(v) => Some(v),
        None => field::knots_mm_s(f[4])?,
    };
    Ok(Message::Vtg(Vtg {
        talker,
        course_cdeg: field::course_cdeg(f[0])?,
        speed_mm_s,
        mode: PositionMode::parse(f.get(8).copied().unwrap_or(""))?,
    }))
}

fn gll(talker: Talker, f: &[&str]) -> Parsed<Message> {
    need(f, 6)?;
    Ok(Message::Gll(Gll {
        talker,
        lat_e7: field::latitude(f[0], f[1])?,
        lon_e7: field::longitude(f[2], f[3])?,
        time: field::time(f[4])?,
        valid: status(f[5])?,
        mode: PositionMode::parse(f.get(6).copied().unwrap_or(""))?,
    }))
}

fn ack(f: &[&str]) -> Parsed<Message> {
    need(f, 2)?;
    Ok(Message::Ack(Ack {
        command: field::uint_req(f[0])?,
        result: field::uint_req(f[1])?,
    }))
}

fn pqtm_error(command: PqtmCommand, f: &[&str]) -> Parsed<Option<Message>> {
    if f.first() == Some(&"ERROR") {
        let error = f
            .get(1)
            .copied()
            .ok_or(Bad)
            .and_then(field::uint_req::<u8>)?;
        return Ok(Some(Message::PqtmReply(PqtmReply {
            command,
            error: Some(error),
        })));
    }
    Ok(None)
}

fn pqtm_reply(command: PqtmCommand, f: &[&str]) -> Parsed<Message> {
    if let Some(m) = pqtm_error(command, f)? {
        return Ok(m);
    }
    if f.first() == Some(&"OK") {
        Ok(Message::PqtmReply(PqtmReply {
            command,
            error: None,
        }))
    } else {
        Err(Bad)
    }
}

fn version(f: &[&str]) -> Parsed<Message> {
    if let Some(m) = pqtm_error(PqtmCommand::VerNo, f)? {
        return Ok(m);
    }
    need(f, 3)?;
    let name = f[0].as_bytes();
    if name.is_empty() || name.len() > VERSION_LEN || !name.is_ascii() {
        return Err(Bad);
    }
    let mut text = [0; VERSION_LEN];
    text[..name.len()].copy_from_slice(name);
    let mut date = f[1].split('/');
    let (Some(y), Some(m), Some(d), None) = (date.next(), date.next(), date.next(), date.next())
    else {
        return Err(Bad);
    };
    let mut time = f[2].split(':');
    let (Some(h), Some(mi), Some(s), None) = (time.next(), time.next(), time.next(), time.next())
    else {
        return Err(Bad);
    };
    Ok(Message::Version(Version {
        text,
        len: name.len() as u8,
        build_date: Date {
            year: field::uint_req(y)?,
            month: field::uint_req(m)?,
            day: field::uint_req(d)?,
        },
        build_time: Time {
            hour: field::uint_req(h)?,
            minute: field::uint_req(mi)?,
            second: field::uint_req(s)?,
            millis: 0,
        },
    }))
}

fn epe(f: &[&str]) -> Parsed<Message> {
    need(f, 6)?;
    if f[0] != "2" {
        return Ok(Message::Other);
    }
    Ok(Message::Epe(Epe {
        north_mm: field::scaled(f[1], 1000, 1)?,
        east_mm: field::scaled(f[2], 1000, 1)?,
        down_mm: field::scaled(f[3], 1000, 1)?,
        epe_2d_mm: field::scaled(f[4], 1000, 1)?,
        epe_3d_mm: field::scaled(f[5], 1000, 1)?,
    }))
}
