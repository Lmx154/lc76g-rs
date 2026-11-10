//! Module settings applied by [`Lc76g::init`](crate::Lc76g::init): [`Config`] and its parts.

/// Position fix interval limits of `PAIR050`, in milliseconds.
pub const FIX_INTERVAL_MS: core::ops::RangeInclusive<u16> = 100..=1000;
/// Largest sentence output rate of `PAIR062` (one sentence every N fixes).
pub const MAX_SENTENCE_RATE: u8 = 20;
/// PPS pulse width limits of `PAIR752`, in milliseconds.
pub const PPS_WIDTH_MS: core::ops::RangeInclusive<u16> = 1..=999;

/// Standard sentence types of `PAIR062`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SentenceType {
    /// GGA, code 0.
    Gga,
    /// GLL, code 1.
    Gll,
    /// GSA, code 2.
    Gsa,
    /// GSV, code 3.
    Gsv,
    /// RMC, code 4.
    Rmc,
    /// VTG, code 5.
    Vtg,
    /// ZDA, code 6.
    Zda,
    /// GRS, code 7.
    Grs,
    /// GST, code 8.
    Gst,
    /// GNS, code 9.
    Gns,
}

impl SentenceType {
    /// All types in `PAIR062` code order.
    pub const ALL: [Self; 10] = [
        Self::Gga,
        Self::Gll,
        Self::Gsa,
        Self::Gsv,
        Self::Rmc,
        Self::Vtg,
        Self::Zda,
        Self::Grs,
        Self::Gst,
        Self::Gns,
    ];

    /// The `<Type>` parameter of `PAIR062`.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Navigation mode of `PAIR080`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NavMode {
    /// 0: general purposes.
    #[default]
    Normal,
    /// 1: running and walking, below 5 m/s.
    Fitness,
    /// 5: drone. On firmware newer than LC76GABNR02A01S latitude and longitude then carry seven
    /// decimals of minutes instead of six; the decoders accept both.
    Drone,
    /// 7: swimming.
    Swimming,
}

impl NavMode {
    /// The `<NavMode>` parameter of `PAIR080`.
    pub const fn code(self) -> u8 {
        match self {
            Self::Normal => 0,
            Self::Fitness => 1,
            Self::Drone => 5,
            Self::Swimming => 7,
        }
    }
}

/// PPS pulse type of `PAIR752`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PpsType {
    /// 0: no pulse.
    Disabled,
    /// 1: after the first fix.
    AfterFirstFix,
    /// 2: 3D fix only.
    Fix3dOnly,
    /// 3: 2D or 3D fix only.
    Fix2dOr3d,
    /// 4: always.
    Always,
}

impl PpsType {
    /// The `<PPSType>` parameter of `PAIR752`.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// PPS output setting of `PAIR752`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pps {
    /// When the pulse is produced.
    pub kind: PpsType,
    /// Pulse width in milliseconds, 1 to 999 (module default 100).
    pub width_ms: u16,
}

/// Output rate of every standard sentence: one sentence every N position fixes, 0 to 20, 0 = off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SentenceRates {
    /// GGA.
    pub gga: u8,
    /// GLL.
    pub gll: u8,
    /// GSA.
    pub gsa: u8,
    /// GSV.
    pub gsv: u8,
    /// RMC.
    pub rmc: u8,
    /// VTG.
    pub vtg: u8,
    /// ZDA.
    pub zda: u8,
    /// GRS.
    pub grs: u8,
    /// GST.
    pub gst: u8,
    /// GNS.
    pub gns: u8,
}

impl SentenceRates {
    /// Rate of one sentence type.
    pub const fn get(&self, sentence: SentenceType) -> u8 {
        match sentence {
            SentenceType::Gga => self.gga,
            SentenceType::Gll => self.gll,
            SentenceType::Gsa => self.gsa,
            SentenceType::Gsv => self.gsv,
            SentenceType::Rmc => self.rmc,
            SentenceType::Vtg => self.vtg,
            SentenceType::Zda => self.zda,
            SentenceType::Grs => self.grs,
            SentenceType::Gst => self.gst,
            SentenceType::Gns => self.gns,
        }
    }
}

impl Default for SentenceRates {
    /// The stream observed from a freshly reset LC76GABNR12A05S: GGA, GLL, GSA, GSV, RMC and VTG
    /// every fix; ZDA, GRS, GST and GNS off. (The specification's "default value 1" would also turn
    /// those four on; the module does not.)
    fn default() -> Self {
        Self {
            gga: 1,
            gll: 1,
            gsa: 1,
            gsv: 1,
            rmc: 1,
            vtg: 1,
            zda: 0,
            grs: 0,
            gst: 0,
            gns: 0,
        }
    }
}

/// Settings written by [`Lc76g::init`](crate::Lc76g::init). `Config::default()` is the module's
/// power-on state, so `init(&Config::default())` returns a module to it from any earlier volatile
/// setting.
///
/// Not included because the specification makes them take effect only after a reboot or involve one:
/// GNSS search mode (`PAIR066`, the module reboots) and baud rate (`PAIR864`, effective after a
/// reboot). Settings only reach the module's flash through
/// [`Lc76g::save_settings`](crate::Lc76g::save_settings).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Position fix interval in milliseconds, 100 to 1000 (default 1000). The specification says that
    /// above 1 Hz (below 1000 ms) only RMC, GGA and GNS follow the fix rate, GSA and GSV stay at 1 Hz and
    /// VTG and GLL stop. LC76GABNR12A05S does not: at 100 ms every enabled sentence is sent at 10 Hz,
    /// 187 sentences and 11.58 kB per second on a line that carries 11.52 kB/s at 115200 baud; the module
    /// then drops fixes and answers commands late or not at all. [`Config::is_valid`] therefore limits GSA,
    /// GSV and GRS to once per second below 1000 ms; [`Config::with_fix_interval`] sets the rates.
    pub fix_interval_ms: u16,
    /// Navigation mode (default normal).
    pub nav_mode: NavMode,
    /// Sentence output rates.
    pub rates: SentenceRates,
    /// Output `$PQTMEPE` (estimated position error) every fix (default off).
    pub epe: bool,
    /// PPS setting; `None` leaves the module's PPS as it is (the specification gives no default pulse
    /// type).
    pub pps: Option<Pps>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            fix_interval_ms: 1000,
            nav_mode: NavMode::Normal,
            rates: SentenceRates::default(),
            epe: false,
            pps: None,
        }
    }
}

impl Config {
    /// [`Config::default`] with `fix_interval_ms` and GSA and GSV at `ceil(1000 / ms)`, the smallest rate
    /// that keeps them at most once per second (1 at 1000 ms, so `with_fix_interval(1000)` equals
    /// `Config::default()`). At 100 ms that is 10; measured on LC76GABNR12A05S this gives 100 fixes per
    /// 10 s at about 65 sentences per second. An interval outside [`FIX_INTERVAL_MS`] gives a
    /// configuration that is not valid.
    pub fn with_fix_interval(ms: u16) -> Self {
        let clamped = ms.clamp(*FIX_INTERVAL_MS.start(), *FIX_INTERVAL_MS.end());
        let rate = 1000u32.div_ceil(u32::from(clamped)) as u8;
        let defaults = SentenceRates::default();
        Self {
            fix_interval_ms: ms,
            rates: SentenceRates {
                gsa: rate,
                gsv: rate,
                ..defaults
            },
            ..Self::default()
        }
    }

    /// True if every value is inside the range the module accepts and the stream fits the line.
    ///
    /// The stream rule: when `fix_interval_ms` is below 1000, each of GSA, GSV and GRS must be off (rate
    /// 0) or satisfy `rate * fix_interval_ms >= 1000`, at most once per second. These types are several
    /// sentences per epoch (about 4 GSA and 11 to 12 GSV of the 19 sentences of one epoch at the default
    /// rates) and the module does not thin them by itself: at 100 ms with the default rates it sends 187
    /// sentences, 11.58 kB per second, more than the 11.52 kB/s of 115200 baud. With GSA and GSV at 10
    /// it sends about 65 sentences per second.
    pub fn is_valid(&self) -> bool {
        let ms = u32::from(self.fix_interval_ms);
        FIX_INTERVAL_MS.contains(&self.fix_interval_ms)
            && SentenceType::ALL
                .iter()
                .all(|s| self.rates.get(*s) <= MAX_SENTENCE_RATE)
            && [SentenceType::Gsa, SentenceType::Gsv, SentenceType::Grs]
                .iter()
                .all(|s| {
                    let rate = u32::from(self.rates.get(*s));
                    ms >= 1000 || rate == 0 || rate * ms >= 1000
                })
            && self.pps.is_none_or(|p| PPS_WIDTH_MS.contains(&p.width_ms))
    }
}
