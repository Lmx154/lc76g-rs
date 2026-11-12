//! On-board check of the ZDA, GNS, GST and GRS decoders against the real module: LC76G on UART1
//! (GPIO24 = TX to the module, GPIO25 = RX from the module, 115200 8N1). Each sentence type is switched
//! on with `$PAIR062`, read for at least 10 s, counted, decoded and compared with the GGA, RMC and GSA of
//! the same epoch; then all four run together. The program ends with `init(&Config::default())`.
//! It never sends a flash write, a baud change, a restart or a low-power command.
//!
//! Privacy: the module reports the real position of the bench. Nothing here prints a latitude or a
//! longitude; raw sentences are printed with the digits of position and altitude fields replaced by `x`
//! and without their checksum, and positions are compared only as differences between two sentences.
#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Duration, Instant, with_timeout};
use lc76g::aux_sentences::{Gns, Grs, Gst, Zda};
use lc76g::sentence::{Gga, Gsa, Quality, Rmc};
use lc76g::{Command, Config, Error, FixBuilder, Lc76g, Message, Sentence, SentenceType};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type Gnss = Lc76g<BufferedUart>;
type GnssError = Error<uart::Error>;

const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 256;

/// Seconds each sentence type is read.
const WINDOW_SECS: u64 = 10;
/// Silence in milliseconds that separates two bursts (epochs); the module sends a burst of about 200 ms
/// at the top of each second.
const GAP: Duration = Duration::from_millis(400);

const KINDS: [&str; 4] = ["ZDA", "GNS", "GST", "GRS"];
const TYPES: [SentenceType; 4] = [
    SentenceType::Zda,
    SentenceType::Gns,
    SentenceType::Gst,
    SentenceType::Grs,
];

static UART_ERRORS: AtomicU32 = AtomicU32::new(0);
static STARTUP_ERRORS: AtomicU32 = AtomicU32::new(0);

macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

struct Line {
    buf: [u8; 240],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            buf: [0; 240],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

fn count_err(e: &GnssError) {
    if let Error::Uart(_) = e {
        UART_ERRORS.fetch_add(1, Relaxed);
    }
}

fn step_ms(prev: u32, t: u32) -> u32 {
    (t + 86_400_000 - prev) % 86_400_000
}

/// Sentence kind of an address: index into `KINDS` for `??ZDA`, `??GNS`, `??GST`, `??GRS`.
fn kind_of(address: &str) -> Option<usize> {
    if address.len() != 5 || !address.starts_with('G') {
        return None;
    }
    match address.get(2..)? {
        "ZDA" => Some(0),
        "GNS" => Some(1),
        "GST" => Some(2),
        "GRS" => Some(3),
        _ => None,
    }
}

/// Writes the sentence without its checksum, the digits of latitude, longitude and altitude replaced by
/// `x` (GNS, GGA, RMC, GLL).
fn masked(raw: &str, out: &mut Line) {
    let body = raw.split('*').next().unwrap_or(raw);
    let kind = body.get(3..6).unwrap_or("");
    for (i, f) in body.split(',').enumerate() {
        if i > 0 {
            let _ = out.write_char(',');
        }
        let hide = match kind {
            "GNS" => matches!(i, 2 | 4 | 9 | 10 | 11),
            "GGA" => matches!(i, 2 | 4 | 9 | 11),
            "RMC" => matches!(i, 3 | 5),
            "GLL" => matches!(i, 1 | 3),
            _ => false,
        };
        for ch in f.chars() {
            let _ = out.write_char(if hide && ch.is_ascii_digit() { 'x' } else { ch });
        }
    }
}

/// Digits after the decimal point.
fn decimals(f: &str) -> u8 {
    f.split_once('.').map_or(0, |(_, d)| d.len() as u8)
}

/// Value of one printed digit of a latitude or longitude with `a` or `b` decimals of minutes, in 1e-7
/// degree, rounded up (the coarser of the two sentences sets the tolerance).
fn tol_deg_e7(a: u8, b: u8) -> i64 {
    let den = 60 * 10i64.pow(u32::from(a.min(b).min(9)));
    (10_000_000 + den - 1) / den
}

/// Value of one printed digit of an altitude with `a` or `b` decimals of metres, in millimetres.
fn tol_mm(a: u8, b: u8) -> i64 {
    let d = u32::from(a.min(b));
    if d >= 3 { 1 } else { 10i64.pow(3 - d) }
}

async fn back_to_1hz(gnss: &mut Gnss) -> u32 {
    let mut used = 0;
    for command in [
        Command::SetSentenceRate {
            sentence: SentenceType::Gsv,
            rate: 0,
        },
        Command::SetFixInterval { ms: 1000 },
    ] {
        let mut done = false;
        for attempt in 1..=10u32 {
            let r = with_timeout(Duration::from_secs(4), gnss.command(&command)).await;
            if let Ok(Ok(())) = r {
                used += attempt;
                done = true;
                break;
            }
        }
        if !done {
            return 0;
        }
    }
    used
}

struct Steps {
    fixes: u32,
    transition: u32,
    settled: u32,
}

/// Reads fixes until `need` consecutive steps of 1000 ms, at most 30 fixes.
async fn settle_1hz(gnss: &mut Gnss, need: u32) -> Steps {
    let mut s = Steps {
        fixes: 0,
        transition: 0,
        settled: 0,
    };
    let mut prev = None;
    while s.fixes < 30 && s.settled < need {
        let fix = match with_timeout(Duration::from_secs(3), gnss.read_fix()).await {
            Ok(Ok(fix)) => fix,
            Ok(Err(e)) => {
                count_err(&e);
                continue;
            }
            Err(_) => break,
        };
        let t = fix.time.map(|t| t.millis_of_day());
        if let (Some(p), Some(t)) = (prev, t) {
            if step_ms(p, t) == 1000 {
                s.settled += 1;
            } else {
                s.settled = 0;
                s.transition += 1;
            }
        }
        prev = t;
        s.fixes += 1;
    }
    s
}

/// Sends a get command through `Command::Raw` and returns the reply sentence that follows the
/// acknowledgement.
async fn query(gnss: &mut Gnss, body: &str) -> Result<Line, &'static str> {
    let mut want = Line::new();
    let _ = write!(want, "${body},");
    match with_timeout(Duration::from_secs(3), gnss.command(&Command::Raw(body))).await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            count_err(&e);
            return Err("command failed");
        }
        Err(_) => return Err("no acknowledgement within 3 s"),
    }
    let end = Instant::now() + Duration::from_millis(2000);
    for _ in 0..80 {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_sentence()).await {
            Ok(Ok(s)) if s.as_str().starts_with(want.as_str()) => {
                let mut line = Line::new();
                let _ = write!(line, "{}", s.as_str());
                return Ok(line);
            }
            Ok(Ok(_)) => {}
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    Err("acknowledged, but no reply sentence followed")
}

/// What arrived of one sentence type.
struct Tally {
    arrived: u16,
    decoded: u16,
    /// Decoded as `Message::Malformed`.
    rejected: u16,
    /// Neither the expected message nor `Malformed`.
    other: u16,
    nfields: [u16; 24],
    /// GNS only: both unit fields (9 and 11) are `M`.
    units_m: u16,
    /// GNS only: distinct mode indicator fields (field 5) and how often each came.
    modes: [[u8; 8]; 3],
    modes_n: [u16; 3],
    modes_used: usize,
    example: Line,
    bad: [Line; 3],
    nbad: usize,
}

impl Tally {
    const fn new() -> Self {
        Self {
            arrived: 0,
            decoded: 0,
            rejected: 0,
            other: 0,
            nfields: [0; 24],
            units_m: 0,
            modes: [[0; 8]; 3],
            modes_n: [0; 3],
            modes_used: 0,
            example: Line::new(),
            bad: [const { Line::new() }; 3],
            nbad: 0,
        }
    }

    fn add_mode(&mut self, m: &str) {
        let mut padded = [0u8; 8];
        let b = m.as_bytes();
        let n = b.len().min(8);
        padded[..n].copy_from_slice(&b[..n]);
        for i in 0..self.modes_used {
            if self.modes[i] == padded {
                self.modes_n[i] += 1;
                return;
            }
        }
        if self.modes_used < 3 {
            self.modes[self.modes_used] = padded;
            self.modes_n[self.modes_used] = 1;
            self.modes_used += 1;
        }
    }
}

/// The sentences of one burst.
struct Epoch {
    n: u16,
    rmc: Option<Rmc>,
    gga: Option<Gga>,
    /// Decimals printed in latitude, longitude, altitude.
    gga_dec: [u8; 3],
    /// The raw GGA and GNS sentences, kept only to compare fields; never printed.
    gga_raw: Line,
    gns_raw: Line,
    gsa: [Option<Gsa>; 8],
    zda: Option<Zda>,
    gns: Option<Gns>,
    gns_dec: [u8; 3],
    gst: Option<Gst>,
    grs: [Option<Grs>; 8],
    order: Line,
}

impl Epoch {
    const fn new() -> Self {
        Self {
            n: 0,
            rmc: None,
            gga: None,
            gga_dec: [0; 3],
            gga_raw: Line::new(),
            gns_raw: Line::new(),
            gsa: [None; 8],
            zda: None,
            gns: None,
            gns_dec: [0; 3],
            gst: None,
            grs: [None; 8],
            order: Line::new(),
        }
    }
}

/// Cross-check counters over the complete epochs of a window.
struct Acc {
    epochs: u32,
    epochs_fix: u32,
    zda_n: u32,
    zda_no_rmc: u32,
    zda_empty: u32,
    zda_cmp: u32,
    zda_tmm: u32,
    zda_dmm: u32,
    gns_n: u32,
    gns_no_gga: u32,
    gns_tcmp: u32,
    gns_tmm: u32,
    gns_pos_cmp: u32,
    gns_lat_mm: u32,
    gns_lon_mm: u32,
    gns_alt_cmp: u32,
    gns_alt_mm: u32,
    gns_sat_cmp: u32,
    gns_sat_mm: u32,
    max_dlat: i64,
    max_dlon: i64,
    max_dalt: i64,
    gst_n: u32,
    gst_neg: u32,
    gst_fix_n: u32,
    gst_fix_missing: u32,
    gst_nofix: u32,
    gst_tcmp: u32,
    gst_tmm: u32,
    gst_last: Option<Gst>,
    grs_n: u32,
    grs_cmp: u32,
    grs_mm: u32,
    grs_empty: u32,
    grs_nogsa: u32,
    grs_tcmp: u32,
    grs_tmm: u32,
    /// First mismatches: (system ID or 0, residuals, satellites in the GSA).
    grs_first: [(u8, u8, u8); 3],
    /// GNS sentences the decoder rejected, compared field by field as text with the GGA.
    raw_n: u32,
    raw_layout: u32,
    raw_time_mm: u32,
    raw_pos_mm: u32,
    raw_sat_mm: u32,
    raw_hdop_mm: u32,
    raw_alt_mm: u32,
    raw_sep_mm: u32,
}

impl Acc {
    const fn new() -> Self {
        Self {
            epochs: 0,
            epochs_fix: 0,
            zda_n: 0,
            zda_no_rmc: 0,
            zda_empty: 0,
            zda_cmp: 0,
            zda_tmm: 0,
            zda_dmm: 0,
            gns_n: 0,
            gns_no_gga: 0,
            gns_tcmp: 0,
            gns_tmm: 0,
            gns_pos_cmp: 0,
            gns_lat_mm: 0,
            gns_lon_mm: 0,
            gns_alt_cmp: 0,
            gns_alt_mm: 0,
            gns_sat_cmp: 0,
            gns_sat_mm: 0,
            max_dlat: 0,
            max_dlon: 0,
            max_dalt: 0,
            gst_n: 0,
            gst_neg: 0,
            gst_fix_n: 0,
            gst_fix_missing: 0,
            gst_nofix: 0,
            gst_tcmp: 0,
            gst_tmm: 0,
            gst_last: None,
            grs_n: 0,
            grs_cmp: 0,
            grs_mm: 0,
            grs_empty: 0,
            grs_nogsa: 0,
            grs_tcmp: 0,
            grs_tmm: 0,
            grs_first: [(0, 0, 0); 3],
            raw_n: 0,
            raw_layout: 0,
            raw_time_mm: 0,
            raw_pos_mm: 0,
            raw_sat_mm: 0,
            raw_hdop_mm: 0,
            raw_alt_mm: 0,
            raw_sep_mm: 0,
        }
    }
}

fn split_fields(raw: &str) -> ([&str; 16], usize) {
    let body = raw.split('*').next().unwrap_or(raw);
    let mut f = [""; 16];
    let mut n = 0;
    for x in body.split(',') {
        if let Some(slot) = f.get_mut(n) {
            *slot = x;
        }
        n += 1;
    }
    (f, n.min(16))
}

fn check_epoch(ep: &Epoch, a: &mut Acc) {
    a.epochs += 1;
    let fix =
        ep.rmc.is_some_and(|r| r.valid) && ep.gga.is_some_and(|g| g.quality != Quality::NoFix);
    a.epochs_fix += fix as u32;

    if let Some(z) = ep.zda {
        a.zda_n += 1;
        match ep.rmc {
            None => a.zda_no_rmc += 1,
            Some(r)
                if r.time.is_none() || r.date.is_none() || z.time.is_none() || z.date.is_none() =>
            {
                a.zda_empty += 1
            }
            Some(r) => {
                a.zda_cmp += 1;
                a.zda_tmm += (z.time != r.time) as u32;
                a.zda_dmm += (z.date != r.date) as u32;
            }
        }
    }

    if ep.gns.is_none() && ep.gns_raw.len > 0 && ep.gga_raw.len > 0 {
        a.raw_n += 1;
        let (n, nn) = split_fields(ep.gns_raw.as_str());
        let (g, _) = split_fields(ep.gga_raw.as_str());
        if nn != 14 {
            a.raw_layout += 1;
        } else {
            a.raw_time_mm += (n[1] != g[1]) as u32;
            a.raw_pos_mm += (n[2] != g[2] || n[3] != g[3] || n[4] != g[4] || n[5] != g[5]) as u32;
            a.raw_sat_mm += (n[7] != g[7]) as u32;
            a.raw_hdop_mm += (n[8] != g[8]) as u32;
            a.raw_alt_mm += (n[9] != g[9]) as u32;
            a.raw_sep_mm += (n[10] != g[11]) as u32;
        }
    }

    if let Some(n) = ep.gns {
        a.gns_n += 1;
        match ep.gga {
            None => a.gns_no_gga += 1,
            Some(g) => {
                if n.time.is_some() && g.time.is_some() {
                    a.gns_tcmp += 1;
                    a.gns_tmm += (n.time != g.time) as u32;
                }
                if let (Some(la), Some(lb), Some(oa), Some(ob)) =
                    (n.lat_e7, g.lat_e7, n.lon_e7, g.lon_e7)
                {
                    a.gns_pos_cmp += 1;
                    let dlat = i64::from(la - lb).abs();
                    let dlon = i64::from(oa - ob).abs();
                    a.max_dlat = a.max_dlat.max(dlat);
                    a.max_dlon = a.max_dlon.max(dlon);
                    a.gns_lat_mm += (dlat > tol_deg_e7(ep.gns_dec[0], ep.gga_dec[0])) as u32;
                    a.gns_lon_mm += (dlon > tol_deg_e7(ep.gns_dec[1], ep.gga_dec[1])) as u32;
                }
                if let (Some(x), Some(y)) = (n.alt_mm, g.alt_mm) {
                    a.gns_alt_cmp += 1;
                    let d = i64::from(x - y).abs();
                    a.max_dalt = a.max_dalt.max(d);
                    a.gns_alt_mm += (d > tol_mm(ep.gns_dec[2], ep.gga_dec[2])) as u32;
                }
                if let (Some(x), Some(y)) = (n.sats_used, g.sats_used) {
                    a.gns_sat_cmp += 1;
                    a.gns_sat_mm += (x != y) as u32;
                }
            }
        }
    }

    if let Some(s) = ep.gst {
        a.gst_n += 1;
        if let (Some(_), Some(g)) = (s.time, ep.gga.and_then(|g| g.time)) {
            a.gst_tcmp += 1;
            a.gst_tmm += (s.time != Some(g)) as u32;
        }
        a.gst_neg += s.orientation_cdeg.is_some_and(|o| o < 0) as u32;
        let missing = [
            s.rms_mm,
            s.semi_major_mm,
            s.semi_minor_mm,
            s.lat_std_mm,
            s.lon_std_mm,
            s.alt_std_mm,
        ]
        .iter()
        .filter(|v| v.is_none())
        .count()
            + s.orientation_cdeg.is_none() as usize;
        if fix {
            a.gst_fix_n += 1;
            a.gst_fix_missing += (missing > 0) as u32;
        } else {
            a.gst_nofix += 1;
        }
        a.gst_last = Some(s);
    }

    for g in ep.grs.iter().flatten() {
        a.grs_n += 1;
        if let (Some(_), Some(t)) = (g.time, ep.gga.and_then(|g| g.time)) {
            a.grs_tcmp += 1;
            a.grs_tmm += (g.time != Some(t)) as u32;
        }
        let gsa = ep
            .gsa
            .iter()
            .flatten()
            .find(|s| match (g.system_id, s.system_id) {
                (Some(x), Some(y)) => x == y,
                _ => s.talker == g.talker,
            });
        match gsa {
            None => a.grs_nogsa += 1,
            Some(s) => {
                let ns = s.sats.iter().flatten().count();
                let nr = g.residual_count();
                if ns == 0 && nr == 0 {
                    a.grs_empty += 1;
                } else {
                    a.grs_cmp += 1;
                    if ns != nr {
                        if let Some(slot) = a.grs_first.get_mut(a.grs_mm as usize) {
                            *slot = (g.system_id.unwrap_or(0), nr as u8, ns as u8);
                        }
                        a.grs_mm += 1;
                    }
                }
            }
        }
    }
}

/// Everything one reading window produced.
struct Win {
    tally: [Tally; 4],
    acc: Acc,
    total: u32,
    fixes: u32,
    fixes_valid: u32,
    dropped: u32,
    checksum_errors: u32,
    uart_errors: u32,
    elapsed_ms: u32,
    first_order: Line,
}

fn handle(s: &Sentence<'_>, ep: &mut Epoch, w: &mut Win, builder: &mut FixBuilder) {
    let addr = s.address();
    w.total += 1;
    ep.n += 1;
    let _ = write!(
        ep.order,
        "{} ",
        addr.get(2..).filter(|_| addr.len() == 5).unwrap_or(addr)
    );
    let msg = s.parse();
    if let Some(fix) = builder.push(&msg) {
        w.fixes += 1;
        w.fixes_valid += fix.valid as u32;
    }
    if let Some(k) = kind_of(addr) {
        let t = &mut w.tally[k];
        t.arrived += 1;
        let mut f = [""; 24];
        let mut nf = 0;
        for (i, x) in s.fields().enumerate() {
            if let Some(slot) = f.get_mut(i) {
                *slot = x;
            }
            nf = i + 1;
        }
        t.nfields[nf.min(23)] += 1;
        if t.example.len == 0 {
            masked(s.as_str(), &mut t.example);
        }
        if k == 1 {
            t.add_mode(f[5]);
            t.units_m += (f[9] == "M" && f[11] == "M") as u16;
        }
        let ok = matches!(
            (k, &msg),
            (0, Message::Zda(_))
                | (1, Message::Gns(_))
                | (2, Message::Gst(_))
                | (3, Message::Grs(_))
        );
        if ok {
            t.decoded += 1;
        } else if matches!(msg, Message::Malformed) {
            t.rejected += 1;
            if k == 1 {
                ep.gns_raw = Line::new();
                let _ = write!(ep.gns_raw, "{}", s.as_str());
            }
            if t.nbad < 3 {
                masked(s.as_str(), &mut t.bad[t.nbad]);
                t.nbad += 1;
            }
        } else {
            t.other += 1;
        }
    }
    match msg {
        Message::Rmc(r) => ep.rmc = Some(r),
        Message::Gga(g) => {
            ep.gga = Some(g);
            ep.gga_raw = Line::new();
            let _ = write!(ep.gga_raw, "{}", s.as_str());
            ep.gga_dec = [
                decimals(s.fields().nth(1).unwrap_or("")),
                decimals(s.fields().nth(3).unwrap_or("")),
                decimals(s.fields().nth(8).unwrap_or("")),
            ];
        }
        Message::Gsa(g) => {
            if let Some(slot) = ep.gsa.iter_mut().find(|x| x.is_none()) {
                *slot = Some(g);
            }
        }
        Message::Zda(z) => ep.zda = Some(z),
        Message::Gns(g) => {
            ep.gns = Some(g);
            ep.gns_dec = [
                decimals(s.fields().nth(1).unwrap_or("")),
                decimals(s.fields().nth(3).unwrap_or("")),
                decimals(s.fields().nth(8).unwrap_or("")),
            ];
        }
        Message::Gst(g) => ep.gst = Some(g),
        Message::Grs(g) => {
            if let Some(slot) = ep.grs.iter_mut().find(|x| x.is_none()) {
                *slot = Some(g);
            }
        }
        _ => {}
    }
}

/// Reads sentences for `secs` seconds, then to the end of the burst in progress. Every sentence goes
/// through `read_sentence`, `parse` and a fresh `FixBuilder`. The first (partial) burst is counted but
/// not cross-checked. Nothing is printed while reading.
async fn collect(gnss: &mut Gnss, secs: u64) -> Win {
    let mut w = Win {
        tally: [const { Tally::new() }; 4],
        acc: Acc::new(),
        total: 0,
        fixes: 0,
        fixes_valid: 0,
        dropped: 0,
        checksum_errors: 0,
        uart_errors: 0,
        elapsed_ms: 0,
        first_order: Line::new(),
    };
    let mut builder = FixBuilder::new();
    let mut ep = Epoch::new();
    let mut skip = true;
    let stats0 = gnss.stats();
    let uart0 = UART_ERRORS.load(Relaxed);
    let t0 = Instant::now();
    let end = t0 + Duration::from_secs(secs);
    let mut last = t0;
    loop {
        let now = Instant::now();
        let until = if now < end { end } else { last + GAP };
        if now >= until {
            break;
        }
        match with_timeout(until - now, gnss.read_sentence()).await {
            Ok(Ok(s)) => {
                let at = Instant::now();
                if at - last > GAP && ep.n > 0 {
                    if !skip {
                        check_epoch(&ep, &mut w.acc);
                        if w.first_order.len == 0 {
                            let _ = write!(w.first_order, "{}", ep.order.as_str());
                        }
                    }
                    skip = false;
                    ep = Epoch::new();
                }
                last = at;
                handle(&s, &mut ep, &mut w, &mut builder);
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => {}
        }
    }
    if ep.n > 0 && !skip {
        check_epoch(&ep, &mut w.acc);
        if w.first_order.len == 0 {
            let _ = write!(w.first_order, "{}", ep.order.as_str());
        }
    }
    let stats1 = gnss.stats();
    w.checksum_errors = stats1.checksum_errors - stats0.checksum_errors;
    w.uart_errors = UART_ERRORS.load(Relaxed) - uart0;
    w.dropped = builder.dropped();
    w.elapsed_ms = t0.elapsed().as_millis() as u32;
    w
}

/// Reads 2.5 s and counts the sentences of each aux kind that arrive in the last 1.3 s.
async fn quiet(gnss: &mut Gnss) -> [u32; 4] {
    let mut late = [0u32; 4];
    let t0 = Instant::now();
    let end = t0 + Duration::from_millis(2500);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_sentence()).await {
            Ok(Ok(s)) => {
                if let Some(k) = kind_of(s.address())
                    && Instant::now() - t0 >= Duration::from_millis(1200)
                {
                    late[k] += 1;
                }
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    late
}

async fn set_rate(gnss: &mut Gnss, failures: &mut u32, sec: &str, k: usize, rate: u8) -> bool {
    let r = with_timeout(
        Duration::from_secs(5),
        gnss.command(&Command::SetSentenceRate {
            sentence: TYPES[k],
            rate,
        }),
    )
    .await;
    let ok = matches!(r, Ok(Ok(())));
    check!(
        *failures,
        ok,
        "{=str} {=str} {=str} SetSentenceRate rate {=u8} acknowledged",
        sec,
        KINDS[k],
        rate
    );
    match r {
        Ok(Err(e)) => {
            count_err(&e);
            defmt::error!("[INFO] {=str} {=str} command error: {}", sec, KINDS[k], e);
        }
        Err(_) => defmt::error!(
            "[INFO] {=str} {=str} no completion within 5 s",
            sec,
            KINDS[k]
        ),
        Ok(Ok(())) => {}
    }
    ok
}

/// Counts and layout of one type. Returns true if the type was emitted.
fn report_type(failures: &mut u32, sec: &str, k: usize, w: &Win, acked: bool) -> bool {
    let t = &w.tally[k];
    let name = KINDS[k];
    let mut hist = Line::new();
    for (n, c) in t.nfields.iter().enumerate() {
        if *c > 0 {
            let _ = write!(hist, "{}fields*{} ", n, c);
        }
    }
    defmt::info!(
        "[STAT] {=str} {=str} in {=u32} ms: arrived {=u16}, decoded {=u16}, rejected (Malformed) {=u16}, other message {=u16}; all sentences {=u32}; layout: {=str}",
        sec,
        name,
        w.elapsed_ms,
        t.arrived,
        t.decoded,
        t.rejected,
        t.other,
        w.total,
        hist.as_str()
    );
    defmt::info!(
        "[STAT] {=str} {=str} window: checksum errors {=u32}, UART errors {=u32}, fixes completed {=u32} ({=u32} valid), epochs dropped {=u32}, complete epochs checked {=u32} ({=u32} with a fix)",
        sec,
        name,
        w.checksum_errors,
        w.uart_errors,
        w.fixes,
        w.fixes_valid,
        w.dropped,
        w.acc.epochs,
        w.acc.epochs_fix
    );
    if t.example.len > 0 {
        defmt::info!(
            "[INFO] {=str} {=str} first sentence seen (masked): {=str}",
            sec,
            name,
            t.example.as_str()
        );
    }
    for i in 0..t.nbad {
        defmt::error!(
            "[INFO] {=str} {=str} REJECTED by the decoder with Malformed (masked): {=str}",
            sec,
            name,
            t.bad[i].as_str()
        );
    }
    if t.arrived == 0 {
        defmt::info!(
            "[INFO] {=str} {=str} not emitted by this firmware: 0 arrived in {=u32} ms (command acknowledged: {=bool})",
            sec,
            name,
            w.elapsed_ms,
            acked
        );
        return false;
    }
    check!(
        *failures,
        t.arrived >= 5 && t.decoded == t.arrived,
        "{=str} {=str} {=str} at least 5 arrived and all decoded: arrived {=u16}, decoded {=u16}, rejected {=u16}, other {=u16}",
        sec,
        name,
        t.arrived,
        t.decoded,
        t.rejected,
        t.other
    );
    true
}

fn skip_reason(a: &Acc) -> &'static str {
    if a.epochs_fix == 0 {
        "no position fix in any checked epoch"
    } else {
        "no epoch had both sentences with values"
    }
}

fn xcheck_zda(failures: &mut u32, sec: &str, a: &Acc) {
    if a.zda_cmp == 0 {
        defmt::info!(
            "[SKIP] {=str} ZDA date and time against RMC: nothing to compare ({=str}); ZDA seen in {=u32} epochs, no RMC in {=u32}, an empty date or time in {=u32}",
            sec,
            skip_reason(a),
            a.zda_n,
            a.zda_no_rmc,
            a.zda_empty
        );
        return;
    }
    check!(
        *failures,
        a.zda_tmm == 0 && a.zda_dmm == 0,
        "{=str} {=str} ZDA time and date equal the RMC of the same epoch: compared {=u32} epochs, time differs {=u32}, date differs {=u32}",
        sec,
        a.zda_cmp,
        a.zda_tmm,
        a.zda_dmm
    );
}

fn xcheck_gns(failures: &mut u32, sec: &str, a: &Acc) {
    if a.raw_n > 0 {
        check!(
            *failures,
            a.raw_layout == 0
                && a.raw_time_mm
                    + a.raw_pos_mm
                    + a.raw_sat_mm
                    + a.raw_hdop_mm
                    + a.raw_alt_mm
                    + a.raw_sep_mm
                    == 0,
            "{=str} {=str} GNS rejected by the decoder, compared as raw text with the GGA of the same epoch (13 data fields: time, position, satellites, HDOP, altitude, separation): epochs {=u32}, other layout {=u32}, differ: time {=u32} position {=u32} satellites {=u32} HDOP {=u32} altitude {=u32} separation {=u32}",
            sec,
            a.raw_n,
            a.raw_layout,
            a.raw_time_mm,
            a.raw_pos_mm,
            a.raw_sat_mm,
            a.raw_hdop_mm,
            a.raw_alt_mm,
            a.raw_sep_mm
        );
    }
    defmt::info!(
        "[STAT] {=str} GNS against GGA: largest difference latitude {=i64} and longitude {=i64} (1e-7 degree), altitude {=i64} mm",
        sec,
        a.max_dlat,
        a.max_dlon,
        a.max_dalt
    );
    if a.gns_tcmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GNS time against GGA: nothing to compare (GNS in {=u32} epochs, no GGA in {=u32})",
            sec,
            a.gns_n,
            a.gns_no_gga
        );
    } else {
        check!(
            *failures,
            a.gns_tmm == 0,
            "{=str} {=str} GNS time equals the GGA time of the same epoch: compared {=u32}, differ {=u32}",
            sec,
            a.gns_tcmp,
            a.gns_tmm
        );
    }
    if a.gns_pos_cmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GNS latitude and longitude against GGA: no epoch with a position in both ({=str}); GNS in {=u32} epochs",
            sec,
            skip_reason(a),
            a.gns_n
        );
    } else {
        check!(
            *failures,
            a.gns_lat_mm == 0 && a.gns_lon_mm == 0,
            "{=str} {=str} GNS latitude and longitude equal GGA within the last printed digit: compared {=u32}, latitude differs {=u32}, longitude differs {=u32}",
            sec,
            a.gns_pos_cmp,
            a.gns_lat_mm,
            a.gns_lon_mm
        );
    }
    if a.gns_alt_cmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GNS altitude against GGA: no epoch with an altitude in both ({=str})",
            sec,
            skip_reason(a)
        );
    } else {
        check!(
            *failures,
            a.gns_alt_mm == 0,
            "{=str} {=str} GNS altitude equals GGA within the last printed digit: compared {=u32}, differ {=u32}",
            sec,
            a.gns_alt_cmp,
            a.gns_alt_mm
        );
    }
    if a.gns_sat_cmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GNS satellite count against GGA: no epoch with a count in both",
            sec
        );
    } else {
        check!(
            *failures,
            a.gns_sat_mm == 0,
            "{=str} {=str} GNS satellites used equal GGA: compared {=u32}, differ {=u32}",
            sec,
            a.gns_sat_cmp,
            a.gns_sat_mm
        );
    }
}

fn xcheck_gst(failures: &mut u32, sec: &str, a: &Acc) {
    if a.gst_n == 0 {
        defmt::info!("[SKIP] {=str} GST checks: no GST in a complete epoch", sec);
        return;
    }
    check!(
        *failures,
        a.gst_neg == 0,
        "{=str} {=str} GST values non-negative (the deviations are unsigned in the decoder; orientation checked): sentences {=u32}, negative {=u32}",
        sec,
        a.gst_n,
        a.gst_neg
    );
    if a.gst_fix_n == 0 {
        defmt::info!(
            "[SKIP] {=str} GST values present and finite with a fix: no epoch with a position fix ({=u32} epochs without)",
            sec,
            a.gst_nofix
        );
    } else {
        check!(
            *failures,
            a.gst_fix_missing == 0,
            "{=str} {=str} GST with a fix: all seven values present: epochs with a fix {=u32}, with a missing value {=u32}",
            sec,
            a.gst_fix_n,
            a.gst_fix_missing
        );
    }
    if a.gst_tcmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GST time against GGA: nothing to compare",
            sec
        );
    } else {
        check!(
            *failures,
            a.gst_tmm == 0,
            "{=str} {=str} GST time equals the GGA time of the same epoch: compared {=u32}, differ {=u32}",
            sec,
            a.gst_tcmp,
            a.gst_tmm
        );
    }
    if let Some(s) = a.gst_last {
        defmt::info!(
            "[STAT] {=str} GST last: rms {} mm, major {} mm, minor {} mm, orientation {} cdeg, lat {} mm, lon {} mm, alt {} mm",
            sec,
            s.rms_mm,
            s.semi_major_mm,
            s.semi_minor_mm,
            s.orientation_cdeg,
            s.lat_std_mm,
            s.lon_std_mm,
            s.alt_std_mm
        );
    }
}

fn xcheck_grs(failures: &mut u32, sec: &str, a: &Acc) {
    if a.grs_n == 0 {
        defmt::info!("[SKIP] {=str} GRS checks: no GRS in a complete epoch", sec);
        return;
    }
    if a.grs_cmp == 0 {
        defmt::info!(
            "[SKIP] {=str} GRS residual count against GSA: no epoch with satellites in use ({=str}); GRS {=u32}, no GSA of the system for {=u32}, both empty {=u32}",
            sec,
            skip_reason(a),
            a.grs_n,
            a.grs_nogsa,
            a.grs_empty
        );
    } else {
        check!(
            *failures,
            a.grs_mm == 0 && a.grs_nogsa == 0,
            "{=str} {=str} GRS carries one residual per satellite of the GSA of the same epoch and system: compared {=u32}, differ {=u32}, no GSA for the system {=u32}",
            sec,
            a.grs_cmp,
            a.grs_mm,
            a.grs_nogsa
        );
        for (sys, nr, ns) in a.grs_first.iter().take(a.grs_mm as usize) {
            defmt::info!(
                "[INFO] {=str} GRS mismatch: system ID {=u8} residuals {=u8}, satellites in GSA {=u8}",
                sec,
                sys,
                nr,
                ns
            );
        }
    }
    if a.grs_tcmp > 0 {
        check!(
            *failures,
            a.grs_tmm == 0,
            "{=str} {=str} GRS time equals the GGA time of the same epoch: compared {=u32}, differ {=u32}",
            sec,
            a.grs_tcmp,
            a.grs_tmm
        );
    }
}

fn report_gns_layout(sec: &str, w: &Win) {
    let t = &w.tally[1];
    for i in 0..t.modes_used {
        let m = core::str::from_utf8(&t.modes[i]).unwrap_or("?");
        defmt::info!(
            "[INFO] {=str} GNS mode indicator field (field 5) seen: {=str} in {=u16} sentences",
            sec,
            m.trim_end_matches('\0'),
            t.modes_n[i]
        );
    }
    defmt::info!(
        "[INFO] {=str} GNS sentences with M in both unit fields (9 and 11): {=u16} of {=u16}",
        sec,
        t.units_m,
        t.arrived
    );
}

async fn type_section(
    gnss: &mut Gnss,
    failures: &mut u32,
    sec: &str,
    k: usize,
    emitted: &mut [bool; 4],
) {
    defmt::info!(
        "[INFO] {=str} {=str}: switching on with rate 1",
        sec,
        KINDS[k]
    );
    let acked = set_rate(gnss, failures, sec, k, 1).await;
    let w = collect(gnss, WINDOW_SECS).await;
    emitted[k] = report_type(failures, sec, k, &w, acked);
    match k {
        0 => xcheck_zda(failures, sec, &w.acc),
        1 => {
            report_gns_layout(sec, &w);
            xcheck_gns(failures, sec, &w.acc);
        }
        2 => xcheck_gst(failures, sec, &w.acc),
        _ => xcheck_grs(failures, sec, &w.acc),
    }
    let off = set_rate(gnss, failures, sec, k, 0).await;
    let late = quiet(gnss).await;
    check!(
        *failures,
        off && late[k] == 0,
        "{=str} {=str} {=str} switched off: acknowledged {=bool}, still arriving in the last 1.3 s of 2.5 s: {=u32}",
        sec,
        KINDS[k],
        off,
        late[k]
    );
}

async fn run(gnss: &mut Gnss, failures: &mut u32) {
    // The reset by the probe leaves a burst of framing errors on the receive line; read through it.
    let end = Instant::now() + Duration::from_millis(500);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_sentence()).await {
            Ok(Err(e)) => count_err(&e),
            Ok(Ok(_)) => {}
            Err(_) => break,
        }
    }
    STARTUP_ERRORS.store(UART_ERRORS.load(Relaxed), Relaxed);

    // A0: start.
    let mut version = None;
    for attempt in 1..=5u32 {
        match with_timeout(Duration::from_secs(4), gnss.version()).await {
            Ok(Ok(v)) => {
                version = Some((v, attempt));
                break;
            }
            Ok(Err(e)) => {
                count_err(&e);
                defmt::info!("[INFO] A0 version attempt {=u32}: {}", attempt, e);
            }
            Err(_) => defmt::info!(
                "[INFO] A0 version attempt {=u32}: no answer in 4 s",
                attempt
            ),
        }
    }
    check!(
        *failures,
        version.is_some(),
        "{=str} A0 link: PQTMVERNO answered"
    );
    let Some((version, attempt)) = version else {
        return;
    };
    defmt::info!(
        "[INFO] A0 firmware {=str}, build date {}, build time {}, answered on attempt {=u32}",
        version.version(),
        version.build_date,
        version.build_time,
        attempt
    );
    defmt::info!(
        "[INFO] A0 GSV off + SetFixInterval(1000) took {=u32} attempts in total (2 = first try, 0 = failed)",
        back_to_1hz(gnss).await
    );
    let r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    check!(
        *failures,
        matches!(r, Ok(Ok(_))),
        "{=str} A0 init(&Config::default()) acknowledged: 1000 ms, ZDA/GRS/GST/GNS rates 0"
    );
    if let Ok(Err(e)) = &r {
        count_err(e);
        defmt::error!("[INFO] A0 init error: {}", e);
    }
    let steps = settle_1hz(gnss, 3).await;
    defmt::info!(
        "[INFO] A0 at 1 Hz: {=u32} fixes read, {=u32} steps not 1000 ms before settling, {=u32} consecutive 1000 ms steps",
        steps.fixes,
        steps.transition,
        steps.settled
    );
    let mut have_fix = false;
    for _ in 0..3 {
        if let Ok(Ok(fix)) = with_timeout(Duration::from_secs(3), gnss.read_fix()).await {
            have_fix = fix.valid;
            defmt::info!(
                "[INFO] A0 position fix: valid {=bool}, quality {}, fix type {}, satellites used {}, HDOP x100 {}",
                fix.valid,
                fix.quality,
                fix.fix_type,
                fix.sats_used,
                fix.hdop_x100
            );
        }
    }
    defmt::info!(
        "[INFO] A0 module has a position fix at the start: {=bool}",
        have_fix
    );

    // A1 to A4: one type at a time.
    let mut emitted = [false; 4];
    for (k, sec) in ["A1", "A2", "A3", "A4"].into_iter().enumerate() {
        type_section(gnss, failures, sec, k, &mut emitted).await;
    }

    // A5: all four together.
    defmt::info!("[INFO] A5 all four on at rate 1, 1 Hz fix interval");
    let mut acked = [false; 4];
    for k in 0..4 {
        acked[k] = set_rate(gnss, failures, "A5", k, 1).await;
    }
    let w = collect(gnss, WINDOW_SECS).await;
    for k in 0..4 {
        let now = report_type(failures, "A5", k, &w, acked[k]);
        if emitted[k] {
            check!(
                *failures,
                now,
                "{=str} A5 {=str} still emitted together with the others (was emitted alone: yes)",
                KINDS[k]
            );
        }
    }
    check!(
        *failures,
        w.checksum_errors == 0,
        "{=str} A5 checksum errors over the window: {=u32}",
        w.checksum_errors
    );
    check!(
        *failures,
        w.uart_errors == 0,
        "{=str} A5 UART errors over the window: {=u32}",
        w.uart_errors
    );
    check!(
        *failures,
        (9..=11).contains(&w.fixes) && w.dropped <= 1,
        "{=str} A5 fixes completed by the fix builder from the same stream: {=u32} in {=u32} ms (expect 10 +-1), epochs dropped {=u32} (<= 1: the partial first burst)",
        w.fixes,
        w.elapsed_ms,
        w.dropped
    );
    let a = &w.acc;
    let cmp = a.raw_n
        + a.zda_cmp
        + a.gns_tcmp
        + a.gns_pos_cmp
        + a.gns_alt_cmp
        + a.gns_sat_cmp
        + a.gst_tcmp
        + a.grs_cmp
        + a.grs_tcmp;
    let mm = a.zda_tmm
        + a.zda_dmm
        + a.gns_tmm
        + a.gns_lat_mm
        + a.gns_lon_mm
        + a.gns_alt_mm
        + a.gns_sat_mm
        + a.gst_tmm
        + a.grs_mm
        + a.grs_tmm
        + a.grs_nogsa
        + a.gst_neg
        + a.gst_fix_missing
        + a.raw_layout
        + a.raw_time_mm
        + a.raw_pos_mm
        + a.raw_sat_mm
        + a.raw_hdop_mm
        + a.raw_alt_mm
        + a.raw_sep_mm;
    if cmp == 0 {
        defmt::info!(
            "[SKIP] A5 cross-checks of the combined stream: nothing to compare ({=str})",
            skip_reason(a)
        );
    } else {
        check!(
            *failures,
            mm == 0,
            "{=str} A5 cross-checks of the combined stream (ZDA/RMC, GNS/GGA, GST, GRS/GSA): comparisons {=u32}, mismatches {=u32}",
            cmp,
            mm
        );
    }
    defmt::info!(
        "[INFO] A5 sentence order of one complete epoch: {=str}",
        w.first_order.as_str()
    );
    let mut off_ok = true;
    for k in 0..4 {
        off_ok &= set_rate(gnss, failures, "A5", k, 0).await;
    }
    let late = quiet(gnss).await;
    check!(
        *failures,
        off_ok && late.iter().all(|n| *n == 0),
        "{=str} A5 all four switched off: acknowledged {=bool}, still arriving in the last 1.3 s: ZDA {=u32} GNS {=u32} GST {=u32} GRS {=u32}",
        off_ok,
        late[0],
        late[1],
        late[2],
        late[3]
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!(
        "[INFO] lc76g_aux: UART1 TX=GPIO24 RX=GPIO25 115200 8N1; positions are never printed"
    );

    let mut tx_buf = [0u8; TX_BUFFER];
    let mut rx_buf = [0u8; RX_BUFFER];
    let uart = BufferedUart::new(
        p.UART1,
        p.PIN_24,
        p.PIN_25,
        Irqs,
        &mut tx_buf,
        &mut rx_buf,
        uart::Config::default(),
    );
    let mut gnss = Lc76g::new(uart);

    run(&mut gnss, &mut failures).await;

    // A6: restore the default configuration.
    let mut r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    if !matches!(r, Ok(Ok(_))) {
        defmt::error!("[INFO] A6 first init failed, trying again after back_to_1hz");
        back_to_1hz(&mut gnss).await;
        r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    }
    check!(
        failures,
        matches!(r, Ok(Ok(_))),
        "{=str} A6 init(&Config::default()) acknowledged"
    );
    if let Ok(Err(e)) = &r {
        defmt::error!("[INFO] A6 init error: {}", e);
    }
    let steps = settle_1hz(&mut gnss, 3).await;
    check!(
        failures,
        steps.settled >= 3,
        "{=str} A6 fix time step 1000 ms over three steps ({=u32} fixes read, {=u32} other steps before)",
        steps.fixes,
        steps.transition
    );
    let w = collect(&mut gnss, 2).await;
    let aux: u16 = w.tally.iter().map(|t| t.arrived).sum();
    check!(
        failures,
        aux == 0 && w.fixes >= 1,
        "{=str} A6 default sentence set: ZDA/GNS/GST/GRS arrived {=u16} in {=u32} ms, fixes {=u32}",
        aux,
        w.elapsed_ms,
        w.fixes
    );
    for (body, want) in [
        ("PAIR051", "$PAIR051,1000*"),
        ("PAIR081", "$PAIR081,0*"),
        ("PAIR063,6", "$PAIR063,6,0*"),
        ("PAIR063,7", "$PAIR063,7,0*"),
        ("PAIR063,8", "$PAIR063,8,0*"),
        ("PAIR063,9", "$PAIR063,9,0*"),
    ] {
        match query(&mut gnss, body).await {
            Ok(line) => check!(
                failures,
                line.as_str().starts_with(want),
                "{=str} A6 read back ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => check!(
                failures,
                false,
                "{=str} A6 read back ${=str}: {=str}",
                body,
                why
            ),
        }
    }
    let errors = UART_ERRORS.load(Relaxed);
    let startup_errors = STARTUP_ERRORS.load(Relaxed);
    check!(
        failures,
        errors == startup_errors,
        "{=str} UART errors after the first 500 ms: {=u32}",
        errors - startup_errors
    );
    defmt::info!(
        "[INFO] UART errors since start: {=u32}, driver stats {}",
        errors,
        gnss.stats()
    );
    hil_common::exit(failures)
}
