//! On-board audit of the whole sentence stream of the LC76G: is every sentence the module sends decoded
//! by the driver? LC76G on UART1 (GPIO24 = TX to the module, GPIO25 = RX from the module, 115200 8N1).
//!
//! The driver reads through `Tee`, a UART wrapper that also feeds every received byte to an independent
//! framer (`Raw`): it verifies each checksum itself and counts sentences by the text between `$` and the
//! first comma. The driver's result for each sentence (decoded as its own type, `Other`, `Malformed`) is
//! paired with the raw sentence by its position in the stream. Windows start and end in a gap of the
//! stream, where the driver has consumed every byte that the wrapper has seen, so both views cover the
//! same bytes. Positions are never printed; sentences that are quoted have the digits of latitude,
//! longitude and altitude replaced by `x`. It never saves to flash, never changes the baud rate, never
//! restarts the module, and ends with `init(&Config::default())`.
#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use critical_section::Mutex;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Duration, Instant, with_timeout};
use lc76g::sentence::Quality;
use lc76g::{Command, Config, Lc76g, Message, NavMode, SentenceType, Stats};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type Gnss = Lc76g<Tee>;

const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 256;
/// Length of one capture window.
const WINDOW_SECS: u64 = 30;
/// A gap of this length without a byte marks the end of a burst: the driver has then consumed
/// everything the wrapper has seen.
const IDLE_MS: u64 = 20;

/// Longest sentence of the driver's framer (`lc76g::nmea::MAX_SENTENCE_LEN`).
const MAXLEN: usize = 128;
const KEYLEN: usize = 10;
const RING: usize = 16;
const ROWS: usize = 48;
const ORDER: usize = 24;

/// UART errors returned by any read: overrun, break, parity, framing.
static UART_ERR: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];

fn uart_errors() -> u32 {
    UART_ERR.iter().map(|e| e.load(Relaxed)).sum()
}

/// Reports one verdict line; the format string starts with `{=str}`, which receives the tag.
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

/// Text buffer for building one report line.
struct Line {
    buf: [u8; 400],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            buf: [0; 400],
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

fn key_str(k: &[u8]) -> &str {
    core::str::from_utf8(k).unwrap_or("?")
}

/// Sentence formatters the driver decodes, whatever the talker.
const SUFFIXES: [&str; 10] = [
    "GGA", "GLL", "GSA", "GSV", "RMC", "VTG", "ZDA", "GST", "GRS", "GNS",
];

fn suffix(key: &str) -> &str {
    if key.len() == 5 && key.starts_with('G') {
        key.get(2..).unwrap_or("")
    } else {
        ""
    }
}

/// True if the driver claims to decode this address (`Talker::from_prefix` and `decode_fields` in
/// `src/sentence.rs`).
fn supported(key: &str) -> bool {
    match key {
        "PAIR001" | "PQTMVERNO" | "PQTMEPE" | "PQTMCFGMSGRATE" | "PQTMSAVEPAR" => true,
        _ => {
            key.len() == 5
                && ["GP", "GL", "GA", "GB", "GQ", "GN"].contains(&key.get(..2).unwrap_or(""))
                && SUFFIXES.contains(&suffix(key))
        }
    }
}

/// Where the decoder of a formatter is.
fn decoder_of(suffix: &str) -> &'static str {
    match suffix {
        "GGA" => "src/sentence.rs fn gga",
        "GLL" => "src/sentence.rs fn gll",
        "GSA" => "src/sentence.rs fn gsa",
        "GSV" => "src/sentence.rs fn gsv",
        "RMC" => "src/sentence.rs fn rmc",
        "VTG" => "src/sentence.rs fn vtg",
        "ZDA" => "src/aux_sentences.rs fn zda",
        "GST" => "src/aux_sentences.rs fn gst",
        "GRS" => "src/aux_sentences.rs fn grs",
        "GNS" => "src/aux_sentences.rs fn gns",
        _ => "src/sentence.rs fn decode / collect_fields (more than 24 fields)",
    }
}

#[derive(Clone, Copy)]
struct Row {
    key: [u8; KEYLEN],
    len: u8,
    /// Sentences with a good checksum, counted from the raw bytes.
    raw: u32,
    /// Lines of this address with a bad checksum, or cut off by the next `$`.
    bad: u32,
    /// The driver's results for the same sentences.
    dec: u32,
    other: u32,
    mal: u32,
}

const ROW0: Row = Row {
    key: [0; KEYLEN],
    len: 0,
    raw: 0,
    bad: 0,
    dec: 0,
    other: 0,
    mal: 0,
};

const IDLE: u8 = 0;
const BODY: u8 = 1;
const SKIP: u8 = 2;

/// The independent view: its own framer, checksum and tally.
struct Raw {
    state: u8,
    buf: [u8; MAXLEN],
    len: usize,
    rows: [Row; ROWS],
    nrows: usize,
    /// Valid sentences since the start; never reset.
    valid_total: u32,
    valid: u32,
    bad: u32,
    long: u32,
    junk: u32,
    ring_key: [[u8; KEYLEN]; RING],
    ring_len: [u8; RING],
    ring_text: [[u8; MAXLEN]; RING],
    ring_tlen: [u8; RING],
    order: [[u8; KEYLEN]; ORDER],
    order_len: [u8; ORDER],
    order_n: usize,
}

static RAW: Mutex<RefCell<Raw>> = Mutex::new(RefCell::new(Raw::new()));

fn with_raw<R>(f: impl FnOnce(&mut Raw) -> R) -> R {
    critical_section::with(|cs| f(&mut RAW.borrow_ref_mut(cs)))
}

fn hex(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

/// `$` body `*` two hex digits, printable ASCII, XOR of the body equal to the digits.
fn good(text: &[u8]) -> bool {
    let n = text.len();
    if n < 4 || text[0] != b'$' || text[n - 3] != b'*' {
        return false;
    }
    if !text.iter().all(|b| (0x20..0x7F).contains(b)) {
        return false;
    }
    let (Some(h), Some(l)) = (hex(text[n - 2]), hex(text[n - 1])) else {
        return false;
    };
    text[1..n - 3].iter().fold(0u8, |a, b| a ^ b) == (h << 4 | l)
}

/// Text from `$` to the first comma or `*`, at most `KEYLEN` bytes.
fn addr_key(text: &[u8]) -> ([u8; KEYLEN], usize) {
    let mut k = [0u8; KEYLEN];
    let mut n = 0;
    for &b in text.iter().skip(1) {
        if b == b',' || b == b'*' || n == KEYLEN {
            break;
        }
        k[n] = if (0x21..0x7F).contains(&b) { b } else { b'?' };
        n += 1;
    }
    (k, n)
}

impl Raw {
    const fn new() -> Self {
        Self {
            state: IDLE,
            buf: [0; MAXLEN],
            len: 0,
            rows: [ROW0; ROWS],
            nrows: 0,
            valid_total: 0,
            valid: 0,
            bad: 0,
            long: 0,
            junk: 0,
            ring_key: [[0; KEYLEN]; RING],
            ring_len: [0; RING],
            ring_text: [[0; MAXLEN]; RING],
            ring_tlen: [0; RING],
            order: [[0; KEYLEN]; ORDER],
            order_len: [0; ORDER],
            order_n: 0,
        }
    }

    /// Starts a new window. The framing state, the ring and `valid_total` carry on.
    fn reset_window(&mut self) {
        self.rows = [ROW0; ROWS];
        self.nrows = 0;
        self.valid = 0;
        self.bad = 0;
        self.long = 0;
        self.junk = 0;
        self.order_n = 0;
    }

    fn row(&mut self, key: &[u8]) -> usize {
        for i in 0..self.nrows {
            if &self.rows[i].key[..self.rows[i].len as usize] == key {
                return i;
            }
        }
        if self.nrows < ROWS {
            let i = self.nrows;
            self.rows[i].key[..key.len()].copy_from_slice(key);
            self.rows[i].len = key.len() as u8;
            self.nrows += 1;
            i
        } else {
            ROWS - 1
        }
    }

    fn bad_line(&mut self) {
        let (k, n) = addr_key(&self.buf[..self.len]);
        self.bad += 1;
        let r = self.row(&k[..n]);
        self.rows[r].bad += 1;
    }

    fn finish(&mut self) {
        if !good(&self.buf[..self.len]) {
            self.bad_line();
            return;
        }
        let (k, n) = addr_key(&self.buf[..self.len]);
        self.valid += 1;
        self.valid_total = self.valid_total.wrapping_add(1);
        let r = self.row(&k[..n]);
        self.rows[r].raw += 1;
        let slot = self.valid_total as usize % RING;
        self.ring_key[slot] = k;
        self.ring_len[slot] = n as u8;
        let len = self.len;
        self.ring_text[slot][..len].copy_from_slice(&self.buf[..len]);
        self.ring_tlen[slot] = len as u8;
        if self.order_n < ORDER {
            self.order[self.order_n] = k;
            self.order_len[self.order_n] = n as u8;
            self.order_n += 1;
        }
    }

    fn feed(&mut self, data: &[u8]) {
        for &b in data {
            match (self.state, b) {
                (state, b'$') => {
                    if state == BODY {
                        self.bad_line();
                    }
                    self.buf[0] = b'$';
                    self.len = 1;
                    self.state = BODY;
                }
                (IDLE, b'\r' | b'\n') => {}
                (IDLE, _) => self.junk += 1,
                (SKIP, b'\r' | b'\n') => self.state = IDLE,
                (SKIP, _) => {}
                (_, b'\r' | b'\n') => {
                    self.state = IDLE;
                    self.finish();
                }
                (_, _) => {
                    if self.len < MAXLEN {
                        self.buf[self.len] = b;
                        self.len += 1;
                    } else {
                        self.long += 1;
                        self.state = SKIP;
                    }
                }
            }
        }
    }
}

/// The UART the driver reads and writes, also feeding every received byte to `RAW`.
struct Tee {
    uart: BufferedUart,
}

impl embedded_io_async::ErrorType for Tee {
    type Error = uart::Error;
}

impl embedded_io_async::Read for Tee {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        match self.uart.read(buf).await {
            Ok(n) => {
                with_raw(|r| r.feed(&buf[..n]));
                Ok(n)
            }
            Err(e) => {
                let k = match e {
                    uart::Error::Overrun => 0,
                    uart::Error::Break => 1,
                    uart::Error::Parity => 2,
                    _ => 3,
                };
                UART_ERR[k].fetch_add(1, Relaxed);
                Err(e)
            }
        }
    }
}

impl embedded_io_async::Write for Tee {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.uart.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.uart.flush().await
    }
}

#[derive(PartialEq)]
enum Class {
    Decoded,
    Other,
    Malformed,
}

fn classify(m: &Message) -> Class {
    match m {
        Message::Other => Class::Other,
        Message::Malformed => Class::Malformed,
        _ => Class::Decoded,
    }
}

/// True if the decoded variant is the one that the raw address announces.
fn variant_matches(m: &Message, key: &str) -> bool {
    let s = suffix(key);
    match m {
        Message::Gga(_) => s == "GGA",
        Message::Rmc(_) => s == "RMC",
        Message::Gsa(_) => s == "GSA",
        Message::Gsv(_) => s == "GSV",
        Message::Vtg(_) => s == "VTG",
        Message::Gll(_) => s == "GLL",
        Message::Zda(_) => s == "ZDA",
        Message::Gns(_) => s == "GNS",
        Message::Gst(_) => s == "GST",
        Message::Grs(_) => s == "GRS",
        Message::Ack(_) => key == "PAIR001",
        Message::Version(_) => key == "PQTMVERNO",
        Message::Epe(_) => key == "PQTMEPE",
        Message::PqtmReply(_) => key == "PQTMCFGMSGRATE" || key == "PQTMSAVEPAR",
        Message::Other | Message::Malformed => true,
    }
}

struct Found {
    tag: &'static str,
    decoder: &'static str,
    line: Line,
}

impl Found {
    const fn new() -> Self {
        Self {
            tag: "",
            decoder: "",
            line: Line::new(),
        }
    }
}

/// One capture window: the driver's side of the comparison.
struct Win {
    s0: Stats,
    s1: Stats,
    /// `valid_total` of the raw view when the window opened.
    base: u32,
    /// Messages the driver returned.
    n: u32,
    /// Messages whose raw sentence was no longer in the ring.
    lost: u32,
    /// Messages whose variant differs from the raw address.
    mism: u32,
    aligned_start: bool,
    aligned_end: bool,
    err0: u32,
    err1: u32,
    ms: u32,
    found: [Found; 4],
    nfound: usize,
    gns: Line,
    has_gns: bool,
}

/// Pairs message number `w.n` of the window with its raw sentence and tallies the driver's result.
fn attribute(w: &mut Win, m: &Message) {
    w.n += 1;
    let s = w.base.wrapping_add(w.n);
    let class = classify(m);
    with_raw(|r| {
        if r.valid_total.wrapping_sub(s) >= RING as u32 {
            w.lost += 1;
            return;
        }
        let slot = s as usize % RING;
        let klen = r.ring_len[slot] as usize;
        let mut key = [0u8; KEYLEN];
        key[..klen].copy_from_slice(&r.ring_key[slot][..klen]);
        let ks = key_str(&key[..klen]);
        let idx = r.row(&key[..klen]);
        match class {
            Class::Decoded => r.rows[idx].dec += 1,
            Class::Other => r.rows[idx].other += 1,
            Class::Malformed => r.rows[idx].mal += 1,
        }
        let text = key_str(&r.ring_text[slot][..r.ring_tlen[slot] as usize]);
        let tag = if !variant_matches(m, ks) {
            w.mism += 1;
            "variant differs from the raw address"
        } else if class == Class::Malformed {
            "Malformed"
        } else if class == Class::Other && SUFFIXES.contains(&suffix(ks)) {
            "Other for a decodable formatter (talker not accepted?)"
        } else {
            ""
        };
        if !tag.is_empty() && w.nfound < w.found.len() {
            let f = &mut w.found[w.nfound];
            f.tag = tag;
            f.decoder = if tag.starts_with("Other") {
                "src/sentence.rs fn Talker::from_prefix / decode_fields"
            } else {
                decoder_of(suffix(ks))
            };
            masked(text, &mut f.line);
            w.nfound += 1;
        }
    });
}

#[derive(Clone, Copy)]
struct Chk {
    n: u32,
    bad: u32,
    maxd: i64,
    first: Option<(i64, i64)>,
}

impl Chk {
    const fn new() -> Self {
        Self {
            n: 0,
            bad: 0,
            maxd: 0,
            first: None,
        }
    }

    fn add(&mut self, ok: bool, d: i64, a: i64, b: i64) {
        self.n += 1;
        self.maxd = self.maxd.max(d);
        if !ok {
            self.bad += 1;
            if self.first.is_none() {
                self.first = Some((a, b));
            }
        }
    }
}

/// The messages of one epoch: from the first sentence of a once-per-epoch type to the next repeat of
/// one.
#[derive(Clone, Copy)]
struct Group {
    gga: Option<lc76g::sentence::Gga>,
    rmc: Option<lc76g::sentence::Rmc>,
    gll: Option<lc76g::sentence::Gll>,
    vtg: Option<lc76g::sentence::Vtg>,
    zda: Option<lc76g::aux_sentences::Zda>,
    gst: Option<lc76g::aux_sentences::Gst>,
    mask: u8,
    gsa_n: u8,
    gsa_slots: u16,
}

impl Group {
    const fn new() -> Self {
        Self {
            gga: None,
            rmc: None,
            gll: None,
            vtg: None,
            zda: None,
            gst: None,
            mask: 0,
            gsa_n: 0,
            gsa_slots: 0,
        }
    }
}

/// T4: cross-checks between the decoded sentences of one epoch, accumulated over T1 to T3.
struct T4 {
    g: Group,
    groups: u32,
    t_rmc: Chk,
    t_gll: Chk,
    t_zda: Chk,
    t_gst: Chk,
    pos_rmc: Chk,
    pos_gll: Chk,
    status: Chk,
    sats: Chk,
    speed: Chk,
    date: Chk,
}

fn ymd(d: lc76g::sentence::Date) -> i64 {
    d.year as i64 * 10_000 + d.month as i64 * 100 + d.day as i64
}

impl T4 {
    const fn new() -> Self {
        Self {
            g: Group::new(),
            groups: 0,
            t_rmc: Chk::new(),
            t_gll: Chk::new(),
            t_zda: Chk::new(),
            t_gst: Chk::new(),
            pos_rmc: Chk::new(),
            pos_gll: Chk::new(),
            status: Chk::new(),
            sats: Chk::new(),
            speed: Chk::new(),
            date: Chk::new(),
        }
    }

    fn push(&mut self, m: &Message) {
        let bit = match m {
            Message::Gga(_) => 1,
            Message::Rmc(_) => 2,
            Message::Gll(_) => 4,
            Message::Vtg(_) => 8,
            Message::Zda(_) => 16,
            Message::Gst(_) => 32,
            _ => 0,
        };
        if bit != 0 && self.g.mask & bit != 0 {
            self.close();
        }
        match m {
            Message::Gga(x) => self.g.gga = Some(*x),
            Message::Rmc(x) => self.g.rmc = Some(*x),
            Message::Gll(x) => self.g.gll = Some(*x),
            Message::Vtg(x) => self.g.vtg = Some(*x),
            Message::Zda(x) => self.g.zda = Some(*x),
            Message::Gst(x) => self.g.gst = Some(*x),
            Message::Gsa(a) => {
                self.g.gsa_n += 1;
                self.g.gsa_slots += a.sats.iter().flatten().count() as u16;
            }
            _ => {}
        }
        self.g.mask |= bit;
    }

    fn close(&mut self) {
        let g = self.g;
        self.g = Group::new();
        if g.mask == 0 {
            return;
        }
        self.groups += 1;
        if let Some(a) = g.gga {
            let ta = a.time.map(|t| t.millis_of_day() as i64);
            let tr = g.rmc.and_then(|r| r.time).map(|t| t.millis_of_day() as i64);
            let tl = g.gll.and_then(|r| r.time).map(|t| t.millis_of_day() as i64);
            let tz = g.zda.and_then(|r| r.time).map(|t| t.millis_of_day() as i64);
            let ts = g.gst.and_then(|r| r.time).map(|t| t.millis_of_day() as i64);
            if let Some(ta) = ta {
                for (chk, t) in [
                    (&mut self.t_rmc, tr),
                    (&mut self.t_gll, tl),
                    (&mut self.t_zda, tz),
                    (&mut self.t_gst, ts),
                ] {
                    if let Some(t) = t {
                        chk.add(t == ta, (t - ta).abs(), t, ta);
                    }
                }
            }
            if let (Some(r), Some(la), Some(lo)) = (g.rmc, a.lat_e7, a.lon_e7)
                && let (Some(ra), Some(ro)) = (r.lat_e7, r.lon_e7)
            {
                let d = (ra as i64 - la as i64)
                    .abs()
                    .max((ro as i64 - lo as i64).abs());
                self.pos_rmc.add(d <= 2, d, d, 0);
            }
            if let (Some(l), Some(la), Some(lo)) = (g.gll, a.lat_e7, a.lon_e7)
                && let (Some(ra), Some(ro)) = (l.lat_e7, l.lon_e7)
            {
                let d = (ra as i64 - la as i64)
                    .abs()
                    .max((ro as i64 - lo as i64).abs());
                self.pos_gll.add(d <= 2, d, d, 0);
            }
            if let Some(r) = g.rmc {
                let fixed = a.quality != Quality::NoFix;
                self.status
                    .add(r.valid == fixed, 0, r.valid as i64, a.quality as i64);
            }
            if g.gsa_n > 0
                && let Some(su) = a.sats_used
            {
                let slots = g.gsa_slots as i64;
                self.sats.add(
                    su as i64 == slots,
                    (su as i64 - slots).abs(),
                    su as i64,
                    slots,
                );
            }
        }
        if let (Some(r), Some(v)) = (g.rmc, g.vtg)
            && let (Some(a), Some(b)) = (r.speed_mm_s, v.speed_mm_s)
        {
            let d = (a as i64 - b as i64).abs();
            self.speed.add(d <= 25, d, a as i64, b as i64);
        }
        if let (Some(r), Some(z)) = (g.rmc, g.zda)
            && let (Some(a), Some(b)) = (r.date, z.date)
        {
            self.date.add(a == b, 0, ymd(a), ymd(b));
        }
    }
}

/// Reads and discards messages for `ms` milliseconds.
async fn skip(gnss: &mut Gnss, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        if with_timeout(end - now, gnss.read()).await.is_err() {
            break;
        }
    }
}

/// Reads until a gap of `IDLE_MS` without a byte; false if there is none within 4 s.
async fn align(gnss: &mut Gnss) -> bool {
    let end = Instant::now() + Duration::from_secs(4);
    while Instant::now() < end {
        if with_timeout(Duration::from_millis(IDLE_MS), gnss.read())
            .await
            .is_err()
        {
            return true;
        }
    }
    false
}

/// Counts the stream for `secs` seconds through the driver, from a gap to a gap.
async fn capture(gnss: &mut Gnss, secs: u64, t4: &mut T4) -> Win {
    let aligned_start = align(gnss).await;
    let s0 = gnss.stats();
    let base = with_raw(|r| {
        r.reset_window();
        r.valid_total
    });
    let mut w = Win {
        s0,
        s1: s0,
        base,
        n: 0,
        lost: 0,
        mism: 0,
        aligned_start,
        aligned_end: false,
        err0: uart_errors(),
        err1: 0,
        ms: 0,
        found: [const { Found::new() }; 4],
        nfound: 0,
        gns: Line::new(),
        has_gns: false,
    };
    let t0 = Instant::now();
    let end = t0 + Duration::from_secs(secs);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        if let Ok(Ok(m)) = with_timeout(end - now, gnss.read()).await {
            attribute(&mut w, &m);
            t4.push(&m);
        }
    }
    let drain_end = Instant::now() + Duration::from_secs(4);
    while Instant::now() < drain_end {
        match with_timeout(Duration::from_millis(IDLE_MS), gnss.read()).await {
            Ok(Ok(m)) => {
                attribute(&mut w, &m);
                t4.push(&m);
            }
            Ok(Err(_)) => {}
            Err(_) => {
                w.aligned_end = true;
                break;
            }
        }
    }
    t4.close();
    w.s1 = gnss.stats();
    w.err1 = uart_errors();
    w.ms = t0.elapsed().as_millis() as u32;
    w
}

/// Raw count of one formatter over all talkers, and of one exact address.
fn raw_of(suffix_wanted: &str) -> u32 {
    with_raw(|r| {
        (0..r.nrows)
            .filter(|&i| suffix(key_str(&r.rows[i].key[..r.rows[i].len as usize])) == suffix_wanted)
            .map(|i| r.rows[i].raw)
            .sum()
    })
}

fn raw_key(wanted: &str) -> u32 {
    with_raw(|r| {
        (0..r.nrows)
            .filter(|&i| key_str(&r.rows[i].key[..r.rows[i].len as usize]) == wanted)
            .map(|i| r.rows[i].raw)
            .sum()
    })
}

/// The table, the comparison of totals and the findings of one window.
fn report(failures: &mut u32, sec: &str, w: &Win) {
    let (rows, nrows, valid, bad, long, junk, order, order_len, order_n) = with_raw(|r| {
        (
            r.rows,
            r.nrows,
            r.valid,
            r.bad,
            r.long,
            r.junk,
            r.order,
            r.order_len,
            r.order_n,
        )
    });
    check!(
        *failures,
        w.aligned_start && w.aligned_end,
        "{=str} {=str} window opened and closed in a gap of the stream (start {=bool}, end {=bool}), {=u32} ms",
        sec,
        w.aligned_start,
        w.aligned_end,
        w.ms
    );
    let mut o = Line::new();
    for i in 0..order_n {
        let _ = write!(o, "{} ", key_str(&order[i][..order_len[i] as usize]));
    }
    defmt::info!(
        "[INFO] {=str} order of the first sentences: {=str}",
        sec,
        o.as_str()
    );
    let mut raw_total = 0u32;
    for row in rows.iter().take(nrows) {
        let ks = key_str(&row.key[..row.len as usize]);
        raw_total += row.raw;
        defmt::info!(
            "[STAT] {=str} {=str} raw={=u32} badchecksum={=u32} decoded={=u32} other={=u32} malformed={=u32}",
            sec,
            ks,
            row.raw,
            row.bad,
            row.dec,
            row.other,
            row.mal
        );
        if supported(ks) {
            check!(
                *failures,
                row.dec == row.raw && row.other == 0 && row.mal == 0,
                "{=str} {=str} {=str} decoded as its own type: raw={=u32} decoded={=u32} other={=u32} malformed={=u32}",
                sec,
                ks,
                row.raw,
                row.dec,
                row.other,
                row.mal
            );
        } else if SUFFIXES.contains(&suffix(ks)) {
            check!(
                *failures,
                false,
                "{=str} {=str} {=str} formatter is decodable but the talker is not accepted: raw={=u32} decoded={=u32} other={=u32} malformed={=u32}",
                sec,
                ks,
                row.raw,
                row.dec,
                row.other,
                row.mal
            );
        } else {
            defmt::info!(
                "[INFO] {=str} {=str} not decoded by the driver: raw={=u32} other={=u32} malformed={=u32}",
                sec,
                ks,
                row.raw,
                row.other,
                row.mal
            );
            check!(
                *failures,
                row.mal == 0,
                "{=str} {=str} {=str} undecoded type is not reported Malformed: raw={=u32} malformed={=u32}",
                sec,
                ks,
                row.raw,
                row.mal
            );
        }
    }
    let drv = w.s1.sentences.wrapping_sub(w.s0.sentences);
    check!(
        *failures,
        raw_total.abs_diff(drv) <= 1 && valid == raw_total,
        "{=str} {=str} totals: raw sentences {=u32}, driver sentences {=u32}",
        sec,
        raw_total,
        drv
    );
    check!(
        *failures,
        w.n == drv && w.lost == 0 && w.mism == 0,
        "{=str} {=str} sentences paired with the raw stream: messages {=u32} against driver sentences {=u32}, lost {=u32}, variant mismatches {=u32}",
        sec,
        w.n,
        drv,
        w.lost,
        w.mism
    );
    let dcs = w.s1.checksum_errors.wrapping_sub(w.s0.checksum_errors);
    check!(
        *failures,
        dcs == bad,
        "{=str} {=str} checksum errors: raw {=u32}, driver {=u32}",
        sec,
        bad,
        dcs
    );
    let dov = w.s1.overflows.wrapping_sub(w.s0.overflows);
    check!(
        *failures,
        dov == 0 && long == 0,
        "{=str} {=str} over-long sentences: raw {=u32}, driver {=u32}",
        sec,
        long,
        dov
    );
    defmt::info!(
        "[STAT] {=str} bytes outside any sentence (not CR LF): {=u32}; driver fixes {=u32}, epochs dropped {=u32}, fixes discarded {=u32}",
        sec,
        junk,
        w.s1.fixes.wrapping_sub(w.s0.fixes),
        w.s1.epochs_dropped.wrapping_sub(w.s0.epochs_dropped),
        w.s1.fixes_discarded.wrapping_sub(w.s0.fixes_discarded)
    );
    check!(
        *failures,
        w.err1 == w.err0,
        "{=str} {=str} UART errors in the window: {=u32} (overrun {=u32}, break {=u32}, parity {=u32}, framing {=u32} since the start)",
        sec,
        w.err1.wrapping_sub(w.err0),
        UART_ERR[0].load(Relaxed),
        UART_ERR[1].load(Relaxed),
        UART_ERR[2].load(Relaxed),
        UART_ERR[3].load(Relaxed)
    );
    for f in w.found.iter().take(w.nfound) {
        check!(
            *failures,
            false,
            "{=str} {=str} FINDING {=str}: {=str} (decoder: {=str})",
            sec,
            f.tag,
            f.line.as_str(),
            f.decoder
        );
    }
    if w.has_gns {
        defmt::info!(
            "[INFO] {=str} GNS sample (masked): {=str}",
            sec,
            w.gns.as_str()
        );
    }
}

/// Fixes the driver completed against the GGA + RMC pairs of the raw stream.
fn fix_pairs(failures: &mut u32, sec: &str, w: &Win) {
    let fixes = w.s1.fixes.wrapping_sub(w.s0.fixes);
    let gga = raw_of("GGA");
    let rmc = raw_of("RMC");
    let pairs = gga.min(rmc);
    check!(
        *failures,
        fixes.abs_diff(pairs) <= 1,
        "{=str} {=str} fixes delivered by the driver {=u32} against GGA + RMC pairs in the raw stream {=u32} (GGA {=u32}, RMC {=u32}), window {=u32} ms",
        sec,
        fixes,
        pairs,
        gga,
        rmc,
        w.ms
    );
}

async fn back_to_1hz(gnss: &mut Gnss) -> bool {
    for command in [
        Command::SetSentenceRate {
            sentence: SentenceType::Gsv,
            rate: 0,
        },
        Command::SetFixInterval { ms: 1000 },
    ] {
        let mut done = false;
        for _ in 1..=10u32 {
            if let Ok(Ok(())) = with_timeout(Duration::from_secs(4), gnss.command(&command)).await {
                done = true;
                break;
            }
        }
        if !done {
            return false;
        }
    }
    true
}

/// `init(&Config::default())`, once more after `back_to_1hz` if the first try fails.
async fn restore_default(gnss: &mut Gnss) -> bool {
    let mut r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    if !matches!(r, Ok(Ok(_))) {
        defmt::error!("[INFO] first init(default) failed, trying again after back_to_1hz");
        back_to_1hz(gnss).await;
        r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    }
    if let Ok(Err(e)) = &r {
        defmt::error!("[INFO] init(default) error: {}", e);
    }
    matches!(r, Ok(Ok(_)))
}

/// Sends a get command through `Command::Raw` and returns the reply sentence that follows the
/// acknowledgement.
async fn query(gnss: &mut Gnss, body: &str) -> Result<Line, &'static str> {
    let mut want = Line::new();
    let _ = write!(want, "${body},");
    match with_timeout(Duration::from_secs(3), gnss.command(&Command::Raw(body))).await {
        Ok(Ok(())) => {}
        Ok(Err(_)) => return Err("command failed"),
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
            Ok(_) => {}
            Err(_) => break,
        }
    }
    Err("acknowledged, but no reply sentence followed")
}

async fn set_rate(
    gnss: &mut Gnss,
    failures: &mut u32,
    sec: &str,
    sentence: SentenceType,
    rate: u8,
) {
    let r = with_timeout(
        Duration::from_secs(5),
        gnss.command(&Command::SetSentenceRate { sentence, rate }),
    )
    .await;
    check!(
        *failures,
        matches!(r, Ok(Ok(()))),
        "{=str} {=str} SetSentenceRate {} = {=u8} acknowledged",
        sec,
        sentence,
        rate
    );
}

fn t4_line(failures: &mut u32, name: &str, c: &Chk, unit: &str) {
    if c.n == 0 {
        defmt::info!(
            "[SKIP] T4 {=str}: no epoch carried both sentences ({=str})",
            name,
            "the type was not emitted or has no value to compare"
        );
        return;
    }
    check!(
        *failures,
        c.bad == 0,
        "{=str} T4 {=str}: {=u32} compared, {=u32} differ, largest difference {=i64} {=str}",
        name,
        c.n,
        c.bad,
        c.maxd,
        unit
    );
    if let Some((a, b)) = c.first {
        defmt::info!(
            "[INFO] T4 {=str} first difference: {=i64} against {=i64}",
            name,
            a,
            b
        );
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!(
        "[INFO] lc76g_stream: UART1 TX=GPIO24 RX=GPIO25 115200 8N1; positions are never printed"
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
    let mut gnss = Lc76g::new(Tee { uart });
    let mut t4 = T4::new();

    // The reset by the probe leaves a burst of framing errors at the start: read through the first
    // second, the errors after it are counted.
    skip(&mut gnss, 1000).await;
    let startup_errors = uart_errors();
    defmt::info!(
        "[INFO] UART errors in the first second: {=u32}",
        startup_errors
    );

    // T0: start.
    let mut version = None;
    for attempt in 1..=5u32 {
        if let Ok(Ok(v)) = with_timeout(Duration::from_secs(4), gnss.version()).await {
            version = Some((v, attempt));
            break;
        }
    }
    check!(failures, version.is_some(), "{=str} T0 PQTMVERNO answered");
    if let Some((v, attempt)) = &version {
        defmt::info!(
            "[INFO] T0 firmware {=str}, answered on attempt {=u32}",
            v.version(),
            *attempt
        );
    }
    let ok = restore_default(&mut gnss).await;
    check!(
        failures,
        ok,
        "{=str} T0 init(&Config::default()) acknowledged"
    );
    skip(&mut gnss, 2000).await;
    let mut rmc_valid = false;
    let mut sats = 0u8;
    let end = Instant::now() + Duration::from_secs(8);
    while Instant::now() < end && !(rmc_valid && sats > 0) {
        let left = end.saturating_duration_since(Instant::now());
        match with_timeout(left, gnss.read()).await {
            Ok(Ok(Message::Rmc(r))) => rmc_valid = r.valid,
            Ok(Ok(Message::Gga(g))) if g.quality != Quality::NoFix => {
                sats = g.sats_used.unwrap_or(0)
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
    check!(
        failures,
        rmc_valid && sats > 0,
        "{=str} T0 position fix: RMC valid {=bool}, satellites used {=u8}",
        rmc_valid,
        sats
    );

    // T1: default stream at 1 Hz.
    let w = capture(&mut gnss, WINDOW_SECS, &mut t4).await;
    report(&mut failures, "T1", &w);

    // T2: flight configuration, as lc76g_smoke configures it.
    let cfg = Config {
        nav_mode: NavMode::Drone,
        epe: true,
        ..Config::with_fix_interval(100)
    };
    let r = with_timeout(Duration::from_secs(15), gnss.init(&cfg)).await;
    check!(
        failures,
        matches!(r, Ok(Ok(_))),
        "{=str} T2 init(with_fix_interval(100), drone, EPE on) acknowledged"
    );
    skip(&mut gnss, 2000).await;
    let w = capture(&mut gnss, WINDOW_SECS, &mut t4).await;
    report(&mut failures, "T2", &w);
    fix_pairs(&mut failures, "T2", &w);

    // T3: everything on at 1 Hz.
    let ok = restore_default(&mut gnss).await;
    check!(
        failures,
        ok,
        "{=str} T3 init(&Config::default()) acknowledged"
    );
    for s in [
        SentenceType::Zda,
        SentenceType::Grs,
        SentenceType::Gst,
        SentenceType::Gns,
    ] {
        set_rate(&mut gnss, &mut failures, "T3", s, 1).await;
    }
    skip(&mut gnss, 2000).await;
    let w = capture(&mut gnss, WINDOW_SECS, &mut t4).await;
    report(&mut failures, "T3", &w);

    // T4: field sanity of the decoded stream, T1 to T3.
    defmt::info!("[INFO] T4 epochs examined: {=u32}", t4.groups);
    t4_line(&mut failures, "time RMC = GGA", &t4.t_rmc, "ms");
    t4_line(&mut failures, "time GLL = GGA", &t4.t_gll, "ms");
    t4_line(&mut failures, "time ZDA = GGA", &t4.t_zda, "ms");
    t4_line(&mut failures, "time GST = GGA", &t4.t_gst, "ms");
    t4_line(
        &mut failures,
        "position RMC = GGA (1e-7 degree)",
        &t4.pos_rmc,
        "units",
    );
    t4_line(
        &mut failures,
        "position GLL = GGA (1e-7 degree)",
        &t4.pos_gll,
        "units",
    );
    t4_line(
        &mut failures,
        "RMC status A = GGA quality not 0",
        &t4.status,
        "",
    );
    t4_line(
        &mut failures,
        "GGA satellites used = non-empty GSA slots of the epoch",
        &t4.sats,
        "satellites",
    );
    t4_line(
        &mut failures,
        "RMC speed = VTG speed (tolerance 25 mm/s)",
        &t4.speed,
        "mm/s",
    );
    t4_line(&mut failures, "RMC date = ZDA date", &t4.date, "");
    for name in ["GSV", "GRS", "GNS"] {
        defmt::info!(
            "[SKIP] T4 {=str}: no value shared with another sentence of the epoch that the packet names (GNS decodes, no partner)",
            name
        );
    }

    // T5: end. Extras off, default configuration, read back, health, verdict.
    for s in [
        SentenceType::Zda,
        SentenceType::Grs,
        SentenceType::Gst,
        SentenceType::Gns,
    ] {
        set_rate(&mut gnss, &mut failures, "T5", s, 0).await;
    }
    let ok = restore_default(&mut gnss).await;
    check!(
        failures,
        ok,
        "{=str} T5 init(&Config::default()) acknowledged"
    );
    skip(&mut gnss, 2000).await;
    let w = capture(&mut gnss, 3, &mut t4).await;
    let extras = raw_of("ZDA") + raw_of("GRS") + raw_of("GST") + raw_of("GNS") + raw_key("PQTMEPE");
    check!(
        failures,
        extras == 0 && raw_of("GGA") > 0 && raw_of("RMC") > 0,
        "{=str} T5 sentence set is the default: GGA {=u32}, RMC {=u32}, extras (ZDA, GRS, GST, GNS, EPE) {=u32}, decoded messages {=u32}",
        raw_of("GGA"),
        raw_of("RMC"),
        extras,
        w.n
    );
    for (body, want) in [("PAIR051", "$PAIR051,1000*"), ("PAIR081", "$PAIR081,0*")] {
        match query(&mut gnss, body).await {
            Ok(line) => check!(
                failures,
                line.as_str().starts_with(want),
                "{=str} T5 read back ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => check!(
                failures,
                false,
                "{=str} T5 read back ${=str}: {=str}",
                body,
                why
            ),
        }
    }
    let errors = uart_errors();
    check!(
        failures,
        errors == startup_errors,
        "{=str} T5 UART errors after the first second: {=u32}",
        errors.wrapping_sub(startup_errors)
    );
    defmt::info!("[INFO] driver stats since start {}", gnss.stats());
    hil_common::exit(failures)
}
