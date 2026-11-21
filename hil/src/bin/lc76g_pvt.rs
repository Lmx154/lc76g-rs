//! Bench probe of the proprietary `$PQTMPVT` sentence (position, NED velocity, time): LC76G on UART1
//! (GPIO24 = TX to the module, GPIO25 = RX from the module, 115200 8N1). The program queries the version
//! (`$PQTMVERNO`), the fix interval (`$PAIR051`) and the `PQTMPVT` rate (`$PQTMCFGMSGRATE,R`), enables
//! `PQTMPVT` once per fix, logs it for 20 s with the RMC time of each epoch, then disables it again and
//! checks that it stopped. It sends nothing else: no fix interval, baud rate, navigation mode,
//! constellation, PPS, restart or save command. The driver is not used; the program frames the raw
//! stream itself so that every `$PAIR` / `$PQTM` reply can be logged in full.
//!
//! Privacy: latitude, longitude, altitude and geoid separation of `PQTMPVT` are replaced by `#` of the
//! same length and the checksum is dropped; of RMC only the time field is logged; GGA and GLL are not
//! logged. A `PQTMPVT` with an unexpected field count is logged with every field from the eighth on
//! masked. A `$PQTM` reply other than VERNO, CFGMSGRATE and EPE is logged as its address only.
#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Duration, Instant, with_timeout};
use embedded_io_async::{Read, Write};
use lc76g::nmea::checksum;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 256;
const CAPTURE_MS: u64 = 20_000;
/// The driver's framer stops at 128 bytes; the first run showed one overflow per epoch with `PQTMPVT`
/// on, so this program frames lines itself up to 255 bytes.
const LINE_MAX: usize = 255;

/// `$` ... `*hh` line assembler with its own checksum test.
struct Framer {
    buf: [u8; LINE_MAX],
    len: usize,
    active: bool,
    checksum_errors: u32,
    overflows: u32,
}

impl Framer {
    const fn new() -> Self {
        Self {
            buf: [0; LINE_MAX],
            len: 0,
            active: false,
            checksum_errors: 0,
            overflows: 0,
        }
    }

    /// Returns the length of a complete, checksum-valid line now held in `buf` (without CR LF).
    fn push(&mut self, b: u8) -> Option<usize> {
        match b {
            b'$' => {
                if self.active {
                    self.checksum_errors += 1;
                }
                self.active = true;
                self.buf[0] = b;
                self.len = 1;
                None
            }
            b'\r' => None,
            b'\n' => {
                if !self.active {
                    return None;
                }
                self.active = false;
                let line = &self.buf[..self.len];
                let ok = line.len() >= 4
                    && line[line.len() - 3] == b'*'
                    && core::str::from_utf8(&line[line.len() - 2..])
                        .ok()
                        .and_then(|h| u8::from_str_radix(h, 16).ok())
                        == Some(checksum(&line[1..line.len() - 3]));
                if ok {
                    Some(self.len)
                } else {
                    self.checksum_errors += 1;
                    None
                }
            }
            _ => {
                if self.active {
                    if self.len == LINE_MAX {
                        self.overflows += 1;
                        self.active = false;
                    } else {
                        self.buf[self.len] = b;
                        self.len += 1;
                    }
                }
                None
            }
        }
    }
}

struct Line {
    buf: [u8; 200],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            buf: [0; 200],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let end = self.len + b.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(b);
        self.len = end;
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Reply {
    None,
    Ok,
    Error,
}

#[derive(Clone, Copy, Default)]
struct Stat {
    n: u32,
    sum: i64,
    sq: i64,
    min: i64,
    max: i64,
}

impl Stat {
    fn add(&mut self, v: i64) {
        if self.n == 0 {
            self.min = v;
            self.max = v;
        }
        self.min = self.min.min(v);
        self.max = self.max.max(v);
        self.n += 1;
        self.sum += v;
        self.sq += v * v;
    }

    fn mean_x10(&self) -> i64 {
        if self.n == 0 { 0 } else { self.sum * 10 / i64::from(self.n) }
    }

    /// Sample standard deviation, times 10.
    fn std_x10(&self) -> u64 {
        let n = i64::from(self.n);
        if n < 2 {
            return 0;
        }
        let var_x100 = (self.sq * n - self.sum * self.sum) * 100 / (n * (n - 1));
        isqrt(var_x100.max(0) as u64)
    }
}

fn isqrt(v: u64) -> u64 {
    let mut r = 0u64;
    while (r + 1) * (r + 1) <= v {
        r += 1;
    }
    r
}

/// "-0.004" -> -4 (mm/s from m/s); None when empty or not a number.
fn milli(s: &str) -> Option<i64> {
    let (neg, s) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let (int, frac) = s.split_once('.').unwrap_or((s, ""));
    if int.is_empty() || !int.bytes().all(|b| b.is_ascii_digit()) || !frac.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let mut v: i64 = int.parse().ok()?;
    let mut digits = frac.bytes();
    for _ in 0..3 {
        v = v * 10 + digits.next().map_or(0, |d| i64::from(d - b'0'));
    }
    Some(if neg { -v } else { v })
}

/// "hhmmss.sss" -> milliseconds of the day.
fn tod_ms(s: &str) -> Option<i64> {
    let h: i64 = s.get(0..2)?.parse().ok()?;
    let m: i64 = s.get(2..4)?.parse().ok()?;
    let sec: i64 = s.get(4..6)?.parse().ok()?;
    let mut frac = 0i64;
    let mut digits = s.get(7..).unwrap_or("").bytes();
    for _ in 0..3 {
        frac = frac * 10 + digits.next().filter(u8::is_ascii_digit).map_or(0, |d| i64::from(d - b'0'));
    }
    Some(((h * 60 + m) * 60 + sec) * 1000 + frac)
}

struct Probe {
    start: Instant,
    capture: bool,
    sentences: u32,
    bytes: u32,
    uart_errors: u32,
    pvt: u32,
    pvt_len: Stat,
    pvt_bad_layout: u32,
    pvt_vel_empty: u32,
    rmc: u32,
    vn: Stat,
    ve: Stat,
    vd: Stat,
    spd: Stat,
    cfg_reply: Reply,
    rmc_tod: Option<i64>,
    rmc_t: u64,
    rmc_paired: bool,
    pvt_tod: Option<i64>,
    pvt_t: u64,
    pvt_paired: bool,
    pvt_prev_t: Option<u64>,
    pvt_period: Stat,
    pvt_minus_rmc: Stat,
}

impl Probe {
    fn new() -> Self {
        Self {
            start: Instant::now(),
            capture: false,
            sentences: 0,
            bytes: 0,
            uart_errors: 0,
            pvt: 0,
            pvt_len: Stat::default(),
            pvt_bad_layout: 0,
            pvt_vel_empty: 0,
            rmc: 0,
            vn: Stat::default(),
            ve: Stat::default(),
            vd: Stat::default(),
            spd: Stat::default(),
            cfg_reply: Reply::None,
            rmc_tod: None,
            rmc_t: 0,
            rmc_paired: true,
            pvt_tod: None,
            pvt_t: 0,
            pvt_paired: true,
            pvt_prev_t: None,
            pvt_period: Stat::default(),
            pvt_minus_rmc: Stat::default(),
        }
    }

    fn us(&self) -> u64 {
        self.start.elapsed().as_micros()
    }

    fn reset_counts(&mut self) {
        let start = self.start;
        *self = Self::new();
        self.start = start;
    }

    fn on_sentence(&mut self, s: &str, t: u64) {
        self.sentences += 1;
        self.bytes += s.len() as u32 + 2;
        let body = s.get(1..).unwrap_or("").split('*').next().unwrap_or("");
        let addr = body.split(',').next().unwrap_or("");
        if addr == "PQTMPVT" {
            self.on_pvt(s, body, t);
        } else if addr.starts_with("PAIR") || matches!(addr, "PQTMVERNO" | "PQTMCFGMSGRATE" | "PQTMEPE") {
            defmt::info!("[INFO] reply t={=u64}us {=str}", t, s);
            if addr == "PQTMCFGMSGRATE" {
                self.cfg_reply = match body.split(',').nth(1) {
                    Some("OK") => Reply::Ok,
                    Some("ERROR") => Reply::Error,
                    _ => self.cfg_reply,
                };
            }
        } else if addr.starts_with("PQTM") {
            defmt::info!("[INFO] other PQTM t={=u64}us {=str} (fields not logged)", t, addr);
        } else if addr.get(2..) == Some("RMC") {
            self.rmc += 1;
            let time = body.split(',').nth(1).unwrap_or("");
            if self.capture {
                defmt::info!("[INFO] RMC t={=u64}us time={=str}", t, time);
            }
            self.rmc_tod = tod_ms(time);
            self.rmc_t = t;
            self.rmc_paired = false;
            if !self.pvt_paired && self.pvt_tod.is_some() && self.pvt_tod == self.rmc_tod {
                self.pair();
            }
        }
    }

    fn pair(&mut self) {
        if self.capture {
            self.pvt_minus_rmc.add(self.pvt_t as i64 - self.rmc_t as i64);
        }
        self.pvt_paired = true;
        self.rmc_paired = true;
    }

    fn on_pvt(&mut self, s: &str, body: &str, t: u64) {
        self.pvt += 1;
        let n = body.split(',').count();
        let mut line = Line::new();
        for (i, f) in body.split(',').enumerate() {
            if i > 0 {
                let _ = line.write_char(',');
            }
            let hide = if n == 20 { matches!(i, 9..=12) } else { i >= 8 };
            for ch in f.chars() {
                let _ = line.write_char(if hide { '#' } else { ch });
            }
        }
        defmt::info!(
            "[INFO] PVT t={=u64}us len={=usize} fields={=usize} {=str}",
            t,
            s.len() + 2,
            n,
            line.as_str()
        );
        if !self.capture {
            return;
        }
        self.pvt_len.add(s.len() as i64 + 2);
        if let Some(prev) = self.pvt_prev_t {
            self.pvt_period.add(t as i64 - prev as i64);
        }
        self.pvt_prev_t = Some(t);
        if n != 20 {
            self.pvt_bad_layout += 1;
            return;
        }
        let f = |k: usize| body.split(',').nth(k).unwrap_or("");
        match (milli(f(13)), milli(f(14)), milli(f(15)), milli(f(16))) {
            (Some(n), Some(e), Some(d), Some(v)) => {
                self.vn.add(n);
                self.ve.add(e);
                self.vd.add(d);
                self.spd.add(v);
            }
            _ => self.pvt_vel_empty += 1,
        }
        self.pvt_tod = tod_ms(f(4));
        self.pvt_t = t;
        self.pvt_paired = false;
        if !self.rmc_paired && self.rmc_tod.is_some() && self.pvt_tod == self.rmc_tod {
            self.pair();
        }
    }

    fn report(&self, framer: &Framer, ms: u64) {
        defmt::info!(
            "[STAT] capture {=u64} ms: {=u32} sentences, {=u32} bytes, PVT {=u32}, RMC {=u32}, PVT bad layout {=u32}, PVT empty velocity {=u32}, uart errors {=u32}, framer checksum errors {=u32} overflows {=u32}",
            ms,
            self.sentences,
            self.bytes,
            self.pvt,
            self.rmc,
            self.pvt_bad_layout,
            self.pvt_vel_empty,
            self.uart_errors,
            framer.checksum_errors,
            framer.overflows
        );
        defmt::info!(
            "[STAT] PVT length bytes incl CRLF: min {=i64} max {=i64} mean_x10 {=i64}",
            self.pvt_len.min,
            self.pvt_len.max,
            self.pvt_len.mean_x10()
        );
        for (name, st) in [("VelN", &self.vn), ("VelE", &self.ve), ("VelD", &self.vd), ("Spd", &self.spd)] {
            defmt::info!(
                "[STAT] {=str} mm/s: n {=u32} mean_x10 {=i64} std_x10 {=u64} min {=i64} max {=i64}",
                name,
                st.n,
                st.mean_x10(),
                st.std_x10(),
                st.min,
                st.max
            );
        }
        defmt::info!(
            "[STAT] PVT period us: n {=u32} min {=i64} mean_x10 {=i64} max {=i64}",
            self.pvt_period.n,
            self.pvt_period.min,
            self.pvt_period.mean_x10(),
            self.pvt_period.max
        );
        defmt::info!(
            "[STAT] PVT arrival minus RMC arrival, same time field, us: pairs {=u32} min {=i64} mean_x10 {=i64} max {=i64}",
            self.pvt_minus_rmc.n,
            self.pvt_minus_rmc.min,
            self.pvt_minus_rmc.mean_x10(),
            self.pvt_minus_rmc.max
        );
    }
}

async fn listen(uart: &mut BufferedUart, framer: &mut Framer, probe: &mut Probe, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    let mut chunk = [0u8; 32];
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, uart.read(&mut chunk)).await {
            Ok(Ok(n)) => {
                let t = probe.us();
                for &b in &chunk[..n] {
                    if let Some(n) = framer.push(b) {
                        let s = core::str::from_utf8(&framer.buf[..n]).unwrap_or("");
                        probe.on_sentence(s, t);
                    }
                }
            }
            Ok(Err(_)) => probe.uart_errors += 1,
            Err(_) => break,
        }
    }
}

async fn send(uart: &mut BufferedUart, probe: &mut Probe, body: &str) {
    let mut line = Line::new();
    let _ = write!(line, "${}*{:02X}\r\n", body, checksum(body.as_bytes()));
    probe.cfg_reply = Reply::None;
    defmt::info!("[INFO] send t={=u64}us {=str}", probe.us(), body);
    let _ = uart.write_all(line.as_str().as_bytes()).await;
    let _ = uart.flush().await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!("[INFO] lc76g_pvt: UART1 TX=GPIO24 RX=GPIO25 115200 8N1; positions are never printed");

    let mut tx_buf = [0u8; TX_BUFFER];
    let mut rx_buf = [0u8; RX_BUFFER];
    let mut uart = BufferedUart::new(
        p.UART1,
        p.PIN_24,
        p.PIN_25,
        Irqs,
        &mut tx_buf,
        &mut rx_buf,
        uart::Config::default(),
    );
    let mut framer = Framer::new();
    let mut probe = Probe::new();

    listen(&mut uart, &mut framer, &mut probe, 2000).await;
    defmt::info!(
        "[STAT] baseline 2000 ms: {=u32} sentences, {=u32} bytes, PVT {=u32}, RMC {=u32}",
        probe.sentences,
        probe.bytes,
        probe.pvt,
        probe.rmc
    );

    send(&mut uart, &mut probe, "PQTMVERNO").await;
    listen(&mut uart, &mut framer, &mut probe, 1500).await;
    send(&mut uart, &mut probe, "PAIR051").await;
    listen(&mut uart, &mut framer, &mut probe, 1500).await;
    send(&mut uart, &mut probe, "PQTMCFGMSGRATE,R,PQTMPVT,1").await;
    listen(&mut uart, &mut framer, &mut probe, 1500).await;
    defmt::info!("[INFO] read reply: {}", probe.cfg_reply);

    send(&mut uart, &mut probe, "PQTMCFGMSGRATE,W,PQTMPVT,1,1").await;
    listen(&mut uart, &mut framer, &mut probe, 1000).await;
    let enable = probe.cfg_reply;
    if enable != Reply::Ok {
        failures += 1;
        defmt::error!("[FAIL] enable answered {} (see the reply lines above); probe stops here", enable);
        probe.reset_counts();
        listen(&mut uart, &mut framer, &mut probe, 3000).await;
        if probe.pvt > 0 {
            send(&mut uart, &mut probe, "PQTMCFGMSGRATE,W,PQTMPVT,0,1").await;
            listen(&mut uart, &mut framer, &mut probe, 1000).await;
        }
        hil_common::exit(failures);
    }
    defmt::info!("[PASS] enable answered OK");

    probe.reset_counts();
    probe.capture = true;
    listen(&mut uart, &mut framer, &mut probe, CAPTURE_MS).await;
    probe.capture = false;
    probe.report(&framer, CAPTURE_MS);
    if probe.pvt == 0 {
        failures += 1;
        defmt::error!("[FAIL] no PQTMPVT in {=u64} ms after OK", CAPTURE_MS);
    }

    let mut off = Reply::None;
    for _ in 0..2 {
        send(&mut uart, &mut probe, "PQTMCFGMSGRATE,W,PQTMPVT,0,1").await;
        listen(&mut uart, &mut framer, &mut probe, 1000).await;
        off = probe.cfg_reply;
        if off == Reply::Ok {
            break;
        }
    }
    if off == Reply::Ok {
        defmt::info!("[PASS] disable answered OK");
    } else {
        failures += 1;
        defmt::error!("[FAIL] disable answered {}", off);
    }
    probe.reset_counts();
    listen(&mut uart, &mut framer, &mut probe, 3000).await;
    if probe.pvt == 0 {
        defmt::info!("[PASS] no PQTMPVT in 3000 ms after disable ({=u32} sentences)", probe.sentences);
    } else {
        failures += 1;
        defmt::error!("[FAIL] {=u32} PQTMPVT in 3000 ms after disable", probe.pvt);
    }
    send(&mut uart, &mut probe, "PQTMCFGMSGRATE,R,PQTMPVT,1").await;
    listen(&mut uart, &mut framer, &mut probe, 1500).await;
    send(&mut uart, &mut probe, "PAIR051").await;
    listen(&mut uart, &mut framer, &mut probe, 1500).await;

    hil_common::exit(failures)
}
