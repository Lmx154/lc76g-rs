//! On-board check of the lc76g driver: LC76G on UART1 (GPIO24 = TX to the module, GPIO25 = RX from the
//! module, 115200 8N1), PPS on GPIO5. The module is not power-cycled between runs, so the test starts
//! from whatever state the previous run left, and always ends with `init(&Config::default())`.
//! It never sends a flash write, a baud change, a restart or a low-power command.
#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::hint::black_box;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::interrupt;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Duration, Instant, with_timeout};
use lc76g::sentence::FixType;
use lc76g::{Command, Config, Error, Fix, Framer, Lc76g, Message, NavMode, SentenceType};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type Gnss = Lc76g<BufferedUart>;
type GnssError = Error<uart::Error>;

/// UART receive buffer. The module sends a burst of about 20 sentences (1.4 kB) at the top of each
/// second; 4096 bytes hold almost three of them, 355 ms of line at 115200 baud, so the test can print
/// over RTT between reads without the UART FIFO overrunning.
const RX_BUFFER: usize = 4096;
/// Longest command is 130 bytes; one fits with room to spare.
const TX_BUFFER: usize = 256;

/// Seconds of the 10 Hz stream check.
const STREAM_SECS: u64 = 10;

/// Rising edges of PPS: count, and the low 32 bits of the microsecond timestamp of the last 16.
static PPS_N: AtomicU32 = AtomicU32::new(0);
static PPS_RING: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];
/// UART errors returned by any read since the start.
static UART_ERRORS: AtomicU32 = AtomicU32::new(0);
/// Value of `UART_ERRORS` when the startup burst of errors had passed.
static STARTUP_ERRORS: AtomicU32 = AtomicU32::new(0);

/// The PPS task runs on an interrupt executor: the driver parses a whole burst without yielding, which
/// holds the thread-mode executor for milliseconds and would delay the edge timestamps. 0xE0 is below
/// the GPIO bank interrupt (embassy-rp runs it at 0xC0) and above thread mode.
const PPS_PRIORITY: u8 = 0xE0;
static PPS_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { PPS_EXECUTOR.on_interrupt() }
}

#[embassy_executor::task]
async fn count_pps(mut pin: Input<'static>) {
    loop {
        pin.wait_for_rising_edge().await;
        let ts = Instant::now().as_micros() as u32;
        let n = PPS_N.load(Relaxed);
        PPS_RING[(n % 16) as usize].store(ts, Relaxed);
        PPS_N.store(n + 1, Relaxed);
    }
}

/// Edge count and the timestamp of the latest edge (retries if an edge lands between the loads).
fn edge_now() -> (u32, u32) {
    loop {
        let n = PPS_N.load(Relaxed);
        if n == 0 {
            return (0, 0);
        }
        let t = PPS_RING[((n - 1) % 16) as usize].load(Relaxed);
        if PPS_N.load(Relaxed) == n {
            return (n, t);
        }
    }
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

/// Square root by Newton iteration (the driver and this test use no libm).
fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut g = f32::from_bits((x.to_bits() >> 1) + 0x1FBD_1DF5);
    for _ in 0..4 {
        g = 0.5 * (g + x / g);
    }
    g
}

/// Cosine by Taylor series, good to 1e-4 below 80 degrees.
fn cos(x: f32) -> f32 {
    let x2 = x * x;
    1.0 - x2 / 2.0 + x2 * x2 / 24.0 - x2 * x2 * x2 / 720.0 + x2 * x2 * x2 * x2 / 40320.0
}

/// Diagonal in metres of the bounding box of positions in 1e-7 degree.
fn spread_m(lat: (i32, i32), lon: (i32, i32)) -> f32 {
    const M_PER_DEG: f32 = 111_195.0;
    let mid_rad =
        ((lat.0 as i64 + lat.1 as i64) / 2) as f32 * 1e-7 * (core::f32::consts::PI / 180.0);
    let dy = (lat.1 - lat.0) as f32 * 1e-7 * M_PER_DEG;
    let dx = (lon.1 - lon.0) as f32 * 1e-7 * M_PER_DEG * cos(mid_rad);
    sqrt(dx * dx + dy * dy)
}

fn count_err(e: &GnssError) {
    if let Error::Uart(_) = e {
        UART_ERRORS.fetch_add(1, Relaxed);
    }
}

/// Milliseconds from fix time `prev` to `t`, both milliseconds of the UTC day.
fn step_ms(prev: u32, t: u32) -> u32 {
    (t + 86_400_000 - prev) % 86_400_000
}

fn fix_ms(fix: &Fix) -> Option<u32> {
    fix.time.map(|t| t.millis_of_day())
}

/// Sentence types seen in a window, merged over talkers: `GGA` for GNGGA and GPGGA, the full address
/// for proprietary sentences.
struct Census {
    keys: [[u8; 8]; 16],
    lens: [u8; 16],
    counts: [u16; 16],
    n: usize,
}

impl Census {
    const fn new() -> Self {
        Self {
            keys: [[0; 8]; 16],
            lens: [0; 16],
            counts: [0; 16],
            n: 0,
        }
    }

    fn add(&mut self, address: &str) {
        let key = if address.len() == 5 && address.starts_with('G') {
            &address[2..]
        } else {
            &address[..address.len().min(8)]
        };
        for i in 0..self.n {
            if &self.keys[i][..self.lens[i] as usize] == key.as_bytes() {
                self.counts[i] += 1;
                return;
            }
        }
        if self.n < 16 {
            self.keys[self.n][..key.len()].copy_from_slice(key.as_bytes());
            self.lens[self.n] = key.len() as u8;
            self.counts[self.n] = 1;
            self.n += 1;
        }
    }

    fn count(&self, key: &str) -> u16 {
        (0..self.n)
            .find(|&i| &self.keys[i][..self.lens[i] as usize] == key.as_bytes())
            .map_or(0, |i| self.counts[i])
    }

    fn describe(&self, out: &mut Line) {
        for i in 0..self.n {
            let key = core::str::from_utf8(&self.keys[i][..self.lens[i] as usize]).unwrap_or("?");
            let _ = write!(out, "{}={} ", key, self.counts[i]);
        }
    }
}

/// Counts the sentence types arriving for `ms` milliseconds.
async fn census(gnss: &mut Gnss, ms: u64) -> Census {
    let mut c = Census::new();
    let end = Instant::now() + Duration::from_millis(ms);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_sentence()).await {
            Ok(Ok(s)) => c.add(s.address()),
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    c
}

/// Sends a get command through `Command::Raw` and returns the reply sentence that follows the
/// acknowledgement. `Err` carries what went wrong.
async fn query(gnss: &mut Gnss, body: &str) -> Result<Line, &'static str> {
    let mut want = Line::new();
    let _ = write!(want, "${body},");
    match with_timeout(Duration::from_secs(3), gnss.command(&Command::Raw(body))).await {
        Ok(Ok(())) => {}
        Ok(Err(e)) => {
            count_err(&e);
            defmt::error!("[INFO] query {=str}: {}", body, e);
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

/// Brings the module to 1 Hz from any state. A module left at 10 Hz with GSV on is saturating the
/// 115200 baud link: it then answers commands late or not at all, so each command is retried. GSV is
/// switched off first because it is most of the traffic. Returns the attempts used by both commands,
/// 0 if one of them never succeeded.
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

/// After a failed `init`: sends the commands of `cfg` one by one and reports each result and time, to
/// name the command that was not acknowledged.
async fn replay(gnss: &mut Gnss, cfg: &Config) {
    for sentence in SentenceType::ALL {
        let rate = cfg.rates.get(sentence);
        replay_one(gnss, &Command::SetSentenceRate { sentence, rate }).await;
    }
    replay_one(gnss, &Command::SetEpe(cfg.epe)).await;
    replay_one(gnss, &Command::SetNavMode(cfg.nav_mode)).await;
    replay_one(
        gnss,
        &Command::SetFixInterval {
            ms: cfg.fix_interval_ms,
        },
    )
    .await;
}

async fn replay_one(gnss: &mut Gnss, command: &Command<'_>) {
    let t0 = Instant::now();
    let r = with_timeout(Duration::from_secs(8), gnss.command(command)).await;
    let ms = t0.elapsed().as_millis() as u32;
    match r {
        Ok(Ok(())) => defmt::info!("[INFO] replay {}: acknowledged in {=u32} ms", command, ms),
        Ok(Err(e)) => defmt::info!("[INFO] replay {}: {} after {=u32} ms", command, e, ms),
        Err(_) => defmt::info!("[INFO] replay {}: no completion in {=u32} ms", command, ms),
    }
}

/// Fix-time steps after a return to 1000 ms.
struct Steps {
    fixes: u32,
    /// Steps other than 1000 ms seen before the stream settled.
    transition: u32,
    /// Consecutive 1000 ms steps at the end.
    settled: u32,
    first: [u32; 6],
}

/// Reads fixes until `need` consecutive steps of 1000 ms, at most 30 fixes.
async fn settle_1hz(gnss: &mut Gnss, need: u32) -> Steps {
    let mut s = Steps {
        fixes: 0,
        transition: 0,
        settled: 0,
        first: [0; 6],
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
        let t = fix_ms(&fix);
        if let (Some(p), Some(t)) = (prev, t) {
            let step = step_ms(p, t);
            if (s.fixes as usize - 1) < s.first.len() {
                s.first[s.fixes as usize - 1] = step;
            }
            if step == 1000 {
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

/// Results of the 10 Hz stream window.
struct Window {
    n: u32,
    valid: u32,
    exact: u32,
    other: u32,
    span: u32,
    min_step: u32,
    jitter: u32,
    max_step: u32,
    odd: [u32; 6],
    bad_year: u32,
    few_sats: u32,
    no_hdop: u32,
    no_pos: u32,
    not_3d: u32,
    min_sats: u8,
    lat: (i32, i32),
    lon: (i32, i32),
    sample: Option<Fix>,
    delay0: (u32, u32, u32),
    delay_by_tenth: [i32; 10],
    pps_first: u32,
    pps_last: u32,
    elapsed_ms: u32,
}

/// Reads fixes for `secs` seconds without printing, so that nothing slows the loop down.
async fn stream(gnss: &mut Gnss, secs: u64) -> Window {
    let mut w = Window {
        n: 0,
        valid: 0,
        exact: 0,
        other: 0,
        span: 0,
        min_step: u32::MAX,
        jitter: 0,
        max_step: 0,
        odd: [0; 6],
        bad_year: 0,
        few_sats: 0,
        no_hdop: 0,
        no_pos: 0,
        not_3d: 0,
        min_sats: u8::MAX,
        lat: (i32::MAX, i32::MIN),
        lon: (i32::MAX, i32::MIN),
        sample: None,
        delay0: (u32::MAX, 0, 0),
        delay_by_tenth: [-1; 10],
        pps_first: PPS_N.load(Relaxed),
        pps_last: 0,
        elapsed_ms: 0,
    };
    let start = Instant::now();
    let end = start + Duration::from_secs(secs);
    let mut prev: Option<u32> = None;
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        let fix = match with_timeout(end - now, gnss.read_fix()).await {
            Ok(Ok(fix)) => fix,
            Ok(Err(e)) => {
                count_err(&e);
                continue;
            }
            Err(_) => break,
        };
        let returned = Instant::now().as_micros() as u32;
        w.n += 1;
        let t = fix_ms(&fix);
        match (prev, t) {
            (Some(p), Some(t)) => {
                let step = step_ms(p, t);
                w.max_step = w.max_step.max(step);
                if step == 100 {
                    w.exact += 1;
                } else if (50..150).contains(&step) {
                    if let Some(slot) = w.odd.get_mut(w.jitter as usize) {
                        *slot = step;
                    }
                    w.jitter += 1;
                } else {
                    w.other += 1;
                }
                w.span += step;
                w.min_step = w.min_step.min(step);
            }
            (Some(_), None) => w.other += 1,
            _ => {}
        }
        prev = t;
        if let Some(t) = t {
            let (edges, last) = edge_now();
            if edges != w.pps_first {
                let delay = returned.wrapping_sub(last);
                w.delay_by_tenth[(t % 1000 / 100) as usize] = (delay / 1000) as i32;
                if t % 1000 < 100 {
                    w.delay0 = (w.delay0.0.min(delay), w.delay0.1.max(delay), w.delay0.2 + 1);
                }
            }
        }
        if fix.valid {
            w.valid += 1;
            w.sample = Some(fix);
            w.bad_year += fix.date.is_none_or(|d| d.year < 2026) as u32;
            let sats = fix.sats_used.unwrap_or(0);
            w.min_sats = w.min_sats.min(sats);
            w.few_sats += (sats < 4) as u32;
            w.no_hdop += fix.hdop_x100.is_none() as u32;
            w.not_3d += (fix.fix_type != Some(FixType::Fix3d)) as u32;
            match (fix.lat_e7, fix.lon_e7) {
                (Some(la), Some(lo)) if la != 0 && lo != 0 => {
                    w.lat = (w.lat.0.min(la), w.lat.1.max(la));
                    w.lon = (w.lon.0.min(lo), w.lon.1.max(lo));
                }
                _ => w.no_pos += 1,
            }
        }
    }
    w.pps_last = PPS_N.load(Relaxed);
    w.elapsed_ms = start.elapsed().as_millis() as u32;
    w
}

/// Frames `$body*CS\r\n` into `out` and returns its length.
fn frame(body: &str, out: &mut [u8; 128]) -> usize {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let b = body.as_bytes();
    out[0] = b'$';
    out[1..1 + b.len()].copy_from_slice(b);
    let n = 1 + b.len();
    let sum = lc76g::nmea::checksum(b);
    out[n] = b'*';
    out[n + 1] = HEX[(sum >> 4) as usize];
    out[n + 2] = HEX[(sum & 15) as usize];
    out[n + 3] = b'\r';
    out[n + 4] = b'\n';
    n + 5
}

/// CPU cost of the sans-IO core: nanoseconds to frame one sentence, and to frame and parse it.
fn bench(body: &str, want_gga: bool) -> (u32, u32, usize, bool) {
    const REPS: u32 = 300;
    let mut buf = [0u8; 128];
    let len = frame(body, &mut buf);
    let bytes = black_box(&buf[..len]);
    let mut framer = Framer::new();

    let t0 = Instant::now();
    for _ in 0..REPS {
        for &b in bytes {
            black_box(framer.push(b).is_some());
        }
    }
    let frame_ns = (t0.elapsed().as_micros() * 1000 / REPS as u64) as u32;

    let t0 = Instant::now();
    for _ in 0..REPS {
        for &b in bytes {
            if let Some(s) = framer.push(b) {
                black_box(s.parse());
            }
        }
    }
    let both_ns = (t0.elapsed().as_micros() * 1000 / REPS as u64) as u32;

    let mut decoded = false;
    for &b in bytes {
        if let Some(s) = framer.push(b) {
            decoded = match s.parse() {
                Message::Gga(g) => want_gga && g.lat_e7.is_some(),
                Message::Rmc(r) => !want_gga && r.lat_e7.is_some(),
                _ => false,
            };
        }
    }
    (frame_ns, both_ns, len, decoded)
}

fn micros(ns: u32, out: &mut Line) {
    let _ = write!(out, "{}.{:02}", ns / 1000, ns % 1000 / 10);
}

/// Everything from S1 to S8. Returns the driver so that the caller can restore the module.
async fn run(mut gnss: Gnss, failures: &mut u32) -> Gnss {
    // The reset by the probe leaves a burst of framing errors on the receive line at the start of every
    // run; read through it so that S1 does not spend its attempts on them.
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
    let startup_errors = UART_ERRORS.load(Relaxed);
    STARTUP_ERRORS.store(startup_errors, Relaxed);
    defmt::info!(
        "[INFO] UART errors in the first 500 ms after the start of the program: {=u32}",
        startup_errors
    );

    // S1: link.
    let mut version = None;
    for attempt in 1..=5u32 {
        match with_timeout(Duration::from_secs(4), gnss.version()).await {
            Ok(Ok(v)) => {
                version = Some((v, attempt));
                break;
            }
            Ok(Err(e)) => {
                count_err(&e);
                defmt::info!("[INFO] S1 version attempt {=u32}: {}", attempt, e);
            }
            Err(_) => defmt::info!(
                "[INFO] S1 version attempt {=u32}: no answer in 4 s",
                attempt
            ),
        }
    }
    check!(
        *failures,
        version.is_some(),
        "{=str} S1 link: PQTMVERNO answered"
    );
    let Some((version, attempt)) = version else {
        return gnss;
    };
    defmt::info!(
        "[INFO] S1 firmware {=str}, build date {}, build time {}, answered on attempt {=u32}",
        version.version(),
        version.build_date,
        version.build_time,
        attempt
    );

    // The module keeps its state between runs: start from 1 Hz, whatever the last run left.
    defmt::info!(
        "[INFO] S1 GSV off + SetFixInterval(1000) from the starting state took {=u32} attempts in total (2 = first try, 0 = failed)",
        back_to_1hz(&mut gnss).await
    );

    // S2: configure for 10 Hz, drone mode, EPE on.
    let cfg = Config {
        nav_mode: NavMode::Drone,
        epe: true,
        ..Config::with_fix_interval(100)
    };
    let flooded = Config {
        fix_interval_ms: 100,
        ..Config::default()
    };
    let r = gnss.init(&flooded).await;
    check!(
        *failures,
        matches!(r, Err(Error::InvalidConfig)),
        "{=str} S2 init(100 ms, default rates) returns InvalidConfig and sends nothing"
    );
    let t0 = Instant::now();
    let r = with_timeout(Duration::from_secs(15), gnss.init(&cfg)).await;
    let init_ms = t0.elapsed().as_millis() as u32;
    let ok = matches!(r, Ok(Ok(_)));
    check!(
        *failures,
        ok,
        "{=str} S2 init (with_fix_interval(100), drone, EPE on) acknowledged, took {=u32} ms",
        init_ms
    );
    match r {
        Ok(Ok(_)) => {}
        Ok(Err(Error::Rejected { command, code })) => {
            defmt::error!(
                "[INFO] S2 rejected: command {} result code {=u8}",
                command,
                code
            );
        }
        Ok(Err(e)) => {
            count_err(&e);
            defmt::error!("[INFO] S2 init error: {}", e);
        }
        Err(_) => defmt::error!("[INFO] S2 init: no completion within 15 s"),
    }
    if !ok {
        replay(&mut gnss, &cfg).await;
        return gnss;
    }

    // Let the stream settle at 10 Hz: leftover 1 Hz data may still be queued.
    let mut settle = 0u32;
    {
        let mut prev = None;
        let mut run_ok = 0;
        while settle < 40 && run_ok < 3 {
            let Ok(Ok(fix)) = with_timeout(Duration::from_secs(3), gnss.read_fix()).await else {
                break;
            };
            let t = fix_ms(&fix);
            match (prev, t) {
                (Some(p), Some(t)) if step_ms(p, t) == 100 => run_ok += 1,
                _ => run_ok = 0,
            }
            prev = t;
            settle += 1;
        }
    }
    defmt::info!(
        "[INFO] S3 settle: {=u32} fixes read before three consecutive 100 ms steps",
        settle
    );

    // S3: stream. The first seconds after switching to drone mode are not measured.
    let _ = stream(&mut gnss, 5).await;
    let stats0 = gnss.stats();
    let uart0 = UART_ERRORS.load(Relaxed);
    let w = stream(&mut gnss, STREAM_SECS).await;
    let stats1 = gnss.stats();
    let uart_errors = UART_ERRORS.load(Relaxed) - uart0;
    let rate_x100 = if w.elapsed_ms > 0 {
        w.n as u64 * 100_000 / w.elapsed_ms as u64
    } else {
        0
    };
    check!(
        *failures,
        (98..=102).contains(&w.n),
        "{=str} S3 fixes in {=u32} ms: {=u32} (expect 100 +-2)",
        w.elapsed_ms,
        w.n
    );
    let epochs_missed = (w.span + 50) as i32 / 100 - w.n.saturating_sub(1) as i32;
    check!(
        *failures,
        epochs_missed.abs() <= 1,
        "{=str} S3 missed epochs: {=i32} (fix time span {=u32} ms / 100 ms, rounded, minus the {=u32} steps read; the fix times jitter, so +-1 is the resolution)",
        epochs_missed,
        w.span,
        w.n.saturating_sub(1)
    );
    defmt::info!(
        "[STAT] S3 fix time steps: {=u32} of {=u32} exactly 100 ms, {=u32} within 50..150 ms (first {} ms), {=u32} outside 50..150 ms; smallest {=u32} ms, largest {=u32} ms",
        w.exact,
        w.n.saturating_sub(1),
        w.jitter,
        w.odd,
        w.other,
        w.min_step,
        w.max_step
    );
    check!(
        *failures,
        stats1.checksum_errors == stats0.checksum_errors
            && stats1.overflows == stats0.overflows
            && stats1.epochs_dropped == stats0.epochs_dropped,
        "{=str} S3 driver counters over the window: checksum errors {=u32}, overflows {=u32}, epochs dropped {=u32}",
        stats1.checksum_errors - stats0.checksum_errors,
        stats1.overflows - stats0.overflows,
        stats1.epochs_dropped - stats0.epochs_dropped
    );
    check!(
        *failures,
        uart_errors == 0,
        "{=str} S3 UART errors over the window: {=u32}",
        uart_errors
    );
    defmt::info!(
        "[STAT] S3 achieved fix rate {=u32}.{=u32:02} Hz ({=u32} fixes in {=u32} ms), sentences {=u32}, fixes discarded {=u32}",
        (rate_x100 / 100) as u32,
        (rate_x100 % 100) as u32,
        w.n,
        w.elapsed_ms,
        stats1.sentences - stats0.sentences,
        stats1.fixes_discarded - stats0.fixes_discarded
    );
    defmt::info!("[INFO] S3 driver stats since start: {}", stats1);

    let c = census(&mut gnss, 2000).await;
    let mut line = Line::new();
    c.describe(&mut line);
    defmt::info!(
        "[INFO] S3 sentence types in 2.0 s at 10 Hz (GSA/GSV every 10th fix): {=str}",
        line.as_str()
    );

    // S4: content.
    let has_fix = w.valid > 0;
    if !has_fix {
        defmt::info!(
            "[SKIP] S4 content: no valid fix in {=u32} fixes (RMC status V), no antenna sky view or no fix yet",
            w.n
        );
    } else {
        defmt::info!("[INFO] S4 valid fixes: {=u32} of {=u32}", w.valid, w.n);
        check!(
            *failures,
            w.bad_year == 0,
            "{=str} S4 year >= 2026 in every valid fix (violations {=u32})",
            w.bad_year
        );
        check!(
            *failures,
            w.few_sats == 0,
            "{=str} S4 satellites used >= 4 (smallest {=u8}, violations {=u32})",
            w.min_sats,
            w.few_sats
        );
        check!(
            *failures,
            w.no_hdop == 0,
            "{=str} S4 HDOP present (missing in {=u32} fixes)",
            w.no_hdop
        );
        check!(
            *failures,
            w.no_pos == 0 && w.lat.0 <= w.lat.1,
            "{=str} S4 latitude and longitude present and non-zero (violations {=u32})",
            w.no_pos
        );
        let spread = if w.lat.0 <= w.lat.1 {
            spread_m(w.lat, w.lon)
        } else {
            f32::MAX
        };
        defmt::info!(
            "[INFO] S4 position range over the window: latitude {=i32} .. {=i32}, longitude {=i32} .. {=i32} (1e-7 degree)",
            w.lat.0,
            w.lat.1,
            w.lon.0,
            w.lon.1
        );
        defmt::info!("[STAT] S4 position spread over the window {=f32} m", spread);
        check!(
            *failures,
            spread < 50.0,
            "{=str} S4 position spread over the window {=f32} m (expect < 50 m)",
            spread
        );
        check!(
            *failures,
            w.not_3d == 0,
            "{=str} S4 GSA fix type 3D in every valid fix (violations {=u32})",
            w.not_3d
        );
        if let Some(fix) = w.sample {
            defmt::info!("[INFO] S4 last fix: {}", fix);
        }
        let mut got_gga = false;
        let mut got_epe = false;
        let end = Instant::now() + Duration::from_millis(2500);
        while !(got_gga && got_epe) {
            let now = Instant::now();
            if now >= end {
                break;
            }
            match with_timeout(end - now, gnss.read_sentence()).await {
                Ok(Ok(s)) => {
                    if !got_gga && s.address().ends_with("GGA") {
                        got_gga = true;
                        defmt::info!("[INFO] S4 raw GGA: {=str}", s.as_str());
                    }
                    if !got_epe && let Message::Epe(epe) = s.parse() {
                        got_epe = true;
                        defmt::info!("[INFO] S4 raw EPE: {=str}", s.as_str());
                        defmt::info!("[INFO] S4 EPE decoded: {}", epe);
                    }
                }
                Ok(Err(e)) => count_err(&e),
                Err(_) => break,
            }
        }
        if !got_gga {
            defmt::info!("[INFO] S4 no GGA seen in 2.5 s");
        }
        if !got_epe {
            defmt::info!("[INFO] S4 no PQTMEPE seen in 2.5 s");
        }
    }

    // S5: cost.
    let gga = "GNGGA,040143.000,3149.3341660,N,11706.9416700,E,1,24,0.48,61.496,M,-0.335,M,,";
    let rmc = "GNRMC,040143.000,A,3149.3341660,N,11706.9416700,E,0.01,0.00,010522,,,A,V";
    let t0 = Instant::now();
    let mut acc = 1u32;
    for i in 0..200_000u32 {
        acc = black_box(acc.wrapping_mul(3).wrapping_add(i));
    }
    black_box(acc);
    defmt::info!(
        "[STAT] S5 calibration: one mul+add loop iteration takes {=u64} ns",
        t0.elapsed().as_micros() * 1000 / 200_000
    );
    let (gga_frame, gga_both, gga_len, gga_ok) = bench(gga, true);
    let (rmc_frame, rmc_both, rmc_len, rmc_ok) = bench(rmc, false);
    check!(
        *failures,
        gga_ok && rmc_ok,
        "{=str} S5 benchmark sentences decode to GGA and RMC with a position"
    );
    let mut line = Line::new();
    let _ = write!(line, "GGA ({} bytes): framing ", gga_len);
    micros(gga_frame, &mut line);
    let _ = write!(line, " us, framing + parse ");
    micros(gga_both, &mut line);
    let _ = write!(line, " us; RMC ({} bytes): framing ", rmc_len);
    micros(rmc_frame, &mut line);
    let _ = write!(line, " us, framing + parse ");
    micros(rmc_both, &mut line);
    let _ = write!(line, " us");
    defmt::info!(
        "[STAT] S5 cost per sentence at {=u32} MHz: {=str}",
        embassy_rp::clocks::clk_sys_freq() / 1_000_000,
        line.as_str()
    );
    defmt::info!(
        "[STAT] S5 size_of Lc76g<BufferedUart> = {=usize} bytes (UART buffers are separate: RX {=usize}, TX {=usize})",
        core::mem::size_of::<Gnss>(),
        RX_BUFFER,
        TX_BUFFER
    );

    defmt::info!(
        "[INFO] UART errors so far: {=u32}",
        UART_ERRORS.load(Relaxed)
    );

    // S6: PPS.
    let edges = w.pps_last - w.pps_first;
    if !has_fix {
        defmt::info!(
            "[SKIP] S6 PPS: no valid fix, the module sends PPS only with a fix ({=u32} edges seen anyway)",
            edges
        );
    } else {
        let first = w.pps_last.saturating_sub(16).max(w.pps_first);
        let mut worst_ppm = 0u32;
        let mut periods = Line::new();
        for i in first + 1..w.pps_last {
            let a = PPS_RING[((i - 1) % 16) as usize].load(Relaxed);
            let b = PPS_RING[(i % 16) as usize].load(Relaxed);
            let period = b.wrapping_sub(a);
            worst_ppm = worst_ppm.max((period as i64 - 1_000_000).unsigned_abs() as u32);
            let _ = write!(periods, "{} ", period);
        }
        let n_periods = (w.pps_last - first).saturating_sub(1) as usize;
        let mut sorted = [0u32; 16];
        for (n, i) in (first + 1..w.pps_last).enumerate() {
            let a = PPS_RING[((i - 1) % 16) as usize].load(Relaxed);
            let b = PPS_RING[(i % 16) as usize].load(Relaxed);
            sorted[n] = b.wrapping_sub(a);
        }
        sorted[..n_periods].sort_unstable();
        let median_ppm = if n_periods > 0 {
            (sorted[n_periods / 2] as i64 - 1_000_000).unsigned_abs() as u32
        } else {
            u32::MAX
        };
        check!(
            *failures,
            edges >= 3 && n_periods >= 2 && median_ppm <= 500,
            "{=str} S6 PPS on GPIO5: {=u32} rising edges in {=u32} ms, median period error {=u32} ppm of 1 s by the MCU timer (expect <= 500)",
            edges,
            w.elapsed_ms,
            median_ppm
        );
        defmt::info!(
            "[STAT] S6 periods in us: {=str}; worst single period error {=u32} ppm",
            periods.as_str(),
            worst_ppm
        );
        if w.delay0.2 > 0 {
            defmt::info!(
                "[INFO] S6 delay from PPS rising edge to read_fix returning the fix of that second (time hh:mm:ss.000): min {=u32} us, max {=u32} us over {=u32} seconds",
                w.delay0.0,
                w.delay0.1,
                w.delay0.2
            );
        } else {
            defmt::info!("[INFO] S6 no fix at .000 seen after a PPS edge");
        }
        let mut line = Line::new();
        for (i, d) in w.delay_by_tenth.iter().enumerate() {
            let _ = write!(line, "{}={} ", i * 100, d);
        }
        defmt::info!(
            "[INFO] S6 delay in ms from the latest PPS edge to read_fix returning, by fix time .000 .. .900 ms (last second): {=str}",
            line.as_str()
        );
    }

    // S7: blocking front-end on the same UART.
    {
        let mut blocking = lc76g::blocking::Lc76g::new(gnss.release());
        let before = blocking.stats();
        let fix = blocking.read_fix();
        let msg = blocking.read();
        let ver = blocking.version();
        let after = blocking.stats();
        gnss = Lc76g::new(blocking.release());
        let decoded = match (&fix, &msg, &ver) {
            (Ok(f), Ok(m), Ok(_)) => {
                !matches!(m, Message::Malformed)
                    && f.time.is_some()
                    && (!f.valid || f.date.is_some())
            }
            _ => false,
        };
        check!(
            *failures,
            decoded && after.checksum_errors == before.checksum_errors,
            "{=str} S7 blocking front-end decoded one fix, one message and the version, checksum errors {=u32}",
            after.checksum_errors - before.checksum_errors
        );
        match (&fix, &msg, &ver) {
            (Ok(f), Ok(m), Ok(v)) => defmt::info!(
                "[INFO] S7 blocking: fix valid {=bool} time {}, message {}, version {=str}",
                f.valid,
                f.time,
                m,
                v.version()
            ),
            _ => defmt::error!(
                "[INFO] S7 blocking: fix {}, message {}, version {}",
                fix.err(),
                msg.err(),
                ver.err()
            ),
        }
    }

    defmt::info!(
        "[INFO] UART errors so far: {=u32}",
        UART_ERRORS.load(Relaxed)
    );

    // S8c (10 Hz, drone state): read the settings back.
    for body in ["PAIR051", "PAIR081", "PAIR063,0", "PAIR063,6"] {
        match query(&mut gnss, body).await {
            Ok(line) => defmt::info!(
                "[INFO] S8c in 10 Hz drone state ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => defmt::info!(
                "[INFO] S8c in 10 Hz drone state ${=str} -> {=str}",
                body,
                why
            ),
        }
    }

    // S8a: init(Config::default()) and the sentence set.
    let r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    let default_ok = matches!(r, Ok(Ok(_)));
    check!(
        *failures,
        default_ok,
        "{=str} S8a init(&Config::default()) acknowledged: every rate command incl. ZDA/GRS/GST/GNS = 0, EPE off, normal mode, 1000 ms"
    );
    match r {
        Ok(Err(Error::Rejected { command, code })) => defmt::error!(
            "[INFO] S8a rejected: command {} result code {=u8}",
            command,
            code
        ),
        Ok(Err(e)) => defmt::error!("[INFO] S8a init error: {}", e),
        Err(_) => defmt::error!("[INFO] S8a init: no completion within 15 s"),
        Ok(Ok(_)) => {}
    }
    let steps = settle_1hz(&mut gnss, 3).await;
    defmt::info!(
        "[INFO] S8d after init(default): {=u32} fixes read, {=u32} steps not 1000 ms before settling, first steps {} ms, then {=u32} consecutive 1000 ms",
        steps.fixes,
        steps.transition,
        steps.first,
        steps.settled
    );
    let c = census(&mut gnss, 2200).await;
    let mut line = Line::new();
    c.describe(&mut line);
    defmt::info!(
        "[INFO] S8a sentence types in 2.2 s after init(default): {=str}",
        line.as_str()
    );

    defmt::info!(
        "[INFO] UART errors so far: {=u32}",
        UART_ERRORS.load(Relaxed)
    );

    // S8b: reset sentence rates.
    // Make the reset visible: switch ZDA on first, then see it go.
    let r = with_timeout(
        Duration::from_secs(5),
        gnss.command(&Command::SetSentenceRate {
            sentence: SentenceType::Zda,
            rate: 1,
        }),
    )
    .await;
    check!(
        *failures,
        matches!(r, Ok(Ok(()))),
        "{=str} S8b SetSentenceRate ZDA = 1 acknowledged, before the reset"
    );
    match query(&mut gnss, "PAIR063,6").await {
        Ok(line) => defmt::info!(
            "[INFO] S8b before reset $PAIR063,6 -> {=str}",
            line.as_str()
        ),
        Err(why) => defmt::info!("[INFO] S8b before reset $PAIR063,6 -> {=str}", why),
    }
    let r = with_timeout(
        Duration::from_secs(5),
        gnss.command(&Command::ResetSentenceRates),
    )
    .await;
    check!(
        *failures,
        matches!(r, Ok(Ok(()))),
        "{=str} S8b ResetSentenceRates ($PAIR062,-1) acknowledged"
    );
    match r {
        Ok(Err(Error::Rejected { command, code })) => defmt::info!(
            "[INFO] S8b ResetSentenceRates rejected: command {} result code {=u8}",
            command,
            code
        ),
        Ok(Err(e)) => defmt::info!("[INFO] S8b ResetSentenceRates error: {}", e),
        Err(_) => defmt::info!("[INFO] S8b ResetSentenceRates: no acknowledgement within 5 s"),
        Ok(Ok(())) => {}
    }
    let c = census(&mut gnss, 2200).await;
    let mut line = Line::new();
    c.describe(&mut line);
    defmt::info!(
        "[INFO] S8b sentence types in 2.2 s after ResetSentenceRates: {=str}",
        line.as_str()
    );
    for body in [
        "PAIR051",
        "PAIR081",
        "PAIR063,0",
        "PAIR063,1",
        "PAIR063,2",
        "PAIR063,3",
        "PAIR063,4",
        "PAIR063,5",
        "PAIR063,6",
        "PAIR063,7",
        "PAIR063,8",
        "PAIR063,9",
    ] {
        match query(&mut gnss, body).await {
            Ok(line) => defmt::info!(
                "[INFO] S8c after reset ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => defmt::info!("[INFO] S8c after reset ${=str} -> {=str}", body, why),
        }
    }
    gnss
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!("[INFO] lc76g_smoke: UART1 TX=GPIO24 RX=GPIO25 115200 8N1, PPS=GPIO5");

    unsafe {
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        nvic.set_priority(interrupt::SWI_IRQ_1, PPS_PRIORITY);
    }
    let pps_spawner = PPS_EXECUTOR.start(interrupt::SWI_IRQ_1);
    pps_spawner.spawn(count_pps(Input::new(p.PIN_5, Pull::None)).unwrap());

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

    let mut gnss = run(Lc76g::new(uart), &mut failures).await;

    // S9: restore. The module ends at 1 Hz, normal mode, default sentences, EPE off.
    let mut r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    if !matches!(r, Ok(Ok(_))) {
        defmt::error!("[INFO] S9 first init failed, trying again after back_to_1hz");
        back_to_1hz(&mut gnss).await;
        r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    }
    check!(
        failures,
        matches!(r, Ok(Ok(_))),
        "{=str} S9 init(&Config::default()) acknowledged"
    );
    if let Ok(Err(e)) = r {
        defmt::error!("[INFO] S9 init error: {}", e);
    }
    let steps = settle_1hz(&mut gnss, 3).await;
    check!(
        failures,
        steps.settled >= 3,
        "{=str} S9 fix time step 1000 ms over three steps ({=u32} fixes read, {=u32} other steps before)",
        steps.fixes,
        steps.transition
    );
    let c = census(&mut gnss, 2200).await;
    let mut line = Line::new();
    c.describe(&mut line);
    check!(
        failures,
        c.count("GGA") > 0
            && c.count("RMC") > 0
            && c.count("PQTMEPE") == 0
            && c.count("ZDA") == 0
            && c.count("GRS") == 0
            && c.count("GST") == 0
            && c.count("GNS") == 0,
        "{=str} S9 sentence set is the default (no EPE, ZDA, GRS, GST, GNS): {=str}",
        line.as_str()
    );
    for (body, want) in [("PAIR051", "$PAIR051,1000*"), ("PAIR081", "$PAIR081,0*")] {
        match query(&mut gnss, body).await {
            Ok(line) => check!(
                failures,
                line.as_str().starts_with(want),
                "{=str} S9 read back ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => check!(
                failures,
                false,
                "{=str} S9 read back ${=str}: {=str}",
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
