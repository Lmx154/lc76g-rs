//! On-board check of the resynchronisation of the lc76g driver after an abandoned command exchange.
//! LC76G on UART1 (GPIO24 = TX to the module, GPIO25 = RX from the module, 115200 8N1), as in
//! `lc76g_smoke`. The module is not power-cycled between runs, and the program always ends with
//! `init(&Config::default())`. It never sends a flash write, a baud change, a restart or a low-power
//! command; the only settings it changes are the ZDA rate (0 at the end) and the ones `init` sets.
//!
//! R0 start, R1 order of answers, R2 abandoned exchange through the driver, R3 the same without the
//! driver's protection, R4 reads under timeouts, R5 end.
#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use lc76g::command::MAX_COMMAND_LEN;
use lc76g::{Command, Config, Error, Lc76g, Message, NavMode, SentenceType};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type Gnss = Lc76g<BufferedUart>;
type GnssError = Error<uart::Error>;

const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 256;

/// Time a command exchange is allowed to run before it is dropped. The command is at most 18 bytes,
/// 1.6 ms at 115200 baud; the module cannot have answered and the answer (17 bytes, 1.5 ms more)
/// cannot have been read by then.
const ABANDON_US: u64 = 3000;
/// Abandoned exchanges wanted per direction in R2.
const REPS: usize = 20;
/// Pairs of queries in R1.
const PAIRS_R1: u32 = 24;
/// Tries of the unprotected sequence in R3.
const TRIES_R3: u32 = 24;
/// Tag of the version answer, which is not a `$PAIR001`.
const VERSION_TAG: u16 = 9999;
/// Seconds of each window in R4.
const WINDOW_SECS: u64 = 10;
/// Timeout of one `read_fix` call in the timed window of R4 (a fix every 100 ms).
const R4_TIMEOUT_MS: u64 = 20;

/// Bodies that the module should reject with the same command number as a valid ZDA rate.
const INVALID_ZDA: [&str; 3] = ["PAIR062,6,21", "PAIR062,6,100", "PAIR062,99,1"];

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

/// Minimum, median and maximum of `values`.
fn spread(values: &[u32]) -> (u32, u32, u32) {
    let mut sorted = [0u32; 64];
    let n = values.len().min(sorted.len());
    sorted[..n].copy_from_slice(&values[..n]);
    sorted[..n].sort_unstable();
    if n == 0 {
        (0, 0, 0)
    } else {
        (sorted[0], sorted[n / 2], sorted[n - 1])
    }
}

/// Counts of the sentence types of a window that the final check needs.
#[derive(Default)]
struct Census {
    gga: u32,
    rmc: u32,
    epe: u32,
    zda: u32,
    grs: u32,
    gst: u32,
    gns: u32,
    all: u32,
}

async fn census(gnss: &mut Gnss, ms: u64) -> Census {
    let mut c = Census::default();
    let end = Instant::now() + Duration::from_millis(ms);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, gnss.read_sentence()).await {
            Ok(Ok(s)) => {
                let a = s.address();
                c.all += 1;
                c.gga += a.ends_with("GGA") as u32;
                c.rmc += a.ends_with("RMC") as u32;
                c.epe += a.ends_with("EPE") as u32;
                c.zda += a.ends_with("ZDA") as u32;
                c.grs += a.ends_with("GRS") as u32;
                c.gst += a.ends_with("GST") as u32;
                c.gns += a.ends_with("GNS") as u32;
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    c
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

/// Brings the module to 1 Hz from any state (see `lc76g_smoke`).
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

/// Reads fixes until `need` consecutive steps of `want_ms`, at most 30 fixes. Returns the fixes read
/// and the consecutive steps at the end.
async fn settle(gnss: &mut Gnss, need: u32, want_ms: u32) -> (u32, u32) {
    let (mut fixes, mut settled) = (0u32, 0u32);
    let mut prev = None;
    while fixes < 30 && settled < need {
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
            if step_ms(p, t) == want_ms {
                settled += 1;
            } else {
                settled = 0;
            }
        }
        prev = t;
        fixes += 1;
    }
    (fixes, settled)
}

/// Writes the commands straight to the UART, back to back, without waiting for the module.
fn raw_write(uart: &mut BufferedUart, commands: &[&Command<'_>]) {
    let mut buf = [0u8; 2 * MAX_COMMAND_LEN];
    let mut n = 0;
    for command in commands {
        n += command.write(&mut buf[n..]).unwrap_or(0);
    }
    let mut rest = &buf[..n];
    while !rest.is_empty() {
        match uart.blocking_write(rest) {
            Ok(0) | Err(_) => break,
            Ok(k) => rest = &rest[k..],
        }
    }
}

/// Command number and result of an acknowledgement, or `VERSION_TAG` for the version answer.
fn tag(message: &Message) -> Option<(u16, u8)> {
    match message {
        Message::Ack(ack) => Some((ack.command, ack.result)),
        Message::Version(_) => Some((VERSION_TAG, 0)),
        _ => None,
    }
}

fn zda_rate(rate: u8) -> Command<'static> {
    Command::SetSentenceRate {
        sentence: SentenceType::Zda,
        rate,
    }
}

// ---------------------------------------------------------------------------------------------
// R1
// ---------------------------------------------------------------------------------------------

const QUERIES: [(Command<'static>, u16, &str); 4] = [
    (Command::Raw("PAIR051"), 51, "PAIR051"),
    (Command::Raw("PAIR081"), 81, "PAIR081"),
    (Command::Raw("PAIR063,6"), 63, "PAIR063,6"),
    (Command::QueryVersion, VERSION_TAG, "PQTMVERNO"),
];
const PAIRS: [(usize, usize); 4] = [(0, 1), (2, 3), (1, 3), (0, 2)];

/// Writes both queries back to back and returns the 1-based positions of their answers among the
/// sentences read afterwards (0 = not seen).
async fn r1_pair(gnss: Gnss, a: usize, b: usize) -> (Gnss, u32, u32) {
    let mut uart = gnss.release();
    raw_write(&mut uart, &[&QUERIES[a].0, &QUERIES[b].0]);
    let mut g = Lc76g::new(uart);
    let (mut pa, mut pb, mut n) = (0u32, 0u32, 0u32);
    let end = Instant::now() + Duration::from_secs(2);
    while (pa == 0 || pb == 0) && n < 400 {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, g.read_sentence()).await {
            Ok(Ok(s)) => {
                n += 1;
                if let Some((t, _)) = tag(&s.parse()) {
                    if t == QUERIES[a].1 && pa == 0 {
                        pa = n;
                    }
                    if t == QUERIES[b].1 && pb == 0 {
                        pb = n;
                    }
                }
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    (g, pa, pb)
}

async fn r1(mut gnss: Gnss, failures: &mut u32) -> Gnss {
    let (mut in_order, mut swapped, mut missing) = (0u32, 0u32, 0u32);
    let mut example: Option<(u32, usize, usize, u32, u32)> = None;
    for p in 0..PAIRS_R1 {
        let (i, j) = PAIRS[(p % 4) as usize];
        let (a, b) = if (p / 4) % 2 == 1 { (j, i) } else { (i, j) };
        let (g, pa, pb) = r1_pair(gnss, a, b).await;
        gnss = g;
        if pa == 0 || pb == 0 {
            missing += 1;
            defmt::info!(
                "[INFO] R1 pair {=u32}: ${=str} then ${=str}: answer positions {=u32}, {=u32} (0 = not seen)",
                p,
                QUERIES[a].2,
                QUERIES[b].2,
                pa,
                pb
            );
        } else if pa < pb {
            in_order += 1;
        } else {
            swapped += 1;
            if example.is_none() {
                example = Some((p, a, b, pa, pb));
            }
        }
        Timer::after_millis(50).await;
    }
    if let Some((p, a, b, pa, pb)) = example {
        defmt::error!(
            "[INFO] R1 first swapped pair {=u32}: sent ${=str} then ${=str}; their answers were sentence {=u32} and {=u32} after the write",
            p,
            QUERIES[a].2,
            QUERIES[b].2,
            pa,
            pb
        );
    }
    check!(
        *failures,
        swapped == 0 && in_order >= 20,
        "{=str} R1 order of answers: {=u32} pairs in order, {=u32} swapped, {=u32} with an answer missing (of {=u32}; first query alternates)",
        in_order,
        swapped,
        missing,
        PAIRS_R1
    );
    gnss
}

// ---------------------------------------------------------------------------------------------
// R2
// ---------------------------------------------------------------------------------------------

enum Case {
    /// The first exchange finished within the abandon time: nothing was abandoned.
    Early,
    Second {
        result: Result<(), GnssError>,
        sentences: u32,
        us: u32,
    },
    SecondTimeout,
}

/// Starts `first`, drops it after `ABANDON_US`, and at once issues `second`.
async fn abandon_then(gnss: &mut Gnss, first: &Command<'_>, second: &Command<'_>) -> Case {
    if let Ok(r) = with_timeout(Duration::from_micros(ABANDON_US), gnss.command(first)).await {
        if let Err(e) = r {
            count_err(&e);
        }
        return Case::Early;
    }
    let s0 = gnss.stats().sentences;
    let t0 = Instant::now();
    match with_timeout(Duration::from_secs(3), gnss.command(second)).await {
        Ok(result) => {
            if let Err(e) = &result {
                count_err(e);
            }
            Case::Second {
                result,
                sentences: gnss.stats().sentences - s0,
                us: t0.elapsed().as_micros() as u32,
            }
        }
        Err(_) => Case::SecondTimeout,
    }
}

async fn r2(gnss: &mut Gnss, failures: &mut u32) {
    // Which body does the module reject with the command number of a valid ZDA rate?
    let mut invalid = None;
    for body in INVALID_ZDA {
        let r = with_timeout(Duration::from_secs(3), gnss.command(&Command::Raw(body))).await;
        match r {
            Ok(Err(Error::Rejected { code, .. })) => {
                defmt::info!(
                    "[INFO] R2 ${=str} is rejected by the module with result {=u8}",
                    body,
                    code
                );
                invalid.get_or_insert(body);
            }
            Ok(Ok(())) => defmt::info!("[INFO] R2 ${=str} is accepted by the module", body),
            _ => defmt::info!("[INFO] R2 ${=str}: no usable answer", body),
        }
    }
    let _ = with_timeout(Duration::from_secs(3), gnss.command(&zda_rate(0))).await;
    let Some(invalid) = invalid else {
        check!(
            *failures,
            false,
            "{=str} R2 no body with command number 62 is rejected by the module: no pair of different answers"
        );
        return;
    };

    // Latency of an ordinary acknowledgement, to show that ABANDON_US is shorter.
    let mut lat = [0u32; 20];
    for slot in lat.iter_mut() {
        let t0 = Instant::now();
        let _ = with_timeout(Duration::from_secs(3), gnss.command(&zda_rate(0))).await;
        *slot = t0.elapsed().as_micros() as u32;
    }
    let (lmin, lmed, lmax) = spread(&lat);
    defmt::info!(
        "[STAT] R2 ordinary command to acknowledgement read: min {=u32} us, median {=u32} us, max {=u32} us (abandon after {=u64} us)",
        lmin,
        lmed,
        lmax,
        ABANDON_US
    );

    // D1: valid abandoned, invalid second (must be Rejected). D2: invalid abandoned, valid second
    // (must be Ok).
    let mut done = [0usize; 2];
    let mut wrong = [0u32; 2];
    let mut early = [0u32; 2];
    let mut persisted = 0u32;
    let mut us = [[0u32; 64]; 2];
    let mut sent = [[0u32; 64]; 2];
    let mut first_wrong = true;
    let mut k = 0usize;
    while (done[0] < REPS || done[1] < REPS) && k < 8 * REPS {
        let mut d = k % 2;
        k += 1;
        if done[d] >= REPS {
            d = 1 - d;
        }
        let value = 2 + (done[0] % 2) as u8;
        let (first, second) = if d == 0 {
            (zda_rate(value), Command::Raw(invalid))
        } else {
            (Command::Raw(invalid), zda_rate(0))
        };
        match abandon_then(gnss, &first, &second).await {
            Case::Early => early[d] += 1,
            Case::Second {
                result,
                sentences,
                us: dur,
            } => {
                let right = if d == 0 {
                    matches!(result, Err(Error::Rejected { .. }))
                } else {
                    result.is_ok()
                };
                let i = done[d].min(63);
                us[d][i] = dur;
                sent[d][i] = sentences;
                done[d] += 1;
                if !right {
                    wrong[d] += 1;
                    if first_wrong {
                        first_wrong = false;
                        defmt::error!(
                            "[INFO] R2 first wrong case: direction {=usize} first {} second {} -> {} after {=u32} sentences, {=u32} us",
                            d + 1,
                            first,
                            second,
                            result,
                            sentences,
                            dur
                        );
                    }
                }
                if d == 0 {
                    let mut want = Line::new();
                    let _ = write!(want, "$PAIR063,6,{}*", value);
                    if let Ok(line) = query(gnss, "PAIR063,6").await
                        && line.as_str().starts_with(want.as_str())
                    {
                        persisted += 1;
                    }
                }
            }
            Case::SecondTimeout => {
                wrong[d] += 1;
                done[d] += 1;
                defmt::error!(
                    "[INFO] R2 direction {=usize}: second call did not finish within 3 s",
                    d + 1
                );
            }
        }
    }
    let _ = with_timeout(Duration::from_secs(3), gnss.command(&zda_rate(0))).await;
    check!(
        *failures,
        done[0] >= REPS && wrong[0] == 0,
        "{=str} R2 direction 1 (valid abandoned, then ${=str} must be Rejected): {=usize} cases, {=u32} wrong, {=u32} early (not abandoned)",
        invalid,
        done[0],
        wrong[0],
        early[0]
    );
    check!(
        *failures,
        done[1] >= REPS && wrong[1] == 0,
        "{=str} R2 direction 2 (${=str} abandoned, then valid must be Ok): {=usize} cases, {=u32} wrong, {=u32} early (not abandoned)",
        invalid,
        done[1],
        wrong[1],
        early[1]
    );
    defmt::info!(
        "[INFO] R2 direction 1: the abandoned command had been executed by the module (ZDA rate read back) in {=u32} of {=usize} cases, so it was fully sent",
        persisted,
        done[0]
    );
    for d in 0..2 {
        let n = done[d].min(64);
        let (dmin, dmed, dmax) = spread(&us[d][..n]);
        let (smin, smed, smax) = spread(&sent[d][..n]);
        defmt::info!(
            "[STAT] R2 direction {=usize} second call: duration min {=u32} / median {=u32} / max {=u32} us; sentences consumed min {=u32} / median {=u32} / max {=u32}",
            d + 1,
            dmin,
            dmed,
            dmax,
            smin,
            smed,
            smax
        );
    }
}

// ---------------------------------------------------------------------------------------------
// R3
// ---------------------------------------------------------------------------------------------

/// The old behaviour: write `first`, wait `ABANDON_US` without reading, write `second`, then take the
/// first acknowledgement of command 62 as the answer to `second`. Returns the first two
/// acknowledgements of command 62 that arrive.
async fn raw_stale(
    gnss: Gnss,
    first: &Command<'_>,
    second: &Command<'_>,
) -> (Gnss, [Option<u8>; 2]) {
    let mut uart = gnss.release();
    raw_write(&mut uart, &[first]);
    Timer::after_micros(ABANDON_US).await;
    raw_write(&mut uart, &[second]);
    let mut g = Lc76g::new(uart);
    let mut acks = [None; 2];
    let mut n = 0;
    let end = Instant::now() + Duration::from_secs(3);
    while acks[1].is_none() && n < 300 {
        let now = Instant::now();
        if now >= end {
            break;
        }
        match with_timeout(end - now, g.read_sentence()).await {
            Ok(Ok(s)) => {
                n += 1;
                if let Some((62, result)) = tag(&s.parse()) {
                    if acks[0].is_none() {
                        acks[0] = Some(result);
                    } else {
                        acks[1] = Some(result);
                    }
                }
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => break,
        }
    }
    (g, acks)
}

async fn r3(mut gnss: Gnss, invalid: &'static str, failures: &mut u32) -> Gnss {
    let (mut stale, mut fine, mut incomplete) = (0u32, 0u32, 0u32);
    let mut example: Option<(u32, u8, u8)> = None;
    for t in 0..TRIES_R3 {
        let (first, second) = if t % 2 == 0 {
            (zda_rate(2), Command::Raw(invalid))
        } else {
            (Command::Raw(invalid), zda_rate(0))
        };
        let (g, acks) = raw_stale(gnss, &first, &second).await;
        gnss = g;
        match acks {
            [Some(a), Some(b)] if a != b => {
                stale += 1;
                if example.is_none() {
                    example = Some((t, a, b));
                }
            }
            [Some(_), Some(_)] => fine += 1,
            _ => incomplete += 1,
        }
        Timer::after_millis(50).await;
    }
    let _ = with_timeout(Duration::from_secs(3), gnss.command(&zda_rate(0))).await;
    if let Some((t, a, b)) = example {
        defmt::info!(
            "[INFO] R3 first stale case, try {=u32}: the first acknowledgement of command 62 had result {=u8} (the abandoned command's), the second's own had result {=u8}",
            t,
            a,
            b
        );
    }
    check!(
        *failures,
        stale >= 1,
        "{=str} R3 without protection: stale acknowledgement taken in {=u32} of {=u32} tries ({=u32} tries had the same result twice, {=u32} incomplete)",
        stale,
        TRIES_R3,
        fine,
        incomplete
    );
    gnss
}

// ---------------------------------------------------------------------------------------------
// R4
// ---------------------------------------------------------------------------------------------

struct R4Window {
    fixes: u32,
    driver_fixes: u32,
    dropped_calls: u32,
    duplicates: u32,
    backwards: u32,
    checksum_errors: u32,
    ms: u32,
}

async fn r4_window(gnss: &mut Gnss, timeout_ms: Option<u64>) -> R4Window {
    let s0 = gnss.stats();
    let start = Instant::now();
    let end = start + Duration::from_secs(WINDOW_SECS);
    let mut w = R4Window {
        fixes: 0,
        driver_fixes: 0,
        dropped_calls: 0,
        duplicates: 0,
        backwards: 0,
        checksum_errors: 0,
        ms: 0,
    };
    let mut prev: Option<u32> = None;
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        let limit = match timeout_ms {
            Some(ms) => Duration::from_millis(ms).min(end - now),
            None => end - now,
        };
        match with_timeout(limit, gnss.read_fix()).await {
            Ok(Ok(fix)) => {
                w.fixes += 1;
                if let Some(t) = fix.time.map(|t| t.millis_of_day()) {
                    if let Some(p) = prev {
                        if t == p {
                            w.duplicates += 1;
                        } else if step_ms(p, t) > 43_200_000 {
                            w.backwards += 1;
                        }
                    }
                    prev = Some(t);
                }
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => w.dropped_calls += 1,
        }
    }
    let s1 = gnss.stats();
    w.driver_fixes = s1.fixes - s0.fixes;
    w.checksum_errors = s1.checksum_errors - s0.checksum_errors;
    w.ms = start.elapsed().as_millis() as u32;
    w
}

async fn r4(gnss: &mut Gnss, failures: &mut u32) {
    let a = r4_window(gnss, None).await;
    defmt::info!(
        "[STAT] R4 untimed window: {=u32} fixes in {=u32} ms, driver completed {=u32}, checksum errors {=u32}, duplicates {=u32}",
        a.fixes,
        a.ms,
        a.driver_fixes,
        a.checksum_errors,
        a.duplicates
    );
    let b = r4_window(gnss, Some(R4_TIMEOUT_MS)).await;
    defmt::info!(
        "[STAT] R4 timed window ({=u64} ms per call): {=u32} fixes in {=u32} ms, driver completed {=u32}, {=u32} calls dropped by the timeout",
        R4_TIMEOUT_MS,
        b.fixes,
        b.ms,
        b.driver_fixes,
        b.dropped_calls
    );
    check!(
        *failures,
        b.fixes.abs_diff(a.fixes) <= 1 && b.fixes == b.driver_fixes && a.fixes >= 98,
        "{=str} R4 fixes received under timeouts {=u32}, untimed {=u32}, completed inside the driver {=u32}",
        b.fixes,
        a.fixes,
        b.driver_fixes
    );
    check!(
        *failures,
        b.checksum_errors == 0 && a.checksum_errors == 0,
        "{=str} R4 checksum errors: timed {=u32}, untimed {=u32}",
        b.checksum_errors,
        a.checksum_errors
    );
    check!(
        *failures,
        b.duplicates == 0 && b.backwards == 0,
        "{=str} R4 fixes decoded twice (same time): {=u32}, going back in time: {=u32}",
        b.duplicates,
        b.backwards
    );
}

// ---------------------------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------------------------

/// R0 to R4. Returns the driver so that the caller can restore the module.
async fn run(mut gnss: Gnss, failures: &mut u32) -> Gnss {
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
        "[INFO] UART errors in the first 500 ms: {=u32}",
        startup_errors
    );

    // R0.
    let mut version = None;
    for attempt in 1..=5u32 {
        match with_timeout(Duration::from_secs(4), gnss.version()).await {
            Ok(Ok(v)) => {
                version = Some((v, attempt));
                break;
            }
            Ok(Err(e)) => count_err(&e),
            Err(_) => {}
        }
    }
    check!(
        *failures,
        version.is_some(),
        "{=str} R0 link: PQTMVERNO answered"
    );
    let Some((version, attempt)) = version else {
        return gnss;
    };
    defmt::info!(
        "[INFO] R0 firmware {=str}, answered on attempt {=u32}",
        version.version(),
        attempt
    );
    defmt::info!(
        "[INFO] R0 GSV off + SetFixInterval(1000) took {=u32} attempts in total (2 = first try, 0 = failed)",
        back_to_1hz(&mut gnss).await
    );
    let cfg = Config {
        nav_mode: NavMode::Drone,
        epe: true,
        ..Config::with_fix_interval(100)
    };
    let t0 = Instant::now();
    let r = with_timeout(Duration::from_secs(15), gnss.init(&cfg)).await;
    let ok = matches!(r, Ok(Ok(_)));
    check!(
        *failures,
        ok,
        "{=str} R0 init (with_fix_interval(100), drone, EPE on) acknowledged, took {=u32} ms",
        t0.elapsed().as_millis() as u32
    );
    if !ok {
        return gnss;
    }
    let (fixes, settled) = settle(&mut gnss, 3, 100).await;
    defmt::info!(
        "[INFO] R0 settle: {=u32} fixes read, {=u32} consecutive 100 ms steps at the end",
        fixes,
        settled
    );
    let _ = settle(&mut gnss, 20, 100).await;

    gnss = r1(gnss, failures).await;
    r2(&mut gnss, failures).await;
    gnss = r3(gnss, INVALID_ZDA[0], failures).await;
    let _ = settle(&mut gnss, 3, 100).await;
    r4(&mut gnss, failures).await;
    gnss
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    defmt::info!("[INFO] lc76g_resync: UART1 TX=GPIO24 RX=GPIO25 115200 8N1");

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

    // R5: restore. The module ends at 1 Hz, normal mode, default sentences, EPE off.
    let _ = with_timeout(Duration::from_secs(3), gnss.command(&zda_rate(0))).await;
    let mut r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    if !matches!(r, Ok(Ok(_))) {
        defmt::error!("[INFO] R5 first init failed, trying again after back_to_1hz");
        back_to_1hz(&mut gnss).await;
        r = with_timeout(Duration::from_secs(15), gnss.init(&Config::default())).await;
    }
    check!(
        failures,
        matches!(r, Ok(Ok(_))),
        "{=str} R5 init(&Config::default()) acknowledged"
    );
    if let Ok(Err(e)) = r {
        defmt::error!("[INFO] R5 init error: {}", e);
    }
    let (fixes, settled) = settle(&mut gnss, 3, 1000).await;
    check!(
        failures,
        settled >= 3,
        "{=str} R5 fix time step 1000 ms over three steps ({=u32} fixes read)",
        fixes
    );
    let c = census(&mut gnss, 2200).await;
    check!(
        failures,
        c.gga > 0
            && c.rmc > 0
            && c.epe == 0
            && c.zda == 0
            && c.grs == 0
            && c.gst == 0
            && c.gns == 0,
        "{=str} R5 sentence set is the default (no EPE, ZDA, GRS, GST, GNS): {=u32} sentences, GGA {=u32}, RMC {=u32}",
        c.all,
        c.gga,
        c.rmc
    );
    for (body, want) in [("PAIR051", "$PAIR051,1000*"), ("PAIR081", "$PAIR081,0*")] {
        match query(&mut gnss, body).await {
            Ok(line) => check!(
                failures,
                line.as_str().starts_with(want),
                "{=str} R5 read back ${=str} -> {=str}",
                body,
                line.as_str()
            ),
            Err(why) => check!(
                failures,
                false,
                "{=str} R5 read back ${=str}: {=str}",
                body,
                why
            ),
        }
    }
    match query(&mut gnss, "PAIR063,6").await {
        Ok(line) => check!(
            failures,
            line.as_str().starts_with("$PAIR063,6,0*"),
            "{=str} R5 read back $PAIR063,6 (ZDA off) -> {=str}",
            line.as_str()
        ),
        Err(why) => check!(
            failures,
            false,
            "{=str} R5 read back $PAIR063,6: {=str}",
            why
        ),
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
