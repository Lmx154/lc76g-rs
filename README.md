# lc76g

Driver for the Quectel LC76G GNSS module over a UART, written for the MARV V2 Rev 1 flight controller. The crate is `no_std` and has three layers. The NMEA framer (`Framer`), the sentence decoders (`Sentence::parse`), the fix builder (`FixBuilder`) and the command encoder (`Command::write`) work on bytes and need no I/O at all. On top of them, `Lc76g` drives a UART through the `embedded-io-async` traits, and `blocking::Lc76g` does the same through the `embedded-io` traits. The only dependencies are `embedded-io` 0.7, `embedded-io-async` 0.7 and, optionally, `defmt`.

**Status: version 0.1.0 (`publish = false`). The API may still change.** It has been exercised on one board and one module firmware; see "Not covered and not verified".

## Quick start

No hardware needed. The bytes go through the framer, each valid sentence is decoded, and the fix builder joins the GGA and RMC of one epoch. The coordinates are made up. A bad checksum or a malformed field comes back as a value, not as a panic.

```rust
use lc76g::{FixBuilder, Framer, Message, Sentence};

let stream: &[u8] = b"$GNGGA,120000.000,0112.345678,N,00234.567890,E,1,12,0.60,10.500,M,-17.000,M,,*62\r\n\
$GNRMC,120000.000,A,0112.345678,N,00234.567890,E,0.250,90.50,111126,,,A,V*0F\r\n\
$GNZDA,120000.000,11,11,2026,,*4D\r\n";

let mut framer = Framer::new();
let mut builder = FixBuilder::new();
let (mut fix, mut zda) = (None, None);
for &byte in stream {
    if let Some(sentence) = framer.push(byte) {
        let message = sentence.parse();
        if let Message::Zda(z) = message {
            zda = Some(z);
        }
        if let Some(f) = builder.push(&message) {
            fix = Some(f);
        }
    }
}
let fix = fix.unwrap();
assert!(fix.valid);
assert_eq!(fix.time.unwrap().millis_of_day(), 12 * 3_600_000);
assert_eq!(fix.alt_mm, Some(10_500));
assert!(fix.lat_deg().unwrap() > 1.2 && fix.lat_deg().unwrap() < 1.21);
assert_eq!(zda.unwrap().date.unwrap().year, 2026);
assert_eq!(framer.stats().sentences, 3);

// Wrong checksum: no sentence is produced.
let wrong = "$GNGGA,120000.000,0112.345678,N,00234.567890,E,1,12,0.60,10.500,M,-17.000,M,,*00";
assert!(Sentence::new(wrong).is_none());
// Right checksum, malformed latitude: the result is a value.
let malformed = "$GNGGA,120000.000,01x2.345678,N,00234.567890,E,1,12,0.60,10.500,M,-17.000,M,,*2B";
assert_eq!(Sentence::new(malformed).unwrap().parse(), Message::Malformed);
```

The driver on a UART. Here a scripted UART stands in for the hardware; on the board the UART is the one described below, and `init(&Config)` (not run here) would follow `version`.

```rust
# use core::convert::Infallible;
# use core::future::Future;
# use core::pin::pin;
# use core::task::{Context, Poll, Waker};
# use embedded_io_async::{ErrorType, Read, Write};
# struct Script<'a> { rx: &'a [u8], tx: Vec<u8> }
# impl ErrorType for Script<'_> { type Error = Infallible; }
# impl Read for Script<'_> {
#     async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
#         let n = buf.len().min(self.rx.len());
#         let (head, tail) = self.rx.split_at(n);
#         buf[..n].copy_from_slice(head);
#         self.rx = tail;
#         Ok(n)
#     }
# }
# impl Write for Script<'_> {
#     async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
#         self.tx.extend_from_slice(buf);
#         Ok(buf.len())
#     }
#     async fn flush(&mut self) -> Result<(), Infallible> { Ok(()) }
# }
# fn block_on<F: Future>(f: F) -> F::Output {
#     let mut f = pin!(f);
#     let mut cx = Context::from_waker(Waker::noop());
#     loop {
#         if let Poll::Ready(v) = f.as_mut().poll(&mut cx) { return v; }
#     }
# }
# let rx = b"$PQTMVERNO,LC76GABNR02A01S,2022/09/14,11:47:03*3D\r\n\
# $GNGGA,120000.000,0112.345678,N,00234.567890,E,1,12,0.60,10.500,M,-17.000,M,,*62\r\n\
# $GNRMC,120000.000,A,0112.345678,N,00234.567890,E,0.250,90.50,111126,,,A,V*0F\r\n";
# block_on(async {
let mut gps = lc76g::Lc76g::new(Script { rx, tx: Vec::new() });
let version = gps.version().await.unwrap();      // sends $PQTMVERNO, waits for the answer
assert_eq!(version.version(), "LC76GABNR02A01S");
let fix = gps.read_fix().await.unwrap();         // next GGA + RMC pair of one epoch
assert!(fix.valid);
assert_eq!(gps.stats().fixes, 1);
# });
```

On the board (UART1, PPS, timeouts, the full sequence) see `hil/src/bin/lc76g_smoke.rs`.

## What the application provides

- A UART at 115200 baud, 8N1 (the default in the rustdoc, and what `lc76g_smoke` uses). The async driver needs `embedded_io_async::Read`, plus `Write` to send commands; `blocking::Lc76g` needs `embedded_io::Read` and `Write`. Receive-only wiring works for `read_sentence`, `read` and `read_fix`.
- A buffered UART: the module sends bursts of about 20 sentences (1.4 kB, 120 ms at 115200 baud), which a UART with a one-byte FIFO would overrun (rustdoc).
- With embassy-rp 0.10 the on-board program uses `embassy_rp::uart::BufferedUart` (interrupt driven, `BufferedInterruptHandler`) as the UART of `Lc76g`, with a 4096-byte receive and a 256-byte transmit buffer.
- Timeouts. The driver owns no timer, pin or task. On a silent line `read*` waits forever; wrap calls in the executor's timeout (async), or give the UART a read timeout (blocking).
- The PPS pin and its time-stamping. The driver does not touch it. `Command::SetPps` configures the module's pulse.

## Rules for correct use

- **No panics on module input.** Malformed input is an error value (`Message::Malformed`, `None`, a counted checksum error or overflow). `src/lib.rs` carries a `deny` block of clippy lints (`unwrap_used`, `expect_used`, `panic`, `unreachable`, `todo`, `unimplemented`, `indexing_slicing`) outside tests; they are enforced when `cargo clippy` runs, which is part of this repository's checks. `forbid(unsafe_code)` is enforced by the compiler. `src/fuzz_tests.rs` holds the property tests. This is lints plus randomised tests, not a proof; the lint block does not cover arithmetic overflow.
- **Dropped reads.** The rustdoc says: dropping a `read*` or `command` future loses nothing the driver already read; a command write cut short may leave a partial sentence on the wire, which the module discards by its checksum. The test `dropping_a_read_future_loses_nothing` (`src/tests.rs`) covers a dropped read. With the blocking driver, a UART read timeout comes out as `Error::Uart` and nothing read is lost.
- **Commands.** `command` returns `Ok(())` when the module acknowledges (`$PAIR001,<id>,0` for PAIR commands, `OK` for PQTM commands; a "being processed" result keeps waiting), `Err(Error::Rejected { command, code })` when it refuses, and `Err(Error::NoAck)` after `ACK_SENTENCE_LIMIT` (100) other sentences without the acknowledgement. On a silent line it waits forever. A `Command::Raw` whose reply is not known returns once sent. **Abandoned commands.** An acknowledgement is matched by command number only, so the driver keeps a private mark that an exchange is open: set before the first byte of a command is written, cleared when the exchange has read its own acknowledgement (accepted or rejected). It stays set when the `command` future is dropped (for example by a timeout), after `Err(Error::NoAck)` and after a UART error. The next `command` (also `version`, `init`, `save_settings`) then first resynchronises: it sends a line end and a read-only query, `$PAIR051`, `$PAIR081` or `$PQTMVERNO` (the first whose number is neither the abandoned command's nor the new command's; none changes a setting or writes the module's flash), and discards sentences, at most `ACK_SENTENCE_LIMIT`, up to the query's answer, so a late acknowledgement of the abandoned command is not taken for the answer to the next one. If the query gets no answer the call returns `Err(Error::NoAck)` without sending the command, and the next call tries again. With the mark clear the driver sends exactly the command bytes, nothing more. This assumes that the module answers commands in the order it received them; that is not verified in this revision. If a resynchronisation is itself cut short or ends in `NoAck`, the driver remembers its query and the next attempt uses another one (also different from the abandoned and the new command), so the late answer of the first query cannot end the second resynchronisation early. What remains: when the caller's own commands are two of the three queries there is no other candidate and the same query is used again, and the driver remembers only the last unanswered query, so after two unfinished resynchronisations in a row the third can choose the first query again; in both cases a late answer of an earlier query can end a resynchronisation early, and the answer to its query can then arrive after the call and be taken for the answer to a later call of that query command (`$PAIR051`, `$PAIR081` or `$PQTMVERNO`).
- **Fixes seen during `command`** are discarded and counted in `Stats::fixes_discarded`.
- **Stored configuration and restarts.** `init` never writes the module's flash; settings are volatile until power loss or reset. `Command::SaveSettings` (`$PAIR513`) writes the module's flash and `Command::SavePqtm` (`$PQTMSAVEPAR`) its NVDM; `Lc76g::save_settings` sends both. `Command::Start` (`Hot`, `Warm`, `Cold`, `FullCold` = `$PAIR004` to `$PAIR007`) restarts the GNSS engine, and `FullCold` also resets the module to factory settings. The rustdoc notes that above 1 Hz the specification requires powering the GNSS system off (`$PAIR382,1`, `$PAIR003`) before `$PAIR513` and on again (`$PAIR002`) after; `save_settings` does not do this. `Command::Raw` can send any sentence body, including these, provided the body is non-empty, at most 124 bytes, printable ASCII and free of `$` and `*`; otherwise `Error::InvalidCommand` is returned and nothing is sent.
- **Other error values.** `Error::Eof`: the UART returned zero bytes from a read or a write (the line is closed). `Error::InvalidConfig`: `init` found a `Config` with a value outside the accepted range, or one that breaks the stream budget; nothing was sent. `Error::InvalidCommand`: a `Command::Raw` body that cannot be framed (above); nothing was sent.
- **`init` is not atomic.** It checks the configuration, queries the version, sets the 1000 ms interval, then the sentence rates and the other settings, and the target interval last. An error part-way leaves the module partly configured and at 1 Hz; call `init` again.
- **`release()`** returns the UART; bytes already read from it but not yet consumed by the driver are lost.

## What the driver does

Decoded sentences, as `Message` variants (new in this revision: GNS, ZDA, GST, GRS):

| Sentence | Yields |
|---|---|
| GGA | time, position, quality, satellites used, HDOP, altitude, geoid separation |
| RMC | time, date, validity, position, speed, course, mode |
| GSA | fix type, PDOP/HDOP/VDOP, up to 12 satellite IDs, system ID |
| GSV | up to four satellites (ID, elevation, azimuth, C/N0) per sentence |
| VTG, GLL | course and speed; position, time, validity |
| **ZDA** (new) | time, date, local zone hour and minute (the module leaves the zone empty); `utc_unix_ms()` |
| **GNS** (new) | time, position, one mode letter per constellation, satellites used, HDOP, altitude, geoid separation |
| **GST** (new) | standard deviations (RMS, ellipse axes, orientation, lat, lon, altitude), mm |
| **GRS** (new) | mode and twelve range residuals, mm; optional system and signal ID |
| `$PAIR001` | acknowledgement of a PAIR command (`Ack`) |
| `$PQTMVERNO`, `$PQTMEPE` | firmware version; estimated position error (N, E, D, 2D, 3D, mm) |
| `$PQTM...,OK/ERROR` | reply to a PQTM command |

`Message::Other` is any valid sentence not listed. `Message::Malformed` is a valid checksum with fields that do not fit the layout. A field the module leaves empty decodes to `None`.

- **Framer.** Bytes in any chunking; a sentence is returned when its line ends and its checksum is right. It resynchronises on `$`. `Framer::stats()` counts sentences, checksum errors (including sentences cut off by the next `$`) and overflows (longer than `MAX_SENTENCE_LEN` = 128 bytes). `Lc76g::stats()` adds fixes, dropped epochs and discarded fixes; counters saturate.
- **Fix builder.** A `Fix` is made when the GGA and RMC of the same UTC time have both arrived. It holds time, date, `valid` (RMC status), quality, `lat_e7`, `lon_e7`, `alt_mm`, `geoid_sep_mm`, `speed_mm_s`, `course_cdeg`, `sats_used`, `hdop_x100`, and from the latest GSA `fix_type`, `pdop_x100`, `vdop_x100` (up to a second older above 1 Hz; `None` before a GSA). A mismatched epoch is dropped and counted.
- **Units.** Integers: 1e-7 degree, mm, mm/s, 0.01 degree, DOP x100. `units` and `Fix::lat_deg` and similar convert to floating point.
- **Commands** (`Command`): version query, fix interval (100 to 1000 ms), sentence rate per type, rate reset, navigation mode, PPS, start types, EPE on/off, save, `Raw`. `Config` and `init` apply a full set; `init` refuses a configuration that breaks the stream budget (below 1000 ms, GSA, GSV and GRS at most once per second; `Config::with_fix_interval` sets rates that fit).
- **Blocking front-end.** `blocking::Lc76g` runs the same code over blocking `embedded-io` traits.

## Not covered and not verified

Not implemented: any sentence outside the table (RLM, other PAIR and PQTM replies decode to `Other`); binary protocols; typed read-back of settings (the on-board program reads them through `Command::Raw` and looks at the text); baud-rate change and constellation selection (`Config` documents both as excluded because the module restarts); the module's EPE merged into `Fix`; time synchronisation from PPS; recovery of a module left flooded by other software. Further module features (jamming status, SBAS, elevation mask, low power, geofence, logging) have no `Command` variant. The earlier version of this README (2026-11-10) recorded one `NoAck` from `init` with the default configuration while the module was running at 10 Hz; it did not happen again in the 16 runs on the final code, and the cause was not found.

Not verified on the real module:

- **GNS, ZDA, GST, GRS.** `lc76g_smoke` never examines a decoded GNS, ZDA, GST or GRS. It switches ZDA on for a moment (S8b) and then checks that all four are absent after `init(&Config::default())`. These four decoders are tested only against example sentences (`src/aux_tests.rs`).
- **GNS layout.** The decoder follows the layout of the module manufacturer's example: 14 or 15 fields with two unit fields (`M`) that standard NMEA 0183 does not have. It has not been checked against sentences from the real module. A standard-layout GNS sentence is rejected (`Message::Malformed`). The mode letters `P` and `S` are rejected too; only `A D E F M N R` are accepted.
- Reading through a DMA UART has not been tried.
- Only one firmware was used: `LC76GABNR12A05S`.

## Measured on the MARV V2 Rev 1 board

From `docs/device.md` (bench module `LC76GABNR12A05S`, RP2350 at 150 MHz) unless noted.

- 115200 baud 8N1 confirmed. Default stream: GGA, GLL, GSA, GSV, RMC, VTG every fix; ZDA, GRS, GST, GNS off.
- 100 ms fix interval with default rates: 187 sentences, 11.6 kB per second (the line carries 11.52 kB/s); fixes drop (97 per 10 s) and commands are answered a second late or never. With GSA and GSV at 10: 100 fixes per 10 s, about 65 sentences per second, replies in milliseconds.
- At 10 Hz, about 57 of 99 fix-time steps are exactly 100 ms, the rest 89 to 107 ms, rarely up to 172 ms.
- `read_fix` returns the fix of a second 75 to 90 ms after the PPS edge. PPS period: 1 s to 5 ppm by the MCU timer, one period 1 to 3 ms off in about half the runs (cause undecided).
- Framing one GGA 49 us; framing and parsing 115 us (RMC: 46 us and 116 us).
- `Lc76g<BufferedUart>` is 440 bytes plus the UART buffers.
- Drone mode: six decimals of minutes in coordinates; the decoders accept six or seven.

## Tests

- Host: `cargo test --locked` runs 85 tests (observed on this tree). Property tests (13, plus one regression test, `src/fuzz_tests.rs`): arbitrary bytes in arbitrary chunks into the framer and the driver; arbitrary and multi-byte text into every field parser and every decoder; check-summed sentences around random fields; arbitrary replies into the command layer.
- On the board, through a debug probe (probe-rs, chip RP235x; the script takes a lock so runs do not overlap): `tools/run.sh alive` and `tools/run.sh lc76g_smoke`. The exit status is 0 when every check passed. `lc76g_smoke` checks the version query, `init` at 10 Hz, 100 fixes in 10 s (plus or minus 2), plausible fixes, decode cost, PPS period, the blocking front-end, and the return to default settings.
- `lc76g_smoke` documents that it never sends a flash write, a baud change, a restart or a low-power command. In the program the only `Raw` bodies are `PAIR051`, `PAIR063,<n>` and `PAIR081` (get commands), and neither `SaveSettings`, `SavePqtm` nor `Start` appears. It ends with `init(&Config::default())`.
- `tools/run.sh lc76g_resync 300` exercises abandoned commands on the module. Observed (2026-11-11, two runs): the module answers two back-to-back queries in the order sent (24 of 24 pairs); a command issued right after an abandoned one of the same number returns its own answer (40 of 40, both directions: accepted after rejected and rejected after accepted); the same sequence without the driver's resynchronisation takes the stale answer (24 of 24, so the hazard is real); 10 Hz fixes read under 20 ms timeouts lose nothing (100 of 100, 444 dropped calls). A resynchronisation cost about 10 ms and 4 sentences at 10 Hz; the module's command-to-acknowledgement time was 4.4 ms at best, 5.6 ms median and 93 ms at worst. The program sends only get commands and NMEA output-rate settings for ZDA (some deliberately invalid, which the module rejects), never a flash write, and ends with `init(&Config::default())`.

## Cargo features

| Feature | Default | Effect |
|---|---|---|
| `defmt` | off | `defmt::Format` derives on public types; enables `embedded-io/defmt` and `embedded-io-async/defmt` |

## Layout and licence

```text
src/            the driver (nmea, sentence, aux_sentences, fix, command, config, units, blocking)
docs/device.md  protocol, timing and measured facts, pins
hil/            on-board programs (own workspace; the only place that uses embassy-rp)
tools/run.sh    builds, flashes and runs one on-board program
```

MIT, see `LICENSE`.
