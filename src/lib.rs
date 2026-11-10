//! Driver for the Quectel LC76G GNSS module on a UART.
//!
//! Built on the `embedded-io` and `embedded-io-async` 0.7 traits only: it
//! works with any executor or none. There are three ways in, from the bottom up:
//!
//! * the sans-IO core, no UART at all: [`nmea::Framer`] turns bytes into validated
//!   [`nmea::Sentence`]s, [`nmea::Sentence::parse`] decodes them into [`sentence::Message`]s,
//!   [`fix::FixBuilder`] assembles them into [`fix::Fix`]es and [`command::Command::write`] encodes
//!   commands. Use this when your firmware drains its own DMA ring buffer.
//! * the async driver, [`Lc76g`] at the crate root, on `embedded_io_async::Read` (+ `Write`);
//! * [`blocking`]: the same driver on `embedded_io::Read` (+ `Write`).
//!
//! Receive-only wiring (module TX to the MCU only) works: `read_sentence`, `read` and `read_fix` need
//! only `Read`. `command`, `version`, `init` and `save_settings` need `Write` as well.
//!
//! The driver owns no timer, pin or task. A silent line makes `read*` wait forever and `command` wait
//! for its acknowledgement until [`ACK_SENTENCE_LIMIT`] other sentences have passed: in async code wrap
//! calls in your executor's timeout, with the blocking driver give the UART a read timeout. Hand the
//! driver a buffered UART (interrupt or DMA ring buffer): the module sends bursts of about 20
//! sentences (1.4 kB, 120 ms at 115200 baud) that a UART with a one-byte FIFO would overrun. Dropping
//! a `read*` or `command` future loses nothing the driver already read; a command write cut short may
//! leave a partial sentence on the wire, which the module discards by its checksum.
//!
//! Units: coordinates are `i32` in 1e-7 degree, altitude `i32` mm, speed `u32` mm/s, course `u16` in
//! 0.01 degree, DOP `u16` x100. [`units`] converts to floating point.
//!
//! Async:
//! ```ignore
//! let mut gnss = lc76g::Lc76g::new(uart);
//! let version = gnss.init(&lc76g::Config::default()).await?;
//! loop {
//!     let fix = gnss.read_fix().await?;
//!     if fix.valid {
//!         let (lat, lon) = (fix.lat_deg(), fix.lon_deg());
//!     }
//! }
//! ```
//!
//! Blocking, receive-only:
//! ```ignore
//! let mut gnss = lc76g::blocking::Lc76g::new(uart);
//! let fix = gnss.read_fix()?;
//! ```
//!
//! Sans-IO:
//! ```ignore
//! let mut framer = lc76g::nmea::Framer::new();
//! let mut builder = lc76g::fix::FixBuilder::new();
//! for &byte in dma_chunk {
//!     if let Some(sentence) = framer.push(byte) {
//!         log_raw(sentence.as_str());
//!         if let Some(fix) = builder.push(&sentence.parse()) { /* ... */ }
//!     }
//! }
//! ```
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod blocking;
pub mod command;
pub mod config;
mod field;
pub mod fix;
pub mod nmea;
pub mod sentence;
pub mod units;

use embedded_io_async::{Read, Write};

pub use command::{Command, CommandId, StartKind};
pub use config::{Config, NavMode, Pps, PpsType, SentenceRates, SentenceType};
pub use fix::{Fix, FixBuilder};
pub use nmea::{Framer, Sentence};
pub use sentence::{Message, Version};

use command::{MAX_COMMAND_LEN, Outcome};

/// Bytes read from the UART at a time.
const RX_CHUNK: usize = 64;

/// Sentences `command` lets pass while waiting for its acknowledgement before giving up with
/// [`Error::NoAck`].
///
/// The module answers a command within milliseconds, but the answer queues behind the burst being
/// sent. With the default stream one epoch is about 20 sentences (GGA, GLL, 4 GSA, 11 GSV, RMC, VTG)
/// and up to 10 epochs per second at 10 Hz add 9 more GGA and RMC each; an "in process" reply can
/// precede the final one. 100 sentences covers several epochs of the busiest stream, which is about
/// five seconds at the default rate, without making a dead line hang for long.
pub const ACK_SENTENCE_LIMIT: usize = 100;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// UART error.
    Uart(E),
    /// The UART returned zero bytes from a read or a write: the line is closed.
    Eof,
    /// [`ACK_SENTENCE_LIMIT`] sentences passed without the acknowledgement of the command.
    NoAck,
    /// The module refused the command. For PAIR commands `code` is the `$PAIR001` result (2 failed,
    /// 3 unsupported, 4 parameter error, 5 busy); for PQTM commands the error code (1 invalid
    /// parameters, 2 execution failed).
    Rejected {
        /// The command refused.
        command: CommandId,
        /// Result or error code.
        code: u8,
    },
    /// The configuration has a value outside the range the module accepts. Nothing was sent.
    InvalidConfig,
    /// A [`Command::Raw`] body that cannot be framed. Nothing was sent.
    InvalidCommand,
}

/// Counters of an [`Lc76g`]. They saturate at `u32::MAX`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stats {
    /// Valid sentences received.
    pub sentences: u32,
    /// Sentences with a bad or missing checksum, or cut off by the next `$`.
    pub checksum_errors: u32,
    /// Sentences longer than [`nmea::MAX_SENTENCE_LEN`].
    pub overflows: u32,
    /// Fixes completed.
    pub fixes: u32,
    /// Epochs abandoned because the GGA and RMC times did not match.
    pub epochs_dropped: u32,
    /// Fixes completed but never returned: during `command`, or replaced in the driver before
    /// `read_fix` took them.
    pub fixes_discarded: u32,
}

/// LC76G on an async UART (115200 baud 8N1 by default).
pub struct Lc76g<UART> {
    uart: UART,
    rx: [u8; RX_CHUNK],
    pos: usize,
    end: usize,
    framer: Framer,
    builder: FixBuilder,
    held: Option<Fix>,
    fixes: u32,
    fixes_discarded: u32,
}

impl<UART> Lc76g<UART> {
    /// Wraps the UART. Nothing is sent or read until a method is called.
    pub const fn new(uart: UART) -> Self {
        Self {
            uart,
            rx: [0; RX_CHUNK],
            pos: 0,
            end: 0,
            framer: Framer::new(),
            builder: FixBuilder::new(),
            held: None,
            fixes: 0,
            fixes_discarded: 0,
        }
    }

    /// Returns the UART. Bytes read from it but not yet consumed are lost.
    pub fn release(self) -> UART {
        self.uart
    }

    /// Counters since construction.
    pub fn stats(&self) -> Stats {
        let framer = self.framer.stats();
        Stats {
            sentences: framer.sentences,
            checksum_errors: framer.checksum_errors,
            overflows: framer.overflows,
            fixes: self.fixes,
            epochs_dropped: self.builder.dropped(),
            fixes_discarded: self.fixes_discarded,
        }
    }

    /// Feeds a decoded message to the fix builder.
    fn ingest(&mut self, message: &Message) -> Option<Fix> {
        let fix = self.builder.push(message);
        if fix.is_some() {
            self.fixes = self.fixes.saturating_add(1);
        }
        fix
    }

    fn discard_fix(&mut self) {
        self.fixes_discarded = self.fixes_discarded.saturating_add(1);
    }
}

impl<UART: Read> Lc76g<UART> {
    async fn next_sentence(&mut self) -> Result<Sentence<'_>, Error<UART::Error>> {
        loop {
            while self.pos < self.end {
                let byte = self.rx[self.pos];
                self.pos += 1;
                if self.framer.advance(byte) {
                    return Ok(self.framer.ready());
                }
            }
            let n = self.uart.read(&mut self.rx).await.map_err(Error::Uart)?;
            if n == 0 {
                return Err(Error::Eof);
            }
            self.pos = 0;
            self.end = n.min(RX_CHUNK);
        }
    }

    /// Next valid sentence, raw. Bytes after it stay in the driver for the next call. The sentence is
    /// not decoded and does not go through the fix builder; use it for logging.
    pub async fn read_sentence(&mut self) -> Result<Sentence<'_>, Error<UART::Error>> {
        self.next_sentence().await
    }

    /// Next valid sentence, decoded. A fix completed by it is kept for the next
    /// [`Lc76g::read_fix`].
    pub async fn read(&mut self) -> Result<Message, Error<UART::Error>> {
        let message = self.next_sentence().await?.parse();
        if let Some(fix) = self.ingest(&message)
            && self.held.replace(fix).is_some()
        {
            self.discard_fix();
        }
        Ok(message)
    }

    /// Next complete fix (see [`FixBuilder`]); other sentences on the way are consumed.
    pub async fn read_fix(&mut self) -> Result<Fix, Error<UART::Error>> {
        if let Some(fix) = self.held.take() {
            return Ok(fix);
        }
        loop {
            let message = self.next_sentence().await?.parse();
            if let Some(fix) = self.ingest(&message) {
                return Ok(fix);
            }
        }
    }
}

impl<UART: Read + Write> Lc76g<UART> {
    /// Sends `command` and consumes sentences until its acknowledgement: `$PAIR001` with the command's
    /// ID for PAIR commands (result 1, "being processed", keeps waiting), `OK` / `ERROR` for PQTM
    /// commands. A command with no known reply ([`Command::Raw`]) returns after it is sent.
    ///
    /// Fixes that complete while waiting are discarded and counted in [`Stats::fixes_discarded`].
    /// Gives up after [`ACK_SENTENCE_LIMIT`] sentences; on a silent line it waits forever, see the
    /// crate docs. A stale acknowledgement of the same command from an earlier, abandoned call is
    /// accepted as the answer.
    pub async fn command(&mut self, command: &Command<'_>) -> Result<(), Error<UART::Error>> {
        self.exchange(command).await.map(|_| ())
    }

    /// Queries the firmware version (`$PQTMVERNO`). This is the link check: it fails unless the module
    /// answers.
    pub async fn version(&mut self) -> Result<Version, Error<UART::Error>> {
        match self.exchange(&Command::QueryVersion).await? {
            Some(Message::Version(version)) => Ok(version),
            // QueryVersion completes only on a version answer, so this is not reached.
            _ => Err(Error::NoAck),
        }
    }

    /// Link check with [`Lc76g::version`], then writes every setting of `config`, each acknowledged.
    /// Validates `config` before sending anything ([`Config::is_valid`], which includes the stream
    /// budget). Never saves to the module's flash; the settings last until power loss or reset (or
    /// until [`Lc76g::save_settings`]).
    ///
    /// Order: version query; fix interval 1000 ms; every sentence rate; EPE; navigation mode; PPS if
    /// set; then the fix interval of `config` unless it is 1000 ms. The module is at 1 Hz while the
    /// rates change, so no intermediate state exceeds the line budget when moving between any two valid
    /// configurations.
    pub async fn init(&mut self, config: &Config) -> Result<Version, Error<UART::Error>> {
        if !config.is_valid() {
            return Err(Error::InvalidConfig);
        }
        let version = self.version().await?;
        self.command(&Command::SetFixInterval { ms: 1000 }).await?;
        for sentence in SentenceType::ALL {
            let rate = config.rates.get(sentence);
            self.command(&Command::SetSentenceRate { sentence, rate })
                .await?;
        }
        self.command(&Command::SetEpe(config.epe)).await?;
        self.command(&Command::SetNavMode(config.nav_mode)).await?;
        if let Some(pps) = config.pps {
            self.command(&Command::SetPps(pps)).await?;
        }
        if config.fix_interval_ms != 1000 {
            self.command(&Command::SetFixInterval {
                ms: config.fix_interval_ms,
            })
            .await?;
        }
        Ok(version)
    }

    /// Saves the current settings to the module's flash (`$PAIR513`) and the PQTM settings to its NVDM
    /// (`$PQTMSAVEPAR`). Explicit and separate from [`Lc76g::init`] to spare the flash. Above 1 Hz the
    /// specification requires the GNSS system to be powered off around `$PAIR513`, see [`Command`].
    pub async fn save_settings(&mut self) -> Result<(), Error<UART::Error>> {
        self.command(&Command::SaveSettings).await?;
        self.command(&Command::SavePqtm).await
    }

    async fn send(&mut self, command: &Command<'_>) -> Result<(), Error<UART::Error>> {
        let mut buf = [0u8; MAX_COMMAND_LEN];
        let len = command.write(&mut buf).ok_or(Error::InvalidCommand)?;
        let mut rest = &buf[..len];
        while !rest.is_empty() {
            match self.uart.write(rest).await.map_err(Error::Uart)? {
                0 => return Err(Error::Eof),
                n => rest = &rest[n..],
            }
        }
        self.uart.flush().await.map_err(Error::Uart)
    }

    async fn exchange(
        &mut self,
        command: &Command<'_>,
    ) -> Result<Option<Message>, Error<UART::Error>> {
        self.send(command).await?;
        let Some(expected) = command.reply() else {
            return Ok(None);
        };
        for _ in 0..ACK_SENTENCE_LIMIT {
            let message = self.next_sentence().await?.parse();
            if self.ingest(&message).is_some() {
                self.discard_fix();
            }
            match expected.outcome(&message) {
                Outcome::Done => return Ok(Some(message)),
                Outcome::Rejected(code) => {
                    return Err(Error::Rejected {
                        command: expected,
                        code,
                    });
                }
                Outcome::Wait | Outcome::Unrelated => {}
            }
        }
        Err(Error::NoAck)
    }
}

#[cfg(test)]
mod tests;
