//! Outgoing sentences: [`Command`] and its encoding.

use crate::config::{NavMode, Pps, SentenceType};
use crate::nmea::{MAX_SENTENCE_LEN, checksum};
use crate::sentence::{Message, PqtmCommand};

/// Size of a buffer that holds any command: the longest sentence plus CR LF.
pub const MAX_COMMAND_LEN: usize = MAX_SENTENCE_LEN + 2;

/// Which restart a [`Command::Start`] performs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StartKind {
    /// `PAIR004`: keep everything in NVRAM (fastest).
    Hot,
    /// `PAIR005`: keep rough time, position and almanac.
    Warm,
    /// `PAIR006`: forget time, position, almanac and ephemeris.
    Cold,
    /// `PAIR007`: cold start and reset the module to factory settings.
    FullCold,
}

/// A command and the reply that ends it: a `$PAIR001` for a PAIR command, `OK` / `ERROR` for a PQTM
/// command, the version sentence for `PQTMVERNO`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandId {
    /// `PAIR<id>`, acknowledged by `$PAIR001,<id>,<result>`.
    Pair(u16),
    /// A PQTM command.
    Pqtm(PqtmCommand),
}

/// How a received message relates to a pending command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Outcome {
    /// Not about this command.
    Unrelated,
    /// Accepted and finished.
    Done,
    /// Accepted, a further reply follows.
    Wait,
    /// Refused with this code.
    Rejected(u8),
}

impl CommandId {
    pub(crate) fn outcome(self, message: &Message) -> Outcome {
        match (self, message) {
            (Self::Pair(id), Message::Ack(ack)) if ack.command == id => match ack.result {
                0 => Outcome::Done,
                1 => Outcome::Wait,
                code => Outcome::Rejected(code),
            },
            (Self::Pqtm(PqtmCommand::VerNo), Message::Version(_)) => Outcome::Done,
            (Self::Pqtm(command), Message::PqtmReply(reply)) if reply.command == command => {
                match reply.error {
                    None => Outcome::Done,
                    Some(code) => Outcome::Rejected(code),
                }
            }
            _ => Outcome::Unrelated,
        }
    }
}

/// A sentence to send to the module.
///
/// Settings changed here are volatile until [`Command::SaveSettings`] (and, for PQTM settings,
/// [`Command::SavePqtm`]). Note `PAIR513`: above 1 Hz the specification requires powering the GNSS
/// system off with `$PAIR382,1*2E` and `$PAIR003*39` before saving and on again with `$PAIR002*38`
/// afterwards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command<'a> {
    /// `$PQTMVERNO`; answered by [`Message::Version`].
    QueryVersion,
    /// `$PAIR050,<ms>`: position fix interval, 100 to 1000.
    SetFixInterval {
        /// Interval in milliseconds.
        ms: u16,
    },
    /// `$PAIR062,<type>,<rate>`: output one sentence type every `rate` fixes, 0 = off.
    SetSentenceRate {
        /// Sentence type.
        sentence: SentenceType,
        /// Rate, 0 to 20.
        rate: u8,
    },
    /// `$PAIR062,-1`: reset all sentence rates to the module's defaults. The specification gives no rate
    /// parameter for this form; LC76GABNR12A05S acknowledges it without one and rejects `$PAIR062,-1,0`
    /// and `$PAIR062,-1,1` with result 4 (parameter error).
    ResetSentenceRates,
    /// `$PAIR080,<mode>`.
    SetNavMode(NavMode),
    /// `$PAIR752,<type>,<width>`.
    SetPps(Pps),
    /// `$PAIR004` to `$PAIR007`: restart the GNSS engine. The module may stay quiet for a while.
    Start(StartKind),
    /// `$PQTMCFGMSGRATE,W,PQTMEPE,<0|1>,2`: `$PQTMEPE` output on or off.
    SetEpe(bool),
    /// `$PAIR513`: save the current settings to the module's flash.
    SaveSettings,
    /// `$PQTMSAVEPAR`: save the PQTM settings (EPE output) to the module's NVDM.
    SavePqtm,
    /// Any sentence body (without `$`, `*`, checksum or CR LF), for example `"PAIR051"`. The
    /// acknowledgement is inferred from a `PAIR<id>` or known `PQTM` name; for anything else no reply
    /// is waited for.
    Raw(&'a str),
}

impl Command<'_> {
    /// The reply that ends the command, `None` if none is known.
    pub fn reply(&self) -> Option<CommandId> {
        Some(match self {
            Self::QueryVersion => CommandId::Pqtm(PqtmCommand::VerNo),
            Self::SetFixInterval { .. } => CommandId::Pair(50),
            Self::SetSentenceRate { .. } | Self::ResetSentenceRates => CommandId::Pair(62),
            Self::SetNavMode(_) => CommandId::Pair(80),
            Self::SetPps(_) => CommandId::Pair(752),
            Self::Start(StartKind::Hot) => CommandId::Pair(4),
            Self::Start(StartKind::Warm) => CommandId::Pair(5),
            Self::Start(StartKind::Cold) => CommandId::Pair(6),
            Self::Start(StartKind::FullCold) => CommandId::Pair(7),
            Self::SetEpe(_) => CommandId::Pqtm(PqtmCommand::CfgMsgRate),
            Self::SaveSettings => CommandId::Pair(513),
            Self::SavePqtm => CommandId::Pqtm(PqtmCommand::SavePar),
            Self::Raw(body) => return raw_reply(body),
        })
    }

    /// Writes `$<body>*<CS>\r\n` into `buf` and returns its length. `None` if `buf` is too small
    /// ([`MAX_COMMAND_LEN`] always suffices) or a [`Command::Raw`] body is empty, longer than
    /// [`MAX_SENTENCE_LEN`] or contains `$`, `*`, a control character or non-ASCII.
    pub fn write(&self, buf: &mut [u8]) -> Option<usize> {
        let mut out = Out { buf, len: 0 };
        out.byte(b'$')?;
        match self {
            Self::QueryVersion => out.str("PQTMVERNO")?,
            Self::SetFixInterval { ms } => {
                out.str("PAIR050,")?;
                out.uint(u32::from(*ms))?;
            }
            Self::SetSentenceRate { sentence, rate } => {
                out.str("PAIR062,")?;
                out.uint(u32::from(sentence.code()))?;
                out.byte(b',')?;
                out.uint(u32::from(*rate))?;
            }
            Self::ResetSentenceRates => out.str("PAIR062,-1")?,
            Self::SetNavMode(mode) => {
                out.str("PAIR080,")?;
                out.uint(u32::from(mode.code()))?;
            }
            Self::SetPps(pps) => {
                out.str("PAIR752,")?;
                out.uint(u32::from(pps.kind.code()))?;
                out.byte(b',')?;
                out.uint(u32::from(pps.width_ms))?;
            }
            Self::Start(kind) => out.str(match kind {
                StartKind::Hot => "PAIR004",
                StartKind::Warm => "PAIR005",
                StartKind::Cold => "PAIR006",
                StartKind::FullCold => "PAIR007",
            })?,
            Self::SetEpe(on) => out.str(if *on {
                "PQTMCFGMSGRATE,W,PQTMEPE,1,2"
            } else {
                "PQTMCFGMSGRATE,W,PQTMEPE,0,2"
            })?,
            Self::SaveSettings => out.str("PAIR513")?,
            Self::SavePqtm => out.str("PQTMSAVEPAR")?,
            Self::Raw(body) => {
                let usable = !body.is_empty()
                    && body.len() + 4 <= MAX_SENTENCE_LEN
                    && body
                        .bytes()
                        .all(|b| (0x20..0x7F).contains(&b) && b != b'$' && b != b'*');
                if !usable {
                    return None;
                }
                out.str(body)?;
            }
        }
        let sum = checksum(&out.buf[1..out.len]);
        out.byte(b'*')?;
        out.byte(HEX[usize::from(sum >> 4)])?;
        out.byte(HEX[usize::from(sum & 15)])?;
        out.str("\r\n")?;
        Some(out.len)
    }
}

const HEX: &[u8; 16] = b"0123456789ABCDEF";

fn raw_reply(body: &str) -> Option<CommandId> {
    let b = body.as_bytes();
    if let Some(id) = b.strip_prefix(b"PAIR")
        && id.len() >= 3
        && id[..3].iter().all(u8::is_ascii_digit)
        && id.get(3).is_none_or(|c| *c == b',')
    {
        let id = id[..3]
            .iter()
            .fold(0u16, |acc, d| acc * 10 + u16::from(d - b'0'));
        return Some(CommandId::Pair(id));
    }
    let name = body.split(',').next()?;
    match name {
        "PQTMCFGMSGRATE" => Some(CommandId::Pqtm(PqtmCommand::CfgMsgRate)),
        "PQTMSAVEPAR" => Some(CommandId::Pqtm(PqtmCommand::SavePar)),
        "PQTMVERNO" => Some(CommandId::Pqtm(PqtmCommand::VerNo)),
        _ => None,
    }
}

struct Out<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Out<'_> {
    fn byte(&mut self, b: u8) -> Option<()> {
        *self.buf.get_mut(self.len)? = b;
        self.len += 1;
        Some(())
    }

    fn str(&mut self, s: &str) -> Option<()> {
        let dst = self.buf.get_mut(self.len..self.len + s.len())?;
        dst.copy_from_slice(s.as_bytes());
        self.len += s.len();
        Some(())
    }

    fn uint(&mut self, v: u32) -> Option<()> {
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut v = v;
        loop {
            digits[n] = b'0' + (v % 10) as u8;
            n += 1;
            v /= 10;
            if v == 0 {
                break;
            }
        }
        while n > 0 {
            n -= 1;
            self.byte(digits[n])?;
        }
        Some(())
    }
}
