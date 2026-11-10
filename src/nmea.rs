//! NMEA 0183 framing: checksum, [`Framer`] (bytes in, validated [`Sentence`]s out) and the borrowed
//! [`Sentence`] view. No UART, no allocation.

use crate::sentence::Message;

/// Longest sentence the [`Framer`] holds, `$` to the last checksum digit, in bytes.
///
/// Section 2.1 of the protocol specification defines the layout but gives no length limit; NMEA 0183
/// itself allows 82 characters including the `$` and CR LF. The longest sentences this crate decodes
/// are RMC and GGA with seven-decimal coordinates in drone mode (about 85 bytes) and GSA with twelve
/// three-digit satellite IDs (about 85 bytes). 128 leaves about 40 % margin over that and still costs
/// only 128 bytes of RAM. Longer input is counted as an overflow and skipped.
pub const MAX_SENTENCE_LEN: usize = 128;

/// XOR of all bytes of `body` (the characters between `$` and `*`), as in section 2.1.
pub fn checksum(body: &[u8]) -> u8 {
    body.iter().fold(0, |acc, b| acc ^ b)
}

fn hex(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

/// True when `text` is `$` + printable ASCII + `*` + two hex digits with a matching checksum.
fn is_valid(text: &[u8]) -> bool {
    let n = text.len();
    if n < 4 || text[0] != b'$' || text[n - 3] != b'*' {
        return false;
    }
    if !text.iter().all(|b| (0x20..0x7F).contains(b)) {
        return false;
    }
    match (hex(text[n - 2]), hex(text[n - 1])) {
        (Some(hi), Some(lo)) => checksum(&text[1..n - 3]) == (hi << 4 | lo),
        _ => false,
    }
}

/// Counters kept by a [`Framer`]. They wrap-free saturate at `u32::MAX`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FramerStats {
    /// Sentences returned (complete, checksum valid).
    pub sentences: u32,
    /// Sentences that ended, or were cut off by the next `$`, without a valid checksum.
    pub checksum_errors: u32,
    /// Sentences longer than [`MAX_SENTENCE_LEN`].
    pub overflows: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    /// Waiting for `$`.
    Idle,
    /// Collecting a sentence.
    Body,
    /// Over-long sentence: ignore everything up to the next `$` or line end.
    Skip,
}

/// Incremental NMEA framer. Feed it bytes in any chunking; it hands back each sentence whose checksum
/// is valid and resynchronises on `$` after garbage, a corrupted sentence or an over-long line.
///
/// A line ends at CR or LF, so the LF of a CR LF pair is simply ignored while waiting for `$`.
pub struct Framer {
    buf: [u8; MAX_SENTENCE_LEN],
    len: usize,
    state: State,
    stats: FramerStats,
}

impl Framer {
    /// Empty framer waiting for `$`.
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_SENTENCE_LEN],
            len: 0,
            state: State::Idle,
            stats: FramerStats {
                sentences: 0,
                checksum_errors: 0,
                overflows: 0,
            },
        }
    }

    /// Counters since construction.
    pub const fn stats(&self) -> FramerStats {
        self.stats
    }

    /// Feeds one byte. Returns the sentence when this byte completed a valid one; it stays valid until
    /// the next call.
    pub fn push(&mut self, byte: u8) -> Option<Sentence<'_>> {
        if self.advance(byte) {
            Some(self.ready())
        } else {
            None
        }
    }

    /// Like [`Framer::push`] but returns only whether a sentence is ready, to be read with
    /// [`Framer::ready`]. Lets a caller loop without holding a borrow.
    pub(crate) fn advance(&mut self, byte: u8) -> bool {
        match (self.state, byte) {
            (state, b'$') => {
                if state == State::Body {
                    self.stats.checksum_errors = self.stats.checksum_errors.saturating_add(1);
                }
                self.buf[0] = b'$';
                self.len = 1;
                self.state = State::Body;
                false
            }
            (State::Idle, _) => false,
            (State::Skip, b'\r' | b'\n') => {
                self.state = State::Idle;
                false
            }
            (State::Skip, _) => false,
            (State::Body, b'\r' | b'\n') => {
                self.state = State::Idle;
                if is_valid(&self.buf[..self.len]) {
                    self.stats.sentences = self.stats.sentences.saturating_add(1);
                    true
                } else {
                    self.stats.checksum_errors = self.stats.checksum_errors.saturating_add(1);
                    false
                }
            }
            (State::Body, _) => {
                if self.len == MAX_SENTENCE_LEN {
                    self.stats.overflows = self.stats.overflows.saturating_add(1);
                    self.state = State::Skip;
                } else {
                    self.buf[self.len] = byte;
                    self.len += 1;
                }
                false
            }
        }
    }

    /// The sentence completed by the last [`Framer::advance`] that returned true.
    pub(crate) fn ready(&self) -> Sentence<'_> {
        Sentence {
            text: core::str::from_utf8(&self.buf[..self.len]).unwrap_or(""),
        }
    }
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

/// One validated NMEA sentence: `$` ... `*` and two checksum digits, without CR LF.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sentence<'a> {
    text: &'a str,
}

impl<'a> Sentence<'a> {
    /// Validates a line obtained elsewhere (`$...*CS`, an optional trailing CR LF is ignored).
    /// Returns `None` if the checksum is wrong or the line is malformed.
    pub fn new(line: &'a str) -> Option<Self> {
        let text = line.trim_end_matches(['\r', '\n']);
        if text.len() <= MAX_SENTENCE_LEN && is_valid(text.as_bytes()) {
            Some(Self { text })
        } else {
            None
        }
    }

    /// The whole sentence from `$` to the checksum, without CR LF. This is what a logger writes raw.
    pub const fn as_str(&self) -> &'a str {
        self.text
    }

    /// Same as [`Sentence::as_str`] as bytes.
    pub const fn as_bytes(&self) -> &'a [u8] {
        self.text.as_bytes()
    }

    /// Characters between `$` and `*`.
    fn body(&self) -> &'a str {
        &self.text[1..self.text.len() - 3]
    }

    /// Address field: talker and formatter (`"GNGGA"`) or proprietary name (`"PAIR001"`,
    /// `"PQTMVERNO"`).
    pub fn address(&self) -> &'a str {
        let body = self.body();
        body.split_once(',').map_or(body, |(address, _)| address)
    }

    /// Data fields after the address, in order. Empty fields are yielded as `""`.
    pub fn fields(&self) -> Fields<'a> {
        Fields {
            rest: self.body().split_once(',').map(|(_, rest)| rest),
        }
    }

    /// Decodes the sentence. See [`Message`].
    pub fn parse(&self) -> Message {
        crate::sentence::decode(self)
    }
}

/// Iterator over the data fields of a [`Sentence`].
#[derive(Clone, Debug)]
pub struct Fields<'a> {
    rest: Option<&'a str>,
}

impl<'a> Iterator for Fields<'a> {
    type Item = &'a str;

    fn next(&mut self) -> Option<&'a str> {
        let rest = self.rest?;
        match rest.split_once(',') {
            Some((head, tail)) => {
                self.rest = Some(tail);
                Some(head)
            }
            None => {
                self.rest = None;
                Some(rest)
            }
        }
    }
}
