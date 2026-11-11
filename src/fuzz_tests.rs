//! Robustness properties: arbitrary bytes from the module never panic anything. Every result is an
//! `Ok`/`Err` or `Some`/`None`; nothing here asserts on values.

use embedded_io::ErrorKind;
use proptest::prelude::*;

use crate::aux_sentences::{Zda, gns, grs, gst, zda};
use crate::command::{MAX_COMMAND_LEN, StartKind};
use crate::config::{NavMode, Pps, PpsType, SentenceType};
use crate::nmea::checksum;
use crate::sentence::{Date, Message, Talker, Time};
use crate::*;

const CASES: u32 = 2500;

const TALKERS: [Talker; 6] = [
    Talker::Gp,
    Talker::Gl,
    Talker::Ga,
    Talker::Gb,
    Talker::Gq,
    Talker::Gn,
];

fn frame_bytes(body: &[u8]) -> Vec<u8> {
    let mut v = vec![b'$'];
    v.extend_from_slice(body);
    v.extend_from_slice(format!("*{:02X}\r\n", checksum(body)).as_bytes());
    v
}

fn exercise(message: &Message, builder: &mut FixBuilder) {
    if let Some(fix) = builder.push(message) {
        let _ = (
            fix.lat_deg(),
            fix.lon_deg(),
            fix.alt_m(),
            fix.speed_m_s(),
            fix.course_deg(),
        );
    }
    if let Message::Zda(z) = message {
        let _ = (z.utc_unix_ms(), z.local_offset_minutes());
    }
    if let Message::Grs(g) = message {
        let _ = g.residual_count();
    }
    if let Message::Version(v) = message {
        let _ = v.version();
    }
}

fn feed_framer(bytes: &[u8]) {
    let mut framer = Framer::new();
    let mut builder = FixBuilder::new();
    for &b in bytes {
        if let Some(sentence) = framer.push(b) {
            let _ = (
                sentence.as_str(),
                sentence.address(),
                sentence.fields().count(),
            );
            let message = sentence.parse();
            exercise(&message, &mut builder);
        }
    }
    let _ = framer.stats();
}

/// Byte source for the driver: hands out `data` in chunks of the given sizes; the write side takes
/// `write_chunks` sizes and optionally claims more than it was given.
struct Chunked {
    data: Vec<u8>,
    pos: usize,
    chunks: Vec<usize>,
    next: usize,
    lie_read: bool,
    lie_write: bool,
    writes: usize,
}

impl Chunked {
    fn new(data: Vec<u8>, chunks: Vec<usize>, lie_read: bool, lie_write: bool) -> Self {
        Self {
            data,
            pos: 0,
            chunks,
            next: 0,
            lie_read,
            lie_write,
            writes: 0,
        }
    }

    fn take(&mut self, buf: &mut [u8]) -> usize {
        let size = if self.chunks.is_empty() {
            buf.len()
        } else {
            let c = self.chunks[self.next % self.chunks.len()];
            self.next += 1;
            c
        };
        let rest = &self.data[self.pos..];
        let n = size.min(rest.len()).min(buf.len());
        buf[..n].copy_from_slice(&rest[..n]);
        self.pos += n;
        if self.lie_read && n > 0 { n + 1000 } else { n }
    }

    fn put(&mut self, buf: &[u8]) -> usize {
        self.writes += 1;
        if self.writes > 400 {
            return 0;
        }
        if self.lie_write {
            buf.len() + 7
        } else {
            buf.len().div_ceil(2)
        }
    }
}

impl embedded_io::ErrorType for Chunked {
    type Error = ErrorKind;
}

impl embedded_io::Read for Chunked {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        Ok(self.take(buf))
    }
}

impl embedded_io::Write for Chunked {
    fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        Ok(self.put(buf))
    }

    fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

fn field_text() -> impl Strategy<Value = String> {
    prop_oneof![
        Just(String::new()),
        "[0-9]{0,12}",
        "[0-9]{0,8}\\.[0-9]{0,12}",
        "[-+]?[0-9.]{0,16}",
        "[0-9.+\\-NSEWMADFRVXKT,*]{0,45}",
        "[ -~]{0,150}",
        any::<String>(),
    ]
}

const SAMPLES: [&str; 13] = [
    "GNRMC,040143.000,A,3149.334166,N,11706.941670,E,0.01,0.00,010522,,,D,V",
    "GNGGA,040143.000,3149.334166,N,11706.941670,E,2,36,0.48,61.496,M,-0.335,M,,",
    "GNGSA,A,3,195,01,194,21,07,30,199,08,14,17,27,,0.71,0.48,0.52,1",
    "GPGSV,3,1,12,195,72,076,42,01,69,158,45,194,66,111,29,21,61,060,44,1",
    "GNVTG,0.00,T,,M,0.01,N,0.02,K,D",
    "GNGLL,3149.334166,N,11706.941670,E,040143.000,A,D",
    "GNZDA,055054.000,19,09,2022,,",
    "GNGNS,053106.000,3149.334190,N,11706.948654,E,DANN,16,0.63,51.287,M,-0.335,M,,,V",
    "GNGST,123624.000,6.3,2.5,2.4,88.4,2.4,2.5,9.2",
    "GNGRS,125524.000,1,-0.4,-0.7,0.5,-4.6,0.2,1.1,-2.2,-0.6,-1.1,9.2,-2.1,3.1,1,1",
    "PAIR001,004,0",
    "PQTMEPE,2,1.000,1.000,1.000,1.414,1.732",
    "PQTMVERNO,LC76GABNR02A01S,2022/09/14,11:47:03",
];

fn multibyte_text() -> impl Strategy<Value = String> {
    proptest::collection::vec(
        proptest::sample::select(vec!['0', '1', '5', '9', '.', '-', 'é', '€', '\u{1F600}']),
        1..=12,
    )
    .prop_map(|chars| chars.into_iter().collect())
}

fn address_text() -> impl Strategy<Value = String> {
    let known = [
        "GPGGA",
        "GNGGA",
        "GNRMC",
        "GPGSA",
        "GPGSV",
        "GLGSV",
        "GNVTG",
        "GNGLL",
        "GNZDA",
        "GNGNS",
        "GNGST",
        "GNGRS",
        "GNRLM",
        "PAIR001",
        "PQTMVERNO",
        "PQTMEPE",
        "PQTMCFGMSGRATE",
        "PQTMSAVEPAR",
        "PAIR",
        "G",
        "GN",
        "GNG",
        "",
    ];
    prop_oneof![
        proptest::sample::select(known.to_vec()).prop_map(String::from),
        "[A-Z0-9]{0,16}",
    ]
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(CASES))]

    // (a) framer: arbitrary bytes, including long runs without CR/LF and many `$`.
    #[test]
    fn framer_arbitrary_bytes(bytes in proptest::collection::vec(any::<u8>(), 0..700)) {
        feed_framer(&bytes);
    }

    #[test]
    fn framer_dollar_heavy_bytes(
        bytes in proptest::collection::vec(prop_oneof![
            Just(b'$'), Just(b'*'), Just(b','), Just(b'\r'), Just(b'\n'), 0x20u8..0x7F, any::<u8>()
        ], 0..900)
    ) {
        feed_framer(&bytes);
    }

    #[test]
    fn sentence_new_arbitrary_text(text in any::<String>(), tail in "[\r\n]{0,3}") {
        let line = format!("{text}{tail}");
        if let Some(s) = Sentence::new(&line) {
            let _ = (s.address(), s.fields().count(), s.parse());
        }
    }

    // (a) through the drivers: any chunking, and a UART that lies about byte counts.
    #[test]
    fn driver_arbitrary_bytes_arbitrary_chunks(
        bytes in proptest::collection::vec(any::<u8>(), 0..900),
        chunks in proptest::collection::vec(0usize..200, 0..8),
        lie_read in any::<bool>(),
    ) {
        let mut gnss = blocking::Lc76g::new(Chunked::new(bytes, chunks, lie_read, false));
        for _ in 0..200 {
            if gnss.read().is_err() {
                break;
            }
        }
        let _ = gnss.stats();
    }

    #[test]
    fn driver_framed_sentences_read_fix_and_read_sentence(
        bodies in proptest::collection::vec(
            (address_text(), proptest::collection::vec(field_text(), 0..30)), 0..12),
        chunks in proptest::collection::vec(1usize..90, 0..6),
        which in 0u8..3,
    ) {
        let mut data = Vec::new();
        for (address, fields) in &bodies {
            let mut body = address.clone().into_bytes();
            for f in fields {
                body.push(b',');
                body.extend_from_slice(f.as_bytes());
            }
            data.extend(frame_bytes(&body));
        }
        let mut gnss = blocking::Lc76g::new(Chunked::new(data, chunks, false, false));
        for _ in 0..100 {
            let stop = match which {
                0 => gnss.read_fix().is_err(),
                1 => gnss.read_sentence().map(|s| s.parse()).is_err(),
                _ => gnss.read().is_err(),
            };
            if stop {
                break;
            }
        }
    }

    // Command layer: arbitrary bodies, arbitrary replies, writers that under- and over-report.
    #[test]
    fn driver_commands_against_arbitrary_replies(
        reply in proptest::collection::vec(any::<u8>(), 0..600),
        body in any::<String>(),
        chunks in proptest::collection::vec(0usize..100, 0..6),
        lie_write in any::<bool>(),
        which in 0u8..5,
    ) {
        let mut gnss = blocking::Lc76g::new(Chunked::new(reply, chunks, false, lie_write));
        let _ = match which {
            0 => gnss.command(&Command::Raw(&body)).map(|_| ()),
            1 => gnss.version().map(|_| ()),
            2 => gnss.init(&Config::default()).map(|_| ()),
            3 => gnss.save_settings(),
            _ => gnss.command(&Command::QueryVersion),
        };
    }

    #[test]
    fn command_write_any_buffer(
        body in any::<String>(),
        size in 0usize..(MAX_COMMAND_LEN + 8),
        ms in any::<u16>(),
        rate in any::<u8>(),
        width in any::<u16>(),
    ) {
        let mut buf = vec![0u8; size];
        let commands = [
            Command::Raw(&body),
            Command::QueryVersion,
            Command::SetFixInterval { ms },
            Command::SetSentenceRate { sentence: SentenceType::Gga, rate },
            Command::ResetSentenceRates,
            Command::SetNavMode(NavMode::Normal),
            Command::SetPps(Pps { kind: PpsType::Always, width_ms: width }),
            Command::Start(StartKind::Hot),
            Command::SetEpe(true),
            Command::SaveSettings,
            Command::SavePqtm,
        ];
        for c in &commands {
            let _ = c.reply();
            if let Some(n) = c.write(&mut buf) {
                prop_assert!(n <= buf.len());
            }
        }
    }

    #[test]
    fn raw_reply_prefixes(tail in "[0-9A-Z,]{0,8}", cut in 0usize..5) {
        let body = format!("PAIR{tail}");
        let body = body.get(..(4 + cut).min(body.len())).unwrap_or("");
        let _ = Command::Raw(body).reply();
    }

    // (c) valid frames around random fields, random field counts including 0 and beyond the
    // decoder's field table, empty / over-long / non-ASCII fields.
    #[test]
    fn framed_random_fields_through_framer(
        address in address_text(),
        fields in proptest::collection::vec(field_text(), 0..40),
        crlf in any::<bool>(),
    ) {
        let mut body = address.into_bytes();
        for f in &fields {
            body.push(b',');
            body.extend_from_slice(f.as_bytes());
        }
        let mut line = frame_bytes(&body);
        if !crlf {
            line.truncate(line.len() - 2);
        }
        feed_framer(&line);
        if let Ok(text) = core::str::from_utf8(&line)
            && let Some(s) = Sentence::new(text)
        {
            let _ = s.parse();
        }
    }

    // (b) decoders and field parsers called directly, including text the framer would refuse.
    #[test]
    fn new_decoders_arbitrary_fields(
        talker in 0usize..6,
        fields in proptest::collection::vec(field_text(), 0..30),
    ) {
        let refs: Vec<&str> = fields.iter().map(String::as_str).collect();
        let t = TALKERS[talker];
        for result in [zda(t, &refs), gns(t, &refs), gst(t, &refs), grs(t, &refs)] {
            if let Ok(Message::Zda(z)) = result {
                let _ = (z.utc_unix_ms(), z.local_offset_minutes());
            }
            if let Ok(Message::Gns(g)) = result
                && let Some(m) = g.modes
            {
                let _ = (m.gps(), m.glonass(), m.galileo(), m.bds(), m.qzss(), m.navic(), m.reported());
            }
            if let Ok(Message::Grs(g)) = result {
                let _ = g.residual_count();
            }
        }
    }

    #[test]
    fn field_parsers_arbitrary_text(a in field_text(), b in field_text(), num in any::<u64>(), den in 1u64..10_000) {
        use crate::field::*;
        let _ = (uint::<u8>(&a), uint::<u16>(&a), uint::<u32>(&a), uint_req::<u8>(&a));
        let _ = (scaled::<i32>(&a, num, den), scaled::<u32>(&a, num, den), scaled::<u16>(&a, num, den));
        let _ = (metres_mm(&a), centi_u16(&a), knots_mm_s(&a), kmh_mm_s(&a), course_cdeg(&a));
        let _ = (latitude(&a, &b), longitude(&a, &b), time(&a), date(&a));
    }

    #[test]
    fn decoded_values_convert_without_panic(
        e7 in any::<i32>(), mm in any::<i32>(), mm_s in any::<u32>(), cd in any::<u16>(), d in any::<u16>(),
        year in any::<u16>(), month in any::<u8>(), day in any::<u8>(),
        hour in any::<u8>(), minute in any::<u8>(), second in any::<u8>(), millis in any::<u16>(),
        lh in any::<i8>(), lm in any::<u8>(),
    ) {
        let _ = (units::degrees(e7), units::metres(mm), units::metres_per_second(mm_s),
                 units::course_degrees(cd), units::dop(d));
        let z = Zda {
            talker: Talker::Gn,
            time: Some(Time { hour, minute, second, millis }),
            date: Some(Date { year, month, day }),
            local_hour: Some(lh),
            local_min: Some(lm),
        };
        let _ = (z.utc_unix_ms(), z.local_offset_minutes());
    }

    // Text whose multi-byte characters straddle the offsets the parsers cut at.
    #[test]
    fn multibyte_text_in_every_field_position(
        a in multibyte_text(),
        b in multibyte_text(),
        sample in 0usize..13,
        index in 0usize..20,
        all_fields in any::<bool>(),
    ) {
        use crate::field::*;
        let _ = (uint::<u8>(&a), uint_req::<u16>(&a), scaled::<i32>(&a, 1000, 1), metres_mm(&a),
                 centi_u16(&a), knots_mm_s(&a), kmh_mm_s(&a), course_cdeg(&a), time(&a), date(&a),
                 latitude(&a, "N"), latitude(&a, &b), longitude(&a, "E"), longitude(&a, &b));
        let mut fields: Vec<&str> = SAMPLES[sample].split(',').collect();
        let address = fields.remove(0);
        if all_fields {
            for (i, f) in fields.iter_mut().enumerate() {
                *f = if i % 2 == 0 { &a } else { &b };
            }
        } else if let Some(slot) = fields.get_mut(index) {
            *slot = &a;
        }
        let _ = crate::sentence::decode_fields(address, &fields);
        let mut body = address.to_string();
        for f in &fields {
            body.push(',');
            body.push_str(f);
        }
        let line = format!("${body}*{:02X}", checksum(body.as_bytes()));
        if let Some(sentence) = Sentence::new(&line) {
            let _ = sentence.parse();
        }
    }
}

#[test]
fn regression_multibyte_cut_offsets() {
    use crate::field::*;
    for text in ["ꬨA®", "€€", "€€.000", "é1é1é1", "1é2.5", "12é1.5", "€€€"] {
        let _ = (
            time(text),
            date(text),
            latitude(text, "N"),
            longitude(text, "E"),
        );
    }
    assert!(latitude("1é2.5", "N").is_err());
    assert!(longitude("12é1.5", "E").is_err());
    assert!(date("€€").is_err());
    assert!(time("€€").is_err());
    assert!(time("ꬨA®").is_err());
    assert!(date("ꬨA®").is_err());
}
