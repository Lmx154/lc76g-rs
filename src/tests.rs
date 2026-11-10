use std::cell::RefCell;
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use embedded_io::ErrorKind;

use crate::command::{MAX_COMMAND_LEN, StartKind};
use crate::config::PPS_WIDTH_MS;
use crate::nmea::{MAX_SENTENCE_LEN, checksum};
use crate::sentence::*;
use crate::*;

// ---- spec example lines (independent vectors) ----

const SPEC_RMC: &str = "$GNRMC,040143.000,A,3149.334166,N,11706.941670,E,0.01,0.00,010522,,,D,V*0E";
const SPEC_GGA: &str =
    "$GNGGA,040143.000,3149.334166,N,11706.941670,E,2,36,0.48,61.496,M,-0.335,M,,*58";
const SPEC_GSA: &str = "$GNGSA,A,3,195,01,194,21,07,30,199,08,14,17,27,,0.71,0.48,0.52,1*34";
const SPEC_GSA_BDS: &str = "$GNGSA,A,3,13,39,28,,,,,,,,,,0.71,0.48,0.52,4*0B";
const SPEC_GSV: &str = "$GPGSV,3,1,12,195,72,076,42,01,69,158,45,194,66,111,29,21,61,060,44,1*6D";
const SPEC_GSV_SHORT: &str = "$GLGSV,2,2,05,88,07,034,29,1*46";
const SPEC_GSV_NO_CN0: &str =
    "$GBGSV,4,2,16,10,59,321,,03,51,192,36,36,41,314,38,02,37,229,32,1*71";
const SPEC_VTG: &str = "$GNVTG,0.00,T,,M,0.01,N,0.02,K,D*25";
const SPEC_GLL: &str = "$GNGLL,3149.334166,N,11706.941670,E,040143.000,A,D*46";
const SPEC_ZDA: &str = "$GNZDA,055054.000,19,09,2022,,*4A";
const SPEC_GNS: &str =
    "$GNGNS,053106.000,3149.334190,N,11706.948654,E,DANN,16,0.63,51.287,M,-0.335,M,,,V*05";
const SPEC_ACK: &str = "$PAIR001,004,0*3F";
const SPEC_EPE: &str = "$PQTMEPE,2,1.000,1.000,1.000,1.414,1.732*52";
const SPEC_VERNO: &str = "$PQTMVERNO,LC76GABNR02A01S,2022/09/14,11:47:03*3D";
const SPEC_CFG_OK: &str = "$PQTMCFGMSGRATE,OK*29";

/// `$body*CS` with the checksum computed here.
fn frame(body: &str) -> String {
    format!("${body}*{:02X}", checksum(body.as_bytes()))
}

fn parse(line: &str) -> Message {
    Sentence::new(line).expect("valid sentence").parse()
}

fn parse_body(body: &str) -> Message {
    parse(&frame(body))
}

fn lines(items: &[&str]) -> Vec<u8> {
    items
        .iter()
        .flat_map(|l| format!("{l}\r\n").into_bytes())
        .collect()
}

fn run<F: Future>(f: F) -> F::Output {
    let mut f = pin!(f);
    match f.as_mut().poll(&mut Context::from_waker(Waker::noop())) {
        Poll::Ready(v) => v,
        Poll::Pending => panic!("future pended"),
    }
}

// ---- checksum, commands ----

#[test]
fn spec_example_lines_have_valid_checksums() {
    for line in [
        SPEC_RMC,
        SPEC_GGA,
        SPEC_GSA,
        SPEC_GSA_BDS,
        SPEC_GSV,
        SPEC_GSV_SHORT,
        SPEC_GSV_NO_CN0,
        SPEC_VTG,
        SPEC_GLL,
        SPEC_ZDA,
        SPEC_GNS,
        SPEC_ACK,
        SPEC_EPE,
        SPEC_VERNO,
        SPEC_CFG_OK,
        "$PAIR001,752,0*3B",
        "$PQTMSAVEPAR,OK*72",
        "$PQTMCFGMSGRATE,ERROR,1*68",
    ] {
        assert!(Sentence::new(line).is_some(), "{line}");
        assert!(Sentence::new(&line.replace('*', "*0")).is_none());
    }
    assert!(Sentence::new("$GNGGA,1*00").is_none());
    assert!(Sentence::new("GNGGA,1*00").is_none());
}

fn encode(command: Command<'_>) -> String {
    let mut buf = [0; MAX_COMMAND_LEN];
    let n = command.write(&mut buf).expect("fits");
    String::from_utf8(buf[..n].to_vec()).unwrap()
}

#[test]
fn commands_encode_to_spec_vectors() {
    let cases = [
        (Command::QueryVersion, "$PQTMVERNO*58\r\n"),
        (Command::SetFixInterval { ms: 1000 }, "$PAIR050,1000*12\r\n"),
        (
            Command::SetSentenceRate {
                sentence: SentenceType::Gga,
                rate: 3,
            },
            "$PAIR062,0,3*3D\r\n",
        ),
        (Command::ResetSentenceRates, "$PAIR062,-1*0E\r\n"),
        (Command::SetNavMode(NavMode::Fitness), "$PAIR080,1*2F\r\n"),
        (
            Command::SetPps(Pps {
                kind: PpsType::Always,
                width_ms: 100,
            }),
            "$PAIR752,4,100*3F\r\n",
        ),
        (Command::Start(StartKind::Hot), "$PAIR004*3E\r\n"),
        (Command::Start(StartKind::Warm), "$PAIR005*3F\r\n"),
        (Command::Start(StartKind::Cold), "$PAIR006*3C\r\n"),
        (Command::Start(StartKind::FullCold), "$PAIR007*3D\r\n"),
        (
            Command::SetEpe(true),
            "$PQTMCFGMSGRATE,W,PQTMEPE,1,2*1D\r\n",
        ),
        (
            Command::SetEpe(false),
            "$PQTMCFGMSGRATE,W,PQTMEPE,0,2*1C\r\n",
        ),
        (Command::SaveSettings, "$PAIR513*3D\r\n"),
        (Command::SavePqtm, "$PQTMSAVEPAR*5A\r\n"),
        (Command::Raw("PAIR051"), "$PAIR051*3E\r\n"),
    ];
    for (command, expected) in cases {
        assert_eq!(encode(command), expected);
    }
}

#[test]
fn command_write_rejects_small_buffers_and_bad_raw_bodies() {
    assert_eq!(Command::QueryVersion.write(&mut [0; 14]), None);
    assert_eq!(Command::QueryVersion.write(&mut [0; 15]), Some(15));
    let long = "A".repeat(MAX_SENTENCE_LEN);
    for body in ["", "A*B", "A$B", "A\nB", "é", long.as_str()] {
        assert_eq!(
            Command::Raw(body).write(&mut [0; MAX_COMMAND_LEN]),
            None,
            "{body:?}"
        );
    }
    let longest = "A".repeat(MAX_SENTENCE_LEN - 4);
    assert!(
        Command::Raw(&longest)
            .write(&mut [0; MAX_COMMAND_LEN])
            .is_some()
    );
}

#[test]
fn commands_know_their_reply() {
    use crate::sentence::PqtmCommand::*;
    assert_eq!(
        Command::Start(StartKind::Warm).reply(),
        Some(CommandId::Pair(5))
    );
    assert_eq!(Command::SaveSettings.reply(), Some(CommandId::Pair(513)));
    assert_eq!(
        Command::SetEpe(true).reply(),
        Some(CommandId::Pqtm(CfgMsgRate))
    );
    assert_eq!(Command::QueryVersion.reply(), Some(CommandId::Pqtm(VerNo)));
    assert_eq!(Command::Raw("PAIR051").reply(), Some(CommandId::Pair(51)));
    assert_eq!(
        Command::Raw("PQTMSAVEPAR").reply(),
        Some(CommandId::Pqtm(SavePar))
    );
    assert_eq!(Command::Raw("PAIR12").reply(), None);
    assert_eq!(Command::Raw("PMTK1").reply(), None);
}

// ---- framer ----

fn feed(framer: &mut nmea::Framer, bytes: &[u8]) -> Vec<String> {
    bytes
        .iter()
        .filter_map(|b| framer.push(*b).map(|s| s.as_str().to_owned()))
        .collect()
}

#[test]
fn framer_returns_valid_sentences_and_rejects_corrupted_ones() {
    let mut framer = nmea::Framer::new();
    let bad = SPEC_GGA.replace("61.496", "61.497");
    let stream = lines(&[SPEC_RMC, &bad, SPEC_VTG]);
    assert_eq!(feed(&mut framer, &stream), [SPEC_RMC, SPEC_VTG]);
    let s = framer.stats();
    assert_eq!((s.sentences, s.checksum_errors, s.overflows), (2, 1, 0));
}

#[test]
fn framer_resynchronises_after_garbage_and_cut_off_sentences() {
    let mut framer = nmea::Framer::new();
    let mut stream = b"\x00\xff junk *58\r\n$GNGG".to_vec();
    stream.extend(lines(&[SPEC_GLL]));
    stream.extend(b"$$$\r\n");
    stream.extend(lines(&[SPEC_ACK]));
    assert_eq!(feed(&mut framer, &stream), [SPEC_GLL, SPEC_ACK]);
    // the cut-off "$GNGG", "$$" and the empty "$" are counted
    assert_eq!(framer.stats().checksum_errors, 4);
}

#[test]
fn framer_skips_overlong_lines_and_keeps_going() {
    let mut framer = nmea::Framer::new();
    let exact = frame(&format!("PAIR{}", "X".repeat(MAX_SENTENCE_LEN - 8)));
    assert_eq!(exact.len(), MAX_SENTENCE_LEN);
    let over = frame(&format!("PAIR{}", "X".repeat(MAX_SENTENCE_LEN - 7)));
    let stream = lines(&[&over, &exact, &over, SPEC_GLL]);
    assert_eq!(feed(&mut framer, &stream), [exact.as_str(), SPEC_GLL]);
    let s = framer.stats();
    assert_eq!((s.sentences, s.checksum_errors, s.overflows), (2, 0, 2));
}

#[test]
fn sentence_address_and_fields() {
    let s = Sentence::new(SPEC_VTG).unwrap();
    assert_eq!(s.address(), "GNVTG");
    let fields: Vec<_> = s.fields().collect();
    assert_eq!(
        fields,
        ["0.00", "T", "", "M", "0.01", "N", "0.02", "K", "D"]
    );
    let s = Sentence::new("$PAIR002*38\r\n").unwrap();
    assert_eq!((s.address(), s.fields().count()), ("PAIR002", 0));
    assert_eq!(s.as_str(), "$PAIR002*38");
    assert_eq!(s.as_bytes(), b"$PAIR002*38");
}

// ---- decoding ----

#[test]
fn decodes_gga() {
    let Message::Gga(g) = parse(SPEC_GGA) else {
        panic!()
    };
    assert_eq!(g.talker, Talker::Gn);
    assert_eq!(g.time.unwrap().millis_of_day(), 14_503_000);
    assert_eq!(
        (g.lat_e7, g.lon_e7),
        (Some(318_222_361), Some(1_171_156_945))
    );
    assert_eq!(g.quality, Quality::Differential);
    assert_eq!((g.sats_used, g.hdop_x100), (Some(36), Some(48)));
    assert_eq!((g.alt_mm, g.geoid_sep_mm), (Some(61_496), Some(-335)));
}

#[test]
fn decodes_rmc_with_nmea_410_fields() {
    let Message::Rmc(r) = parse(SPEC_RMC) else {
        panic!()
    };
    assert!(r.valid);
    assert_eq!(
        r.date,
        Some(Date {
            year: 2022,
            month: 5,
            day: 1
        })
    );
    assert_eq!((r.speed_mm_s, r.course_cdeg), (Some(5), Some(0)));
    assert_eq!(
        (r.mode, r.nav_status),
        (Some(PositionMode::Differential), Some(b'V'))
    );
    assert_eq!(r.lat_e7, Some(318_222_361));
}

#[test]
fn decodes_gsa_with_system_id_and_empty_slots() {
    let Message::Gsa(g) = parse(SPEC_GSA) else {
        panic!()
    };
    assert!(g.automatic);
    assert_eq!(g.fix_type, FixType::Fix3d);
    assert_eq!(g.sats[0], Some(195));
    assert_eq!(g.sats[10], Some(27));
    assert_eq!(g.sats[11], None);
    assert_eq!(
        (g.pdop_x100, g.hdop_x100, g.vdop_x100),
        (Some(71), Some(48), Some(52))
    );
    assert_eq!(g.system_id, Some(1));
    let Message::Gsa(g) = parse(SPEC_GSA_BDS) else {
        panic!()
    };
    assert_eq!((g.sats.iter().flatten().count(), g.system_id), (3, Some(4)));
}

#[test]
fn decodes_gsv_with_signal_id_and_untracked_satellites() {
    let Message::Gsv(g) = parse(SPEC_GSV) else {
        panic!()
    };
    assert_eq!(
        (g.talker, g.total_sentences, g.sentence, g.sats_in_view),
        (Talker::Gp, 3, 1, 12)
    );
    assert_eq!(
        g.sats[0],
        Some(SatInfo {
            id: 195,
            elevation: Some(72),
            azimuth: Some(76),
            cn0: Some(42)
        })
    );
    assert_eq!(g.signal_id, Some(1));
    let Message::Gsv(g) = parse(SPEC_GSV_SHORT) else {
        panic!()
    };
    assert_eq!((g.talker, g.sats.iter().flatten().count()), (Talker::Gl, 1));
    let Message::Gsv(g) = parse(SPEC_GSV_NO_CN0) else {
        panic!()
    };
    assert_eq!(g.sats[0].unwrap().cn0, None);
    assert_eq!(g.sats[3].unwrap().id, 2);
}

#[test]
fn decodes_vtg_and_gll() {
    let Message::Vtg(v) = parse(SPEC_VTG) else {
        panic!()
    };
    // 0.02 km/h = 5.56 mm/s, taken from the km/h field
    assert_eq!(
        (v.course_cdeg, v.speed_mm_s, v.mode),
        (Some(0), Some(6), Some(PositionMode::Differential))
    );
    let Message::Gll(l) = parse(SPEC_GLL) else {
        panic!()
    };
    assert_eq!(
        (l.lat_e7, l.lon_e7, l.valid),
        (Some(318_222_361), Some(1_171_156_945), true)
    );
    assert_eq!(l.time.unwrap().hour, 4);
}

#[test]
fn empty_fields_decode_to_none() {
    let Message::Gga(g) = parse_body("GNGGA,,,,,,0,00,99.99,,,,,,") else {
        panic!()
    };
    assert_eq!(
        (g.time, g.lat_e7, g.lon_e7, g.alt_mm, g.geoid_sep_mm),
        (None, None, None, None, None)
    );
    assert_eq!(
        (g.quality, g.sats_used, g.hdop_x100),
        (Quality::NoFix, Some(0), Some(9999))
    );
    let Message::Rmc(r) = parse_body("GNRMC,,V,,,,,,,,,,N,V") else {
        panic!()
    };
    assert!(!r.valid);
    assert_eq!(
        (r.time, r.lat_e7, r.speed_mm_s, r.course_cdeg, r.date),
        (None, None, None, None, None)
    );
    assert_eq!(r.mode, Some(PositionMode::NoFix));
    let Message::Gsa(g) = parse_body("GNGSA,A,1,,,,,,,,,,,,,99.00,99.00,99.00,1") else {
        panic!()
    };
    assert_eq!(
        (g.fix_type, g.sats.iter().flatten().count()),
        (FixType::NoFix, 0)
    );
    let Message::Vtg(v) = parse_body("GNVTG,,T,,M,,N,,K,N") else {
        panic!()
    };
    assert_eq!((v.course_cdeg, v.speed_mm_s), (None, None));
    let Message::Gll(l) = parse_body("GNGLL,,,,,,V,N") else {
        panic!()
    };
    assert_eq!((l.lat_e7, l.time, l.valid), (None, None, false));
}

#[test]
fn decodes_proprietary_replies() {
    assert_eq!(
        parse(SPEC_ACK),
        Message::Ack(Ack {
            command: 4,
            result: 0
        })
    );
    assert_eq!(
        parse_body("PAIR001,062,4"),
        Message::Ack(Ack {
            command: 62,
            result: 4
        })
    );
    let Message::Version(v) = parse(SPEC_VERNO) else {
        panic!()
    };
    assert_eq!(v.version(), "LC76GABNR02A01S");
    assert_eq!(
        v.build_date,
        Date {
            year: 2022,
            month: 9,
            day: 14
        }
    );
    assert_eq!(
        (v.build_time.hour, v.build_time.minute, v.build_time.second),
        (11, 47, 3)
    );
    assert_eq!(
        parse(SPEC_EPE),
        Message::Epe(Epe {
            north_mm: Some(1000),
            east_mm: Some(1000),
            down_mm: Some(1000),
            epe_2d_mm: Some(1414),
            epe_3d_mm: Some(1732),
        })
    );
    let ok = |command| {
        Message::PqtmReply(PqtmReply {
            command,
            error: None,
        })
    };
    assert_eq!(parse(SPEC_CFG_OK), ok(PqtmCommand::CfgMsgRate));
    assert_eq!(parse_body("PQTMSAVEPAR,OK"), ok(PqtmCommand::SavePar));
    assert_eq!(
        parse_body("PQTMCFGMSGRATE,ERROR,1"),
        Message::PqtmReply(PqtmReply {
            command: PqtmCommand::CfgMsgRate,
            error: Some(1)
        })
    );
    assert_eq!(
        parse_body("PQTMVERNO,ERROR,2"),
        Message::PqtmReply(PqtmReply {
            command: PqtmCommand::VerNo,
            error: Some(2)
        })
    );
}

#[test]
fn undecoded_and_malformed_sentences() {
    assert_eq!(parse(SPEC_ZDA), Message::Other);
    assert_eq!(parse(SPEC_GNS), Message::Other);
    assert_eq!(parse_body("PAIR010,0,0,2044,369413"), Message::Other);
    assert_eq!(
        parse_body("GNGGA,040143.000,3149.334166"),
        Message::Malformed
    );
    assert_eq!(
        parse_body("GNRMC,040143.000,X,,,,,,,010522"),
        Message::Malformed
    );
    assert_eq!(
        parse_body("GNGGA,25:01:43,,,,,0,00,,,M,,M,,"),
        Message::Malformed
    );
    assert_eq!(parse_body("PAIR001,abc,0"), Message::Malformed);
    assert_eq!(
        parse_body("PQTMVERNO,v1,2022-09-14,11:47:03"),
        Message::Malformed
    );
}

fn gll_pos(lat: &str, ns: &str, lon: &str, ew: &str) -> (Option<i32>, Option<i32>) {
    match parse_body(&format!("GNGLL,{lat},{ns},{lon},{ew},040143.000,A,A")) {
        Message::Gll(g) => (g.lat_e7, g.lon_e7),
        other => panic!("{other:?}"),
    }
}

#[test]
fn coordinates_convert_exactly_with_signs_and_decimals() {
    // 33 51.123456 S, 151 12.3456789 W: six decimals, seven decimals (drone mode)
    assert_eq!(
        gll_pos("3351.123456", "S", "15112.345678", "W"),
        (Some(-338_520_576), Some(-1_512_057_613))
    );
    assert_eq!(
        gll_pos("3351.1234560", "N", "15112.3456789", "E"),
        (Some(338_520_576), Some(1_512_057_613))
    );
    assert_eq!(
        gll_pos("3351", "N", "15112", "E"),
        (Some(338_500_000), Some(1_512_000_000))
    );
    assert_eq!(
        gll_pos("9000.000000000", "N", "18000.000000000", "W"),
        (Some(900_000_000), Some(-1_800_000_000))
    );
    assert_eq!(gll_pos("", "", "", ""), (None, None));
}

#[test]
fn coordinates_round_to_nearest_half_away_from_zero() {
    let lat = |m: &str, ns: &str| gll_pos(&format!("0000.{m}"), ns, "", "").0;
    assert_eq!(lat("000002", "N"), Some(0)); // 0.33 of 1e-7 degree
    assert_eq!(lat("000003", "N"), Some(1)); // exactly 0.5
    assert_eq!(lat("000003", "S"), Some(-1));
    assert_eq!(lat("0000040", "N"), Some(1)); // 0.67, seven decimals
    assert_eq!(lat("0000009", "N"), Some(0)); // 0.15
}

#[test]
fn invalid_coordinates_are_malformed() {
    for (lat, ns) in [
        ("3160.000000", "N"),
        ("9000.000001", "N"),
        ("3149.3341660000", "N"),
        ("314.5", "N"),
        ("3149.33", "X"),
        ("31a9.33", "N"),
    ] {
        let m = parse_body(&format!("GNGLL,{lat},{ns},,,040143.000,A,A"));
        assert_eq!(m, Message::Malformed, "{lat} {ns}");
    }
}

#[test]
fn speed_and_course_convert_with_exact_rounding() {
    let speed = |knots: &str| match parse_body(&format!("GNRMC,,A,,,,,{knots},,,,,A,V")) {
        Message::Rmc(r) => r.speed_mm_s,
        other => panic!("{other:?}"),
    };
    assert_eq!(speed("0.00"), Some(0));
    assert_eq!(speed("1"), Some(514)); // 514.44
    assert_eq!(speed("1.00"), Some(514));
    assert_eq!(speed("0.01"), Some(5));
    assert_eq!(speed("0.03"), Some(15)); // 15.43
    assert_eq!(speed("100.5"), Some(51_702)); // 51701.9
    assert_eq!(parse_body("GNRMC,,A,,,,,-1,,,,,A,V"), Message::Malformed);
    let course = |c: &str| match parse_body(&format!("GNRMC,,A,,,,,,{c},,,,A,V")) {
        Message::Rmc(r) => Some(r.course_cdeg),
        _ => None,
    };
    assert_eq!(course("359.99"), Some(Some(35_999)));
    assert_eq!(course("12.345"), Some(Some(1235)));
    assert_eq!(course("361"), None);
}

#[test]
fn time_and_date_helpers() {
    let t = |s: &str| match parse_body(&format!("GNGLL,,,,,{s},A,A")) {
        Message::Gll(l) => l.time,
        _ => None,
    };
    assert_eq!(t("000000.000").unwrap().millis_of_day(), 0);
    assert_eq!(t("235959.999").unwrap().millis_of_day(), 86_399_999);
    assert_eq!(t("123519.5").unwrap().millis, 500);
    assert_eq!(t("123519").unwrap().millis, 0);
    assert_eq!(t("123519.123987").unwrap().millis, 123);
    assert_eq!(t("123560.000").unwrap().second, 60);
    assert_eq!(t("126000.000"), None);
}

// ---- fix assembly ----

fn gga(time: &str, lat: &str) -> Message {
    parse_body(&format!(
        "GNGGA,{time},{lat},N,11706.941670,E,1,12,0.90,10.000,M,-1.000,M,,"
    ))
}

fn rmc(time: &str) -> Message {
    parse_body(&format!(
        "GNRMC,{time},A,3149.334166,N,11706.941670,E,2.00,90.00,010522,,,A,V"
    ))
}

#[test]
fn fix_completes_when_the_second_of_gga_and_rmc_arrives() {
    let mut b = FixBuilder::new();
    assert_eq!(b.push(&gga("040143.000", "3149.334166")), None);
    let fix = b.push(&rmc("040143.000")).expect("fix");
    assert!(fix.valid);
    assert_eq!(fix.quality, Quality::Gps);
    assert_eq!(fix.lat_e7, Some(318_222_361));
    assert_eq!(
        (fix.alt_mm, fix.geoid_sep_mm, fix.sats_used, fix.hdop_x100),
        (Some(10_000), Some(-1000), Some(12), Some(90))
    );
    assert_eq!((fix.speed_mm_s, fix.course_cdeg), (Some(1029), Some(9000)));
    assert_eq!(
        fix.date,
        Some(Date {
            year: 2022,
            month: 5,
            day: 1
        })
    );
    assert_eq!(fix.time.unwrap().millis_of_day(), 14_503_000);
    assert_eq!(
        (fix.fix_type, fix.pdop_x100, fix.vdop_x100),
        (None, None, None)
    );
    // RMC first, then GGA
    assert_eq!(b.push(&rmc("040144.000")), None);
    assert!(b.push(&gga("040144.000", "3149.334166")).is_some());
    assert_eq!(b.dropped(), 0);
    assert!((fix.lat_deg().unwrap() - 31.822_236_1).abs() < 1e-9);
    assert!((fix.speed_m_s().unwrap() - 1.029).abs() < 1e-6);
}

#[test]
fn fix_epoch_with_mismatched_time_is_dropped_and_counted() {
    let mut b = FixBuilder::new();
    assert_eq!(b.push(&gga("040143.000", "3149.334166")), None);
    assert_eq!(b.push(&rmc("040143.100")), None);
    assert_eq!(b.dropped(), 1);
    assert!(b.push(&gga("040143.100", "3149.334166")).is_some());
    assert_eq!(b.dropped(), 1);
    // a repeated GGA of the same epoch is not a new epoch
    assert_eq!(b.push(&gga("040143.200", "3149.334166")), None);
    assert_eq!(b.push(&gga("040143.200", "3149.334166")), None);
    assert!(b.push(&rmc("040143.200")).is_some());
    assert_eq!(b.dropped(), 1);
}

#[test]
fn fix_takes_fix_type_and_dops_from_the_latest_gsa() {
    let mut b = FixBuilder::new();
    b.push(&parse(SPEC_GSA));
    b.push(&gga("040143.000", "3149.334166"));
    let fix = b.push(&rmc("040143.000")).unwrap();
    assert_eq!(
        (fix.fix_type, fix.pdop_x100, fix.vdop_x100),
        (Some(FixType::Fix3d), Some(71), Some(52))
    );
    b.push(&parse_body("GNGSA,A,2,,,,,,,,,,,,,3.00,2.00,,1"));
    b.push(&rmc("040144.000"));
    let fix = b.push(&gga("040144.000", "3149.334166")).unwrap();
    assert_eq!(
        (fix.fix_type, fix.pdop_x100, fix.vdop_x100),
        (Some(FixType::Fix2d), Some(300), None)
    );
}

#[test]
fn no_fix_lines_still_complete_an_epoch_without_time() {
    let mut b = FixBuilder::new();
    b.push(&parse_body("GNGGA,,,,,,0,00,99.99,,,,,,"));
    let fix = b.push(&parse_body("GNRMC,,V,,,,,,,,,,N,V")).unwrap();
    assert!(!fix.valid);
    assert_eq!(
        (fix.quality, fix.time, fix.lat_e7),
        (Quality::NoFix, None, None)
    );
}

#[test]
fn config_defaults_and_validation() {
    let c = Config::default();
    assert!(c.is_valid());
    assert_eq!(
        (c.fix_interval_ms, c.nav_mode, c.epe, c.pps),
        (1000, NavMode::Normal, false, None)
    );
    assert_eq!(
        SentenceType::ALL.map(|s| c.rates.get(s)),
        [1, 1, 1, 1, 1, 1, 0, 0, 0, 0]
    );
    for s in SentenceType::ALL {
        assert_eq!(SentenceType::ALL[usize::from(s.code())], s);
    }
    assert!(
        !Config {
            fix_interval_ms: 100,
            ..c
        }
        .is_valid()
    );
    assert!(
        !Config {
            fix_interval_ms: 99,
            ..c
        }
        .is_valid()
    );
    assert!(
        !Config {
            fix_interval_ms: 1001,
            ..c
        }
        .is_valid()
    );
    assert!(
        !Config {
            rates: SentenceRates { vtg: 21, ..c.rates },
            ..c
        }
        .is_valid()
    );
    let pps = |width_ms| {
        Some(Pps {
            kind: PpsType::Always,
            width_ms,
        })
    };
    assert!(
        Config {
            pps: pps(*PPS_WIDTH_MS.end()),
            ..c
        }
        .is_valid()
    );
    assert!(!Config { pps: pps(0), ..c }.is_valid());
    assert!(
        !Config {
            pps: pps(1000),
            ..c
        }
        .is_valid()
    );
}

#[test]
fn stream_budget_rule_and_with_fix_interval() {
    let c = Config::default();
    assert_eq!(Config::with_fix_interval(1000), c);
    let fast = Config::with_fix_interval(100);
    assert!(fast.is_valid());
    assert_eq!(
        (fast.fix_interval_ms, fast.rates.gsa, fast.rates.gsv),
        (100, 10, 10)
    );
    assert_eq!(
        Config {
            rates: SentenceRates {
                gsa: 10,
                gsv: 10,
                ..c.rates
            },
            ..fast
        },
        fast
    );
    assert_eq!(Config::with_fix_interval(200).rates.gsa, 5);
    assert_eq!(Config::with_fix_interval(300).rates.gsv, 4);
    for ms in 100..=1000 {
        let c = Config::with_fix_interval(ms);
        assert!(c.is_valid(), "{ms}");
        assert!(u32::from(c.rates.gsa) * u32::from(ms) >= 1000, "{ms}");
        assert!((u32::from(c.rates.gsa) - 1) * u32::from(ms) < 1000, "{ms}");
    }
    let grs = |rate| Config {
        rates: SentenceRates {
            grs: rate,
            ..fast.rates
        },
        ..fast
    };
    assert!(!grs(1).is_valid());
    assert!(grs(10).is_valid());
    assert!(grs(0).is_valid());
    let gsv = |rate| Config {
        rates: SentenceRates {
            gsv: rate,
            ..fast.rates
        },
        ..fast
    };
    assert!(gsv(0).is_valid());
    assert!(!gsv(9).is_valid());
    assert!(
        Config {
            rates: SentenceRates { grs: 1, ..c.rates },
            ..c
        }
        .is_valid()
    );
}

// ---- driver over a fake UART ----

type Responder = Box<dyn FnMut(&str) -> Vec<u8>>;

#[derive(Default)]
struct Shared {
    rx: VecDeque<u8>,
    tx: Vec<u8>,
    scanned: usize,
    written: Vec<String>,
    responder: Option<Responder>,
    chunks: Vec<usize>,
    next_chunk: usize,
    write_limit: Option<usize>,
    pend_when_empty: bool,
}

impl Shared {
    fn take(&mut self, out: &mut [u8]) -> usize {
        let chunk = if self.chunks.is_empty() {
            usize::MAX
        } else {
            self.next_chunk += 1;
            self.chunks[(self.next_chunk - 1) % self.chunks.len()]
        };
        let n = chunk.min(out.len()).min(self.rx.len());
        for (slot, b) in out.iter_mut().zip(self.rx.drain(..n)) {
            *slot = b;
        }
        n
    }

    fn put(&mut self, data: &[u8]) -> usize {
        let n = self.write_limit.unwrap_or(usize::MAX).min(data.len());
        self.tx.extend(&data[..n]);
        while let Some(end) = self.tx[self.scanned..].iter().position(|b| *b == b'\n') {
            let line =
                String::from_utf8(self.tx[self.scanned..self.scanned + end].to_vec()).unwrap();
            self.scanned += end + 1;
            let line = line.trim_end_matches('\r').to_owned();
            if let Some(responder) = self.responder.as_mut() {
                let reply = responder(&line);
                self.rx.extend(reply);
            }
            self.written.push(line);
        }
        n
    }
}

#[derive(Clone, Default)]
struct Fake(Rc<RefCell<Shared>>);

impl Fake {
    fn with_stream(bytes: &[u8]) -> Self {
        let fake = Self::default();
        fake.feed(bytes);
        fake
    }

    fn feed(&self, bytes: &[u8]) {
        self.0.borrow_mut().rx.extend(bytes);
    }

    fn chunks(self, chunks: &[usize]) -> Self {
        self.0.borrow_mut().chunks = chunks.to_vec();
        self
    }

    fn responder(self, f: impl FnMut(&str) -> Vec<u8> + 'static) -> Self {
        self.0.borrow_mut().responder = Some(Box::new(f));
        self
    }

    fn written(&self) -> Vec<String> {
        self.0.borrow().written.clone()
    }
}

impl embedded_io::ErrorType for Fake {
    type Error = ErrorKind;
}

impl embedded_io_async::Read for Fake {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        poll_fn(|_| {
            let mut s = self.0.borrow_mut();
            if s.rx.is_empty() && s.pend_when_empty {
                Poll::Pending
            } else {
                Poll::Ready(Ok(s.take(buf)))
            }
        })
        .await
    }
}

impl embedded_io_async::Write for Fake {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        Ok(self.0.borrow_mut().put(buf))
    }

    async fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

impl embedded_io::Read for Fake {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        Ok(self.0.borrow_mut().take(buf))
    }
}

impl embedded_io::Write for Fake {
    fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        Ok(self.0.borrow_mut().put(buf))
    }

    fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }
}

/// A module that acknowledges everything and sends one GGA before each answer.
fn answer(line: &str) -> String {
    let body = line.trim_start_matches('$').split('*').next().unwrap();
    if let Some(id) = body.strip_prefix("PAIR") {
        frame(&format!("PAIR001,{},0", &id[..3]))
    } else if body == "PQTMVERNO" {
        frame("PQTMVERNO,LC76GABNR12A05S,2025/01/02,03:04:05")
    } else {
        frame(&format!("{},OK", body.split(',').next().unwrap()))
    }
}

fn friendly(line: &str) -> Vec<u8> {
    lines(&[SPEC_GGA, &answer(line)])
}

fn epoch() -> Vec<u8> {
    lines(&[
        SPEC_GGA,
        SPEC_GLL,
        SPEC_GSA,
        SPEC_GSV,
        SPEC_GSV_SHORT,
        SPEC_RMC,
        SPEC_VTG,
    ])
}

#[test]
fn read_fix_works_for_any_chunking() {
    let two_epochs = [epoch(), epoch()].concat();
    let first_line = SPEC_GGA.len() + 2;
    for chunks in [vec![1], vec![7], vec![64], vec![first_line - 3, 50, 3, 1]] {
        let mut gnss = Lc76g::new(Fake::with_stream(&two_epochs).chunks(&chunks));
        for _ in 0..2 {
            let fix = run(gnss.read_fix()).unwrap();
            assert_eq!(fix.lat_e7, Some(318_222_361), "{chunks:?}");
            assert_eq!(fix.fix_type, Some(FixType::Fix3d));
            assert_eq!(fix.speed_mm_s, Some(5));
        }
        let s = gnss.stats();
        assert_eq!(
            (s.sentences, s.fixes, s.checksum_errors, s.epochs_dropped),
            (13, 2, 0, 0)
        );
        assert_eq!(run(gnss.read_fix()), Err(Error::Eof));
    }
}

#[test]
fn read_sentence_and_read_keep_unconsumed_bytes() {
    let mut gnss = Lc76g::new(Fake::with_stream(&[epoch(), epoch()].concat()));
    // raw reads bypass the fix builder: the first epoch has lost its GGA and yields no fix
    assert_eq!(run(gnss.read_sentence()).unwrap().as_str(), SPEC_GGA);
    assert!(matches!(run(gnss.read()), Ok(Message::Gll(_))));
    assert!(matches!(run(gnss.read()), Ok(Message::Gsa(_))));
    for _ in 0..4 {
        run(gnss.read()).unwrap();
    }
    assert_eq!(gnss.stats().fixes, 0);
    // read() completes the second epoch's fix at its RMC and holds it for read_fix()
    for _ in 0..5 {
        run(gnss.read()).unwrap();
    }
    assert!(matches!(run(gnss.read()), Ok(Message::Rmc(_))));
    assert_eq!(run(gnss.read_fix()).unwrap().alt_mm, Some(61_496));
    assert!(matches!(run(gnss.read()), Ok(Message::Vtg(_))));
    assert_eq!(run(gnss.read()), Err(Error::Eof));
}

#[test]
fn receive_only_uart_is_enough_for_reading() {
    struct OnlyRead(Fake);
    impl embedded_io::ErrorType for OnlyRead {
        type Error = ErrorKind;
    }
    impl embedded_io_async::Read for OnlyRead {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
            self.0.read(buf).await
        }
    }
    let mut gnss = Lc76g::new(OnlyRead(Fake::with_stream(&epoch())));
    assert!(run(gnss.read_fix()).unwrap().valid);
    let _uart: OnlyRead = gnss.release();
}

#[test]
fn damaged_stream_is_counted_and_survived() {
    let mut stream = b"\xde\xad$GNGGA,broken\r\n".to_vec();
    stream.extend(epoch());
    let mut gnss = Lc76g::new(Fake::with_stream(&stream).chunks(&[5]));
    assert!(run(gnss.read_fix()).is_ok());
    let s = gnss.stats();
    assert_eq!((s.checksum_errors, s.sentences), (1, 6));
}

#[test]
fn dropping_a_read_future_loses_nothing() {
    let fake = Fake::default();
    fake.0.borrow_mut().pend_when_empty = true;
    let stream = epoch();
    let cut = stream.len() / 3;
    fake.feed(&stream[..cut]);
    let mut gnss = Lc76g::new(fake.clone());
    {
        let mut f = pin!(gnss.read_fix());
        assert_eq!(
            f.as_mut().poll(&mut Context::from_waker(Waker::noop())),
            Poll::Pending
        );
    } // timeout: the future is dropped mid-stream
    fake.feed(&stream[cut..]);
    assert_eq!(run(gnss.read_fix()).unwrap().lat_e7, Some(318_222_361));
    assert_eq!(gnss.stats().sentences, 6);
}

#[test]
fn init_checks_the_link_then_writes_every_setting_and_never_saves() {
    let fake = Fake::default().responder(friendly).chunks(&[7]);
    let mut gnss = Lc76g::new(fake.clone());
    let config = Config {
        epe: true,
        pps: Some(Pps {
            kind: PpsType::Fix3dOnly,
            width_ms: 50,
        }),
        nav_mode: NavMode::Drone,
        ..Config::with_fix_interval(100)
    };
    let version = run(gnss.init(&config)).unwrap();
    assert_eq!(version.version(), "LC76GABNR12A05S");
    let bodies: Vec<String> = fake
        .written()
        .iter()
        .map(|l| {
            l.trim_start_matches('$')
                .split('*')
                .next()
                .unwrap()
                .to_owned()
        })
        .collect();
    assert_eq!(
        bodies,
        [
            "PQTMVERNO",
            "PAIR050,1000",
            "PAIR062,0,1",
            "PAIR062,1,1",
            "PAIR062,2,10",
            "PAIR062,3,10",
            "PAIR062,4,1",
            "PAIR062,5,1",
            "PAIR062,6,0",
            "PAIR062,7,0",
            "PAIR062,8,0",
            "PAIR062,9,0",
            "PQTMCFGMSGRATE,W,PQTMEPE,1,2",
            "PAIR080,5",
            "PAIR752,2,50",
            "PAIR050,100",
        ]
    );
    assert!(fake.written().iter().all(|l| Sentence::new(l).is_some()));
    assert!(
        !bodies
            .iter()
            .any(|b| b.contains("513") || b.contains("SAVE"))
    );
    // each command consumed its own GGA: the fixes never completed, nothing pending
    assert_eq!(gnss.stats().fixes_discarded, 0);
}

#[test]
fn init_default_config_has_no_optional_commands() {
    let fake = Fake::default().responder(friendly);
    let mut gnss = Lc76g::new(fake.clone());
    run(gnss.init(&Config::default())).unwrap();
    let w = fake.written();
    assert_eq!(w.len(), 14);
    assert_eq!(w[1], "$PAIR050,1000*12");
    assert_eq!(w[12], "$PQTMCFGMSGRATE,W,PQTMEPE,0,2*1C");
    assert_eq!(w[13], "$PAIR080,0*2E");
    assert_eq!(w.iter().filter(|l| l.starts_with("$PAIR050")).count(), 1);
}

#[test]
fn invalid_config_sends_nothing() {
    let fake = Fake::default().responder(friendly);
    let mut gnss = Lc76g::new(fake.clone());
    let config = Config {
        fix_interval_ms: 50,
        ..Config::default()
    };
    assert_eq!(run(gnss.init(&config)), Err(Error::InvalidConfig));
    let flooded = Config {
        fix_interval_ms: 100,
        ..Config::default()
    };
    assert_eq!(run(gnss.init(&flooded)), Err(Error::InvalidConfig));
    assert!(fake.written().is_empty());
}

#[test]
fn rejected_command_reports_id_and_code_and_stops_init() {
    let fake = Fake::default().responder(|line| {
        if line.starts_with("$PAIR080") {
            lines(&[&frame("PAIR001,080,4")])
        } else {
            friendly(line)
        }
    });
    let mut gnss = Lc76g::new(fake.clone());
    let config = Config {
        nav_mode: NavMode::Drone,
        ..Config::default()
    };
    assert_eq!(
        run(gnss.init(&config)),
        Err(Error::Rejected {
            command: CommandId::Pair(80),
            code: 4
        })
    );
    assert_eq!(fake.written().last().unwrap(), "$PAIR080,5*2B");
    // PQTM errors carry the PQTM code
    let fake = Fake::default().responder(|_| lines(&[&frame("PQTMCFGMSGRATE,ERROR,2")]));
    let mut gnss = Lc76g::new(fake);
    assert_eq!(
        run(gnss.command(&Command::SetEpe(true))),
        Err(Error::Rejected {
            command: CommandId::Pqtm(PqtmCommand::CfgMsgRate),
            code: 2
        })
    );
}

#[test]
fn being_processed_ack_keeps_waiting_for_the_final_one() {
    let fake = Fake::default().responder(|_| {
        lines(&[
            SPEC_GGA,
            &frame("PAIR001,050,0"), // another command's ack is ignored
            &frame("PAIR001,004,1"),
            SPEC_RMC,
            &frame("PAIR001,004,0"),
            SPEC_VTG,
        ])
    });
    let mut gnss = Lc76g::new(fake.clone());
    run(gnss.command(&Command::Start(StartKind::Hot))).unwrap();
    // the GGA and RMC seen while waiting completed a fix, which command() discarded and counted
    let s = gnss.stats();
    assert_eq!((s.fixes, s.fixes_discarded), (1, 1));
    // the sentence after the ack is still there for the next read
    assert!(matches!(run(gnss.read()), Ok(Message::Vtg(_))));
}

#[test]
fn missing_acknowledgement_gives_up_after_the_sentence_limit() {
    let fake = Fake::default().responder(|_| {
        let mut out = Vec::new();
        for _ in 0..ACK_SENTENCE_LIMIT + 5 {
            out.extend(lines(&[SPEC_VTG]));
        }
        out
    });
    let mut gnss = Lc76g::new(fake);
    assert_eq!(run(gnss.command(&Command::SaveSettings)), Err(Error::NoAck));
    assert_eq!(gnss.stats().sentences as usize, ACK_SENTENCE_LIMIT);
    // the driver is still usable and the unread sentences are still there
    assert!(matches!(run(gnss.read()), Ok(Message::Vtg(_))));
}

#[test]
fn silent_line_surfaces_as_eof_or_pending_not_a_spin() {
    let mut gnss = Lc76g::new(Fake::default());
    assert_eq!(run(gnss.version()), Err(Error::Eof));
    let fake = Fake::default();
    fake.0.borrow_mut().pend_when_empty = true;
    let mut gnss = Lc76g::new(fake);
    let mut f = pin!(gnss.version());
    assert_eq!(
        f.as_mut().poll(&mut Context::from_waker(Waker::noop())),
        Poll::Pending
    );
}

#[test]
fn raw_command_without_known_reply_returns_after_sending() {
    let fake = Fake::default();
    let mut gnss = Lc76g::new(fake.clone());
    run(gnss.command(&Command::Raw("PMTK000"))).unwrap();
    assert_eq!(fake.written(), [frame("PMTK000")]);
    assert_eq!(
        run(gnss.command(&Command::Raw("A*B"))),
        Err(Error::InvalidCommand)
    );
    assert_eq!(fake.written().len(), 1);
}

#[test]
fn short_writes_are_completed() {
    let fake = Fake::default().responder(friendly);
    fake.0.borrow_mut().write_limit = Some(3);
    let mut gnss = Lc76g::new(fake.clone());
    run(gnss.command(&Command::SetNavMode(NavMode::Swimming))).unwrap();
    assert_eq!(fake.written(), ["$PAIR080,7*29"]);
}

#[test]
fn save_settings_is_explicit_and_saves_both_stores() {
    let fake = Fake::default().responder(friendly);
    let mut gnss = Lc76g::new(fake.clone());
    run(gnss.save_settings()).unwrap();
    assert_eq!(fake.written(), ["$PAIR513*3D", "$PQTMSAVEPAR*5A"]);
}

#[test]
fn blocking_driver_runs_the_same_code() {
    let fake = Fake::default().responder(friendly);
    let mut gnss = blocking::Lc76g::new(fake.clone());
    let version = gnss.init(&Config::default()).unwrap();
    assert_eq!(version.version(), "LC76GABNR12A05S");
    assert_eq!(fake.written().len(), 14);
    fake.feed(&epoch());
    assert_eq!(gnss.read_fix().unwrap().quality, Quality::Differential);
    assert_eq!(gnss.read_sentence().unwrap().address(), "GNVTG");
    assert_eq!(gnss.read(), Err(Error::Eof));
    assert_eq!(gnss.stats().fixes, 1);
    let _ = gnss.release();
}
