//! Tests of ZDA, GNS, GST and GRS. Spec lines are the examples of the Quectel LC26G/LC76G/LC86G GNSS
//! Protocol Specification V1.1.

use proptest::prelude::*;

use crate::aux_sentences::{ConstellationModes, Gns, Grs, Gst, ResidualMode, Zda};
use crate::nmea::checksum;
use crate::sentence::{Date, Message, PositionMode, Talker, Time};
use crate::{Framer, Sentence};

/// Section 2.2.7, page 25.
const SPEC_ZDA: &str = "$GNZDA,055054.000,19,09,2022,,*4A";
/// Section 2.2.8, page 28.
const SPEC_GNS: &str =
    "$GNGNS,053106.000,3149.334190,N,11706.948654,E,DANN,16,0.63,51.287,M,-0.335,M,,,V*05";
/// Section 2.2.9, page 29. (The field table of the same page shows 5.9 for AltD, the example 9.2.)
const SPEC_GST: &str = "$GNGST,123624.000,6.3,2.5,2.4,88.4,2.4,2.5,9.2*43";
/// Section 2.2.10, page 31.
const SPEC_GRS: [&str; 5] = [
    "$GNGRS,125524.000,1,-0.4,-0.7,0.5,-4.6,0.2,1.1,-2.2,-0.6,-1.1,9.2,-2.1,3.1,1,1*42",
    "$GNGRS,125524.000,1,-11.4,,,,,,,,,,,,1,1*52",
    "$GNGRS,125524.000,1,19.4,-5.0,11.4,6.3,-118,3.3,-7.5,,,,,,2,1*79",
    "$GNGRS,125524.000,1,-5.6,4.6,21.1,,,,,,,,,,3,7*51",
    "$GNGRS,125524.000,1,-5.8,-10.3,2.4,2.1,-2.3,20.0,-25.1,-9.1,,,,,4,1*7A",
];

fn frame(body: &str) -> String {
    format!("${body}*{:02X}", checksum(body.as_bytes()))
}

fn parse(line: &str) -> Message {
    Sentence::new(line).expect("valid sentence").parse()
}

fn parse_body(body: &str) -> Message {
    parse(&frame(body))
}

fn empty(address: &str, fields: usize) -> String {
    format!("{address}{}", ",".repeat(fields))
}

fn time(hour: u8, minute: u8, second: u8, millis: u16) -> Option<Time> {
    Some(Time {
        hour,
        minute,
        second,
        millis,
    })
}

fn zda(line: &str) -> Zda {
    match parse(line) {
        Message::Zda(z) => z,
        m => panic!("{m:?}"),
    }
}

fn gns(line: &str) -> Gns {
    match parse(line) {
        Message::Gns(g) => g,
        m => panic!("{m:?}"),
    }
}

fn gst(line: &str) -> Gst {
    match parse(line) {
        Message::Gst(g) => g,
        m => panic!("{m:?}"),
    }
}

fn grs(line: &str) -> Grs {
    match parse(line) {
        Message::Grs(g) => g,
        m => panic!("{m:?}"),
    }
}

#[test]
fn zda_spec_example() {
    let z = zda(SPEC_ZDA);
    assert_eq!(z.talker, Talker::Gn);
    assert_eq!(z.time, time(5, 50, 54, 0));
    assert_eq!(
        z.date,
        Some(Date {
            year: 2022,
            month: 9,
            day: 19
        })
    );
    assert_eq!((z.local_hour, z.local_min), (None, None));
    assert_eq!(z.utc_unix_ms(), Some(1_663_566_654_000));
    assert_eq!(z.local_offset_minutes(), None);
}

#[test]
fn zda_empty_fields_and_zone() {
    let z = zda(&frame(&empty("GNZDA", 6)));
    assert_eq!(
        z,
        Zda {
            talker: Talker::Gn,
            time: None,
            date: None,
            local_hour: None,
            local_min: None
        }
    );
    assert_eq!(z.utc_unix_ms(), None);
    let z = zda(&frame("GPZDA,235959.500,29,02,2024,-05,30"));
    assert_eq!(z.talker, Talker::Gp);
    assert_eq!((z.local_hour, z.local_min), (Some(-5), Some(30)));
    assert_eq!(z.local_offset_minutes(), Some(-330));
    assert_eq!(z.utc_unix_ms(), Some(1_709_251_199_500));
    let z = zda(&frame("GNZDA,000000.000,31,12,1999,+13,59"));
    assert_eq!(z.local_offset_minutes(), Some(839));
    assert_eq!(
        z.utc_unix_ms(),
        Some(946_598_400_000 + 86_400_000 - 86_400_000)
    );
    let z = zda(&frame("GNZDA,000000.000,31,04,2024,,"));
    assert!(z.date.is_some());
    assert_eq!(z.utc_unix_ms(), None);
    let z = zda(&frame("GNZDA,000000.000,29,02,2023,,"));
    assert_eq!(z.utc_unix_ms(), None);
}

#[test]
fn zda_malformed() {
    for body in [
        "GNZDA,055054.000,19,09,2022,",
        "GNZDA,055054.000,19,09,2022,,,",
        "GNZDA",
        "GNZDA,055054.000,19,,2022,,",
        "GNZDA,055054.000,,09,2022,,",
        "GNZDA,055054.000,0,09,2022,,",
        "GNZDA,055054.000,32,09,2022,,",
        "GNZDA,055054.000,19,13,2022,,",
        "GNZDA,055054.000,19,09,10000,,",
        "GNZDA,055054.000,19,09,2022,14,",
        "GNZDA,055054.000,19,09,2022,-14,",
        "GNZDA,055054.000,19,09,2022,-,",
        "GNZDA,055054.000,19,09,2022,x,",
        "GNZDA,055054.000,19,09,2022,,60",
        "GNZDA,055054.000,19,09,2022,,-1",
        "GNZDA,0550,19,09,2022,,",
        "GNZDA,055054.000,1x,09,2022,,",
    ] {
        assert_eq!(parse_body(body), Message::Malformed, "{body}");
    }
}

#[test]
fn gns_spec_example() {
    let g = gns(SPEC_GNS);
    assert_eq!(g.talker, Talker::Gn);
    assert_eq!(g.time, time(5, 31, 6, 0));
    assert_eq!(g.lat_e7, Some(318_222_365));
    assert_eq!(g.lon_e7, Some(1_171_158_109));
    let modes = g.modes.expect("mode indicator");
    assert_eq!(
        modes.modes,
        [
            Some(PositionMode::Differential),
            Some(PositionMode::Autonomous),
            Some(PositionMode::NoFix),
            Some(PositionMode::NoFix),
            None,
            None
        ]
    );
    assert_eq!(modes.gps(), Some(PositionMode::Differential));
    assert_eq!(modes.glonass(), Some(PositionMode::Autonomous));
    assert_eq!(modes.galileo(), Some(PositionMode::NoFix));
    assert_eq!(modes.bds(), Some(PositionMode::NoFix));
    assert_eq!((modes.qzss(), modes.navic()), (None, None));
    assert_eq!(modes.reported(), 4);
    assert_eq!(g.sats_used, Some(16));
    assert_eq!(g.hdop_x100, Some(63));
    assert_eq!(g.alt_mm, Some(51_287));
    assert_eq!(g.geoid_sep_mm, Some(-335));
    assert_eq!((g.diff_age_ms, g.diff_station), (None, None));
    assert_eq!(g.nav_status, Some(b'V'));
}

#[test]
fn gns_empty_fields_short_form_and_all_six_modes() {
    let g = gns(&frame(&empty("GNGNS", 14)));
    assert_eq!(
        g,
        Gns {
            talker: Talker::Gn,
            time: None,
            lat_e7: None,
            lon_e7: None,
            modes: None,
            sats_used: None,
            hdop_x100: None,
            alt_mm: None,
            geoid_sep_mm: None,
            diff_age_ms: None,
            diff_station: None,
            nav_status: None
        }
    );
    let g = gns(&frame(&empty("GNGNS", 15)));
    assert_eq!(g.nav_status, None);
    let g = gns(&frame(
        "GPGNS,010203.040,,,,,ANDEMR,3,99.00,-1.5,M,2.250,M,12.5,0123,V",
    ));
    assert_eq!(g.talker, Talker::Gp);
    assert_eq!(g.time, time(1, 2, 3, 40));
    assert_eq!(
        g.modes,
        Some(ConstellationModes {
            modes: [
                Some(PositionMode::Autonomous),
                Some(PositionMode::NoFix),
                Some(PositionMode::Differential),
                Some(PositionMode::Estimated),
                Some(PositionMode::Manual),
                Some(PositionMode::Rtk)
            ]
        })
    );
    assert_eq!(g.modes.map(|m| m.reported()), Some(6));
    assert_eq!(g.hdop_x100, Some(9900));
    assert_eq!(g.alt_mm, Some(-1500));
    assert_eq!(g.geoid_sep_mm, Some(2250));
    assert_eq!((g.diff_age_ms, g.diff_station), (Some(12_500), Some(123)));
    let g = gns(&frame("GNGNS,,,,,,F,,,,,,,,,"));
    assert_eq!(
        g.modes.map(|m| m.modes[0]),
        Some(Some(PositionMode::FloatRtk))
    );
}

#[test]
fn gns_malformed() {
    for body in [
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,M,",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,M,,,V,",
        "GNGNS",
        "GNGNS,053106.000,,,,,DANNADD,1,1.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,X,1,1.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,DA?N,1,1.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,F,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,X,,,V",
        "GNGNS,053106.000,3149.334190,Q,,,N,1,1.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,x,1.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,1,1.0.0,1.0,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,1,1.0,a,M,1.0,M,,,V",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,M,-1,,V",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,M,,x,V",
        "GNGNS,053106.000,,,,,N,1,1.0,1.0,M,1.0,M,,,VV",
        "GNGNS,25,,,,,N,1,1.0,1.0,M,1.0,M,,,V",
    ] {
        assert_eq!(parse_body(body), Message::Malformed, "{body}");
    }
}

#[test]
fn gns_standard_layout() {
    let g = gns(&frame(
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,-0.335,,,V",
    ));
    assert_eq!(g.talker, Talker::Gn);
    assert_eq!(g.time, time(5, 31, 6, 0));
    assert_eq!(g.lat_e7, Some(318_222_365));
    assert_eq!(g.lon_e7, Some(1_171_158_109));
    let modes = g.modes.expect("mode indicator");
    assert_eq!(
        modes.modes,
        [
            Some(PositionMode::Autonomous),
            Some(PositionMode::Autonomous),
            Some(PositionMode::Autonomous),
            Some(PositionMode::Autonomous),
            Some(PositionMode::NoFix),
            Some(PositionMode::NoFix)
        ]
    );
    assert_eq!(modes.reported(), 6);
    assert_eq!(g.sats_used, Some(24));
    assert_eq!(g.hdop_x100, Some(60));
    assert_eq!(g.alt_mm, Some(51_287));
    assert_eq!(g.geoid_sep_mm, Some(-335));
    assert_eq!((g.diff_age_ms, g.diff_station), (None, None));
    assert_eq!(g.nav_status, Some(b'V'));
    let g = gns(&frame(
        "GPGNS,010203.040,,,,,ANDEMR,3,99.00,-1.5,2.250,12.5,0123,V",
    ));
    assert_eq!(g.talker, Talker::Gp);
    assert_eq!(g.time, time(1, 2, 3, 40));
    assert_eq!((g.lat_e7, g.lon_e7), (None, None));
    assert_eq!(g.modes.map(|m| m.reported()), Some(6));
    assert_eq!(g.hdop_x100, Some(9900));
    assert_eq!(g.alt_mm, Some(-1500));
    assert_eq!(g.geoid_sep_mm, Some(2250));
    assert_eq!((g.diff_age_ms, g.diff_station), (Some(12_500), Some(123)));
    let g = gns(&frame(&empty("GNGNS", 13)));
    assert_eq!(
        g,
        Gns {
            talker: Talker::Gn,
            time: None,
            lat_e7: None,
            lon_e7: None,
            modes: None,
            sats_used: None,
            hdop_x100: None,
            alt_mm: None,
            geoid_sep_mm: None,
            diff_age_ms: None,
            diff_station: None,
            nav_status: None
        }
    );
}

#[test]
fn gns_standard_layout_malformed() {
    for body in [
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,-0.335,,V",
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,-0.335,,,V,,,",
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,M,-0.335,,,V",
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,-0.335,,,VV",
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,-0.335,x,,V",
        "GNGNS,053106.000,3149.334190,N,11706.948654,E,AAAANN,24,0.60,51.287,M,,,V",
    ] {
        assert_eq!(parse_body(body), Message::Malformed, "{body}");
    }
}

#[test]
fn gst_spec_example() {
    let g = gst(SPEC_GST);
    assert_eq!(g.talker, Talker::Gn);
    assert_eq!(g.time, time(12, 36, 24, 0));
    assert_eq!(g.rms_mm, Some(6300));
    assert_eq!(g.semi_major_mm, Some(2500));
    assert_eq!(g.semi_minor_mm, Some(2400));
    assert_eq!(g.orientation_cdeg, Some(8840));
    assert_eq!(g.lat_std_mm, Some(2400));
    assert_eq!(g.lon_std_mm, Some(2500));
    assert_eq!(g.alt_std_mm, Some(9200));
}

#[test]
fn gst_empty_fields_and_talker() {
    let g = gst(&frame(&empty("GAGST", 8)));
    assert_eq!(
        g,
        Gst {
            talker: Talker::Ga,
            time: None,
            rms_mm: None,
            semi_major_mm: None,
            semi_minor_mm: None,
            orientation_cdeg: None,
            lat_std_mm: None,
            lon_std_mm: None,
            alt_std_mm: None
        }
    );
    let g = gst(&frame("GNGST,000000.000,0,0.0005,,-12.5,,3,"));
    assert_eq!(g.rms_mm, Some(0));
    assert_eq!(g.semi_major_mm, Some(1));
    assert_eq!(g.semi_minor_mm, None);
    assert_eq!(g.orientation_cdeg, Some(-1250));
    assert_eq!(g.lat_std_mm, None);
    assert_eq!(g.lon_std_mm, Some(3000));
}

#[test]
fn gst_malformed() {
    for body in [
        "GNGST,123624.000,6.3,2.5,2.4,88.4,2.4,2.5",
        "GNGST,123624.000,6.3,2.5,2.4,88.4,2.4,2.5,9.2,",
        "GNGST",
        "GNGST,123624.000,-6.3,2.5,2.4,88.4,2.4,2.5,9.2",
        "GNGST,123624.000,6.3,abc,2.4,88.4,2.4,2.5,9.2",
        "GNGST,123624.000,6.3,2.5,2.4,8x.4,2.4,2.5,9.2",
        "GNGST,123624.000,6.3,2.5,2.4,88.4,2.4,2.5,9.2.1",
        "GNGST,1236,6.3,2.5,2.4,88.4,2.4,2.5,9.2",
        "GNGST,123624.000,99999999999999999999,2.5,2.4,88.4,2.4,2.5,9.2",
    ] {
        assert_eq!(parse_body(body), Message::Malformed, "{body}");
    }
}

#[test]
fn grs_spec_examples() {
    let none = None;
    let expected: [([Option<i32>; 12], u8, u8); 5] = [
        (
            [
                Some(-400),
                Some(-700),
                Some(500),
                Some(-4600),
                Some(200),
                Some(1100),
                Some(-2200),
                Some(-600),
                Some(-1100),
                Some(9200),
                Some(-2100),
                Some(3100),
            ],
            1,
            1,
        ),
        (
            [
                Some(-11_400),
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
            ],
            1,
            1,
        ),
        (
            [
                Some(19_400),
                Some(-5000),
                Some(11_400),
                Some(6300),
                Some(-118_000),
                Some(3300),
                Some(-7500),
                none,
                none,
                none,
                none,
                none,
            ],
            2,
            1,
        ),
        (
            [
                Some(-5600),
                Some(4600),
                Some(21_100),
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
                none,
            ],
            3,
            7,
        ),
        (
            [
                Some(-5800),
                Some(-10_300),
                Some(2400),
                Some(2100),
                Some(-2300),
                Some(20_000),
                Some(-25_100),
                Some(-9100),
                none,
                none,
                none,
                none,
            ],
            4,
            1,
        ),
    ];
    for (line, (residuals, system, signal)) in SPEC_GRS.iter().zip(expected) {
        let g = grs(line);
        assert_eq!(g.talker, Talker::Gn);
        assert_eq!(g.time, time(12, 55, 24, 0));
        assert_eq!(g.mode, ResidualMode::Recomputed);
        assert_eq!(g.residuals_mm, residuals, "{line}");
        assert_eq!(g.system_id, Some(system));
        assert_eq!(g.signal_id, Some(signal));
        assert_eq!(g.residual_count(), residuals.iter().flatten().count());
    }
    assert_eq!(grs(SPEC_GRS[0]).residual_count(), 12);
    assert_eq!(grs(SPEC_GRS[3]).residual_count(), 3);
}

#[test]
fn grs_empty_fields_mode_zero_and_short_form() {
    let g = grs(&frame("GLGRS,,0,,,,,,,,,,,,,,"));
    assert_eq!(
        g,
        Grs {
            talker: Talker::Gl,
            time: None,
            mode: ResidualMode::UsedForFix,
            residuals_mm: [None; 12],
            system_id: None,
            signal_id: None
        }
    );
    let g = grs(&frame("GNGRS,125524.000,0,1.5,-999,999.000,,,,,,,,,"));
    assert_eq!(g.mode, ResidualMode::UsedForFix);
    assert_eq!(
        &g.residuals_mm[..4],
        &[Some(1500), Some(-999_000), Some(999_000), None]
    );
    assert_eq!((g.system_id, g.signal_id), (None, None));
}

#[test]
fn grs_malformed() {
    let twelve = ",1".repeat(12);
    for body in [
        format!("GNGRS,125524.000,1{}", ",1".repeat(11)),
        format!("GNGRS,125524.000,1{twelve},1"),
        format!("GNGRS,125524.000,1{twelve},1,1,1"),
        "GNGRS".to_string(),
        format!("GNGRS,125524.000,2{twelve},1,1"),
        format!("GNGRS,125524.000,{twelve},1,1"),
        format!("GNGRS,125524.000,x{twelve},1,1"),
        format!("GNGRS,125524.000,1,1000.0{},1,1", ",1".repeat(11)),
        format!("GNGRS,125524.000,1,-999.001{},1,1", ",1".repeat(11)),
        format!("GNGRS,125524.000,1,q{},1,1", ",1".repeat(11)),
        format!("GNGRS,125524.000,1{twelve},x,1"),
        format!("GNGRS,125524.000,1{twelve},1,300"),
        format!("GNGRS,12,1{twelve},1,1"),
    ] {
        assert_eq!(parse_body(&body), Message::Malformed, "{body}");
    }
}

#[test]
fn the_four_sentences_no_longer_fall_into_other() {
    assert!(matches!(parse(SPEC_ZDA), Message::Zda(_)));
    assert!(matches!(parse(SPEC_GNS), Message::Gns(_)));
    assert!(matches!(parse(SPEC_GST), Message::Gst(_)));
    for line in SPEC_GRS {
        assert!(matches!(parse(line), Message::Grs(_)));
    }
    assert_eq!(parse_body("GNRLM,1,2"), Message::Other);
}

proptest! {
    #[test]
    fn printable_bytes_framed_as_each_sentence_never_panic(
        kind in 0usize..4,
        talker in 0usize..6,
        body in proptest::collection::vec(0x20u8..0x7F, 0..110),
    ) {
        let name = ["ZDA", "GNS", "GST", "GRS"][kind];
        let talker = ["GP", "GL", "GA", "GB", "GQ", "GN"][talker];
        let text: String = body.iter().map(|&b| char::from(b)).collect();
        let body = format!("{talker}{name},{text}");
        let line = format!("${body}*{:02X}", checksum(body.as_bytes()));
        if let Some(sentence) = Sentence::new(&line) {
            let _ = sentence.parse();
        }
        let mut framer = Framer::new();
        for b in line.bytes().chain(*b"\r\n") {
            if let Some(sentence) = framer.push(b) {
                let _ = sentence.parse();
            }
        }
    }

    #[test]
    fn field_shaped_text_never_panics(
        kind in 0usize..4,
        fields in proptest::collection::vec("[0-9.+\\-NSEWMADFRVX]{0,5}", 0..=18),
    ) {
        let name = ["ZDA", "GNS", "GST", "GRS"][kind];
        let body = format!("GN{name},{}", fields.join(","));
        if let Some(sentence) = Sentence::new(&frame(&body)) {
            let _ = sentence.parse();
        }
    }

    #[test]
    fn gst_millimetres_match_the_decimal_text(mm in 0u32..=4_000_000, orient in -36_000i32..=36_000) {
        let metres = format!("{}.{:03}", mm / 1000, mm % 1000);
        let sign = if orient < 0 { "-" } else { "" };
        let degrees = format!("{sign}{}.{:02}", orient.abs() / 100, orient.abs() % 100);
        let g = gst(&frame(&format!("GNGST,010203.004,{metres},{metres},{metres},{degrees},{metres},{metres},{metres}")));
        prop_assert_eq!(g.rms_mm, Some(mm));
        prop_assert_eq!(g.alt_std_mm, Some(mm));
        prop_assert_eq!(g.orientation_cdeg, Some(orient));
    }

    #[test]
    fn grs_residuals_match_the_decimal_text(mm in -999_000i32..=999_000) {
        let text = format!("{}{}.{:03}", if mm < 0 { "-" } else { "" }, mm.abs() / 1000, mm.abs() % 1000);
        let g = grs(&frame(&format!("GNGRS,010203.004,0,{text}{},1,1", ",".repeat(11))));
        prop_assert_eq!(g.residuals_mm[0], Some(mm));
        prop_assert_eq!(g.residual_count(), 1);
    }

    #[test]
    fn zda_unix_time_matches_a_day_count(day in 1u8..=28, month in 1u8..=12, year in 1970u16..=2099) {
        let line = frame(&format!("GNZDA,000000.000,{day:02},{month:02},{year:04},,"));
        let z = zda(&line);
        let leap = |y: u16| y.is_multiple_of(4) && (!y.is_multiple_of(100) || y.is_multiple_of(400));
        let months = [31, if leap(year) { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
        let mut days: i64 = (1970..year).map(|y| if leap(y) { 366 } else { 365 }).sum();
        days += months[..usize::from(month - 1)].iter().sum::<i64>();
        days += i64::from(day) - 1;
        prop_assert_eq!(z.utc_unix_ms(), Some(days * 86_400_000));
    }
}
