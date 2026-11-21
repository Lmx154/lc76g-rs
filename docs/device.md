# LC76G device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the specification listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the specification or on hardware. "Measured" means measured on this board.

## Quectel LC76G (GNSS, UART1 and PPS on the bench)

Pins: see "Pins and bus" below.

- Protocol specification V1.1 (link below). Bench module `LC76GABNR12A05S`, build 2025/09/25 10:10:11 (`$PQTMVERNO`,
  p100). 115200 baud 8N1 confirmed. Get commands answer `$PAIR001,<id>,0`, then the value (p39): `$PAIR051,1000*13`.
- PPS: `$PAIR752,<type>,<width_ms>` (p78): type 0 off, 1 after first fix, 2 3D fix only, 3 2D/3D fix, 4 always.
  Default type: UNKNOWN (the module pulsed at 1 Hz, 100 ms wide, with a fix; not read back).
- Default sentences (measured): GGA, GLL, GSA, GSV, RMC, VTG every fix; ZDA, GRS, GST, GNS off. The spec's "default 1"
  for all types (p41) is wrong here. `$PAIR062,-1` resets the rates (ZDA 1 -> 0 verified); `$PAIR062,-1,0` and `,-1,1`
  are rejected with `$PAIR001,062,4`.
- `$PAIR050,100` gives 10 Hz (p38), but here GGA, GLL, RMC, VTG, `$PQTMEPE` and (at rate 1) GSA and GSV all go at
  10 Hz; the spec says GSA/GSV stay at 1 Hz and VTG/GLL stop. That is 187 sentences, 11.6 kB per second: the whole
  capacity of 115200 baud (11.52 kB/s). Fixes drop (97 per 10 s) and commands are answered a second late or never.
  One epoch at the default rates is 19 sentences (4 GSA, 11 to 12 GSV). `Config::is_valid` therefore requires GSA, GSV
  and GRS off or at most once per second below 1000 ms; GSA/GSV at 10 (`with_fix_interval(100)`): 100 fixes per 10 s,
  ~65 sentences/s, replies in milliseconds. `init` sets 1000 ms first and the target interval last, so no step floods.
- Two `$PAIR050` back to back restart the module (2026-11-20, bench, `LC76GABNR12A05S`). When the first really changes
- Power-on (measured 2026-11-20, `LC76GABNR12A05S`, three power-ons after 22 to 47 s off): the module starts at the fix interval set before the power-off (200 ms) with its default sentence set; a `$PAIR050` in its first second or two gets `$PAIR001,050,1` and never a final result (100 sentences, about 1.5 s); the same command 5 s later is accepted.
  the rate and the second follows soon after, the module answers `$PAIR001,050,1` (being processed), sends nothing
  more, restarts by itself about 1.8 s later and returns at its stored settings (1 Hz, stored sentence set). Spacing
  from the first acknowledgement to the second send: 18, 26, 46, 47, 50, 70, 120, 170 ms restarted it (8 of 8); 207,
  271, 370, 470, 519, 771, 970 ms were accepted (7 of 7); 97 ms accepted once. A resend does not help. After every
  applied change the module sends an unsolicited `$PAIR001,124,0` 114 to 241 ms after the acknowledgement. `init`
  waits `FIX_INTERVAL_SETTLE_MS` = 500 ms (about 3x the latest restart, 2x the latest `124`) before the last
  fix-interval command, with the delay passed to it; at 1000 ms it sends one command and never waits.
- 10 Hz fix times are not on a 100 ms grid: about 57 of 99 steps are exactly 100 ms, the rest 89 to 107 ms, rarely up
  to 172 ms. After a return to 1 Hz one step of 1277 to 1483 ms precedes the 1000 ms steps.
- Drone mode (`$PAIR080,5`): coordinates still have six decimals of minutes (`ddmm.mmmmmm,N,dddmm.mmmmmm,W`), not
  seven as the note on p51 says; the driver accepts both. `$PQTMEPE,2,1.224,1.253,2.415,1.752,2.983*57` follows each fix.
- Bench fix: 3D, 19 to 25 satellites used, HDOP 0.5; at rest the position moves 1 to 13 m within 10 s in drone mode.
- PPS (GPIO5, plain input): 1 s period to 5 ppm by the MCU timer; in about half the runs one period is 1 to 3 ms off
  (module or edge latency, undecided). `read_fix` returns the fix of a second 75 to 90 ms after the PPS edge.
- RP2350 at 150 MHz: framing one GGA 49 us, framing + parse 115 us (RMC 46 / 116 us; 209 / 174 before `Decimal::scaled`
  got a 64-bit path); `Lc76g<BufferedUart>` 440 bytes plus the buffers (RX 4096, TX 256). Bursts are about 1.4 kB, so
  use a buffered UART. After a probe reset the receive line can show a burst of framing errors (0 to 228 seen).
- `$PAIR513*3D` saves the module's settings to its flash. Send it only when a permanent change is intended.

### PQTMPVT probe (2026-11-21, `LC76GABNR12A05S` dated 2025/09/25, `hil/src/bin/lc76g_pvt.rs`, one run)

- Specification: `PQTMPVT` is in protocol specification V1.4 (2024-03-05, chapter 2.3.8, pages 40 to 42; added in V1.3), not in V1.1.
  Fields: MsgVer (1), TOW (ms), Date (UTC YYYYMMDD), Time (UTC hhmmss.sss), Quality (as GGA), FixMode (0/2/3), NumSatUsed,
  LeapS (s), Lat, Lon (degree), Alt (m above mean sea level), Sep (m), VelN, VelE, VelD ("Down velocity", m/s), Spd (m/s),
  Heading (degree), HDOP, PDOP. The specification states no sign for VelD beyond its name.
- The module was at its flight configuration (`$PAIR051,200`, 19 sentences/s). `$PQTMCFGMSGRATE,R,PQTMPVT,1` answered
  `$PQTMCFGMSGRATE,OK,PQTMPVT,0,1` (off); `,W,PQTMPVT,1,1` and `,W,PQTMPVT,0,1` each answered `$PQTMCFGMSGRATE,OK` in
  6 ms. No restart, no change of the fix interval (read back 200 ms after), no `PQTMPVT` in the 3 s after disabling.
- Output: one `PQTMPVT` per fix (100 in 20 s, 0 malformed), period 175 to 222 ms (mean 200.0 ms), 20 fields as specified,
  136 to 139 bytes with CR LF: longer than `MAX_SENTENCE_LEN` (128), so the driver's framer drops every one as an
  overflow (seen in a first run: 105 overflows in 20 s). Example, position fields masked:
  `$PQTMPVT,1,542618800,20261121,064320.800,2,3,34,18,###,###,###,###,-0.002,0.004,0.000,0.001,251.18,0.42,0.80`.
- Timing: Time equals the RMC time of the same epoch (100 of 100 paired); TOW is GPS time of week (UTC + LeapS 18 s).
  The `PQTMPVT` arrives 14.5 to 16.1 ms after that RMC (mean 15.8 ms), in the same burst.
- At rest (board on the bench, fix with 32 to 34 satellites, quality 2), mm/s: VelN mean -7.6, sd 12.1, -48 to 11;
  VelE mean -0.6, sd 13.8, -41 to 34; VelD mean -0.5, sd 0.7, -3 to 0; Spd 1 to 45; Heading held at one value.
  The sign of VelD and its response to vertical motion are not checked (needs a lift-and-lower test).
- Load at 5 Hz: about 690 bytes/s more (2.0 kB/s with, 1.3 kB/s without, of 11.5 kB/s), 24 sentences/s instead of 19.

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 5 | PWM5 | GPIO input (bench) | LC76G PPS on silk S5; otherwise the servo PWM5 pad, IO block row 7 |
| 24 | ELRS_RX | UART1 TX (F2) | IO block row 1, silk T1; MCU output, to the GPS RX |
| 25 | ELRS_TX | UART1 RX (F2) | IO block row 2, silk R1; MCU input, from the GPS TX |

Net names follow the schematic's design intent: UART1 nets are named ELRS. The nets keep these names whatever is attached.

| Peripheral | GPIOs | Device |
|---|---|---|
| UART1 | TX 24, RX 25 | IO block rows 1/2, silk T1/R1 |

Bench configuration (attached by hand on the current board; the PCB itself provides the pins above): LC76G GPS on silk T1/R1: T1 = GPIO24 = UART1 TX (MCU output, to GPS RX); R1 = GPIO25 = UART1 RX (MCU input, from GPS TX). PPS on silk S5 = GPIO5, a plain GPIO input (S5 is otherwise the servo PWM5 pad).

A pin alternate does not create an extra controller. UART1 is the controller on the T1/R1 port (GPIO24/25), so the F11 UART1 TX/RX functions on GPIO26/27 cannot be used alongside it.

## Datasheet

| Device | URL |
|---|---|
| Quectel LC76G protocol specification V1.1 | https://quectel.com/content/uploads/2024/02/Quectel_LC26GABLC76GLC86G_Series_GNSS_Protocol_Specification_V1.1.pdf |
