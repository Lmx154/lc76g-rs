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
