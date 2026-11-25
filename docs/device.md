# LC76G device facts

Part of [lc76g](../README.md)

Module and board facts the driver was written against (MARV V2 Rev 1, LC76G on UART1 with PPS). Page numbers are PDF pages of the Quectel protocol specification linked at the end. Items marked UNKNOWN are not established. Figures labelled observed were seen on the bench module `LC76GABNR12A05S` (build 2025/09/25 10:10:11, from `$PQTMVERNO`, p100).

```mermaid
flowchart LR
  mcu["MCU UART1<br/>TX GPIO24, RX GPIO25"] <--> gps["LC76G"]
  gps -- "PPS" --> g5["GPIO5 (plain input)"]
```

## Protocol

- 115200 baud 8N1. Get commands answer `$PAIR001,<id>,0`, then the value (p39), for example `$PAIR051,1000*13`.
- PPS: `$PAIR752,<type>,<width_ms>` (p78): type 0 off, 1 after first fix, 2 3D fix only, 3 2D/3D fix, 4 always. Default type UNKNOWN (the module pulsed at 1 Hz, 100 ms wide, with a fix; not read back).
- `$PAIR513*3D` saves the module's settings to flash; send it only when a permanent change is intended.
- Default sentences (observed): GGA, GLL, GSA, GSV, RMC, VTG every fix; ZDA, GRS, GST, GNS off. The specification's "default 1" for all types (p41) does not hold here. `$PAIR062,-1` resets the rates; `$PAIR062,-1,0` and `,-1,1` are rejected with `$PAIR001,062,4`.
- Drone mode (`$PAIR080,5`): coordinates keep six decimals of minutes (`ddmm.mmmmmm,N,dddmm.mmmmmm,W`), not seven as the note on p51 says; the driver accepts both. `$PQTMEPE,2,...` follows each fix.

## 10 Hz stream budget

| Setting | Observed result |
|---|---|
| `$PAIR050,100` with default rates | GGA, GLL, RMC, VTG, `$PQTMEPE` and (at rate 1) GSA and GSV all at 10 Hz (the specification says GSA/GSV stay at 1 Hz and VTG/GLL stop): 187 sentences and 11.6 kB per second, the whole capacity of 115200 baud (11.52 kB/s). Fixes drop (97 per 10 s) and commands are answered a second late or never |
| GSA and GSV at 10 (`with_fix_interval(100)`) | 100 fixes per 10 s, about 65 sentences per second, replies in milliseconds |

One epoch at the default rates is 19 sentences (4 GSA, 11 to 12 GSV). `Config::is_valid` therefore requires GSA, GSV and GRS off or at most once per second below 1000 ms. `init` sets 1000 ms first and the target interval last, so no step floods. At 10 Hz fix times are not on a 100 ms grid: about 57 of 99 steps are exactly 100 ms, the rest 89 to 107 ms, rarely up to 172 ms; after a return to 1 Hz one step of 1277 to 1483 ms precedes the 1000 ms steps.

Fix-interval restart and power-on behaviour are described in [driver](driver.md).

## `$PQTMPVT`

`PQTMPVT` is in protocol specification V1.4 (chapter 2.3.8, pages 40 to 42; added in V1.3), not in V1.1. Fields after the address: MsgVer (1), TOW (ms), Date (UTC YYYYMMDD), Time (UTC hhmmss.sss), Quality (as GGA), FixMode (0/2/3), NumSatUsed, LeapS (s), Lat, Lon (degree), Alt (m above mean sea level), Sep (m), VelN, VelE, VelD (down velocity, m/s; the specification states no sign beyond the name), Spd (m/s), Heading (degree), HDOP, PDOP: 19 fields.

| Item | Observed |
|---|---|
| Rate command | `$PQTMCFGMSGRATE,R,PQTMPVT,1` answers `$PQTMCFGMSGRATE,OK,PQTMPVT,0,1` (off); `,W,PQTMPVT,1,1` and `,W,PQTMPVT,0,1` each answer `$PQTMCFGMSGRATE,OK` in 6 ms; no restart, no change of the fix interval |
| Output | one per fix (100 in 20 s, 0 malformed), period 175 to 222 ms (mean 200.0 ms), 136 to 139 bytes with CR LF |
| Timing | Time equals the RMC time of the same epoch (100 of 100); TOW is GPS time of week (UTC + LeapS 18 s); arrives 14.5 to 16.1 ms after that RMC (mean 15.8 ms), in the same burst |
| Load at 5 Hz | about 690 bytes/s more (2.0 kB/s with, 1.3 kB/s without, of 11.5 kB/s), 24 sentences/s instead of 19 |
| At rest, mm/s | VelN mean -7.6 (sd 12.1), VelE mean -0.6 (sd 13.8), VelD mean -0.5 (sd 0.7), Spd 1 to 45, heading steady. The sign of VelD and its response to vertical motion need a lift-and-lower test |

`HDOP` and `PDOP` of 99.99 (invalid) stay 9999 in the decoder; velocities decode to signed mm/s, heading to 0.01 degree.

## Timing and cost (RP2350 at 150 MHz)

| Item | Value |
|---|---|
| Framing one GGA | 49 us |
| Framing and parsing | 115 us (RMC 46 us and 116 us) |
| `Lc76g<BufferedUart>` | 440 bytes plus the UART buffers (RX 4096, TX 256) with the 128-byte framer; the 160-byte framer adds 32 bytes, and the command buffers on the stack of `command` grow by 32 bytes each (164 and 162 bytes) |
| Burst size | about 1.4 kB, so use a buffered UART |
| PPS | 1 s period to 5 ppm by the MCU timer, one period 1 to 3 ms off in about half the runs (cause undecided); `read_fix` returns the fix of a second 75 to 90 ms after the PPS edge |
| Bench fix | 3D, 19 to 25 satellites (up to 34 in another run), HDOP 0.5; at rest the position moves 1 to 13 m within 10 s in drone mode |

After a probe reset the receive line can show a burst of framing errors (0 to 228 seen).

## Pins and bus (MARV V2)

All GPIO are 3.3 V logic and not 5 V tolerant.

| GPIO | Net | Function |
|---|---|---|
| 24 | ELRS_RX | UART1 TX, IO block row 1, silk T1; MCU output to the GPS RX |
| 25 | ELRS_TX | UART1 RX, IO block row 2, silk R1; MCU input from the GPS TX |
| 5 | PWM5 | PPS as a plain GPIO input on silk S5 (otherwise the servo PWM5 pad, IO block row 7) |

The UART1 nets keep their ELRS names whatever is attached. The module is attached by hand to the IO block; the PCB provides the pins. UART1 is the controller on the T1/R1 port, so the UART1 functions on GPIO26/27 cannot be used alongside it.

## Specification

Quectel LC76G protocol specification V1.1: https://quectel.com/content/uploads/2024/02/Quectel_LC26GABLC76GLC86G_Series_GNSS_Protocol_Specification_V1.1.pdf

## Related

[driver](driver.md) · [sentences](sentences.md) · [hil](../hil/README.md) · [README](../README.md)
