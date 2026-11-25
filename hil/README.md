# hil

Part of [lc76g](../README.md)

On-board test programs for the MARV V2 Rev 1 board with the LC76G on UART1 and PPS on GPIO5 (pins in [device](../docs/device.md)). This folder is its own workspace and the only user of embassy-rp. `tools/run.sh <name> [timeout_s]` builds one program, flashes it through a debug probe (probe-rs, chip RP235x) under a lock that serialises probe use, and runs it. Exit status 0 means every check passed; logs go to `hil/logs/`. No program sends a flash write, a baud change, a restart or a low-power command. The driver programs end with `init(&Config::default())`; `lc76g_pvt` does not use the driver and disables the PVT output again.

| Program | Run | What it checks |
|---|---|---|
| `alive` | `tools/run.sh alive` | tool chain, probe and board |
| `lc76g_smoke` | `tools/run.sh lc76g_smoke` | version query, `init` at 10 Hz, 100 fixes in 10 s (plus or minus 2), plausible fixes, decode cost, PPS period, the blocking front-end, return to default settings. Its only `Raw` bodies are the get commands `PAIR051`, `PAIR063,<n>` and `PAIR081` |
| `lc76g_aux` | `tools/run.sh lc76g_aux 150` | decodes ZDA, GNS, GST and GRS from the real module and cross-checks them against RMC, GGA and GSA; positions are printed masked |
| `lc76g_stream` | `tools/run.sh lc76g_stream 170` | audits the whole stream: a UART wrapper tallies every sentence in the raw bytes (by talker and type, own checksum test) and compares it with what the driver made of them, at the default 1 Hz stream, the 10 Hz configuration and everything on. Expected: equal counts, 0 `Malformed`, 0 checksum errors, 0 bytes outside a sentence, 10 Hz fixes equal to GGA + RMC pairs, and agreement across sentences of one epoch (time, position, status, satellites in use, speed, dates) |
| `lc76g_resync` | `tools/run.sh lc76g_resync 300` | abandoned commands: the module answers back-to-back queries in order sent; a command right after an abandoned one of the same number returns its own answer (both directions); the same sequence without resynchronisation takes the stale answer; 10 Hz fixes read under 20 ms timeouts lose nothing. A resynchronisation costs about 10 ms and 4 sentences at 10 Hz; command-to-acknowledgement time is 4.4 ms at best, 5.6 ms median, 93 ms worst. Sends only get commands and ZDA output-rate settings (some deliberately invalid) |
| `lc76g_pvt` | `tools/run.sh lc76g_pvt` | probe of `$PQTMPVT` with the program's own framer: the rate command, output timing and at-rest values (see [device](../docs/device.md)) |

## Related

[driver](../docs/driver.md) · [device](../docs/device.md)
