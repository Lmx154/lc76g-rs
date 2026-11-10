# lc76g

Driver for the Quectel LC76G GNSS module on a UART, for the MARV V2 Rev 1 flight controller (RP2354B). `embedded-hal` has no UART traits, so the crate depends on `embedded-io` 0.7 and `embedded-io-async` 0.7 instead, and optionally `defmt`. There are three ways in:

- the async driver `Lc76g`, on `embedded_io_async::Read` (plus `Write` to send commands);
- `blocking::Lc76g`, the same code on the blocking `embedded_io` traits;
- the core with no UART at all: `Framer` (bytes in, checked sentences out), `Sentence::parse`, `FixBuilder` and `Command::write`, for firmware that drains its own DMA ring buffer.

## Usage

```rust
let mut gps = lc76g::Lc76g::new(buffered_uart);           // 115200 baud 8N1
let config = lc76g::Config {
    nav_mode: lc76g::NavMode::Drone,
    ..lc76g::Config::with_fix_interval(100)               // 10 Hz
};
gps.init(&config).await?;
loop {
    let fix = gps.read_fix().await?;                      // one GGA + RMC pair of the same second
    if fix.valid {
        let (lat, lon, alt) = (fix.lat_e7, fix.lon_e7, fix.alt_mm);
    }
}
```

- A fix is integers only: 1e-7 degree, mm, mm/s, 0.01 degree. `read` returns each decoded sentence and `read_sentence` the raw text, for logging.
- Give the driver a buffered UART: the module sends bursts of about 1.4 kB.
- Above 1 Hz the satellite lists (GSA, GSV) may go out at most once per second, and `init` refuses a configuration that breaks this. This module's full default stream at 10 Hz is more than 115200 baud can carry, and a flooded module stops answering commands. `Config::with_fix_interval` sets the rates to fit.
- At 10 Hz the module's fix times are not on a 100 ms grid: steps between about 90 and 107 ms are normal. Use `fix.time` for timing, not the order of arrival.
- A fix reaches the application 0.07 to 0.18 s after the PPS edge of its second. The driver does not touch the PPS pin; the application time-stamps it.
- `init` never writes the module's flash, so settings last until the module loses power. `save_settings` is a separate call.
- The driver has no timer. On a silent line a read waits forever, so wrap calls in a timeout.
- Decoding costs about 115 us per sentence on the RP2350, under 1 % of one core at 10 Hz.

## What the application provides

- A buffered UART: `embedded_io_async::Read` for the async driver, the blocking `embedded_io::Read` for `blocking`. `Write` is needed as well to send commands.

Dependencies: `embedded-io` 0.7, `embedded-io-async` 0.7 and, optionally, `defmt`.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |

## Measured on the MARV V2 Rev 1 board

- 10 Hz, 100 fixes per 10 s with no checksum error.
- About 115 us to decode a sentence.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh lc76g_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `lc76g_smoke` | firmware version, configuration to 10 Hz, fix rate and counters, plausible fixes, decode cost, PPS period, the blocking front-end, return to the module's defaults; never writes the module's flash |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: UART1 TX on GPIO24 (silk T1) and RX on GPIO25 (silk R1), 115200 baud 8N1; PPS on GPIO5 (silk S5), a plain GPIO input.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive` and `lc76g_smoke` exited 0 when last run.
- GPS: once during development, `init` with the default configuration returned `NoAck` while the module was running at 10 Hz. It did not happen again in the 16 runs on the final code. The cause is unknown.

Not done for this device:

- Baud-rate change and constellation selection (both restart the module).
- The module's accuracy estimate merged into the fix (the message is decoded on its own).
- Time synchronisation from PPS.
- Recovery of a module that other software left flooded.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and lc76g_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   protocol, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the specification listed in `docs/device.md`.
