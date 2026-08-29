# arduino — the robot's Uno firmware

One sketch folder, `duct_cleaning_robot_firmware/`. Open
`duct_cleaning_robot_firmware.ino` in the Arduino IDE; every other file appears
as a tab beside it.

**Board:** Arduino Uno · **Shield:** W5100/W5500 Ethernet · **Serial:** 115200

## Choosing the transport

One `#define` at the top of `Config.h`, and it is the only difference between
what used to be three separate sketches:

```c
#define LINK_TRANSPORT LINK_ETHERNET   // the rig: UDP on 192.168.50.20:5005
#define LINK_TRANSPORT LINK_SERIAL     // the bench: the USB tether
```

Verified on `arduino:avr:uno`:

| Build | Flash | SRAM |
|---|---|---|
| `LINK_ETHERNET` | 17,052 B (52%) | 693 B (33%) |
| `LINK_SERIAL` | 7,618 B (23%) | 544 B (26%) |

The Pi is pinned to UDP (`UNO_TRANSPORT` in `uno_motors.py`), so `LINK_ETHERNET`
is the build that matches the ground station. On that build the USB port is a
console: it is drained so a terminal cannot stall the link, but commands
arriving on it are discarded unless `SERIAL_COMMANDS` is flipped true.

## The tabs

| File | What lives there |
|---|---|
| `duct_cleaning_robot_firmware.ino` | `setup()` / `loop()` — the orchestration and the failsafe |
| `Config.h` | Every pin and every tuning number, with the reasoning attached |
| `Timers.cpp` | The three PWM timers — and the line that makes `millis()` 64× fast |
| `Outputs.cpp` | Wheels, rod, brush, lamp, `safeState()` |
| `Commands.cpp` | The wire protocol and the arcade mixer |
| `Link.cpp` | The transport, and the W5100 recovery the rig needs |
| `Telemetry.cpp` | The `L= R= ACT= BRUSH= LIGHT=` line |
| `ResetCause.cpp` | Why the board restarted — brown-out vs. true power loss |

## Booting: it waits for the ground station

`setup()` does not fall through to `loop()` until the Pi actually sends a command
it understands, and it ACKs that command so the link is proven in both
directions before anything else happens. It retries for as long as it takes.

```
RESET: flags=0x1 POWER-ON  ram=LOST (true power loss)  boot#0
W5100: W5100 ok   link: UP
W5100: chip=51 config VERIFIED (IP reads back)
WAIT: holding in setup() until the ground station speaks
WAIT: the shield is only reset if it PROVES faulty, not because it is quiet
WAIT: 3s, chip=51 configured, link=UP - no command yet
WAIT: 6s, chip=51 configured, link=UP - no command yet
WAIT: ground station is up, seq=41, waited 8213 ms, repairs=0
```

**The wait repairs the shield only on positive evidence of a fault** — chip
undetected, or its IP not reading back. A healthy shield that nobody is talking
to is left alone, however long the silence lasts.

That distinction is the entire design, and it is not theoretical. A boot-time
probe like this existed on 2026-08-27 and was removed the same day: it reset the
W5100 whenever no packet arrived, reasoning that the Pi transmits at 50 Hz so
silence must mean a deaf shield. But when the rig is powered on as a whole, this
board is listening about five seconds in while the Pi is still most of a minute
from starting the ground station. The probe read that normal silence as a fault
and fired six chip resets into a perfectly good shield — which is exactly what
*"turn off pi and uno 10-15 min, after start its not connected, 3-4 times on off
uno and it works"* was. **Waiting was never the problem. Resetting on silence
was.**

Blocking here is safe: `outputsBegin()`, `timersBegin()` and `safeState()` have
all run first, so every output is already at a stopped level and nothing can
command them while the board waits. The rod's software PWM is serviced
throughout, and the USB port is drained so a terminal cannot fill the RX buffer.

Two knobs in `Config.h`:

| Constant | Default | Meaning |
|---|---|---|
| `LINK_WAIT_FOR_GROUND_STATION` | `true` | `false` skips the wait entirely |
| `LINK_WAIT_TIMEOUT_MS` | `0` | `0` = wait forever; otherwise give up and run anyway |

Falling through without a link is not a failure mode — `loop()`'s failsafe and
`linkService()` are built for exactly that. The wait buys a deterministic boot
and a console that says what is happening, not safety the loop lacked.

## "It does not start until I press the reset button"

Fixed 2026-08-29 by bringing the Ethernet up **before** `timersBegin()`, not
after. If you only read one thing in this file, read this — the bug was invisible
and the fix is an ordering.

`timersBegin()` puts Timer0 on prescaler 1 for 62.5 kHz wheel PWM, which makes
`delay()` run 64× fast. The W5100 driver measures two things with `delay()` that
it genuinely needs:

| In the library | Written for | Got, at prescaler 1 |
|---|---|---|
| `W5100Class::init()` — wait out the shield's CAT811 reset supervisor | 560 ms | **8.75 ms** |
| `W5100Class::softReset()` — poll for reset completion, `delay(1)` × 20 | ~20 ms | **0.31 ms** |

On a cold start this rig's 5 V rail crawls up through a linear regulator burning
(12−5) × 0.23 A, and the W5100 needs its full budget to finish resetting. Starved
to 0.31 ms it does not answer, `softReset()` times out, `isW5100()` returns 0, and
`init()` sets `chip = 0`. There is no case for 0 in the library's register
dispatch — every access afterwards is **W5500 framing aimed at a W5100**, so the
shield is deaf with a perfectly good cable.

Press the reset button and the rail is already up, the chip answers on the first
poll, and everything works. That is the entire "restart the Arduino and it starts
working" symptom.

`linkBegin()` now runs while Timer0 is still stock, so those delays are real.
Anything that re-enters the Ethernet library *after* the switch — the silent-link
repair in `linkService()`, and the boot wait's repair — wraps itself in a
`StockTimer0` guard (`Timers.h`) that restores the stock prescaler for the
duration and puts it back afterwards. Two register writes. Safe while driving:
at boot nothing moves, and in `loop()` that repair cannot run until the link has
been silent long enough for the failsafe to have stopped every output.

### If it still does not come up

Read the serial console at 115200 during a failed cold start. **The banner
answers a question no amount of firmware can:**

- **Banner prints** (`RESET: flags=...`, `W5100: ...`) — the sketch is running
  and the shield is the problem. Look at `chip=`: 51 means the library is
  talking to a real W5100 and everything downstream can be believed; 0 means
  detection failed and the fault is the rail or the shield.
- **Console silent** — the AVR itself never started. It came up before Vcc was
  valid and is in an undefined state. No delay, retry or reset *in software* can
  help, because none of it executes. Fix the rail: feed the Uno 5 V directly
  rather than 12 V through its linear regulator, or add bulk capacitance.

`RESET:` also distinguishes the two power faults. `BROWN-OUT` with
`ram=KEPT` means the rail dipped but held; `ram=LOST (true power loss)` means it
reached zero. A `boot#` that climbs on its own is a board resetting repeatedly.

## Three things that will bite

**1. `millis()` runs 64× fast.** `Timers.cpp` puts Timer0 on prescaler 1 to get
62.5 kHz PWM on the wheels, and Timer0 is what drives `millis()`, `micros()` and
`delay()`. A bare `300` is not 300 ms, it is 4.7 ms. Every duration in `Config.h`
goes through `REAL_MS()` / `REAL_US()`; anything that does not is a bug. This has
already caused two real faults — a loop pause that was 78 µs instead of 5 ms, and
a soft-PWM period that would have run at 16 kHz.

**2. Run with the microSD slot empty.** D4 is the right wheel's direction line
*and* the shield's card chip-select. It goes LOW whenever that wheel is driven
negative, and LOW selects a card, which then drives MISO during the SPI reads
this sketch makes to the W5100 on every pass of `loop()`. The symptom is an
Ethernet link that dies when the rod stops. There is no software fix — one pin
cannot be both a chip select and a motor gate.

**3. Test the failsafe with the wheels off the ground.** Unplug the tether
mid-drive and confirm every channel stops within a third of a second.

## What happened to the old sketches

`uno_eth_link/`, `uno_usb_link/` and `uno_serial_drive/` were consolidated here
on 2026-08-29. They were ~95% identical and had drifted apart, and the copy that
drifted was always the one nobody was testing that week — the "brush is always
on" fix had to be applied to two files by hand the same day.

`uno_eth_link/src/` — a 23-file class refactor — was **not** carried forward, and
this is the part worth knowing about. It was never `#include`d by the sketch, so
it had never run, but the Arduino IDE compiles everything under a sketch's `src/`
regardless: it was being built into the binary, which is most of why the old
Ethernet build was 1,118 bytes larger than this one. Its `Config.h` was confident,
fully commented, and **wrong on every tuned value** against the running board:

| | `src/Config.h` | The board |
|---|---|---|
| `MIN_DUTY` | 0 | **90** |
| `DEADBAND` | 4 | **12** |
| `INVERT_2` | false | **true** |
| `SERIAL_BAUD` | 250000 | **115200** |
| `FAILSAFE_MS` | 300 (unscaled) | **300 × 64** |
| soft-PWM period | 4000 (unscaled) | **4000 × 64** |
| brush stop | direction asserted | **both lines LOW** |
| light direction | held HIGH | **held LOW** |

Adopting it would have given a robot whose failsafe fired every 5 ms and whose
right wheel ran backwards. The class decomposition is in git history at `3db5676`
if it is ever wanted; the numbers in it are not.

The old sketches are recoverable the same way:

```bash
git show 3db5676:arduino/uno_eth_link/uno_eth_link.ino > /tmp/old_eth.ino
```

## Building from the command line

```bash
arduino-cli compile --fqbn arduino:avr:uno duct_cleaning_robot_firmware
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 duct_cleaning_robot_firmware
```

Needs `arduino-cli core install arduino:avr` and `arduino-cli lib install Ethernet`.
