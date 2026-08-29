#include "Timers.h"
#include "Config.h"

void timersBegin() {
    // --- Timer2: D3, the BRUSH ---------------------------------------------
    // Prescaler 1 AND fast PWM -> 62.5 kHz.
    //
    // The prescaler alone gave 31.37 kHz, not 62.5: the Arduino core leaves
    // Timer2 in PHASE-CORRECT PWM (WGM=001), which counts up and back down and
    // so takes 510 ticks per period instead of 256. Setting WGM21 alongside the
    // core's WGM20 makes WGM=011 (fast PWM, TOP=255):
    //
    //     16e6 / (256 * 1) = 62500 Hz
    TCCR2A |= _BV(WGM21);
    TCCR2B = (TCCR2B & 0b11111000) | 0b001;

    // --- Timer0: D6 AND D5, BOTH WHEELS ------------------------------------
    // One prescaler write sets both channels, so left and right cannot drift
    // apart. Mismatched channels respond differently to the same demand, which
    // reads as a pull to one side.
    //
    // WARNING: THIS IS THE LINE THAT BREAKS millis(). At prescaler 1 instead of
    // the stock 64, millis(), micros() and delay() all run 64x fast. Everything
    // this sketch measures goes through REAL_MS()/REAL_US() to compensate;
    // anything else that calls millis() — the Ethernet library's internal
    // timeouts — is NOT compensated and runs 64x short. That is survivable only
    // because this build uses a static IP and never waits on DHCP.
    TCCR0B = (TCCR0B & 0b11111000) | 0b001;

    // --- Timer1: D9, the lamp ----------------------------------------------
    // The core leaves Timer1 in 8-bit phase-correct with prescaler 64 = 490 Hz.
    // Fast PWM 8-bit (WGM13:0 = 0101) with prescaler 1 gives 62.5 kHz, the same
    // number every other driven pin runs at. Here the frequency buys freedom
    // from visible flicker rather than motor matching. Timer1 drives no timing
    // in this sketch, so nothing else moves with it.
    TCCR1A = (TCCR1A & 0b11111100) | _BV(WGM10);
    TCCR1B = (TCCR1B & 0b11100000) | _BV(WGM12) | 0b001;
}
