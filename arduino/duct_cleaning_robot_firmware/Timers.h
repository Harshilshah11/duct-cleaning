#pragma once
#include <Arduino.h>

/*
 * Timers — put all three PWM timers on 62.5 kHz.
 *
 * MUST be called from setup(), and calling it is what makes MILLIS_SCALE true:
 * Timer0 goes to prescaler 1 here, and Timer0 is millis(). See Config.h.
 */
void timersBegin();

/*
 * StockTimer0 — put Timer0 back on its stock prescaler for the lifetime of the
 * object, then restore whatever it was.
 *
 * THIS EXISTS BECAUSE THE ETHERNET LIBRARY MEASURES TIME IN delay(), AND WE
 * BROKE delay(). timersBegin() puts Timer0 on prescaler 1 for 62.5 kHz wheel
 * PWM, which makes delay() run 64x fast — and the W5100 driver depends on two
 * delays being real:
 *
 *   W5100Class::init()      delay(560)  waits out the shield's CAT811 reset
 *                                       supervisor (240 ms typical, 560 ms
 *                                       worst case) -> got 8.75 ms
 *   W5100Class::softReset() delay(1) x20 polls for reset completion, ~20 ms
 *                                       of budget -> got 0.31 ms
 *
 * On a cold, slowly-rising rail the chip needs that budget. Starved of it,
 * softReset() times out, isW5100() returns 0, and init() sets chip = 0 — a
 * shield the library then addresses with W5500 framing, i.e. deaf. Press the
 * reset button once the rail is up and the chip answers on the first poll, so
 * it works. That is the whole "does not start until I reset it" fault.
 *
 * Wrap every call into the Ethernet library that can re-init or reset the chip
 * in one of these. Cheap: two register writes.
 *
 * SAFE WHILE DRIVING? The wheels' PWM frequency drops to ~976 Hz for the
 * duration, and millis() counts real rather than fast. Both are fine where this
 * is used: at boot nothing is moving, and in loop() the only caller is the
 * silent-link repair, which cannot run until the failsafe has already stopped
 * every output 300 ms into the silence.
 */
struct StockTimer0 {
    uint8_t saved;
    StockTimer0()  { saved = TCCR0B; TCCR0B = (TCCR0B & 0b11111000) | 0b011; }
    ~StockTimer0() { TCCR0B = saved; }
};
