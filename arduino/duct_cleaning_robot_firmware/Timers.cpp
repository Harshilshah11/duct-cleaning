#include "Timers.h"
#include "Config.h"

void timersBegin() {
    // Timer2 (D3, brush): fast PWM + prescaler 1 = 62.5 kHz. WGM21 matters -
    // the core leaves it phase-correct, which halves the frequency.
    TCCR2A |= _BV(WGM21);
    TCCR2B = (TCCR2B & 0b11111000) | 0b001;

    // Timer0 (D5+D6, both wheels): one prescaler governs the pair so they
    // cannot drift apart. THIS is the line that makes millis() 64x fast.
    TCCR0B = (TCCR0B & 0b11111000) | 0b001;

    // Timer1 (D9, lamp): fast PWM 8-bit, prescaler 1 = 62.5 kHz.
    TCCR1A = (TCCR1A & 0b11111100) | _BV(WGM10);
    TCCR1B = (TCCR1B & 0b11100000) | _BV(WGM12) | 0b001;
}
