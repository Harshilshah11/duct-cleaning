#pragma once
#include <Arduino.h>

void timersBegin();

/* Timer0 back to its stock prescaler for this object's lifetime, so delay()
 * inside the Ethernet library is real milliseconds again - init() needs 560 ms
 * for the shield's reset supervisor and softReset() ~20 ms to poll. Starved of
 * those, detection fails and the chip is left unaddressable. */
struct StockTimer0 {
    uint8_t saved;
    StockTimer0()  { saved = TCCR0B; TCCR0B = (TCCR0B & 0b11111000) | 0b011; }
    ~StockTimer0() { TCCR0B = saved; }
};
