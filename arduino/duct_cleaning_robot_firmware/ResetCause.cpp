#include "ResetCause.h"
#include <avr/wdt.h>

uint8_t  resetFlags __attribute__((section(".noinit")));
uint16_t bootCount  __attribute__((section(".noinit")));
uint16_t bootMagic  __attribute__((section(".noinit")));

uint16_t prevVerdict      __attribute__((section(".noinit")));
uint16_t prevPackets      __attribute__((section(".noinit")));
uint16_t prevServiceCount __attribute__((section(".noinit")));
uint16_t prevChip         __attribute__((section(".noinit")));
uint16_t prevLinkUp       __attribute__((section(".noinit")));
static uint16_t sVerdict, sPackets, sServices, sChip, sUp;

static bool ramSurvived = false;
static const uint16_t BOOT_MAGIC = 0xB07F;

void captureResetCause(void) __attribute__((naked, used, section(".init3")));
void captureResetCause(void) {
    resetFlags = MCUSR;
    MCUSR = 0;
    // wdt_disable() IS NOT OPTIONAL HERE. Clearing MCUSR without it leaves an
    // armed watchdog running: it fires again before setup() finishes and the
    // board reset-loops, which is exactly the "6 boots in 35 seconds" already
    // recorded against this rig. This is the canonical .init3 pair.
    wdt_disable();
}

void resetCauseBeginBoot() {
    if (bootMagic != BOOT_MAGIC) {
        bootMagic = BOOT_MAGIC;
        bootCount = 0;
        ramSurvived = false;
        prevVerdict = prevPackets = prevServiceCount = prevChip = prevLinkUp = 0xFFFF;
    } else {
        bootCount++;
        ramSurvived = true;
    }
}

void resetCauseReport() {
    Serial.print(F("RESET: 0x"));
    Serial.print(resetFlags, HEX);
    Serial.print(ramSurvived ? F(" ram=KEPT boot#") : F(" ram=LOST boot#"));
    Serial.println(bootCount);
}

void resetCauseStash(uint16_t verdict, uint16_t packets, uint16_t services,
                     uint16_t chip, uint16_t up) {
    // Written continuously so whatever the board was doing when it died is kept.
    prevVerdict = verdict; prevPackets = packets; prevServiceCount = services;
    prevChip = chip; prevLinkUp = up;
    (void)sVerdict; (void)sPackets; (void)sServices; (void)sChip; (void)sUp;
}
