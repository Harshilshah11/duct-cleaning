#include "ResetCause.h"

// .noinit: deliberately not zeroed by the C runtime, so these survive a reset.
uint8_t  resetFlags __attribute__((section(".noinit")));
uint16_t bootCount  __attribute__((section(".noinit")));
uint16_t bootMagic  __attribute__((section(".noinit")));

static bool ramSurvived = false;
static const uint16_t BOOT_MAGIC = 0xB07F;

/* Runs before main(), ahead of the .bss clear — the only point at which MCUSR
 * still holds what the hardware put there. */
void captureResetCause(void) __attribute__((naked, used, section(".init3")));
void captureResetCause(void) {
    resetFlags = MCUSR;
    MCUSR = 0;
}

void resetCauseBeginBoot() {
    if (bootMagic != BOOT_MAGIC) {
        bootMagic = BOOT_MAGIC;
        bootCount = 0;
        ramSurvived = false;        // magic was garbage: the rail reached zero
    } else {
        bootCount++;
        ramSurvived = true;         // magic intact: a reset, rail held
    }
}

void resetCauseReport() {
    Serial.print(F("RESET: flags=0x"));
    Serial.print(resetFlags, HEX);
    if (resetFlags & _BV(PORF))  Serial.print(F(" POWER-ON"));
    if (resetFlags & _BV(EXTRF)) Serial.print(F(" EXTERNAL"));
    if (resetFlags & _BV(BORF))  Serial.print(F(" BROWN-OUT"));
    if (resetFlags & _BV(WDRF))  Serial.print(F(" WATCHDOG"));
    Serial.print(ramSurvived ? F("  ram=KEPT (reset, rail held)")
                             : F("  ram=LOST (true power loss)"));
    Serial.print(F("  boot#"));
    Serial.println(bootCount);
}
