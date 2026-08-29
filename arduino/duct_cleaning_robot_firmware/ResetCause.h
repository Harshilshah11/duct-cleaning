#pragma once
#include <Arduino.h>

/*
 * ResetCause — WHY DID THE BOARD RESTART?
 *
 * Operator, 2026-08-27: "when i turn on brush to robo disconnect connect
 * disconnect connect continuosly". The ping log agreed — seventeen down/up flaps
 * in four minutes, only while the brush was running. That is either the AVR
 * resetting or the shield wedging, and the two want completely different fixes,
 * so the board now says which.
 *
 * MCUSR holds the reset source but must be read before the C runtime clobbers
 * it — hence the .init3 hook in the .cpp, which runs ahead of the .bss clear in
 * .init4. BORF there is a BROWN-OUT: the 5V rail sagged below the detector
 * threshold and the chip reset itself, which is the signature a motor inrush
 * leaves.
 *
 * Optiboot may clear MCUSR before we ever see it, which is what the .noinit
 * counter is for. .noinit survives a RESET but not a POWER LOSS — RAM holds its
 * contents down to roughly 1.5V while the brown-out detector trips at about
 * 2.7V. So an intact magic word means the board reset with power broadly
 * maintained; a garbage magic means the rail actually collapsed. That
 * distinction is the whole question, and it does not depend on the bootloader
 * leaving MCUSR alone.
 */

/* Call FIRST in setup(), before anything else can use RAM. */
void resetCauseBeginBoot();

/* Print the verdict. Needs Serial.begin() to have run. */
void resetCauseReport();
