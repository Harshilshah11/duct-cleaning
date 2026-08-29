#pragma once
#include <Arduino.h>

/* MCUSR must be read before the C runtime clears it (.init3). The .noinit
 * counter survives a reset but not a power loss, so an intact magic word means
 * the rail held - which is how a brown-out is told from a true power cut. */
void resetCauseBeginBoot();
void resetCauseReport();

/* Carried across a reset in .noinit, so a boot that failed on the barrel jack
 * can be read out after plugging USB in (which resets the board). */
extern uint16_t prevVerdict, prevPackets, prevServiceCount, prevChip, prevLinkUp;
void resetCauseStash(uint16_t verdict, uint16_t packets, uint16_t services,
                     uint16_t chip, uint16_t up);
