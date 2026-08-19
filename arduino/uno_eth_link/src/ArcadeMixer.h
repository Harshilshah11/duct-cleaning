#pragma once
#include <Arduino.h>

/*
 * arcadeMix — stick (-1000..1000) to wheel demands (-maxPwm..maxPwm).
 *
 * THIS MUST STAY NUMERICALLY IDENTICAL TO mix() IN ground_station/uno_serial.py,
 * because BOTH feed this board: uno_motors.py mixes on the Pi and sends the "M"
 * form, while joystick_link.py sends the raw calibrated stick as "J" and expects
 * the mixing to happen here. Change one and you must change the other, or the
 * robot steers differently depending on which program is driving.
 *
 * y drives both wheels together, x drives them in opposition. Full deflection on
 * both axes would demand 2.0 from one wheel, so the PAIR is scaled down together
 * — clipping each wheel on its own instead bends the turn as the robot speeds up.
 *
 * No deadband here: the Pi has already removed it, and applying a second one
 * would silently eat part of that calibration.
 */
void arcadeMix(int x, int y, int maxPwm, int *left, int *right);
