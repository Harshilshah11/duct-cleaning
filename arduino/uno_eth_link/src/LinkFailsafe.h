#pragma once
#include <Arduino.h>

/*
 * LinkFailsafe — "have I heard from the ground station recently?"
 *
 * THIS IS THE REASON THE SKETCH IS MORE THAN A PARSE LOOP. A robot that keeps
 * driving on its last command after the tether dies is the failure this exists
 * to prevent. TEST IT WITH THE WHEELS OFF THE GROUND: unplug the cable
 * mid-drive and confirm both motors stop within a third of a second.
 *
 * The class only tracks state and reports EDGES; it owns no outputs. The caller
 * decides what a trip means, which keeps the safety action visible in loop()
 * instead of buried in a helper.
 */
class LinkFailsafe {
public:
    explicit LinkFailsafe(unsigned long timeoutMs);

    /* Record a valid command. Returns true ONLY on the down -> up edge, so the
     * caller can log "LINK UP" once rather than on every packet. */
    bool feed();

    /* Returns true ONCE, on the up -> down edge, when the timeout has elapsed
     * since the last feed(). Call it every pass of loop(). */
    bool expired();

    bool          isUp()  const { return _up; }
    unsigned long count() const { return _count; }

private:
    const unsigned long _timeoutMs;
    unsigned long       _lastMs;
    unsigned long       _count;
    bool                _up;
};
