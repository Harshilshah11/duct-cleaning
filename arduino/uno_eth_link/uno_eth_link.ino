/*
 * uno_eth_link — Arduino Uno + W5100/W5500 Ethernet shield, driving a dual
 * channel motor driver from ground station joystick data (guide Steps 8-9).
 *
 *   Pi -> Uno  "CMD <seq> M <l> <r>\n"                        wheels only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act>\n"                  + actuator, sign only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush>\n"          + brush, 0 or 1
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush> <light>\n"  + light, 0..255
 *   Pi -> Uno  "CMD <seq> J <x> <y>\n"                        raw stick, -1000..1000
 *   Pi -> Uno  "CMD <seq> STOP\n"                             explicit neutral
 *   Uno -> Pi  "ACK <seq>\n"                                  back to sender addr/port
 *
 * EVERY OUTPUT RIDES IN THE SAME PACKET, as trailing fields on M. They are
 * deliberately not separate datagrams: one packet per frame keeps a SINGLE
 * failsafe clock covering wheels, actuator and brush together, and keeps the
 * traffic at one datagram per 20 ms. A drive command that stopped refreshing
 * some other output's private timer would let that output keep running after
 * the tether died — which for the rod means driving into its end stop, and for
 * the brush means a spinning brush nobody can stop.
 *
 * Trailing fields are OPTIONAL and default to 0 (see the parse in loop()), so
 * an older sender degrades to "wheels only, everything else stopped" rather
 * than to "everything else stuck at its last value".
 *
 * <act> is signed by the Pi: positive extends, negative retracts. ONLY THE SIGN
 * IS USED — this channel lost its PWM pin to the light on 2026-08-14 and now has
 * direction and nothing else, so 0 does NOT stop the rod, it retracts it. The
 * 3-position switch truth table (GPIO16/19, active-LOW) is decoded in
 * ground_station/inputs.py, NOT here:
 *     16=0 19=1 -> EXTEND     16=1 19=1 -> STOP     16=1 19=0 -> RETRACT
 * Both legs closed is a broken interlock; inputs.py calls that FAULT and
 * uno_motors.py sends 0. See applyActuator() for why that is no longer a stop.
 *
 * <brush> is a plain 0/1 from the panel's TOGGLE switch (Pi GPIO13, active-LOW
 * like every switch on that panel — closed reads as ON). The brush has no speed
 * control: a 2-position toggle cannot express one, and by the time it was added
 * the Uno had no PWM pin left to give it.
 *
 * <light> is the panel potentiometer (ADS1115 A2), scaled to 0..255 on the Pi
 * in uno_motors.py. It is unsigned — a lamp has no reverse. The pot could take
 * this job only because the actuator stopped needing a speed demand in the same
 * change; before that, one pot could not serve both.
 *
 * The ACK goes to udp.remoteIP()/remotePort(), NOT to a hardcoded Pi address:
 * the Pi's sending socket is on an ephemeral port, so a fixed reply port would
 * land nowhere. This also means any machine on the LAN can test the link.
 *
 * For the M form, arcade mixing happens on the Pi (ground_station/uno_motors.py)
 * and this sketch only applies per-wheel demands. Steering can then be retuned
 * without a reflash, and the code next to the motors stays small enough to
 * audit. The J form is the exception: joystick_link.py sends the calibrated but
 * UNMIXED stick, so mixJoystick() below does that one job — see its comment for
 * why the two must stay numerically identical.
 *
 * ---------------------------------------------------------------------------
 * MOTOR PIN MAP — chosen around the shield, NOT freely
 * ---------------------------------------------------------------------------
 *
 *   channel 1 / LEFT    DIR1 = D9        PWM1 = D3    (Timer2)
 *   channel 2 / RIGHT   DIR2 = D8        PWM2 = D6    (Timer0)
 *   linear actuator     ACT_DIR = D2     (no PWM — direction only)
 *   brush motor         BRUSH = D7       (on/off)
 *   light               LIGHT_DIR = A0   LIGHT_PWM = D5 (Timer0)
 *
 * EVERY PIN IS NOW SPOKEN FOR. D0/D1 are the USB serial this telemetry goes out
 * on, D4 and D10-D13 belong to the shield, and all four of the Uno's usable PWM
 * pins (D3, D5, D6, D9) are allocated — D9 to a direction line, which is the one
 * place a PWM pin is still spendable if something else ever needs dimming.
 * A1-A5 remain as plain digital I/O.
 *
 * REWIRED 2026-08-14: D7 is freed and the old DIR1=D7 / PWM1=D9 / PWM2=D3 map
 * is gone. DIR1 took over D9 (a direction line only needs digitalWrite, so
 * spending a PWM-capable pin on it is fine), PWM1 moved to D3, and PWM2 moved
 * to the newly used D6.
 *
 * The obvious map (PWM on D10/D11) is IMPOSSIBLE with this shield fitted: the
 * W5100/W5500 owns D10 (chip select), D11 (MOSI), D12 (MISO) and D13 (SCK) for
 * SPI, plus D4 for the microSD slot. Driving motors from D10/D11 breaks the
 * Ethernet link and the motors together. D3/D6/D8/D9 clear all of those.
 *
 * D6 IS ON TIMER0, AND THAT HAS ONE REAL CONSEQUENCE. Timer0 also generates
 * millis(), which the failsafe below is timed off. Calling analogWrite() on D6
 * does NOT disturb millis() — only changing Timer0's prescaler would, and
 * nothing here does. What it DOES do is make a 0 duty cycle unreliable: on
 * Timer0 pins, analogWrite(pin, 0) can still emit a narrow pulse every period,
 * which is enough to leave a motor creeping. So every full stop on a PWM pin
 * goes through digitalWrite(pin, LOW), never analogWrite(pin, 0). That is why
 * applyMotor() and safeState() below look asymmetric — it is deliberate, and
 * removing it reintroduces a robot that will not quite stop.
 *
 * The two channels run on different timers and therefore different default PWM
 * frequencies: D3 ~490 Hz (Timer2), D6 ~980 Hz (Timer0). Both drive the bridge
 * fine, but the channels are NOT interchangeable — if you ever raise one
 * timer's frequency, raise BOTH. Mismatched channels respond differently to the
 * same demand, which reads as a mechanical fault.
 *
 * ---------------------------------------------------------------------------
 * ADDRESSING — why .50.20 and not .1.20
 * ---------------------------------------------------------------------------
 *
 * This board must NOT be 192.168.1.20. That is the ground station Pi's own
 * wlan0 address, and a host always delivers traffic for its own address
 * locally, so every packet the Pi sent to the Uno would be answered by the Pi
 * itself and never reach the wire. The eth0 segment is 192.168.50.0/24, so the
 * Uno lives at 192.168.50.20 and ground_station/uno_link.py targets that.
 *
 * FAILSAFE — the reason this sketch is more than a print loop. If no valid
 * packet arrives for FAILSAFE_MS, safeState() stops both motors and the link is
 * marked down. A robot that keeps driving on its last command after the tether
 * dies is the failure mode this exists to prevent, so TEST IT WITH THE WHEELS
 * OFF THE GROUND: unplug the Ethernet cable mid-drive and confirm both motors
 * stop within a third of a second.
 *
 * OTHER WIRING NOTES:
 *   - The shield's microSD slot shares SPI on pin 4. An unselected SD card
 *     holds MISO and the Ethernet chip never talks. setup() parks pin 4 HIGH to
 *     deselect it, which is why that line is there even with no card in.
 *
 * The MAC below is locally administered (the 0x02 bit in the first byte) and
 * must be unique on your LAN. Newer shields ship with a real MAC on a sticker —
 * use that if yours has one.
 *
 * Build: Arduino IDE, board "Arduino Uno", stock Ethernet library.
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// --- Network (address plan from ground_station/config.py) --------------------
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
IPAddress ip(192, 168, 50, 20);
const uint16_t LISTEN_PORT = 5005;

// --- Motor driver pins (see the pin-map note above before changing) ----------
const uint8_t DIR1 = 9;    // channel 1 direction (LEFT)
const uint8_t PWM1 = 3;    // channel 1 speed, Timer2  (~490 Hz)
const uint8_t DIR2 = 8;    // channel 2 direction (RIGHT)
const uint8_t PWM2 = 6;    // channel 2 speed, Timer0  (~980 Hz, see pin note)

// --- Linear actuator: DIRECTION ONLY, no PWM (rewired 2026-08-14) ------------
// Direction comes from the ground station's 3-position actuator switch
// (GPIO16/19 on the Pi), decoded there and arriving as one signed number.
// ONLY ITS SIGN IS USED NOW: > 0 extends, <= 0 retracts.
//
// D5 USED TO BE THIS CHANNEL'S PWM AND IS NOW THE LIGHT'S. That is a real loss
// of capability, not a tidy-up: with no PWM/enable pin this channel has no speed
// control and, more importantly, no OFF. See applyActuator() for what that costs
// the failsafe before you run the rod near an end stop.
//
// D2 is a plain digital pin — it was chosen over the originally proposed D13
// because D13 is the SPI CLOCK the W5100 shield uses. The SPI peripheral drives
// SCK during every packet, so a direction line there would be pulsed by the
// Ethernet traffic itself and would corrupt the link at the same time.
const uint8_t ACT_DIR = 2;

// Flip if EXTEND on the panel drives the rod the wrong way.
const bool INVERT_ACT = false;

// --- Panel light: dimmable, added 2026-08-14 ---------------------------------
// Brightness follows the panel potentiometer (ADS1115 A2 on the Pi), scaled to
// 0..255 there and applied here. The pot was free to take this job precisely
// because the actuator above lost its PWM pin, and with it any use for a speed
// demand — the two changes are one change.
//
// A0 is an ANALOG pin driven as a plain digital output, which is legal on the
// Uno (A0 == D14) and is what makes this fit at all: every real digital pin is
// spoken for. It carries the driver channel's direction line, which a lamp does
// not actually need — see applyLight().
const uint8_t LIGHT_DIR = A0;
const uint8_t LIGHT_PWM = 5;   // Timer0 (~980 Hz) — same 0-duty caveat as D6

// --- Brush motor: single-pin on/off, added 2026-08-14 ------------------------
// Driven straight from the panel's TOGGLE switch (Pi GPIO13). One line only —
// a relay/MOSFET module, or a driver with its enable jumpered high — so there
// is no speed control here, which is all a 2-position toggle can express
// anyway. This is why the brush gets no PWM pin: by the time it was added the
// Uno had NONE left (3 and 6 are the drive motors, 5 the actuator, 9 is DIR1,
// 10/11 belong to the shield's SPI).
const uint8_t BRUSH_PIN = 7;

// Many relay boards are ACTIVE-LOW — the coil pulls in when the pin goes low,
// and such a board will run the brush the whole time the Uno is in reset if
// this is wrong. Set false for those.
const bool BRUSH_ACTIVE_HIGH = true;

// Flip either of these if a wheel spins the wrong way. Doing it here is much
// cheaper than re-soldering, and far safer than negating on the Pi — the Pi
// feeds BOTH this link and uno_serial.py, so a sign flip there would silently
// desync the two transports.
const bool INVERT_1 = false;
const bool INVERT_2 = false;

// --- Failsafe ----------------------------------------------------------------
// 300 ms: long enough to ride out a handful of dropped datagrams at the 50 Hz
// command rate (15 in a row), short enough that the robot stops within a third
// of a second of a real tether failure.
const unsigned long FAILSAFE_MS = 300;

// --- Tuning ------------------------------------------------------------------
// Below this the motor buzzes and heats without turning, so treat it as zero.
const int DEADBAND = 12;
const int MAX_PWM = 255;

// --- Buffers -----------------------------------------------------------------
// NOT UDP_TX_PACKET_MAX_SIZE. That constant is 24 bytes in the stock library and
// silently truncates anything longer, which looks exactly like a garbled link.
// 96 bytes is affordable out of the Uno's 2 KB of SRAM; keep the Pi side's
// MAX_PAYLOAD below it.
const uint16_t RX_BUFFER = 96;
char packet[RX_BUFFER];

EthernetUDP udp;

unsigned long lastPacketMs = 0;
bool linkUp = false;
uint16_t lastSeq = 0;
unsigned long packetsReceived = 0;

// Bench telemetry over USB serial. LINK UP / LINK DOWN alone cannot tell
// "commands are flowing" from "nothing ever arrived" — both are silent in the
// second case, because LINK DOWN only fires on a transition out of LINK UP.
// These print the demand actually applied, which is also what you read when
// deciding whether INVERT_1 / INVERT_2 need flipping.
int curL = 0;
int curR = 0;
int curA = 0;
int curB = 0;
int curLight = 0;
int printedL = 0;
int printedR = 0;
int printedA = 0;
int printedB = 0;
int printedLight = 0;
unsigned long lastPrintMs = 0;

const uint8_t STATUS_LED = LED_BUILTIN;

/* Apply one motor channel. Sign picks direction, magnitude becomes PWM. */
void applyMotor(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert) {
  if (invert) demand = -demand;
  if (demand > MAX_PWM) demand = MAX_PWM;
  if (demand < -MAX_PWM) demand = -MAX_PWM;
  if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

  // Direction is set BEFORE the new PWM value. The other order spends a few
  // microseconds driving the old direction at the new speed, which is a current
  // spike through the bridge on every reversal.
  digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);

  int duty = demand >= 0 ? demand : -demand;
  if (duty == 0) {
    // NOT analogWrite(pwmPin, 0) — on a Timer0 pin (PWM2 = D6) that can still
    // emit a narrow pulse each period and leave the motor creeping. See the
    // pin-map note at the top.
    digitalWrite(pwmPin, LOW);
  } else {
    analogWrite(pwmPin, duty);
  }
}

/* Brush motor: on or off, nothing in between. Any non-zero demand means ON, so
 * the Pi can send a plain 0/1 and this still behaves if someone later sends a
 * speed there by mistake. */
void applyBrush(int on) {
  bool run = (on != 0);
  digitalWrite(BRUSH_PIN, (run == BRUSH_ACTIVE_HIGH) ? HIGH : LOW);
}

/* Linear actuator: DIRECTION ONLY. Sign is the whole signal — >0 extends, <=0
 * retracts — because D5, this channel's old PWM pin, now dims the light.
 *
 * READ THIS BEFORE TRUSTING THE FAILSAFE. With no PWM or enable line, a demand
 * of 0 cannot mean "hold still"; it can only mean "drive the other way". This
 * pin picks a direction and nothing in this sketch can cut the rod's power.
 * safeState() parks D2 LOW, which brings the rod to rest ONLY if the driver's
 * enable is tied to something that dies with it. If that enable is jumpered
 * permanently high, the rod keeps travelling after a tether failure until it
 * hits its end stop, and no change here can prevent that. Wire the driver a real
 * enable line if the rod must be able to stop.
 */
void applyActuator(int demand, bool invert) {
  if (invert) demand = -demand;
  digitalWrite(ACT_DIR, demand > 0 ? HIGH : LOW);
}

/* Panel light: brightness 0..255, straight off the potentiometer.
 *
 * LIGHT_DIR is held HIGH rather than steered. A lamp has no reverse, but a motor
 * driver channel used as a dimmer still wants a defined polarity on its
 * direction input; on a driver that ignores the pin this costs nothing.
 *
 * NO DEADBAND here, deliberately unlike applyMotor(). A motor below ~12 buzzes
 * and heats without turning, so folding that to zero is right. A lamp at 12/255
 * is simply dim, and applying the same rule would give the pot a dead patch at
 * the bottom of its travel that reads as a broken knob.
 */
void applyLight(int level) {
  if (level < 0) level = 0;
  if (level > MAX_PWM) level = MAX_PWM;

  digitalWrite(LIGHT_DIR, HIGH);
  if (level == 0) {
    // D5 is a Timer0 pin, so analogWrite(pin, 0) can still emit a narrow pulse
    // every period. On a motor that is a creep; on a lamp it is a faint glow
    // that will not go out. digitalWrite is the only certain dark.
    digitalWrite(LIGHT_PWM, LOW);
  } else {
    analogWrite(LIGHT_PWM, level);
  }
}

/* Both motors to neutral. Runs on every failsafe trip, so it is unconditional
 * and must not depend on any prior state. The LED doubles as a link lamp: lit
 * means commands are arriving, dark means failsafe. */
void safeState() {
  // digitalWrite, not analogWrite(pin, 0): PWM2 is on Timer0, where a 0 duty is
  // not a guaranteed dead level. This is the one place that must be certain.
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  // The actuator is parked, not stopped — D2 is a direction line and this
  // channel has no enable pin since D5 became the light. Read applyActuator()
  // before assuming this brings the rod to rest; on a driver whose enable is
  // jumpered high, it does not.
  digitalWrite(ACT_DIR, LOW);
  // Brush off too — via applyBrush so an active-LOW module gets the right
  // level. A spinning brush is the loudest thing on the robot; it must not be
  // what survives a failsafe.
  applyBrush(0);
  // Light out. It is the one output here that poses no motion hazard, so
  // leaving it lit was tempting — but the cameras stream over the SAME tether,
  // so once the link is down there is nobody left to see by it, and this rig
  // already browns out under load (the Pi logs under-voltage). Dark is cheaper.
  applyLight(0);
  digitalWrite(STATUS_LED, LOW);
}

/* Arcade mix for the "J" form: stick (-1000..1000) -> wheels (-255..255).
 *
 * Mirrors mix() in ground_station/uno_serial.py exactly, because BOTH feed this
 * board: uno_motors.py mixes on the Pi and sends M, while joystick_link.py
 * sends the raw calibrated stick as J and expects the mixing here. If you
 * change one, change the other, or the robot steers differently depending on
 * which program is driving.
 *
 * y drives both wheels together, x drives them in opposition. Full deflection
 * on both axes would demand 2.0 from one wheel, so the PAIR is scaled down
 * together — clipping each wheel on its own instead bends the turn as the robot
 * speeds up. No deadband here: the Pi has already removed it, and applying a
 * second one would silently eat part of that calibration.
 */
void mixJoystick(int x, int y, int *left, int *right) {
  long l = (long)y + (long)x;
  long r = (long)y - (long)x;

  long peak = 1000;                    // == max(1.0, ...) in the Python
  if (labs(l) > peak) peak = labs(l);
  if (labs(r) > peak) peak = labs(r);

  *left = (int)(l * MAX_PWM / peak);
  *right = (int)(r * MAX_PWM / peak);
}

void setup() {
  Serial.begin(115200);

  // Outputs are driven to a stopped state BEFORE they become outputs, so the
  // pins cannot glitch high in the gap between pinMode and the first write.
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(ACT_DIR, LOW);
  digitalWrite(LIGHT_DIR, LOW);
  digitalWrite(LIGHT_PWM, LOW);
  // The brush's OFF level is written BEFORE pinMode, and on an active-LOW
  // module that level is HIGH. Writing it first switches on the input pull-up,
  // which holds the module off across the gap; leaving the pin floating there
  // is exactly how a relay board ends up running the brush during reset.
  digitalWrite(BRUSH_PIN, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(BRUSH_PIN, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ACT_DIR, OUTPUT);
  // A0 as a digital output. pinMode(A0, OUTPUT) is the whole ceremony — nothing
  // else is needed to stop it being an ADC input.
  pinMode(LIGHT_DIR, OUTPUT);
  pinMode(LIGHT_PWM, OUTPUT);
  // NOTE: STATUS_LED is LED_BUILTIN = D13, which is also the SPI clock the
  // shield uses. With the shield fitted this lamp tracks Ethernet traffic
  // rather than link state, and is not a reliable indicator. Harmless, but do
  // not read anything into it — and never put a real signal on D13.
  pinMode(STATUS_LED, OUTPUT);
  safeState();

  // Deselect the shield's SD card before Ethernet.begin() touches the bus.
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Static IP — no DHCP. Ethernet.begin(mac, ip) cannot fail or block, unlike
  // the DHCP form which stalls ~60 s when no server answers. On a point-to-point
  // tether there is no DHCP server at all, so static is the only sane choice.
  Ethernet.begin(mac, ip);

  // Report hardware trouble rather than sitting mute.
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("ERROR: no Ethernet shield detected - check it is seated"));
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("WARN: Ethernet cable is not connected"));
  }

  udp.begin(LISTEN_PORT);

  Serial.print(F("uno_eth_link listening on "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.println(LISTEN_PORT);
  Serial.println(F("DIR1=D9 PWM1=D3 (left)  DIR2=D8 PWM2=D6 (right)"));
  Serial.println(F("ACT_DIR=D2 (linear actuator, DIRECTION ONLY - no stop)"));
  Serial.println(F("BRUSH=D7 on/off (panel TOGGLE, Pi GPIO13)"));
  Serial.println(F("LIGHT_DIR=A0 LIGHT_PWM=D5 (pot-dimmed, 0-255)"));
  Serial.print(F("failsafe after "));
  Serial.print(FAILSAFE_MS);
  Serial.println(F(" ms of silence"));
}

void loop() {
  int size = udp.parsePacket();
  if (size > 0) {
    int n = udp.read(packet, RX_BUFFER - 1);
    if (n < 0) {
      n = 0;
    }
    packet[n] = '\0';

    // Anything longer than the buffer is still queued in the W5x00; drop the
    // remainder so the next parsePacket() starts on a clean packet boundary
    // instead of returning the tail as a bogus command.
    if (size > n) {
      udp.flush();
      Serial.print(F("WARN: packet truncated, "));
      Serial.print(size);
      Serial.println(F(" bytes"));
    }

    unsigned int seq = 0;
    int left = 0;
    int right = 0;
    bool understood = false;

    int jx = 0;
    int jy = 0;
    int act = 0;
    int brush = 0;
    int light = 0;

    // ONE pattern for every length of M, judged by how many fields sscanf
    // actually filled. Testing the short patterns as separate branches does not
    // work: "CMD %u M %d %d" happily matches the LONGER string too — sscanf
    // simply stops early and returns 3 — so a shorter branch placed first would
    // silently swallow <act>/<brush> and freeze them at their last value.
    //
    // Anything the sender omitted stays 0, which is also the safe default: a
    // sender that cannot talk about the actuator, the brush or the light must
    // never be able to leave any of them running.
    int nf = sscanf(packet, "CMD %u M %d %d %d %d %d",
                    &seq, &left, &right, &act, &brush, &light);
    if (nf >= 3) {
      if (nf < 4) act = 0;
      if (nf < 5) brush = 0;
      if (nf < 6) light = 0;
      applyMotor(DIR1, PWM1, left, INVERT_1);
      applyMotor(DIR2, PWM2, right, INVERT_2);
      applyActuator(act, INVERT_ACT);
      applyBrush(brush);
      applyLight(light);
      curL = left;
      curR = right;
      curA = act;
      curB = brush;
      curLight = light;
      understood = true;
    } else if (sscanf(packet, "CMD %u J %d %d", &seq, &jx, &jy) == 3) {
      // Raw stick from joystick_link.py, already centred and deadbanded on the
      // Pi but NOT mixed. Mixing happens here for this form only.
      mixJoystick(jx, jy, &left, &right);
      applyMotor(DIR1, PWM1, left, INVERT_1);
      applyMotor(DIR2, PWM2, right, INVERT_2);
      // J carries no actuator, brush or light, so none of them run.
      applyActuator(0, INVERT_ACT);
      applyBrush(0);
      applyLight(0);
      curL = left;
      curR = right;
      curA = 0;
      curB = 0;
      curLight = 0;
      understood = true;
    } else if (sscanf(packet, "CMD %u STOP", &seq) == 1) {
      curL = 0;
      curR = 0;
      curA = 0;
      curB = 0;
      curLight = 0;
      safeState();
      understood = true;
    } else if (sscanf(packet, "CMD %u", &seq) == 1) {
      // A bare keepalive with no payload. Valid: it proves the link is alive
      // and refreshes the failsafe without changing the motor demand.
      understood = true;
    }

    if (understood) {
      lastSeq = (uint16_t)seq;
      packetsReceived++;
      lastPacketMs = millis();

      if (!linkUp) {
        linkUp = true;
        Serial.println(F("LINK UP"));
      }
      digitalWrite(STATUS_LED, HIGH);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(F("ACK "));
      udp.print(lastSeq);
      udp.print(F("\n"));
      udp.endPacket();
    } else {
      Serial.print(F("WARN: unparsable packet: "));
      Serial.println(packet);
    }
  }

  // millis() subtraction, never `millis() > last + FAILSAFE_MS`. Unsigned
  // wraparound at ~49 days makes the additive form compare wrong exactly once,
  // and this form stays correct across the rollover.
  if (linkUp && (millis() - lastPacketMs) >= FAILSAFE_MS) {
    linkUp = false;
    curL = 0;
    curR = 0;
    curA = 0;
    curB = 0;
    safeState();
    Serial.print(F("LINK DOWN - failsafe after "));
    Serial.print(packetsReceived);
    Serial.println(F(" packets"));
  }

  // Report on change (rate-limited, or a moving stick floods the port at 50 Hz)
  // and on a 3 s heartbeat, so a resting link still proves itself.
  unsigned long now = millis();
  bool changed = (curL != printedL || curR != printedR || curA != printedA
                  || curB != printedB || curLight != printedLight)
                 && (now - lastPrintMs) > 200;
  if (changed || (now - lastPrintMs) > 3000) {
    printedL = curL;
    printedR = curR;
    printedA = curA;
    printedB = curB;
    printedLight = curLight;
    lastPrintMs = now;
    Serial.print(F("L="));
    Serial.print(curL);
    Serial.print(F(" R="));
    Serial.print(curR);
    Serial.print(F(" ACT="));
    Serial.print(curA);
    Serial.print(F(" BRUSH="));
    Serial.print(curB ? F("ON") : F("off"));
    Serial.print(F(" LIGHT="));
    Serial.print(curLight);
    Serial.print(F("  pkts="));
    Serial.println(packetsReceived);
  }
}
