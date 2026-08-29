/*
 * uno_serial_drive — Arduino Uno on the Pi's USB tether, driving a dual-channel
 * motor driver from ground station joystick data.
 *
 *   Pi  -> Uno   "CMD <seq> M <left> <right>\n"    left/right = -255..255
 *   Pi  -> Uno   "CMD <seq> STOP\n"                explicit neutral
 *   Uno -> Pi    "ACK <seq>\n"
 *
 * Same framing as uno_eth_link.ino so the two links stay interchangeable, but
 * over USB CDC serial instead of UDP. The Pi does the arcade mixing (see
 * ground_station/uno_serial.py) and sends per-motor demands; this sketch only
 * applies them. Keeping the mixing off the Uno means steering can be retuned
 * without a reflash, and it keeps the code that runs next to the motors small
 * enough to audit in one sitting.
 *
 * PIN MAP — the rig's wiring, confirmed by the operator 2026-08-27:
 *
 *   channel 1 / LEFT    DIR1 = D7    PWM1 = D6   (Timer0 OC0A)
 *   channel 2 / RIGHT   DIR2 = D4    PWM2 = D5   (Timer0 OC0B)
 *
 * BROUGHT INTO LINE 2026-08-27. This sketch used to carry the original
 * 2026-08-14 map, DIR1=D9 / PWM1=D10 / PWM2=D11, from back when it was the only
 * sketch and the shield was off. The rig has since been rewired and the other
 * two sketches moved with it; this one had not. A robot has ONE wiring loom, so
 * a sketch holding a different map is not a variant, it is a trap: flash it and
 * the left wheel drives from a pin that is now the brush's gate.
 *
 * THE OLD MAP'S SHIELD WARNING IS GONE WITH IT, and that is an improvement
 * rather than a loss. D10 and D11 collided with the W5100/W5500 directly (D10 is
 * its chip select, D11 its MOSI), so the old map could only be used with the
 * shield off. D3 and D6 clear the shield entirely, so this sketch no longer
 * cares whether the shield is fitted. D4 still belongs to the shield's microSD
 * slot; nothing here touches it.
 *
 * DIR1 SPENDS NO PWM PIN. A direction line only needs digitalWrite, so it sits
 * on A1 - an analog-capable pin used as plain digital I/O - and leaves every
 * timer pin for something that actually needs to be dimmed. That is the same
 * reasoning the old map used to justify D9, applied to the current wiring.
 *
 * BOTH CHANNELS ARE ON TIMER0 SINCE THE 2026-08-29 REWIRE, and that fixes a
 * defect this sketch used to carry. PWM1 was D3 (Timer2, ~490 Hz) and PWM2 was
 * D6 (Timer0, ~980 Hz) - NOT EQUAL, and channels on different frequencies answer
 * the same demand differently, which reads as a pull to one side on a straight
 * run. D6 and D5 are OC0A and OC0B of the same timer, so they cannot disagree:
 * one prescaler governs both. This sketch still leaves Timer0 at the stock
 * prescaler, so both run at about 980 Hz - equal, which is what mattered.
 *
 * THIS SKETCH DRIVES WHEELS ONLY - no brush, no actuator, no light. It predates
 * all three. uno_usb_link is the current USB-tether build and drives everything;
 * prefer it unless you specifically want something this small to audit.
 *
 * FAILSAFE — the reason this is more than a parse loop. If no valid command
 * arrives for FAILSAFE_MS, safeState() stops both motors. A tethered robot that
 * keeps driving on its last command after the USB cable pops out is exactly the
 * failure this prevents. TEST IT WITH THE WHEELS OFF THE GROUND: unplug the USB
 * cable mid-drive and confirm both motors stop within a third of a second.
 *
 * Build: Arduino IDE, board "Arduino Uno", no external libraries.
 */

// --- Motor driver pins -------------------------------------------------------
const uint8_t DIR1 = 7;    // channel 1 direction  (LEFT)  - was A1, rewired 2026-08-29
const uint8_t PWM1 = 6;    // channel 1 speed, Timer0 OC0A  - was D3
// D4 IS THE SHIELD'S microSD CHIP SELECT, and it goes LOW when the right wheel
// runs in the negative direction. Harmless with the slot EMPTY, which is how
// this board must be run. See the note in uno_eth_link for the full story.
const uint8_t DIR2 = 4;    // channel 2 direction  (RIGHT) - was D8; SD CS
const uint8_t PWM2 = 5;    // channel 2 speed, Timer0 OC0B  - was D6, SAME timer as PWM1

const uint8_t STATUS_LED = LED_BUILTIN;

// --- Failsafe ----------------------------------------------------------------
// 300 ms, matching uno_eth_link.ino: long enough to ride out a handful of missed
// frames at the 50 Hz command rate, short enough that the robot stops within a
// third of a second of a real tether failure.
const unsigned long FAILSAFE_MS = 300;

// --- Tuning ------------------------------------------------------------------
// Below this the motor buzzes and heats without turning, so treat it as zero.
// Raise it if the drivetrain stalls at low demand; lower it for finer crawl.
const int DEADBAND = 12;

// Hard ceiling on demand. Drop this to tame a robot that is too quick to drive
// safely indoors — it scales everything, so steering stays proportional.
const int MAX_PWM = 255;

// --- Serial framing ----------------------------------------------------------
// Uno has 2 KB of SRAM total; 64 bytes is plenty for "CMD 65535 M -255 -255"
// and leaves room for everything else.
const uint8_t RX_BUFFER = 64;
char line[RX_BUFFER];
uint8_t lineLen = 0;

unsigned long lastCommandMs = 0;
bool linkUp = false;
uint16_t lastSeq = 0;
unsigned long commandsReceived = 0;

int currentLeft = 0;
int currentRight = 0;

/* Apply one motor channel. Sign picks direction, magnitude becomes PWM. */
void applyMotor(uint8_t dirPin, uint8_t pwmPin, int demand) {
  if (demand > MAX_PWM) demand = MAX_PWM;
  if (demand < -MAX_PWM) demand = -MAX_PWM;
  if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

  // Direction is set BEFORE the new PWM value. Doing it the other way round
  // spends a few microseconds driving the old direction at the new speed, which
  // is a current spike through the bridge on every reversal.
  digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, demand >= 0 ? demand : -demand);
}

/* Both motors to neutral. Runs on every failsafe trip, so it must be
 * unconditional and must not depend on any prior state. */
void safeState() {
  currentLeft = 0;
  currentRight = 0;
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(STATUS_LED, LOW);
}

/* Print a pin the way the schematic names it. Added 2026-08-29 with the rewire,
 * because the banner below was a hand-typed literal still naming the ORIGINAL
 * 2026-08-14 map - D9/D10/D8/D11 - two rewires after it stopped being true. A
 * banner exists to catch a stale board; one that is typed out restates the bug
 * it was meant to reveal, and this repo has now been bitten by that three times.
 * Derived from the constants, it cannot go stale again. */
void printPin(uint8_t pin) {
  if (pin >= A0) {
    Serial.print('A');
    Serial.print(pin - A0);
  } else {
    Serial.print('D');
    Serial.print(pin);
  }
}

void setup() {
  // Outputs are driven to a stopped state BEFORE they are made outputs, so the
  // pins cannot glitch high for the instant between pinMode and the first write.
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  safeState();

  Serial.begin(115200);
  Serial.println(F("uno_serial_drive ready"));
  Serial.print(F("DIR1="));          printPin(DIR1);
  Serial.print(F(" PWM1="));         printPin(PWM1);
  Serial.print(F(" (left)  DIR2=")); printPin(DIR2);
  Serial.print(F(" PWM2="));         printPin(PWM2);
  Serial.print(F(" (right)  failsafe "));
  Serial.print(FAILSAFE_MS);
  Serial.println(F(" ms"));
}

/* Parse one complete line and act on it. */
void handleLine(char *buf) {
  unsigned int seq = 0;
  int left = 0;
  int right = 0;

  // "CMD <seq> M <left> <right>" — the common case, checked first.
  if (sscanf(buf, "CMD %u M %d %d", &seq, &left, &right) == 3) {
    currentLeft = left;
    currentRight = right;
    applyMotor(DIR1, PWM1, left);
    applyMotor(DIR2, PWM2, right);
  } else if (sscanf(buf, "CMD %u STOP", &seq) == 1) {
    safeState();
  } else {
    Serial.print(F("WARN: unparsable: "));
    Serial.println(buf);
    return;
  }

  lastSeq = (uint16_t)seq;
  commandsReceived++;
  lastCommandMs = millis();

  if (!linkUp) {
    linkUp = true;
    Serial.println(F("LINK UP"));
  }
  digitalWrite(STATUS_LED, HIGH);

  Serial.print(F("ACK "));
  Serial.println(lastSeq);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;                     // tolerate CRLF senders
    }
    if (c == '\n') {
      line[lineLen] = '\0';
      if (lineLen > 0) {
        handleLine(line);
      }
      lineLen = 0;
      continue;
    }
    if (lineLen < RX_BUFFER - 1) {
      line[lineLen++] = c;
    } else {
      // Overlong line: drop it rather than let it wrap and fabricate a command
      // out of two halves. Resync happens at the next newline.
      lineLen = 0;
      Serial.println(F("WARN: line too long, dropped"));
    }
  }

  // millis() subtraction, never `millis() > last + FAILSAFE_MS`. Unsigned
  // wraparound at ~49 days makes the additive form compare wrong exactly once,
  // and this form stays correct across the rollover.
  if (linkUp && (millis() - lastCommandMs) >= FAILSAFE_MS) {
    linkUp = false;
    safeState();
    Serial.print(F("LINK DOWN - failsafe after "));
    Serial.print(commandsReceived);
    Serial.println(F(" commands"));
  }
}
