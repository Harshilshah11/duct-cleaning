/*
 * eth_init_test2 - does the W5100 come up on a cold start? Nothing else.
 *
 * Isolates the shield bring-up from the rest of the firmware: no timer
 * prescaler change, no motor logic, no failsafe, no command parser. Same MAC,
 * IP and retry loop as the real sketch, so a difference in behaviour between
 * this and duct_cleaning_robot_firmware points at the code that is missing here.
 *
 * THE LAMP IS THE READOUT, so this can be tested on the robot with no laptop:
 *
 *     FAST blink (100 ms)  chip=51 and the IP reads back    -> shield GOOD
 *     SLOW blink (1000 ms) chip detected, config did NOT land
 *     DOUBLE blink         chip=0, detection failed          -> deaf shield
 *
 * It also answers any UDP packet on 5005 with "ACK", so the Pi can confirm the
 * link end to end without a serial cable.
 */
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <utility/w5100.h>

byte      mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
IPAddress ip(192, 168, 50, 20);
const uint8_t IP4[4] = {192, 168, 50, 20};
EthernetUDP udp;

const uint8_t LIGHT_DIR = 8, LIGHT_PWM = 9;
const uint8_t BRUSH_DIR = 2, BRUSH_PWM = 3;
const uint8_t DIR1 = 7, PWM1 = 6, DIR2 = 4, PWM2 = 5;
const uint8_t ACT_DIR = A3, ACT_PWM = A2;

char buf[64];
uint8_t verdict = 0;            // 0 = deaf, 1 = not configured, 2 = good

static bool chipDetected() { return W5100.getChip() == 51; }

static bool configured() {
    if (!chipDetected()) return false;
    uint8_t got[4];
    W5100.readSIPR(got);
    return got[0] == IP4[0] && got[1] == IP4[1] && got[2] == IP4[2] && got[3] == IP4[3];
}

void setup() {
    // Park everything that can move, brush first - its pins float from reset.
    pinMode(BRUSH_PWM, OUTPUT); digitalWrite(BRUSH_PWM, LOW);
    pinMode(BRUSH_DIR, OUTPUT); digitalWrite(BRUSH_DIR, LOW);
    pinMode(PWM1, OUTPUT); digitalWrite(PWM1, LOW);
    pinMode(DIR1, OUTPUT); digitalWrite(DIR1, LOW);
    pinMode(PWM2, OUTPUT); digitalWrite(PWM2, LOW);
    pinMode(ACT_PWM, OUTPUT); digitalWrite(ACT_PWM, LOW);
    pinMode(ACT_DIR, OUTPUT); digitalWrite(ACT_DIR, LOW);
    pinMode(LIGHT_DIR, OUTPUT); digitalWrite(LIGHT_DIR, LOW);
    pinMode(LIGHT_PWM, OUTPUT); digitalWrite(LIGHT_PWM, LOW);

    // D4 HIGH = SD deselected. It is also the right wheel's direction line,
    // harmless with PWM2 low.
    pinMode(DIR2, OUTPUT); digitalWrite(DIR2, HIGH);

    // ---- ADDED IN v2: the real firmware's outputsBegin() + safeState() ----
    // Everything the working v1 did NOT do. D13 is the headline: it is
    // LED_BUILTIN and also the SPI CLOCK the shield runs on.
    pinMode(13, OUTPUT);            // PIN_STATUS_LED == LED_BUILTIN == SCK
    digitalWrite(13, LOW);          // safeState() -> setLinkLed(false)

    // safeState() drives DIR2 LOW - the opposite of the SD-deselect above.
    digitalWrite(DIR2, LOW);

    Serial.begin(115200);
    Serial.println(F("eth_init_test2 (with outputsBegin/safeState pins)"));

    Ethernet.begin(mac, ip);
    for (uint8_t t = 1; t <= 8; t++) {
        if (!chipDetected())      Ethernet.begin(mac, ip);
        else if (!configured()) { W5100.writeMR(0x80); delay(10); Ethernet.begin(mac, ip); }
        else break;
        delay(400);
    }

    verdict = configured() ? 2 : (chipDetected() ? 1 : 0);

    Serial.print(F("chip="));
    Serial.print(W5100.getChip());
    Serial.println(verdict == 2 ? F(" VERIFIED (fast blink)")
                 : verdict == 1 ? F(" detected, NOT configured (slow blink)")
                                : F(" NOT DETECTED (double blink)"));
    udp.begin(5005);
}

static void flash(unsigned ms) {
    digitalWrite(LIGHT_PWM, HIGH); delay(ms);
    digitalWrite(LIGHT_PWM, LOW);  delay(ms);
}

void loop() {
    if (verdict == 2)      flash(100);
    else if (verdict == 1) flash(1000);
    else { flash(120); flash(120); delay(700); }

    // Answer anything, so the Pi can prove the link without a serial cable.
    int n = udp.parsePacket();
    if (n > 0) {
        int got = udp.read(buf, sizeof(buf) - 1);
        if (got < 0) got = 0;
        buf[got] = '\0';
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.print(F("ACK\n"));
        udp.endPacket();
        Serial.println(F("packet -> ACK"));
    }
}
