#include <Arduino.h>

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x0C, 0xC5, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0x05, 0xA1, 0x3B, 0xFC, 0xAF, 0x09, 0xE2, 0x06};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0x9E, 0x1B, 0xAD, 0xA4, 0x64, 0x94, 0x14, 0x54, 0x49, 0x07, 0x07, 0xF6, 0x50, 0x41, 0x3D, 0xE7};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

uint8_t coords[12];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;

//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping RFM95
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

// GPS NEO-6M
static const int RXPin = 8, TXPin = 7;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);

void get_coords()
{

    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
        while (gpsSerial.available())
        {
            char c = gpsSerial.read();
            //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
            gps.encode(c);
        }
    }

    if (gps.location.isValid())
    {

        LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
        LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

        coords[0] = (LatitudeBinary >> 16) & 0xFF;
        coords[1] = (LatitudeBinary >> 8) & 0xFF;
        coords[2] = LatitudeBinary & 0xFF;

        coords[3] = (LongitudeBinary >> 16) & 0xFF;
        coords[4] = (LongitudeBinary >> 8) & 0xFF;
        coords[5] = LongitudeBinary & 0xFF;

        altitudeGps = gps.altitude.meters();
        coords[6] = (altitudeGps >> 8) & 0xFF;
        coords[7] = altitudeGps & 0xFF;

        hdopGps = gps.hdop.value() / 10;
        coords[8] = hdopGps & 0xFF;
        coords[9] = gps.time.hour();
        coords[10] = gps.time.minute();
        coords[11] = gps.time.second();
    }
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // Prepare upstream data transmission at the next possible time.
        get_coords();
        LMIC_setTxData2(1, (uint8_t *)coords, sizeof(coords), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));

        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
        break;
    case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
            Serial.println(F("---- data start ----"));
            for (int i = 0; i < LMIC.dataLen; i++)
            {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
                {
                    Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                if (i < LMIC.dataLen - 1) 
                {
                    Serial.print(F("-"));
                }
            }
            Serial.println();
            Serial.println(F("---- data end ----"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Starting"));

    gpsSerial.begin(9600);
    delay(5000);
    gpsSerial.print("$PMTK301,2*2E\r\n"); // Select SBAS as DGPS source (RTCM)
    gpsSerial.print("$PMTK313,1*2E\r\n"); // Enable to search a SBAS satellite
    //gpsSerial.print("$PMTK513,1*28\r\n");

#ifdef VCC_ENABLE
        // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Added by Ruben: Problems with downlink and OTAA
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}