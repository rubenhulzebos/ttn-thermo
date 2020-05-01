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

/*
 *  MAX6675 Module   ==>   Arduino
 *    CS             ==>     D10
 *    SO             ==>     D12 -> D9
 *    SCK            ==>     D13 -> D8
 *    Vcc            ==>     Vcc (5v OK)
 *    Gnd            ==>     Gnd
 * */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 9
#define DS1820vcc 8
#define DS1820gnd 7

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress DS1820addr;

#define MAX6675_CS   10
#define MAX6675_SO   9
#define MAX6675_SCK  8

//uint8_t coords[12];
uint8_t temperatureData[2];

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x0F, 0xE7, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x78, 0xC7, 0x2A, 0x68, 0xED, 0x83, 0xF0 };
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xAF, 0x84, 0xC0, 0x98, 0x5E, 0xB7, 0xF6, 0xA2, 0xC0, 0x69, 0x1A, 0x0F, 0x1F, 0xCC, 0xE0, 0x64 };
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

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

double readThermocouple() {

  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}

// function to print a device address
void printDS1820Address(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printDS1820Address(deviceAddress);
  Serial.println(" ");
}

void powerOnDS1820(void)
{
  digitalWrite(DS1820vcc, HIGH);
  digitalWrite(DS1820gnd, LOW);
}

void powerOffDS1820(void)
{
  digitalWrite(DS1820vcc, LOW);
  digitalWrite(DS1820gnd, LOW);
}

void get_temp()
{
  uint16_t thermodata = readThermocouple();
  Serial.print(thermodata);
  Serial.println('c');

  temperatureData[0] = (uint8_t)thermodata;
  temperatureData[1] = (uint8_t)(thermodata >> 8);
}

void get_tempDS1820()
{
  Serial.print("Requesting temperatures...");
  powerOnDS1820();
  sensors.requestTemperatures();
  Serial.println("DONE");

 

  float tempC = sensors.getTempC(DS1820addr);
  Serial.print("Temp C: ");
  Serial.println(tempC);

  tempC = tempC * 100;

  uint16_t tempC16;
  tempC16 = (uint16_t)tempC;
  
  Serial.print("tempC16: ");
  Serial.println(tempC16);

  temperatureData[0] = (uint8_t)tempC16;
  temperatureData[1] = (uint8_t)(tempC16 >> 8);

  Serial.print("Byte 0: ");
  Serial.println(temperatureData[0], HEX);

  Serial.print("Byte 1: ");
  Serial.println(temperatureData[1], HEX);

  powerOffDS1820();
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
        //get_temp();
        get_tempDS1820();
        //get_coords();
        //LMIC_setTxData2(1, (uint8_t *)coords, sizeof(coords), 0);
        LMIC_setTxData2(1, (uint8_t *)temperatureData, sizeof(temperatureData), 0);
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



void setupDS1820()
{
  pinMode(DS1820vcc, OUTPUT); 
  pinMode(DS1820gnd, OUTPUT); 
  powerOnDS1820();

  // Start up the library
  sensors.begin();
  
  // locate devices on the bus
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // search for devices on the bus and assign based on an index.
  if (!sensors.getAddress(DS1820addr, 0)) Serial.println("Unable to find address for Device 0"); 
  
  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printDS1820Address(DS1820addr);
  Serial.println();
  powerOffDS1820();
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Starting"));

    setupDS1820();

    delay(2000);
    
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