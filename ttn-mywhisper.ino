#include <TheThingsNetwork.h>

/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preconfigured (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate a DevAddr, NwkSKey and
   AppSKey. Each device should have their own unique values for these
   fields.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#define RFID
#ifdef RFID
#include <MFRC522.h>
#define NODE_ID 10
constexpr uint8_t RST_PIN = 9;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = 4;     // Configurable, see typical pin layout above
 
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class

MFRC522::MIFARE_Key key; 
 uint8_t nuidPICC[4];
#endif
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
//#include <T2WhisperNode.h>
#include <LowPower.h>

//#define BMP_SENSOR
#ifdef BMP_SENSOR
Adafruit_BMP085 bmp;
#endif


//#define DHT_SENSOR

#ifdef DHT_SENSOR
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT22 //DHT11, DHT21, DHT22

DHT dht(DHTPIN, DHTTYPE);
#endif



//#define bibernode2
#ifdef bibernode2
static const u1_t NWKSKEY[16] = { 0xA2, 0x90, 0xE6, 0x58, 0xD0, 0x5A, 0x1E, 0x1B, 0x99, 0x84, 0x6C, 0xD0, 0xD1, 0x97, 0xFD, 0x1C };

static const u1_t APPSKEY[16] ={ 0x85, 0xB1, 0x18, 0x2A, 0xA1, 0x90, 0x82, 0xD4, 0xD5, 0xDA, 0x3F, 0x48, 0xF3, 0x6C, 0xCF, 0x56 };

static const u4_t DEVADDR = 0x260115AE;
#endif




#define bibernode1
#ifdef bibernode1
static const u1_t NWKSKEY[16] = { 0x2D, 0x89, 0xF5, 0x50, 0x64, 0x06, 0x3B, 0xA3, 0x67, 0xB8, 0x71, 0x80, 0x63, 0x6C, 0xB2, 0xA1 };

static const u1_t APPSKEY[16] = { 0x73, 0x8A, 0xD7, 0xD0, 0x17, 0x67, 0x5D, 0xF2, 0xC2, 0x11, 0xC4, 0x71, 0x3A, 0x97, 0x4D, 0x7E };

static const u4_t DEVADDR = 0x260118EC;
#endif


//#define newbiberachnode
#ifdef newbiberachnode
static const u1_t NWKSKEY[16] = { 0xA6, 0xAD, 0xC0, 0xDA, 0x26, 0xA1, 0x02, 0xF4, 0x56, 0x6E, 0x44, 0x6B, 0x6E, 0xF8, 0x29, 0x93 };

static const u1_t APPSKEY[16] = { 0xE0, 0x8C, 0x92, 0x11, 0x61, 0x36, 0xE2, 0xC6, 0xBF, 0x92, 0x0D, 0xF5, 0x04, 0x4C, 0x94, 0xF2 };

static const u4_t DEVADDR = 0x260116E5;
#endif


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 6;
int SLEEP_TIME = 0;

// Pin mapping
/*const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
  };
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = {2, A2, LMIC_UNUSED_PIN},
  };

  ORIGINAL WHISPER
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = {2, A2, LMIC_UNUSED_PIN},
  };
  
mywhisper !
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {2, 7, 9},
};
*/

//Adafruit lora module
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 8,
  .dio = {2, 3, LMIC_UNUSED_PIN},
};
/*
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = {2, A2, A3},
  };*/
  
  
  
#ifdef RFID  
/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
 
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}
#endif
long readVcc()
{
  // Also encode the battery level into the next 2 bytes
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  // Vcc (in mV); 1125300 = 1.1*1023*1000
  long vcc = 1125300L / ((high << 8) | low);
  return (vcc);

  /*  data[4] = vcc >> 8;
    data[5] = vcc;
  */
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));

        
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));

        SLEEP_TIME = LMIC.frame[LMIC.dataBeg]; //*100 :-)
        Serial.print("New Sleeptime: ");
        Serial.println(SLEEP_TIME);
      }
      

      
      // Schedule next transmission
     // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(SLEEP_TIME), do_send);

     // Serial.println ("let there be sleep");
      // LMIC_shutdown();
     /*
      for (int i = 0; i < SLEEP_TIME / SLEEP_8S; i ++)
      {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        Serial.print (i);
        Serial.println(" sleep");
      }
     */
      //LMIC_reset ();
      // init_lmic();


      // Start job
    // do_send(&sendjob);

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

void do_send(osjob_t* j) {
  CayenneLPP lpp(51);
     long vcc;

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else
 
  {




    float t = 10.11;

    // Get Voltage readings from supply/battery
    // voltage = T2Utils::readVoltage(T2_WPN_VBAT_VOLTAGE, T2_WPN_VBAT_CONTROL) / 10;

    //voltage = 400.00;
#ifdef BMP_SENSOR
    t = bmp.readTemperature();
#endif

#ifdef DHT_SENSOR
    t = dht.readTemperature();  //Temperatur auslesen
    Serial.println("Temperature:");
    Serial.println (t);
#endif
    lpp.reset();

#ifndef RFID
    vcc = readVcc() / 100;
    Serial.println("Spannung: ");
    Serial.println(vcc);
   lpp.addTemperature(0, t);
    lpp.addAnalogInput(1, (float)vcc);
#else
  lpp.addDigitalInput(0,NODE_ID);
  for (int k = 1; k < 5; k++)
  {
  lpp.addDigitalInput (k, nuidPICC[k-1]);
  Serial.println( nuidPICC[k-1]); 

  }
#endif
    // Prepare upstream data transmission at the next possible time.
    #ifdef RFID
    LMIC_setTxData2(77, (char *)lpp.getBuffer(), lpp.getSize(), 0);
    #else
    LMIC_setTxData2(3, (char *)lpp.getBuffer(), lpp.getSize(), 0); 
    #endif
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}



void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);//
  delay(1000);
#endif

#ifdef BMP_SENSOR
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
#endif

#ifdef DHT_SENSOR
  dht.begin();
#endif

#ifdef RFID
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  Serial.println(F("This code scan the MIFARE Classsic NUID."));
  Serial.print(F("Using the following key:"));
  printHex(key.keyByte, MFRC522::MF_KEY_SIZE);
  #endif  
  delay (600);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);


#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  Serial.println ("cfgeu868");
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

    // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  
  // Start job
 //#ifndef RFID
   do_send(&sendjob);
 //#endif

  
}

void loop() 
{
    os_runloop_once();
  #ifdef RFID
// Look for new cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if ( ! rfid.PICC_ReadCardSerial())
    return;

  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }

  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }
   
    Serial.println(F("The NUID tag is:"));
    Serial.print(F("In hex: "));
    printHex(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    Serial.print(F("In dec: "));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    do_send(&sendjob);
    Serial.println();
  }
  else 
  {
    Serial.println(F("Card read previously.Anyhow we send :-)"));
        do_send(&sendjob);
  }

  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
  #endif

}


