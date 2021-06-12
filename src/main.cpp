#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <config.h>
#include <devices.h>

TinyGPS gps;
SoftwareSerial ss(GPS_RX, GPS_TX);

union FBConv {
  float f_value;
  uint8_t b_value[4];
};

union ULBConv {
  unsigned long ul_value;
  uint8_t b_value[4];
};

FBConv u_lat, u_lon, u_alt;
ULBConv u_hdop;

unsigned long age = TinyGPS::GPS_INVALID_AGE;
bool lmic_txrx = false;

uint8_t tx_power = 14;
uint8_t tx_sf = 7;

static osjob_t sendjob;

const unsigned TX_INTERVAL = 1;

const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {8, 9, LMIC_UNUSED_PIN},
};

static void smartdelay(unsigned long ms);
static void toggle_indicator_led();
static void do_send(osjob_t* j);
#ifdef UART_DEBUG
static void printHex2(unsigned v);
#endif

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  pinMode(INDICATE_LED, OUTPUT);
  digitalWrite(INDICATE_LED, LOW);
  #ifdef UART_DEBUG
  Serial.println(F("Starting"));
  #endif
  os_init();
  #ifdef UART_DEBUG
  Serial.println(F("OS INIT"));
  #endif
  digitalWrite(INDICATE_LED, LOW);
  for (int i = 0; i < 6; i++) {
      toggle_indicator_led();
      delay(200);
  }
  LMIC_reset();
  LMIC_setLinkCheckMode(0);
  LMIC_setAdrMode(0);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  do_send(&sendjob);
}

void loop() {
  if (!lmic_txrx) {
    gps.f_get_position(&u_lat.f_value, &u_lon.f_value, &age);

    if ( (gps.hdop() != TinyGPS::GPS_INVALID_HDOP) && (u_alt.f_value != TinyGPS::GPS_INVALID_F_ANGLE) && (u_alt.f_value != TinyGPS::GPS_INVALID_F_ANGLE) && (gps.f_altitude() != TinyGPS::GPS_INVALID_F_ALTITUDE) && (age != TinyGPS::GPS_INVALID_AGE) ) {
      u_alt.f_value = gps.f_altitude();
      u_hdop.ul_value = gps.hdop();
    }
    else {
      age = TinyGPS::GPS_INVALID_AGE;
    }

    #ifdef UART_DEBUG
    Serial.print(u_hdop.ul_value);
    Serial.print(" ");
    Serial.print(u_lat.f_value);
    Serial.print(" ");
    Serial.print(u_lon.f_value);
    Serial.print(" ");
    Serial.print(u_alt.f_value);
    Serial.print(" ");
    Serial.print(age);
    Serial.println("");
    #endif
    smartdelay(100);
  }
  os_runloop_once();
}

static void toggle_indicator_led() {
    if (digitalRead(INDICATE_LED)) {
        digitalWrite(INDICATE_LED, LOW);
    }
    else {
        digitalWrite(INDICATE_LED, HIGH);
    }
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

#ifdef UART_DEBUG
static void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}
#endif

static void do_send(osjob_t* j) {
  lmic_txrx = true;
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    #ifdef UART_DEBUG
    Serial.println(F("OP_TXRXPEND, not sending"));
    #endif
  }
  else {

    switch (tx_sf)
    {
    case 8:
      LMIC_setDrTxpow(DR_SF10, tx_power);
      break;
    case 9:
      LMIC_setDrTxpow(DR_SF9, tx_power);
      break;
    case 10:
      LMIC_setDrTxpow(DR_SF10, tx_power);
      break;
    case 11:
      LMIC_setDrTxpow(DR_SF11, tx_power);
      break;
    case 12:
      LMIC_setDrTxpow(DR_SF12, tx_power);
      break;
    default:
      LMIC_setDrTxpow(DR_SF7, tx_power);;
    }
    if (age > GPS_MAXAGE) {
      uint8_t mydata[] = { 0x00 };
      LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    }
    else {
      uint8_t mydata[] = { u_lat.b_value[0], u_lat.b_value[1], u_lat.b_value[2], u_lat.b_value[3], u_lon.b_value[0], u_lon.b_value[1], u_lon.b_value[2], u_lon.b_value[3], u_alt.b_value[0], u_alt.b_value[1], u_alt.b_value[2], u_alt.b_value[3], u_hdop.b_value[0], u_hdop.b_value[1], u_hdop.b_value[2], u_hdop.b_value[3] };
      LMIC_setTxData2(2, mydata, sizeof(mydata), 0);
    }
  }
}

void onEvent (ev_t ev) {
    #ifdef UART_DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
          #ifdef UART_DEBUG
          Serial.println(F("EV_SCAN_TIMEOUT"));
          #endif
          break;
        case EV_BEACON_FOUND:
          #ifdef UART_DEBUG
          Serial.println(F("EV_BEACON_FOUND"));
          #endif
          break;
        case EV_BEACON_MISSED:
          #ifdef UART_DEBUG
          Serial.println(F("EV_BEACON_MISSED"));
          #endif
            break;
        case EV_BEACON_TRACKED:
          #ifdef UART_DEBUG
          Serial.println(F("EV_BEACON_TRACKED"));
          #endif
          break;
        case EV_JOINING:
          #ifdef UART_DEBUG
          Serial.println(F("EV_JOINING"));
          #endif
          break;
        case EV_JOINED:
          #ifdef UART_DEBUG
          Serial.println(F("EV_JOINED"));
          #endif
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              #ifdef UART_DEBUG
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {                                                     
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
                digitalWrite(INDICATE_LED, HIGH);
                delay(10);
                digitalWrite(INDICATE_LED, LOW);
              #endif
            }
            LMIC_setLinkCheckMode(0);
            LMIC_setAdrMode(0);
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            #ifdef UART_DEBUG
            Serial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
            #ifdef UART_DEBUG
            Serial.println(F("EV_REJOIN_FAILED"));
            #endif
            break;
        case EV_TXCOMPLETE:
            #ifdef UART_DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif
            if (LMIC.txrxFlags & TXRX_ACK)
              #ifdef UART_DEBUG
              Serial.println(F("Received ack"));
              #endif
            if (LMIC.dataLen) {
                uint8_t payload[LMIC.dataLen];
                #ifdef UART_DEBUG
                Serial.print("Received: ");
                #endif
                for (int i = 0; i < LMIC.dataLen; i++) {
                    payload[i] = LMIC.frame[LMIC.dataBeg + i];
                    #ifdef UART_DEBUG
                    Serial.print(payload[i], HEX);
                    Serial.print(" ");
                    #endif
                }
                #ifdef UART_DEBUG
                Serial.print(" | SIZE: ");
                Serial.print(sizeof(payload));
                Serial.println("");
                #endif
                if (payload[0] == DOWNLINK_TGL_LED) {
                    toggle_indicator_led();
                }

                if ( (payload[0] == DOWNLINK_SETSF) && (sizeof(payload) == 2) ) {
                    #ifdef UART_DEBUG
                    Serial.print("setting SF ");
                    Serial.println(payload[1]);
                    #endif
                    tx_sf = payload[1];
                }

            }
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            lmic_txrx = false;
            break;
        case EV_LOST_TSYNC:
            #ifdef UART_DEBUG
            Serial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
            #ifdef UART_DEBUG
            Serial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            #ifdef UART_DEBUG
            Serial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            #ifdef UART_DEBUG
            Serial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
            #ifdef UART_DEBUG
            Serial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            #ifdef UART_DEBUG
            Serial.println(F("EV_TXSTART"));
            #endif
            break;
        case EV_TXCANCELED:
            #ifdef UART_DEBUG
            Serial.println(F("EV_TXCANCELED"));
            #endif
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            #ifdef UART_DEBUG
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            #endif
            break;

        default:
            #ifdef UART_DEBUG
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            #endif
            break;
    }
}