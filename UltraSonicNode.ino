/*******************************************************************************
 * 
 * FILE UltraSonic distance sensor LoRa Node
 * AUTHOR Finn Morris
 * DESC This program uses an ultrasonic sensor to measure distance and LoRaWAN OTAA on AU915 to connect to Curtin University Gateways
 * 
 * DEVICE: TTG0 LoRa OLED SX1276
 * VERSION 1.0.0
 * DATE CREATED 12/07/2022
 * DATE MODIFIED 12/07/2022
 *
 *
 *DEV EUI 00C3D91D28E437F9
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


// defines UltraSonic HC-SR04 pins numbers
const int trigPin = 13;
const int echoPin = 12;

//Variables for UltraSonic
long duration;
int distance;

//packet counter
int counter = 0;

// This EUI must be in little-endian format, so least-significant-byte first.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xF9, 0x37, 0xE4, 0x28, 0x1D, 0xD9, 0xC3, 0x00 };

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply).
static const u1_t PROGMEM APPKEY[16] = { 0x69, 0x2A, 0x66, 0x33, 0x05, 0xFA, 0x97, 0xAD, 0x2E, 0x8C, 0xC2, 0x41, 0x7B, 0x13, 0xE7, 0x4B };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
 
static osjob_t sendjob;
 
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;
 
// For Heltec Wifi LoRa 32, TTGO LoRa, TTGO LoRa32 V1.0 and 
// TTGO LoRa32 V1.3:
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

//For TTGO LoRa32 V2.x use:
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
 
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
 
void do_send(osjob_t* j){

  //Perform a read of the UltraSonic sensor before transmitting
  
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);

    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("LORA SENDER");
        display.setCursor(0,20);
        display.setTextSize(1);
        display.print("LoRa packet not sending.");
        display.setCursor(0,30);
        display.print("Counter:");
        display.setCursor(50,30);
        display.print(counter);      
        display.display();
        counter++;
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (xref2u1_t)&distance, sizeof(distance), 0);
        Serial.println(F("Packet queued"));
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("LORA SENDER");
        display.setCursor(0,20);
        display.setTextSize(1);
        display.print("LoRa packet queued.");
        display.setCursor(0,30);
        display.print("Counter:");
        display.setCursor(50,30);
        display.print(counter); 
        display.setCursor(0,50);
        display.println("Distance: ");
        display.setCursor(60,50);
        display.print(distance);     
        display.display();
        counter++;
    }
}
    // Next TX is scheduled after TX_COMPLETE event.
 
void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    //Set up pins for the Ultra Sonic sensor
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(9600); // Starts the serial communication

  
    //reset OLED display via software
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
  
    //initialize OLED
    Wire.begin(OLED_SDA, OLED_SCL);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("LORA SENDER ");
    display.display();
    
     
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
 
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}
 
void loop() {
  os_runloop_once();
}
