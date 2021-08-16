/**
 * @file script-serverntp-november
 * @author David Caceres (david.caceres1@unmsm.edu.pe)
 * @brief apirest and serverntp for radar synchronous 
 * @version 0.1
 * @date 2020-11-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#define ethernet_h_
#include <Arduino.h>
//#include <HardwareSerial.h>
#include <IWatchdog.h>
//#include <STM32RTC.h>
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <EthernetUdp.h>
#include <Thunderbolt.h>
#include <ArduinoJson.h>
#include <aREST.h>
#include <rBase64.h>
#include <SPI.h>

//#include "SPI.h"
//#include "Wire.h"
//#include "Ucglib.h"

/*
  Hardware SPI Pins:
    Arduino Uno    sclk=13, data=11  
*/

//Ucglib_ILI9341_18x240x320_SWSPI ucg(/*sclk=*/ 7, /*data=*/ 6, /*cd=*/ 5, /*cs=*/ 3, /*reset=*/ 4);
//Ucglib_ILI9341_18x240x320_SWSPI ucg(/*sclk=*/ 13, /*data=*/ 11, /*cd=*/ 9, /*cs=*/ 10, /*reset=*/ 8);
//Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/9, /*cs=*/10, /*reset=*/8);
//Ucglib_ILI9341_18x240x320_SWSPI ucg(/*sclk=*/ 4, /*data=*/ 3, /*cd=*/ 6, /*cs=*/ 7, /*reset=*/ 5);

//!remember this conection has definied in the shield
//all PA5 is SCLK
//all PA6 is MISO
//all PA7 is MOSI
//all PD14 is CS

#define MODE_DEBUG // help to developer debuged code more faster defining small fragment to send by serial monitor
#define IP_STATIC_ROJ
//#define CLOCK_MASTER // set memory map of clock master, notdefine is set memory map of synchro VIPIR
#define MODE_INTERRUPT_DEBUG
#define MODE_FUNCTION_TSIPPACKET_DEBUG
//#define MODULE_SPI_MASTER //if is defined the fpga contain a instantiation of spi master, so the nucleo receive any data in MISO
//#define MODE_DEBUG_DISPLAY // THIS DEFINITION HELP US IN DEBUGED CODE USING A DISPLAY, IN THE FUTURE I WISH IMPLEMENT IT
//#define developed_new_function // not comment when the firsst version already done

#define DEBUG_RC
#ifdef DEBUG_RC
#define DEBUG_RC_BEGIN(x) Serial.begin(x)
#define DEBUG_RC_PRINT(x) Serial.print(x)
#define DEBUG_RC_PRINT2(x, y) Serial.print(x, y)
#define DEBUG_RC_PRINTLN(x) Serial.println(x)
#define DEBUG_RC_PRINTLN2(x, y) Serial.println(x, y)
#else
#define DEBUG_RC_BEGIN(x)
#define DEBUG_RC_PRINT(x)
#define DEBUG_RC_PRINT2(x, y)
#define DEBUG_RC_PRINTLN(x)
#define DEBUG_RC_PRINTLN2(x, y)
#endif

//************************************
//*********NTPpacket struct***********
//************************************
typedef struct
{
    struct firstHeader
    {
        uint32_t PRECISION : 8;
        uint32_t POLL : 8;    //int with signed
        uint32_t STRATUM : 8; //int with signed
        uint32_t MODE : 3;
        uint32_t VERSION : 3;
        uint32_t LI : 2;
    } header;

    struct rootDelay
    {
        uint32_t DECIMAL : 16;
        uint32_t INTEGER : 16;
    } rDelay;

    struct rootDispersion
    {
        uint32_t DECIMAL : 16;
        uint32_t INTEGER : 16;
    } rDisp;

    struct referenceIdentifier
    {
        uint32_t FOURTH_ASCII : 8;
        uint32_t THIRD_ASCII : 8;
        uint32_t SECOND_ASCII : 8;
        uint32_t FIRST_ASCII : 8;
    } rId;

    struct referenceTimestamp
    {
        uint64_t DECIMAL : 32;
        uint64_t INTEGER : 32;
    } refTamp;

    struct originTimestamp
    {
        uint64_t DECIMAL : 32;
        uint64_t INTEGER : 32;
    } orgTamp;

    struct receiveTimestamp
    {
        uint64_t DECIMAL : 32;
        uint64_t INTEGER : 32;
    } rcverTamp;

    struct transmitTimestamp
    {
        uint64_t DECIMAL : 32;
        uint64_t INTEGER : 32;
    } transTamp;

} packetNTP_t;

// ---------------------------
// Globals
// ---------------------------
//uint32_t ppsTimestamp = 0;        // time in microseconds of last pps
uint32_t unixTime = 1603810945; // current second count since 1970

// -----------------------------------
// code david
// -----------------------------------
volatile uint32_t pps = 0; // before called ppsTimestamp, micros() is a type uint32_t
uint32_t lastpps = 0;      //
uint32_t lasttime = 0;     // time last pps
//------------------------------------
//------------------------------------
// Receive timestamp variables
uint32_t softRxTimestamp = 0;     // time in microseconds when client packet arrives ... gotta improve this to symmetric interleaved with NIC interruptions
uint32_t fracRxSecond = 0;        //difference in microseconds between PPS and softRxTimestamp
uint32_t secondsTimestampRx = 0;  // integer part of fixed point receive timestamp
uint32_t fractionTimestampRx = 0; // fraction part of fixed point receive timestamp
// Transmit timestamp variables
uint32_t softTxTimestamp = 0;     // time in microseconds when server packet will be transmited ... gotta improve this to symmetric interleaved with NIC interruptions
uint32_t fracTxSecond = 0;        //difference in microseconds between PPS and softTxTimestamp
uint32_t secondsTimestampTx = 0;  //  integer part of fixed point transmit timestamp
uint32_t fractionTimestampTx = 0; // fraction part of fixed point transmit timestamp

// ---------------------------
// *magic number* that turns an 'uint32_t' number into a 'uint32_t' fractional representation
// of the same number (times 10e-6) in a fixed point format
// Example: you have 4
// 4 times 4294.9763 is 17179.9052
// get the integer part of the number (17179) and represent it as 32-bits
// the result is: 00000000000000000100001100011011 that is equal to the fractional part of a fixed point
//              0.00000000000000000100001100011011
// representation of 4x10e-6
// after multiplication of the number of micorsecond obtained with micros() function with a double 4294.9763
// we only care about the integer part. This is our fractional timestamp.
// ---------------------------
//const double MULTIPLIER = 4294.9763; // deatiled explanation below
const double MULTIPLIER = 4294.967296;
// ---------------------------
// Interruption pins
// ---------------------------
//uint32_t PPS_PIN = PB_5;
uint32_t PPS_PIN = PB5;
uint32_t RESET_PIN = PE9; //PF13

//PB7 AND PB14 WILL BE USED FOR SHOW LEDs
const uint32_t led = PB0;      //green led
const uint32_t ledPIN = PB7;   //blue led
const uint32_t ledPIN2 = PB14; //red led
uint32_t resetFpga = 0;
// ---------------------------
// ---------------------------
// SPI def
// ---------------------------
int SPI_delay = 10;
#define SS (uint8_t) PD14 //chip selector in SPIslave
// ---------------------------
// ---------------------------
uint32_t lasttimepacketsize = 0;

#define UNIX_EPOCH 2208988800UL // 1970 - 1900 in seconds
#define NTP_PORT 123            // Time Server Port

// ---------------------------
// buffers for receiving and sending data
// ---------------------------
static const int NTP_PACKET_SIZE = 48;
byte packetBufferRx[NTP_PACKET_SIZE];
byte packetBufferTx[NTP_PACKET_SIZE];

// ---------------------------
//GPS instances
// ---------------------------
HardwareSerial Serial2(PG_9, PG_14); //PD_6 is Rx and PD_5 is Tx
Thunderbolt tbolt(Serial2);          // Use HardwareSerial #2
GPSTime timeInfo;
GPSStatus statusInfo;
byte timingFlags = 0xF; // received from TSIP packets

//define the nested struct
packetNTP_t NTPpacketRx, NTPpacketTx;

//STM32RTC &rtc = STM32RTC::getInstance();

//****************************************
//********Create register addreses********
//****************************************

#ifdef CLOCK_MASTER

/**
 * @brief MUX register addresses
 * 
 */
typedef struct
{
    const uint8_t CH_MUX_SELECTOR = 0x0E;
    const uint8_t CH_MUX_ENABLE = 0x0F;

} __attribute__((packed)) chMux_t;

chMux_t chMux;

/**
 * @brief PPS Divider register addresses for channel 0
 * 
 */
typedef struct
{
    const uint8_t ADDR_PER_TRUE = 0x50;
    const uint8_t ADDR_DIV_NUM = 0x51;
    const uint8_t ADDR_PHASE3 = 0x52;
    const uint8_t ADDR_PHASE2 = 0x53;
    const uint8_t ADDR_PHASE1 = 0x54;
    const uint8_t ADDR_PHASE0 = 0x55; //lsb
    const uint8_t ADDR_WIDTH = 0x56;
    const uint8_t ADDR_START = 0x57;
    const uint8_t ADDR_STOP = 0x58;
} __attribute__((packed)) ppsDivider0_t;

ppsDivider0_t ppsDivider0;

/**
 * @brief PPS Divider register addresses for channel 1
 * 
 */
typedef struct
{
    const uint8_t ADDR_PER_TRUE = 0x59;
    const uint8_t ADDR_DIV_NUM = 0x5A;
    const uint8_t ADDR_PHASE3 = 0x5B;
    const uint8_t ADDR_PHASE2 = 0x5C;
    const uint8_t ADDR_PHASE1 = 0x5D;
    const uint8_t ADDR_PHASE0 = 0x5E; //lsb
    const uint8_t ADDR_WIDTH = 0x5F;
    const uint8_t ADDR_START = 0x60;
    const uint8_t ADDR_STOP = 0x61;
} __attribute__((packed)) ppsDivider1_t;

ppsDivider1_t ppsDivider1;
#else
/**
 * @brief PPS Divider register addresses for channel 0
 * 
 */
typedef struct
{
    const uint8_t ADDR_PER_TRUE = 0x00;
    const uint8_t ADDR_DIV_NUM = 0x01;
    const uint8_t ADDR_PHASE3 = 0x02;
    const uint8_t ADDR_PHASE2 = 0x03;
    const uint8_t ADDR_PHASE1 = 0x04;
    const uint8_t ADDR_PHASE0 = 0x05; //lsb
    const uint8_t ADDR_WIDTH = 0x06;
    const uint8_t ADDR_START = 0x07;
    const uint8_t ADDR_STOP = 0x08;
} __attribute__((packed)) ppsDivider0_t;

ppsDivider0_t ppsDivider0;

typedef struct
{
    const uint8_t ADDR_WIDTH3 = 0x10;
    const uint8_t ADDR_WIDTH2 = 0x11;
    const uint8_t ADDR_WIDTH1 = 0x12;
    const uint8_t ADDR_WIDTH0 = 0x13; //lsb
    const uint8_t ADDR_ENABLE = 0x14; //enable
    const uint8_t ADDR_PERIODE3 = 0x15;
    const uint8_t ADDR_PERIODE2 = 0x16;
    const uint8_t ADDR_PERIODE1 = 0x17;
    const uint8_t ADDR_PERIODE0 = 0x18; //lsb
    const uint8_t ADDR_DELAY3 = 0x19;
    const uint8_t ADDR_DELAY2 = 0x1A;
    const uint8_t ADDR_DELAY1 = 0x1B;
    const uint8_t ADDR_DELAY0 = 0x1C; //lsb

} __attribute__((packed)) signalTR0_t;

signalTR0_t signalTR0;

#endif

//****************************************
//********Create register data************
//****************************************

typedef struct
{
    uint8_t ch_mux_selector_byte;
    uint8_t ch_mux_enable_byte;

} __attribute__((packed)) chMuxVar_t;

chMuxVar_t chMuxVar;

typedef struct
{
    uint8_t periodicTrue_byte;
    uint8_t divider_byte;
    uint8_t phase_w3_byte;
    uint8_t phase_w2_byte;
    uint8_t phase_w1_byte;
    uint8_t phase_w0_byte;
    uint8_t width_byte;
    uint8_t start_byte;
    uint8_t stop_byte;
} __attribute__((packed)) ppsDividerVar_t;

ppsDividerVar_t ppsDividerVar;

typedef struct
{
    uint8_t width_w3_byte;
    uint8_t width_w2_byte;
    uint8_t width_w1_byte;
    uint8_t width_w0_byte;
    uint8_t periode_w3_byte;
    uint8_t periode_w2_byte;
    uint8_t periode_w1_byte;
    uint8_t periode_w0_byte;
    uint8_t delay_w3_byte;
    uint8_t delay_w2_byte;
    uint8_t delay_w1_byte;
    uint8_t delay_w0_byte;
    uint8_t enable_byte;

} __attribute__((packed)) signalTRVar_t;

signalTRVar_t signalTRVar;

byte response_1 = 0;
byte response_2 = 0;
byte response[] = {0x0, 0x0, 0x0};

void markPps();

//uint8_t mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

#ifdef IP_STATIC_ROJ
//IPAddress ip(10, 10, 10, 28);
//IPAddress dns(10, 10, 10, 1);
//IPAddress gateway(10, 10, 10, 1);
//IPAddress subnet(255, 255, 255, 0);
IPAddress ip(192, 168, 1, 28);
IPAddress dns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
#else
IPAddress ip(192, 168, 1, 13); // NTP Server public IP Address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
#endif

EthernetServer server(80);
//EthernetClient ethClient;
//****************************************
//*********Create aREST instance**********
//****************************************

aREST rest = aREST();

EthernetUDP Udp; // An Ethernet UDP instance
//****************************************
//****Variables to be exposed to the API**
//****************************************

uint8_t channel = 20;
String mode = "";
uint32_t periode = 50; // microseconds
uint32_t Delay = 25;   //
uint32_t pulse = 50;   // microseconds
uint32_t width = 25;
uint8_t enable = 0;

long pxPeriode = 0; //for graphic in the tft
long pxDelay = 0;
long pxPulse = 0;
long pxWidth = 0;

//****************************************
//**** API FUNCTIONS DECLARATIONS ********
//****************************************
int getStatusTrimble(String command);
int reset(String command);
//int changeip(String command);
int setChannel(String command);
int ppsMode(String command);
int trMode(String comand);
int clockMode(String command);
//int start(String command);
//int stop(String command);
String status = "Hola mundo"; // status string variable, you can put any status message here.

//****************************************
//*********prototype function*************
//****************************************
void SendPacketSPI(byte address, byte buf_data, byte *resp);
int SendPacketSPI2(byte address, byte buf_data, byte *resp);

void display_version(GPSVersion);
void displayTime(GPSTime);
void displayMemberElements(packetNTP_t *); //prototype function for showing the packetNTP
void setPacketNtp(packetNTP_t *);          //prototype function for setting the packetNTP

// Declare functions to be conditioned for using modules in fpga clock master
void convertToPpsDivider(uint8_t channel, uint8_t enable);
//****************************************
//****************************************
//****************************************

void processGPS();
void processPPS();
void processNTP();
unsigned int usec2ntp(unsigned int usec);
uint32_t timeout = 3000;
bool ppsConnected = false;

uint8_t z = 127; // start value

typedef struct
{
    const uint8_t size_8 = 5;
    const uint8_t size_10 = 6;
    const uint8_t size_12 = 8;
    const uint8_t size_14 = 10; //lsb
    const uint8_t size_18 = 14;
    const uint8_t size_24 = 20;
} __attribute__((packed)) font_t;

font_t font;

uint8_t button = PC13;
//ucg_t ucg1;
/*
static const unsigned char logo16_glcd_bmp[] PROGMEM =
    {B00000000, B11000000,
     B00000001, B11000000,
     B00000001, B11000000,
     B00000011, B11100000,
     B11110011, B11100000,
     B11111110, B11111000,
     B01111110, B11111111,
     B00110011, B10011111,
     B00011111, B11111100,
     B00001101, B01110000,
     B00011011, B10100000,
     B00111111, B11100000,
     B00111111, B11110000,
     B01111100, B11110000,
     B01110000, B01110000,
     B00000000, B00110000};


void ucg_DrawL90BFWithArg(ucg_t *ucg)
{
    ucg->device_cb(ucg, 25, &(ucg->arg));
}
*/
/*
void ucg_DrawBitmapLine(ucg_t *ucg, ucg_int_t x, ucg_int_t y, ucg_int_t dir, ucg_int_t len, const unsigned char *bitmap)
{*/
/*
  ucg->arg.pixel.rgb.color[0] = ucg->arg.rgb[0].color[0];
  ucg->arg.pixel.rgb.color[1] = ucg->arg.rgb[0].color[1];
  ucg->arg.pixel.rgb.color[2] = ucg->arg.rgb[0].color[2];
  */
/*ucg->arg.pixel.pos.x = x;
    ucg->arg.pixel.pos.y = y;
    ucg->arg.dir = dir;
    ucg->arg.len = len;
    ucg->arg.bitmap = bitmap;
    ucg->arg.pixel_skip = 0;
    //ucg->arg.scale = 0;
    ucg_DrawL90BFWithArg(ucg);
}*/

uint32_t lcg_rnd(void)
{
    z = (uint8_t)((uint16_t)65 * (uint16_t)z + (uint16_t)17);
    return (uint32_t)z;
}

bool buttonState = false;

int option;

void setup()
{
    //configure bottom for change the show tft between pulse and server.
    pinMode(button, INPUT);
    // start Serials
    Serial.begin(1000000); //debug?
    Serial.flush();

    //ucg.begin(UCG_FONT_MODE_TRANSPARENT);
    /*ucg.setRotate270();
    ucg.clearScreen();*/

    //ucg_DrawBitmapLine(&ucg1, 0, 0, 0, 10, logo16_glcd_bmp);

    /*ucg.clearScreen();
    ucg.setColor(76, 255, 255); //cyan
    ucg.drawFrame(0, 0, ucg.getWidth(), ucg.getHeight());

    ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
    ucg.setFont(ucg_font_helvB12_tr); //12 pixel height
    ucg.setFontPosBottom();

    ucg.setColor(76, 255, 255);                                                                                                 //cyan
    ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("Setting up PPS pin interrupt") / 2) * font.size_12), ucg.getHeight() / 8); //28 is for the quantity of letters you want print in the tft.
    ucg.print("Setting up PPS pin interrupt");*/
    Serial.println("Setting up PPS pin interrupt");

    pinMode(ledPIN, OUTPUT);
    pinMode(ledPIN2, OUTPUT);
    pinMode(PPS_PIN, INPUT);
    pinMode(RESET_PIN, OUTPUT);
    attachInterrupt(PPS_PIN, markPps, RISING); //ISR is an function Interrupt Service Routine, digitalPinToInterrupt(PPS_PIN)
    delay(100);

    /*while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }*/

    //const char host[] = "10.10.10.30"; //prueba arduino
    //Udp.beginPacket(host,NTPport);                  //prueba arduino

    // ucg.setColor(76, 255, 255); //cyan
    //ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("Setting up trimble init") / 2) * font.size_12), (ucg.getHeight() * 2) / 8);
    //ucg.print("Setting up trimble init"); //replace this space for trimble configuration version!

    Serial.println("Setting up trimble init");
    Serial2.begin(TSIP_BAUD_RATE);

    // API Restful configuration
    // Init variables and expose them to REST API
    rest.variable("status", &status);
    // Function to be exposed
    rest.function("ppsmode", ppsMode);
    rest.function("trmode", trMode);
    rest.function("clockmode", clockMode);
    rest.function("setchannel", setChannel);
    rest.function("reset", reset);
    rest.function("getStatusTrimble", getStatusTrimble);
    // Give name & ID to the device (ID should be 6 characters long)
    rest.set_id("001");
    rest.set_name("DSA_controller");

    tbolt.flush();
    if (tbolt.getSoftwareVersionInfo() == false) // this call is synchronous (waits for a response - but will timeout)
    {
        Serial.println("Unable to start Thunder;bolt, nothing more to do.");
        while (1)
        {
            ;
        }
    }
    else
    {
        display_version(tbolt.getVersion());
    }

#ifdef MODE_DEBUG
    Serial.println("End configuration of Trimble.");
#endif

    //inicializar SPI
    //ucg.setColor(76, 255, 255); //cyan
    //ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("Setting up bus SPI") / 2) * font.size_12), (ucg.getHeight() * 3) / 8);
    //ucg.print("Setting up bus SPI");

    //Udp.begin(NTP_PORT);this line should be after initialize the ip

    digitalWrite(RESET_PIN, HIGH); //reset because the module fpga need initialize in the FSM called "waitPps"
    delayMicroseconds(10);
    digitalWrite(RESET_PIN, LOW);

#ifdef __IWATCHDOG_H__
    IWatchdog.clearReset();
    if (IWatchdog.isReset(true))
    {
        // LED blinks to indicate reset
        Serial.println("IWDG reset!!");
        for (uint8_t idx = 0; idx < 5; idx++)
        {
            digitalWrite(ledPIN, HIGH);
            delay(100);
            digitalWrite(ledPIN, LOW);
            delay(100);
        }
    }
    // Start watchdog
    IWatchdog.clearReset();

    //IWatchdog.begin(4000000);
    IWatchdog.begin(12000000);
    if (!IWatchdog.isEnabled())
    {
        // LED blinks indefinitely
        while (1)
        {
            digitalWrite(ledPIN, HIGH);
            delay(500);
            digitalWrite(ledPIN, LOW);
            delay(500);
            Serial.println("Watchdog is not enabled!");
        }
    }

    /* ucg.setColor(76, 255, 255); //cyan
    ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("Setting up iwatchdog") / 2) * font.size_12), (ucg.getHeight() * 4) / 8);
    ucg.print("Setting up iwatchdog");*/
#endif

    Serial.println("Setting up bus SPI");
    //pinMode(SS, OUTPUT);
    //digitalWrite(SS, HIGH);
    SPI.setSCLK(PA_5);
    SPI.setMISO(PA_6);
    SPI.setMOSI(PA_7);
    SPI.begin();
    SPI.beginTransaction(SS, SPISettings(2000000, MSBFIRST, SPI_MODE_0)); //con eso el clk es de 10MHz pero de la nucleo, en el fpga se sincroniza con su propio clk

#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

#ifdef IP_STATIC_ROJ
    Ethernet.begin(ip, subnet, gateway);
#else
    Ethernet.begin(ip, subnet, gateway);
#endif
    delay(1000);
    server.begin();
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());
    delay(10);
    Udp.begin(NTP_PORT);
    Serial.println("paso la instruccion udp.begin");

#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif
    // ucg.setFont(ucg_font_ncenB18_hr); //ucg_font_ncenB18_hr is 18 pixel height, ucg_font_helvB18_tr
    // ucg.setFontPosBottom();
    // ucg.setColor(76, 255, 255); //cyan
    // ucg.setPrintPos((ucg.getWidth() / 2) - ((12 / 2) * font.size_18), (ucg.getHeight() * 4) / 6);
    // ucg.print(Ethernet.localIP());
    // ucg.setPrintPos((ucg.getWidth() / 2) - ((13 / 2) * font.size_18), (ucg.getHeight() * 5) / 6);
    // ucg.print(Ethernet.subnetMask());
    // ucg.setPrintPos((ucg.getWidth() / 2) - ((11 / 2) * font.size_18), (ucg.getHeight() * 6) / 6);
    // ucg.print(Ethernet.gatewayIP());
    // delay(4000);
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    delay(1000);
    //uint8_t *mac=Ethernet.MACAddress();
    //Serial.println(*mac);

#ifdef MODE_DEBUG

    Serial.println("Starting loop.");
    /*for (int aa = 0; aa < 4; aa++)
    {
#ifdef __IWATCHDOG_H__
        IWatchdog.reload();
#endif
        ucg.clearScreen();
        ucg.setFont(ucg_font_ncenB14_hr);
        ucg.setFontPosBaseline();
        ucg.setScale2x2();
        ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("Starting loop.") / 2) * font.size_14), ucg.getHeight() / 2);
        ucg.print("Starting loop.");
        delay(500);
    }*/
#endif
    /*ucg.undoScale();
    ucg.clearScreen();*/
}

void loop()
{
    if (Serial.available() > 0)
    {
        //leemos la opcion enviada
        option = Serial.read();
        if (option == 'r')
        {
            Serial.println("OFF");
            while (1)
                ; //force reset iwatchdog
        }
    }
    IWatchdog.reload();

    // listen for incoming clients
    EthernetClient ethClient = server.available();
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    if (server.available() > 0)
        Serial.println(ethClient);
        /*
  while(!ethClient.available()){
    ;
    }*/

#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    rest.handle(ethClient); //listen petition aRest
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    /*  if (digitalRead(button))
    {
        buttonState = !buttonState;
        ucg.clearScreen();
    }
    if (buttonState)
    {
        #ifdef __IWATCHDOG_H__
        IWatchdog.reload();
        #endif
        //20% and 80% proportion, 20% for reset fpga and clock. 80% signal TR and PPS.
        //5 is a gap, 2*5 is when is simetric.
        ucg.setColor(255, 0, 255); //pink for reset signal and clock
        ucg.drawRFrame(5, (ucg.getHeight() * 0 / 100) + 5, ucg.getWidth() - 2 * 5, (ucg.getHeight() * 20 / 100) - 2 * 5, 4);
        ucg.drawLine((ucg.getWidth() / 2), 5, (ucg.getWidth() / 2), (ucg.getHeight() * 20 / 100) - 5);
        ucg.setColor(255, 255, 0); //yellow for PPS signal
        ucg.drawRFrame(5, (ucg.getHeight() * 20 / 100) + 5, ucg.getWidth() - 2 * 5, (ucg.getHeight() * 40 / 100) - 2 * 5, 4);
        ucg.setColor(0, 255, 255); //cyan for TR signal
        ucg.drawRFrame(5, (ucg.getHeight() * 60 / 100) + 5, ucg.getWidth() - 2 * 5, (ucg.getHeight() * 40 / 100) - 2 * 5, 4);
        ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
        ucg.setColor(255, 0, 255);
        ucg.setFontPosTop();
        ucg.setPrintPos(10, 8);
        ucg.setFont(ucg_font_helvB08_tr);
        ucg.print("reset fpga");
        ucg.setPrintPos(70, 8);
        ucg.print("[the last time]");
        ucg.setPrintPos(170, 8);
        ucg.print("clock signal");
        ucg.setPrintPos(230, 8);
        ucg.print("[the last time]");
        ucg.setFontMode(UCG_FONT_MODE_SOLID);
        ucg.setFont(ucg_font_7x13_mr);
        ucg.setColor(0, 255, 255, 255); // use white as main color for the font
        ucg.setColor(1, 0, 0, 0);       // use black as background for SOLID mode
        ucg.setPrintPos(10, 10);

        #ifdef __IWATCHDOG_H__
        IWatchdog.reload();
        #endif

        if (resetFpga == 1)
            ucg.print("ON");
        else
        {
            //ucg.print("OFF"); // extra spaces
        }*/

    /**
         * @brief dynamic signal for TR and PPS
         * 
         */
    /* int yLevel;
        int xCtePps = 40;
        int yCtePps = 124; //144-20
        int xCte = 40;
        int yCte = 220; //240-20
        if (enable == 0)
        {
            yLevel = 40;
        }
        else
        {
            yLevel = 0;
        }
        pxPulse = map((long)pulse, 0, 100, 0, 270);
        ucg.setFontPosTop();
        ucg.setFontMode(UCG_FONT_MODE_SOLID);
        ucg.setFont(ucg_font_7x13_mr);
        ucg.setColor(255, 255, 0); //yellow for PPS signal
        ucg.setColor(1, 0, 0, 0);  // use black as background for SOLID mode
        ucg.setPrintPos(10, 48);    //letters
        ucg.print("PPS signal");
        ucg.setPrintPos(150, 48);
        ucg.print("CH 1");
        ucg.setPrintPos(200, 48);
        ucg.print("[the last time]");
        ucg.drawLine(10, 124, 40, 124);
        ucg.drawLine(xCtePps, yCtePps, xCtePps, yCtePps - yLevel);
        ucg.drawLine(xCtePps, yCtePps - yLevel, xCtePps + pxPulse, yCtePps - yLevel);
        ucg.drawLine(xCtePps + pxPulse, yCtePps, xCtePps + pxPulse, yCtePps - yLevel);
        ucg.drawLine(xCtePps + pxPulse, yCtePps, 305, yCtePps);

#ifdef __IWATCHDOG_H__
        IWatchdog.reload();
#endif

        pxPeriode = map((long)periode, 0, 100, 0, 270); //0-100us-->0-270px (in width there is 320px, 50px dont use for dynamic signal)
        pxDelay = map((long)Delay, 0, 100, 0, 270);
        pxWidth = map((long)width, 0, 100, 0, 270);
        ucg.setFontPosTop();
        ucg.setFontMode(UCG_FONT_MODE_SOLID);
        ucg.setFont(ucg_font_7x13_mr);
        ucg.setColor(0, 255, 255); //cyan for TR signal
        ucg.setColor(1, 0, 0, 0);  // use black as background for SOLID mode
        ucg.setPrintPos(10, 144);   //letters
        ucg.print("TR signal");
        ucg.setPrintPos(150, 144);
        ucg.print("CH 2");
        ucg.setPrintPos(200, 144);
        ucg.print("[the last time]");
        ucg.drawLine(10, 124, 40, 124);
        ucg.drawLine(10, 220, 40, 220);
        for (int ii = 0; ii <= 270; ii = ii + pxPeriode)
        {
            ucg.drawLine(xCte + ii, yCte, xCte + pxDelay + ii, yCte);
            ucg.drawLine(xCte + pxDelay + ii, yCte, xCte + pxDelay + ii, yCte - yLevel);
            ucg.drawLine(xCte + pxDelay + ii, yCte - yLevel, xCte + pxDelay + pxWidth + ii, yCte - yLevel);
            ucg.drawLine(xCte + pxDelay + pxWidth + ii, yCte, xCte + pxDelay + pxWidth + ii, yCte - yLevel);
            Serial.println(ii);
        }
        
      }*/

    /*ucg.setFont(ucg_font_ncenR24_tr);
    ucg.setColor(76, 233, 29); //green
    //ucg.setColor(0, 255, 0);
    ucg.setColor(1, 255, 0, 0);

    ucg.setPrintPos(60, 160); //240,320
    ucg.print("Hello World!");*/

    /*  // get a random value between 0 and 255
    uint8_t rnd = lcg_rnd();
    ucg_int_t y = 0;
    ucg_int_t h = 14;
    if (buttonState == false)
    {
        y += h;
        ucg.setFontMode(UCG_FONT_MODE_TRANSPARENT);
        ucg.setPrintPos(4, 120);
        ucg.setFontPosCenter();
        ucg.setFont(ucg_font_helvB12_tr);
        ucg.print("Time server NTP: ");
        ucg.setFontMode(UCG_FONT_MODE_SOLID);
        //ucg.setFont(ucg_font_7x13_mr);
        ucg.setFont(ucg_font_helvB14_tr);
        ucg.setColor(0, 255, 255, 255); // use white as main color for the font
        ucg.setColor(1, 0, 0, 0);       // use black as background for SOLID mode
        ucg.setPrintPos(150, 120);
        //ucg.print(rnd);
        ucg.print(" 01:14:27 p.m"); // extra spaces
        delay(500);
        ucg.setPrintPos(80, y);
        //ucg.print("dav");
        //ucg.print("  ");
        delay(500);
    }*/
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif
    /*delay(1000);
    ucg.clearScreen();
    ucg.setRotate90();
       ucg.setFont(ucg_font_ncenR24_tr);
    ucg.setColor(76, 233, 29);//green
    //ucg.setColor(0, 255, 0);
    ucg.setColor(1, 255, 0, 0);
    
    ucg.setPrintPos(60, 160);//240,320
    ucg.print("david!");*/

    processGPS();

#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    if ((micros() - lastpps) > 1110000) //this value is cause not exactly 1 second
    {
        ppsConnected = false;
#ifdef MODE_DEBUG
        Serial.println("pps disconnected");
#endif
    }

    if (ppsConnected)
    {
        digitalWrite(ledPIN2, HIGH); // Turn the LED red on
        processNTP();
    }
    else
    {
        digitalWrite(ledPIN2, LOW); // Turn the LED red off when pps is disconnected
    }
}

/*Timestamp PPS*/
void markPps()
{

    // this function marks the moment in microseconds when a rising edge was detected
    pps = micros(); // micros() works upto one TCNT0 overflow IQRs

#ifdef MODE_INTERRUPT_DEBUG
//digitalWrite(ledPIN, HIGH);
//delay(100);
//digitalWrite(ledPIN, LOW);
//Serial.println("interrupcion");
#endif
    //unixTime++;
}

void processGPS()
{
    while (Serial2.available())
    {
        tbolt.readSerial();
        processPPS();
        //gps.encode(GPSSerial.read());
    }
}

void processPPS()
{
    if (pps > 0)
    {
        lastpps = pps; //register the last pps
        pps = 0;       //flag to enter to "if" every ppssecond
        if (tbolt.getSoftwareVersionInfo() == true)
        {
            lasttime = tbolt.getSecondsSince1900Epoch() - UNIX_EPOCH;
            // PPS is after the timestamp -> add one second
            lasttime++;
            //timeval valnow;
            ppsConnected = true;
        }
        else //possible desconnection wire serial DB9
        {
            lastpps = 0;
            lasttime = 0;

#ifdef __IWATCHDOG_H__
            IWatchdog.reload();
#endif

            //led red toggle indicated disconnection
            digitalWrite(ledPIN2, HIGH);
            delay(300);
            digitalWrite(ledPIN2, LOW);

#ifdef __IWATCHDOG_H__
            IWatchdog.reload();
#endif

#ifdef MODE_DEBUG
            Serial.println("wire RS232 disconnected");
#endif
        }
    } // not else 'cause should wait minimun 1 seg for to say that ppssignal is disconnected
}

/*
   convert microseconds to fraction of second * 2^32 (i.e., the lsw of
   a 64-bit ntp timestamp).  This routine uses the factorization
   2^32/10^6 = 4096 + 256 - 1825/32 which results in a max conversion
   error of 3 * 10^-7 and an average error of half that.
 */
unsigned int usec2ntp(unsigned int usec)
{
    unsigned int t = (usec * 1825) >> 5;
    return ((usec << 12) + (usec << 8) - t);
}

void processNTP()
{
    int packetSize = Udp.parsePacket();

#ifdef MODE_DEBUG // contador de 3segundos, se comentara despues
    uint32_t renewMillis = millis();
    if (renewMillis - lasttimepacketsize >= timeout)
    {
        lasttimepacketsize = renewMillis;
        Serial.println(packetSize);
    }
#endif

    if (packetSize)
    {
        Serial.println(packetSize);
        Serial.println("Packet received!");

        while (!Serial2.available())
        {
            ;
        }
        tbolt.readSerial(); //its important before of use any thunderbolt function

        //secondsTimestampRx = tbolt.getSecondsSince1900Epoch() - UNIX_EPOCH;
        secondsTimestampRx = lasttime;            //secondtimestamp is valued and replace in the instant pps is recogized for the variable "lasttime"
        softRxTimestamp = micros();               // THIS CAN BE IMPROVED WITH A RECEIVE INTERRUP ROUTINE FROM THE NIC
        fracRxSecond = softRxTimestamp - lastpps; // negative right after overflow after 70min (improve this)
        // We multiply the uint16_t representation of the fractional second value ('fracRxSecond')
        // times the MULTIPLIER value. The MULTIPLIER value is a number that helps us to get a
        // DECIMAL representation of the BINARY value that we will use in the fractional part of
        // the fixed point timestamps.

        //code snippet for preventing when the microseg overload more of one second
        while (fracRxSecond >= 1000000)
        {
            fracRxSecond = fracRxSecond - 1000000;
            secondsTimestampRx++;
#ifdef MODE_DEBUG
            Serial.println("warning the fracRxsecond is more than one second");
#endif
        }

        fractionTimestampRx = (uint32_t)((fracRxSecond * MULTIPLIER));
        Serial.println(fracRxSecond);

        Serial.println(fractionTimestampRx);

        //lee el pauqete udp para fijarse el puerto y la ip del cielte
        //por la parte final del loop se enpaqueta con udp.write y gracias a udp.ip y udp.remoteport se sabe aquien cliente responder
        //las siguinetes 3 lineasa pueden ir al inicio del if pero quitaria el tiempo exacto cuando se ha recibido el paquete RxNTP
        Udp.read(packetBufferRx, NTP_PACKET_SIZE);
        IPAddress Remote = Udp.remoteIP();
        int PortNum = Udp.remotePort();

        //setPacketNtp(&NTPpacketTx);
        //displayMemberElements(&NTPpacketRx);

        packetBufferTx[0] = 0b00100100; // LI, Version, Mode : no leap - ver4 - Server** (revisar si corresponde server)
        packetBufferTx[1] = 1;          // stratum
        packetBufferTx[2] = 16;         // polling minimum
        packetBufferTx[3] = 0xEF;       // precision 11101111

        // root delay ... it doesnt matter bc it will always be constant
        packetBufferTx[4] = 0;
        packetBufferTx[5] = 0;
        packetBufferTx[6] = 0;
        packetBufferTx[7] = 0;

        // root dispTxersion ... let's go with zero bc we are using the reference signal from GPS
        packetBufferTx[8] = 0;
        packetBufferTx[9] = 0;
        packetBufferTx[10] = 0;
        packetBufferTx[11] = 0;

        // reference ID
        packetBufferTx[12] = 71; // "G"
        packetBufferTx[13] = 80; // "P"
        packetBufferTx[14] = 83; // "S"
        packetBufferTx[15] = 0;  // "0"

        // reference timestamp: last tiem when we synchronized with GPS
        // ... let's go with last PPS (it is the same as the second part of the receive timestamp)
        packetBufferTx[16] = (uint8_t)((secondsTimestampRx >> 24) & 0XFF);
        packetBufferTx[17] = (uint8_t)((secondsTimestampRx >> 16) & 0xFF);
        packetBufferTx[18] = (uint8_t)((secondsTimestampRx >> 8) & 0xFF);
        packetBufferTx[19] = (uint8_t)((secondsTimestampRx)&0xFF);
        packetBufferTx[20] = 0;
        packetBufferTx[21] = 0;
        packetBufferTx[22] = 0;
        packetBufferTx[23] = 0;

        // origin timestamp: copy originate timestamp from incoming UDP transmit timestamp
        packetBufferTx[24] = packetBufferRx[40];
        packetBufferTx[25] = packetBufferRx[41];
        packetBufferTx[26] = packetBufferRx[42];
        packetBufferTx[27] = packetBufferRx[43];
        packetBufferTx[28] = packetBufferRx[44];
        packetBufferTx[29] = packetBufferRx[45];
        packetBufferTx[30] = packetBufferRx[46];
        packetBufferTx[31] = packetBufferRx[47];

        //receive timestamp
        packetBufferTx[32] = (secondsTimestampRx >> 24) & 0XFF;
        packetBufferTx[33] = (secondsTimestampRx >> 16) & 0xFF;
        packetBufferTx[34] = (secondsTimestampRx >> 8) & 0xFF;
        packetBufferTx[35] = (secondsTimestampRx)&0xFF;
        packetBufferTx[36] = (fractionTimestampRx >> 24) & 0XFF;
        packetBufferTx[37] = (fractionTimestampRx >> 16) & 0XFF;
        packetBufferTx[38] = (fractionTimestampRx >> 8) & 0XFF;
        packetBufferTx[39] = (fractionTimestampRx)&0XFF;

        secondsTimestampTx = lasttime;            // convert to seconds since 1900 (consumes one instruction)
        softTxTimestamp = micros();               // THIS CAN BE IMPROVED WITH A TRANSMIT INTERRUP ROUTINE FROM THE NIC
                                                  // if working with interrupts for symmetric interleaved mode, the actual
                                                  // timestamp is transmited in the following NTP packet
        fracTxSecond = softTxTimestamp - lastpps; // check for negatives after overflow after 70min

        while (fracTxSecond >= 1000000)
        {
            fracTxSecond = fracTxSecond - 1000000;
            secondsTimestampTx++;
#ifdef MODE_DEBUG
            Serial.println("warning the fracRxsecond is more than one second");
            //      ucg.setColor(255, 0, 0); //red alert
            //    ucg.setPrintPos((ucg.getWidth() / 2) - ((sizeof("warning the fracRxsecond is more than one second") / 2) * font.size_12), (ucg.getHeight() * 3) / 8);
            //   ucg.print("warning the fracRxsecond is more than one second");
#endif
        }

#ifdef __IWATCHDOG_H__
        IWatchdog.reload();
#endif

        fractionTimestampTx = (uint32_t)((fracTxSecond * MULTIPLIER));

        Serial.println(fracTxSecond);
        Serial.println(fractionTimestampTx);
        //fractionTimestampTx = usec2ntp(fracTxSecond);
        Serial.println(fractionTimestampTx);
        //transmitt timestamp
        packetBufferTx[40] = (secondsTimestampTx >> 24) & 0XFF;
        packetBufferTx[41] = (secondsTimestampTx >> 16) & 0xFF;
        packetBufferTx[42] = (secondsTimestampTx >> 8) & 0xFF;
        packetBufferTx[43] = (secondsTimestampTx)&0xFF;
        packetBufferTx[44] = (fractionTimestampTx >> 24) & 0XFF;
        packetBufferTx[45] = (fractionTimestampTx >> 16) & 0XFF;
        packetBufferTx[46] = (fractionTimestampTx >> 8) & 0XFF;
        packetBufferTx[47] = (fractionTimestampTx)&0XFF;

        // Reply to the IP address and port that sent the NTP request
        Udp.beginPacket(Remote, PortNum);
        Udp.write(packetBufferTx, NTP_PACKET_SIZE);
        Udp.endPacket();

#ifdef __IWATCHDOG_H__
        IWatchdog.reload();
#endif

#ifdef MODE_DEBUG
        for (int j = 44; j < NTP_PACKET_SIZE; j++)
        {
            //Serial.print(j);
            //Serial.print("byte: ");
            Serial.print(packetBufferTx[j], BIN);
        }
        Serial.println("");
#endif
    }
}

void setPacketNtp(packetNTP_t *psetPacket)
{
    psetPacket->header.LI = 3;      // sin ajuste de segundo intercalar
    psetPacket->header.VERSION = 4; // version 4
    psetPacket->header.MODE = 4;    // server mode
    psetPacket->header.STRATUM = 1; // stratum 1
    psetPacket->header.POLL = 16;   // intervalo max entre mensajes NTP
    psetPacket->header.PRECISION = 0xEF;

    // root delay ... it doesnt matter bc it will always be constant
    psetPacket->rDelay.INTEGER = 0;
    psetPacket->rDelay.DECIMAL = 0;

    // root dispTxersion ... let's go with zero bc we are using the reference signal from GPS
    psetPacket->rDisp.INTEGER = 0;
    psetPacket->rDisp.DECIMAL = 0;

    // reference ID
    psetPacket->rId.FIRST_ASCII = 71;  // "G"
    psetPacket->rId.SECOND_ASCII = 80; // "P"
    psetPacket->rId.THIRD_ASCII = 83;  // "S"
    psetPacket->rId.FOURTH_ASCII = 0;  // "0"

    // reference timestamp: last tiem when we synchronized with GPS
    // ... let's go with last PPS (it is the same as the second part of the receive timestamp)
    psetPacket->refTamp.DECIMAL = 0;
    psetPacket->refTamp.INTEGER = secondsTimestampRx;

    // origin timestamp: copy originate timestamp from incoming UDP transmit timestamp

    psetPacket->orgTamp.DECIMAL = (packetBufferRx[43] << 24) | (packetBufferRx[42] << 16) | (packetBufferRx[41] << 8) | packetBufferRx[40];
    psetPacket->orgTamp.INTEGER = (packetBufferRx[47] << 24) | (packetBufferRx[46] << 16) | (packetBufferRx[45] << 8) | packetBufferRx[44];

    //receive timestamp
    psetPacket->rcverTamp.DECIMAL = fractionTimestampRx;
    psetPacket->rcverTamp.INTEGER = secondsTimestampRx;

#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif
}

void displayMemberElements(packetNTP_t *pNTPpacketRx)
{
    char buff[384];
    sprintf(buff, "LI: %lu, VN: %lu, MODE: %lu, STRATUM: %lu, POLL: %lu, PRECISION: %lu", pNTPpacketRx->header.LI, pNTPpacketRx->header.MODE, pNTPpacketRx->header.POLL, pNTPpacketRx->header.PRECISION, pNTPpacketRx->header.STRATUM, pNTPpacketRx->header.VERSION);
    Serial.print(buff);
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif
}

//**********************************************
//***Info Display of the library thunderbolt****
//**********************************************

// Display version info from GPS receiver
void display_version(GPSVersion ver)
{
    char buf[50];
    sprintf(buf, "Thunderbolt ver: app:%d.%02d core:%d.%02d\n", ver.app.major_ver, ver.app.minor_ver, ver.core.major_ver, ver.core.minor_ver);
    Serial.print(buf);
}

// Display status info from GPS receiver
void display_status(GPSStatus s)
{
    Serial.print("Status: ");
    Serial.print(s.rcvr_status);
    Serial.print(" Mode = ");
    Serial.print(s.rcvr_mode);
    Serial.print(" Lat = ");
    Serial.print(s.latitude * RAD_TO_DEG);
    Serial.print(" Lng = ");
    Serial.print(s.longitude * RAD_TO_DEG);
    Serial.print(" Alt = ");
    Serial.print(s.altitude * METERS_TO_FEET);
    Serial.print((s.critical_alarms) ? " (Critical Alarm!)" : "");
    Serial.println((s.minor_alarms) ? " (Minor Alarm!)" : "");
}

// Display UTC time info fro GPS receiver
void displayTime(GPSTime t)
{
    char buf[20];
    sprintf(buf, "time: %02d:%02d:%02d\n", t.Hour, t.Minute, t.Second);
    Serial.print(buf);
}

//*********************************************
//*********************************************
//*********************************************

/**
 * @brief decode Json
 * 
 * @param command 
 * @return DynamicJsonDocument 
 */
DynamicJsonDocument decode_command(String command)
{
    const char *raw_content = command.c_str();
    char decoded_content[command.length()];
    size_t len = rbase64_decode(decoded_content, (char *)raw_content, strlen(raw_content));
    Serial.print("  - Decoded Output Buffer Length: ");
    Serial.println(len);
    Serial.print("  - Decoded: ");
    Serial.println(decoded_content);
    Serial.println();

    const size_t capacity = 2048;
    DynamicJsonDocument doc(capacity);
    deserializeJson(doc, decoded_content);
    return doc;
}

//************************************
//**** API FUNCTIONS DEFINITIONS *****
//************************************
int getStatusTrimble(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"getStatusTrimble\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);
    enable = (uint8_t)doc["enable"];
    Serial.print("enable: ");
    Serial.println(enable);

    if (enable == 1)
    {
        GPSStatus status = tbolt.getStatus();
        display_status(status);
    }

    return 1;
}
int reset(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"reset\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);
    resetFpga = (uint32_t)doc["reset"];

    Serial.print("estado del led para el resetFpga: ");
    Serial.println(resetFpga);

    digitalWrite(ledPIN, resetFpga);

    return 1;
}

int ppsMode(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"ppsmode\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);
    channel = (uint8_t)doc["channel"];
    enable = (uint8_t)doc["enable"];
    Serial.print("enable: ");
    Serial.println(enable);

    convertToPpsDivider(channel, enable);

    return 1;
}

int trMode(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"trmode\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);
    //channel = (uint8_t)doc["channel"];
    periode = (uint32_t)doc["periode"];
    Delay = (uint32_t)doc["Delay"];
    width = (uint32_t)doc["width"];
    enable = (uint8_t)doc["enable"];

    //Serial.print("channel: ");
    //Serial.println(channel);
    Serial.print("periode: ");
    Serial.println(periode);
    Serial.print("Delay: ");
    Serial.println(Delay);
    Serial.print("width: ");
    Serial.println(width);
    Serial.print("enable: ");
    Serial.println(enable);

    signalTRVar.periode_w3_byte = (periode >> 24);
    signalTRVar.periode_w2_byte = (periode >> 16);
    signalTRVar.periode_w1_byte = (periode >> 8);
    signalTRVar.periode_w0_byte = (byte)periode; //lsb

    signalTRVar.width_w3_byte = (width >> 24);
    signalTRVar.width_w2_byte = (width >> 16);
    signalTRVar.width_w1_byte = (width >> 8);
    signalTRVar.width_w0_byte = (byte)width; //lsb

    signalTRVar.delay_w3_byte = (Delay >> 24);
    signalTRVar.delay_w2_byte = (Delay >> 16);
    signalTRVar.delay_w1_byte = (Delay >> 8);
    signalTRVar.delay_w0_byte = (byte)Delay; //lsb

    signalTRVar.enable_byte = enable;

    SendPacketSPI2(signalTR0.ADDR_ENABLE, signalTRVar.enable_byte, response);
    SendPacketSPI2(signalTR0.ADDR_PERIODE3, signalTRVar.periode_w3_byte, response);
    SendPacketSPI2(signalTR0.ADDR_PERIODE2, signalTRVar.periode_w2_byte, response);
    SendPacketSPI2(signalTR0.ADDR_PERIODE1, signalTRVar.periode_w1_byte, response);
    SendPacketSPI2(signalTR0.ADDR_PERIODE0, signalTRVar.periode_w0_byte, response); //lsb
    SendPacketSPI2(signalTR0.ADDR_WIDTH3, signalTRVar.width_w3_byte, response);
    SendPacketSPI2(signalTR0.ADDR_WIDTH2, signalTRVar.width_w2_byte, response);
    SendPacketSPI2(signalTR0.ADDR_WIDTH1, signalTRVar.width_w1_byte, response);
    SendPacketSPI2(signalTR0.ADDR_WIDTH0, signalTRVar.width_w0_byte, response); //lsb
    SendPacketSPI2(signalTR0.ADDR_DELAY3, signalTRVar.delay_w3_byte, response);
    SendPacketSPI2(signalTR0.ADDR_DELAY2, signalTRVar.delay_w2_byte, response);
    SendPacketSPI2(signalTR0.ADDR_DELAY1, signalTRVar.delay_w1_byte, response);
    SendPacketSPI2(signalTR0.ADDR_DELAY0, signalTRVar.delay_w0_byte, response); //lsb

    return 1;
}

int clockMode(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"clockmode\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);
    channel = (uint8_t)doc["channel"];
    enable = (uint8_t)doc["enable"];
    Serial.print("enable: ");
    Serial.println(enable);
    return 1;
}

int setChannel(String command)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    Serial.println("\"setchannel\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    DynamicJsonDocument doc = decode_command(command);

    channel = (uint8_t)doc["channel"];
    mode = String((const char *)doc["mode"]);
    periode = (uint32_t)doc["periode"];
    Delay = (uint32_t)doc["Delay"];
    pulse = (uint8_t)doc["pulse"];
    enable = (uint8_t)doc["enable"];

    Serial.print("channel: ");
    Serial.println(channel);
    Serial.print("mode: ");
    Serial.println(mode);
    Serial.print("periode: ");
    Serial.println(periode);
    Serial.print("Delay: ");
    Serial.println(Delay);
    Serial.print("pulse: ");
    Serial.println(pulse);

    digitalWrite(ledPIN2, HIGH);
    delay(2000);
    digitalWrite(ledPIN2, LOW);

    return 1;
}

//************************************
//**** END API FUNCTIONS *************
//************************************

//**********************************************
//**** FUNCTIONS used into api function*********
//**********************************************

void convertToPpsDivider(uint8_t _channel, uint8_t _enable)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    channel = _channel;
    enable = _enable;

    ppsDividerVar.periodicTrue_byte = 0x01;
    ppsDividerVar.divider_byte = 0x02;
    ppsDividerVar.phase_w3_byte = 0x00;
    ppsDividerVar.phase_w2_byte = 0x00;
    ppsDividerVar.phase_w1_byte = 0x00;
    ppsDividerVar.phase_w0_byte = 0x00; //lsb
    ppsDividerVar.width_byte = 0x64;    //10useg
    if (enable == 1)
    {
        ppsDividerVar.start_byte = 0x01;
        ppsDividerVar.stop_byte = 0x00;
    }
    else if (enable == 0)
    {
        ppsDividerVar.start_byte = 0x00;
        ppsDividerVar.stop_byte = 0x01;
    }
    else
    {
        Serial.println("error, enable puede ser estado 0 o 1");
    }

#ifdef DEBUG_MODE
    DEBUG_RC_PRINT("Periodic True: ");
    DEBUG_RC_PRINTLN(ppsDividerVar.periodicTrue_byte);
    DEBUG_RC_PRINT("Divider: ");
    DEBUG_RC_PRINTLN(ppsDividerVar.divider_byte);
    DEBUG_RC_PRINTLN("Phase: ");
    DEBUG_RC_PRINTLN2(ppsDividerVar.phase_w3_byte, HEX);
    DEBUG_RC_PRINTLN2(ppsDividerVar.phase_w2_byte, HEX);
    DEBUG_RC_PRINTLN2(ppsDividerVar.phase_w1_byte, HEX);
    DEBUG_RC_PRINTLN2(ppsDividerVar.phase_w0_byte, HEX);
    DEBUG_RC_PRINT("Width: ");
    DEBUG_RC_PRINTLN(ppsDividerVar.width_byte);
    DEBUG_RC_PRINT("Start: ");
    DEBUG_RC_PRINTLN(ppsDividerVar.start_byte);
    DEBUG_RC_PRINT("Stop: ");
    DEBUG_RC_PRINTLN(ppsDividerVar.stop_byte);

    //Start sending via SPI
    DEBUG_RC_PRINTLN("Start Sending via SPI ... ");
#endif

#ifdef CLOCK_MASTER
    if (channel == 0x00)
    {
        SendPacketSPI(chMux.CH_MUX_ENABLE, chMuxVar.ch_mux_enable_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_PER_TRUE, ppsDividerVar.periodicTrue_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_DIV_NUM, ppsDividerVar.divider_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_PHASE3, ppsDividerVar.phase_w3_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_PHASE2, ppsDividerVar.phase_w2_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_PHASE1, ppsDividerVar.phase_w1_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_PHASE0, ppsDividerVar.phase_w0_byte, response); //lsb
        SendPacketSPI(ppsDivider0.ADDR_WIDTH, ppsDividerVar.width_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_START, ppsDividerVar.start_byte, response);
        SendPacketSPI(ppsDivider0.ADDR_STOP, ppsDividerVar.stop_byte, response);
    }
    else if (channel == 0x01)
    {
        SendPacketSPI(ppsDivider1.ADDR_PER_TRUE, ppsDividerVar.periodicTrue_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_DIV_NUM, ppsDividerVar.divider_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_PHASE3, ppsDividerVar.phase_w3_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_PHASE2, ppsDividerVar.phase_w2_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_PHASE1, ppsDividerVar.phase_w1_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_PHASE0, ppsDividerVar.phase_w0_byte, response); //lsb
        SendPacketSPI(ppsDivider1.ADDR_WIDTH, ppsDividerVar.width_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_START, ppsDividerVar.start_byte, response);
        SendPacketSPI(ppsDivider1.ADDR_STOP, ppsDividerVar.stop_byte, response);
    }
#else
    SendPacketSPI(ppsDivider0.ADDR_PER_TRUE, ppsDividerVar.periodicTrue_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_DIV_NUM, ppsDividerVar.divider_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_PHASE3, ppsDividerVar.phase_w3_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_PHASE2, ppsDividerVar.phase_w2_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_PHASE1, ppsDividerVar.phase_w1_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_PHASE0, ppsDividerVar.phase_w0_byte, response); //lsb
    SendPacketSPI(ppsDivider0.ADDR_WIDTH, ppsDividerVar.width_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_START, ppsDividerVar.start_byte, response);
    SendPacketSPI(ppsDivider0.ADDR_STOP, ppsDividerVar.stop_byte, response);
#endif
    DEBUG_RC_PRINTLN("");
    DEBUG_RC_PRINTLN("Transmission ended");
    DEBUG_RC_PRINTLN("***************************************************");
}

void SendPacketSPI(byte address, byte buf_data, byte *resp)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    //*********************************************************************************************
    digitalWrite(SS, LOW);
    SPI.transfer(SS, 0xF0); // start byte for the packet
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
//byte resp[] = {0x0,0x0};
#ifdef MODE_DEBUG
    DEBUG_RC_PRINTLN("********************************");
    DEBUG_RC_PRINTLN("Begin SPI after transfer.(0xF0)");
    DEBUG_RC_PRINT("address: ");
    DEBUG_RC_PRINT2(address, HEX);
    DEBUG_RC_PRINT(", data: ");
    DEBUG_RC_PRINTLN2(buf_data, HEX);
#endif

    digitalWrite(SS, LOW);
    resp[0] = SPI.transfer(SS, address);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
#ifdef MODULE_SPI_MASTER
/*DEBUG_RC_PRINT("response resp[0] address: ");
  //DEBUG_RC_PRINT("Response: ");
  DEBUG_RC_PRINT2(resp[0], HEX);*/
#endif

    digitalWrite(SS, LOW);
    resp[1] = SPI.transfer(SS, buf_data);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);

#ifdef MODULE_SPI_MASTER
/*DEBUG_RC_PRINT(",");
  DEBUG_RC_PRINT("response resp[1] data: ");
  DEBUG_RC_PRINTLN2(resp[1], HEX);
  DEBUG_RC_PRINTLN("Verifying...");*/
#endif

#ifdef MODULE_SPI_MASTER
    bitClear(address, 7); //............................................................
    digitalWrite(SS, LOW);
    SPI.transfer(SS, address);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
#endif

#ifdef MODULE_SPI_MASTER
    digitalWrite(SS, LOW);
    resp[2] = SPI.transfer(SS, 0);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
    /* DEBUG_RC_PRINT("response resp[2]: ");
  DEBUG_RC_PRINTLN2(resp[2], HEX);
  if (resp[2] == buf_data)
  {
    DEBUG_RC_PRINTLN("PASS!!!");
  }
  else
  {
    DEBUG_RC_PRINTLN("FAILED!!!");
  }*/
#endif
    //return resp;
}

int SendPacketSPI2(byte address, byte buf_data, byte *resp)
{
#ifdef __IWATCHDOG_H__
    IWatchdog.reload();
#endif

    //*********************************************************************************************
    digitalWrite(SS, LOW);
    SPI.transfer(0xF0); // start byte for the packet
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
//byte resp[] = {0x0,0x0};
#ifdef MODE_DEBUG
    DEBUG_RC_PRINTLN("********************************");
    DEBUG_RC_PRINTLN("Begin SPI after transfer.(0xF0)");
    DEBUG_RC_PRINT("address: ");
    DEBUG_RC_PRINT2(address, HEX);
    DEBUG_RC_PRINT(", data: ");
    DEBUG_RC_PRINTLN2(buf_data, HEX);
#endif

    digitalWrite(SS, LOW);
    resp[0] = SPI.transfer(address);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
#ifdef MODULE_SPI_MASTER
/*DEBUG_RC_PRINT("response resp[0] address: ");
  //DEBUG_RC_PRINT("Response: ");
  DEBUG_RC_PRINT2(resp[0], HEX);*/
#endif

    digitalWrite(SS, LOW);
    resp[1] = SPI.transfer(buf_data);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);

#ifdef MODULE_SPI_MASTER
/*DEBUG_RC_PRINT(",");
  DEBUG_RC_PRINT("response resp[1] data: ");
  DEBUG_RC_PRINTLN2(resp[1], HEX);
  DEBUG_RC_PRINTLN("Verifying...");*/
#endif

#ifdef MODULE_SPI_MASTER
    bitClear(address, 7); //............................................................
    digitalWrite(SS, LOW);
    SPI.transfer(address);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
#endif

#ifdef MODULE_SPI_MASTER
    digitalWrite(SS, LOW);
    resp[2] = SPI.transfer(0);
    digitalWrite(SS, HIGH);
    delayMicroseconds(SPI_delay);
    /* DEBUG_RC_PRINT("response resp[2]: ");
  DEBUG_RC_PRINTLN2(resp[2], HEX);
  if (resp[2] == buf_data)
  {
    DEBUG_RC_PRINTLN("PASS!!!");
  }
  else
  {
    DEBUG_RC_PRINTLN("FAILED!!!");
  }*/
#endif
    //return resp;
    return 1;
}

#ifdef developed_new_function
int NTPgettimeofday(struct timeval *tv, struct timezone *tz)
{
    if (bfixed == false)
        return -1;
    uint64_t delta = micros64() - lastpps;

    tv->tv_sec = lasttime;
    while (delta >= 1000000)
    {
        delta = delta - 1000000;
        tv->tv_sec++;
    }
    tv->tv_usec = delta;
    return 1;
}
//***************************************************************************
//REMENBER
//***************************************************************************
/*
 Structure returned by gettimeofday(2) system call, and used in other calls.
*/

struct timeval
{
    time_t tv_sec;       //seconds
    suseconds_t tv_usec; //and microseconds
};
//***************************************************************************

struct timeval tvalPaket, tvalNow;

void processNTP()
{

    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
        NTPgettimeofday(&tvalPaket, NULL);
        Udp.read(packetBuffer, NTP_PACKET_SIZE);
        IPAddress Remote = Udp.remoteIP();
        int PortNum = Udp.remotePort();

        unsigned char reply[48];
        uint32 tx_ts_upper, tx_ts_lower;
        memcpy(reply, ntp_packet_template, 48);
        /* XXX set Leap Indicator */
        /* Copy client transmit timestamp into origin timestamp */
        memcpy(reply + 24, packetBuffer + 40, 8);

        // reference timestamp
        // fixme: remember cooect last recieve
        NTPgettimeofday(&tvalNow, NULL);

        tempval = seventyYears + tvalNow.tv_sec - 1; // one second before, usually get gps time once a second -> FIXME

        reply[16] = (tempval >> 24) & 0XFF;
        reply[17] = (tempval >> 16) & 0xFF;
        reply[18] = (tempval >> 8) & 0xFF;
        reply[19] = (tempval)&0xFF;

        // synced at full seconds with pps signal

        reply[20] = 0;
        reply[21] = 0;
        reply[22] = 0;
        reply[23] = 0;

        //receive timestamp
        tempval = seventyYears + tvalPaket.tv_sec;
        reply[32] = (tempval >> 24) & 0XFF;
        reply[33] = (tempval >> 16) & 0xFF;
        reply[34] = (tempval >> 8) & 0xFF;
        reply[35] = (tempval)&0xFF;

        tempval = usec2ntp(tvalPaket.tv_usec);

        reply[36] = (tempval >> 24) & 0XFF;
        reply[37] = (tempval >> 16) & 0XFF;
        reply[38] = (tempval >> 8) & 0XFF;
        reply[39] = (tempval)&0XFF;

        //transmitt timestamp
        NTPgettimeofday(&tvalNow, NULL);

        tempval = seventyYears + tvalNow.tv_sec;
        reply[40] = (tempval >> 24) & 0XFF;
        reply[41] = (tempval >> 16) & 0xFF;
        reply[42] = (tempval >> 8) & 0xFF;
        reply[43] = (tempval)&0xFF;

        tempval = usec2ntp(tvalNow.tv_usec);
        reply[44] = (tempval >> 24) & 0XFF;
        reply[45] = (tempval >> 16) & 0XFF;
        reply[46] = (tempval >> 8) & 0XFF;
        reply[47] = (tempval)&0XFF;

        // Reply to the IP address and port that sent the NTP request

        Udp.beginPacket(Remote, PortNum);
        Udp.write(reply, NTP_PACKET_SIZE);
        Udp.endPacket();
    }
}
#endif

/*
void processPPS()
{
  if (pps > 0)
  {
    lastpps = pps;
    pps = 0;
    if (tbolt.getSoftwareVersionInfo())
    {
      lasttime = tmConvert_t(timeInfo.year(), tbolt.timeInfo.year() , timeInfo.Day, timeInfo.Hour, timeInfo.Minute, timeInfo.Second);
      // PPS es posterior a la marca de tiempo -> agregar un segundo
      lasttime++;
      timeval valnow; //declaration an struct with contain two typedata, one of them is (time_t tv_sec;)
      bfixed = true;
    }
    else
    {
      lastpps = 0;
      lasttime = 0;
    }
  }
}

void processGPS()
{
  while (Serial2.available()) // Read and parse TSIP packets arriving from the Thunderbolt when available
  {
    processPPS();
    tbolt.readSerial(); 
  } 
} */