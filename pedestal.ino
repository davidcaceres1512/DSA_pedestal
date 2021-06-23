#define ethernet_h_
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <aREST.h>
//#include <PubSubClient.h>
#include <IWatchdog.h>
#include <rBase64.h>
#include <ArduinoJson.h>
//#include <RunningMedian.h>
#include <NTPClient.h>
#include <EthernetUdp.h>
#include <STM32RTC.h>
#include "Pedestal.h"
#include <PangolinMQTT.h>

//#define PEDESTAL_SIMULATION

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#ifdef LED_BUILTIN
const int ledPin = LED_BUILTIN;
#else
const int ledPin = 13;
#endif

HardwareSerial Serial2(PD_6, PD_5);
HardwareSerial Serial4(PD_0, PD_1);
HardwareSerial Serial6(PG_9, PG_14);

Pedestal<uint8_t, PEDESTAL_MAX_BUFFER_SIZE> pedestal(Serial6);

uint8_t requested_axis = TX_ELEVATION; // It can be TX_AZIMUTH or TX_ELEVATION
double speed = 20;
double step = 5;    // Only if opmode = REQ_SCAN
double top = 60;    // Only if opmode = REQ_SCAN
double bottom = 45; // Only if opmode = REQ_SCAN
//double table_position[] = {0.1,31.6,45.0,90.0,179.0,120.0,10.1};
float table_position[200];
char decoded_table[800];
//StaticJsonDocument<2048> jsondoc;
DynamicJsonDocument jsondoc(2048);
char decoded_content[2048];

const long timeout = 500; // Timeout for the analysis of the data.
long lastTime = 0;

// IP address in case DHCP fails


IPAddress ip(10,10,10,27);
IPAddress subnet(255,255,255,0);
IPAddress gateway(10,10,10,1);
IPAddress mydns(10,10,10,1);

// Ethernet server
EthernetServer server(80);
// Ethernet client
EthernetClient ethClient;

// Create aREST instance
aREST rest = aREST();

PangolinMQTT mqttClient;
#define MQTT_HOST IPAddress(10,10,10,29)
//#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 1883
#define START_WITH_CLEAN_SESSION true
volatile bool mqtt_connected = false;
volatile bool mqtt_irq_flag = false;
uint8_t *mqtt_buffer = (uint8_t *)malloc(PEDESTAL_MAX_BUFFER_SIZE);
char coded_buffer[RBASE64_ENC_SIZECALC(PEDESTAL_MAX_BUFFER_SIZE)];
size_t coded_buffer_len = 0;
uint8_t interrupt_counter = 0;

/*
PubSubClient client(ethClient);
char broker[] = "broker.mqtt-dashboard.com";    // name address for mqtt broker (using DNS)
long lastReconnectAttempt = 0; // Reconnection of MQTT
bool mqtt_flag = false;
*/

//RunningMedian loop_time = RunningMedian(500);
unsigned long loop_max = 0;

const int port = 443; // port to listen on

STM32RTC &rtc = STM32RTC::getInstance();

EthernetUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pe.pool.ntp.org", 0, 10000);

const byte interruptPPS = 22;

//********************************************************************************************
//******************************** API FUNCTIONS DECLARATIONS ********************************
//********************************************************************************************
int scan(String command);
int table(String command);
int fixed_pos(String command);
int fixed_spd(String command);
int write_data(String command);
int changeip(String command);
int reset(String command);
int start(String command);
int stop(String command);
int calibrate(String command);
String status = "Hola mundo"; // status string variable, you can put any status message here.

#ifdef PEDESTAL_SIMULATION
void Pedestal_simulation_callback(void)
{
    uint8_t x = pedestal.getOpMode();
    if (x == OPMODE_SCAN || x == OPMODE_TABLE)
    {
        if (pedestal.getOpAxis() == TX_AZIMUTH)
        {
            pedestal.simAzPosition = fmod(pedestal.simAzPosition + pedestal.simAzSpeed * 0.02, 360.0);
            if (pedestal.simAzPosition < 360.0)
                pedestal.simAzPosition = pedestal.simAzPosition + 360.0;
        }
        else if (pedestal.getOpAxis() == TX_ELEVATION)
        {
            pedestal.simElPosition = constrain(pedestal.simElPosition + pedestal.simElSpeed * 0.02, 0, 180);
        }
    }
    pedestal.commandTX(MODE_POSITION, RX_AZIMUTH, (double *)&pedestal.simAzPosition, RX_FUNCTION, Serial4);
    delay(1);
    pedestal.commandTX(MODE_POSITION, RX_ELEVATION, (double *)&pedestal.simElPosition, RX_FUNCTION, Serial4);
    //Serial.println("Simulation data: Az="+String(pedestal.simAzPosition)+" El="+String(pedestal.simElPosition));
}
#endif

void onMqttConnect(bool sessionPresent)
{
    Serial.printf("Connected as %s: Max safe payload %u\n", mqttClient.getClientId(), mqttClient.getMaxPayloadSize());
    mqttClient.subscribe("JRO_topic", 0);
    String temp_payload = "Connected to broker. Operation mode: " + String(pedestal.getOpMode());
    //mqttClient.publish("JRO_topic",0,false,std::string(temp_payload.c_str()));
    //mqttClient.publish("JRO_topic",std::string((char*)coded_buffer),coded_buffer_len,0,false);
    mqttClient.publish("JRO_topic", (uint8_t *)temp_payload.c_str(), (size_t)temp_payload.length(), 0, false);
    mqtt_connected = true;
}
/*
void onMqttMessage(const char* topic, uint8_t* payload, struct PANGO_PROPS props, size_t len, size_t index, size_t total) {
  PANGO::dumphex(payload,len);
}
*/
void onMqttDisconnect(int8_t reason)
{
    Serial.printf("Disconnected from MQTT reason=%d\n", reason);
    mqtt_connected = false;
}

void mqtt_publish_buffer(void)
{
    if (interrupt_counter == 100)
    {
        mqtt_irq_flag = true;
        interrupt_counter = 0;
    }
    interrupt_counter++;
#ifdef PEDESTAL_SIMULATION
    uint8_t x = pedestal.getOpMode();
    if (x == OPMODE_SCAN || x == OPMODE_TABLE)
    {
        if (pedestal.getOpAxis() == TX_AZIMUTH)
        {
            pedestal.simAzPosition = fmod(pedestal.simAzPosition + pedestal.simAzSpeed * 0.02, 360.0);
            if (pedestal.simAzPosition < 360.0)
                pedestal.simAzPosition = pedestal.simAzPosition + 360.0;
        }
        else if (pedestal.getOpAxis() == TX_ELEVATION)
        {
            pedestal.simElPosition = constrain(pedestal.simElPosition + pedestal.simElSpeed * 0.02, 0, 180);
        }
    }
    pedestal.commandTX(MODE_POSITION, RX_AZIMUTH, (double *)&pedestal.simAzPosition, RX_FUNCTION, Serial4);
    delay(1);
    pedestal.commandTX(MODE_POSITION, RX_ELEVATION, (double *)&pedestal.simElPosition, RX_FUNCTION, Serial4);
    //Serial.println("Simulation data: Az="+String(pedestal.simAzPosition)+" El="+String(pedestal.simElPosition));
#endif
}

int option;

void setup()
{
    // put your setup code here, to run once:
    lastTime = 0;

    TIM_TypeDef *Instance1 = TIM1;
    //TIM_TypeDef *Instance2 = TIM2;
    // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
    //HardwareTimer *SimTim = new HardwareTimer(Instance2);
    HardwareTimer *PPSTim = new HardwareTimer(Instance1);

    pinMode(ledPin, OUTPUT);
    pinMode(interruptPPS, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(interruptPPS), mqtt_publish_buffer, RISING);
    Serial.begin(1000000);
    Serial.flush();
    Serial4.begin(115200);
    Serial4.flush();

    if (IWatchdog.isReset(true))
    {
        // LED blinks to indicate reset
        Serial.println("IWDG reset!!");
        for (uint8_t idx = 0; idx < 5; idx++)
        {
            digitalWrite(ledPin, HIGH);
            delay(100);
            digitalWrite(ledPin, LOW);
            delay(100);
        }
    }
    IWatchdog.clearReset();

    Serial.println("Begin program.");
    Serial.println();

    Serial.println("Initializing pedestal.");
    if (!pedestal.initialize())
        Serial.println("Error initializing Pedestal.");
    //if(!pedestal.setStopMode())
    //Serial.println("Error setting Stop mode.");
    //delay(10000);
    Serial.println("Size of class Pedestal: " + String(sizeof(pedestal)));
#ifdef PEDESTAL_SIMULATION
    if (!pedestal.handle())
        Serial.println("Error on handle.");
    Serial.println("Test reading azimuth: " + String(pedestal.getAzPosition(), 6));
    Serial.println("Test reading elevation: " + String(pedestal.getElPosition(), 6));
    Serial.println("Iterations analizing buffer: " + String(pedestal._max_read_iterations));
#endif

    pedestal.freeBuffer();

    // API Restful configuration
    // Init variables and expose them to REST API
    rest.variable("status", &status);
    // Function to be exposed
    rest.function("position", fixed_pos);
    rest.function("speed", fixed_spd);
    rest.function("scan", scan);
    rest.function("table", table);
    rest.function("write_data", write_data);
    rest.function("changeip", changeip);
    rest.function("reset", reset);
    rest.function("start", start);
    rest.function("stop", stop);
    rest.function("calibrate", calibrate);
    // Give name & ID to the device (ID should be 6 characters long)
    rest.set_id("001");
    rest.set_name("DSA_controller");

    // Ethernet configuration
    Serial.println("Configuring Ethernet interface...");
    // Start the Ethernet connection and the server
    //if (Ethernet.begin() == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    //Ethernet.begin(ip,subnet,gateway,mydns);
    Ethernet.begin(ip, subnet, gateway);
    //}
    delay(1000);
    server.begin();
    Serial.print("Server is at ");
    Serial.println(Ethernet.localIP());

    // NTP configuration
    timeClient.begin();
    Serial.println("NTP client started.");
    Serial.println("Geting NTP server time reference.");
    rtc.begin();
    timeClient.update();
    rtc.setEpoch(timeClient.getEpochTime());
    Serial.print("Epoch time from NTP server: ");
    Serial.println(timeClient.getEpochTime());
    Serial.println(timeClient.getFormattedTime() + " UTC");

    // MQTT configuration
    mqtt_connected = false;
    mqtt_irq_flag = false;
    coded_buffer_len = 0;
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    //mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setWill("NUCLEO_DIED", 2, false, "It's 'Alpha': probably still some bugs");
    mqttClient.setCleanSession(START_WITH_CLEAN_SESSION);
    //  mqttClient.setClientId("f429zi");
    mqttClient.connect();

    // Start watchdog
    IWatchdog.clearReset();
    //IWatchdog.begin(4000000);
    IWatchdog.begin(12000000);
    if (!IWatchdog.isEnabled())
    {
        // LED blinks indefinitely
        while (1)
        {
            digitalWrite(ledPin, HIGH);
            delay(500);
            digitalWrite(ledPin, LOW);
            delay(500);
            Serial.println("Watchdog is not enabled!");
        }
    }

    interrupt_counter = 0;
    PPSTim->setOverflow(100, HERTZ_FORMAT); // 100 Hz
    PPSTim->attachInterrupt(mqtt_publish_buffer);
    PPSTim->resume();

    Serial.println("Leaving \"setup()\" now.");
    Serial.println();
    IWatchdog.reload();
    if (!pedestal.handle())
        Serial.println("Error on handle.");
    IWatchdog.reload();
    pedestal._max_uart_buffer_size = 0;
    pedestal._max_read_iterations = 0;
}

void loop()
{
    if (Serial.available()>0){
    //leemos la opcion enviada
    option=Serial.read();
    if(option=='a') {
      Serial.println("OFF");
      delay(12001);
    }
  }


    //unsigned long start_loop = micros();
    // put your main code here, to run repeatedly:
    // listen for incoming clients
    ethClient = server.available();
    IWatchdog.reload();

    rest.handle(ethClient);
    IWatchdog.reload();

    if (!pedestal.handle())
        Serial.println("Error on handle.");
    IWatchdog.reload();

    if (mqtt_irq_flag && mqtt_connected)
    {
        timeClient.update();
        rtc.setEpoch(timeClient.getEpochTime());
        IWatchdog.reload();
        uint32_t temptime = timeClient.getEpochTime();
        uint16_t i = 0;
        while (pedestal.getBufferSize() > 69)
        {
            *(mqtt_buffer + i) = pedestal.shiftBuffer();
            i++;
        }
        uint16_t j = 0;
        while (j < pedestal.getBufferSize())
        {
            *(mqtt_buffer + i + j) = pedestal.getBufferData(j);
            j++;
        }
        *(mqtt_buffer + i + j) = (byte)(temptime & 0xFF);
        *(mqtt_buffer + i + j + 1) = (byte)((temptime >> 8) & 0xFF);
        *(mqtt_buffer + i + j + 2) = (byte)((temptime >> 16) & 0xFF);
        *(mqtt_buffer + i + j + 3) = (byte)((temptime >> 24) & 0xFF);
        coded_buffer_len = rbase64_encode((char *)coded_buffer, (char *)mqtt_buffer, i + j + 4);
        IWatchdog.reload();
        //mqttClient.publish("JRO_topic",0,false,std::string((char*)coded_buffer));
        if (coded_buffer_len >= 8192)
            Serial.println("WARNING: MQTT buffer is too big " + String(coded_buffer_len));
        mqttClient.publish("JRO_topic", (uint8_t *)coded_buffer, coded_buffer_len, 0, false);
        mqtt_irq_flag = false;
    }

    long now = millis();
    if (now - lastTime >= timeout)
    {
        lastTime = now;

        if (!mqtt_connected)
        {
            mqttClient.setCleanSession(START_WITH_CLEAN_SESSION);
            mqttClient.connect();
        }

        Serial.println("Operation mode: 0x0" + String(pedestal.getOpMode(), HEX));
        Serial.println("Azimuth speed: " + String(pedestal.getAzSpeed()));
        Serial.println("Elevation speed: " + String(pedestal.getElSpeed()));
        Serial.println("Azimuth position: " + String(pedestal.getAzPosition()));
        Serial.println("Elevation position: " + String(pedestal.getElPosition()));
        Serial.println("Buffer index: " + String(pedestal.getBufferIndex()));
        Serial.println("Buffer size: " + String(pedestal.getBufferSize()));
        Serial.println("Maximum UART data to handle: " + String(pedestal._max_uart_buffer_size));
        Serial.println("Maximum iterations analizing buffer: " + String(pedestal._max_read_iterations));
        Serial.println();
        //pedestal._max_uart_buffer_size = 0;
        //pedestal._max_read_iterations = 0;
        if (pedestal._max_uart_buffer_size == 0)
            status = "PEDESTAL_ERROR"; // The pedestal is not responding or not connected to the controller
        else
            switch (pedestal.getOpMode())
            {
            case OPMODE_IDLE:
                status = "OPMODE_IDLE"; // The controller is in idle mode waiting for a command.
            case OPMODE_STOP:
                status = "OPMODE_STOP"; // The controller is in stop mode. The user can send speed and position commands.
            case OPMODE_CALIBRATION:
                status = "OPMODE_CALIBRATION"; // The controller is in calibration mode. The user has to send a stop command before changing to another mode.
            case OPMODE_TABLE:
                status = "OPMODE_TABLE"; // The controller is in table mode. The user has to send a stop command before changing to another mode.
            case OPMODE_SCAN:
                status = "OPMODE_SCAN"; // The controller is in scan mode. The user has to send a stop command before changing to another mode.
            default:
                status = "OPMODE_ERROR"; // Unknown operation mode, there has been some kind of error.
            }
    }
    IWatchdog.reload();
    //unsigned long end_loop = micros();
    //loop_time.add(end_loop-start_loop);
    //unsigned long x = loop_time.getHighest();
    //if (loop_max < x)
    //loop_max = x;
}

bool decode_command(String command)
{
    const char *raw_content = command.c_str();
    //char decoded_content[command.length()];
    size_t len = rbase64_decode(decoded_content, (char *)raw_content, command.length());
    Serial.print("  - Decoded Output Buffer Length: ");
    Serial.println(len);
    Serial.print("  - Decoded: ");
    Serial.println(decoded_content);
    Serial.println();

    //const size_t capacity = 2048;
    //DynamicJsonDocument doc(capacity);
    //deserializeJson(jsondoc, decoded_content,command.length());
    deserializeJson(jsondoc, decoded_content, len);
    return true;
}

//*******************************************************************************************
//******************************** API FUNCTIONS DEFINITIONS ********************************
//*******************************************************************************************

int fixed_pos(String command)
{
    Serial.println("\"position\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    //DynamicJsonDocument doc = decode_command(command);
    decode_command(command);
    String axis = String((const char *)jsondoc["axis"]);
    requested_axis = NONE_AXIS;
    double fix_position = jsondoc["position"];
    Serial.println("Position is: " + String(fix_position));

    if (axis.equalsIgnoreCase("azimuth"))
        requested_axis = TX_AZIMUTH;
    else if (axis.equalsIgnoreCase("elevation"))
        requested_axis = TX_ELEVATION;
    else
        return 0;

    if (!pedestal.setAxisPosition(requested_axis, (double *)(&fix_position)))
        Serial.println("Error sending position command.");

    return 1;
}
int fixed_spd(String command)
{
    Serial.println("\"speed\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    //DynamicJsonDocument doc = decode_command(command);
    decode_command(command);
    String axis = String((const char *)jsondoc["axis"]);
    requested_axis = NONE_AXIS;
    double fix_speed = jsondoc["speed"];
    Serial.println("Speed is: " + String(fix_speed));

    if (axis.equalsIgnoreCase("azimuth"))
        requested_axis = TX_AZIMUTH;
    else if (axis.equalsIgnoreCase("elevation"))
        requested_axis = TX_ELEVATION;
    else
        return 0;

    if (!pedestal.setAxisSpeed(requested_axis, (double *)(&fix_speed)))
        Serial.println("Error sending speed command.");

    return 1;
}
int scan(String command)
{
    Serial.println("\"scan\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    //DynamicJsonDocument doc = decode_command(command);
    decode_command(command);
    String axis = String((const char *)jsondoc["axis"]);
    requested_axis = NONE_AXIS;
    speed = jsondoc["speed"];
    step = jsondoc["step"];
    bottom = jsondoc["bottom"];
    top = jsondoc["top"];

    if (axis.equalsIgnoreCase("azimuth"))
        requested_axis = TX_AZIMUTH;
    else if (axis.equalsIgnoreCase("elevation"))
        requested_axis = TX_ELEVATION;
    else
    {
        Serial.println("Scan error.");
        return 0;
    }

    if (!pedestal.setScanMode(requested_axis, speed, step, bottom, top))
        Serial.println("Error setting Scan mode.");

    return 1;
}
int table(String command)
{
    Serial.println("\"table\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    //DynamicJsonDocument doc = decode_command(command);
    decode_command(command);
    Serial.println("Command decoded.");

    speed = jsondoc["speed"];
    String axis = String((const char *)jsondoc["axis"]);
    Serial.println("String \"axis\" is: " + axis);
    requested_axis = NONE_AXIS;
    if (axis.equalsIgnoreCase("azimuth"))
        requested_axis = TX_AZIMUTH;
    else if (axis.equalsIgnoreCase("elevation"))
        requested_axis = TX_ELEVATION;
    else
        return 0;
    //String coded_table_str = String((const char*)jsondoc["table"]);
    const char *coded_table = jsondoc["table"];
    String coded_table_str = String(coded_table);
    Serial.println("String \"table\" is: " + coded_table_str);
    const char *coded_table_content = coded_table_str.c_str();
    //const char* coded_table_content = str_coded_table.c_str();
    size_t decodedlength = rbase64_dec_len((char *)coded_table_content, coded_table_str.length());
    Serial.println("JSON character coded table size is: " + String(coded_table_str.length()));
    rbase64_decode(decoded_table, (char *)coded_table_content, coded_table_str.length());
    CharArrayToFloatArray(table_position, decoded_table, decodedlength);

    if (!pedestal.setTableMode(requested_axis, speed, table_position, decodedlength / 4))
        Serial.println("Error setting Table mode.");

    return 1;
}
int write_data(String command)
{
    Serial.println("\"write_data\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);

    return 1;
}
int changeip(String command)
{
    Serial.println("\"changeip\" command selected.");
    Serial.println("command content: " + command);
    //DynamicJsonDocument doc = decode_command(command);
    decode_command(command);
    JsonArray ip_j = jsondoc["ip"];
    JsonArray gateway_j = jsondoc["gateway"];
    JsonArray mydns_j = jsondoc["mydns"];
    JsonArray subnet_j = jsondoc["subnet"];
    Serial.println(ip);
    ip = {ip_j[0], ip_j[1], ip_j[2], ip_j[3]};
    gateway = {gateway_j[0], gateway_j[1], gateway_j[2], gateway_j[3]};
    mydns = {mydns_j[0], mydns_j[1], mydns_j[2], mydns_j[3]};
    subnet = {subnet_j[0], subnet_j[1], subnet_j[2], subnet_j[3]};
    Serial.println(ip);

    return 1;
}
int reset(String command)
{
    Serial.println("\"reset\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);

    return 1;
}
int start(String command)
{
    Serial.println("\"start\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);

    return 1;
}
int stop(String command)
{
    Serial.println("\"stop\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);
    if (!pedestal.setStopMode())
        Serial.println("Error setting Table mode.");

    return 1;
}
int calibrate(String command)
{
    Serial.println("\"calibrate\" command selected.");
    Serial.print("command content: ");
    Serial.println(command);

    return 1;
}