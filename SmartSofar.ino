#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <time.h>
#include <ESPAsyncTCP.h> 
#include <ESPAsyncWebSrv.h>
#include <UptimeString.h>
//#include "TinyMqtt.h"

const char* deviceName = "SmartSofar";
const char* version = "v1.0";
const char* WIFI_SSID = ""; //specify your Wifi SSID
const char* WIFI_PASSWORD = ""; //specify your wifi password
const char* SHELLY_IP = "192.168.1."; //specify your Shelly M3 Pro IP address
uint8_t INVERTER_SLAVE_ID[] = {1,2}; //{0x01,0x02}; Define a Slave ID for each inverter (each separated by a comma), It's important to define the slave ID(s) that are set in the inverter(s) settings. 
int ImmediateAdjustW = 500; //in ms, immediatelly adjust power if first Shelly power measured value > specified value 
int DelayBetweenReads = 1000; //in ms, delay between 2 Shelly reads
int ChecksToReach = 2; //numbers of successive values in the range of the Delta to be achieved in between Shelly reads, only then power adjustment will be made;
int MaxCycles = 10; //maximum cycles of Shelly reads allowed before adjusting ayway;
int MaxDelta = 30; //maximum Delta in W in between Shelly reads values to be accounted as acceptable;

/* Configuration of NTP */
#define MY_NTP_SERVER "at.pool.ntp.org" //provide NTP server
// choose your time zone from this list
// https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
#define MY_TZ "CET-1CEST,M3.5.0,M10.5.0/3" //provide time zone based on list in CSV
time_t now;                          // this are the seconds since Epoch (1970) - UTC
tm tm;                             // the structure tm holds time information in a more convenient way *

// Timers settings
#define SYSHEALTH_INTERVAL    10000 // get system uptime & ESP free heap
#define HEARTBEAT_INTERVAL    15000 // min frequency: 1s, max 60s */ //default=10000
#define SOFAR_INTERVAL        10000 //default=5000
#define SHELLY_INTERVAL        2000 //default = 2000
#define DISPLAY_INTERVAL        2000 //default=2000
#define MIN_TIME_BETWEEN_TWO_REG_READS 1400 //default=1000

UptimeString uptimeString;

//#define PORT 1883
//MqttBroker broker(PORT);
//MqttClient mqtt_client(&broker);
//std::string topicMode=std::string(deviceName)+"/setmode";
//std::string topicPower=std::string(deviceName)+"/setpower";

const char* wifiName = WIFI_SSID;

AsyncWebServer server(80);
const char* PARAM_MESSAGE = "message";
const char* PARAM_INPUT_1 = "cmd";
const char* PARAM_INPUT_2 = "val";

const int INVERTER_COUNTS = sizeof(INVERTER_SLAVE_ID);
int INVERTER_RUNNINGSTATE[INVERTER_COUNTS];
int BATTERY_POWER[INVERTER_COUNTS];
int pwAdjRequest = 0;
int pwManRequest = 0;
int pwLastManRequest = -1;
int Delta = 40; //Minimum delta (W) before to re-adjust charge/discharge setpoint
int SetPoint = 0; //Charge/discharge setpoint
bool IsAuto = 0;
char currentLog[1500];
int DisplayCycle = 0;
String DisplayData[INVERTER_COUNTS+1][4];

// Update these to match your inverter/network.
#define INVERTER_ME3000       // Uncomment for ME3000
//#define INVERTER_HYBRID     // Uncomment for Hybrid

#define SERIAL_COMMUNICATION_CONTROL_PIN D5 // Transmission set pin
#define RS485_TX HIGH
#define RS485_RX LOW
#define SOFAR_RXPin        D6  // Serial Receive pin
#define SOFAR_TXPin        D7  // Serial Transmit pin

struct modbusResponse;
bool g_sofar_last_heartbeat = false;
bool g_regReadOk = false;
static unsigned long  lastHeartBeat = 0;

#if (! defined INVERTER_ME3000) && ! defined INVERTER_HYBRID
#error You must specify the inverter type.
#endif

#ifdef INVERTER_ME3000
#define MAX_POWER   3000    // ME3000 is 3000W max.
#elif defined INVERTER_HYBRID
#define MAX_POWER   6000
#endif

//analisys of Me300SP responses:
//- most of the queries are answered within 150ms
//- ocasinally response is much later, up to 1200ms
//We need to listen for those delayed responses, otherwise we might send another request, and read response for the previous query
#define RS485_TRIES 120       // x 10mS to wait for RS485 input chars.

#define MAX_DROP_VALUE_REPORT_PAUSE_SEC 3600  //if value drops, don't report it for 1h

// SoftwareSerial is used to create a second serial port, which will be deidcated to RS485.
// The built-in serial port remains available for flashing and debugging.
SoftwareSerial RS485Serial(SOFAR_RXPin, SOFAR_TXPin);

uint32_t g_sofar_heartbeat_ok  = 0;
uint32_t g_sofar_heartbeat_err = 0;

uint32_t g_sofar_req_read_ok  = 0;
uint32_t g_sofar_req_read_err = 0;

// Sofar run states
#define waiting 0
#define check 1

#ifdef INVERTER_ME3000
#define charging          2
#define checkDischarge    3
#define discharging       4
#define epsState          5
#define faultState        6
#define permanentFaultState 7

#elif defined INVERTER_HYBRID
#define normal              2
#define epsState            3
#define faultState          4
#define permanentFaultState 5
#define normal1             6

// State names are a bit strange - makes sense to also match to these?
#define charging      2
#define discharging   6
#endif

#define MAX_FRAME_SIZE          255
#define MODBUS_FN_READSINGLEREG 0x03
#define MODBUS_FN_HEARTBEAT     0x49
#define SOFAR_FN_PASSIVEMODE    0x42
#define SOFAR_PARAM_STANDBY     0x5555

// SoFar ME3000 Information Registers
#define SOFAR_REG_RUNSTATE  0x0200
#define SOFAR_REG_GRIDV     0x0206
#define SOFAR_REG_GRIDA     0x0207
#define SOFAR_REG_GRIDFREQ  0x020c
#define SOFAR_REG_BATTW     0x020d
#define SOFAR_REG_BATTV     0x020e
#define SOFAR_REG_BATTA     0x020f
#define SOFAR_REG_BATTSOC   0x0210
#define SOFAR_REG_BATTTEMP  0x0211
#define SOFAR_REG_GRIDW     0x0212
#define SOFAR_REG_LOADW     0x0213
#define SOFAR_REG_PVW       0x0215
#define SOFAR_REG_PVDAY     0x0218
#define SOFAR_REG_EXPDAY    0x0219
#define SOFAR_REG_IMPDAY    0x021a
#define SOFAR_REG_LOADDAY   0x021b
#define SOFAR_REG_BATTCYC   0x022c
#define SOFAR_REG_PVA       0x0236
#define SOFAR_REG_INTTEMP   0x0238
#define SOFAR_REG_HSTEMP    0x0239
#define SOFAR_REG_PV1       0x0252
#define SOFAR_REG_PV2       0x0255

#define SOFAR_REG_FIRST     SOFAR_REG_RUNSTATE
#define SOFAR_REG_LAST      SOFAR_REG_PV2
#define SOFAR_REG_COUNT     ((SOFAR_REG_LAST-SOFAR_REG_FIRST)+1)

//we read all registers in bulk, to minimize number of round trips
//this essentialy is up-to-date copy of sofar registers
int16_t all_regs[SOFAR_REG_COUNT];

#define SOFAR_FN_STANDBY    0x0100
#define SOFAR_FN_DISCHARGE  0x0101
#define SOFAR_FN_CHARGE     0x0102
#define SOFAR_FN_AUTO       0x0103
#define SOFAR_FN_UNKNOWN    0x010F
#define SOFAR_FN_FIRST      SOFAR_FN_STANDBY

struct status_register
{
  uint16_t regnum;
  String   name;
  double   multiplier;
  bool     must_go_up; //value must go up over time, by max MAX_INCREASE_BETWEEN_READINGS between readings
  int16_t  prev_value;
  unsigned long value_drop_time; //in seconds since value dropped
  String   EngUnit;
};

static struct status_register  status_reads[] =
{
  { SOFAR_REG_RUNSTATE, "running_state"      ,1, false, 0,0,""},
  { SOFAR_REG_GRIDV,    "grid_voltage"     ,0.1, false, 0,0,"V"},
  { SOFAR_REG_GRIDFREQ, "grid_freq"       ,0.01, false, 0,0,"Hz"},
  { SOFAR_REG_BATTV,    "battery_voltage" ,0.01, false, 0,0,"V"},
  { SOFAR_REG_BATTW,    "battery_power"      ,10, false, 0,0,"W"},
  { SOFAR_REG_BATTSOC,  "battery_soc"        ,1, false, 0,0,"%"},
  { SOFAR_REG_BATTTEMP, "battery_temp"       ,1, false, 0,0,"°C"},
  { SOFAR_REG_BATTCYC,  "battery_cycles"     ,1, false, 0,0,""},
//IZIZ: not relable?  { SOFAR_REG_GRIDA, "grid_current" ,0.01, false, 0,0,""},
//Sent in "power"     { SOFAR_REG_GRIDW,  "grid_power",10, false, 0,0,""},
//Sent in "power"     { SOFAR_REG_BATTW,  "battery_power"  ,10, false, 0,0,""},
//Not important       { SOFAR_REG_BATTA, "battery_current" ,0.01, false, 0,0,""},
//Sent in "power"     { SOFAR_REG_LOADW, "consumption_power" ,10, false, 0,0,""},
//Sent in "power"     { SOFAR_REG_PVW, "pv_power" ,10, false, 0,0,""},
//Not important       { SOFAR_REG_PVA, "solarPV_current" ,0.01, false, 0,0,""}, //amps
  { SOFAR_REG_PVDAY, "today_generated" ,10, true, 0,0,"Wh"},
#ifdef INVERTER_ME3000
  { SOFAR_REG_EXPDAY, "today_exported" ,10, true, 0,0,"Wh"},
  { SOFAR_REG_IMPDAY, "today_imported" ,10, true, 0,0,"Wh"},
#elif defined INVERTER_HYBRID
  { SOFAR_REG_PV1, "Solarpv1" ,1, false, 0,0,"Wh"},
  { SOFAR_REG_PV2, "Solarpv2" ,1, false, 0,0,"Wh"},
#endif
  { SOFAR_REG_LOADDAY, "today_consumed" ,10, true,  0,0,"Wh"},
  { SOFAR_REG_INTTEMP, "inverter_temp"   ,1, false, 0,0,"°C"},
  { SOFAR_REG_HSTEMP, "inverter_HStemp"  ,1, false, 0,0,"°C"}
};

const int primaryInfos = 3;
String WebData[primaryInfos+((sizeof(status_reads)/sizeof(struct status_register))*INVERTER_COUNTS)][4]; //Primary Informations + status_reads * device(s) ... 4 = Item, Parameter, Value, EngUnit

// This is the return object for the sendModbus() function. Since we are a modbus master, we
// are primarily interested in the responses to our commands.

struct modbusResponse
{
  uint8_t errorLevel;
  uint8_t data[MAX_FRAME_SIZE];
  uint8_t dataSize;
  const char* errorMessage;
};

int reg_to_data_index(uint16_t reg_addr)
{
  return reg_addr - SOFAR_REG_FIRST;
}

bool sofar_getSingleRegFromCache(uint16_t reg, int16_t& val)
{
  const int index = reg_to_data_index(reg);
  if(index >= SOFAR_REG_COUNT)
  {
    return false;
  }
  
  val = all_regs[index];
  return true;
}

bool sofar_addStateInfo(String &state, uint16_t reg, const String& human, double multiplier, bool must_go_up, int16_t& prev_value, unsigned long& value_drop_time, String EngUnit, int index)
{
  int16_t val;
  if(sofar_getSingleRegFromCache(reg, val) == false)
  {
    return false;
  }

  prev_value = val;

  if (!( state == ""))
    state += ",";

  char szLine[64] = "";
  String topicPub=String(deviceName)+"/Charger"+String(INVERTER_SLAVE_ID[index])+"_"+human;
  if(multiplier < 1)
  {
    snprintf(szLine, sizeof(szLine)-1, "%s: %.2f", human.c_str(), ((double)val)*multiplier);
    //mqtt_client.publish(topicPub.c_str(), String(((double)val)*multiplier));
  }
  else
  {
    snprintf(szLine, sizeof(szLine)-1, "%s: %d", human.c_str(), (int)(val*multiplier));
    //mqtt_client.publish(topicPub.c_str(), String((int)(val*multiplier)));
  }

  for(unsigned int l = 0; l < (sizeof(WebData) / sizeof(WebData[0])); l++) {
    if(WebData[l][0]=="Charger"+String(INVERTER_SLAVE_ID[index]) && WebData[l][1]==human.c_str()) {
      
      if(human!="running_state") {
        if(multiplier < 1) {
          WebData[l][2] = String(((double)val)*multiplier);
        } else {
          WebData[l][2] = String((int)(val*multiplier));
        }
        if(human=="battery_power"){BATTERY_POWER[index]=(int)(val*multiplier);}
      }
      if(human=="running_state"){
        int run_state = (int)(val*multiplier);
        switch(run_state) {
          case waiting:
              WebData[l][2]="Standby";
          break;

          case check:
            WebData[l][2]="Checking";
          break;

          case charging:
            WebData[l][2]="Charging";
          break;

#ifdef INVERTER_ME3000
          case checkDischarge:
            WebData[l][2]="Check Dis.";
          break;
#endif
          case discharging:
            WebData[l][2]="Dischar.";
          break;

          case epsState:
            WebData[l][2]="EPS State";
          break;

          case faultState:
            WebData[l][2]="FAULT";
          break;

          case permanentFaultState:
            WebData[l][2]="PERMFAULT";
          break;

          default:
            WebData[l][2]="Runstate?";
          break;
        }
        INVERTER_RUNNINGSTATE[index]=(int)(val*multiplier);
      }
      break;
    }
  }    
  
  state += szLine;
  return true;
}

// Wemos OLED Shield set up. 64x48, pins D1 and D2
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

// Update the OLED. Use "NULL" for no change or "" for an empty line.
String oledLine1;
String oledLine2;
String oledLine3;
String oledLine4;

void updateOLED(String line1, String line2, String line3, String line4)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  if(line1 != "NULL")
  {
    display.println(line1);
    oledLine1 = line1;
  }
  else
    display.println(oledLine1);

  display.setCursor(0,12);

  if(line2 != "NULL")
  {
    display.println(line2);
    oledLine2 = line2;
  }
  else
    display.println(oledLine2);

  display.setCursor(0,24);

  if(line3 != "NULL")
  {
    display.println(line3);
    oledLine3 = line3;
  }
  else
    display.println(oledLine3);

  display.setCursor(0,36);

  if(line4 != "NULL")
  {
    display.println(line4);
    oledLine4 = line4;
  }
  else
    display.println(oledLine4);

  display.display();
}

void RefreshDisplay() {
  static unsigned long  lastRun = 0;
  if(checkTimer(&lastRun, DISPLAY_INTERVAL)) {
    updateOLED(DisplayData[DisplayCycle][0], DisplayData[DisplayCycle][1], DisplayData[DisplayCycle][2], DisplayData[DisplayCycle][3]);
    DisplayCycle += 1;
    if (DisplayCycle > INVERTER_COUNTS) {DisplayCycle = 0;}
  }
}

void sofar_getData()
{   
  char szLine[500] = "";

  static unsigned long  lastRun = 0;
  if(checkTimer(&lastRun, SOFAR_INTERVAL))
  {
    
    for(int i = 0; i < INVERTER_COUNTS; i++) {

      delay(300);

      g_regReadOk = refresh_registers(SOFAR_REG_FIRST, SOFAR_REG_COUNT, INVERTER_SLAVE_ID[i]);

      if (g_regReadOk==false)
      {
        snprintf(szLine, sizeof(szLine)-1, "Error : Size read error on charger %s", String(INVERTER_SLAVE_ID[i]));
        DoLog(szLine);
        UpdateDisplayArray(i+1, "NULL", "NULL", "Charger " + String(INVERTER_SLAVE_ID[i]), "Read Error");
        continue;
      }

      String state = ""; //"{";
      state.reserve(500);

      for(unsigned int l = 0; l < sizeof(status_reads)/sizeof(struct status_register); l++)
      {
        if(sofar_addStateInfo(state, status_reads[l].regnum, 
                                    status_reads[l].name, 
                                    status_reads[l].multiplier,
                                    status_reads[l].must_go_up,
                                    status_reads[l].prev_value,
                                    status_reads[l].value_drop_time,
                                    status_reads[l].EngUnit,
                                    i) == false)
        {
          //we send either all, or no values
          continue;
        }
      }

      updateRunstate(i);

      snprintf(szLine, sizeof(szLine)-1, "Info : Read charger %s = %s", String(INVERTER_SLAVE_ID[i]), state.c_str());
      DoLog(szLine);
      

    }
  }
}

/**
 * Flush the RS485 buffers in both directions. The doc for Serial.flush() implies it only
 * flushes outbound characters now... I assume RS485Serial is the same.
 */
void sofar_flushRS485()
{
  RS485Serial.flush();
  delay(100);

  while(RS485Serial.available())
    RS485Serial.read();
}

void ensure_delay_between_reads()
{
  static unsigned long lastSentData = 0;
  const unsigned long l_now = millis();
  if(l_now < lastSentData)
  {
    //timer wrap - do full wait
    delay(MIN_TIME_BETWEEN_TWO_REG_READS);
  }
  else if(l_now - lastSentData < MIN_TIME_BETWEEN_TWO_REG_READS)
  {
    delay(l_now - lastSentData);
  }

  lastSentData = millis();
}

//protocol description:
//file:///C:/Users/irekz/Downloads/SOFARSOLAR%20ModBus-RTU%20Communication%20Protocol.pdf
//https://community.openenergymonitor.org/uploads/short-url/dF8j79PsvtCS0xQ5aPxHXO4HZDE.pdf
int sofar_sendModbus(uint8_t frame[], byte frameSize, modbusResponse *resp, uint8_t SLAVE_ID, bool flush = false)
{
  //Calculate the CRC and overwrite the last two bytes.
  calcCRC(frame, frameSize);

  // Make sure there are no spurious characters in the in/out buffer, before we send request
  if(flush || RS485Serial.available() > 0)
  {
    sofar_flushRS485();
  }

  ensure_delay_between_reads();

  //Send
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX);
  RS485Serial.write(frame, frameSize);

  // It's important to reset the SERIAL_COMMUNICATION_CONTROL_PIN as soon as
  // we finish sending so that the serial port can start to buffer the response.
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);

  const int ret = sofar_listen(resp, frame[1], SLAVE_ID);
  if(ret == -3) //we got hartbeat instead of response we want, repeat
  {
    return sofar_listen(resp, frame[1], SLAVE_ID);
  }
  else
  {
    return ret;
  }
}

// Listen for a response.
int sofar_listen(modbusResponse *resp, uint8_t req, uint8_t SLAVE_ID)
{
  char szLine[128];
  uint8_t   inFrame[MAX_FRAME_SIZE] = {0};
  uint8_t   inByteNum = 0;
  uint8_t   inFrameSize = 0;
  uint8_t   inFunctionCode = 0;
  uint8_t   inDataBytes = 0;
  int   done = 0;
  modbusResponse  dummy;

#ifdef SOFAR_LOG_RESPONSE_TIME
  const unsigned long t_start = millis();
#endif  
  
  if(!resp)
    resp = &dummy;      // Just in case we ever want to interpret here.

  resp->dataSize = 0;
  resp->errorLevel = 0;

  while((!done) && (inByteNum < sizeof(inFrame)))
  {
    int tries = 0;

    while((!RS485Serial.available()) && (tries++ < RS485_TRIES))
      delay(20); //orig = 10

    if(tries >= RS485_TRIES)
    {
      //Log("Timeout waiting for RS485 response.");
      //char szLine[64];
      snprintf(szLine, sizeof(szLine)-1, "Warning : Timeout waiting for RS485 response");
      DoLog(szLine);
      break;
    }

    inFrame[inByteNum] = RS485Serial.read();

    //Process the byte
    switch(inByteNum)
    {
      case 0:
        if(inFrame[inByteNum] != SLAVE_ID)   //If we're looking for the first byte but it dosn't match the slave ID, we're just going to drop it.
          inByteNum--;          // Will be incremented again at the end of the loop.
        break;

      case 1:
        //This is the second byte in a frame, where the function code lives.
        inFunctionCode = inFrame[inByteNum];
        break;

      case 2:
        //This is the third byte in a frame, which tells us the number of data bytes to follow.
        if((inDataBytes = inFrame[inByteNum]) > sizeof(inFrame))
        inByteNum = -1;       // Frame is too big?
        break;

      default:
        if(inByteNum < inDataBytes + 3)
        {
          //This is presumed to be a data byte.
          resp->data[inByteNum - 3] = inFrame[inByteNum];
          resp->dataSize++;
        }
        else if(inByteNum > inDataBytes + 3)
          done = 1;
    }

    inByteNum++;
  }

  inFrameSize = inByteNum;

  //Now check to see if the last two bytes are a valid CRC.
  //If we don't have a response pointer we don't care.
  if(inFrameSize < 5)
  {
    snprintf(szLine, sizeof(szLine)-1, "Error : Response too short: %d - %X %X %X %X", inFrameSize, inFrame[0], inFrame[1], inFrame[2], inFrame[3]);
    resp->errorLevel = 2;
    resp->errorMessage = szLine;
  }
  else if(checkCRC(inFrame, inFrameSize))
  {
    if(inFunctionCode != req)
    {
      if(inFunctionCode == MODBUS_FN_HEARTBEAT && 
         RS485Serial.available() > 0)
      {
        snprintf(szLine, sizeof(szLine)-1, "Info : Got heartbeat during listen, but more data available: %d", RS485Serial.available());
        resp->errorLevel = 3;
        resp->errorMessage = "Warn: got heartbeat instead data, repeat listen";
      }
      else
      {
        static int errc = 0;
        if(errc < 100)
        {
          snprintf(szLine, sizeof(szLine)-1, "Info : Fun resp: %d, expected: %d, len: %d",inFunctionCode, req, inByteNum);
          DoLog(szLine);
          errc++;
        }
        
        resp->errorLevel = 4;
        resp->errorMessage = "Error: invalid response type";
      }
    }    
    else
    {
      resp->errorLevel = 0;
      resp->errorMessage = "Valid data frame";
    }
  }
  else
  {
    resp->errorLevel = 1;
    resp->errorMessage = "Error: invalid data frame";
  }

  if(resp->errorLevel)
  {
    DoLog(resp->errorMessage);
  }
  
#ifdef SOFAR_LOG_RESPONSE_TIME
  int t_elapsed = (int)(millis()-t_start);
  if(resp->errorLevel)
  {
    t_elapsed = -t_elapsed;
  }
  
  if(req == 3)
  {
    if(t_elapsed > 400)
    {
      resp_stats_reg.push_back(t_elapsed);
    }
  }
  else
  {
    if(t_elapsed > 400)
    {
      resp_stats_hb.push_back(t_elapsed);
    }
  }
#endif  

  return -resp->errorLevel;
}

int sofar_sendPassiveCmd(uint8_t SLAVE_ID, uint16_t cmd, uint16_t param, String pubTopic)
{
  char szLine[128];
  modbusResponse  rs;
  uint8_t frame[] = { SLAVE_ID, SOFAR_FN_PASSIVEMODE, (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xff), (uint8_t)(param >> 8), (uint8_t)(param & 0xff), 0, 0 };
  int   err = -1;

  if(sofar_sendModbus(frame, sizeof(frame), &rs, SLAVE_ID, true)) {
    //retMsg += rs.errorMessage;
  //else if(rs.dataSize != 2)
    //retMsg += "ERR: Reponse is " + String(rs.dataSize) + " bytes?";
  } else  {
    if(rs.data[1] == 0)
    {
      err = 0;
    }
    else if(rs.data[1] == 1)
    {
      //for details of error codes see: https://github.com/greentangerine/ME3000/blob/master/ME3000SP%20Passive%20mode%20protocol.pdf?fbclid=IwAR3xBjKbdOZ3OxD3G1XDv6lOms5wBjtBbxs8t6IxvEHnisR3rnkj4PWyO7s
      err = 1;      
    }
    else
    {
      err = rs.data[1];
    }
  }

  if(err) {
    snprintf(szLine, sizeof(szLine)-1, "Error : Cannot send adjusting command to charger %s", String(SLAVE_ID));//, retMsg);
    DoLog(szLine);
  } else {
    lastHeartBeat = millis();
  }
    
  return err;
}

void sofar_heartbeat()
{

long now = millis();

if(lastHeartBeat > now)
  lastHeartBeat = 0;

 if(now >= lastHeartBeat + HEARTBEAT_INTERVAL)
 {
    lastHeartBeat = now;
    static unsigned long  errInRow[INVERTER_COUNTS];
    char szLine[128];
    for(int i = 0; i < INVERTER_COUNTS; i++) {
      uint8_t sendHeartbeat[] = {INVERTER_SLAVE_ID[i], MODBUS_FN_HEARTBEAT, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
      int ret;
      updateOLED("NULL", "Heart B "  + String(INVERTER_SLAVE_ID[i]), "NULL", "NULL");
      if(!(ret = sofar_sendModbus(sendHeartbeat, sizeof(sendHeartbeat), NULL, INVERTER_SLAVE_ID[i], true)))
      {
        g_sofar_last_heartbeat = true;
        g_sofar_heartbeat_ok++;
        errInRow[i] = 0;
        updateOLED("NULL", "Heart B."  + String(INVERTER_SLAVE_ID[i]), "NULL", "NULL");
        snprintf(szLine, sizeof(szLine)-1, "Info : Charger %s -> Online", String(INVERTER_SLAVE_ID[i]));
        DoLog(szLine);
      }
      else
      {
        g_sofar_last_heartbeat = false;
        g_sofar_heartbeat_err++;
        errInRow[i]++;
        updateOLED("NULL", "RS485->"  + String(INVERTER_SLAVE_ID[i]), "NULL", "NULL");
        if(errInRow[i] > 2)
        {
          snprintf(szLine, sizeof(szLine)-1, "Warning : Charger %s -> Offline (Heartbeats will continue to be sent...)", String(INVERTER_SLAVE_ID[i]));
          DoLog(szLine);
        } else {
          snprintf(szLine, sizeof(szLine)-1, "Warning : Can't contact charger %s -> , trying sending heartbeat again ... %s/2", String(INVERTER_SLAVE_ID[i]), String(errInRow[i]));
          DoLog(szLine);   
        }
      }
    }
  }
}

//calcCRC and checkCRC are based on...
//https://github.com/angeloc/simplemodbusng/blob/master/SimpleModbusMaster/SimpleModbusMaster.cpp

void calcCRC(uint8_t frame[], byte frameSize) 
{
  unsigned int temp = 0xffff, flag;

  for(unsigned char i = 0; i < frameSize - 2; i++)
  {
    temp = temp ^ frame[i];

    for(unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;

      if(flag)
        temp ^= 0xA001;
    }
  }

  // Bytes are reversed.
  frame[frameSize - 2] = temp & 0xff;
  frame[frameSize - 1] = temp >> 8;
}

bool checkCRC(uint8_t frame[], byte frameSize) 
{
  unsigned int calculated_crc, received_crc;

  received_crc = ((frame[frameSize-2] << 8) | frame[frameSize-1]);
  calcCRC(frame, frameSize);
  calculated_crc = ((frame[frameSize-2] << 8) | frame[frameSize-1]);
  return (received_crc = calculated_crc);
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

const char log_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>SmartSofar Logs</title>
  <meta name="viewport" content="max-width=device-width, width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    
    textarea {
      display: block;
      max-width: 800px;
      max-height: 700px;
      text-align: left;
      background-color: #cccccc;
      border: 1px solid #999999;
      padding: 10px 20px;
      border-radius: 5px;
    }
  </style>
</head>
<body>
<h3>SmartSofar Logs</h3>
<center>
<table>
  <tr>
    <th><input type="checkbox" id="stopScroll"/><label for="stopScroll">Stop Automatic Scroll</label></th>
    <th><button class="custom-button" type="button" onclick="window.location.href='/';">Go Back</button></th>
    </tr>
</table>
<br>
<textarea readonly id="logTextarea"></textarea>
</center>
<script>
var myLog = "";
var initForm = 0;
setInterval(function() 
{
  getLog();
}, 2000); 
function getLog() {
  var xhttp = new XMLHttpRequest();
  var logTextarea = document.getElementById("logTextarea");
  var stopScroll = document.getElementById("stopScroll");
  var screenWidth = screen.availWidth-90;
  var screenHeight = screen.availHeight-400;
  if (initForm==0){
    initForm=1;
    logTextarea.style.width=screenWidth+"px";
    logTextarea.style.height=screenHeight+"px";
  }
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var newLog = this.responseText;
      if(myLog!=newLog) {
        myLog=newLog;
        logTextarea.value += myLog;
        if(stopScroll.checked==false){logTextarea.scrollTop = logTextarea.scrollHeight;}
      }
    }
  };
  xhttp.open("GET", "logRefresh", true);
  xhttp.send();
}
</script>
</body>
</html>
)rawliteral";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>SmartSofar</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; border: 1px solid #999999; background-color: #ccc; border-radius: 6px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}

    .wp-table tr:nth-child(odd) {
      background-color: #fff;
    }
    .wp-table tr:nth-child(even) {
      background-color: #f1f1f1;
    }
    .wp-table tr {
      border-bottom: 1px solid #ddd;
    }
    .wp-table th:first-child, 
    .wp-table td:first-child {
      padding-left: 16px;
    }
    .wp-table td, 
    .wp-table th {
      padding: 8px 8px;
      display: table-cell;
      text-align: left;
      vertical-align: top;
    }
    .wp-table th {
      font-weight: bold;
    }
    .wp-table {
      font-size: 13px!important;
      border: 1px solid #ccc;
      border-collapse: collapse;
      border-spacing: 0;
      width: 300px;
      display: table;
    }
    input[type=range] {
      -webkit-appearance: none;
      background: #f1f1f1;
      width: 300px !important;
    }
    .custom-button {
      border: 1px solid #999999;
      background-color: #ccc;
      color: black;
      padding: 10px 20px;
      border-radius: 5px;
      cursor: pointer;
      width: 120px;
      height: 68px;
    }
    button:disabled,
    button[disabled]{
      border: 1px solid #999999;
      background-color: #cccccc;
      color: #666666;
    }
  </style>
</head>
<body>
  <h3>SmartSofar</h3>
  %BUTTONMODEPLACEHOLDER%
  <center>

  <div>
    <h4 id="titlePwAdjust">Power Adjustment</h4>
    %PWAMOUNTPLACEHOLDER%
    %SLIDERPLACEHOLDER%
    <br>
    <br>
    <table>
      <tr>
        <th>%BUTTONSTANDBYPLACEHOLDER%</th>
        <th><button class="custom-button" type="button" onclick="window.location.href='/log';">Logs</button></th>
        <th><button class="custom-button" type="button" onclick="window.location.href='/setup';">Setup</button></th>
      </tr>
    </table>
  </div>
  <h4>Data Collection</h4>
  <div id="myDataDiv">
  <table id="myDataTable" class="wp-table">
    <tr>
      <th>Item</th>
      <th>Parameter</th>
      <th>Value</th>
      <th>EngUnit</th>
    </tr>
    %DATAPLACEHOLDER%
  </table>
  </div>
  </center>
<script>
var inAction = 0;
var prevPwMessage = "";

function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  var titleMode = document.getElementById("titleMode");
  element.disabled=true;
  if(element.checked){ 
    xhr.open("GET", "/update?cmd="+element.id+"&val=1", true);
    disableSlider(1);
    titleMode.innerHTML = "Mode Auto";
  } else {
    xhr.open("GET", "/update?cmd="+element.id+"&val=0", true);
    disableSlider(0);
    titleMode.innerHTML = "Mode Manu";
  }
  xhr.send();
}
function updateSlider(element) {
  inAction=0;
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/update?cmd="+element.id+"&val="+element.value, true);
  xhr.send();
}
function disableSlider(value) {
  var sliderElement = document.getElementById("sliderPw");
  sliderElement.disabled = value;
  if(value==1){
    sliderElement.value = 0;
  }
  var buttonStandby = document.getElementById("buttonStandby");
  buttonStandby.disabled = value;
}
function updateSliderDiv(element) {
  var sliderDiv = document.getElementById("sliderAmount");
  var power = element.value;
  if(inAction==0){prevPwMessage = sliderDiv.innerHTML;}
  inAction=1;
  if (power>0){sliderDiv.innerHTML = "Set : Charge "+power+"W";}
  if (power<0){sliderDiv.innerHTML = "Set : Discharge "+Math.abs(power)+"W";}
  if (power==0){sliderDiv.innerHTML = "Set : Standby (0W)";}
  
}
function setStandby() {
  var sliderElement = document.getElementById("sliderPw");
  sliderElement.value = 0;
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/update?cmd="+sliderElement.id+"&val=0", true);
  xhr.send();
}

setInterval(function() 
{
  getData();
  updatePwDiv();
}, 1000); 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("myDataDiv").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "refresh", true);
  xhttp.send();
}
function updatePwDiv() {
  var modeAuto="-1";
  var power="";
  var table = document.getElementById("myDataTable");
  for (var r = 0, n = table.rows.length; r < n; r++) {
    for (var c = 0, m = table.rows[r].cells.length; c < m; c++) {
      if(table.rows[r].cells[c].innerHTML=="SetPoint" && table.rows[r].cells[c+1].innerHTML=="ModeAuto"){
        modeAuto = table.rows[r].cells[c+2].innerHTML;
      }
      if(table.rows[r].cells[c].innerHTML=="SetPoint" && table.rows[r].cells[c+1].innerHTML=="Power"){
        power = table.rows[r].cells[c+2].innerHTML;
      }
    }
    if(modeAuto!="-1" && power!="") {break;}
  }
  var sliderDiv = document.getElementById("sliderAmount");
  var power_s = "";
  if(power>0){power_s="Charging SetPoint : "+power+"W";}
  if(power<0){power_s="Discharging SetPoint : "+Math.abs(power)+"W";}
  if(power==0){
    if (modeAuto=="0"){
      power_s="Standby (0W)";
    } else {
      power_s="No Action (0W)";
    }
  }
  if(inAction==0 && prevPwMessage!=power_s){
    prevPwMessage=power_s;
    sliderDiv.innerHTML = power_s;
    var titleMode = document.getElementById("titleMode");
    var buttonElement = document.getElementById("buttonMode");
    var sliderElement = document.getElementById("sliderPw");
    var standbyElement = document.getElementById("buttonStandby");
    if(sliderElement.value!=power){sliderElement.value=power;}
    if (modeAuto=="0"){
      if(titleMode.innerHTML!="Mode Manu"){titleMode.innerHTML="Mode Manu";}
      if(buttonElement.checked==true){buttonElement.checked=false;}
      if(sliderElement.disabled==true){sliderElement.disabled=false;}
      if(standbyElement.disabled==true){standbyElement.disabled=false;}
    } else {
      if(titleMode.innerHTML!="Mode Auto"){titleMode.innerHTML="Mode Auto";}
      if(buttonElement.checked==false){buttonElement.checked=true;}
      if(sliderElement.disabled==false){sliderElement.disabled=true;}
      if(standbyElement.disabled==false){standbyElement.disabled=true;}
    }
    if(buttonElement.disabled==true){buttonElement.disabled=false;}
  }
}

</script>
</body>
</html>
)rawliteral";

// Replaces placeholder with button section in your web page
String processor(const String& var){
  if(var == "BUTTONMODEPLACEHOLDER"){
    String buttons = "";
    String isChecked = outputState(1);
    String typeMode = "";
    if (isChecked != "") {
      typeMode = "Mode Auto";
    } else {
      typeMode = "Mode Manu";
    }
    buttons += "<h4 id=\"titleMode\">"+typeMode+"</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"buttonMode\" " + isChecked + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  if(var == "DATAPLACEHOLDER"){
    String data = "";
    for(unsigned int l = 0; l < (sizeof(WebData) / sizeof(WebData[0])); l++) {
      if (WebData[l][0].length()>0) {
        data += "<tr><th>"+WebData[l][0]+"</th><th>"+WebData[l][1]+"</th><th>"+WebData[l][2]+"</th><th>"+WebData[l][3]+"</th></tr>";
      }
    }
    return data;
  }
  if(var == "PWAMOUNTPLACEHOLDER"){
    String power_s = WebData[1][2];
    int power = power_s.toInt();
    if(power>0){power_s="Charging SetPoint : "+String(power)+"W";}
    if(power<0){power_s="Discharging SetPoint : "+String(abs(power))+"W";}
    if(power==0){
      if(IsAuto==1){
        power_s="No Action (0W)";
      } else {
        power_s="Standby (0W)";
      }
    }
    return "<div id=\"sliderAmount\">"+String(power_s)+"</div>";
  }
  if(var == "SLIDERPLACEHOLDER"){
    int MaxSetPoint = MAX_POWER * INVERTER_COUNTS;
    String isDisabled = "";
    if (IsAuto==1) {
      isDisabled="disabled";
      pwManRequest=0;
    }
    return "-"+String(MaxSetPoint/1000)+"k<input type=\"range\" id=\"sliderPw\" min=\"-"+String(MaxSetPoint)+"\" max=\""+String(MaxSetPoint)+"\" step=\"50\" value=\""+WebData[1][2]+"\" oninput=\"updateSliderDiv(this)\" onchange=\"updateSlider(this)\" "+isDisabled+" />"+String(MaxSetPoint/1000)+"k"; //oninput=\"updateSliderDiv(this)\"
  }
  if(var == "BUTTONSTANDBYPLACEHOLDER"){
    String isDisabled = "";
    if (IsAuto==1) {
      isDisabled="disabled";
    }
    return "<div><button class=\"custom-button\" id=\"buttonStandby\" type=\"button\" "+isDisabled+" onclick=\"setStandby()\">Standby</button></div>";
  }
  return String();
}

String outputState(int output){
  if (EEPROM.read(output-1)==1) {
    IsAuto = 1;
    return "checked";
  } else {
    IsAuto = 0;
    return "";
  }
}

// Connect to WiFi
void setup_wifi()
{
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifiName);
  updateOLED("NULL", "NULL", "WiFi..", "NULL");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    updateOLED("NULL", "NULL", "WiFi...", "NULL");
  }

  WiFi.hostname(deviceName);
  const IPAddress& ipAddress = WiFi.localIP();
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(ipAddress);
  updateOLED("NULL", "Wifi", "Connected", "NULL");
  delay(1000);
  updateOLED("NULL", "IP="+String(ipAddress[0])+"."+String(ipAddress[1]), "."+String(ipAddress[2])+"."+String(ipAddress[3]), "NULL");
  delay(5000);
  updateOLED("NULL", "", "", "NULL");
}

// void onPublish(const MqttClient* /* source */, const Topic& topic, const char* payload, size_t /* length */) {
//   char szLine[128];
//   if(String(topic.c_str())==String(deviceName)+"/setmode") {
//     int val = String(payload).toInt();
//     if(val==0 || val==1) {
//       if (EEPROM.read(0)!=val) {
//         EEPROM.write(0, val);
//         EEPROM.commit();
//         if (val==1) {
//           pwManRequest = 0;
//           IsAuto=1;
//           WebData[1][2] = String(pwManRequest);
//           snprintf(szLine, sizeof(szLine)-1, "Info: MQTT -> Changed mode to 'auto'");
//         } else {
//           IsAuto=0;
//           snprintf(szLine, sizeof(szLine)-1, "Info: MQTT -> Changed mode to 'manu'");
//         }
//         mqtt_client.publish(std::string(deviceName)+"/SetPoint_ModeAuto", String(IsAuto));
//       }
//     } else {
//       snprintf(szLine, sizeof(szLine)-1, "Error: MQTT -> value must be 0(mode manu) or 1(mode auto)");
//     }
//     DoLog(szLine);
//   }
//   if(String(topic.c_str())==String(deviceName)+"/setpower") {
//     if(IsAuto==0) {
//       int MaxSetPoint = MAX_POWER * INVERTER_COUNTS;
//       int val = String(payload).toInt();
//       if(val>=(MaxSetPoint*-1) && val<=MaxSetPoint) {
//         pwManRequest = val;
//         WebData[1][2] = String(pwManRequest);
//         snprintf(szLine, sizeof(szLine)-1, "Info: MQTT -> set power to %sW", String(pwManRequest));
//         mqtt_client.publish(std::string(deviceName)+"/SetPoint_Power", String(pwManRequest));
//       } else {
//         snprintf(szLine, sizeof(szLine)-1, "Error: MQTT -> value must be set between -%sW and +%sW", String(MaxSetPoint), String(MaxSetPoint));
//       }
//     } else {
//       snprintf(szLine, sizeof(szLine)-1, "Error: MQTT -> cannot adjust power by MQTT command while mode is set to auto");
//     }
//     DoLog(szLine);
//   }
// }

void setup()
{

  Serial.begin(115200);

    //Turn on the OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED with the I2C addr 0x3C (for the 64x48)
  display.clearDisplay();
  display.display();
  updateOLED(deviceName, "Starting..", "", version);
  delay(1000);

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

    //Setup WIFI
  setup_wifi();

  //Put SD-CARD code here to read settings
  //Not yet implemented ...

  configTime(MY_TZ, MY_NTP_SERVER);

  WebData[0][0] = "EnergyMeter";
  WebData[0][1] = "Power";
  WebData[0][3] = "W";
  WebData[1][0] = "SetPoint";
  WebData[1][1] = "Power";
  WebData[1][2] = "0"; //added
  WebData[1][3] = "W";
  WebData[2][0] = "SetPoint";
  WebData[2][1] = "ModeAuto";
  WebData[2][3] = "";

  // broker.begin();
  // mqtt_client.setCallback(onPublish);
  // mqtt_client.subscribe(topicMode);
  // mqtt_client.subscribe(topicPower);

  int pos = 0;
  for(int i = 0; i < INVERTER_COUNTS; i++) {
    for(unsigned int l = 0; l < sizeof(status_reads)/sizeof(struct status_register); l++) {
      WebData[primaryInfos+pos][0] = "Charger"+String(i+1);
      WebData[primaryInfos+pos][1] = status_reads[l].name;
      WebData[primaryInfos+pos][3] = status_reads[l].EngUnit;
      pos+=1;
    }
  }

  EEPROM.begin(2);  //EEPROM.begin(Size)
  if (EEPROM.read(0)==1) {
    IsAuto = 1;
  } else {
    IsAuto = 0;
  }
  WebData[2][2]=String(IsAuto);
  //mqtt_client.publish(std::string(deviceName)+"/SetPoint_ModeAuto", String(IsAuto));
  
  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);

  RS485Serial.begin(9600);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", log_html, processor);
  });

  server.on("/logRefresh", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send(200, "text/plain", currentLog);
    memset(currentLog, 0, sizeof(currentLog)); //clear currentLog char array...
  }); 

  server.on("/refresh", HTTP_GET, [] (AsyncWebServerRequest *request) {
    int modeStatus = EEPROM.read(0);
    if(IsAuto!=modeStatus){
      IsAuto=modeStatus;
      WebData[2][2]=String(IsAuto);;
    }

    // mqtt_client.publish(std::string(deviceName)+"/SetPoint_ModeAuto", String(IsAuto));

    String data = "<table id=\"myDataTable\" class=\"wp-table\"><tr><th>Item</th><th>Parameter</th><th>Value</th><th>EngUnit</th></tr>";
    for(unsigned int l = 0; l < (sizeof(WebData) / sizeof(WebData[0])); l++) {
      if (WebData[l][0].length()>0) {
        data += "<tr><th>"+WebData[l][0]+"</th><th>"+WebData[l][1]+"</th><th>"+WebData[l][2]+"</th><th>"+WebData[l][3]+"</th></tr>";
      }
    }
    data += "</table>";
    request->send(200, "text/plain", data);
  });

  // Send a GET request to <ESP_IP>/update?cmd=<inputMessage1>&val=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage1;
    String inputMessage2;
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();

      if(inputMessage1=="buttonMode") { // Button Mode Auto (ID=1)
        if (EEPROM.read(0)!=inputMessage2.toInt()) {
          EEPROM.write(0, inputMessage2.toInt());
          EEPROM.commit();
          IsAuto = inputMessage2.toInt();
          WebData[2][2]=String(IsAuto);
          if (inputMessage2.toInt()==1) {
            pwManRequest = 0;
            WebData[1][2] = String(pwManRequest);
            //mqtt_client.publish(std::string(deviceName)+"/SetPoint_Power", String(pwManRequest));
          }
        }
      }

      if(inputMessage1=="sliderPw") { // Power Slider
        pwManRequest = inputMessage2.toInt();
        WebData[1][2] = String(pwManRequest);
        //mqtt_client.publish(std::string(deviceName)+"/SetPoint_Power", String(pwManRequest));
      }

      if(inputMessage1=="setmode") { // Set the Auto/Manu modes via http command
        if(inputMessage2.toInt()==0 || inputMessage2.toInt()==1) {
          if (EEPROM.read(0)!=inputMessage2.toInt()) {
            EEPROM.write(0, inputMessage2.toInt());
            EEPROM.commit();
            IsAuto = inputMessage2.toInt();
            WebData[2][2]=String(IsAuto);
            if (inputMessage2.toInt()==1) {
              pwManRequest = 0;
              WebData[1][2] = String(pwManRequest);
            }
          }
        } else {
            inputMessage2 = "Error: value must be 0(mode manu) or 1(mode auto)";
        }
      }

      if(inputMessage1=="setpower") { // Set the Power via http command
        if(IsAuto==0) {
          int MaxSetPoint = MAX_POWER * INVERTER_COUNTS;
          if(inputMessage2.toInt()>=(MaxSetPoint*-1) && inputMessage2.toInt()<=MaxSetPoint) {
            pwManRequest = inputMessage2.toInt();
            WebData[1][2] = String(pwManRequest);
          } else {
            inputMessage2 = "Error: value must be set between -"+String(MaxSetPoint)+"W and +"+String(MaxSetPoint)+"W";
          }
        } else {
          inputMessage2 = "Error: cannot adjust power by HTTP command while mode is set to auto";
        }
      }

    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    char szLine[128];
    snprintf(szLine, sizeof(szLine)-1, "Info : New HTTP command -> '%s' - Set to '%s'", inputMessage1.c_str(), inputMessage2.c_str());
    DoLog(szLine);
    request->send(200, "text/plain", "OK");
  });
    /////////////////////////////////////

    // Send a GET request to <IP>/get?message=<message>
    server.on("/data.json", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        message = "{";
        for(unsigned int l = 0; l < (sizeof(WebData) / sizeof(WebData[0])); l++) {
          if (WebData[l][0].length()>0) {
            if (message.length()>1) {message += ", ";};
            String EngUnit = WebData[l][3];
            if(EngUnit.indexOf("°")>-1){EngUnit="degC";}
            message += "\"" + WebData[l][0] + "_" + WebData[l][1] + "_Value\": \"" + WebData[l][2] + "\", \"" + WebData[l][0] + "_" + WebData[l][1] + "_EngUnit\": \"" + EngUnit + "\"";
          }
        }
        message += "}";
        request->send(200, "text/plain", message);
    });

    server.onNotFound(notFound);

    server.begin();

}

void loop()
{

  updateOLED(deviceName, "NULL", "NULL", "NULL");

  sofar_heartbeat();
  
  sofar_getData();

  if (getShellyData()) {
    if (IsAuto==1) {
      pwLastManRequest=-1;
      if (SetChargeDischargeAuto(pwAdjRequest)) {
        updateOLED("NULL", "Adjusting", "NULL", "NULL");
      } else {
        updateOLED("NULL", "Mode Auto", "NULL", "NULL");
      }
    }   
  }

  if (IsAuto==0) {
    if(pwLastManRequest!=pwManRequest){
      pwLastManRequest = pwManRequest;
      if (SetChargeDischargeManu(pwManRequest)) {
        updateOLED("NULL", "Adjusting", "NULL", "NULL");
      }
    } else {
      updateOLED("NULL", "Mode Manu", "NULL", "NULL");
    }
  }

  //broker.loop();
  //mqtt_client.loop();

  SysHealth();

  RefreshDisplay();

}

void SysHealth() {
  static unsigned long  lastRun = 0;
  if(checkTimer(&lastRun, SYSHEALTH_INTERVAL)) {
    char szLine[64] = "";
    snprintf(szLine, sizeof(szLine)-1, "Info : Uptime = %s, ESP Free Heap = %db", uptimeString.getUptime2(), ESP.getFreeHeap());
    DoLog(szLine);
  }
}

bool getShellyData()
{
  static unsigned long  lastRun = 0;
  if(checkTimer(&lastRun, SHELLY_INTERVAL))
  {
    char szLine[128];
    // wait for WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;

      int GoodChecks = 0; //count of good successive values in the range of the Delta;
      int pwPrevValue = -99999; //previous measured value;
      int averagePw = 0; //variable used to host the average of measured power reads;
      int CurrentCycle = 0; //current check cycle;

      while(CurrentCycle<=MaxCycles) {
        CurrentCycle+=1;
        if (http.begin(client, "http://" + String(SHELLY_IP) + "/rpc/Shelly.GetStatus")) {  // HTTP

          int httpCode = http.GET();
          String payload = http.getString();
          http.end(); //moved here

          if (httpCode > 0) {

            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {

              int total_act_power = getFromJson(payload,"total_act_power");
              averagePw += total_act_power;  

                if(abs(total_act_power-pwPrevValue)<=MaxDelta){
                  GoodChecks+=1;
                } else {
                  GoodChecks-=1;
                  if(GoodChecks<0){GoodChecks=0;}
                }
                pwPrevValue=total_act_power;

                if(GoodChecks==ChecksToReach || abs(total_act_power)>ImmediateAdjustW || IsAuto==0){
                  averagePw = averagePw / CurrentCycle;
                  WebData[0][2] = String(averagePw);
                  //mqtt_client.publish(std::string(deviceName)+"/EnergyMeter_Power", String(averagePw));
                  if(IsAuto==1) {
                    if(abs(averagePw)>ImmediateAdjustW){
                      snprintf(szLine, sizeof(szLine)-1, "Info : Energy Meter Power = %s W, immediate power adjustment (variation>%sw)", String(averagePw), String(ImmediateAdjustW));
                      DoLog(szLine);
                      } else {
                      snprintf(szLine, sizeof(szLine)-1, "Info : Energy Meter Power = %s W, check based on initial value + %s consecutive values under %s W between them. (Cycles=%s/%s)", String(averagePw), String(ChecksToReach), String(MaxDelta), String(CurrentCycle), String(MaxCycles));
                      DoLog(szLine);
                    }
                  } else {
                    snprintf(szLine, sizeof(szLine)-1, "Info : Energy Meter Power = %s W", String(averagePw));
                    DoLog(szLine);
                  }
                  if (averagePw == 0) {UpdateDisplayArray(0, "NULL", "NULL", "Balance", String(averagePw) + "W");}
                  if (averagePw > 0) {UpdateDisplayArray(0, "NULL", "NULL", "Importing", String(averagePw) + "W");}
                  if (averagePw < 0) {UpdateDisplayArray(0, "NULL", "NULL", "Exporting", String(abs(averagePw)) + "W");}
                  pwAdjRequest = averagePw;
                  return true;
                }
            }
          } else {
            snprintf(szLine, sizeof(szLine)-1, "Error : [HTTP] GET... failed, error: %s", http.errorToString(httpCode).c_str());
            DoLog(szLine);
            UpdateDisplayArray(0, "NULL", "NULL", "Shelly", "errHTTP");
          }
        } else { //Unable to connect to Shelly device
          snprintf(szLine, sizeof(szLine)-1, "Warning : [HTTP] Unable to connect Shelly device!");
          DoLog(szLine);
        }
        delay(DelayBetweenReads);
      }
      averagePw = averagePw / CurrentCycle;
      if (averagePw == 0) {UpdateDisplayArray(0, "NULL", "NULL", "Balance", String(averagePw) + "W");}
      if (averagePw > 0) {UpdateDisplayArray(0, "NULL", "NULL", "Importing", String(averagePw) + "W");}
      if (averagePw < 0) {UpdateDisplayArray(0, "NULL", "NULL", "Exporting", String(abs(averagePw)) + "W");}
      WebData[0][2] = String(averagePw);
      //mqtt_client.publish(std::string(deviceName)+"/EnergyMeter_Power", String(averagePw));
      snprintf(szLine, sizeof(szLine)-1, "Warning : Shelly measures max cycles reached!");
      DoLog(szLine);
      snprintf(szLine, sizeof(szLine)-1, "Info : Energy Meter Power = %s W, average calculated from last %s measures", String(averagePw), String(MaxCycles));
      DoLog(szLine);
      pwAdjRequest = averagePw;
      return true;
    } else { //no wifi, exit
      UpdateDisplayArray(0, "NULL", "NULL", "Shelly", "NoWifi");
    }
    return false;
  } else {
    return false;
  }
}

bool SetChargeDischargeAuto(int meterValue) {

    char szLine[64];

    if (abs(meterValue) < Delta) {return false;}

    pwAdjRequest = 0;

    String cmd, fnType;
    uint16_t fnCode = 0, fnParam = 0;
    int i, index;

    int resetCounter = 0;
    for(i = 0; i < INVERTER_COUNTS; i++) {
      if (INVERTER_RUNNINGSTATE[i] == 0) {resetCounter += 1;}
    }
    if (resetCounter == INVERTER_COUNTS) { //&& meterValue<0
      SetPoint = 0;
      snprintf(szLine, sizeof(szLine)-1, "Warning : (Auto) Charger(s) returns 'waiting' or 'bad' running state!");
      DoLog(szLine);
    }

    SetPoint = SetPoint - meterValue;
    int MaxSetPoint = MAX_POWER * INVERTER_COUNTS;
    if (SetPoint > 0) {
      if (abs(SetPoint) > MaxSetPoint) {SetPoint = MaxSetPoint;}
      fnCode = SOFAR_FN_CHARGE;
      fnType = "charge";
    } else {
      if (abs(SetPoint) > MaxSetPoint) {SetPoint = -MaxSetPoint;}
      fnCode = SOFAR_FN_DISCHARGE;
      fnType = "discharge";
    }
    WebData[1][2] = String(SetPoint);
    fnParam = abs(SetPoint) / INVERTER_COUNTS;
  
    if(fnCode) {
      for(i = 0; i < INVERTER_COUNTS; i++) {
        index = i + 1;
        if (fnCode == SOFAR_FN_CHARGE) {
          cmd == "charge_" + String(index);
        } else {
          cmd == "discharge_" + String(index);
        }
        snprintf(szLine, sizeof(szLine)-1, "Info : (Auto) Sending %s to inverter %s : %s W", fnType, String(INVERTER_SLAVE_ID[i]), String(fnParam));
        DoLog(szLine);
        delay(300);
        sofar_sendPassiveCmd(INVERTER_SLAVE_ID[i], fnCode, fnParam, cmd);
        //delay(300);
      }
    delay(2000);
    return true;
    }
  return false;
}

bool SetChargeDischargeManu(int NewSetPoint) {
  char szLine[64];
  String cmd, fnType;
  uint16_t fnCode = 0, fnParam = 0;
  int i, index;
  int MaxSetPoint = MAX_POWER * INVERTER_COUNTS;
  if (NewSetPoint > 0) {
    if (abs(NewSetPoint) > MaxSetPoint) {NewSetPoint = MaxSetPoint;}
    fnCode = SOFAR_FN_CHARGE;
    fnType = "charge";
    fnParam = abs(NewSetPoint) / INVERTER_COUNTS;
  }
  if (NewSetPoint < 0) {
    if (abs(NewSetPoint) > MaxSetPoint) {NewSetPoint = -MaxSetPoint;}
    fnCode = SOFAR_FN_DISCHARGE;
    fnType = "discharge";
    fnParam = abs(NewSetPoint) / INVERTER_COUNTS;
  }
  if (NewSetPoint==0) {
    fnCode = SOFAR_FN_STANDBY;
    fnType = "standby";
    fnParam = SOFAR_PARAM_STANDBY;
  }
  WebData[1][2] = String(NewSetPoint);

  if(fnCode) {
    for(i = 0; i < INVERTER_COUNTS; i++) {
      index = i + 1;
      if (fnCode == SOFAR_FN_CHARGE) {
        cmd == "charge_" + String(index);
        snprintf(szLine, sizeof(szLine)-1, "Info : (Manu) Sending %s to inverter %s : %s W", fnType, String(INVERTER_SLAVE_ID[i]), String(fnParam));
        DoLog(szLine);
      }
      if (fnCode == SOFAR_FN_DISCHARGE) {
        cmd == "discharge_" + String(index);
        snprintf(szLine, sizeof(szLine)-1, "Info : (Manu) Sending %s to inverter %s : %s W", fnType, String(INVERTER_SLAVE_ID[i]), String(fnParam));
        DoLog(szLine);
      }
      if (fnCode == SOFAR_FN_STANDBY) {
        cmd == "standby_" + String(index);
        snprintf(szLine, sizeof(szLine)-1, "Info : (Manu) Sending %s to inverter %s", fnType, String(INVERTER_SLAVE_ID[i]));
        DoLog(szLine);
      }
      delay(300);
      sofar_sendPassiveCmd(INVERTER_SLAVE_ID[i], fnCode, fnParam, cmd);
      //delay(300);
    }
    delay(2000);
    return true;
  }
  return false;
}

/**
 * Check to see if the elapsed interval has passed since the passed in
 * millis() value. If it has, return true and update the lastRun. Note
 * that millis() overflows after 50 days, so we need to deal with that
 * too... in our case we just zero the last run, which means the timer
 * could be shorter but it's not critical... not worth the extra effort
 * of doing it properly for once in 50 days.
 */
bool checkTimer(unsigned long *lastRun, unsigned long interval)
{
  unsigned long now = millis();

  if(*lastRun > now)
    *lastRun = 0;

  if(now >= *lastRun + interval)
  {
    *lastRun = now;
    return true;
  }

  return false;
}

bool read_batch_of_registers(modbusResponse *rs, uint8_t regCount, uint16_t startReg, uint8_t SLAVE_ID)
{
  uint8_t frame[] = { SLAVE_ID, MODBUS_FN_READSINGLEREG, (uint8_t)(startReg >> 8), (uint8_t)(startReg & 0xff), 0, regCount, 0, 0 };

  int ret = sofar_sendModbus(frame, sizeof(frame), rs, SLAVE_ID);
  if(ret == 0)
  {
    g_sofar_req_read_ok++;
    return true;
  }
  else
  {
    sofar_flushRS485();
    g_sofar_req_read_err++;    
    return false;
  }
}

bool refresh_registers(uint16_t start_reg, uint8_t  reg_count, uint8_t SLAVE_ID)
{
  char szLine[64];
  modbusResponse rs;
  if(read_batch_of_registers(&rs, reg_count, start_reg, SLAVE_ID))
  {
    if(reg_count*2 != rs.dataSize)
    {
      snprintf(szLine, sizeof(szLine)-1, "Error : Size read error on charger %s : %s, %s", String(SLAVE_ID), String(reg_count*2), String(rs.dataSize));
      DoLog(szLine);
      return false;
    }
    
    for(int i = 0; i < rs.dataSize; i+=2) 
    {
      all_regs[reg_to_data_index(start_reg)+(i/2)] = (rs.data[i] << 8) | rs.data[i+1];
    }
  }
  else
  {
    g_regReadOk = false; //we set true only if all are OK, false on any bad read
    return false;
  }
  
  return true;
}

int getFromJson(String input, String search) {
  String readState;
  readState = midString(input,search,",");
  readState.replace('"',' ');
  readState.replace(':',' ');
  readState.trim();
  return readState.toInt();
}

String midString(String str, String start, String finish){
  int locStart = str.indexOf(start);
  if (locStart==-1) return "";
  locStart += start.length();
  int locFinish = str.indexOf(finish, locStart);
  if (locFinish==-1) return "";
  return str.substring(locStart, locFinish);
}

void UpdateDisplayArray(int row, String col1, String col2, String col3, String col4) {
  DisplayData[row][0]=col1;
  DisplayData[row][1]=col2;
  DisplayData[row][2]=col3;
  DisplayData[row][3]=col4;
}

void updateRunstate(int index)
{
  switch(INVERTER_RUNNINGSTATE[index])
  {
    case waiting:
        UpdateDisplayArray(index+1, "NULL", "NULL", "Standby " + String(INVERTER_SLAVE_ID[index]), "");
      break;

      case check:
        UpdateDisplayArray(index+1,"NULL", "NULL", "Checking " + String(INVERTER_SLAVE_ID[index]), "NULL");
      break;

      case charging:
        UpdateDisplayArray(index+1, "NULL", "NULL", "Charge " + String(INVERTER_SLAVE_ID[index]), String(BATTERY_POWER[index])+"W"); //String(batteryWatts(index))
      break;

#ifdef INVERTER_ME3000
      case checkDischarge:
        UpdateDisplayArray(index+1, "NULL", "NULL", "Check Dis " + String(INVERTER_SLAVE_ID[index]), "NULL");
        break;
#endif
      case discharging:
        UpdateDisplayArray(index+1, "NULL", "NULL", "Dischar. " + String(INVERTER_SLAVE_ID[index]), String(BATTERY_POWER[index])+"W"); //String(batteryWatts(index))
        break;

      case epsState:
        UpdateDisplayArray(index+1, "NULL", "NULL", "EPS State " + String(INVERTER_SLAVE_ID[index]), "NULL");
      break;

      case faultState:
        UpdateDisplayArray(index+1, "NULL", "NULL", "FAULT " + String(INVERTER_SLAVE_ID[index]), "NULL");
      break;

      case permanentFaultState:
        UpdateDisplayArray(index+1, "NULL", "NULL", "PERMFAULT " + String(INVERTER_SLAVE_ID[index]), "NULL");
      break;

      default:
        UpdateDisplayArray(index+1, "NULL", "NULL", "Runstate? " + String(INVERTER_SLAVE_ID[index]), "NULL");
      break;
  }
}

void DoLog(const char* msg) {
  time(&now); // read the current time
  localtime_r(&now, &tm); // update the structure tm with the current time
  char logLine[500];
  snprintf(logLine, sizeof(logLine)-1, "\n%04s-%02s-%02s %02s:%02s:%02s : %s", String(tm.tm_year + 1900), String(tm.tm_mon + 1), String(tm.tm_mday), String(tm.tm_hour), String(tm.tm_min), String(tm.tm_sec), msg);
  if (strlen(logLine) + strlen(currentLog) >= sizeof(currentLog)) memset(currentLog, 0, sizeof(currentLog));
  strcat(currentLog,logLine);
  //Serial.println(currentLog);
  Serial.println(logLine);
}