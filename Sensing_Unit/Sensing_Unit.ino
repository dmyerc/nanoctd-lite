// NANOCTD AANDERA STREAMING CODE FOR SUMMER 2026
#include <Wire.h>
#include "MS5837.h"
#include "KellerLD.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>
#include <RTClib.h>
#include <Adafruit_AD569x.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TSYS01.h>
#include <SerialTransfer.h>
#include <math.h>
#include "dataPacket.h"
#include <SoftwareSerial.h>

////////// General definitions
#define BAUD_RATE 115200
#define UART_ENABLE 1
#define IA_SCALE_OUTPUT 1      // 0 = ADA 4530 Electrometer, 1 = Instrument Amplifier v1

////////// RTD definitions
#define RREF 430.0
#define RNOMINAL 100.0

////////// Board Selection
#define ELECTROMETER_BOARD_01 1  // Arduino Mega 16 Relay Hack Board (John's Board)
#define ELECTROMETER_BOARD_02 2  // Arduino Mega 8 Relay Hack Board (Victoria/Luke Board)
#define ELECTROMETER_BOARD_03 3  // nanoCTD board version 1 using Arduino R3/R4
#define ELECTROMETER_BOARD_04 4  // nanoCTD board version 1 using Nucleo board

#define RTC_ENABLE 1
#define CLOSE_RELAY HIGH
#define OPEN_RELAY LOW
#define NUM_RELAYS 9
// -1 indicates a disabled relay. This allows for indeces to be associated with a phyiscal location such as the nCTD board
const int relayPins[NUM_RELAYS] = { 2, 3, 4, 5, 6, 7, 14, 15, 16 };
int16_t shuntRelays[] = { 9 };  // this is the 1-based index into relay pins
#define NUM_RTDS 1
#define THERMO1_CS 9
#define THERMO2_CS 10
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(THERMO2_CS, 11, 12, 13);
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(THERMO1_CS, 11, 12, 13);
#define ONE_WIRE_BUS 0

////////// RS-485 setup, see included dataPacket.h file for structure

SerialTransfer myTransfer;
dataPacket transmitted_Packet;
uint16_t packet_Counter = 0;

#endif

////////// AANDERAA Setup
#define AANDERAA_TX 8
#define AANDERAA_RX A3
SoftwareSerial aanderaaSerial (AANDERAA_RX, AANDERAA_TX);

////////// BME280 Pressure,Humidity and Temperature
Adafruit_BME280 bme;

////////// AD5693 DAC
Adafruit_AD569x dac;

////////// Blue Robotics Bar30 and Bar100 pressure sensors
MS5837 bar30;
KellerLD bar100;

////////// Blue Robotics temperature sensor
TSYS01 BRTempSensor;

////////// ADS1115 Digitizer
Adafruit_ADS1115 ads1115;
adsGain_t adsGainSettings[] = { GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN };
float adsGainRanges[] = { 4.096 * 3 / 2, 4.096, 2.048, 1.024, 0.512, 0.256 };
int32_t adsCurrentGainSetting = 1;


#if RTC_ENABLE != 0
RTC_DS3231 rtc;
#endif

////////// Variables needed to filter 60 Hz noise using least squares regressiog
const int32_t LS_FILTER_MAX_SAMPLES = 100;
float voltageScaled[LS_FILTER_MAX_SAMPLES];
float meanVoltage;
int32_t readCount = 25;
float X1[LS_FILTER_MAX_SAMPLES];
float X2[LS_FILTER_MAX_SAMPLES];
float XX[9];
float XXInverse[9];
float XXDeterminant;
float angularFrequency = 2 * PI * 60.0;

char stringToWrite[100];  // Character array used by serial port, SD Card, etc.

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(100000);
  for (int32_t loop1 = 0; loop1 < NUM_RELAYS; loop1++) {
    if (relayPins[loop1] >= 0) {
      pinMode(relayPins[loop1], OUTPUT);
      digitalWrite(relayPins[loop1], OPEN_RELAY);
    }
  }

  digitalWrite(relayPins[0], CLOSE_RELAY);
  digitalWrite(relayPins[4], CLOSE_RELAY);

  SPI.begin();
  
  ads1115.setGain(adsGainSettings[adsCurrentGainSetting]);
  ads1115.setDataRate(RATE_ADS1115_250SPS);
  ads1115.begin();

  //thermo2.begin(MAX31865_4WIRE);
  thermo1.begin(MAX31865_4WIRE);

  //sensors.begin();
  BRTempSensor.init();

  bme.begin();

  dac.begin(0x4C, &Wire);
  dac.reset();
  dac.setMode(NORMAL_MODE, true, false);

  bar30.init();
  bar30.setFluidDensity(1029);
  bar100.init();
  bar100.setFluidDensity(1029);

  Serial1.begin(BAUD_RATE, SERIAL_8N1);
  myTransfer.begin(Serial1);;
  aanderaaSerial.begin(9600);

#if RTC_ENABLE != 0
  rtc.begin();

  // Set RTC to upload time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  DateTime now = rtc.now();
  Serial.print("RTC set to: ");
  Serial.print(now.year());
  Serial.print("-");
  Serial.print(now.month());
  Serial.print("-");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());

#endif


char timeBuffer[50];


void loop() {
  // Serial.println("Made it to loop");
  digitalWrite(relayPins[0], CLOSE_RELAY);
  digitalWrite(relayPins[4], CLOSE_RELAY);

  transmitted_Packet = {};
  transmitted_Packet.packetID = packet_Counter;

  ReadBar30Stream(&transmitted_Packet);
  ReadTempBRStream(&transmitted_Packet);
  ReadVoltageStream(&transmitted_Packet, 0);
  StreamAanderaa(&transmitted_Packet);

  digitalWrite(relayPins[0], OPEN_RELAY);
  digitalWrite(relayPins[4], OPEN_RELAY);
  digitalWrite(relayPins[1], CLOSE_RELAY);
  digitalWrite(relayPins[5], CLOSE_RELAY);
  ReadVoltageStream(&transmitted_Packet, 1);

  
  digitalWrite(relayPins[1], OPEN_RELAY);
  digitalWrite(relayPins[5], OPEN_RELAY);
  digitalWrite(relayPins[2], CLOSE_RELAY);
  digitalWrite(relayPins[6], CLOSE_RELAY);
  ReadVoltageStream(&transmitted_Packet, 2);
  digitalWrite(relayPins[2], OPEN_RELAY);
  digitalWrite(relayPins[6], OPEN_RELAY);


  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(transmitted_Packet, sendSize);
  myTransfer.sendData(sendSize);

  packet_Counter++;

//  Serial.print("Packet ");
//  Serial.println(transmitted_Packet.packetID);
//  Serial.print("AANDERA (Conductivity, Temp, Salinity): (");
//  Serial.print(transmitted_Packet.aanderaa_cond);
//  Serial.print(",");
//  Serial.print(transmitted_Packet.aanderaa_temp);
//  Serial.print(",");
//  Serial.print(transmitted_Packet.aanderaa_sal);
//  Serial.println(")");

//  Serial.print(" Voltage1: ");
//  Serial.print(transmitted_Packet.voltage_mV[0]);
//  Serial.print(" Voltage2: ");
//  Serial.print(transmitted_Packet.voltage_mV[1]);
//  Serial.print(" Voltage3: ");
//  Serial.print(transmitted_Packet.voltage_mV[2]);
//  Serial.print("     Temp: ");
//  Serial.print(transmitted_Packet.temp_C);
//  Serial.print(" Pressure: ");
//  Serial.println(transmitted_Packet.pressure_hPa);
  // Serial.print(": ");
}

void ReadTempBRStream(dataPacket * transmitted_Packet){
  BRTempSensor.read();
  float temp_C = BRTempSensor.temperature();
  // Serial.print("BR temp is: ");
  // Serial.print(temp_C);
  transmitted_Packet->temp_C = temp_C;
}

void ReadVoltageStream(dataPacket * transmitted_Packet, int index){
  int32_t readCount = 10;
  int32_t readingsToDiscard = 2;
  int16_t voltage = 0;
  int32_t voltageSummed = 0;
  float voltageScaled = 0.0;

  for (int loop1 = 0; loop1 < readCount; loop1++) {
    voltage = ads1115.readADC_Differential_0_1();
    // Serial.println(String(micros()/1000000.0,6) + ", " + String(voltage));
    if (loop1 >= readingsToDiscard) {
      voltageSummed += voltage;
    }
  }

  voltageScaled = voltageSummed * adsGainRanges[adsCurrentGainSetting] / 32768.0 * 1000.0 / (readCount - readingsToDiscard);
  if (  IA_SCALE_OUTPUT == 1) {
    voltageScaled = (voltageScaled - 1500.0) / 5.0;
  }
  transmitted_Packet->voltage_mV[index] = voltageScaled;
}

void ReadBar30Stream(dataPacket * transmitted_Packet) {
  float pressure_hPa = 0.0;
  bar30.read();
  pressure_hPa = bar30.pressure();
  transmitted_Packet->pressure_hPa = pressure_hPa;
}


void ReadRTDStream(dataPacket * transmitted_Packet) {
  int16_t readCount = 1;
  int16_t rtdNumber = 1;


  float temperature = 0.0;
  //for (int loop1 = 0; loop1 < readCount; loop1++) {
      thermo1.begin(MAX31865_4WIRE);
      temperature += thermo1.temperature(RNOMINAL, RREF) / readCount;
  float rtd_C = thermo1.temperature(RNOMINAL, RREF);
  //}
  // Serial.print("RTD temperature reading is: ");
  // Serial.println(temperature);

  // transmitted_Packet->rtd_C = rtd_C;
}

void StreamAanderaa (dataPacket * transmitted_Packet) {
  aanderaaSerial.begin(9600);
  aanderaaSerial.write("\r\n");

  delay(200);

  while (aanderaaSerial.available()) {
    aanderaaSerial.read();
  }
  aanderaaSerial.write("Do sample\r\n");

  unsigned long startTime = millis();
  String line = "";

  while (aanderaaSerial.available() <= 0 && millis() - startTime  < 2000) {
    delay(50);
  }

  const int bufferSize = 180;
  char lineBuffer[bufferSize];
  int index = 0;

  if (aanderaaSerial.available() > 0) {
    while (millis() - startTime < 3000) {
      if (aanderaaSerial.available()) {
        char c = aanderaaSerial.read();
        if (c == '\n' || index >= bufferSize - 1) {
          lineBuffer[index] = '\0';
          if (strstr(lineBuffer, "MEASUREMENT") != NULL) {
            line = String(lineBuffer);
            break;
          }
          index = 0;
        } else if (c != '\r') {
          lineBuffer[index++] = c;
        }
        if (c == '#') break;
      }
    }
  }
  float cond = 0.0, temp = 0.0, sal = 0.0;

  // Parse fields
  int start, end;
  int idx = line.indexOf("Conductivity");
  if (idx >= 0) {
    start = line.indexOf('\t', idx) + 1;
    end = line.indexOf('\t', start);
    cond = line.substring(start, end).toFloat();
  }

  idx = line.indexOf("Temperature", end);
  if (idx >= 0) {
    start = line.indexOf('\t', idx) + 1;
    end = line.indexOf('\t', start);
    temp = line.substring(start, end).toFloat();
  }

  idx = line.indexOf("Salinity", end);
  if (idx >= 0) {
    start = line.indexOf('\t', idx) + 1;
    end = line.indexOf('\t', start);
    sal = line.substring(start, end).toFloat();
  }

  transmitted_Packet->aanderaa_cond = cond;
  transmitted_Packet->aanderaa_temp = temp;
  transmitted_Packet->aanderaa_sal = sal;

  aanderaaSerial.end();

}
