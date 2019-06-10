//-------------------------------------------------------------------------------
// Tiny Circuits Thomas the Tank Engine Eye Mechanism Transmitter.
// Referenced and derived from TinyCircuits TinyScreen with RF Radio Example sketch.
// Author: Jacob Henry - Twitter.com@TDoc_41
// Author: Ben Rose - TinyCircuits
//-------------------------------------------------------------------------------

#include <SPI.h>
#include "RH_RF22.h"

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/* Channel configuration
 *  
 *  Important note on radio channels via https://en.wikipedia.org/wiki/LPD433: 
 *  
 *  Radio control enthusiasts were able to use frequencies from channel 03 through 67 
 *  on the above chart for radio control of any form of model (air or ground-based), all with 
 *  odd channel numbers (03, 05, etc. up to ch. 67) as read on the chart,[8] with each sanctioned 
 *  frequency having 50 kHz of bandwidth separation between each adjacent channel. 
 */
#define CHANNEL_COUNT 69
#define AFC_BANDWIDTH 0.05f /* khz */
#define CHANNEL_SELECT 3

RH_RF22 rf22(7,3);
float channels[CHANNEL_COUNT];

/* Pin configuration */
const int joyClick = 10;      // joystick click pin
const int xAxis  = 1;         // joystick X axis to A1 
const int yAxis  = 0;         // joystick Y axis to A0

void configureChannels()
{
  float freq = 433.075f;
  for(int i = 0; i < CHANNEL_COUNT; i++)
  {
    channels[i] = freq;
    freq += 0.025f;
  }
}

void setup() 
{
  SerialMonitorInterface.begin(115200);
//  while(!SerialMonitorInterface)
//    delay(100);
 
  if (!rf22.init())
    SerialMonitorInterface.println("init failed"); 
  configureChannels();
  rf22.setTxPower(RH_RF22_TXPOW_20DBM);
  rf22.setModemConfig(RH_RF22::GFSK_Rb125Fd125);  //FSK_Rb2_4Fd36
  rf22.setFrequency(channels[CHANNEL_SELECT - 1], AFC_BANDWIDTH);

  SerialMonitorInterface.print("Channel ");
  SerialMonitorInterface.print(CHANNEL_SELECT);
  SerialMonitorInterface.print(" [Freq: ");
  SerialMonitorInterface.print(channels[CHANNEL_SELECT - 1], 3);
  SerialMonitorInterface.print(" MHz, Bandwidth: ");
  SerialMonitorInterface.print(AFC_BANDWIDTH, 3);
  SerialMonitorInterface.println(" KHz]");

  pinMode(joyClick, INPUT_PULLUP);   // joystick click pin  
}

int max = 2540;
int min = 510;
int servoRange = max - min;
int mid = 1450;
double msPerDegree = servoRange/180.0;
double servoSpeedRating = 0.09/(msPerDegree * 60); //unused
int maxTravelDegrees = 60; //The max degrees the servos can travel. Must be <= 180.
int degLimitMS = msPerDegree * maxTravelDegrees;

int prevXReading = -1;
int prevYReading = -1;
void loop()
{ 
  int xReading = readAxis(xAxis);
  int yReading = readAxis(yAxis);

  if(prevXReading != xReading || prevYReading != yReading)
  {
    uint8_t data[8];
    data[0] = (xReading >> 24) & 0xFF; //0
    data[1] = (xReading >> 16) & 0xFF; //0
    data[2] = (xReading >> 8) & 0xFF;  //12
    data[3] = xReading & 0xFF;
    data[4] = (yReading >> 24) & 0xFF; //0
    data[5] = (yReading >> 16) & 0xFF; //0
    data[6] = (yReading >> 8) & 0xFF;  //12
    data[7] = yReading & 0xFF;
    rf22.send(data, 8); //send control data packet and wait
    rf22.waitPacketSent();  

    prevXReading = xReading;
    prevYReading = yReading;
  }
}

// Reads a joystick axis (0 or 1 for x or y) and scales the 
//  analog input range to a range from 0 to <range>
int readAxis(int thisAxis) 
{  
  // map the reading from the analog input range to the output range
  return map(analogRead(thisAxis), 0, 1023, mid - (degLimitMS / 2), mid + (degLimitMS / 2));
}
