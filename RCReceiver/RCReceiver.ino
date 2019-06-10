//-------------------------------------------------------------------------------
// Tiny Circuits Thomas the Tank Engine Eye Mechanism Receiver.
// Referenced and derived from TinyCircuits TinyScreen with RF Radio Example sketch.
// Author: Jacob Henry - Twitter.com@TDoc_41
// Author: Ben Rose - TinyCircuits
//-------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include "RH_RF22.h"
#include <ServoDriver.h>

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

/* Servo configuration */
ServoDriver servoDriver(NO_R_REMOVED);//this value affects the I2C address, which can be changed by
                                //removing resistors R1-R3. Then the corresponding R1_REMOVED,
                                //R2_REMOVED, R1_R2_REMOVED, R1_R4_REMOVED and so on can be set.
                                //Default is NO_R_REMOVED

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
  rf22.setModemConfig(RH_RF22::GFSK_Rb125Fd125);
  rf22.setFrequency(channels[CHANNEL_SELECT - 1], AFC_BANDWIDTH);
  
  SerialMonitorInterface.print("Channel ");
  SerialMonitorInterface.print(CHANNEL_SELECT);
  SerialMonitorInterface.print(" [Freq: ");
  SerialMonitorInterface.print(channels[CHANNEL_SELECT - 1], 3);
  SerialMonitorInterface.print(" MHz, Bandwidth: ");
  SerialMonitorInterface.print(AFC_BANDWIDTH, 3);
  SerialMonitorInterface.println(" KHz]");
  
  loadServoDrivers();
}

void loop()
{
  if (rf22.available())
  {
    uint8_t buf[8];
    uint8_t len = sizeof(buf);
    if (rf22.recv(buf, &len))
    {
      uint32_t xval = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
      servoDriver.setServo(3, xval);
      uint32_t yval = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7];
      servoDriver.setServo(4, yval);
    }
  }
}

/* This method loads the drivers for the servo controller shield for communication with up to 4 servo motors */
void loadServoDrivers()
{
  Wire.begin();
  pinMode(9, OUTPUT); //Pin 9 is the reset pin for the servo controller TinyShield
  
  // THE FOLLOWING 2 LINES MUST BE COMMENTED OUT FOR COMPATIBILITY WITH BLUETOOTH SHIELD
  // BECAUSE DROPPING THE POWER LOW ON PIN 9 RESETS THE BLUETOOTH AND IT WILL NOT RECOVER
  // digitalWrite(9, LOW);
  // delay(10);
  
  digitalWrite(9, HIGH);
  delay(100);
  
  //Set the period to 20000us or 20ms, correct for driving most servos
  if(servoDriver.begin(20000) != 0)
  {
    SerialMonitorInterface.println("Motor driver not detected!");
    while(1);
  }
  
  //The failsafe turns off the PWM output if a command is not sent in a certain amount of time.
  //Failsafe is set in milliseconds- comment or set to 0 to disable
  servoDriver.setFailsafe(1000);
}

void moveAllServos(int to)
{
  servoDriver.setServo(1, to);
  servoDriver.setServo(2, to);
  servoDriver.setServo(3, to);
  servoDriver.setServo(4, to);
}
