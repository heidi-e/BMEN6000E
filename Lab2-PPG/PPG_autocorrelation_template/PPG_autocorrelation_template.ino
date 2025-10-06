#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "arduinoFFT.h"
#include <SPI.h>
#include <SD.h>

MAX30105 PPGSensor;

#define I2C_SPEED_FAST 400000
#define MAX_BRIGHTNESS 255

const int chipSelect = 4;
File pulseRecord;

// sensor input/output variables
int i;
uint32_t irBuffer[256]; //infrared LED sensor data
uint32_t redBuffer[256];  //red LED sensor data
byte readLED = 13; //Blinks with each data read
byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

// variables for pre-existing maxim SpO2/HR calculations
int32_t bufferLength = 256; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// Variables to calculate HR
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
float sum;
float irDC;
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
double peak;
float peakloc;
float currIR;
float diffIR[256];
float myHR;
const int buffsize = ;
float HRBuff[buffsize];
float sumHR;
float myavgHR;
int j;

void setup() {

  Serial.begin(115200);
  Serial.println("Initializing...");

//  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  SD.begin(chipSelect);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD not found ");
    return;
  }
  Serial.println("SD found");
  File pulseRecord = SD.open("HRLog.txt", FILE_WRITE);
  if (pulseRecord){
  pulseRecord.println("SpO2 (library)  SpO2 (my calcs)  PR (library)  PR (my calcs)  HRV");
  pulseRecord.close();
  }
  else {
    Serial.println("error opening HRLog.txt");
    return;
  }

  // Initialize sensor
  if (!PPGSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }

  PPGSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  PPGSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  PPGSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {
  
  //read the first 100 samples, and determine the signal range
  for (i = 0 ; i < bufferLength ; i++)
  {
    while (PPGSensor.available() == false) //do we have new data?
      PPGSensor.check(); //Check the sensor for new data

    redBuffer[i] = PPGSensor.getRed();
    irBuffer[i] = PPGSensor.getIR();
    PPGSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (int i = 64; i < 256; i++)
    {
      redBuffer[i - 64] = redBuffer[i];
      irBuffer[i - 64] = irBuffer[i];
      diffIR[i - 64] = diffIR[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (int i = 192; i < 256; i++)
    {
      while (PPGSensor.available() == false) //do we have new data?
      
      PPGSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = PPGSensor.getRed();
      irBuffer[i] = PPGSensor.getIR();
      float prevIR = ;
      currIR =;
      diffIR[i] = ;

      PPGSensor.nextSample(); //We're finished with this sample so move to next sample
    }
  
  // Calculate HR and SpO2 using existing library
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  // Remove DC component (2 pts)

  // Calculate autocorrelation (4 pts)

  // Find index of maximum value (3 pts)

  // Convert index of max value to HR (4 pts)

  // Calculate moving average HR (1 pt)
 

    // Serial.print(spo2, DEC);
    // Serial.print("\t");
    // Serial.print(mySpO2, 0);
    // Serial.print("\t");
    Serial.print(heartRate, DEC);
    Serial.print("\t");
    Serial.print(myHR, 0);
    Serial.println("\t");
    // Serial.print(HRV, 0);
    // Serial.println("\t");

      // pulseRecord = SD.open("HRLog.txt", FILE_WRITE);
  
  // if (pulseRecord) {
  //   pulseRecord.print(heartRate, DEC);
  //   pulseRecord.print("\t");
  //   pulseRecord.print(myavgHR);
  //   pulseRecord.print("\t");
  //   pulseRecord.close();
  // }
  // else {
   // Serial.println("error opening HRLog.txt");
  //   return;
  // }
  
  }
    
}

