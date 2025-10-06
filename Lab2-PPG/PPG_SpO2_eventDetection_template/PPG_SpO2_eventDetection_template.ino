#include <Wire.h>
#include "MAX30105.h" // You will need to download this library. Open the library manager and type "Max3010x" and download the sparkfun library.
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <SPI.h>
#include <SD.h>


MAX30105 PPGSensor;

#define I2C_SPEED_FAST 400000
#define MAX_BRIGHTNESS 255

const int chipSelect = 4;
File pulseRecord;

// Sensor input/output variables
int i;
uint32_t irBuffer[200]; //infrared LED sensor data
uint32_t redBuffer[200];  //red LED sensor data
byte readLED = 13; //Blinks with each data read
byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 200; //DO NOT ADJUST
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

// Variables for pre-existing maxim SpO2/HR calculations
int32_t bufferLength = 200; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

// Variables to calculate SpO2
float maxRed;
float minRed;
float maxIR;
float minIR;
float sumRed;
float sumIR;
float redDC;
float irDC;
float redAC;
float irAC;
float R;
float mySpO2;



// Variables to calculate HR
float currIR;
float diffIR[200];
byte isPos = 0;
byte prevPos = 0;
float diffThresh = 1500;
float IRthresh = 50000;
int beatDetected;
float beatTime;
float prevBeatTime =0;
float timeBetweenBeats;
float myHR;
const int buffsize = 8;
float HRBuff[buffsize];
float sumHR;
float myavgHR;
int j;

// Variables for HRV calculation
float IBIavg;
float IBI;
float sumVar;
float variance;
float HRV;

const int IBIbuffsize = 8;
float IBIBuff[IBIbuffsize];
int ibiIndex = 0;


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
  
  // Creating a window of length 200 samples
  for (i = 0 ; i < bufferLength ; i++)
  {
    while (PPGSensor.available() == false) //do we have new data?
      PPGSensor.check(); //Check the sensor for new data

    redBuffer[i] = PPGSensor.getRed();
    irBuffer[i] = PPGSensor.getIR();
    PPGSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //Sliding the window by 50 samples
  while (1) {
  
    //dumping the first 50 sets of samples in the memory and shift the last 150 sets of samples to the top
    for (int i = 50; i < 200; i++)
    {
      redBuffer[i - 50] = redBuffer[i];
      irBuffer[i - 50] = irBuffer[i];
      diffIR[i - 50] = diffIR[i];
      
    }

    //Adding the new 50 samples to the sliding window
    for (int i = 150; i < 200; i++)
    {
      while (PPGSensor.available() == false) //do we have new data?
      
      PPGSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = PPGSensor.getRed();
      irBuffer[i] = PPGSensor.getIR();

      // Calculate the derivative for event detection
      float prevIR = irBuffer[i-1];
      currIR = irBuffer[i];
      diffIR[i] = (currIR - prevIR) * sampleRate;

    
      PPGSensor.nextSample(); //We're finished with this sample so move to next sample

      //For observation of the red signal, IR signal and the derivative or IR
      // Serial.print(redBuffer[i], DEC);
      // Serial.print("\t");
      // Serial.print(irBuffer[i], DEC);
      // Serial.print("\t");
      // Serial.print(diffIR[i]);
      // Serial.println("\t");
      
    }
    
    // Calculate HR and SpO2 using existing library
    // For comparison to your calculated values
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
    // Calculate SpO2 manually using equation from TI application report (5 pts)
    
    // define variables
    sumRed = 0;
    sumIR = 0;
    maxRed = redBuffer[0];
    minRed = redBuffer[0];
    maxIR  = irBuffer[0];
    minIR  = irBuffer[0];

    for (int i = 1; i < bufferLength; i++) {
        
        // compute sums of red and IR signal
        sumRed += redBuffer[i];
        sumIR  += irBuffer[i];

        // find min and max values
        if (redBuffer[i] > maxRed) maxRed = redBuffer[i];
        if (redBuffer[i] < minRed) minRed = redBuffer[i];
        if (irBuffer[i]  > maxIR)  maxIR  = irBuffer[i];
        if (irBuffer[i]  < minIR)  minIR  = irBuffer[i];
    
        // copmute DC (take the average of red and of IR)
        redDC = sumRed / bufferLength;
        irDC  = sumIR  / bufferLength;

        
        // compute AC (amplitude of wave)
        redAC = (maxRed - minRed) / 2.0;  
        irAC  = (maxIR - minIR) / 2.0;

        // compute R (ratio of normalized amplitudes between red and IR channels)
        R = ((redAC / redDC) / (irAC / irDC));
        mySpO2 = 110 - 25 * R;
    

    // Calculate HR manually using event detection //HR detection uses the IR component (5 pts)
    
    prevPos = isPos;
    // comparision expression
    // true (1) --> slope is positive
    // false (0) --> slope is negative
    isPos = (diffIR[i] > diffThresh);  // true if slope is rising

    // for peak detection - slope switched from pos to neg and above IR threshold
    if (prevPos == 1 && isPos == 0 && irBuffer[i] > IRthresh) {
        beatDetected++;
        beatTime = millis();
        timeBetweenBeats = beatTime - prevBeatTime;

        if (timeBetweenBeats > 300 && timeBetweenBeats < 2000) { 
            // ignore out of range intervals (<30 bpm or >200 bpm)
            myHR = 60000.0 / timeBetweenBeats; // bpm

            // keep last 8 HR values
            HRBuff[j % buffsize] = myHR;
            j++;

            // compute moving average of HR
            sumHR = 0;
            for (int k = 0; k < min(j, buffsize); k++) {
                sumHR += HRBuff[k];
            }
            myavgHR = sumHR / min(j, buffsize);
        }

        prevBeatTime = beatTime;
    }



    // Calculate HRV manually (5pts (BONUS))
    IBI = beatTime - prevBeatTime;   // ms between beats
    IBIBuff[ibiIndex % IBIbuffsize] = IBI;
    ibiIndex++;

    sumVar = 0;
    for (int k = 0; k < min(ibiIndex, IBIbuffsize); k++) {
        sumVar += IBIBuff[k];
    }
    IBIavg = sumVar / min(ibiIndex, IBIbuffsize);

    //HRV as SSN
    // variance
    sumVar = 0;
    for (int k = 0; k < min(ibiIndex, IBIbuffsize); k++) {
        sumVar += pow(IBIBuff[k] - IBIavg, 2);
    }
    variance = sumVar / min(ibiIndex, IBIbuffsize);

    // HRV = standard deviation of IBI
    HRV = sqrt(variance);
    
    // HRV as RMSSD

    // float sumSqDiff = 0;
    // for (int k = 1; k < min(ibiIndex, IBIbuffsize); k++) {
    //     float diff = IBIBuff[k] - IBIBuff[k-1];
    //     sumSqDiff += diff * diff;
    // }
    // HRV = sqrt(sumSqDiff / (min(ibiIndex, IBIbuffsize)-1));


    // Calculate moving average of HR (1 pt)

    
    Serial.print(spo2, DEC);
    Serial.print("\t");
    Serial.print(mySpO2, 0);
    Serial.print("\t");
    Serial.print(heartRate, DEC);
    Serial.print("\t");
    Serial.print(myavgHR, 0);
    Serial.print("\t");
    Serial.print(HRV, 0);
    Serial.println("\t");

  // pulseRecord = SD.open("HRLog.txt", FILE_WRITE);
  
  // if (pulseRecord) {
  //   pulseRecord.print(spo2, DEC);
  //   pulseRecord.print("\t");
  //   pulseRecord.print(mySpO2, 0);
  //   pulseRecord.print("\t");
  //   pulseRecord.print(heartRate, DEC);
  //   pulseRecord.print("\t");
  //   pulseRecord.print(myavgHR);
  //   pulseRecord.print("\t");
  //   pulseRecord.print(HRV);
  //   pulseRecord.println("\t");
  //   pulseRecord.close();
  // }
  // else {
  //  Serial.println("error opening HRLog.txt");
  // }
  }
  }
  
  
}
