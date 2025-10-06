/**************************************************************************
Thermistors
BMEN 6000 - Signal Processing for Medical Devices 2025
 **************************************************************************/

// Heidi Eren
// Lab 1 Arduino code


// Fill in variable values correctly: 5 pts
// Required libraries

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// Input parameters

float R1 = 10000;
float R2 = 10000;
float R3 = 10000;
float inVoltage = 3.3;
float sampRate = 256;

// DAQ variables

float V1;
float V2;

// Temp calculation variables

float Vdiff;
float resistance;
float logR;
float SHa = 0.00242954349576175;
float SHb = 0.0000107263530098312;
float SHc = 9.31866759185675E-07;
float temp;

// Moving average variables

int i = 0;
const int buffsize = 10;
float tempBuff[buffsize];
float sumTemp = 0;
float avgTemp;

// Save data to SD card

const int chipSelect = 4;
File tempRecord;

// Initialize data acqusition and calculations

void setup() {

  Serial.begin(9600);
  while (!Serial);

// Load data onto SD card as txt file

SD.begin(chipSelect);
if (!SD.begin(chipSelect)) {
  Serial.println("SD not found ");
  return;
}
Serial.println("SD found");

tempRecord = SD.open("tempLog.txt", FILE_WRITE);
if (tempRecord){
tempRecord.println("Voltage (1)  Voltage (2)  Voltage (diff)  Resistance  Temp (C)  Average temp (C)");
tempRecord.close();
}
else {
  Serial.println("error opening tempRecord.txt");
  return;
}

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

}

void loop() {

  // Read voltage from Wheatstone bridge

  V1 = analogRead(A2);  
  V2 = analogRead(A1);

  // Calculate difference between nodes (equivalent to voltage across bridge)
  // 3 pts

  V1 = (V1 * 3.3) / 1023;   //convert bits to voltage
  V2 = (V2 * 3.3) / 1023;
  Vdiff = V2 - V1;


  // Calculate resistance from voltage
  // 7 pts
  resistance = (((R2 * inVoltage - (R1 + R2) * Vdiff) / (R1 * inVoltage + (R1 + R2)* Vdiff))) * R3;


  // Calculate temperature from resistance
  // 10 pts
  logR = log(resistance);
  temp = (1/(SHa + SHb * logR + SHc*pow(logR,3))) - 273.15;

  // Moving average
  // 10 pts
  sumTemp -= tempBuff[i];    //remove previous temp sample
  tempBuff[i] = temp;    //replace with new temp sample
  sumTemp += tempBuff[i];    //compute sum of temp list

  i++;
  

  if (i >= buffsize) {i = 0;}    //make sure the buffsize is not exceeded
  avgTemp = sumTemp / buffsize;    //update avg temp for new temp sample

  // Output (Serial.print prints to the serial monitor; tempRecord.print prints to the SD card)

  Serial.print(V1);
  Serial.print("\t");
  Serial.print(V2);
  Serial.print("\t");
  Serial.print(Vdiff);
  Serial.print("\t");
  Serial.print(resistance);
  Serial.print("\t");
  Serial.print(temp);
  Serial.print("\t");
  Serial.println(avgTemp);
  

  // Uncomment to use SD card
  
  tempRecord = SD.open("tempLog.txt", FILE_WRITE);
  
  if (tempRecord) {
    tempRecord.print(V1);
    tempRecord.print("\t");
    tempRecord.print(V2);
    tempRecord.print("\t");
    tempRecord.print(Vdiff);
    tempRecord.print("\t");
    tempRecord.print(resistance);
    tempRecord.print("\t");
    tempRecord.print(temp);
    tempRecord.print("\t");
    tempRecord.println(avgTemp);
    tempRecord.close();
  }
  // else {
  //   Serial.println("error opening tempLog.txt");
  //   return;
  // }
  
  // Implement sampling rate: 5 pts
  // sample rate = 256 ms → ~3.9 Hz sampling rate
  delay(1000);

}





