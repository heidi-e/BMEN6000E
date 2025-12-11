/* 
Lab Report 5 - Adafruit AS7341 Color Sensor
Derivative calculations
Heidi Eren
Allen Hong
 */

// make sure to install the Adafruit AS7341 library
#include <Adafruit_AS7341.h>
#include <math.h> 

Adafruit_AS7341 as7341;

// 10-point moving average
int i = 0;
const int buffsize = 10;
float colorBuff[buffsize];
float sumColor = 0;
float avgColor = 0;

// 50-point moving average
int j = 0;
const int j_buffsize = 50;
float j_colorBuff[j_buffsize];
float j_sumColor = 0;
float j_avgColor = 0;



bool firstSample = true;
float prevValue = 0.0;
float deriv = 0.0;

// tune thresholds based on readings
const float MIN_OK = 10.0;     // lower bound
const float MAX_OK = 125.0;    // upper bound
const float DERIV_THRESHOLD = 5.0;  // how big a jump per sample is “significant”

int badCount = 0;
const int BAD_COUNT_THRESHOLD = 3;  // number of consecutive bad samples before warning


void setup() {
  Serial.begin(115200);
  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

  if (!as7341.begin()) {
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  // init moving-average buffers to 0
  for (int k = 0; k < buffsize; k++) {
    colorBuff[k] = 0.0;
  }
  for (int k = 0; k < j_buffsize; k++) {
    j_colorBuff[k] = 0.0;
  }
}

void loop() {
  // you don’t actually need this for-loop; loop() is called repeatedly anyway,
  // but I’ll keep your structure and just rename the counter to avoid clashes.
  for (int sample = 0; sample < 100000; sample++) {

    // 16-bit values
    uint16_t readings[12]; 

    // Read all channels at the same time and store in as7341 object
    if (!as7341.readAllChannels()) {
      Serial.println("Error reading all channels!");
      return;
    }

    readings[0] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);   // Violet
    readings[1] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);   // Indigo
    readings[2] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);   // Blue
    readings[3] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);   // Cyan
    readings[4] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);   // Green
    readings[5] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);   // Yellow
    readings[6] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);   // Orange
    readings[7] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);   // Red
    readings[8] = as7341.getChannel(AS7341_CHANNEL_CLEAR);
    readings[9] = as7341.getChannel(AS7341_CHANNEL_NIR);

    float F1 = readings[0];

    // compute 10-point moving average
    sumColor -= colorBuff[i];     // remove previous sample
    colorBuff[i] = violet_f;      // replace with new color value
    sumColor += colorBuff[i];     //compute sum of color values

    i++;
    if (i >= buffsize) {
      i = 0;
    }
    avgColor = sumColor / buffsize;


    // compute 50-point moving average
    j_sumColor -= j_colorBuff[j];
    j_colorBuff[j] = violet_f;
    j_sumColor += j_colorBuff[j];

    j++;
    if (j >= j_buffsize) {
      j = 0;
    }
    j_avgColor = j_sumColor / j_buffsize;

    // moving average
    Serial.print("None:");
    Serial.print(F1);

    Serial.print(" Ten:");
    Serial.print(avgColor);

    Serial.print(" Fifty:");
    Serial.println(j_avgColor);

    // derivative calculations
    // here we can change what currentValue is
    // it can be any of the moving averages or the raw value
    // for testing purposes we will use the raw 16-bit ADU value
    float currentValue = F1;

    if (!firstSample) {
      deriv = currentValue - prevValue;
    } else {
      firstSample = false;
      deriv = 0.0;
    }
    prevValue = currentValue;

    Serial.print(" | deriv:");
    Serial.print(deriv);

    // check for out-of-range values and output warning
    bool valueTooLow  = (currentValue < MIN_OK);
    bool valueTooHigh = (currentValue > MAX_OK);
    bool bigJump      = (fabs(deriv) > DERIV_THRESHOLD);

    bool badNow = (valueTooLow || valueTooHigh) && bigJump;
    
    // count number of times current value crosses threshold
    if (badNow) {
      badCount++;
    } else {
      badCount = 0;
    }

    bool stripOutOfRange = (badCount >= BAD_COUNT_THRESHOLD);

    if (stripOutOfRange) {
      Serial.print(" | WARNING: Test strip out of range");
      // in the future, we can add a flashing LED on the device here
    }
    Serial.println();

    delay(10);
  }
}
