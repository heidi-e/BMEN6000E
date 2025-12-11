/* 
Lab Report 5 - Adafruit AS7341 Color Sensor
Calibration curve and moving average calculations
Heidi Eren
Allen Hong
 */

#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;

// Moving average variables
// 10 point moving average
int i = 0;
const int buffsize = 10;
float colorBuff[buffsize];
float sumColor = 0;
float avgColor;

// 50-point moving average
int j = 0;
const int j_buffsize = 50;
float j_colorBuff[j_buffsize];
float j_sumColor = 0;
float j_avgColor;


float ketone_concentration;

void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }
  
  if (!as7341.begin()){
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
  if (!as7341.readAllChannels()) {
    Serial.println("Error");
    return;
  }

  int F1  = as7341.getChannel(AS7341_CHANNEL_415nm_F1);   // violet
  int F2  = as7341.getChannel(AS7341_CHANNEL_445nm_F2);   // indigo
  int F3  = as7341.getChannel(AS7341_CHANNEL_480nm_F3);   // blue
  int F4  = as7341.getChannel(AS7341_CHANNEL_515nm_F4);   // cyan
  int F5  = as7341.getChannel(AS7341_CHANNEL_555nm_F5);   // green
  int F6  = as7341.getChannel(AS7341_CHANNEL_590nm_F6);   // yellow
  int F7  = as7341.getChannel(AS7341_CHANNEL_630nm_F7);   // orange
  int F8  = as7341.getChannel(AS7341_CHANNEL_680nm_F8);   // red
  int CLR = as7341.getChannel(AS7341_CHANNEL_CLEAR);
  int NIR = as7341.getChannel(AS7341_CHANNEL_NIR);

  // print the violet channel light intensity 
  // Serial.print("Violet415:");
  // Serial.print(F1);
  
  // convert 16-bit ADU value to ketone concentration
  // use calibration curve 
  ketone_concentration = 129.3 * exp(-0.017 * F1);
  Serial.print("Detected ketone concentration:")
  Serial.println(ketone_concentration);

  // output ketone concentration
  if (ketone_concentration > 80) {
  Serial.println("Ketone level: High");
  Serial.println("Seek medical help");
  }
  else if (ketone_concentration > 30 && ketone_concentration <= 40) {
    Serial.println("Ketone level: Moderate");
  }
  else if (ketone_concentration > 15 && ketone_concentration <= 20) {
    Serial.println("Ketone level: Small");
  }
  else if (ketone_concentration <= 15) {  
    Serial.println("Ketone level: Negative/Trace");
  }

  // compute 10-point moving average using number of samples actually collected
  sumColor -= colorBuff[i];  // remove previous sample
  colorBuff[i] = F1;   // replace with new color value
  sumColor += colorBuff[i]; //compute sum of color values
  i++;
  if (i >= buffsize) {i = 0;}  //make sure the buffsize is not exceeded
  avgColor = sumColor/buffsize; //update average color value for new sample

  // compute 50-point moving average using number of samples actually collected
  j_sumColor -= j_colorBuff[j];  // remove previous sample
  j_colorBuff[j] = F1;   // replace with new color value
  j_sumColor += j_colorBuff[j]; //compute sum of color values
  j++;
  if (j >= j_buffsize) {j = 0;}  //make sure the buffsize is not exceeded
  j_avgColor = j_sumColor/j_buffsize; //update average color value for new sample

  // print on serial plotter

  // moving average
  Serial.print("None:");
  Serial.print(F1);

  Serial.print(" Ten:");
  Serial.print(avgColor);

  Serial.print(" Fifty:");
  Serial.println(j_avgColor);
}

