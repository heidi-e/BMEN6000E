#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// Initial conditions
float dQ;
float Q = 0;
float dG;
float G = 163;
float dI;
float I = 11.26;

int i;
int j;


// Variables
float g_error = 0, g_prev = 0, g_deriv = 0, sumG = 0;
float G_set = 120.0; // target glucose (mg/dL)

float t_min = 0.0;
float dt_min = 0.1;

// PID controller gains
float Prop = 0.3; // set a value here initially (height of peak is reduced)
float Deriv = 0.005; // increase this to reduce overshoot (narrowness of graph)
float Integer = 0.05; // increase this to reduce steady-state offset
float addIn;

// bolus insulin control signal
float bolusThreshold = 160.0;  // mg/dL
float bolusDose = 0.5;         // µU/mL/min injection strength
float bolusDuration = 5.0;     // minutes
float bolusTimer = 0.0;        // timer to limit bolus duration
float addIn_bolus;

float mealStart = 0;     // min from simulation start
float mealDuration = 15.0; // min over which glucose is absorbed
float mealSize = 15000.0;  // mg of glucose (15 g)


//Meal size - mg/min delivered to the gut
float D;

// Non-diabetic model
// float beta = 20;
// float eta = 4.086;
// float gam = 40;
// float R0 = 2.1;
// float E = pow(10, -3);
// float S = 3.06 * pow(10, -3);
// float kq = 0.098;
// float Imax = 0.28;
// float alpha = pow(10, 4);
// float ki = 0.01;


// Diabetic model
float beta = 10;
float eta = 4.641;
float gam = 25;
float R0 = 2.5;
float E = 2.5 * pow(10, -3);
float S = 1.14 * pow(10, -3);
float kq = 0.026;
float Imax = 0.93;
float alpha = pow(10, 4);
float ki = 0.06;

const int chipSelect = 4;
File glucoseSystem;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Initializing...");

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  SD.begin(chipSelect);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD not found ");
    return;
  }
  Serial.println("SD found");

  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
  if (glucoseSystem){
    glucoseSystem.println("Q \t G \t I");
    glucoseSystem.close();
  }
  else {
    Serial.println("error opening gluSys.txt");
    return;
  }

}

void loop() {

// Add meal "input" (3 pts)

if (t_min >= mealStart && t_min < mealStart + mealDuration) {
  D = mealSize / mealDuration;   // constant inflow rate
} else {
  D = 0.0;                       // no meal after digestion
}


// Bolus control (5 pts)
if (G > bolusThreshold && bolusTimer <= 0) {
    addIn_bolus = bolusDose;         // trigger bolus
    bolusTimer = bolusDuration; // countdown active
  }
  if (bolusTimer > 0) {
    bolusTimer -= dt_min;
  } else {
    addIn_bolus = 0; // bolus off
  }



// PID control (7 pts)

// convert bits to voltage
float V = analogRead(A2) * (3.3 / 1023);

// input curve fitting linear response (from part 1)
float G_sensor = (V - 1.3995) / 0.0064;   // glucose in mg/dL
if (G_sensor < 0) G_sensor = 0;           // clamp to 0 if below baseline

g_error = G_set - G_sensor; // PID error
sumG += g_error * dt_min;  // integral term
g_deriv = (g_error - g_prev) / dt_min; // derivative term
g_prev = g_error;

// closed loop PID controller system
addIn = Prop * g_error + Integer * sumG + Deriv * g_deriv;
if (addIn < 0) addIn = 0;


// Differential equations for Q, G, I (12 pts)
//I = (Imax * pow(G, 2))/ (ki * (alpha + pow(G,2)));
//Q = (beta * D) / alpha;

// D(t) = amount of ingested glucose from food intake (mg)
dQ = - (beta * Q + eta*D)/(pow(gam,2) + pow(Q, 2));
// G(t) = blood glucose concentration (mg/dI)
dG = R0 - (E + S*I) * G + kq * Q;
// I(t) = concentration of insulin in bloodstream (μU/ml)
dI = Imax * (pow(G,2)/(alpha + pow(G, 2))) - ki * I + addIn + addIn_bolus;


// integrate
Q += dQ * dt_min;
G += dG * dt_min;
I += dI * dt_min;


  Serial.print(Q);
  Serial.print("\t");
  Serial.print(G);
  Serial.print("\t");
  Serial.print(I);
  Serial.println("\t");

//  File glucoseSystem = SD.open("gluSys.txt", FILE_WRITE);
//     if (glucoseSystem) {
//     glucoseSystem.print(Q);
//     glucoseSystem.print("\t");
//     glucoseSystem.print(G);
//     glucoseSystem.print("\t");
//     glucoseSystem.println(I);
//     glucoseSystem.close();
//   }
//   else {
//     Serial.println("error opening gluSys.txt");
//     return;
//   }
  t_min += dt_min;
  delay(100);

}
