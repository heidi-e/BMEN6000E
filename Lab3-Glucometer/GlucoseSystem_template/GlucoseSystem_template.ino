#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// Initial conditions
float dQ;
float Q = ;
float dG;
float G = ;
float dI;
float I;

int i;
int j;

// PID gains for bolus insulin control
float Kp = 0.02;
float Ki = 0.002;
float Kd = 0.001;

// Variables
float g_error, g_prev = 0, g_deriv, sumG = 0;
float bolus = 0;    // bolus insulin control signal


// PID controller gains
float P = ; // set a value here initially
float Deriv = 0; // increase this to reduce overshoot
float Integer = 0; // increase this to reduce steady-state offset
float addIn;
float sumG = ;

float mealStart = 5.0;     // minutes from simulation start
float mealDuration = 30.0; // minutes over which glucose is absorbed
float mealSize = 60000.0;  // mg of glucose (≈60 g)


//Meal size - mg/min delivered to the gut
float D = 0;

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

// gam = Y
// eta = n

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


// Differential equations for Q, G, I (12 pts)

I = (Imax * pow(G, 2))/ (ki * (alpha + pow(G,2)));
Q = (beta * D) / alpha;
G = 

// D(t) is the amount of ingested glucose from food intake (mg)
dQ = - (beta * Q + eta*D)/(pow(gam,2) + pow(Q, 2));

dG = R0 - (E + S*I) * G + kq * Q;

dI = Imax * (pow(G,2)/(alpha + pow(G, 2))) - ki * I


// Bolus control (5 pts)


// Variables
float g_error, g_prev = 0, g_deriv, sumG = 0;
float bolus = 0;    // bolus insulin control signal

// --- inside loop ---
g_error = G_set - G;                 // glucose error (mg/dL)
sumG += g_error;                     // integral
g_deriv = g_error - g_prev;          // derivative
bolus = P*g_error + Integer*sumG + Deriv*g_deriv;  // bolus control (units/min)
g_prev = g_error;

// Inject bolus into the insulin dynamics
float dI = Imax * (pow(G,2)/(alpha + pow(G,2))) - ki * I + bolus;

// integrate
Q += dQ * dt_min;
G += dG * dt_min;
I += dI * dt_min;



// PID control (7 pts)
float Vref = 3.3;            // in voltage
int ADCmax = 1023;           // 10-bit Arduino Uno

// convert bits to voltage
float V = analogRead(A2) * (Vref / ADCmax);

// input curve fitting linear response
float G_sensor = (V - 1.3995) / 0.0064;   // glucose in mg/dL
if (G_sensor < 0) G_sensor = 0;           // clamp to 0 if below baseline


float G_set = 120.0;                       // target glucose (mg/dL)
float G_meas = G_sensor;                   // measured glucose
float g_error = G_set - G_meas;            // PID error

sumG += g_error;                           // integral term
float g_deriv = (g_error - g_prev);        // derivative term


addIn = P * g_error + Integer * sumG + Deriv * g_deriv;

  // Serial.print(Q);
  // Serial.print("\t");
  // Serial.print(G);
  // Serial.print("\t");
  // Serial.print(I);
  // Serial.println("\t");

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

  delay(100);

}
