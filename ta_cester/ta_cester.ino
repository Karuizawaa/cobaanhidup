#include "Servo.h"

Servo falcon;
// Pins
#define ENCA 2
#define PWM 5
#define IN1 6
#define IN2 7

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);
  falcon.attach(13);
  pinMode(ENCA,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Convert count/s to RPM
  float v2 = velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;

  // Compute the control signal u
  float kp = 5;
  float ki = 0;
  float vt = 60;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  gasFalcon(u);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void gasFalcon(int input){
  if(input>0) falcon.writeMicroseconds((input/13)+1500);
  else falcon.writeMicroseconds(1500);
}

void readEncoder(){
  // Compute velocity with method 2
  long currT = micros();
  float deltaT = ((float) (1))/1.0e6;
  velocity_i = 1/deltaT; //rev per seconds
  prevT_i = currT;
}
