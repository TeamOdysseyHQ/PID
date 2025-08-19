#include <Arduino.h>
#include <util/atomic.h>


class PID{
  private:
    // PID parameters
    float kp, kd, ki, umax;
    float eprev, eintegral; 

  public:
  // Constructor
  PID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  void setParams_PID(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void PID_calc(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

// No. Of motors
const int  NMOTORS = 2;

// Pins
const int ENCO_CH_1[] = {0,6};
const int ENCO_CH_2[] = {2,4};
const int PWM[] = {10,8};
const int DIR[] = {1,5};
const int SLEEP[] = {9,7};

// Globals
long prevT = 0;
int posPrev[NMOTORS] = {0,0};

volatile int posi[] = {0,0};
volatile float velocity_i[] = {0,0};
volatile long prevT_i[] = {0,0};

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

// PID class instances
PID pid[NMOTORS];

void setMotor(int dir, int pwmVal,int sleepval ,int PWM, int DIR, int SLEEP){
  if(sleepval == 1){
    digitalWrite(SLEEP, HIGH);  
  }
  else{
    if(dir == 1){
      digitalWrite(DIR, HIGH);
    }
    else{
      digitalWrite(DIR, LOW);
    }  
    digitalWrite(SLEEP, LOW);
    analogWrite(PWM, pwmVal);
  }
}

template <int j>
void readEncoder(){
  int b = digitalRead(ENCO_CH_2[j]);
  int increment = 0;
  if(b > 0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  posi[j] = posi[j] + increment;

  // Computation of velocity thriugh continous time differentiatial between the pulses
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i[j]))/1.0e6;
  velocity_i[j] = increment/deltaT;
  prevT_i[j] = currT;
}

void setup() {
  Serial.begin(115200);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(ENCO_CH_1[k],INPUT);
    pinMode(ENCO_CH_2[k],INPUT);
    pinMode(PWM[k],OUTPUT);
    pinMode(SLEEP[k],OUTPUT);
    pinMode(DIR[k],OUTPUT);

    pid[k].setParams_PID(8,0,0,255);
  }
  
  attachInterrupt(digitalPinToInterrupt(ENCO_CH_1[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCO_CH_1[1]),readEncoder<1>,RISING);
  
  Serial.println("target rpm");
}

void loop() {

  // Read in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  int velocity2[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
      velocity2[k] = velocity_i[k];
    }
  }

  // Computation of velocity using discrete pulse counting
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1[NMOTORS];  

  for(int k = 0; k < NMOTORS; k++){
    velocity1[k] = (pos[k] - posPrev[k])/deltaT;
    posPrev[k] = pos[k];
    prevT = currT;
  }

  float v1[NMOTORS] = {0,0};
  float v2[NMOTORS] = {0,0};
  float v1Filt[NMOTORS] = {0,0};
  float v1Prev[NMOTORS] = {0,0};
  float v2Filt[NMOTORS] = {0,0};
  float v2Prev[NMOTORS] = {0,0};

  for (int k = 0; k < NMOTORS; k++){
    // Convert velocity to RPM
    // 7 (Pulse per revolution) * 181 (Motor's Gear Ratio) * 60 (seconds per minute) 
    v1[k] = velocity1[k]/1267.0*60.0;
    v2[k] = velocity2[k]/1267.0*60.0;

    // Low-pass filter (25 Hz cutoff)
    v1Filt[k] = 0.854*v1Filt[k] + 0.0728*v1[k] + 0.0728*v1Prev[k];
    v1Prev[k] = v1[k];
    v2Filt[k] = 0.854*v2Filt[k] + 0.0728*v2[k] + 0.0728*v2Prev[k];
    v2Prev[k] = v2[k];
  }

  int vt[NMOTORS] = {20,20};

  // time difference
  currT = micros();
  deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;


  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].PID_calc(pos[k],vt[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,0,PWM[k],DIR[k],SLEEP[k]);
  }

  for(int k = 0; k < NMOTORS; k++){
    Serial.print(vt[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();
}

