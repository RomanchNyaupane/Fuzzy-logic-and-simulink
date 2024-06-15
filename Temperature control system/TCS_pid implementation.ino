// in version 3, i have implemented temperature sensor in feedback
#include <util/atomic.h>
#include <math.h>

float kp = 10;
float ki=0.5;
float kd = 0.2;
long prevT = 0;
float pid_ki = 0;
float prev_err = 0;
float ref, err, feedback_temp, voltage_eqv, pid_output, pid_kp, pid_kd;

// constants and variable for temperature sensor
int ThermistorPin = A0;
int Vo;
float R1 = 9430;
float logR2, R2, T;
float c1 = 1.00949522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ref = 17;
  pinMode(10,OUTPUT);
  pinMode(A0,INPUT);
}

void loop() {
  //defining time for intergal and other purposes
  long currT = micros(); //gives microsecond value of each second passed in time
  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT=currT;

  //feedback:
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  Serial.print("R2");
  Serial.println(R2);
  logR2 = log(R2);
  Serial.print("logR2");
  Serial.println(logR2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Serial.print("sum");
  Serial.println(c1 + c2*logR2 + c3*logR2*logR2*logR2);
  T = T - 273.15;
  Serial.print("T");
  Serial.println(T);
  feedback_temp = T; //temperature sensor
  err = ref-feedback_temp;
  pid_controller(feedback_temp,err,deltaT);
}
void pid_controller(float feedback_temp,float err, float deltaT){
  pid_kp = kp*err;
  pid_ki = ki*( pid_ki+err*deltaT);
  pid_kd = kd*(err-prev_err)/deltaT;
  prev_err = err;
  pid_output = pid_kp + pid_ki + pid_kd;
  
  //Serial.print("PID integr OUT");
  //Serial.println(pid_ki);
  //Serial.print("PID prop OUT");
  //Serial.println(pid_kp);

  //Serial.print("feedback temperature");
  //Serial.println(feedback_temp);
  
  //voltage_eqv = (0.006*pid_output*pid_output*pid_output-0.25*pid_output*pid_output+8.7*pid_output+3.6212);
  voltage_eqv = 12.92909 + 3.996862*pid_output + 0.21102*pow(pid_output,2) - 0.00517697*pow(pid_output,3) - 0.0002181445*pow(pid_output,4) + 0.00000801641*pow(pid_output,5);
  if(voltage_eqv<0){voltage_eqv= -1*voltage_eqv;}
  //Serial.print("Output voltage");
  //Serial.println(voltage_eqv);
  analogWrite(10, voltage_eqv);

  }
