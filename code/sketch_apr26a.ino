

/************************************************************
************************************************************/
#include <Wire.h>
#include <GP2Y0E03.h>
#include <Servo.h>
/************************************************************
************************************************************/
const int GP2Y0E03_I2C_ADDR = 0x40;
GP2Y0E03 gp2y0e03( GP2Y0E03_I2C_ADDR );


Servo MyServo;


float distance;
/************************************************************
************************************************************/
float a = -0.0000810776;
float b = -0.00226357;
float c = 1.02328;

/************************************************************
************************************************************/
float distance_setpoint = 20;

float elapsedTime, Time, timePrev;
float distanceError = 0.0;
float distanceError_prev = 0.0;
float distanceError_prev2 = 0.0;
float dist_diference;
float period = 50;
float period1;

float last_not_zero;
/************************************************************
************************************************************/
float kp = 0.9;
float ki =  0.08;
float kd = -1000;

double PID_p, PID_d, PID_i, PID_total ,PID_d_prev;
float max_PID_total;

float theta = 0.7;
float theta_d = 0.1;
/******************************
******************************/
void setup() {
  Serial.begin(9600);
  Wire.begin();
  MyServo.attach(9);

  MyServo.write(125);
  Time = millis();
  PID_i = 0.0;

  delay(500);
}

/******************************
******************************/
void loop() {
  if (millis() > Time + period)
  {

    distance = gp2y0e03.get_length();
    if ( distance == -1 )
    {
      distance = 0.0;
    }
    // correct distance;
    distance =  a * pow( distance , 3) + b * pow(distance , 2) + c * distance;
    period1 = Time - millis();
    Time = millis();


    if (distance != 0.0)
    {
      last_not_zero = distance;
    }

    if (distance == 0.0 && last_not_zero > distance_setpoint)
    {
      distance = distance_setpoint * 2;
    }
    distanceError = distance - distance_setpoint;
    distanceError = theta * distanceError + (1 - theta) * distanceError_prev;

    PID_p = (distanceError +(distanceError_prev2 - 4*distanceError_prev + 3*distanceError)/(2.0 * period1) + period/1000)* kp;

    
    PID_d = kd * (distanceError_prev2 - 4*distanceError_prev + 3*distanceError)/(2.0 * period1);
    PID_d = theta_d * PID_d + (1 - theta_d) * PID_d_prev;
    PID_d_prev = PID_d;
    if (-1 > PID_d || PID_d > 1)
    {
       PID_i = PID_i;
    }
    else{
    PID_i = PID_i + (ki * distanceError);
    }
    if (-2 < distanceError && distanceError < 2)
    {
      PID_i = PID_i - (ki * distanceError);
    }
    
    PID_total = PID_p + PID_d + PID_i;

    max_PID_total = 100;
    PID_total = map(PID_total, -1 * max_PID_total, max_PID_total, -50, 50);
    //PID_total = -1* PID_total;
    MyServo.write(125.0 + PID_total);

    distanceError_prev2 = distanceError_prev;
    distanceError_prev = distanceError;

    Serial.print(PID_p);
    Serial.print("\t");
    Serial.print(PID_i);
    Serial.print("\t");
    Serial.println(PID_d);
    //Serial.print("\t");
    //Serial.println(PID_total);
  }
}
