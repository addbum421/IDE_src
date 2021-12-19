#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
#include "medianfilter.h"

float ir_distance(short value){ // return value unit: mm
  float val;
  float dist;
  int a, b;
  a = 99.5; 
  b = 260.7; 
  float volt = value;
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  dist =  (100 + 300.0 / (b - a) * (val - a)) ;
  return dist;
}
MedianFilter<ir_distance>  filter;

// configurable parameters
#define SND_VEL 346.0 
#define INTERVAL 25 
#define _SERVO_SPEED 50 
#define _DIST_MIN 100 
#define _DIST_MAX 400 

#define _DUTY_MIN 550 
#define _DUTY_NEU 1475 
#define _DUTY_MAX 2400 

#define _DUTY_B 1450
#define _DUTY_B_MR 1310
#define _DUTY_B_PR 1510

#define _INTERVAL_DIST    10
#define _INTERVAL_SERVO    10 
#define _INTERVAL_SERIAL    100

int a, b;
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; 
unsigned long last_sampling_time; 
float dist_target = 255.0;
float scale; 
float dist_ema = 0;
float alpha = 0.38;
float angle;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;
float duty_chg_per_interval;
float duty_target, duty_curr;
float dist_cali;
float control;
float error_prev = 0;
float error_curr = 0;
float _KP =  0.33;
float _KD = 88.0;
float _KI = 0.002;
float pterm;
float dterm;
float iterm;
float _ITERM_MAX = 20;

Servo myservo;

void setup() {

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_B);
  duty_target = duty_curr = _DUTY_B;
  digitalWrite(PIN_LED, 0);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; 
  dist_raw = dist_prev = 0.0; 
  scale = 0.001 * 0.5 * SND_VEL;
  a = 65; 
  b = 250; 
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  duty_chg_per_interval = float((_DUTY_MAX - _DUTY_MIN)) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);
// initialize serial port
  Serial.begin(57600);
// initialize last sampling time
  last_sampling_time = 0;
  filter.init();
}

void loop(){
    unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(event_dist) {
    event_dist = false;
    if (filter.ready()){
      dist_raw = filter.read() + 3;
    }
    //dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
    dist_ema = alpha * dist_raw + (1-alpha) * dist_ema;
    error_curr = dist_target - dist_ema;

    
    pterm = _KP * error_curr;
    //if (pterm < 0 ) pterm *= 1.8;
    
    dterm = _KD * (error_curr - error_prev);
    if (dist_ema < 270 && dist_ema > 240){
     if (abs(error_curr - error_prev) < 15) dterm = 0; ;
    }
    
    iterm += _KI * error_curr; 
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
    control = dterm + pterm + iterm;
    //if (control > 0 ) control *= 2.0;
    //if (dist_ema < 265 && dist_ema > 245){
    //  if (abs(error_curr - error_prev) < 50) dterm = 0; ;
    //}
    duty_target = _DUTY_B + control;
    
    if (duty_target < _DUTY_B_MR ) duty_target = _DUTY_B_MR; 
    if (duty_target > _DUTY_B_PR ) duty_target = _DUTY_B_PR; 


    error_prev = error_curr;
    
  }
  

  if(event_servo) {
    event_servo = false;
    //myservo.writeMicroseconds(duty_curr);
   // myservo.writeMicroseconds(_DUTY_B);
    //myservo.writeMicroseconds(_DUTY_B_MR);
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
      }
    myservo.writeMicroseconds(duty_curr);
  }
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T: ");
    Serial.print(dist_target);
    Serial.print(",P:");  
    Serial.print(map(pterm, -1000, 1000,510,610));
    Serial.print(",D: ");
    Serial.print(map(dterm, -1000, 1000, 510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000, 1000,510,610));
    Serial.print(", DTT: ");
    Serial.print(map(duty_target, 1000, 2000,410,510)); 
    Serial.print(",DTC: ");
    Serial.print(map(duty_curr, 1000, 2000,410,510));
    Serial.println(", -G:245, +G: 265,m:0, M:800");
    
  }

  
  }
