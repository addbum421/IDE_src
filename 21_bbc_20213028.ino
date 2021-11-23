#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define SND_VEL 346.0 
#define INTERVAL 25 
#define _SERVO_SPEED 90 
#define _DIST_MIN 100 
#define _DIST_MAX 400 

#define _DUTY_MIN 550 
#define _DUTY_NEU 1475 
#define _DUTY_MAX 2400 

#define _DUTY_B 1425
#define _DUTY_B_MR 1325
#define _DUTY_B_PR 1525

#define _INTERVAL_DIST    50
#define _INTERVAL_SERVO    20 
#define _INTERVAL_SERIAL    100

int a, b;
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; 
unsigned long last_sampling_time; 
float scale; 
float dist_ema = 0;
float alpha = 0.3;
float angle;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;
float duty_chg_per_interval;
float duty_target, duty_curr;
float dist_cali;

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
  a = 69; 
  b = 230; 
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  duty_chg_per_interval = float((_DUTY_MAX - _DUTY_MIN)) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0);
// initialize serial port
  Serial.begin(57600);
// initialize last sampling time
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
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
    dist_raw = ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
    dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;
    if (dist_ema > 255.0) duty_target = _DUTY_B_MR;
    else duty_target = _DUTY_B_PR;

  }

  if(event_servo) {
    event_servo = false;
    myservo.writeMicroseconds(duty_curr);
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
      }
  
  }
  if(event_serial) {
    event_serial = false;
        Serial.print("min:0,max:500,dist:");
    Serial.print(dist_raw);
    Serial.print(",dist_cali:");
    Serial.print(dist_cali);
    Serial.print(", dist_ema:");
    Serial.println(dist_ema);
  }


  
  }
