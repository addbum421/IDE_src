#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// configurable parameters
#define SND_VEL 346.0 
#define INTERVAL 25 
#define _SERVO_SPEED 78 
#define _DIST_MIN 100 
#define _DIST_MAX 400 

#define _DUTY_MIN 550 
#define _DUTY_NEU 1475 
#define _DUTY_MAX 2400 

#define _DUTY_B 1410
#define _DUTY_B_MR 1010
#define _DUTY_B_PR 1810

#define _INTERVAL_DIST    10
#define _INTERVAL_SERVO    10 
#define _INTERVAL_SERIAL    100

int a, b;
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev; 
unsigned long last_sampling_time; 
float scale; 
float dist_ema = 0;
float alpha = 0.5;
float angle;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;
float duty_chg_per_interval;
float duty_target, duty_curr;
float dist_cali;
float control;
float error_prev = 0;
float error_curr = 0;
float _KP = 0.4;
float _KD = 30.0;
//float _KP = 0.7;
//float _KD = 22.0;
float pterm;
float dterm;

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
    error_curr = dist_ema - 320;

    
    pterm = _KP * error_curr;
    
    dterm = _KD * (error_curr - error_prev);
    control = dterm + pterm;
    //if (control > 0 ) control *= 1.8;
    if (dist_ema > 500 || dist_ema < 8) control = 0;
    duty_target = _DUTY_B - control;
    
    
    
    if (duty_target < _DUTY_B_MR ) duty_target = _DUTY_B_MR; 
    if (duty_target > _DUTY_B_PR ) duty_target = _DUTY_B_PR; 

    error_prev = error_curr;
    
    
    /*
    Serial.print("error_curr:");
    Serial.print(error_curr);
    Serial.print("control :");
    Serial.print(control);
    */
  }
  

  if(event_servo) {
    event_servo = false;
    myservo.writeMicroseconds(duty_curr);
    //myservo.writeMicroseconds(_DUTY_B);
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
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",dterm:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    //Serial.print(",Low:200,dist_target:255,High:310,Max:410");
  }


  
  }
