#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_POTENTIOMETER 0
#define PIN_SERVO 10

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 20      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 60.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 400.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Target Distance
#define _TARGET_LOW  100.0
#define _TARGET_HIGH 250.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time; // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)
  // initialize serial port
  Serial.begin(1000000);
}

void loop() {
  float  dist_raw, dist;
  int a_value, duty;
  
  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  a_value = analogRead(PIN_POTENTIOMETER);
  duty = (a_value / 1023.0) * (_DIST_MAX - _DIST_MIN) + _DIST_MIN;
  dist = (6762.0/(a_value-9)-4.0)*10.0 - 60.0;

  dist_raw = duty;

  if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (100.0 < dist_raw && dist_raw < 250.0) {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
    dist_prev = dist_raw;
  } else {
    digitalWrite(PIN_LED, 1);
    dist_prev = dist_raw; 
  }

  // Apply ema filter here  
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // adjust servo position according to the USS read value

  // add your code here!
  if (dist_ema <= 100.0) {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if (100.0 < dist_ema && dist_ema < 250.0) {
    myservo.writeMicroseconds(_DUTY_MIN + 1856 * (dist_ema - 100) / 150);
  }
  if (250.0 <= dist_ema) {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  // Use _TARGET_LOW, _TARGTE_HIGH

  // output the distance to the serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",IR:");   Serial.print(a_value);
  Serial.print(",dist:");  Serial.print(dist);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(duty);  
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
