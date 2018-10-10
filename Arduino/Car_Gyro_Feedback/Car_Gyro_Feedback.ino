#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// Ultrasonic
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

// IMU
#define LSM9DS1_XGCS 49
#define LSM9DS1_MCS 47

// Motor
#define motorPin 8
#define maxSpeed 255
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// Servo
#define servoPin 7
Servo steeringServo;
#define servoOffset 94
#define servoRange 30

// Other
float heading = 0;
unsigned long timer = 0;
#define heading_desired 0
#define Pgain 1.5
#define Igain 0.5
float errorInt = 0;
float gyrozbias = 0;

float gyroz();
int dtms();
int getPingDist();

void setup() {
  Serial.begin(115200);
  
  // Set up motors
  pinMode(motorPin,OUTPUT);
  //analogWrite(motorPin,0);
  analogWrite(motorPin, 0.5 * maxSpeed);
  
  // Set up IMU
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  if (!lsm.begin()) {
      while (1);
  }
  int nmeas = 1000;
  for (int i = 0; i < nmeas; i++) {
    gyrozbias += gyroz();
    delay(2);
  }
  gyrozbias /= nmeas;

  // Set up servo
  steeringServo.attach(servoPin);
  steeringServo.write(servoOffset);

  // Set up ultrasonic
  pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
  pinMode(pingTrigPin,OUTPUT);
  pinMode(pingEchoPin,INPUT);
}

void loop() {
  // Steering Control
  float dt = dtms() * 0.001;
  heading += (gyroz()-gyrozbias) * dt;
  if (heading > 180) {
    heading -= 360;
  }
  if (heading < -180) {
    heading += 360;
  }
  
  float error = heading_desired - heading;
  errorInt += error * dt;

  int steeringAngle = Pgain * error + Igain * errorInt;
  
  steeringServo.write(constrain(-steeringAngle+servoOffset,servoOffset-servoRange,servoOffset+servoRange));

  // Emergency Stop
  int dist = getPingDist();
  if (dist < 45) {
    analogWrite(motorPin, 0); // Stops car when it is within 30cm of an obstacle
  }
  else {
    analogWrite(motorPin, 0.4 * maxSpeed);
  }

  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("     Distance: ");
  Serial.print(dist);
  Serial.print("     Gyro Bias: ");
  Serial.print(gyrozbias);
  Serial.print("     GyroZ: ");
  Serial.println(gyroz());
}

int dtms() {
  unsigned long currtime = millis();
  unsigned long deltatime;
  if (currtime > timer) {
    deltatime = currtime - timer;
  }
  else {
    deltatime = currtime;
  }
  timer = currtime;
  return deltatime;
}

float gyroz() {
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  return g.gyro.z;
}

int getPingDist() {
  // Returns distance from ultrasonic in cm
  
  const long timeout_us = 3000; // 3000us = 51cm max range

  digitalWrite(pingTrigPin, LOW); // Off for 2us
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH); // On for 5us
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
 
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us); // time for signal to come back
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }

  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  int pingDist = constrain(0.017*echo_time,5.0,50.0); // convert time to distance

  return pingDist;
}
