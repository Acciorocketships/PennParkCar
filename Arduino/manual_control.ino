// Other Setup
struct meas {
  float x;
  float y;
  float z;
};
/*
typedef struct meas Measurement;
unsigned long timer;
float filterout;
float errorInt = 0;

#define headingDesired 0
#define Pgain 0.1
#define Igain 0


// IMU Setup
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#define LSM9DS1_XGCS 49
#define LSM9DS1_MCS 47
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
sensors_event_t a, m, g, temp;
Measurement accel;
Measurement gyro;
Measurement mag;
float psiIMU;
float gyrozbias;


// GPS Setup
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);
SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
}
float lat;
float lng;
float vel;
float psiGPS;
int satellites;
*/

// Ultrasonic
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

// Motor
#define motorPin 8
#define maxSpeed 255
#define motorSpeed 0.6

// Servo
#include <Servo.h>
#define servoPin 7
Servo steeringServo;
#define servoOffset 94
#define servoRange 30

//
// some useful stuff
// http://gammon.com.au/i2c
//

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

//
// global variables
// (yucky but needed to make i2c interrupts work)
//
byte piCommand;
byte piData[2];


void setup() {
  /*
  // IMU Initialization
  if (!lsm.begin()) {
      Serial.println("Unable to initialize the IMU");
      while (1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  int nmeas = 1000;
  for (int i = 0; i < nmeas; i++) {
    getIMU();
    gyrozbias += gyro.z;
    delay(2);
  }
  gyrozbias /= nmeas;

  // GPS Initialization
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  getGPS();
  */

  // Set up motors
  pinMode(motorPin,OUTPUT);
  //analogWrite(motorPin,0);
  analogWrite(motorPin, motorSpeed * maxSpeed*);

  // Set up ultrasonic
  pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
  pinMode(pingTrigPin,OUTPUT);
  pinMode(pingEchoPin,INPUT);

  // Set up servo
  steeringServo.attach(servoPin);
  steeringServo.write(servoOffset);

  // set up I2C
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveDataI2C);
  Wire.onRequest(sendDataI2C);

  // Other Initialization
  Serial.begin(115200);
  timer = millis();
  psiIMU = psiGPS;
  filterout = 0;
  
}

void loop() {
  /*
  // Get Data
  float dt = looptime();
  getGPS();
  getIMU();

  // Integrate psiIMU
  psiIMU += -(gyro.z-gyrozbias) * dt;
  
  // Filter
  float heading;
  if (vel > 0.36) {
    float wc = 1;
    float alpha = 1 / (1 + wc * dt);
    float filterin = rectifyAngle( psiGPS - psiIMU );
    filterout = alpha * filterout + (1-alpha) * filterin;
    heading = rectifyAngle(filterout + psiIMU);
  }
  else {
    heading = rectifyAngle(psiIMU);
  }


  // Calculate Error and Error Integral
  float error = headingDesired - heading;
  errorInt += error * dt;
  errorInt = constrain(errorInt,-10,10);

  // Control Law
  int steeringAngle = Pgain * error + Igain * errorInt;
  */


  // Steering
  steeringServo.write(constrain(steeringAngle+servoOffset,servoOffset-servoRange,servoOffset+servoRange));

  // Emergency Stop
  int dist = getPingDist();
  if (dist < 45) {
    analogWrite(motorPin, 0); // Stops car when it is within 30cm of an obstacle
  }
  else {
    analogWrite(motorPin, motorSpeed * maxSpeed);
  }
  Serial.print(piData[0]); Serial.print(" "); Serial.println(piData[1]);
  /*
  // Print
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("    PsiIMU: ");
  Serial.print(psiIMU);
  Serial.print("    PsiGPS: ");
  Serial.print(psiGPS);
  Serial.print("    Velocity: ");
  Serial.print(vel);
  Serial.print("    Timer: ");
  Serial.println(timer);
  */
}

/*
void getGPS() {
  if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
      lat = (GPS.latitudeDegrees - 40.0);
      lng = (GPS.longitudeDegrees + 75.0);
      vel = GPS.speed;
      psiGPS = rectifyAngle(GPS.angle);
      satellites = GPS.satellites;
  } 
}

void getIMU() {
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp);
  accel.x = a.acceleration.x;
  accel.y = a.acceleration.y;
  accel.z = a.acceleration.z;
  mag.x = m.magnetic.x;
  mag.y = m.magnetic.y;
  mag.z = m.magnetic.z;
  gyro.x = g.gyro.x;
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;
}

float looptime() {
    float dt = millis() - timer;
    timer = millis();
    return dt * 0.001;
}

float rectifyAngle(float angle) {
  while (angle < -180) {
    angle += 360;
  }
  while (angle > 180) {
    angle -= 360;
  }
  return angle;
}
*/

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

void receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      if (piCommand == 255) {
          piData[0] = Wire.read();
          piData[1] = Wire.read();
      }
      //
      // now clear the buffer, just in case
      //
      while (Wire.available()) {Wire.read();}
}
