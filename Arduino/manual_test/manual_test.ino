// Other Setup
struct meas {
  float x;
  float y;
  float z;
};

typedef struct meas Measurement;
unsigned long timer;
float filterout;

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

//I2C
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
byte piCommand;
byte piData[5];


float steeringAngle = 0;
float velocity = 0;


void setup() {
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

  
  // Set up motors
  pinMode(motorPin,OUTPUT);
  analogWrite(motorPin,0);
  
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
  getGPS();
  getIMU();
  // Steering

  steeringServo.write(constrain(steeringAngle+servoOffset,servoOffset-servoRange,servoOffset+servoRange));


  // Emergency Stop
  int dist = getPingDist();
  if (dist < 45) {
    analogWrite(motorPin, 0); // Stops car when it is within 30cm of an obstacle
  }
  else {
    analogWrite(motorPin, motorSpeed * maxSpeed * velocity);
  }
  

}

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
float read_float(){
  byte data[4];
  data[0] = Wire.read();
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read();
  float f =  *(float *) &data[0];
  return f;
}

void sendDataI2C(void) {
    if (piCommand == 1) {
        float dataBuffer[6];
        dataBuffer[0] = lat;
        dataBuffer[1] = lng;
        dataBuffer[2] = vel;
        dataBuffer[3] = gyro.x;
        dataBuffer[4] = gyro.y;
        dataBuffer[5] = gyro.z - gyrozbias;
        Wire.write((byte*) &dataBuffer[0], 6*sizeof(float));
        Serial.println("sending floats");
    }
    //not using byte mode
    else if (piCommand == 2) {
        byte dataBuffer[4];
        dataBuffer[0] = 1/ 256;
        dataBuffer[1] = (1 - 256*dataBuffer[0]);
        dataBuffer[2] = 1;
        dataBuffer[3] = 1;
        Wire.write(&dataBuffer[0], 4);
        Serial.println("sending bytes");
    }

}

void receiveDataI2C(int nPoints) {
      piCommand = Wire.read();
      //
      // if Pi is sending data, parse it into incoming data array
      //
      if (piCommand == 255) {
          piData[0] = Wire.read();
          if(piData[0] == true){
            float desHeading = read_float();
            float estHeading = read_float();
            float desSpeed = read_float();
            float estSpeed = read_float();
            velocity =  0.9 * desSpeed;
            steeringAngle = 0.5 * rectifyAngle(desHeading - estHeading);
          }
          else if(piData[0] = false){
            Serial.println("Turn on Manual");
            //read commands for auto control
          }
      }
      //
      // now clear the buffer, just in case
      //
      while (Wire.available()) {Wire.read();}
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
