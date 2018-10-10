#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

#define motorPin 8 // PWM for motor
#define maxSpeed 255

void setup() {
  Serial.begin(115200);  

  // Set up ultrasonic
  pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
  pinMode(pingTrigPin,OUTPUT);
  pinMode(pingEchoPin,INPUT);

  // Set up motors
  pinMode(motorPin,OUTPUT);
  analogWrite(motorPin,0);
}

void loop() {
  int dist = getPingDist();
  Serial.println(dist);
  if(dist < 45) {
    analogWrite(motorPin, 0); // Stops car when it is within 30cm of an obstacle
  }
  else {
    analogWrite(motorPin, 0.5 * maxSpeed);
  }
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
