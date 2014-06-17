#include <Servo.h>
#include <NewPing.h>

#define MOTOR_A_FORWARD 12
#define MOTOR_A_BREAK 9
#define MOTOR_A_POWER 3
#define MOTOR_B_FORWARD 13
#define MOTOR_B_BREAK 8
#define MOTOR_B_POWER 11

#define PING_TRIGGER_PIN 4
#define PING_ECHO_PIN 7
#define PING_MAX_DISTANCE 200
#define SERVO_PIN 5

long lastPingCheck = 0;
long pingCheckInterval = 150;
boolean canGoForward = true;
int minimumForwardDistance = 10;
Servo pingServo;

NewPing sonar(PING_TRIGGER_PIN, PING_ECHO_PIN, PING_MAX_DISTANCE);

void setup() {
  pinMode(MOTOR_A_FORWARD, OUTPUT); //Initiates Motor Channel B pin
  pinMode(MOTOR_A_BREAK, OUTPUT);  //Initiates Brake Channel B pin
  pinMode(MOTOR_A_POWER, OUTPUT);  //Initiates Power Channel B pin
  pinMode(MOTOR_B_FORWARD, OUTPUT); //Initiates Motor Channel B pin
  pinMode(MOTOR_B_BREAK, OUTPUT);  //Initiates Brake Channel B pin
  pinMode(MOTOR_B_POWER, OUTPUT);  //Initiates Power Channel B pin

  Serial.begin(9600);
  
  pingServo.attach(SERVO_PIN);

  // Setup motion check
  pingServo.write(60);
  delay(2000);
  pingServo.write(100);
  delay(2000);
  pingServo.write(40);
  delay(2000);
  pingServo.write(60);
  
  moveForward(150);
}

void loop() {
  unsigned long currentTime = millis();
  boolean forwardCheck = canGoForward;
  unsigned int pingDelay = 0;
  unsigned int distanceAhead = 0;

  if (currentTime - lastPingCheck > pingCheckInterval) {
    lastPingCheck = currentTime;
    pingDelay = sonar.ping();
    
    if (pingDelay > 0) {
      distanceAhead = pingDelay / US_ROUNDTRIP_CM;
      forwardCheck = (distanceAhead > minimumForwardDistance);
    }
  }
  
  if (forwardCheck != canGoForward) {
    Serial.print(">>> POSSIBLE OBSTACLE AT DISTANCE: ");
    Serial.print(distanceAhead);
    Serial.print(" WITH PING DELAY: ");
    Serial.println(pingDelay);

    canGoForward = forwardCheck;
    
    if (canGoForward) {
      moveForward(150);
    } else {
      fullStop();
      checkPeriphery();
    }
  }
}

void moveForward(int motorSpeed) {
  Serial.println(">>> MOVE FORWARD");
  digitalWrite(MOTOR_A_FORWARD, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_A_BREAK, LOW);   //Disengage the Brake for Channel A
  digitalWrite(MOTOR_B_FORWARD, HIGH); //Establishes forward direction of Channel B
  digitalWrite(MOTOR_B_BREAK, LOW);   //Disengage the Brake for Channel B
  analogWrite(MOTOR_A_POWER, motorSpeed);   //Spins the motor on Channel A
  analogWrite(MOTOR_B_POWER, motorSpeed);   //Spins the motor on Channel B
}

void fullStop() {
  digitalWrite(MOTOR_A_BREAK, HIGH);  //Engage the Brake for Channel A
  digitalWrite(MOTOR_B_BREAK, HIGH);  //Engage the Brake for Channel B
}

void checkPeriphery() {
  Serial.println(">>> CHECKING PERIPHERY");
  pingServo.write(40);

  delay(2000);

  unsigned int leftPingDelay = sonar.ping();
  unsigned int leftDistanceAhead = leftPingDelay / US_ROUNDTRIP_CM;
  
  Serial.print("LEFT DISTANCE: ");
  Serial.println(leftDistanceAhead);
  
  pingServo.write(100);
  
  delay(2000);

  unsigned int rightPingDelay = sonar.ping();
  unsigned int rightDistanceAhead = rightPingDelay / US_ROUNDTRIP_CM;
  
  Serial.print("RIGHT DISTANCE: ");
  Serial.println(rightDistanceAhead);

  pingServo.write(60);
  
  if (leftDistanceAhead < rightDistanceAhead) {
    digitalWrite(MOTOR_A_FORWARD, LOW); //Establishes forward direction of Channel A
    digitalWrite(MOTOR_A_BREAK, LOW);   //Disengage the Brake for Channel A
    analogWrite(MOTOR_A_POWER, 150);   //Spins the motor on Channel A
  } else {
    digitalWrite(MOTOR_B_FORWARD, LOW); //Establishes forward direction of Channel B
    digitalWrite(MOTOR_B_BREAK, LOW);   //Disengage the Brake for Channel B
    analogWrite(MOTOR_B_POWER, 150);   //Spins the motor on Channel B
  }
  
  delay(1000);
  
  fullStop();
  
  canGoForward = true;
  
  moveForward(150);
}

