/*
 * Assignment 1
 * Zumo Search and Rescue Robot
 * 
 * Jack Allister - 23042098
 */
#include <ZumoMotors.h>

/* Module Constants */
static const char CALIBRATION = 'C';
static const char FORWARD = 'W';
static const char LEFT = 'A';
static const char RIGHT = 'D';
static const char BACKWARE = 'S';

static const int MAX_SPEED = 100;

/* Module variables */
ZumoMotors motors;

unsigned long calLeftTime = 0;
unsigned long calRightTime = 0;

/* Module Prototypes */
void calibrateRobot();
void turnLeft();
void turnRight();

/* Module code */
void setup() {
  Serial.begin(9600);

  /* Make sure the robot is not moving */
  motors.setSpeeds(0, 0);
}

void loop() {

  if (Serial.available() > 0)
  {
    char recvByte = toupper(Serial.read());

    switch (recvByte)
    {
      case CALIBRATION:
      {
        calibrateRobot();
        break;
      }
    
    }
  }
}

void calibrateRobot()
{
  static const unsigned long WAIT_TIME = 500;
  
  unsigned long startTime;
  unsigned long endTime;
  char dummy;
  
  Serial.println("Calibration mode");

  /* 
   *  This section calculates how long it takes to turn 90 degress, this is used
   *  so that we have accurate turning for the arduino.
   */
  Serial.println("Send space once then again, when the robot has turned 90* left");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();
  delay(WAIT_TIME);
  
  startTime = millis();
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);

  /* Wait until the second press */
  while (Serial.available() == 0)
  {
    
  }
  dummy = Serial.read();
  endTime = millis();
  motors.setSpeeds(0, 0);

  /* Below is the time it takes to turn 90 degress left */
  calLeftTime = endTime - startTime;
  Serial.print("Turn left time: ");
  Serial.println(calLeftTime);
  
  /* Now we do the same again but for right turns */
  Serial.println("Send space once then again, when the robot has turned 90* right");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();
  delay(WAIT_TIME);
  
  startTime = millis();
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);

  /* Wait until the second press */
  while (Serial.available() == 0)
  {
    
  }
  dummy = Serial.read();
  endTime = millis();
  motors.setSpeeds(0, 0);

  /* Below is the time it takes to turn 90 degress left */
  calRightTime = endTime - startTime;
  Serial.print("Turn right time: ");
  Serial.println(calRightTime);

  Serial.println("Calibration complete!");
}

void turnLeft()
{
  /* Turns the robot left 90 degress */
}

void turnRight()
{
  /* Turns the robot right 90 degress */
}


