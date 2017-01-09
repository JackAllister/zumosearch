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

/* Module variables */
ZumoMotors motors;

/* Module Prototypes */
void calibrateRobot();

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

}

