/*
 * Assignment 1
 * Zumo Search and Rescue Robot
 * 
 * Jack Allister - 23042098
 */
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

/* Module typedefs */
typedef enum
{
  GUIDED_NAVIGATE,
  SEARCH_ROOM,
  AUTONOMOUS_NAVIGATE
} OPERATING_MODE;

/* Module constants */
#define NUM_SENSORS 6

static const char CALIBRATE = 'C';
static const char FORWARD = 'W';
static const char BACKWARD = 'S';
static const char LEFT = 'A';
static const char RIGHT = 'D';
static const char STOP_ROBOT = 0x20;

static const int MAX_SPEED = 100;

/* Module variables */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;

bool isMoving = false;

OPERATING_MODE robotMode = GUIDED_NAVIGATE;

/* Module prototypes */
void parseGuidedNavigate();
bool isWallFound();
void robotForward();
void robotBackward();
void turnLeft();
void turnRight();
void robotStop();
void calibrateSensors();

/* Module code */
void setup() 
{
  Serial.begin(9600);

  reflectanceSensors.init();

  /* Make sure the robot is not moving */
  motors.setSpeeds(0, 0);
}

void loop() 
{
  switch (robotMode)
  {
    case GUIDED_NAVIGATE:
    {
      parseGuidedNavigate();

      if (isMoving && (isWallFound() == true))
      {
        robotStop();
        Serial.println("Wall found");
      }
        
      break;
    }

    case SEARCH_ROOM:
    {
      break;
    }

    case AUTONOMOUS_NAVIGATE:
    {
      break;
    }
  }
  
}

void parseGuidedNavigate()
{
  if (Serial.available() > 0)
  {
    char recvByte = toupper(Serial.read());

    switch (recvByte)
    {
      case CALIBRATE:
      {
        calibrateSensors();
        break;
      }

      case FORWARD:
      {
        robotForward();
        break;
      }

      case BACKWARD:
      {
        robotBackward();
        break;
      }

      case LEFT:
      {
        turnLeft();
        break;
      }

      case RIGHT:
      {
        turnRight();
        break;
      }

      case STOP_ROBOT:
      {
        robotStop();
        break;
      } 
    }
  }
}

bool isWallFound()
{
  static const int LINE_VALUE = 650;
  
  unsigned int sensorValues[NUM_SENSORS];
  int i;
  bool result = true;

  reflectanceSensors.readLine(sensorValues);

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.println();

  /* Check sensors 1-4 to see if wall detected */
  for (i = 1; i < 4; i++)
  {
    if (sensorValues[i] <= LINE_VALUE)
    {
      result = false;
    }
  }

  return result;
}

void robotForward()
{
  Serial.println("Robot moving forwards");
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  isMoving = true;
}

void robotBackward()
{
  Serial.println("Robot moving backwards");
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
  isMoving = true;
}

void turnLeft()
{
  Serial.println("Robot turning left");
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  isMoving = true;
}

void turnRight()
{
  Serial.println("Robot turning right");
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
  isMoving = true;
}

void robotStop()
{
  Serial.println("Robot stopping");
  motors.setSpeeds(0, 0);
  isMoving = false;
}

void calibrateSensors()
{
  static const unsigned long MOVE_TIME = 350;
  
  char dummy;
  int i;

  Serial.println("Calibrating sensors");
  Serial.println("Move the robot so that the sensors are just before a line/wall");
  Serial.println("Press space to continue");
  while (Serial.available() == 0)
  {
  
  }
  dummy = Serial.read();

  for (i = 0; i < 10; i++)
  {
    unsigned long startTime = millis();
    motors.setSpeeds(MAX_SPEED, MAX_SPEED);
    while (millis() - startTime < MOVE_TIME)
    {
      reflectanceSensors.calibrate();
    }

    startTime = millis();
    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
    while (millis() - startTime < MOVE_TIME)
    {
      reflectanceSensors.calibrate();
    }
  }
  motors.setSpeeds(0, 0);

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("Minimum values: ");
    Serial.print(reflectanceSensors.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print("Maximum values: ");
    Serial.print(reflectanceSensors.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Sensor calibration complete!");
}

