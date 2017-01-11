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

typedef enum
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  STOPPED
} CURRENT_MOVEMENT;

/* Module constants */
#define NUM_SENSORS 6

static const char CHAR_CALIBRATE = '1';
static const char CHAR_WALL_DETECT = 'C';
static const char CHAR_FORWARD = 'W';
static const char CHAR_BACKWARD = 'S';
static const char CHAR_LEFT = 'A';
static const char CHAR_RIGHT = 'D';
static const char CHAR_STOP = 0x20;

static const int MAX_SPEED = 120;

/* Module variables */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;

OPERATING_MODE robotMode = GUIDED_NAVIGATE;
CURRENT_MOVEMENT currMovement = STOPPED;
bool wallDetect = true;

/* Speeds used for path correction */
int lastLeftSpeed = 0;
int lastRightSpeed = 0;

/* Module prototypes */
void parseGuidedNavigate();
bool correctPath();
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

      if (currMovement == FORWARD)
      {
        if (wallDetect == true)
        {
          if (isWallFound() == true)
          {
            robotStop();
            Serial.println("Wall found, press C to re-enable when moved.");
            wallDetect = false;
          }
          else if (correctPath() == true)
          {
            /* Only correct the path if a wall has not been found */
            Serial.print("Path corrected, wheels speeds: ");
            Serial.print(lastLeftSpeed);
            Serial.print(' ');
            Serial.println(lastRightSpeed);
          } 
        }
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
      case CHAR_CALIBRATE:
      {
        calibrateSensors();
        break;
      }

      case CHAR_FORWARD:
      {
        robotForward();
        break;
      }

      case CHAR_BACKWARD:
      {
        robotBackward();
        break;
      }

      case CHAR_LEFT:
      {
        turnLeft();
        break;
      }

      case CHAR_RIGHT:
      {
        turnRight();
        break;
      }

      case CHAR_STOP:
      {
        robotStop();
        break;
      } 

      case CHAR_WALL_DETECT:
      {
        if (wallDetect == false)
        {
          wallDetect = true;
          Serial.println("Wall detect enabled");
        }
        else
        {
          Serial.println("Wall detect already enabled");
        }
        break;
      }
    }
  }
}

bool correctPath()
{
  static const int LEFT_SENSOR = 0;
  static const int RIGHT_SENSOR = 5;
  static const int LINE_VALUE = 400;

  unsigned int sensorValues[NUM_SENSORS];
  bool result = false;

  /* Corrects the path to stop a side from accidentally going on a wall */

  /* Correction can only happen once per second */
  reflectanceSensors.readLine(sensorValues);
  
  if (sensorValues[LEFT_SENSOR] >= LINE_VALUE)
  {
    motors.setSpeeds(0, 0);
    while (sensorValues[LEFT_SENSOR] >= LINE_VALUE)
    {
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
      
      reflectanceSensors.readLine(sensorValues);
    }
    motors.setSpeeds(lastLeftSpeed, lastRightSpeed);
    
    /* If far left sensor on a line correct path a little */
    Serial.print("Corrected left: ");
    Serial.print(lastLeftSpeed);
    Serial.print(" ");
    Serial.println(lastRightSpeed);
    result = true;
  }
  else if (sensorValues[RIGHT_SENSOR] >= LINE_VALUE)
  {
    motors.setSpeeds(0, 0);
    while (sensorValues[RIGHT_SENSOR] >= LINE_VALUE)
    {
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
      
      reflectanceSensors.readLine(sensorValues);
    }
    motors.setSpeeds(lastLeftSpeed, lastRightSpeed);

    /* If far right sensor on a line correct path a little */
    Serial.print("Corrected right: ");
    Serial.print(lastLeftSpeed);
    Serial.print(" ");
    Serial.println(lastRightSpeed);
    result = true;
  }

  return result;
}

bool isWallFound()
{
  static const int LINE_VALUE = 400;
  
  unsigned int sensorValues[NUM_SENSORS];
  int i;
  bool result = false;

  reflectanceSensors.readLine(sensorValues);

//  for (i = 0; i < NUM_SENSORS; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print(' ');
//  }
//  Serial.println();

  /* Check sensors 1-4 to see if wall detected */
  for (i = 1; i < 4; i++)
  {
    if (sensorValues[i] >= LINE_VALUE)
    {
      result = true;
    }
  }

  return result;
}

void robotForward()
{
  Serial.println("Robot moving forwards");
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = FORWARD;
  lastLeftSpeed = MAX_SPEED;
  lastRightSpeed = MAX_SPEED; 
}

void robotBackward()
{
  Serial.println("Robot moving backwards");
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = BACKWARD;
  lastLeftSpeed = -MAX_SPEED;
  lastRightSpeed = -MAX_SPEED; 
}

void turnLeft()
{
  Serial.println("Robot turning left");
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);

  /* Set values needed for collision detection */
  currMovement = LEFT;
  lastLeftSpeed = -MAX_SPEED;
  lastRightSpeed = MAX_SPEED; 
}

void turnRight()
{
  Serial.println("Robot turning right");
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);

  /* Set values needed for collision detection */
    currMovement = RIGHT;
  lastLeftSpeed = MAX_SPEED;
  lastRightSpeed = -MAX_SPEED; 
}

void robotStop()
{
  Serial.println("Robot stopping");
  motors.setSpeeds(0, 0);

  /* Set values needed for collision detection */
  currMovement = STOPPED;
  lastLeftSpeed = 0;
  lastRightSpeed = 0; 
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

