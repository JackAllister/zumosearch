/*
 * Assignment 1
 * Zumo Search and Rescue Robot
 * 
 * Jack Allister - 23042098
 */
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <NewPing.h>

/********************** Module typedefs **********************/
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
  SEARCH,
  NONE
} MOVEMENT;

typedef struct
{
  OPERATING_MODE mode;
  MOVEMENT movement;
  unsigned long time;
  
  int roomID; /* -1 for corridor */
} MOVEMENT_COORD;

/********************** Module constants **********************/
#define LED_PIN 13
#define NUM_SENSORS 6

/* Ultrasonic sensor */
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define MAX_DISTANCE 20

#define MAX_COORDINATES 60
#define MAX_ROOMS 5
#define MAX_SPEED 120

/* Keypress characters */
#define CHAR_CALIBRATE '1'
#define CHAR_CHECK_ROOM '2'
#define CHAR_START_AUTONOMOUS '3'
#define CHAR_WALL_DETECT 'C'
#define CHAR_ROOM 'R'
#define CHAR_AUTONOMOUS 'E'
#define CHAR_FORWARD 'W'
#define CHAR_BACKWARD 'S'
#define CHAR_LEFT 'A'
#define CHAR_RIGHT 'D'
#define CHAR_STOP 0x20

/********************** Module variables **********************/

/* Variables relating to the robot for control */
ZumoMotors motors;
ZumoReflectanceSensorArray reflectanceSensors;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

/* Operating mode */
OPERATING_MODE robotMode = GUIDED_NAVIGATE;
MOVEMENT currMovement = NONE;
bool wallDetect = true;

/* counter for rooms, also an array of boolean to show object found */
int roomCount = 0;
bool roomFound[MAX_ROOMS] = {false, false, false, false, false};

/* Array of directions used for autonomous mode */
MOVEMENT_COORD movLog[MAX_COORDINATES];
int movLogCount = 0;

/********************** Module prototypes **********************/
void parseGuidedNavigate(char recv);
void parseSearchRoom(char recv);
bool parseMovement(char recv);

void runAutonomousMode();
MOVEMENT calcMovement(MOVEMENT_COORD mov);
void moveDirection(MOVEMENT movement);

bool checkForObject();
bool correctPath();
bool isWallFound();
int isSensorsOver(int startSensor, int endSensor);
void calibrateSensors();

/********************** Module code **********************/
void setup() 
{
  /* Initialise serial needed for wireless coms via XBee */
  Serial.begin(9600);

  /* Initialise reflectance sensors for wall detection */
  reflectanceSensors.init();

  /* Make sure the robot is not moving */
  motors.setSpeeds(0, 0);

  /* Initialise LED pin */
  pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
  char recvByte = 0;

  /* Check serial to see if we have received any commands */
  if (Serial.available() != 0)
  {
    /* Convert to upper case so command 'e' is same as 'E' */
    recvByte = toupper(Serial.read());
  }
  
  switch (robotMode)
  {
    case GUIDED_NAVIGATE:
    {
      if (recvByte != 0)
      {
        /* Check if received byte is a movement character */
        if (parseMovement(recvByte) == false)
        {
          /* If not equal to movement character parse other characters */
          parseGuidedNavigate(recvByte);
        }
      }

      if (currMovement == FORWARD)
      {
        if (wallDetect == true)
        {
          /* 
           * Wall detection and path correction only work if enabled and 
           * zumo going forward. This is because there is no point having
           * the zumo use its sensors if travelling backwards as none on
           * rear side.
           */
          if (isWallFound() == true)
          {
            moveDirection(NONE);
            Serial.println("Wall found, press C to re-enable when moved.");
            wallDetect = false;
          }
          else
          {
            /* Only correct the path if a wall has not been found */
            correctPath();
          }    
        }
      }        
      break;
    }

    case SEARCH_ROOM:
    {
      if (recvByte != 0)
      {
        /* Again parse movement and other room search mode commands */
        if (parseMovement(recvByte) == false)
        {
          parseSearchRoom(recvByte);
        }
      }
      break;
    }

    case AUTONOMOUS_NAVIGATE:
    {
      if (recvByte != 0)
      {
        /* Again parse movement and other autonomous commands (start/leavemode) */
        if (parseMovement(recvByte) == false)
        {
          parseAutonomous(recvByte);
        }
      }
      break;
    }
  }
  
}

void parseGuidedNavigate(char recv)
{
  switch (recv)
  {
    case CHAR_CALIBRATE:
    {
      calibrateSensors();
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

    case CHAR_ROOM:
    {
      robotMode = SEARCH_ROOM;
      roomCount++;
      
      Serial.print("Room search: ");
      Serial.println(roomCount);
      break;
    }

    case CHAR_AUTONOMOUS:
    {
      robotMode = AUTONOMOUS_NAVIGATE;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Autonomous mode started, rotate 180* then press 3");
      break;
    }
  }
}

void parseSearchRoom(char recv)
{ 
  switch (recv)
  {
    case CHAR_CHECK_ROOM:
    {
      checkForObject();
      break;
    }

    case CHAR_ROOM:
    {
      robotMode = GUIDED_NAVIGATE;
      Serial.println("Leaving search mode, going back to guided");
      break;
    }
  } 
}

void parseAutonomous(char recv)
{
  switch (recv)
  {
    case CHAR_START_AUTONOMOUS:
    {
      runAutonomousMode();
      break;
    }

    case CHAR_AUTONOMOUS:
    {
      robotMode = GUIDED_NAVIGATE;
      digitalWrite(LED_PIN, LOW);
      Serial.println("Leaving autonomous mode, going back to guided");
      break;
    }
  }
}

bool parseMovement(char recv)
{
  bool result = true;
  
  switch (recv)
  {
    case CHAR_FORWARD:
    {
      moveDirection(FORWARD);
      break;
    }

    case CHAR_BACKWARD:
    {
      moveDirection(BACKWARD);
      break;
    }

    case CHAR_LEFT:
    {
      moveDirection(LEFT);
      break;
    }

    case CHAR_RIGHT:
    {
      moveDirection(RIGHT);
      break;
    }

    case CHAR_STOP:
    {
      moveDirection(NONE);
      break;
    } 

    default:
    {
      /* 
       * If character does not match a movement key 
       * this function should return false.
       */
       result = false;
    }
  }

  return result;
}

void runAutonomousMode()
{  
  static const int MIN_LOG_TIME = 400;
  static const int MAX_LOG_TIME = 8000;
  
  int i = 0;
  MOVEMENT nextMovement;
  unsigned long startTime;

  for (i = movLogCount; i != -1; i--)
  {
    Serial.print("Running action: ");
    Serial.println(i);

    /* This if statement eliminates any possible 'empty' rooms from route back */
    if ((movLog[i].mode != SEARCH_ROOM) || (roomFound[movLog[i].roomID] == true))
    {
      /* Check to see if relates to a search or movmeent */
      if (movLog[i].movement != SEARCH)
      {
        if ((movLog[i].time >= MIN_LOG_TIME) &&
            (movLog[i].time <= MAX_LOG_TIME))
        {      
          nextMovement = calcMovement(movLog[i]);
        
          /* Move in inverse direction to backtrace */
          startTime = millis();
          moveDirection(nextMovement);
          while (millis() - startTime < movLog[i].time)
          {
            if (nextMovement == FORWARD)
            {     
              if (isWallFound() == false)
                correctPath();
              else
                break;
            }
          }
          motors.setSpeeds(0, 0);
        }
      }
      else
        checkForObject();
    }
  }
  moveDirection(NONE);
}

MOVEMENT calcMovement(MOVEMENT_COORD mov)
{
  MOVEMENT result = NONE;
  
  switch (mov.movement)
  {
    case FORWARD:
    {
      if (mov.mode == SEARCH_ROOM)
        result = BACKWARD;
      else
        result = FORWARD;
      break;
    }

    case BACKWARD:
    {
      if (mov.mode == SEARCH_ROOM)
        result = FORWARD;
      else
        result = BACKWARD;
      break;
    }

    case LEFT:
    {
      if (mov.mode == SEARCH_ROOM)
        result = LEFT;
      else
        result = RIGHT;
      break;
    }

    case RIGHT:
    {
      if (mov.mode == SEARCH_ROOM)
        result = RIGHT;
      else
        result = LEFT;
      break;
    }
  }

  return result;
}


void moveDirection(MOVEMENT movement)
{
  /* Method for storing movement coordinates */
  if (robotMode != AUTONOMOUS_NAVIGATE)
  {
    /* Set movement and start time */
    unsigned long currTime = millis();
    movLog[movLogCount].mode = robotMode;
    movLog[movLogCount].movement = movement;
    movLog[movLogCount].time = currTime;

    /* 
     * If we movement is in search mode we note down the room.
     * If not we just put -1 to signal it is a corridor
     */
    if (robotMode == SEARCH_ROOM)
      movLog[movLogCount].roomID = roomCount;
    else
      movLog[movLogCount].roomID = -1;

    /* Here we correct previous log time so that it is delta/diff */
    if (movLogCount != 0)
    {
      movLog[movLogCount-1].time = currTime - movLog[movLogCount-1].time;
    }
    movLogCount++;
  }

  /* Switch case for actually moving the robot from the relative command */
  switch (movement)
  {
    case FORWARD:
    {
      /* Set values needed for collision detection */
      currMovement = FORWARD;
      
      Serial.println("Robot moving forwards");
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
      break; 
    }

    case BACKWARD:
    {
      /* Set values needed for collision detection */
      currMovement = BACKWARD;
      
      Serial.println("Robot moving backwards");
      motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
      break;
    }

    case LEFT:
    {
      /* Set values needed for collision detection */
      currMovement = LEFT; 
      
      Serial.println("Robot turning left");
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
      break;
    }

    case RIGHT:
    {
      /* Set values needed for collision detection */
      currMovement = RIGHT;
      
      Serial.println("Robot turning right");
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
      break;
    }

    case NONE:
    {
      Serial.println("Robot stopping");
      motors.setSpeeds(0, 0);
    
      /* Set values needed for collision detection */
      currMovement = NONE;
      break;
    } 
  }
}

bool checkForObject()
{
  static const unsigned long TURN_TIME = 750;

  bool found = false;
  unsigned long startTime;
  unsigned long pingDist;
  
  Serial.println("Checking room for objects");
  /* Perform a quick scan of the room using US to find items */ 

  /* Check left direction */
  Serial.println("Checking left");
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    delay(50);
    pingDist = sonar.ping_cm();
    if ((pingDist != 0) && (found == false))
    {
      /* If we get here means object found in left of room */
      Serial.print("Object found in left of room ");
      Serial.print(pingDist);
      Serial.println(" cm away.");
      found = true;
    } 
  }

  /* Move right to midpoint */
  motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    /* Do nothing here as returning */
  }

  /* Check right direction */
  Serial.println("Checking right");
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    delay(50);
    pingDist = sonar.ping_cm();
    if ((pingDist != 0) && (found == false))
    {
      /* If we get here means object found in right of room */
      Serial.print("Object found in right of room ");
      Serial.print(pingDist);
      Serial.println(" cm away.");
      found = true;
    }
  }

  /* Move left to midpoint */
  motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
  startTime = millis();
  while ((millis() - startTime) < TURN_TIME)
  {
    /* Do nothing here as returning */
  }

  motors.setSpeeds(0, 0);
  if (found == false)
    Serial.println("No objects found.");

  /* Method for storing movement coordinates */
  if (robotMode != AUTONOMOUS_NAVIGATE)
  {
    /* Set movement and start time */
    unsigned long currTime = millis();
    movLog[movLogCount].mode = robotMode;
    movLog[movLogCount].movement = SEARCH;
    movLog[movLogCount].time = currTime;
    movLog[movLogCount].roomID = roomCount;

    /* Here we correct previous log time so that it is delta/diff */
    if (movLogCount != 0)
    {
      movLog[movLogCount-1].time = currTime - movLog[movLogCount-1].time;
    }
    movLogCount++;
  }

  roomFound[roomCount] = found;
  return found;
}

bool correctPath()
{
  static const unsigned long CORR_INTERVAL = 50;
  static const int LEFT_START = 0;
  static const int RIGHT_START = 5;

  static unsigned long lastRun = 0;
  bool result = false;
  MOVEMENT lastMovement;

  /* Corrects the path to stop a side from accidentally going on a wall */

  /* Correction happens every 50ms */
  if ((lastRun == 0) || (millis() - lastRun > CORR_INTERVAL))
  {
    lastRun = millis();

    /* Check to make sure only left sensor is on a line */
    if ((isSensorsOver(LEFT_START, LEFT_START) > 0) &&
        (isSensorsOver(RIGHT_START, RIGHT_START) == 0))
    {
      /* If true correct path by moving the robot right until not on line */
      motors.setSpeeds(MAX_SPEED, -MAX_SPEED);
      do
      {
        
      } while (isSensorsOver(LEFT_START, LEFT_START) > 0);
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
      
      Serial.println("Corrected left");
      result = true;
    }
    else if ((isSensorsOver(RIGHT_START, RIGHT_START) > 0) &&
             (isSensorsOver(LEFT_START, LEFT_START) == 0))
    {
      /* If true correct path by moving the robot left until not on line */
      motors.setSpeeds(-MAX_SPEED, MAX_SPEED);
      do
      {
        
      } while (isSensorsOver(RIGHT_START, RIGHT_START) > 0);
      motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  
      /* If far right sensor on a line correct path a little */
      Serial.println("Corrected right.");
      result = true;
    }
  }

  return result;
}

bool isWallFound()
{
  static const int START_SENSOR = 0;
  static const int END_SENSOR = 5;

  int sensorCount = isSensorsOver(START_SENSOR, END_SENSOR);

  /* Return true if more than one sensor detects a line */
  return (sensorCount > 1);
}

int isSensorsOver(int startSensor, int endSensor)
{
  static const int LINE_VALUE = 300;
  
  int sensorCount = 0;
  unsigned int sensorValues[NUM_SENSORS];
  int i;

  reflectanceSensors.readLine(sensorValues);
  
  for (i = startSensor; i <= endSensor; i++)
  {    
    if (sensorValues[i] >= LINE_VALUE)
      sensorCount++;
  }

  return sensorCount;
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

