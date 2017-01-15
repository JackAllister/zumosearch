/**
 * @file robotcode.ino
 * @author Jack Allister - b3042098
 * @date 15 Jan 2017
 * @brief Zumo Search and Rescue Robot using Arduino Uno
 *
 * Implementation of the robot code for task 1-6 of assignment 1.
 * Allows manual control, room searching and autonomous navigation.
 */
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <NewPing.h>

/********************** Module typedefs **********************/

/**
 * @brief enum for which operating 'mode' the robot is in
 *
 * Robot can only be in one mode at a time.\n
 * GUIDED_NAVIGATE - Robot controlled wirelessly from a computer via XBee.\n
 * SEARCH_ROOM - Mode for searching a room, allows use of ultrasonic sensor.\n
 * AUTONOMOUS_NAVIGATE - Mode uses stored movements and actions to return to start.\n
 */
typedef enum
{
  GUIDED_NAVIGATE,
  SEARCH_ROOM,
  AUTONOMOUS_NAVIGATE
} OPERATING_MODE;

/**
 * @brief enum for which movement/action is taking place
 *
 * Robot can be doing 1 of 6 possible movements at a time.\n
 * FORWARD, BACKWARD, LEFT, RIGHT\n
 * Searching or no action/movement.
 */
typedef enum
{
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  SEARCH,
  NONE
} MOVEMENT;

/**
 * @brief Structure that is used for logging of actions
 *
 * Stores the current operating mode of the robot when logged.\n
 * Movement so that the movement can be replicated in autonomous.\n
 * The overall time that this action is taking place for.\n
 * The current roomID/number, if in a corridor stored as -1.
 */
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

/**
 * @brief Runs once at boot of arduino. Responsible for settings up peripherals
 *
 * Responsible for setting up everything needed for the peripherals.\n
 * Xbee, reflectance sensors, motors, LED pin.
 */
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

/**
 * @brief System loop that is ran continuously.
 *
 * This loop has been designed to be as non-blocking as possible.\n
 * This allows data from the XBee to be parsed as quickly as possible.\n
 * \n
 * A state machine has been included within here to process action relating
 * to the current operating mode that the robot is in (see OPERATING_MODE).
 */
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

/**
 * @brief Responsible for parsing special commands in GUIDED_NAVIGATE mode.
 *
 * This procedure is responsible for parsing commands only needed while in
 * GUIDED_NAVIGATE mode. For example enabling wall detect, calibration,
 * switching to SEARCH_ROOM mode etc.
 *
 * @param recv - Received command byte from serial/XBee.
 */
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

/**
 * @brief Responsible for parsing special commands in SEARCH_ROOM mode
 *
 * This procedure is responsible for parsing commands only used in SEARCH_ROOM
 * operating mode. For example starting a room search for objects and switching
 * out of SEARCH_ROOM back to GUIDED_NAVIGATE.
 *
 * @param recv - Received command byte from serial/XBee.
 */
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

/**
 * @brief Responsible for parsing special commands in AUTONOMOUS_NAVIGATE mode
 *
 * This procedure is responsible for parsing commands only used in AUTONOMOUS_NAVIGATE
 * operating mode. For example starting autonomos navigation and returning
 * back to GUIDED_NAVIGATE mode.
 *
 * @param recv - Received command byte from serial/XBee.
 */
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

/**
 * @brief Responsible for parsing movement commands in any mode
 *
 * This procedure is responsible for parsing movement commands used in any
 * operating mode. These commands however WILL NOT work while calibration
 * is taking place or the autnomous navigation is running through logged
 * movements.\n
 * If the received command is not a movement command the function returns false,
 * This is to show the parent function that the command may be a special mode
 * dependent command.
 *
 * @param recv - Received command byte from serial/XBee.
 * @return boolean - Whether a valid movement command was found.
 */
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

/**
 * @brief Runs the logged movements/actions stored from guided mode.
 *
 * This function allows the robot to navigate back to the start of the building
 * blueprint.\n
 * Actions/movements that take less than 400ms or more than 8000ms are ignored.
 * This is to try and reduce the amount of redundant actions taking place and
 * speed up the overall process of autnomous searching and returning back to
 * the start.\n
 * Actions are stored in the movLog array which is of type MOVEMENT_COORD.
 *
 */
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

/**
 * @brief Calculates the optimised/needed movement needed in autonomos mode.
 *
 * The optimised/needed movement is calculated using a switch case.\n
 * Because movements are parsed in a reverse order in runAutonomousMode we needed
 * to inverse some commands for example movements/actions taken place in
 * SEARCH_ROOM mode.
 *
 * @param mov - MOVEMENT_COORD structure/object from movLog to be used in calculation.
 * @return MOVEMENT - the optimised movement for the robot to take.
 */
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

/**
 * @brief Moves the robot in a certain direction
 *
 * This function is responsible for moving the robot in a certain direction.
 * The passed in parameter is either passed in via Serial/Xbee or from the
 * autonomous navigation loop.\n
 * Logging of movements also takes place when in SEARCH_ROOM or GUIDED_NAVIGATE
 * mode, this allows AUTONOMOUS_NAVIGATE mode to use the logged data for it's
 * return path.
 *
 * @param movement - The direction that the robot needs to go.
 */
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

/**
 * @brief Uses the ultrasonic sensor to search the room for an object.
 *
 * Robot rotates left while the ultrasonic sensor is on to try and find objects.
 * The robot then returns right while doing the same thing.
 * Maximum distance for the ultrasonic sensor is set at 20cm using the constant
 * MAX_DISTANCE this can be changed to allow up to a maximum of 200cm.
 *
 * @return boolean - Whether an object was discovered.
 */
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

/**
 * @brief procedure for path correction to stop wall collisions
 *
 * Path correction that stops either the left or right side of the robot from
 * accidentally driving over a wall boundary. This function uses the reflectance
 * sensors to do so.\n
 * Path correction can only happen once every 50ms.
 *
 * @return boolean - Whether path correction has taken place.
 */
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

/**
 * @brief Checks to see if a wall is found using reflectance sensors.
 *
 * Procedure checks to see if more than one sensor is on a wall. If so the
 * the function returns true.
 *
 * @return boolean - Returns if the robot is on a wall.
 */
bool isWallFound()
{
  static const int START_SENSOR = 0;
  static const int END_SENSOR = 5;

  int sensorCount = isSensorsOver(START_SENSOR, END_SENSOR);

  /* Return true if more than one sensor detects a line */
  return (sensorCount > 1);
}

/**
 * @brief Function to check how many sensors are on a wall.
 *
 * Checks to see how many sensors are on a wall. Function has the ability to
 * only check certain sensors, this is done by the passed in parameters.
 *
 * @param startSensor - The first sensor to check.
 * @param endSensor - The last sensor to check (endSensor > startSensor).
 * @return int - The number of sensors that are on a wall.
 */
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

/**
 * @brief Calibrates the reflectance sensors on the robot.
 *
 * Procedure calibrates the reflectance sensors by moving the robot backwards
 * and forwards over a wall boundary.\n
 * This allows accurate wall detection.
 */
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
