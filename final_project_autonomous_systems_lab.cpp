#include <Wire.h>
#include <Zumo32U4.h>
#include "ZumoController.h"

const uint16_t maxSpeed = 400;

// Hardware modules
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proximitySensors;

// Time and control variables
#define SAMPLERATE 10
unsigned long lastMillis = 0;
float batteryVoltage = 0;
unsigned long lastMicros = 0;
int16_t lastError = 0;
uint32_t lastCheckTime = 0;
int16_t lineFound = 0;
bool endofCourse = false;
bool firstObstacle = false;
bool pointToPointStarted = false;
uint8_t frontSensor;
bool secondObstacle = false;

String control_state = "LineFollow";

// Target point for point-to-point controller
float desired_path[][2] = {
  {1.22, 0.55}
};
int num_points = sizeof(desired_path) / sizeof(desired_path[0]);

// Controller instance
ZumoController zumoController;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
const uint8_t OBSTACLE_THRESHOLD = 6;

// Print bar for sensor visualization
void printlnlnBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  Serial.println(barChars[height]);
}

// Calibrate line sensors by rotating left and right
void calibrateSensors()
{
  delay(1000);
  for (uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
      motors.setSpeeds(-200, 200);  // Rotate left
    else
      motors.setSpeeds(200, -200);  // Rotate right

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Show calibrated sensor readings once
void showReadings()
{
  lineSensors.readCalibrated(lineSensorValues);
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
    printlnlnBar(barHeight);
  }
  delay(1000);
}

// Executes point-to-point navigation after line-following is done
void pointToPointRoad()
{
  Serial.println("ponttopointstart");

  // Reset state and odometry
  zumoController.motorsSetSpeed(0, 0);
  zumoController.reset();
  delay(2500);
  zumoController.car_state.posx = 0.0;
  zumoController.car_state.posy = 0.0;
  zumoController.car_state.theta = 0.0;

  while (true)
  {
    proximitySensors.read();
    frontSensor = proximitySensors.countsFrontWithLeftLeds();

    // Avoid dynamic obstacle in P2P phase
    if (frontSensor >= OBSTACLE_THRESHOLD && !secondObstacle)
    {
      Serial.println("Avoiding Dynamic");
      secondObstacle = true;

      motors.setSpeeds(-250, 250); delay(500);  // Turn left
      motors.setSpeeds(250, 250);  delay(800);  // Move forward
      motors.setSpeeds(250, -250); delay(200);  // Turn right
      motors.setSpeeds(250, 250);  delay(700);  // Go forward again

      zumoController.motorsSetSpeed(0, 0);
      delay(2500);
    }

    // Update timing
    unsigned long dtMicros = micros() - lastMicros;
    lastMicros = micros();
    zumoController.dt_time = dtMicros / 1000000.0f;

    // Odometry and control
    zumoController.odometry();
    zumoController.P2P_CTRL(desired_path, num_points);

    // Logging current state
    Serial.print(millis());
    Serial.print(",");
    Serial.print(lastError);
    Serial.print(",");
    Serial.print(zumoController.car_state.posx, 2);
    Serial.print(" , ");
    Serial.print(zumoController.car_state.posy, 2);
    Serial.print(" , ");
    Serial.print(zumoController.path_state.v_forward * 100.0, 2);
    Serial.print(",");
    Serial.println("PathControl");
    Serial.println(batteryVoltage, 2);

    // Stop if goal reached
    if (zumoController.path_state.v_forward < 0.02f && zumoController.path_state.dist < 0.05f)
    {
      zumoController.motorsSetSpeed(0, 0);
      Serial.println("Reached goal!");
      break;
    }

    delay(50);  // 20 Hz loop
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Time_ms,Error,PosX_cm,PosY_cm,Velocity_cmPerSec,ControlState");

  lastMillis = millis();
  lastMicros = micros();

  lineSensors.initFiveSensors();
  proximitySensors.initFrontSensor();
  buzzer.play(">g32>>c32");

  Serial.println("Calibrating");
  calibrateSensors();
  showReadings();

  Serial.println("Go!");
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying());
}

void loop()
{
  if (!endofCourse)
  {
    proximitySensors.read();
    frontSensor = proximitySensors.countsFrontWithLeftLeds();

    // Handle first obstacle in line-follow mode
    if (frontSensor >= OBSTACLE_THRESHOLD)
    {
      firstObstacle = true;

      zumoController.odometry();
      Serial.print(millis()); Serial.print(",");
      Serial.print(0); Serial.print(",");
      Serial.print(zumoController.car_state.posx, 2); Serial.print(",");
      Serial.print(zumoController.car_state.posy, 2); Serial.print(",");
      Serial.print(0.0); Serial.print(",");
      Serial.println("AvoidObstacle_Start");

      // Maneuver around obstacle
      motors.setSpeeds(-250, 250); delay(550);
      motors.setSpeeds(250, 250);  delay(700);
      motors.setSpeeds(200, -250); delay(350);
      motors.setSpeeds(250, 250);

      // Search for line
      while (true)
      {
        int16_t pos = lineSensors.readLine(lineSensorValues);
        zumoController.odometry();
        Serial.print(millis()); Serial.print(",");
        Serial.print(0); Serial.print(",");
        Serial.print(zumoController.car_state.posx, 2); Serial.print(",");
        Serial.print(zumoController.car_state.posy, 2); Serial.print(",");
        Serial.print(0.0); Serial.print(",");
        Serial.println("AvoidObstacle_SearchLine");

        if (pos > 1000 && pos < 3000)
        {
          motors.setSpeeds(0, 0);
          break;
        }
      }

      Serial.println("Line found");
      delay(300);
    }

    // Check if end of line is reached
    if (firstObstacle)
    {
      lineSensors.readCalibrated(lineSensorValues);
      int16_t sumOfsensors = 0;
      for (uint8_t i = 0; i < NUM_SENSORS; i++)
        sumOfsensors += lineSensorValues[i];

      if (sumOfsensors >= 3500)
      {
        if (lineFound == 0)
          lineFound++;
        else
        {
          Serial.println("We are at the end!!!");
          motors.setSpeeds(0, 0);
          delay(500);
          endofCourse = true;
          pointToPointRoad();  // Switch to P2P mode
          return;
        }
      }
    }

    // Line following behavior
    int16_t position = lineSensors.readLine(lineSensorValues);
    int16_t error = position - 2000;

    zumoController.odometry();
    Serial.print(millis()); Serial.print(",");
    Serial.print(error); Serial.print(",");
    Serial.print(zumoController.car_state.posx, 2); Serial.print(",");
    Serial.print(zumoController.car_state.posy, 2); Serial.print(",");
    Serial.print(zumoController.path_state.v_forward, 2); Serial.print(",");
    Serial.println("LineFollow");

    // Simple proportional-derivative controller for line following
    int16_t speedDifference = error / 4 + 6 * (error - lastError);
    lastError = error;

    int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

    leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

    motors.setSpeeds(leftSpeed, rightSpeed);
  }
}
