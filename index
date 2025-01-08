#include <NewPing.h>

// Define pins for sensors
#define TRIGGER_PIN_LEFT  A3
#define ECHO_PIN_LEFT     A0

#define TRIGGER_PIN_FRONT  A4
#define ECHO_PIN_FRONT     A1

#define TRIGGER_PIN_RIGHT  A5
#define ECHO_PIN_RIGHT     A2

// Constants
#define MAX_DISTANCE 100

// Direction control definitions
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

// PID Constants
float kp = 0.7;
float kd = 0.5;
float ki = 0.4;
float previousProportionalError;
float cumulativeError;
int sensorOffset = 5;

// Thresholds
int wallThreshold = 13;
int frontWallThreshold = 7;

bool hasFrontWall;
bool hasLeftWall;
bool hasRightWall;
bool initialTurn;
bool isFollowingRightWall;
bool isFollowingLeftWall;

// Motor control pins
int motorPinLeft1 = 2;
int motorPinLeft2 = 3;
int motorPinRight1 = 4;
int motorPinRight2 = 5;

int pwmLeftMotor = 10;
int pwmRightMotor = 11;

int defaultMotorSpeed = 70;
int pwmRightMotorSpeed;
int pwmLeftMotorSpeed;

// LEDs
int statusLED = 13;
int ledIndicator1 = 8;
int ledIndicator2 = 9;

// Ultrasonic sensors
NewPing sonarLeft(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

unsigned int sonarPingInterval = 30; // Frequency of pings (milliseconds)

float previousLeftReading, previousRightReading, leftReading, rightReading, frontReading, previousFrontReading, currentLeftSensor, currentRightSensor, currentFrontSensor;

void setup() {
    Serial.begin(115200); // Open serial monitor 

    for (int pin = 2; pin <= 13; pin++) {
        pinMode(pin, OUTPUT); // Initialize motor and LED pins
    }

    initialTurn = false;
    isFollowingRightWall = false;
    isFollowingLeftWall = false;
}

void loop() {
    readSensorValues();
    detectWalls();

    if (!initialTurn) {
        initiatePID();
    } 
    else if (isFollowingLeftWall) {
        followWallPID(true);
    } 
    else if (isFollowingRightWall) {
        followWallPID(false);
    }

    if (hasLeftWall && !hasRightWall && hasFrontWall) {
        followWallPID(false);

        if (!initialTurn) {
            initialTurn = true;
            isFollowingRightWall = true;

            digitalWrite(ledIndicator2, LOW);
            digitalWrite(ledIndicator1, HIGH);
        }
    }

    if (!hasLeftWall && hasRightWall && hasFrontWall) {
        followWallPID(true);

        if (!initialTurn) {
            initialTurn = true;
            isFollowingLeftWall = true;
            digitalWrite(statusLED, HIGH);
        }
    }

    if ((leftReading == 0 || leftReading > 100) && (rightReading == 0 || rightReading > 100) && (frontReading == 0 || frontReading > 100)) {
        setMotorDirection(STOP);
    }

    Serial.print("Left: ");
    Serial.print(leftReading);
    Serial.print(" cm ");
    Serial.print("Right: ");
    Serial.print(rightReading);
    Serial.print(" cm ");
    Serial.print("Front: ");
    Serial.println(frontReading);

    Serial.print("Error=");
    Serial.println(cumulativeError);
}

// Set motor direction
void setMotorDirection(int direction) {
    if (direction == FORWARD) {
        digitalWrite(motorPinLeft1, LOW);
        digitalWrite(motorPinLeft2, HIGH);
        digitalWrite(motorPinRight1, LOW);
        digitalWrite(motorPinRight2, HIGH);
    } 
    else if (direction == LEFT) {
        digitalWrite(motorPinLeft1, HIGH);
        digitalWrite(motorPinLeft2, LOW);
        digitalWrite(motorPinRight1, LOW);
        digitalWrite(motorPinRight2, HIGH);
    } 
    else if (direction == RIGHT) {
        digitalWrite(motorPinLeft1, LOW);
        digitalWrite(motorPinLeft2, HIGH);
        digitalWrite(motorPinRight1, HIGH);
        digitalWrite(motorPinRight2, LOW);
    } 
    else if (direction == STOP) {
        digitalWrite(motorPinLeft1, HIGH);
        digitalWrite(motorPinLeft2, HIGH);
        digitalWrite(motorPinRight1, HIGH);
        digitalWrite(motorPinRight2, HIGH);
    } 
    else if (direction == BACKWARD) {
        digitalWrite(motorPinLeft1, HIGH);
        digitalWrite(motorPinLeft2, LOW);
        digitalWrite(motorPinRight1, HIGH);
        digitalWrite(motorPinRight2, LOW);
    }
}

// Read sensor values
void readSensorValues() {
    currentLeftSensor = sonarLeft.ping_cm();
    currentRightSensor = sonarRight.ping_cm();
    currentFrontSensor = sonarFront.ping_cm();

    leftReading = (currentLeftSensor + previousLeftReading) / 2;
    rightReading = (currentRightSensor + previousRightReading) / 2;
    frontReading = (currentFrontSensor + previousFrontReading) / 2;

    previousLeftReading = leftReading;
    previousRightReading = rightReading;
    previousFrontReading = frontReading;
}

// Detect walls
void detectWalls() {
    hasLeftWall = leftReading < wallThreshold;
    hasRightWall = rightReading < wallThreshold;
    hasFrontWall = frontReading < frontWallThreshold;
}

// PID for starting navigation
void initiatePID() {
    float proportionalError = leftReading - rightReading;
    float derivativeError = proportionalError - previousProportionalError;
    float integralError = (2.0 / 3.0) * cumulativeError + proportionalError;

    cumulativeError = kp * proportionalError + kd * derivativeError + ki * integralError;
    previousProportionalError = proportionalError;

    pwmRightMotorSpeed = defaultMotorSpeed + cumulativeError;
    pwmLeftMotorSpeed = defaultMotorSpeed - cumulativeError;

    adjustMotorSpeedAndDirection();
}

// PID for wall-following
void followWallPID(bool isLeftWall) {
    float proportionalError = isLeftWall ? leftReading - rightReading - sensorOffset : leftReading - rightReading + sensorOffset;
    float derivativeError = proportionalError - previousProportionalError;
    float integralError = (2.0 / 3.0) * cumulativeError + proportionalError;

    cumulativeError = kp * proportionalError + kd * derivativeError + ki * integralError;
    previousProportionalError = proportionalError;

    pwmRightMotorSpeed = defaultMotorSpeed + cumulativeError;
    pwmLeftMotorSpeed = defaultMotorSpeed - cumulativeError;

    adjustMotorSpeedAndDirection();
}

// Adjust motor speed and direction based on PID output
void adjustMotorSpeedAndDirection() {
    if (pwmRightMotorSpeed < 0) {
        pwmRightMotorSpeed = map(pwmRightMotorSpeed, 0, -255, 0, 255);
        analogWrite(pwmLeftMotor, pwmLeftMotorSpeed);
        analogWrite(pwmRightMotor, pwmRightMotorSpeed);
        setMotorDirection(RIGHT);
    } 
    else if (pwmLeftMotorSpeed < 0) {
        pwmLeftMotorSpeed = map(pwmLeftMotorSpeed, 0, -255, 0, 255);
        analogWrite(pwmLeftMotor, pwmLeftMotorSpeed);
        analogWrite(pwmRightMotor, pwmRightMotorSpeed);
        setMotorDirection(LEFT);
    } 
    else {
        analogWrite(pwmLeftMotor, pwmLeftMotorSpeed);
        analogWrite(pwmRightMotor, pwmRightMotorSpeed);
        setMotorDirection(FORWARD);
    }
}
