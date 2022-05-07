/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       XDrive.h                                                  */
/*    Author:       Omar Zuaiter                                              */
/*    Created:      Sun Oct 5 2020                                            */
/*    Description:  XDrive Chassis Class                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef XDRIVE_H
#define XDRIVE_H
#include "math.h"
#include "motor_CIM.h"
#include "vex.h"
#include <vector>

using namespace vex;
class Chassis {

private:
  int factorial(int n) { return (n <= 1 ? 1 : n * factorial(n - 1)); }
  int nchoosek(int n, int k) {
    return factorial(n) / (factorial((n - k)) * (factorial(k)));
  }
  /**
   * The desired position of the robot relative to the field
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double globalTarget[3] = {0, 0, 0};
  /**
   * The desired position of the robot relative to the field
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double globalLookTarget[2] = {-1, -1};
  /**
   * The old desired position of the robot relative to the field
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double globalFinalTarget[2] = {0, 0};

  /**
   * The position of the robot as it moves around the field relative to the
   * field
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double globalPosition[3] = {0, 0, 0};

  /**
   * The velocity of the robot as it moves around the field relative to the
   * field
   *
   * 0 - (Inches / Second) X Velocity
   * 1 - (Inches / Second) Y Velocity
   * 2 - (Degrees / Second) Angular Velocity
   */
  double globalVelocity[3] = {0, 0, 0};

  /**
   * The desired position of the robot relative to the robot
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double localTarget[3] = {0, 0, 0};

  /**
   * The old desired position of the robot relative to the robot
   *
   * 0 - (Inches) X Position
   * 1 - (Inches) Y Position
   * 2 - (Degrees) Angular Position
   */
  double localFinalTarget[2] = {0, 0};

  /**
   * The velocity of the robot as it moves around the field relative to the
   * robot
   *
   * 0 - (Inches / Second) X Velocity
   * 1 - (Inches / Second) Y Velocity
   * 2 - (Degrees / Second) Angular Velocity
   */
  double localVelocity[3] = {0, 0, 0};

  /**
   * The desired velocity of the robot as it moves around the field relative to
   * the robot
   *
   * 0 - (Percent) X Velocity
   * 1 - (Percent) Y Velocity
   * 2 - (Percent) Angular Velocity
   */
  double localDesiredVelocity[3] = {0, 0, 0};
  /**
   * The maximum velocity of the robot as it moves around the field relative to
   * the robot
   *
   * 0 - (Percent) Maximum Translational Velocity
   * 2 - (Percent) Maximum Angular Velocity
   */
  double localMaximumVelocity[2] = {141, 100};
  /**
   * The Velocity Constants for the robot
   *
   * Outer Vector
   * 0 - Local Translational Velocity Calculation
   * 1 - Local Angular Velocity Calculation
   *
   *    Inner Vector
   *    0 - Constant
   *    1- Minumum Speed
   *    2 - Kickout Tolerance
   *    3 - Kickout Loops Required
   */
  const std::vector<std::vector<double>> PIDConstants;

  /**
   * The amount of times the  velocity calculations have passed
   *
   * 0 - Local Translational Velocity Calculation
   * 2 - Local Angular Velocity Calculation
   * 3 - Kill Counter
   */
  double counter[3] = {0, 0, 0};

  /**
   * The maximum amount of cycles a motion is allowed to go through before it
   * will no longer wait for the motion to complete
   */
  double maxKillCounter = 0;

  /**
   * An internal boolean that will kick the chassis motions if the time for the
   * motion has excedeeded the maximum time allotted
   */
  bool killOverride = false;

  /**
   * The chassis is within a specified tolerance to the target location
   *
   * 0 - (boolean) Local Translational Movement completed
   * 1 - (boolean) Angular movement completed
   */
  bool movementCompleted[2] = {true, true};
  /**
   * This controls when the point calculation loop should control the motion of
   * the robot
   */
  bool motionControlRunning = false;

  /**
   * The collection of motors that will be powering the front left side of the
   * chassis
   */
  motor_accel_cim &frontLeftMotor;

  /**
   * The collection of motors that will be powering the front right side of the
   * chassis
   */
  motor_accel_cim &frontRightMotor;

  /**
   * The collection of motors that will be powering the back left side of the
   * chassis
   */
  motor_accel_cim &backLeftMotor;

  /**
   * The collection of motors that will be powering the back left side of the
   * chassis
   */
  motor_accel_cim &backRightMotor;

  /**
   * The main Gyro that is used for the measuring turns made by the chassis.
   */
  inertial &leftGyro;
  inertial &rightGyro;

  encoder &frontWheelEncoder;
  encoder &leftWheelEncoder;
  encoder &rightWheelEncoder;
  encoder &backWheelEncoder;

  vision &frontLeftCamera;
  vision &frontRightCamera;
  vision &backLeftCamera;
  vision &backRightCamera;

  vision::signature &BLACKLINE;

  /**
   * The effective wheel radius of the robot in inches
   */
  const double trackerWheelRadius;

  const int ticksPerRev;

  /**
   *  The gyro needs to be inverted if it is measuring the wrong way
   */
  bool gyroIsInverted = false;

  void pointTransformation(double *gtarget, double *ltarget) {
    double xVector, yVector, angularVector;
    xVector = gtarget[0] - globalPosition[0];
    yVector = gtarget[1] - globalPosition[1];
    angularVector = atan2(yVector, xVector) - (gtarget[2] - 90) * M_PI / 180;
    fmod(angularVector, 2.0 * M_PI);
    xVector *= xVector;
    yVector *= yVector;
    ltarget[0] = sqrt(xVector + yVector) * cos(angularVector);
    ltarget[1] = sqrt(xVector + yVector) * sin(angularVector);
    ltarget[2] = gtarget[2] - globalPosition[2];
    if (ltarget[2] > 180) {
      ltarget[2] -= 360;
    } else if (ltarget[2] < -180) {
      ltarget[2] += 360;
    }
  }

public:
  Chassis(motor_accel_cim &frontLeftMotor, motor_accel_cim &frontRightMotor,
          motor_accel_cim &backLeftMotor, motor_accel_cim &backRightMotor,
          inertial &leftGyro, inertial &rightGyro, encoder &frontWheelEncoder,
          encoder &leftWheelEncoder, encoder &rightWheelEncoder,
          encoder &backWheelEncoder,
          std::vector<std::vector<double>> PIDConstants,
          const double trackerWheelRadius, const double ticksPerRev,
          vision &frontLeftCamera, vision &frontRightCamera,
          vision &backLeftCamera, vision &backRightCamera,
          vision::signature &BLACKLINE)
      : frontLeftMotor(frontLeftMotor), frontRightMotor(frontRightMotor),
        backLeftMotor(backLeftMotor), backRightMotor(backRightMotor),
        leftGyro(leftGyro), rightGyro(rightGyro),
        frontWheelEncoder(frontWheelEncoder),
        leftWheelEncoder(leftWheelEncoder),
        rightWheelEncoder(rightWheelEncoder),
        backWheelEncoder(backWheelEncoder), PIDConstants(PIDConstants),
        trackerWheelRadius(trackerWheelRadius), ticksPerRev(ticksPerRev),
        frontLeftCamera(frontLeftCamera), frontRightCamera(frontRightCamera),
        backLeftCamera(backLeftCamera), backRightCamera(backRightCamera),
        BLACKLINE(BLACKLINE) {}

  void initialize() {
    leftGyro.startCalibration();
    rightGyro.startCalibration();
    cout << "Calibrating" << endl << flush;
    wait(1, sec);
    waitUntil(!leftGyro.isCalibrating());
    waitUntil(!rightGyro.isCalibrating());
    wait(1, sec);
    cout << "Calibration Done" << endl << flush;
    resetMotors();
    resetLocation(0, 0, 0);
  }

  /**
   * This gets a specified calculated local desired velocity value
   *
   * @param index specifies which local velocity is wanted
   *        0 - (Inches / Second) X Velocity
   *        1 - (Inches / Second) Y Velocity
   *        2 - (Degrees / Second) Angular Velocity
   * @return returns the specified local velcity that is desired.
   */
  double getLocalDesiredVelocity(int index) {
    return localDesiredVelocity[index];
  }

  /**
   * This gets a specified calculated local desired velocity value
   *
   * @param index specifies which local velocity is wanted
   *        0 - (Inches / Second) X Velocity
   *        1 - (Inches / Second) Y Velocity
   *        2 - (Degrees / Second) Angular Velocity
   * @return returns the specified local velcity that is desired.
   */
  double getLocalVelocity(int index) { return localVelocity[index]; }
  /**
   * Gets the value of a specified index from the localTarget array
   *
   * @param index the index of the localTarget array
   *        0 -  The local x target of the robot
   *        1 -  The local y target of the robot
   *        2 -  The local theta target of the robot
   * @return double the value of a specified index from the localTarget
   * array
   */
  double getLocalTarget(int index) { return localTarget[index]; }
  /**
   * Gets the value of a specified index from the globalTarget array
   *
   * @param index the index of the globalTarget array
   *        0 -  The global x target of the robot
   *        1 -  The global y target of the robot
   *        2 -  The global theta target of the robot
   * @return double the value of a specified index from the globalTarget
   * array
   */
  double getGlobalTarget(int index) { return globalTarget[index]; }
  /**
   * Gets if the point control calculations is on or off
   *
   * @return the value of motionControlRunning
   */
  bool getMotionControlRunning() { return motionControlRunning; }

  /**
   * Gets the value of a specified index from the movementCompleted array
   *
   * @param index the index of the movementCompleted array
   *        0 -  Local Translational Movement completed
   *        1 -  Angular movement completed
   * @return boolean the value of a specified index from the movementCompleted
   * array
   */
  bool getMovementCompleted(int index) { return movementCompleted[index]; }
  /**
   * Gets the value of a specified index from the globalPosition array
   *
   * @param index the index of the globalPosition array
   *        0 -  The global x postion of the robot
   *        1 -  The global y postion of the robot
   *        2 -  The global theta position of the robot
   * @return double the value of a specified index from the globalPosition
   * array
   */
  double getPosition(int index) { return globalPosition[index]; }
  /**
   * Sets the speed of the individual motors on the chassis from desired local
   * velocities
   *
   * @param xVelocity the side to side speed desired in percent
   * @param yVelocity the front to back speed desired in percent
   * @param omega the rotation speed desired in percent
   */
  void setSpeed(double xVelocity, double yVelocity, double omega) {
    double flSpeed =
        (yVelocity + xVelocity) * (1 - fabs(omega) / localMaximumVelocity[1]) -
        omega;
    double frSpeed =
        (yVelocity - xVelocity) * (1 - fabs(omega) / localMaximumVelocity[1]) +
        omega;
    double blSpeed =
        (yVelocity - xVelocity) * (1 - fabs(omega) / localMaximumVelocity[1]) -
        omega;
    double brSpeed =
        (yVelocity + xVelocity) * (1 - fabs(omega) / localMaximumVelocity[1]) +
        omega;
    double max = fmax(
        fmax(fmax(fmax(localMaximumVelocity[0], fabs(flSpeed)), fabs(frSpeed)),
             fabs(blSpeed)),
        fabs(brSpeed));
    if (max > localMaximumVelocity[0] && fabs(max) > 0) {
      flSpeed *= localMaximumVelocity[0] / max;
      frSpeed *= localMaximumVelocity[0] / max;
      blSpeed *= localMaximumVelocity[0] / max;
      brSpeed *= localMaximumVelocity[0] / max;
    }
    frontLeftMotor.spin(flSpeed);
    frontRightMotor.spin(frSpeed);
    backLeftMotor.spin(blSpeed);
    backRightMotor.spin(brSpeed);
  }
  /**
   * Sets the speed of the individual motors on the chassis from desired local
   * velocities
   *
   * @param xVelocity the side to side speed desired in percent
   * @param yVelocity the front to back speed desired in percent
   * @param omega the rotation speed desired in percent
   */
  void setSpeedDriver(double xVelocity, double yVelocity, double omega) {
    frontLeftMotor.spin(yVelocity + xVelocity - omega);
    frontRightMotor.spin(yVelocity - xVelocity + omega);
    backLeftMotor.spin(yVelocity - xVelocity - omega);
    backRightMotor.spin(yVelocity + xVelocity + omega);
  }

  /**
   * Sets the point control calculations as on or off
   *
   * @param turnOn the desired value of motionControlRunning
   */
  void setMotionControlRunning(bool turnOn) { motionControlRunning = turnOn; }

  /**
   * Sets the Gyro to be inverted or not
   *
   * @param desired the desired value of the gyroIsInverted boolean
   */
  void setGyroIsInverted(bool desired) { gyroIsInverted = desired; }

  /**
   * Sets the maximum amount of time before an action is killed on the chassis
   *
   * @param miliSeconds the amount of time wanted to be the desired value.
   */
  void setKillTimer(double miliSeconds) {
    maxKillCounter = miliSeconds / LOOP_TIME;
  }
  /**
   * Resets the motors to a default postion and sets the motors to a coast brake
   * setting.
   */
  void resetMotors() {
    frontWheelEncoder.resetRotation();
    leftWheelEncoder.resetRotation();
    rightWheelEncoder.resetRotation();
    backWheelEncoder.resetRotation();
  }
  /**
   * Checks to see if the robot is stationary
   *
   * @return boolean value if stationary is true
   */
  bool isDone() { return movementCompleted[0] && movementCompleted[1]; }

  void calculatePosition() {
    int frontVector = frontWheelEncoder.value();
    int leftVector = leftWheelEncoder.value();
    int rightVector = rightWheelEncoder.value();
    int backVector = backWheelEncoder.value();
    frontWheelEncoder.setPosition(0, rotationUnits::raw);
    leftWheelEncoder.setPosition(0, rotationUnits::raw);
    rightWheelEncoder.setPosition(0, rotationUnits::raw);
    backWheelEncoder.setPosition(0, rotationUnits::raw);
    if (!gyroIsInverted) {
      globalPosition[2] = (leftGyro.heading() + rightGyro.heading()) / 2.0;
    } else {
      globalPosition[2] =
          360 - (leftGyro.heading() + rightGyro.heading()) / 2.0;
    }
    localVelocity[2] = 0;
    /*(SensorDifference) / 2 / ticksPerRev * 360 * M_PI / 180 *
     * wheelRadius/(LOOP_TIME/1000) / baseWidth * 2 * 180 / M_PI
     */
    localVelocity[0] = (frontVector + backVector) * M_PI * 1000.0 *
                       trackerWheelRadius / ticksPerRev / LOOP_TIME;
    //(Sensors Sum)/2*360/TicksPerRev*M_PI/180*wheelRadius/(LOOP_TIME/1000)
    localVelocity[1] = (leftVector + rightVector) * M_PI * 1000.0 *
                       trackerWheelRadius / ticksPerRev / LOOP_TIME;
    //(Sensors Sum)/2*360/TicksPerRev*M_PI/180*wheelRadius/(LOOP_TIME/1000)
    globalVelocity[2] = localVelocity[2];
    globalVelocity[0] =
        localVelocity[0] *
            sin((globalPosition[2] + globalVelocity[2] * LOOP_TIME / 2000) *
                M_PI / 180) +
        localVelocity[1] *
            cos((globalPosition[2] + globalVelocity[2] * LOOP_TIME / 2000) *
                M_PI / 180);
    globalVelocity[1] =
        -1 * localVelocity[0] *
            cos((globalPosition[2] + globalVelocity[2] * LOOP_TIME / 2000) *
                M_PI / 180) +
        localVelocity[1] *
            sin((globalPosition[2] + globalVelocity[2] * LOOP_TIME / 2000) *
                M_PI / 180);
    globalPosition[0] += globalVelocity[0] * LOOP_TIME / 1000;
    globalPosition[1] += globalVelocity[1] * LOOP_TIME / 1000;
    double xVector, yVector, angularVector;
    if (globalLookTarget[0] >= 0 && globalLookTarget[1] >= 0) {
      xVector = globalLookTarget[0] - globalPosition[0];
      yVector = globalLookTarget[1] - globalPosition[1];
      angularVector = atan2(yVector, xVector) * 180 / M_PI;
      globalTarget[2] = angularVector;
    }
    pointTransformation(globalTarget, localTarget);
    pointTransformation(globalFinalTarget, localFinalTarget);
  }

  void pointControl() {
    localDesiredVelocity[2] =
        (!movementCompleted[1]
             ? sgn(localTarget[2]) *
                   (pow(fabs(localTarget[2]), 1.5) * PIDConstants[1][0] +
                    PIDConstants[1][1])
             : 0);
    if (fabs(localDesiredVelocity[2]) > localMaximumVelocity[1]) {
      localDesiredVelocity[2] =
          sgn(localDesiredVelocity[2]) * localMaximumVelocity[1];
    }
    if (fabs(localTarget[2]) < PIDConstants[1][2]) {
      counter[1]++;
    } else {
      counter[1] = 0;
    }
    movementCompleted[1] = counter[1] > PIDConstants[1][3];

    double targetDistance =
        sqrt(pow(localTarget[0], 2) + pow(localTarget[1], 2));
    localDesiredVelocity[0] =
        (!movementCompleted[0]
             ? min((pow(fabs(localTarget[0]), 1.5) * PIDConstants[0][0] +
                    PIDConstants[0][1]),
                   localMaximumVelocity[0])
             : 0) *
        sgn(localTarget[0]);
    localDesiredVelocity[1] =
        (!movementCompleted[0]
             ? min((pow(fabs(localTarget[1]), 1.5) * PIDConstants[0][0] +
                    PIDConstants[0][1]),
                   localMaximumVelocity[0])
             : 0) *
        sgn(localTarget[1]);
    if (targetDistance < PIDConstants[0][2]) {
      counter[0]++;
    } else {
      counter[0] = 0;
    }
    movementCompleted[0] = counter[0] > PIDConstants[0][3];
  }

  void setAllMaximumVelocities(double translationalVelocity, double omega) {
    localMaximumVelocity[0] = translationalVelocity;
    localMaximumVelocity[1] = omega;
  }
  void lookAt(double angularPos, bool wait = false) {
    setMotionControlRunning(true);
    globalLookTarget[0] = -1;
    globalLookTarget[1] = -1;
    globalTarget[2] = angularPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      while ((!getMovementCompleted(0) || !getMovementCompleted(1)) &&
             !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }

  void lookAtPos(double xPos, double yPos, bool wait = false) {
    setMotionControlRunning(true);
    globalLookTarget[0] = xPos;
    globalLookTarget[1] = yPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      while ((!getMovementCompleted(0) || !getMovementCompleted(1)) &&
             !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }

  void lookAtArea(double angularPos, bool wait = false, double threshold = 15) {
    setMotionControlRunning(true);
    globalLookTarget[0] = -1;
    globalLookTarget[1] = -1;
    globalTarget[2] = angularPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      task::sleep(LOOP_TIME * 10);
      while (fabs(localTarget[2]) > threshold && !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }
  void lookAtAreaPos(double xPos, double yPos, bool wait = false,
                     double threshold = 15) {
    setMotionControlRunning(true);
    globalLookTarget[0] = xPos;
    globalLookTarget[1] = yPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      task::sleep(LOOP_TIME * 10);
      while (fabs(localTarget[2]) > threshold && !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }
  void goTo(double xPos, double yPos, bool wait = false) {
    setMotionControlRunning(true);
    globalTarget[0] = xPos;
    globalTarget[1] = yPos;
    globalFinalTarget[0] = xPos;
    globalFinalTarget[1] = yPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      while ((!getMovementCompleted(0) || !getMovementCompleted(1)) &&
             !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }
  void goThrough(double xPos, double yPos, bool wait = false,
                 double threshold = 10, double angularThreshold = 30) {
    setMotionControlRunning(true);
    globalTarget[0] = xPos;
    globalTarget[1] = yPos;
    globalFinalTarget[0] = xPos;
    globalFinalTarget[1] = yPos;
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    if (wait) {
      resetKillSwitch();
      task::sleep(LOOP_TIME * 10);
      while (
          (sqrt(pow(localTarget[0], 2) + pow(localTarget[1], 2)) > threshold ||
           fabs(localTarget[2]) > angularThreshold) &&
          !killOverride) {
        task::sleep(LOOP_TIME);
      }
    }
  }

  void goForGlobal(double xGlobal, double yGlobal, bool wait = false) {
    goTo(globalPosition[0] + xGlobal, globalPosition[1] + yGlobal, wait);
  }

  void goFor(double xLocal, double yLocal, bool wait = false) {
    goTo(globalPosition[0] + cos(globalPosition[2] * M_PI / 180) * yLocal +
             sin((globalPosition[2]) * M_PI / 180) * xLocal,
         globalPosition[1] + sin(globalPosition[2] * M_PI / 180) * yLocal -
             cos(globalPosition[2] * M_PI / 180) * xLocal,
         wait);
  }
  void followPath(const vector<vector<double>> controlPoints, const int Scale,
                  const int percentFinish,
                  const vector<vector<double>> angleChanges) {
    double points[Scale + 1][2];
    int n = controlPoints.size() - 1;
    for (int t = 0; t <= Scale; t++) {
      points[t][0] = 0;
      points[t][1] = 0;
      for (int j = 0; j <= n; j++) {
        points[t][0] +=
            nchoosek(n, j) * pow(1 - (double)(t / (double)Scale), n - j) *
            (pow((double)(t / (double)Scale), j)) * controlPoints[j][0];
        points[t][1] +=
            nchoosek(n, j) * pow(1 - (double)(t / (double)Scale), n - j) *
            (pow((double)(t / (double)Scale), j)) * controlPoints[j][1];
      }
    }
    setMotionControlRunning(true);
    globalFinalTarget[0] = points[Scale][0];
    globalFinalTarget[1] = points[Scale][1];
    movementCompleted[0] = false;
    movementCompleted[1] = false;
    int i = 0;
    int angleCounter = 0;
    resetKillSwitch();
    while (i < Scale * percentFinish / 100 && !killOverride) {
      if (sqrt(pow(localTarget[0], 2) + pow(localTarget[1], 2)) < 8 &&
          fabs(localTarget[2]) < 30) {
        resetKillSwitch();
        i++;
        globalTarget[0] = points[i][0];
        globalTarget[1] = points[i][1];
        if (!angleChanges.empty() && angleChanges.size() > angleCounter &&
            !angleChanges[angleCounter].empty() &&
            Scale * angleChanges[angleCounter][1] / 100 <= i) {
          globalTarget[2] = angleChanges[angleCounter][0];
          angleCounter++;
        }
      }
      task::sleep(LOOP_TIME);
    }
    (percentFinish == 100
         ? goTo(globalFinalTarget[0], globalFinalTarget[1], true)
         : (percentFinish == 99
                ? goThrough(globalFinalTarget[0], globalFinalTarget[1], true, 4)
                : goTo(globalFinalTarget[0], globalFinalTarget[1], false)));
  }
  void resetFromGoal() {
    double c2c = 23 * 12;
    resetLocation(-c2c * cos(globalPosition[2] * M_PI / 180),
                  -c2c * sin(globalPosition[2] * M_PI / 180));
  }
  void resetLocation(double xPos, double yPos, double angularPos) {
    globalTarget[0] = xPos;
    globalPosition[0] = xPos;
    globalFinalTarget[0] = xPos;
    globalTarget[1] = yPos;
    globalPosition[1] = yPos;
    globalFinalTarget[1] = yPos;
    globalLookTarget[0] = -1;
    globalLookTarget[1] = -1;
    globalTarget[2] = angularPos;
    // globalPosition[2] = angularPos;
    if (!gyroIsInverted) {
      leftGyro.setHeading(angularPos, deg);
      rightGyro.setHeading(angularPos, deg);
    } else {
      leftGyro.setHeading(360 - angularPos, deg);
      rightGyro.setHeading(360 - angularPos, deg);
    }
    setMotionControlRunning(true);
  }
  void resetLocation(double xPos, double yPos) {
    globalTarget[0] = xPos;
    globalPosition[0] = xPos;
    globalFinalTarget[0] = xPos;
    globalTarget[1] = yPos;
    globalPosition[1] = yPos;
    globalFinalTarget[1] = yPos;
    globalLookTarget[0] = -1;
    globalLookTarget[1] = -1;
    globalTarget[2] = globalPosition[2];
    setMotionControlRunning(true);
  }
  void resetKillSwitch() {
    killOverride = false;
    counter[2] = 0;
  }

  void killSwitch() {
    if (maxKillCounter) {
      counter[2]++;
      killOverride = counter[2] > maxKillCounter;
    } else {
      killOverride = false;
    }
  }
  void debug() {
    cout << getPosition(0) << "\t" << getPosition(1) << "\t" << getPosition(2)
         << "\t" << getLocalDesiredVelocity(0) << "\t"
         << getLocalDesiredVelocity(1) << "\t" << getLocalDesiredVelocity(2)
         << endl
         << flush;
    /*cout << frontLeftMotor.outputPercentage() << "\t"
         << frontRightMotor.outputPercentage() << "\t"
         << backLeftMotor.outputPercentage() << "\t"
         << backRightMotor.outputPercentage() << "\t" << getPosition(2) << "\t"
         << getLocalDesiredVelocity(2) << endl
         << flush;*/
    // cout << leftGyro.heading() << "\t" << rightGyro.heading() << endl <<
    // flush;
    // cout << maxKillCounter << "\t" << counter[2] << endl << flush;
  }

  void lineUp(bool front = true, float timeOut = 10) {
    setMotionControlRunning(false);
    class Line {
    public:
      int originX;
      int originY;
      int centerX;
      int centerY;
      int width;
      int height;
      bool exists;
      Line(vision::object given)
          : originX(given.originX), originY(given.originY),
            centerX(given.centerX), centerY(given.centerY), width(given.width),
            height(given.height), exists(given.exists) {}
      Line() : exists(false) {}
    };
    int counter = 0;
    int allignedCounter = 0;
    float maxRotationSpeed = 30;
    float maxForwardSpeed = 30;
    float forwardSpeed, rotationSpeed;
    const int CameraTarget = 190;
    Line rightLine, leftLine;
    while (true) {
      if (front) {
        frontRightCamera.takeSnapshot(BLACKLINE);
        rightLine = Line(frontRightCamera.largestObject);
        frontLeftCamera.takeSnapshot(BLACKLINE);
        leftLine = Line(frontLeftCamera.largestObject);
      } else {
        backRightCamera.takeSnapshot(BLACKLINE);
        leftLine = Line(backRightCamera.largestObject);
        backLeftCamera.takeSnapshot(BLACKLINE);
        rightLine = Line(backLeftCamera.largestObject);
      }
      if (rightLine.exists && leftLine.exists) {
        int rotationError = leftLine.centerY - rightLine.centerY;
        forwardSpeed =
            (front ? 1 : -1) * 2.5 *
            (CameraTarget - (leftLine.centerY + rightLine.centerY) / 2.0);
        rotationSpeed = (abs(rotationError) > 10 ? 0.5 : 1.5) * (rotationError);
      } else if (rightLine.exists && !leftLine.exists) {
        forwardSpeed = 0;
        rotationSpeed = 15;
      } else if (leftLine.exists && !rightLine.exists) {
        forwardSpeed = 0;
        rotationSpeed = -15;
      } else {
        rotationSpeed = 0;
        forwardSpeed = (front ? 1 : -1) * -30;
      }
      forwardSpeed =
          sgn(forwardSpeed) * fmin(fabs(forwardSpeed), maxForwardSpeed);
      rotationSpeed =
          sgn(rotationSpeed) * fmin(fabs(rotationSpeed), maxRotationSpeed);
      setSpeedDriver(0, forwardSpeed, rotationSpeed);
      cout << rotationSpeed << " | " << forwardSpeed << endl << flush;
      if (rightLine.exists && leftLine.exists && fabs(rotationSpeed) < 8 &&
          fabs(forwardSpeed) < 8) {
        if (allignedCounter++ > 5) {
          break;
        }
      } else {
        allignedCounter = 0;
      }
      if (counter++ > timeOut * 1000 / (LOOP_TIME)) {
        break;
      }
      wait(LOOP_TIME, msec);
    }
    setSpeedDriver(0, 0, 0);
    resetFromGoal();
    setMotionControlRunning(true);
  }
};
#endif