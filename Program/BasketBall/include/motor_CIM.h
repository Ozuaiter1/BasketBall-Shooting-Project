/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       motor_cim.h                                               */
/*    Author:       Omar Zuaiter                                              */
/*    Created:      Jan 25 2022                                               */
/*    Description:  CIM Smart Motor Class                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef CIM_MOTOR
#define CIM_MOTOR
#include "vex.h"
#include <vector>
using namespace vex;
#define MAX_RPM 5330.0
class motor_cim {
private:
  motor_victor &cimMotor; // motor we are powering
  encoder &motorEncoder;  // encoder attached to the gearbox
  const float kP;         // ?
  const float kI;         // ?
  const float kD;         // ?
  const float gearRatio;  // driven/driving teeth
  const float ticksRev;   // ticks per revolution
  float desiredSpeed = 0; // rpm
  float currentSpeed;     // rpm
  float outputSpeed;      // pct
  float error = 0;
  float lastError = 0;
  float proportional = 0;
  float integral = 0;
  float derivative = 0;

public:
  motor_cim(motor_victor &cimMotor, encoder &motorEncoder, const float kP,
            const float kI, const float kD, const float gearRatio,
            const float ticksRev)
      : cimMotor(cimMotor), motorEncoder(motorEncoder), kP(kP), kI(kI), kD(kD),
        gearRatio(gearRatio), ticksRev(ticksRev){};
  void setVelocity(float desiredRPM) {
    desiredSpeed = sgn(desiredRPM) * min(fabs(desiredRPM), MAX_RPM / gearRatio);
  }
  float velocity() { return currentSpeed; }
  float outputPercentage() { return outputSpeed; }
  float effectiveness() {
    return (fabs(desiredSpeed) > 1 ? 100.0 * currentSpeed / desiredSpeed : 0);
  }
  void stop() { desiredSpeed = 0; }
  void controlLoop() {
    currentSpeed = motorEncoder.velocity(rpm) * 360.0 / gearRatio / ticksRev;
    // motorEncoder.velocity(rpm)??????????? what the frick does this output,
    // needs testing
    lastError = error;
    error = desiredSpeed - currentSpeed;
    proportional = kP * error;
    integral += kI * LOOP_TIME * (error + lastError) / 2000.0;
    derivative = kD * (error - lastError);
    outputSpeed = 100.0 * (desiredSpeed) / gearRatio / MAX_RPM + proportional +
                  integral + derivative;
    cimMotor.spin(fwd, outputSpeed, pct);
  }
};

class motor_dumb_cim {
private:
  motor_victor &cimMotor; // motor we are powering
  float outputSpeed;      // pct
public:
  motor_dumb_cim(motor_victor &cimMotor) : cimMotor(cimMotor){};
  void setVelocity(float percentage) { outputSpeed = percentage; }
  float outputPercentage() { return outputSpeed; }
  void spin(float percentage) { cimMotor.spin(fwd, percentage, pct); }
  void spin() { cimMotor.spin(fwd, outputSpeed, pct); }
  void stop() {
    outputSpeed = 0;
    cimMotor.stop();
  }
};
class motor_accel_cim {
private:
  vector<motor_victor> cimMotors; // motor we are powering
  float outputSpeed = 0;          // pct
  const float accelTime;
  float accel;
  float desiredSpeed = 0; // pct
  const bool reverseFlag;

public:
  motor_accel_cim(vector<motor_victor> cimMotors, float accelTime,
                  bool reverseFlag)
      : cimMotors(cimMotors), accelTime(accelTime), reverseFlag(reverseFlag) {
    accel = 100.0 * LOOP_TIME / 1000.0 / accelTime;
  };
  void setVelocity(float percentage) {
    desiredSpeed = fmax(
        -100, fmin(percentage +
                       (sgn(percentage) < 0 ? (reverseFlag ? -1 : 1) * 7 : 0),
                   100));
  }
  float outputPercentage() { return outputSpeed; }

  void spin(float percentage) {
    setVelocity(percentage);
    spin();
  }
  void spin() {
    float difference = desiredSpeed - outputSpeed;
    if (fabs(difference) > accel) {
      outputSpeed += sgn(difference) * accel;
    } else {
      outputSpeed = desiredSpeed;
    }
    for (auto &cimMotor : cimMotors) // access by reference to avoid copying
    {
      cimMotor.spin(fwd, outputSpeed, pct);
    }
  }
  void stop() { desiredSpeed = 0; }
};

#endif