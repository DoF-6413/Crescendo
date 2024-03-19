// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import frc.robot.Constants.RobotStateConstants;

/** Custom DOF PID Control Solution */
public class PIDController {
  private double kP = 1.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double tolerance = 0.0;
  private double tolerancePercent = 0.0;
  private double positionError = 0.0;
  private double velocityError = 0.0;
  private double totalError = 0.0;
  private double prevError = 0.0;
  private double setpoint = 0.0;
  private boolean atSetpoint = false;

  /** Sets PID Controller without kI or kD */
  public PIDController(double kP) {
    this.kP = kP;
    this.kI = 0.0;
    this.kD = 0.0;
  }

  /** Sets PID Controller without kD */
  public PIDController(double kP, double kI) {
    this.kP = kP;
    this.kI = kI;
    this.kD = 0.0;
  }

  /** Sets PID Controller */
  public PIDController(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /**
   * Updates tolerance as a hardcoded value
   *
   * @param tolerance
   */
  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
    this.tolerancePercent = 0.0;
  }

  /**
   * Updates tolerance as a percent of the setpoint
   *
   * @param percent percentage of setpoint, -1 to 1
   */
  public void setTolerancePercent(double percent) {
    this.tolerancePercent = percent;
    this.tolerance = 0.0;
  }

  /** Updates kP Value */
  public void setP(double kP) {
    this.kP = kP;
  }

  /** Updates kI Value */
  public void setI(double kI) {
    this.kI = kI;
  }

  /** Updates kD Value */
  public void setD(double kD) {
    this.kD = kD;
  }

  /** Updates all PID Values */
  public void setPID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /**
   * Gets tolerance
   *
   * @param isPercent if true, tolerance is a percentage of setpoint, if false then tolerance is
   *     hardcoded value
   */
  public double getTolerance(boolean isPercent) {
    if (isPercent) {
      return tolerancePercent;
    }
    return tolerance;
  }

  /** Gets kP Value */
  public double getP() {
    return kP;
  }

  /** Gets kI Value */
  public double getI() {
    return kI;
  }

  /** Gets kD Value */
  public double getD() {
    return kD;
  }

  /** Gets all PID values in an array [kP, kI, kD] */
  public double[] getPID() {
    double[] pid = {kP, kI, kD};
    return pid;
  }

  /** Sets New Setpoint */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  /** Gets Current Setpoint */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Calculates voltage from measurement, setpoint, and max setpoint value, PARAMETERS NEED THE SAME
   * UNITS
   *
   * @param measurement Current Measurement (Behavior or Location) of System
   * @param maxValue The Max Value the Setpoint Could Ever Achieve
   * @return Returns Voltage -12 to 12
   */
  public double calculateForVoltage(double measurement, double maxValue) {

    // PROPORTIONAL: Current error
    positionError = setpoint - measurement;

    // DERIVATIVE: Velocity error = change in position / time
    velocityError = (positionError - prevError) / RobotStateConstants.LOOP_PERIODIC_SEC;

    // update previous error
    prevError = positionError;

    // INTEGRAL: Total Error = Area Under Graph = position error * time, accumulated for each
    // periodic loop
    totalError += (positionError * RobotStateConstants.LOOP_PERIODIC_SEC);

    // raw voltage output + PID tuning = calculated voltage
    /**
     * The setpoint * volts / maxValue runs the motor at the speed it should theoretically run at
     * and the PID gets it closer to the setpoint by adding up the errors multiplied by respective
     * constants
     */
    double desiredVoltage =
        (setpoint + (kP * positionError) + (kI * totalError) + (kD * velocityError))
            * RobotStateConstants.BATTERY_VOLTAGE
            / maxValue;

    if (setpoint == 0) {
      desiredVoltage = 0;
    }
    return desiredVoltage;
  }

  /** Returns whether or not the measurment is at the setpoint with tolerance */
  public boolean atSetpoint(double measurement) {
    /**
     * if the tolerance has been updated, use that value to calculate if the measurement is within
     * the setpoint +/- the tolerance, otherwise if the tolerance has been set as a percent,
     * calculate accordingly
     */
    if (tolerance != 0) {
      if ((setpoint - tolerance <= measurement) && (measurement <= setpoint + tolerance)) {
        atSetpoint = true;
      } else {
        atSetpoint = false;
      }
    } else if (tolerancePercent != 0) {
      if ((setpoint * (1 - tolerancePercent) <= measurement)
          && (measurement <= setpoint * (1 + tolerancePercent))) {
        atSetpoint = true;
      } else {
        atSetpoint = false;
      }
    }
    return atSetpoint;
  }
}
