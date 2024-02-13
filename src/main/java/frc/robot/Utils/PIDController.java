// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import frc.robot.Constants.RobotStateConstants;

/** Custom DOF PID Control Solution */
public class PIDController {
  private double kP;
  private double kI;
  private double kD;
  private double tolerance = 0.0;
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

  /** Updates Tolerance As A Decimal Percentage */
  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
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

  /** Gets Tolerance As A Decimal Percentage */
  public double getTolerance() {
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

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Calculates voltage from measurement, setpoint, and max setpoint value, PARAMETERS NEED THE SAME
   * UNITS
   *
   * @param measurement Current Measurement (Behavior or Location) of System
   * @param maxValue The Max Value the Setpoint Could Ever Achieve
   * @return Returns Voltage [-12 to 12]
   */
  public double calculateForVoltage(double measurement, double maxValue) {

    // PROPORTIONAL: Current error
    positionError = setpoint - measurement;
    // System.out.println("POSITION ERROR: " + positionError);

    // DERIVATIVE: Velocity error = change in position / time
    velocityError = (positionError - prevError) / RobotStateConstants.LOOP_PERIODIC_SEC;

    // update previous error
    prevError = positionError;

    // INTEGRAL: Total Error = Area Under Graph = position error * time, accumulated for each
    // periodic loop
    totalError += (positionError * RobotStateConstants.LOOP_PERIODIC_SEC);

    // raw voltage output + PID tuning = calculated voltage
    // * MATH BEHIND HOW IT WORKS: error is in volts, so add it to the RPM before scaling back down
    // */
    double desiredVoltage =
        // The setpoint * volts / maxValue  runs the motor at the speed it should theoretically run
        // at and the PID part gets it closer to the setpoint by adding
        (setpoint + (kP * positionError) + (kI * totalError) + (kD * velocityError))
            * RobotStateConstants.BATTERY_VOLTAGE
            / maxValue;

    // TODO: Implement tolerence

    return desiredVoltage;
  }

  /**
   * Calculates motor percent speed from measurement
   *
   * @param measurement Current Measurement (Behavior or Location) of System
   * @return Returns motor percent speed [-1 to 1]
   */
  //   public double calculate(double measurement) {
  //     // updates setpoint so accessible within class
  //     // this.setpoint = setpoint;
  //     System.out.println("ERROR VALUE " + error);

  //     // update previous error
  //     prevError = error;

  //     // PROPORTIONAL: Current error
  //     error = setpoint - measurement;
  //     p = error * kP;

  //     // INTEGRAL: Total Error = Area Under Graph = error accumulated for each periodic loop
  //     i += error * kI;
  //     System.out.println("I VALUE!! " + i);

  //     // DERIVATIVE: Velocity error = change in position / time
  //     d = ((error - prevError) / RobotStateConstants.LOOP_PERIODIC_SEC) * kD;

  //     double motorPercentSpeed = p + i + d;

  //     if (motorPercentSpeed > 1) {
  //       motorPercentSpeed = 1;
  //     } else if (motorPercentSpeed < -1) {
  //       motorPercentSpeed = -1;
  //     }

  //     System.out.println("MOTOR PERCENT SPEED " + motorPercentSpeed);
  //     return motorPercentSpeed;
  //   }

  /**
   * Returns whether or not the measurment is at the setpoint, including with the tolerance
   */
  public boolean atSetpoint(double voltage, double measurement) {
    if ((measurement >= setpoint - tolerance) && (measurement <= setpoint + tolerance)) {
      return true;
    }
    return false;
  }
}
