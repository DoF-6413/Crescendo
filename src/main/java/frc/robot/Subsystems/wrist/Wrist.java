// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final ProfiledPIDController wristPIDController;
  private SimpleMotorFeedforward wristFeedforward;

  private static double goal = 0.0;
  private static boolean isPIDEnabled = true;
  private static boolean isTestingEnabled = false;

  /** Creates a new Wrist, the second joint of the wrist mechanism */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating Wrist");
    this.io = io;

    /** Creates a new Profiled PID controller for the Wrist */
    wristPIDController =
        new ProfiledPIDController(
            WristConstants.KP,
            WristConstants.KI,
            WristConstants.KD,
            new TrapezoidProfile.Constraints(
                WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
    wristPIDController.setGoal(WristConstants.DEFAULT_POSITION_DEG);
    wristPIDController.setTolerance(WristConstants.ANGLE_TOLERANCE);
    wristPIDController.disableContinuousInput();

    /** Creates a new Feedforward Contoller for the Wrist */
    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);

    // Puts adjustable PPID and FF values onto the SmartDashboard for the testing mode
    SmartDashboard.putNumber("wristkP", 0.8);
    SmartDashboard.putNumber("wristkI", 0.0);
    SmartDashboard.putNumber("wristkD", 0.01);
    SmartDashboard.putNumber("wristkS", 0.2);
    SmartDashboard.putNumber("wristkV", 0.0001);
    SmartDashboard.putNumber("wristkA", 0.0);
    SmartDashboard.putNumber("wristMaxAcceleration", 0.0);
  }

  @Override
  public void periodic() {
    /** Periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    if (isPIDEnabled) {
      setWristPercentSpeed(
          wristPIDController.calculate(inputs.wristAbsolutePositionRad)
              + (wristFeedforward.calculate(inputs.wristVelocityRadPerSec)
                  / RobotStateConstants
                      .BATTERY_VOLTAGE)); // Feedforward divided by 12 since it returns a voltage
    }

    if (isTestingEnabled) {
      testPIDFValues();
    }

    SmartDashboard.putNumber(
        "WristSetpointDeg", Units.radiansToDegrees(wristPIDController.getGoal().position));
  }

  /** Updates the set of loggable inputs for the Wrist */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets Wrist to a percentage of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setWristPercentSpeed(double percent) {
    io.setWristPercentSpeed(percent);
  }

  /**
   * Sets voltage of the Wrist
   *
   * @param volts -12 to 12
   */
  public void setWristMotorVoltage(double volts) {
    setWristMotorVoltage(volts);
  }

  /**
   * Sets brake mode of the Wrist
   *
   * @param enable Sets brake mode if true, coast if false
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Updates the angle that the Wrist should be at using the WPI PID controller
   *
   * @param goal Angle (Radians)
   */
  public void setGoal(double goal) {
    wristPIDController.setGoal(goal);
  }

  /** Returns whether the Wrist is at it's goal or not */
  public boolean atSetpoint() {
    return wristPIDController.atSetpoint();
  }

  /**
   * Changes the angle goal of the Wrist
   *
   * @param increment Angle (Radians)
   */
  public void incrementWristGoal(double increment) {
    wristPIDController.setGoal(wristPIDController.getGoal().position + increment);
  }

  /**
   * @param enabled True = Enable, False = Disable
   */
  public void enablePID(boolean enabled) {
    isPIDEnabled = enabled;
  }

  /**
   * @param enabled True = Enable, False = Disable
   */
  public void enableTesting(boolean enabled) {
    isTestingEnabled = enabled;
  }

  /** Updates the PID values from SmartDashboard */
  public void updatePIDController(double kp, double ki, double kd) {
    WristConstants.KP = kp;
    WristConstants.KI = ki;
    WristConstants.KD = kd;
    wristPIDController.setPID(WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  /** Updates the Trapezoidal Constraints from the SmartDashboard */
  public void updateTrapezoidalConstraints(double maxAcc) {
    WristConstants.MAX_ACCELERATION = maxAcc;
    wristPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
  }

  /** Updates the PID values from SmartDashboard */
  public void updateFFController(double ks, double kv, double ka) {
    WristConstants.KS = ks;
    WristConstants.KV = kv;
    WristConstants.KA = ka;
    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);
  }

  /** Updates PPID and FF values from the SmartDashboard during testing mode */
  public void testPIDFValues() {
    if (WristConstants.KP != SmartDashboard.getNumber("wristkP", 0.8)
        || WristConstants.KI != SmartDashboard.getNumber("wristkI", 0.0)
        || WristConstants.KD != SmartDashboard.getNumber("wristkD", 0.01)) {
      updatePIDController(
          SmartDashboard.getNumber("wristkP", 0.8),
          SmartDashboard.getNumber("wristkI", 0.0),
          SmartDashboard.getNumber("wristkD", 0.01));
    }
    if (WristConstants.KS != SmartDashboard.getNumber("wristkS", 0.2)
        || WristConstants.KV != SmartDashboard.getNumber("wristkV", 0.0001)
        || WristConstants.KA != SmartDashboard.getNumber("wristkA", 0.0)) {
      updateFFController(
          SmartDashboard.getNumber("wristkS", 0.2),
          SmartDashboard.getNumber("wristkV", 0.0001),
          SmartDashboard.getNumber("wristkA", 0.0));
    }

    if (WristConstants.MAX_ACCELERATION != SmartDashboard.getNumber("wristMaxAcceleration", 0.0)) {
      updateTrapezoidalConstraints(SmartDashboard.getNumber("armMaxAcceleration", 0.0));
    }
  }
}
