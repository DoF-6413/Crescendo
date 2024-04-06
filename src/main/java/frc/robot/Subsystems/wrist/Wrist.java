// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;

import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Wrist PID controller */
  // private final ProfiledPIDController wristPIDController;
  private final PIDController wristPIDController;

  private SimpleMotorFeedforward wristFeedforward;
  private boolean isEnabled = true;
  private final ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");
  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;

  private static GenericEntry wristkp,
      wristki,
      wristkd,
      wristks,
      wristkv,
      wristka,
      wristGoal,
      wristMaxAcceleration;
  private static double goal = 0.0;

  /** Creates a new Wrist, the second joint of the wrist mechanism */
  public Wrist(WristIO io) {
    System.out.println("[Init] Creating Wrist");
    this.io = io;

    /** Creates a new PID controller for the Wrist */
    wristPIDController = new PIDController(1.0, WristConstants.KI, WristConstants.KD);
    // wristPIDController =
    //     new ProfiledPIDController(
    //         WristConstants.KP,
    //         WristConstants.KI,
    //         WristConstants.KD,
    //         new TrapezoidProfile.Constraints(
    //             WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
    // wristPIDController.setGoal(0);
    wristPIDController.setSetpoint(0);
    wristPIDController.setTolerance(WristConstants.ANGLE_TOLERANCE);
    wristPIDController.disableContinuousInput();

    wristFeedforward =
        new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);

    SmartDashboard.putNumber("wristkP", 0.0);
    SmartDashboard.putNumber("wristkI", 0.0);
    SmartDashboard.putNumber("wristkD", 0.0);
    // wristkp = wristTab.add("wristkp", 1.0).getEntry();
    // wristki = wristTab.add("wristki", 0.0).getEntry();
    // wristkd = wristTab.add("wristkd", 0.0).getEntry();
    wristGoal = wristTab.add("wristGoal", 0.0).getEntry();
    wristMaxAcceleration = wristTab.add("wristMaxAcceleration", 0.0).getEntry();
    wristks = wristTab.add("wristks", 0.0).getEntry();
    wristkv = wristTab.add("wristkv", 0.0).getEntry();
    wristka = wristTab.add("wristka", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    /** Periodically updates inputs and logs them */
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);
    if(isEnabled){

      setWristPercentSpeed(
          wristPIDController.calculate(inputs.wristAbsolutePositionRad)
              + (wristFeedforward.calculate(inputs.wristVelocityRadPerSec)
                  / RobotStateConstants
                      .BATTERY_VOLTAGE)); // Feedforward divided by 12 since it returns a voltage
    }

    if (
    // kP != wristkp.getDouble(0.0)
    //   || kI != wristki.getDouble(0.0)
    //   || kD != wristkd.getDouble(0.0)
    kP != SmartDashboard.getNumber("wristkP", 1.0)
        || kI != SmartDashboard.getNumber("wristkI", 0.0)
        || kD != SmartDashboard.getNumber("wristkD", 0.0)) {
      updatePIDController();
    }

    // if (WristConstants.KS != wristks.getDouble(0.0)
    //     || WristConstants.KV != wristkv.getDouble(0.0)
    //     || WristConstants.KA != wristka.getDouble(0.0)) {
    //   updateFFController();
    // }

    // if (goal != wristGoal.getDouble(0.0)) {
    //   updateGoal();
    // }

    // if (WristConstants.MAX_ACCELERATION != wristMaxAcceleration.getDouble(0.0)) {
    //   updateTrapezoidalConstraints();
    // }
    SmartDashboard.putNumber(
        "WristSetpointDeg", Units.radiansToDegrees(wristPIDController.getSetpoint()));
  }

  /** Updates the set of loggable inputs for the Wrist */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Updates the PID values from Shuffleboard */
  public void updatePIDController() {
    //   // kP = wristkp.getDouble(0.0);
    //   // kI = wristki.getDouble(0.0);
    //   // kD = wristkd.getDouble(0.0);
    kP = SmartDashboard.getNumber("wristkP", 1.0);
    kI = SmartDashboard.getNumber("wristkI", 0.0);
    kD = SmartDashboard.getNumber("wristkD", 0.0);
    wristPIDController.setPID(kP, kI, kD);
    //   // wristPIDController.setPID(WristConstants.KP, WristConstants.KI, WristConstants.KD);
  }

  /** Updates the goal from Shuffleboard */
  // public void updateGoal() {
  //   // goal = wristGoal.getDouble(0.0);
  //   // wristPIDController.setGoal(goal);
  //   wristPIDController.setSetpoint(goal);
  // }

  /** Updates the Trapezoidal Constraints from the ShuffleBoard */
  // public void updateTrapezoidalConstraints() {
  //   WristConstants.MAX_ACCELERATION = wristMaxAcceleration.getDouble(0.0);
  //   wristPIDController.setConstraints(
  //       new TrapezoidProfile.Constraints(
  //           WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));
  // }

  /** Updates the PID values from Shuffleboard */
  // public void updateFFController() {
  //   WristConstants.KS = wristks.getDouble(0.0);
  //   WristConstants.KV = wristkv.getDouble(0.0);
  //   WristConstants.KA = wristka.getDouble(0.0);
  //   wristFeedforward =
  //       new SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA);
  // }

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
    wristPIDController.setSetpoint(goal);
    // wristPIDController.setGoal(goal);
  }

  /** Returns whether the Wrist is at it's goal or not */
  public boolean atGoal() {
    return wristPIDController.atSetpoint();
    // return wristPIDController.atGoal();
  }

  /**
   * Changes the angle goal of the Wrist
   *
   * @param increment Angle (Radians)
   */
  public void incrementWristGoal(double increment) {
    // wristPIDController.setGoal(wristPIDController.getGoal().position + increment);
    wristPIDController.setSetpoint(wristPIDController.getSetpoint() + increment);
  }

  /**
   * @param enabled True = Enable, False = Disable
   */
  public void enablePID(boolean enabled){
    isEnabled = enabled;
  }
}
