// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private final ProfiledPIDController armPIDController;
  private SimpleMotorFeedforward armFeedforward;
  private final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static GenericEntry armkP;
  private static GenericEntry armkI;
  private static GenericEntry armkD;
  private static GenericEntry armMaxVelocity;
  private static GenericEntry armMaxAcceleration;
  private static GenericEntry armkS;
  private static GenericEntry armkV;
  private static GenericEntry armkA;
  private static GenericEntry armGoal;
  private static double goal = 0.0;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;

    // Initalizing the Arm PID Contoller
    armPIDController =
        new ProfiledPIDController(
            ArmConstants.KP,
            ArmConstants.KI,
            ArmConstants.KD,
            new TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION));
    armPIDController.setGoal(0);
    armPIDController.setTolerance(ArmConstants.ANGLE_TOLERANCE);
    armPIDController.disableContinuousInput();

    // Initalizing the Arm FF Controller
    armFeedforward = new SimpleMotorFeedforward(ArmConstants.KS, ArmConstants.KV, ArmConstants.KA);

    // TODO: Delete once final PID Numbers are Decided
    armkP = armTab.add("armkp", 1.0).getEntry();
    armkI = armTab.add("armki", 0.0).getEntry();
    armkD = armTab.add("armkd", 0.0).getEntry();
    armGoal = armTab.add("armGoal", 0.0).getEntry();
    armMaxVelocity = armTab.add("armMaxVelocity", 0.0).getEntry();
    armMaxAcceleration = armTab.add("armMaxAcceleration", 0.0).getEntry();
    armkS = armTab.add("armks", 0.0).getEntry();
    armkV = armTab.add("armkV", 0.0).getEntry();
    armkA = armTab.add("armkA", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // updates the inputs
    this.updateInputs();
    // log the inputs
    Logger.processInputs("Arm", armInputs);

    // Updates Arm Speed based on PID Control
    setArmPercentSpeed(
        armPIDController.calculate(armInputs.armAbsolutePositionRad)
            + (armFeedforward.calculate(armInputs.armVelocityRadPerSec)
                / RobotStateConstants
                    .BATTERY_VOLTAGE)); // Feedforward divided by 12 since it returns a voltage

    if (ArmConstants.KP != armkP.getDouble(0.0)
        || ArmConstants.KI != armkI.getDouble(0.0)
        || ArmConstants.KD != armkD.getDouble(0.0)) {
      updatePIDController();
    }

    if (ArmConstants.KS != armkS.getDouble(0.0)
        || ArmConstants.KV != armkV.getDouble(0.0)
        || ArmConstants.KA != armkA.getDouble(0.0)) {
      updateFFController();
    }

    if (goal != armGoal.getDouble(0.0)) {
      updateGoal();
    }

    if (ArmConstants.MAX_VELOCITY != armMaxVelocity.getDouble(0.0)
        || ArmConstants.MAX_ACCELERATION != armMaxAcceleration.getDouble(0.0)) {
      updateTrapezoidalConstraints();
    }
  }

  /** Updates the PID values for the Arm from ShuffleBoard */
  public void updatePIDController() {
    ArmConstants.KP = armkP.getDouble(0.0);
    ArmConstants.KP = armkI.getDouble(0.0);
    ArmConstants.KP = armkD.getDouble(0.0);
    armPIDController.setPID(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
  }

  /** Updates the Goal from ShuffleBoard */
  public void updateGoal() {
    goal = armGoal.getDouble(0.0);
    armPIDController.setGoal(goal);
  }

  /** Updates the Trapezoidal Constraints from the ShuffleBoard */
  public void updateTrapezoidalConstraints() {
    ArmConstants.MAX_VELOCITY = armMaxVelocity.getDouble(0.0);
    ArmConstants.MAX_ACCELERATION = armMaxAcceleration.getDouble(0.0);
    armPIDController.setConstraints(
        new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION));
  }

  /** Updates the FF values from Shuffleboard */
  public void updateFFController() {
    ArmConstants.KS = armkS.getDouble(0.0);
    ArmConstants.KV = armkV.getDouble(0.0);
    ArmConstants.KA = armkA.getDouble(0.0);
    armFeedforward = new SimpleMotorFeedforward(ArmConstants.KS, ArmConstants.KV, ArmConstants.KA);
  }

  /** Updates the Outputs of the Motors based on What Mode we are In */
  public void updateInputs() {
    io.updateInputs(armInputs);
  }

  /**
   * Sets the Arm motor to a percent of its maximum speed
   *
   * @param percent -1 to 1
   */
  public void setArmPercentSpeed(double percent) {
    io.setArmPercentSpeed(percent);
  }
  /**
   * Sets the voltage of the Arm motor
   *
   * @param volts -12 to 12
   */
  public void setArmMotorVoltage(double volts) {

    io.setArmVoltage(volts);
  }

  /**
   * Sets the Brake Mode for the Arm (Brake means motor holds position, Coast means easy to move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  /**
   * Updates the angle that the arm should be at using the WPI PID controller
   *
   * @param goal Angle (Radians)
   */
  public void setGoal(double goal) {
    armPIDController.setGoal(goal);
  }

  public double getGoal() {
    return armPIDController.getGoal().position;
  }
  /**
   * Changes the angle goal of the Arm
   *
   * @param increment Angle (Radians)
   */
  public void incrementArmGoal(double increment) {
    armPIDController.setGoal(armPIDController.getGoal().position + increment);
  }

  /** Returns whether the arm is at its goal or not */
  public boolean atGoal() {
    return armPIDController.atGoal();
  }
}
