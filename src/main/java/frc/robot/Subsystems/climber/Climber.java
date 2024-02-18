// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private static PIDController climberLeftPIDController;
  private static PIDController climberRightPIDController;

  private double climberSetpoint = 0.0;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");
    this.io = io;

    climberLeftPIDController = new PIDController(ClimberConstants.LEFT_CLIMBER_KP, ClimberConstants.LEFT_CLIMBER_KI,
        ClimberConstants.LEFT_CLIMBER_KD);
    climberRightPIDController = new PIDController(ClimberConstants.RIGHT_CLIMBER_KP, ClimberConstants.RIGHT_CLIMBER_KI,
        ClimberConstants.RIGHT_CLIMBER_KD);

    climberLeftPIDController.setSetpoint(climberSetpoint);
    climberRightPIDController.setSetpoint(climberSetpoint);

    // Set Tolerence
    climberLeftPIDController.setTolerance(ClimberConstants.CLIMBER_TOLERANCE * climberSetpoint);
    climberRightPIDController.setTolerance(ClimberConstants.CLIMBER_TOLERANCE * climberSetpoint);

    SmartDashboard.putNumber("climberRightkp", 0.0);
    SmartDashboard.putNumber("climberRightki", 0.0);
    SmartDashboard.putNumber("climberRightkd", 0.0);
    SmartDashboard.putNumber("climberLeftkp", 0.0);
    SmartDashboard.putNumber("climberLeftki", 0.0);
    SmartDashboard.putNumber("climberLeftkd", 0.0);
    SmartDashboard.putNumber("climberSetpoint", 0.0);
  }

  /** Periodically updates the inputs and outputs of the Climber */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Climber", inputs);

    if (ClimberConstants.RIGHT_CLIMBER_KP != SmartDashboard.getNumber("climberRightkp", 0.0)
        || ClimberConstants.RIGHT_CLIMBER_KI != SmartDashboard.getNumber("climberRightki", 0.0)
        || ClimberConstants.RIGHT_CLIMBER_KD != SmartDashboard.getNumber("climberRightkd", 0.0)
        || ClimberConstants.LEFT_CLIMBER_KP != SmartDashboard.getNumber("climberLeftkp", 0.0)
        || ClimberConstants.LEFT_CLIMBER_KI != SmartDashboard.getNumber("climberLeftki", 0.0)
        || ClimberConstants.LEFT_CLIMBER_KD != SmartDashboard.getNumber("climberLeftkd", 0.0)) {
      updatePIDController();
    }

    if (climberSetpoint != SmartDashboard.getNumber("climberSetpoint", 0.0)) {
      updateSetpoint();
    }

    io.setRightClimberVoltage(
        climberRightPIDController.calculate(inputs.rightClimberPositionRad, climberSetpoint));

    io.setLeftClimberVoltage(
        climberLeftPIDController.calculate(inputs.leftClimberPositionRad, climberSetpoint));
  }

  public void updatePIDController() {
    ClimberConstants.LEFT_CLIMBER_KP = SmartDashboard.getNumber("climberLeftkp", 0.0);
    ClimberConstants.LEFT_CLIMBER_KD = SmartDashboard.getNumber("climberLeftkd", 0.0);
    ClimberConstants.LEFT_CLIMBER_KI = SmartDashboard.getNumber("climberLeftki", 0.0);
    ClimberConstants.RIGHT_CLIMBER_KP = SmartDashboard.getNumber("climberRightkp", 0.0);
    ClimberConstants.RIGHT_CLIMBER_KI = SmartDashboard.getNumber("climberRightki", 0.0);
    ClimberConstants.RIGHT_CLIMBER_KD = SmartDashboard.getNumber("climberRightkd", 0.0);

    climberLeftPIDController.setPID(ClimberConstants.LEFT_CLIMBER_KP, ClimberConstants.LEFT_CLIMBER_KI,
        ClimberConstants.LEFT_CLIMBER_KD);
    climberRightPIDController.setPID(ClimberConstants.RIGHT_CLIMBER_KP, ClimberConstants.RIGHT_CLIMBER_KI,
        ClimberConstants.RIGHT_CLIMBER_KD);
  }

  public void updateSetpoint() {
    climberSetpoint = SmartDashboard.getNumber("climberRightSetpoint", 0.0);
    climberRightPIDController.setSetpoint(climberSetpoint);
  }

  /** Updates the inputs for the Climber */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setBothClimberVoltage(double volts) {
    io.setBothClimberVoltage(volts);
  }

  public void setLeftClimberVoltage(double volts) {
    io.setLeftClimberVoltage(volts);
  }

  public void setRightClimberVoltage(double volts) {
    io.setRightClimberVoltage(volts);
  }

  public void setBothClimberPercentSpeed(double percent) {
    io.setBothClimberPercentSpeed(percent);
  }

  public void setLeftClimberPercentSpeed(double percent) {
    io.setLeftClimberPercentSpeed(percent);
  }

  public void setRightClimberPercentSpeed(double percent) {
    io.setRightClimberPercentSpeed(percent);
  }
}
