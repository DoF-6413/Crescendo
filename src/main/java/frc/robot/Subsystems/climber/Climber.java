// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final PIDController climberLeftPIDController;
  private final PIDController climberRightPIDController;
  private double climberSetpoint = 0.0;
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  private GenericEntry climberLeftkp;
  private GenericEntry climberLeftki;
  private GenericEntry climberLeftkd;
  private GenericEntry climberRightkp;
  private GenericEntry climberRightki;
  private GenericEntry climberRightkd;
  private GenericEntry climberSetpointSetter;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    System.out.println("[Init] Creating Climber");
    this.io = io;

    climberLeftPIDController =
        new PIDController(
            ClimberConstants.LEFT_KP, ClimberConstants.LEFT_KI, ClimberConstants.LEFT_KD);
    climberRightPIDController =
        new PIDController(
            ClimberConstants.RIGHT_KP, ClimberConstants.RIGHT_KI, ClimberConstants.RIGHT_KD);

    climberLeftPIDController.setSetpoint(climberSetpoint);
    climberRightPIDController.setSetpoint(climberSetpoint);

    // Set Tolerance
    climberLeftPIDController.setTolerance(ClimberConstants.TOLERANCE_PERCENT * climberSetpoint);
    climberRightPIDController.setTolerance(ClimberConstants.TOLERANCE_PERCENT * climberSetpoint);
    
    // TODO: Delete once final PID Numbers are Decided
    climberLeftkp = climberTab.add("climberLeftkp", 0.0).getEntry();
    climberLeftki = climberTab.add("climberLeftki", 0.0).getEntry();
    climberLeftkd = climberTab.add("climberLeftkd", 0.0).getEntry();
    climberRightkp = climberTab.add("climberRightkp", 0.0).getEntry();
    climberRightki = climberTab.add("climberRightki", 0.0).getEntry();
    climberRightkd = climberTab.add("climberRightkd", 0.0).getEntry();
    climberSetpointSetter = climberTab.add("climberSetpoint", 0.0).getEntry();
  }
  
  /** Periodically updates the inputs and outputs of the Climber */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Climber", inputs);

    io.setLeftClimberPercentSpeed(
        climberLeftPIDController.calculate(inputs.leftClimberPositionMeters));
  
    io.setRightClimberPercentSpeed(
        climberRightPIDController.calculate(inputs.rightClimberPositionMeters));

    if (ClimberConstants.RIGHT_KP != climberRightkp.getDouble(0.0)
        || ClimberConstants.RIGHT_KI != climberRightki.getDouble(0.0)
        || ClimberConstants.RIGHT_KD != climberRightkd.getDouble(0.0)
        || ClimberConstants.LEFT_KP != climberLeftkp.getDouble(0.0)
        || ClimberConstants.LEFT_KI != climberLeftki.getDouble(0.0)
        || ClimberConstants.LEFT_KD != climberLeftkd.getDouble(0.0)) {
      updatePIDController();
    }

    if (climberSetpoint != climberSetpointSetter.getDouble(0.0)) {
      updateSetpoint();
    }
  }

  public void updatePIDController() {
    ClimberConstants.LEFT_KP = climberLeftkp.getDouble(0.0);
    ClimberConstants.LEFT_KD = climberLeftkd.getDouble(0.0);
    ClimberConstants.LEFT_KI = climberLeftki.getDouble(0.0);
    ClimberConstants.RIGHT_KP = climberRightkp.getDouble(0.0);
    ClimberConstants.RIGHT_KI = climberRightki.getDouble(0.0);
    ClimberConstants.RIGHT_KD = climberRightkd.getDouble(0.0);
    climberLeftPIDController.setPID(
        ClimberConstants.LEFT_KP, ClimberConstants.LEFT_KI, ClimberConstants.LEFT_KD);
    climberRightPIDController.setPID(
        ClimberConstants.RIGHT_KP, ClimberConstants.RIGHT_KI, ClimberConstants.RIGHT_KD);
  }

  public void updateSetpoint() {
    climberSetpoint = climberSetpointSetter.getDouble(0.0);
    climberLeftPIDController.setSetpoint(climberSetpoint);
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
