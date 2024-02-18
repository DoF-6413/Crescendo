// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

// import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private static PIDController armPIDController;
  private double armSetpoint = 0.0;

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating arm");
    this.io = io;
    armPIDController = new PIDController(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
    armPIDController.setSetpoint(armSetpoint);
    armPIDController.setTolerance(ArmConstants.ARM_TOLERANCE_PERCENT * armSetpoint);
    armPIDController.disableContinuousInput();

    SmartDashboard.putNumber("armkp", 0.0);
    SmartDashboard.putNumber("armki", 0.0);
    SmartDashboard.putNumber("armkd", 0.0);
    SmartDashboard.putNumber("armSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Arm", armInputs);

    if (ArmConstants.ARM_KP != SmartDashboard.getNumber("armkp", 0.0)
        || ArmConstants.ARM_KI != SmartDashboard.getNumber("armki", 0.0)
        || ArmConstants.ARM_KD != SmartDashboard.getNumber("armkd", 0.0)) {
      updatePIDController();
    }

    if (armSetpoint != SmartDashboard.getNumber("armSetpoint", 0.0)) {
      updateSetpoint();
    }

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("armError", armSetpoint - armInputs.armTurnPositionRad);
    SmartDashboard.putNumber("armCurrentkP", armPIDController.getP());
    SmartDashboard.putNumber("armCurrentkI", armPIDController.getI());
    SmartDashboard.putNumber("armCurrentkD", armPIDController.getD());
    SmartDashboard.putNumber("armCurrentSetpoint", armPIDController.getSetpoint());

    io.setArmPercentSpeed(armPIDController.calculate(armInputs.armTurnPositionRad));
  }

  public void updatePIDController() {
    ArmConstants.ARM_KP = SmartDashboard.getNumber("armkp", 0.0);
    ArmConstants.ARM_KI = SmartDashboard.getNumber("armki", 0.0);
    ArmConstants.ARM_KD = SmartDashboard.getNumber("armkd", 0.0);
    armPIDController.setPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD);
  }

  public void updateSetpoint() {
    armSetpoint = SmartDashboard.getNumber("armSetpoint", 0.0);
    armPIDController.setSetpoint(armSetpoint);
  }

  public void updateInputs() {
    io.updateInputs(armInputs);
  }

  /** Sets speed for the arm */
  public void setArmPercentSpeed(double percent) {
    io.setArmPercentSpeed(percent);
  }

  /** Sets speed for the arm */
  public void setArmMotorVoltage(double volts) {
    io.setArmVoltage(volts);
  }
}
