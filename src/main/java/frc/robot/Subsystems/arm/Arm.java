// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotStateConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
  private static PIDController armPIDController;
  private double setpoint = 0.0;
  private double p = 0.0;
  private double i = 0.0;
  private double d = 0.0;
  
  public Arm(ArmIO arm) {
    System.out.println("[Init] Creating arm");
    this.io = arm;
    armPIDController = new PIDController(p, i, d, RobotStateConstants.LOOP_PERIODIC_SEC);
    armPIDController.setSetpoint(setpoint);
    armPIDController.enableContinuousInput(ArmConstants.ARM_MIN_ANGLE_RAD, ArmConstants.ARM_MAX_ANGLE_RAD);
    
    SmartDashboard.putNumber("armkp", 0.0);
    SmartDashboard.putNumber("armki", 0.0);
    SmartDashboard.putNumber("armkd", 0.0);
    SmartDashboard.putNumber("armSetpoint", 0.0);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("arm", armInputs);

    if (p != SmartDashboard.getNumber("armkp", 0.0)
        || i != SmartDashboard.getNumber("armki", 0.0)
        || d != SmartDashboard.getNumber("armkd", 0.0)) {
      updatePIDController();
    }

    if (setpoint != SmartDashboard.getNumber("armSetpoint", 0.0)) {
      updateSetpoint();
    }

    // Gets the current PID values that the PID contollers are set to
    SmartDashboard.putNumber("Error", setpoint - armInputs.armTurnPositionRad);
    SmartDashboard.putNumber("armCurrentkP", armPIDController.getP());
    SmartDashboard.putNumber("armCurrentkI", armPIDController.getI());
    SmartDashboard.putNumber("armCurrentkD", armPIDController.getD());

    io.setArmMotorSpeed(armPIDController.calculate(armInputs.armTurnPositionRad));
  }

  public void updatePIDController() {
    p = SmartDashboard.getNumber("armkp", 0.0);
    i = SmartDashboard.getNumber("armki", 0.0);
    d = SmartDashboard.getNumber("armkd", 0.0);
    armPIDController.setPID(p, i, d);
  }

  public void updateSetpoint() {
    setpoint = SmartDashboard.getNumber("armSetpoint", 0.0);
    armPIDController.setSetpoint(setpoint);
  }

  public void updateInputs() {
    io.updateInputs(armInputs);
  }

  /** Sets speed for the arm */
  public void setArmMotorSpeed(double Speed) {
    io.setArmMotorSpeed(Speed);
  }
}
