// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static PIDController wristPIDController;
  private double setpoint = 0.0;
  private double tolerance = 0.05;
  private double p = 0.0;
  private double i = 0.0;
  private double d = 0.0;

  public Wrist(WristIO io) {
    System.out.println("[Init] Creating wrist");
    this.io = io;

    wristPIDController = new PIDController(p, i, d);
    wristPIDController.setSetpoint(setpoint);
    wristPIDController.setTolerance(tolerance * setpoint);
    wristPIDController.disableContinuousInput();

    SmartDashboard.putNumber("wristkp", 0.0);
    SmartDashboard.putNumber("wristki", 0.0);
    SmartDashboard.putNumber("wristkd", 0.0);
    SmartDashboard.putNumber("wristSetpoint", 0.0);  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Wrist", inputs);

    
    if (p != SmartDashboard.getNumber("wristkp", 0.0)
        || i != SmartDashboard.getNumber("wristki", 0.0)
        || d != SmartDashboard.getNumber("wristkd", 0.0)) {
      updatePIDController();
    }

    if (setpoint != SmartDashboard.getNumber("wristSetpoint", 0.0)) {
      updateSetpoint();
    }

     // Gets the current PID values that the PID contollers are set to
     SmartDashboard.putNumber("wristError", setpoint - inputs.wristPositionRad);
     SmartDashboard.putNumber("wristCurrentkP", wristPIDController.getP());
     SmartDashboard.putNumber("wristCurrentkI", wristPIDController.getI());
     SmartDashboard.putNumber("wristCurrentkD", wristPIDController.getD());
     SmartDashboard.putNumber("wristCurrentSetpoint", wristPIDController.getSetpoint());
 
     io.setWristPercentSpeed(wristPIDController.calculate(inputs.wristPositionRad)); 
  }

  
  public void updatePIDController() {
    p = SmartDashboard.getNumber("wristkp", 0.0);
    i = SmartDashboard.getNumber("wristki", 0.0);
    d = SmartDashboard.getNumber("wristkd", 0.0);
    wristPIDController.setPID(p, i, d);
  }

  public void updateSetpoint() {
    setpoint = SmartDashboard.getNumber("wristSetpoint", 0.0);
    wristPIDController.setSetpoint(setpoint);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets speed for the first motor of the wrist */
  public void setWristPercentSpeed(double percent) {
    io.setWristPercentSpeed(percent);
  }

  public void setWristMotorVoltage(double volts) {
    setWristMotorVoltage(volts);
  }
}
