// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Over the Bumper (OTB) Intake */
public class OTBIntake extends SubsystemBase {
  private final OTBIntakeIO io;
  private final OTBIntakeIOInputsAutoLogged inputs = new OTBIntakeIOInputsAutoLogged();

  private final PIDController otbIntakePID;

  private double setpointRPM = 0.0;

  public OTBIntake(OTBIntakeIO io) {
    System.out.println("[Init] Creating OTB Intake");
    this.io = io;
    otbIntakePID =
        new PIDController(OTBIntakeConstants.KP, OTBIntakeConstants.KI, OTBIntakeConstants.KD);
    otbIntakePID.setSetpoint(setpointRPM);
    otbIntakePID.setTolerance(setpointRPM * OTBIntakeConstants.TOLERANCE);

    SmartDashboard.putNumber("OTBIntakekP", 0.0);
    SmartDashboard.putNumber("OTBIntakekI", 0.0);
    SmartDashboard.putNumber("OTBIntakekD", 0.0);
    SmartDashboard.putNumber("OTBIntakeSetpoint", 0.0);
  }

  /** Periodically updates the inputs and outputs of the OTB Intake */
  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("OTBIntake", inputs);

    if (OTBIntakeConstants.KP != SmartDashboard.getNumber("OTBIntakekP", 0)
        || OTBIntakeConstants.KI != SmartDashboard.getNumber("OTBIntakekI", 0)
        || OTBIntakeConstants.KD != SmartDashboard.getNumber("OTBIntakekD", 0)) {
      updatePIDController();
    }

    if (setpointRPM != SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    SmartDashboard.putNumber("Get Setpoint OTB Intake", otbIntakePID.getSetpoint());
    SmartDashboard.putNumber("Get OTB kP", otbIntakePID.getP());
    SmartDashboard.putNumber("Get OTB kI", otbIntakePID.getI());
    SmartDashboard.putNumber("Get OTB kD", otbIntakePID.getD());

    SmartDashboard.putNumber("OTBIntakeError", setpointRPM - inputs.otbIntakeVelocityRPM);
    SmartDashboard.putBoolean("OTB At Setpoint", atSetpoint());

    if (inputs.otbIntakeVelocityRPM < 0) {
      setOTBIntakeVoltage(0.0);
    } else {
      setOTBIntakeVoltage(
          otbIntakePID.calculateForVoltage(
              inputs.otbIntakeVelocityRPM, 1400)); // maxValue based on sim
    }
  }

  public void updatePIDController() {
    OTBIntakeConstants.KP = SmartDashboard.getNumber("OTBIntakekP", 0);
    OTBIntakeConstants.KI = SmartDashboard.getNumber("OTBIntakekI", 0);
    OTBIntakeConstants.KD = SmartDashboard.getNumber("OTBIntakekD", 0);
    otbIntakePID.setPID(OTBIntakeConstants.KP, OTBIntakeConstants.KI, OTBIntakeConstants.KD);
  }

  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0);
    otbIntakePID.setSetpoint(setpointRPM);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void setOTBIntakeVoltage(double volts) {
    io.setOTBIntakeVoltage(volts);
  }

  public void setOTBIntakePercentSpeed(double percent) {
    io.setOTBIntakePercentSpeed(percent);
  }

  public boolean atSetpoint() {
    return otbIntakePID.atSetpoint(inputs.otbIntakeVelocityRPM);
  }
}
