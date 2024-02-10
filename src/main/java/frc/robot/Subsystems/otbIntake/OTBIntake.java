// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OTBIntakeConstants;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Over the Bumper (OTB) Intake */
public class OTBIntake extends SubsystemBase {
  private final OTBIntakeIO io;
  private final OTBIntakeIOInputsAutoLogged inputs = new OTBIntakeIOInputsAutoLogged();
  private static PIDController otbIntakePID;
  private double setpointRPM = 0.0;

  public OTBIntake(OTBIntakeIO io) {
    System.out.println("[Init] Creating OTB Intake");
    this.io = io;
    otbIntakePID =
        new PIDController(
            OTBIntakeConstants.OTB_INTAKE_KP,
            OTBIntakeConstants.OTB_INTAKE_KI,
            OTBIntakeConstants.OTB_INTAKE_KD);
    otbIntakePID.setTolerance(setpointRPM * OTBIntakeConstants.OTB_INTAKE_TOLERANCE);

    SmartDashboard.putNumber("OTBIntakekp", 0.0);
    SmartDashboard.putNumber("OTBIntakeki", 0.0);
    SmartDashboard.putNumber("OTBIntakekd", 0.0);

    SmartDashboard.putNumber("OTBIntakeSetpoint", 0.0);
  }

  /** Periodically updates the inputs and outputs of the OTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("OTBIntake", inputs);

    if (OTBIntakeConstants.OTB_INTAKE_KP != SmartDashboard.getNumber("OTBIntakekp", 0)
        || OTBIntakeConstants.OTB_INTAKE_KI != SmartDashboard.getNumber("otbIntakeki", 0)
        || OTBIntakeConstants.OTB_INTAKE_KD != SmartDashboard.getNumber("otbIntakekd", 0)) {
      updatePIDController();
    }

    if (setpointRPM != SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    SmartDashboard.putNumber("Setpoint OTB Intake", setpointRPM);
    SmartDashboard.putNumber("OTBIntakeError", setpointRPM - getOTBIntakeRPM());

    io.setOTBIntakeVoltage(otbIntakePID.calculateForVoltage(getOTBIntakeRPM(), setpointRPM));
  }

  public void updatePIDController() {
    OTBIntakeConstants.OTB_INTAKE_KP = SmartDashboard.getNumber("OTBIntakekp", 0);
    OTBIntakeConstants.OTB_INTAKE_KI = SmartDashboard.getNumber("OTBIntakeki", 0);
    OTBIntakeConstants.OTB_INTAKE_KD = SmartDashboard.getNumber("OTBIntakekd", 0);
    otbIntakePID =
        new PIDController(
            OTBIntakeConstants.OTB_INTAKE_KP,
            OTBIntakeConstants.OTB_INTAKE_KI,
            OTBIntakeConstants.OTB_INTAKE_KD);
  }

  /** Updates inputs for the OTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the OTB Intake */
  public void setOTBIntakeVoltage(double volts) {
    io.setOTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the OTB Intake */
  public void setOTBIntakePercentSpeed(double percent) {
    io.setOTBIntakePercentSpeed(percent);
  }

  public double getOTBIntakeRPM() {
    return inputs.otbIntakeMotorRPM;
  }

  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0);
    otbIntakePID.setSetpoint(setpointRPM);
  }
}
