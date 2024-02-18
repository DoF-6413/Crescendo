// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class OTBIntake extends SubsystemBase {
  private final OTBIntakeIO io;
  private final OTBIntakeIOInputsAutoLogged inputs = new OTBIntakeIOInputsAutoLogged();

  private final PIDController otbIntakePID;
  
  private double setpointRPM = 0.0;
  
  /** Runs the real life motor for the Over the Bumper (OTB) Intake with CAN SPARKMAX Speed Contollers and Neo motor*/
  public OTBIntake(OTBIntakeIO io) {
    System.out.println("[Init] Creating OTB Intake");
    this.io = io;
    otbIntakePID =
        new PIDController(OTBIntakeConstants.KP, OTBIntakeConstants.KI, OTBIntakeConstants.KD);
    otbIntakePID.setSetpoint(setpointRPM);
    otbIntakePID.setTolerance(setpointRPM * OTBIntakeConstants.TOLERANCE_PERCENT);

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

    otbIntakePID.calculateForVoltage(inputs.otbIntakeVelocityRPM, OTBIntakeConstants.MAX_VALUE);

    if (OTBIntakeConstants.KP != SmartDashboard.getNumber("OTBIntakekP", 0)
        || OTBIntakeConstants.KI != SmartDashboard.getNumber("OTBIntakekI", 0)
        || OTBIntakeConstants.KD != SmartDashboard.getNumber("OTBIntakekD", 0)) {
      updatePIDController();
    }

    if (setpointRPM != SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    // Outputs the setpoint and PID values onto the SmartDashboard to ensure they get updated
    SmartDashboard.putNumber("OTBIntakeGetSetpoint", otbIntakePID.getSetpoint());
    SmartDashboard.putNumber("OTBIntakeGetkP", otbIntakePID.getP());
    SmartDashboard.putNumber("OTBIntakeGetkI", otbIntakePID.getI());
    SmartDashboard.putNumber("OTBIntakeGetkD", otbIntakePID.getD());

    SmartDashboard.putNumber("OTBIntakeError", setpointRPM - inputs.otbIntakeVelocityRPM);
    SmartDashboard.putBoolean("OTBIntakeAtSetpoint", atSetpoint());
  }

  /** Updates the PID values based on what is put on Shuffleboard */
  public void updatePIDController() {
    OTBIntakeConstants.KP = SmartDashboard.getNumber("OTBIntakekP", 0);
    OTBIntakeConstants.KI = SmartDashboard.getNumber("OTBIntakekI", 0);
    OTBIntakeConstants.KD = SmartDashboard.getNumber("OTBIntakekD", 0);
    otbIntakePID.setPID(OTBIntakeConstants.KP, OTBIntakeConstants.KI, OTBIntakeConstants.KD);
  }

  /** Updates the setpoint based on what is put on Shuffleboard */
  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("OTBIntakeSetpoint", 0.0);
    otbIntakePID.setSetpoint(setpointRPM);
  }

  /** Updates inputs for the OTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the voltage for the OTB Intake
   *
   * @param volts [-12 to 12]
   */
  public void setOTBIntakeVoltage(double volts) {
    io.setOTBIntakeVoltage(volts);
  }

  /**
   * Sets the speed for the OTB Intake
   *
   * @param percent [-1 to 1]
   */
  public void setOTBIntakePercentSpeed(double percent) {
    io.setOTBIntakePercentSpeed(percent);
  }

  /**
   * Sets the Brake Mode for the OTB Intake 
   * <p>Brake means motor holds position, Coast means easy to move
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Returns where the OTB Intake RPM is within the setpoint, including tolerance */
  public boolean atSetpoint() {
    return otbIntakePID.atSetpoint(inputs.otbIntakeVelocityRPM);
  }
}
