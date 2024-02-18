package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  private static PIDController utbIntakePIDController;
  private double utbIntakeSetpoint = 0.0;

  /**
   * creates an Under the Bumper (UTB) Intake, the subsystem that intakes pieces from under the
   * bumper
   */
  public UTBIntake(UTBIntakeIO io) {

    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    /** creates a new PIDController for the UTBIntake */
    utbIntakePIDController =
        new PIDController(
          UTBIntakeConstants.KP,
          UTBIntakeConstants.KI,
          UTBIntakeConstants.KD);

    /** sets tolerance and setpoint for UTBIntake PIDController */
    utbIntakePIDController.setTolerance(utbIntakeSetpoint * UTBIntakeConstants.TOLERANCE_PERCENT);
    utbIntakePIDController.setSetpoint(utbIntakeSetpoint);

    SmartDashboard.putNumber("utbIntakekp", 0.0);
    SmartDashboard.putNumber("utbIntakeki", 0.0);
    SmartDashboard.putNumber("utbIntakekd", 0.0);

    SmartDashboard.putNumber("utbIntakeSetpoint", 0.0);
  }

  @Override
  /** Periodically updates the inputs and outputs of the utb Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("utbIntake", inputs);

    //updates UTB Intake voltage from PID calculations
    setUTBIntakeVoltage(
        utbIntakePIDController.calculateForVoltage(
            inputs.utbIntakeRPM, UTBIntakeConstants.MAX_RPM));

    //TODO: delete once PID values are finalized
    if (UTBIntakeConstants.KP != SmartDashboard.getNumber("utbIntakekp", 0.0)
        || UTBIntakeConstants.KI != SmartDashboard.getNumber("utbIntakeki", 0.0)
        || UTBIntakeConstants.KD != SmartDashboard.getNumber("utbIntakekd", 0.0)) {
      updatePIDController();
    }

    //TODO: delete once PID values are finalized
    if (utbIntakeSetpoint != SmartDashboard.getNumber("utbIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    SmartDashboard.getNumber("utbIntakeRealSetpoint", utbIntakePIDController.getSetpoint());
  }

  /** updates PID values if SmartDashboard gets updated */
  public void updatePIDController() {
    UTBIntakeConstants.KP = SmartDashboard.getNumber("utbIntakekp", 0.0);
    UTBIntakeConstants.KI = SmartDashboard.getNumber("utbIntakeki", 0.0);
    UTBIntakeConstants.KD = SmartDashboard.getNumber("utbIntakekd", 0.0);

    utbIntakePIDController.setPID(
        UTBIntakeConstants.KP,
        UTBIntakeConstants.KI,
        UTBIntakeConstants.KD);
  }

  /** updates setpoint if SmartDashboard gets updated */
  public void updateSetpoint() {
    utbIntakeSetpoint = SmartDashboard.getNumber("utbIntakeSetpoint", 0.0);
    utbIntakePIDController.setSetpoint(utbIntakeSetpoint);
  }

  /** Updates the inputs for the utb Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the utb Intake :) */
  public void setUTBIntakeVoltage(double volts) {
    io.setUTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the utb Intake :) */
  public void setUTBIntakePercentSpeed(double percent) {
    io.setUTBIntakePercentSpeed(percent);
  }

  /** Returns the speed of the wheels for the intake found under the bumpers in RPM */
  public double getUTBIntakeRPM() {
    return inputs.utbIntakeRPM;
  }
}
