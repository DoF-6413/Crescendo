package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Under the Bumper (UTB) Intake */
public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  private static PIDController UTBintakePIDController;
  private double UTBIntakeSetpoint = 0.0;

  public UTBIntake(UTBIntakeIO io) {

    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    UTBintakePIDController =
        new PIDController(
            UTBIntakeConstants.KP,
            UTBIntakeConstants.KI,
            UTBIntakeConstants.KD
            );

    UTBintakePIDController.setTolerance(UTBIntakeSetpoint * TOLERANCE_PERCENT);
    UTBintakePIDController.setSetpoint(UTBIntakeSetpoint);

    SmartDashboard.putNumber("UTBIntakekp", 0.0);
    SmartDashboard.putNumber("UTBIntakeki", 0.0);
    SmartDashboard.putNumber("UTBIntakekd", 0.0);

    SmartDashboard.putNumber("UTBIntakeSetpoint", 0.0);
  }

  @Override
  /** Periodically updates the inputs and outputs of the UTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake", inputs);

    if (UTBIntakeConstants.UTB_INTAKE_KP != SmartDashboard.getNumber("UTBIntakekp", 0.0)
        || UTBIntakeConstants.UTB_INTAKE_KI != SmartDashboard.getNumber("UTBIntakeki", 0.0)
        || UTBIntakeConstants.UTB_INTAKE_KD != SmartDashboard.getNumber("UTBIntakekd", 0.0)) {
      updatePIDController();
    }

    if (UTBIntakeSetpoint != SmartDashboard.getNumber("UTBIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    SmartDashboard.getNumber("UTBIntakeRealSetpoint", UTBintakePIDController.getSetpoint());

    if (inputs.utbIntakeRPM < 0) {
      setUTBIntakeVoltage(0.01);
    } else {
      setUTBIntakeVoltage(UTBintakePIDController.calculateForVoltage(inputs.utbIntakeRPM, 2800));
    }
  }

  public void updatePIDController() {
    UTBIntakeConstants.UTB_INTAKE_KP = SmartDashboard.getNumber("UTBIntakekp", 0.0);
    UTBIntakeConstants.UTB_INTAKE_KI = SmartDashboard.getNumber("UTBIntakeki", 0.0);
    UTBIntakeConstants.UTB_INTAKE_KD = SmartDashboard.getNumber("UTBIntakekd", 0.0);

    UTBintakePIDController.setPID(
        UTBIntakeConstants.UTB_INTAKE_KP,
        UTBIntakeConstants.UTB_INTAKE_KI,
        UTBIntakeConstants.UTB_INTAKE_KD
        );
  }

  public void updateSetpoint() {
    UTBIntakeSetpoint = SmartDashboard.getNumber("UTBIntakeSetpoint", 0.0);
    UTBintakePIDController.setSetpoint(UTBIntakeSetpoint);
  }

  /** Updates the inputs for the UTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Sets intake voltage for the UTB Intake :) */
  public void setUTBIntakeVoltage(double volts) {
    io.setUTBIntakeVoltage(volts);
  }

  /** Sets intake speed for the UTB Intake :) */
  public void setUTBIntakePercentSpeed(double percent) {
    io.setUTBIntakePercentSpeed(percent);
  }

  /** Returns the speed of the wheels for the intake found under the bumpers in RPM */
  public double getUTBIntakeRPM() {
    return inputs.utbIntakeRPM;
  }
}
