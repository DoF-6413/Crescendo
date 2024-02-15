package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UTBIntakeConstants;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Under the Bumper (UTB) Intake */
public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  private static PIDController UTBintakePIDController;
  private double setpointRPM = 0.0;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  public UTBIntake(UTBIntakeIO io) {

    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    UTBintakePIDController =
        new PIDController(
            kP, kI, kD
            // UTBIntakeConstants.UTB_INTAKE_KP,
            // UTBIntakeConstants.UTB_INTAKE_KI,
            // UTBIntakeConstants.UTB_INTAKE_KD
            );

    UTBintakePIDController.setTolerance(setpointRPM * UTBIntakeConstants.UTB_INTAKE_TOLERANCE);
    UTBintakePIDController.setSetpoint(setpointRPM);

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

    if (kP != SmartDashboard.getNumber("UTBIntakekp", 0.0)
        || kI != SmartDashboard.getNumber("UTBIntakeki", 0.0)
        || kD != SmartDashboard.getNumber("UTBIntakekd", 0.0)) {
      updatePIDController();
    }

    if (setpointRPM != SmartDashboard.getNumber("UTBIntakeSetpoint", 0.0)) {
      updateSetpoint();
    }

    SmartDashboard.getNumber("UTBIntakeRealSetpoint", UTBintakePIDController.getSetpoint());

    if (inputs.utbIntakeRPM < 0) {
      setUTBIntakeVoltage(0);
    } else {
      setUTBIntakeVoltage(UTBintakePIDController.calculateForVoltage(inputs.utbIntakeRPM, 2800));
    }
    System.out.println(UTBintakePIDController.calculateForVoltage(inputs.utbIntakeRPM, 2800));
  }

  public void updatePIDController() {
    // UTBIntakeConstants.UTB_INTAKE_KP = SmartDashboard.getNumber("UTBIntakekp", 0.0);
    // UTBIntakeConstants.UTB_INTAKE_KI = SmartDashboard.getNumber("UTBIntakeki", 0.0);
    // UTBIntakeConstants.UTB_INTAKE_KD = SmartDashboard.getNumber("UTBIntakekd", 0.0);

    kP = SmartDashboard.getNumber("UTBIntakekp", 0.0);
    kI = SmartDashboard.getNumber("UTBIntakeki", 0.0);
    kD = SmartDashboard.getNumber("UTBIntakekd", 0.0);

    UTBintakePIDController.setPID(
        kP, kI, kD
        // UTBIntakeConstants.UTB_INTAKE_KP,
        // UTBIntakeConstants.UTB_INTAKE_KI,
        // UTBIntakeConstants.UTB_INTAKE_KD
        );
  }

  public void updateSetpoint() {
    setpointRPM = SmartDashboard.getNumber("UTBIntakeSetpoint", 0.0);
    UTBintakePIDController.setSetpoint(setpointRPM);
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
