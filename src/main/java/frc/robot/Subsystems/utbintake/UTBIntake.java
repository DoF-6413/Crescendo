package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.PIDController;
import org.littletonrobotics.junction.Logger;

public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();

  /** UTB Intake PID controller */
  private final PIDController utbIntakePIDController;

  private double utbIntakeSetpoint = 0.0;

  /**
   * Creates an Under the Bumper (UTB) Intake, the subsystem that intakes game pieces from under the
   * bumper
   */
  public UTBIntake(UTBIntakeIO io) {
    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    /** Creates a new PIDController for the UTB Intake */
    utbIntakePIDController =
        new PIDController(UTBIntakeConstants.KP, UTBIntakeConstants.KI, UTBIntakeConstants.KD);

    utbIntakePIDController.setSetpoint(utbIntakeSetpoint);
    /** Sets tolerance and setpoint for UTB Intake PIDController */
    utbIntakePIDController.setTolerance(utbIntakeSetpoint * UTBIntakeConstants.TOLERANCE_PERCENT);
  }

  @Override
  /** Periodically updates the inputs and outputs of the UTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake", inputs);

    // updates UTB Intake voltage from PID calculations
    // setUTBIntakeVoltage(
    //     utbIntakePIDController.calculateForVoltage(
    //         inputs.utbIntakeRPM, UTBIntakeConstants.MAX_RPM));
  }

  /** Updates the inputs for the UTB Intake */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /**
   * Sets UTB Intake voltage for the UTB Intake
   *
   * @param volts -12 to 12
   */
  public void setUTBIntakeVoltage(double volts) {
    io.setUTBIntakeVoltage(volts);
  }

  /**
   * Sets the speed of the UTB Intake to a percentage of it max speed
   *
   * @param percent -1 to 1
   */
  public void setUTBIntakePercentSpeed(double percent) {
    io.setUTBIntakePercentSpeed(percent);
  }

  /**
   * Sets brake mode of the UTB Intake
   *
   * @param enable Enables brake mode if true, coast if false
   */
  public void setUTBIntakeBrakeMode(boolean enable) {
    io.setUTBIntakeBrakeMode(enable);
  }

  /**
   * Sets the UTB Intake PID setpoint
   *
   * @param setpoint RPM
   */
  public void setUTBSetpoint(double setpoint) {
    utbIntakePIDController.setSetpoint(setpoint);
    utbIntakePIDController.setTolerance(setpoint * UTBIntakeConstants.TOLERANCE_PERCENT);
  }

  public double getCurrentDraw() {
    return inputs.utbIntakeCurrentAmps[0];
  }
}
