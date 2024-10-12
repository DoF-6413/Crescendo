package frc.robot.Subsystems.utbintake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();

  private PIDController utbIntakePIDController;

  /**
   * Creates an Under the Bumper (UTB) Intake, the subsystem that intakes game pieces from under the
   * bumper
   */
  public UTBIntake(UTBIntakeIO io) {
    System.out.println("[Init] Creating UTB Intake");
    this.io = io;

    utbIntakePIDController =
        new PIDController(UTBIntakeConstants.KP, UTBIntakeConstants.KI, UTBIntakeConstants.KD);

    SmartDashboard.putNumber("UTB KP", 0);
    SmartDashboard.putBoolean("UTB PID Tuning", false);
  }

  @Override
  /** Periodically updates the inputs and outputs of the UTB Intake */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake", inputs);

    if (SmartDashboard.getBoolean("UTB PID Tuning", false) == true) {
      if (UTBIntakeConstants.KP != SmartDashboard.getNumber("UTB KP", 0)) {
        UTBIntakeConstants.KP = SmartDashboard.getNumber("UTB KP", 0);
        utbIntakePIDController.setP(UTBIntakeConstants.KP);
      }

      // updates UTB Intake voltage from PID calculations
      setVoltage(
          utbIntakePIDController.calculate(
              (inputs.topUTBIntakeRPM + inputs.bottomUTBIntakeRPM) / 2));
    }
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
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the speed of the UTB Intake to a percentage of it max speed
   *
   * @param percent -1 to 1
   */
  public void setPercentSpeed(double percent) {
    io.setPercentSpeed(percent);
  }

  /**
   * Sets brake mode of the UTB Intake
   *
   * @param enable Enables brake mode if true, coast if false
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
  }

  public void setSetpoint(double setpoint) {
    utbIntakePIDController.setSetpoint(setpoint);
  }

  public boolean bothAtSetpoint() {
    return utbIntakePIDController.atSetpoint();
  }
}
