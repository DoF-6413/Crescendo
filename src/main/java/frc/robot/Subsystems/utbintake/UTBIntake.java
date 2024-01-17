package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Runs the motor for the Under the Bumper (UTB) Intake */
public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIO io;
  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  private final int index;

  public UTBIntake(UTBIntakeIO io, int index) {
    System.out.println("[Init] Creating UTB Intake");
    this.io = io;
    this.index = index;
  }

  public void periodic() {
    this.updateInputs();
    Logger.processInputs("UTBIntake" + Integer.toString(index), inputs);
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
    io.setUTBIntakeVoltage(percent * 12);
  }

  /**
   * Returns the speed of the wheels for the intake found under the bumpers in Radians per second
   */
  public double getUTBIntakeVelocityRadPerSecond() {
    return inputs.UTBIntakeVelocityRadPerSec;
    // return Units.rotationsToRadians(inputs.UTBIntakeVelocityRadPerSec) /
    // UTBIntakeConstants.GEAR_RATIO;
  }
}
