package frc.robot.Subsystems.utbintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Under the Bumper (UTB) Intake */
public class UTBIntake extends SubsystemBase {

  private final UTBIntakeIOInputsAutoLogged inputs = new UTBIntakeIOInputsAutoLogged();
  private UTBIntakeIO UTBIntakeIO;

  public UTBIntake(UTBIntakeIO io) {
    UTBIntakeIO = io;
  }

  public void periodic() {
    UTBIntakeIO.updateInputs(inputs);
    Logger.processInputs("UTBIntake", inputs);
  }

  /**
   * Returns the speed of the wheels for the intake found under the bumpers in Radians per second
   */
  public double getUTBIntakeVelocityRadPerSecond() {
    return inputs.UTBIntakeVelocityRadPerSec; // TODO: Math???
  }
}
