package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {
  @AutoLog
  public static class UTBIntakeIOInputs {
    public double UTBIntakeVelocityRadPerSec = 0.0;
    public double UTBIntakeAppliedVolts = 0.0;
    public double[] UTBIntakeCurrentAmps = new double[] {};
  }

  public default void updateInputs(UTBIntakeIOInputs inputs) {}
}
