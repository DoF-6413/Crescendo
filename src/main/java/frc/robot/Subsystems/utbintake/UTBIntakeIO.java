package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {
  @AutoLog
  public static class UTBIntakeIOInputs {
    public double UTBIntakeVelocityRadPerSec = 0.0;
    public double UTBIntakeAppliedVolts = 0.0;
    public double[] UTBIntakeCurrentAmps = new double[] {};
  }

  public default void updateInputs(UTBIntakeIOInputs inputs) {}

  public default void setUTBIntakeVoltage(double volts) {}
}
