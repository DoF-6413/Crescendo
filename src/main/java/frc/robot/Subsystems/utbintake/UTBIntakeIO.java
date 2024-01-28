package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {
  @AutoLog
  public static class UTBIntakeIOInputs {
    public double utbIntakeVelocityRadPerSec = 0.0;
    public double utbIntakeAppliedVolts = 0.0;
    public double[] utbIntakeCurrentAmps = new double[] {};
  }

  public default void updateInputs(UTBIntakeIOInputs inputs) {}

  public default void setUTBIntakeVoltage(double volts) {}

  public default void setUTBIntakePercentSpeed(double percent) {}
}
