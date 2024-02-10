package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {
  @AutoLog
  public static class UTBIntakeIOInputs {
    public double utbIntakeRPM = 0.0;
    public double utbIntakeAppliedVolts = 0.0;
    public double[] utbIntakeCurrentAmps = new double[] {};
    public double UTBIntakepositionrad = 0.0;
    public double UTBintakeRPM = 0.0;
  }

  public default void updateInputs(UTBIntakeIOInputs inputs) {}

  public default void setUTBIntakeVoltage(double volts) {}

  public default void setUTBIntakePercentSpeed(double percent) {}
}
