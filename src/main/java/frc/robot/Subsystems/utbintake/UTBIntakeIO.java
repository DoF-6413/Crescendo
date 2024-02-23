package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {
  @AutoLog
  public static class UTBIntakeIOInputs {
    /** The velocity of the UTB Intake in Rotations per Minute */
    public double utbIntakeRPM = 0.0;
    /** Number of volts being sent to the UTB Intake motor */
    public double utbIntakeAppliedVolts = 0.0;
    /** Number of amps being used by the UTB Intake motor */
    public double[] utbIntakeCurrentAmps = new double[] {};
    /** Tempature, in Celsius, of the UTB Intake motor */
    public double[] utbIntakeTempCelsius = new double[] {};
  }

  /**
   * Sets UTB intake Percent Speed
   *
   * @param percent [-1 to 1]
   */
  public default void updateInputs(UTBIntakeIOInputs inputs) {}

  /**
   * Sets UTB intake Voltage
   *
   * @param volts [-12 to 12]
   */
  public default void setUTBIntakeVoltage(double volts) {}

  /**
   * Sets UTB intake Voltage
   *
   * @param volts [-12 to 12]
   */
  public default void setUTBIntakePercentSpeed(double percent) {}

  /**
   * Sets brake mode
   *
   * @param isEnabled boolean for is brake mode true or false
   */
  public default void setUTBIntakeBrakeMode(boolean isEnabled) {}

  public default void enableUTB(boolean auxYIsPressed) {}
}
