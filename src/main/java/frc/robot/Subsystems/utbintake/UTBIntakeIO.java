package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the UTB Intake */
public interface UTBIntakeIO {

  @AutoLog
  public static class UTBIntakeIOInputs {
    /** The velocity of the UTB Intake in Rotations per Minute */
    public double topUTBIntakeRPM = 0.0;
    /** Number of volts being sent to the UTB Intake motor */
    public double topUTBIntakeAppliedVolts = 0.0;
    /** Number of amps being used by the UTB Intake motor */
    public double topUTBIntakeCurrentAmps = 0.0;
    /** Tempature, in Celsius, of the UTB Intake motor */
    public double topUTBIntakeTempCelsius = 0.0;

    /** The velocity of the UTB Intake in Rotations per Minute */
    public double bottomUTBIntakeRPM = 0.0;
    /** Number of volts being sent to the UTB Intake motor */
    public double bottomUTBIntakeAppliedVolts = 0.0;
    /** Number of amps being used by the UTB Intake motor */
    public double bottomUTBIntakeCurrentAmps = 0.0;
    /** Tempature, in Celsius, of the UTB Intake motor */
    public double bottomUTBIntakeTempCelsius = 0.0;
  }

  /**
   * Sets UTB intake Percent Speed
   *
   * @param percent -1 to 1
   */
  public default void updateInputs(UTBIntakeIOInputs inputs) {}

  /**
   * Sets UTB intake Voltage
   *
   * @param volts -12 to 12
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets UTB intake Voltage
   *
   * @param volts -12 to 12
   */
  public default void setPercentSpeed(double percent) {}

  /**
   * Sets brake mode of the UTB Intake
   *
   * @param enable Enables brake mode if true, coast if false
   */
  public default void setBrakeMode(boolean enable) {}
}
