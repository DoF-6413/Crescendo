package frc.robot.Subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the Feeder */
public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    /** Velocity of the Feeder Motor in Rotations per Minute */
    public double feederRPM = 0.0;
    /** Number of volts being sent to the Feeder Motor */
    public double feederAppliedVolts = 0.0;
    /** Number of amps used by the Feeder motor */
    public double[] feederCurrentAmps = new double[] {};
    /** Tempature of the Feeder motor in Celsius */
    public double[] feederTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for both Shooter Motors */
  public default void updateInputs(FeederIOInputs inputs) {}

  /**
   * Sets the voltage of the Feeder motor
   *
   * @param volts [-12 to 12]
   */
  public default void setFeederVoltage(double volts) {}

  /**
   * Sets the speed of the Feeder motor based on a percent of its maximum speed
   *
   * @param percent [-1 to 1]
   */
  public default void setFeederPercentSpeed(double percent) {}

  /**
   * Sets the Feeder motor to brake mode
   *
   * @param enable
   */
  public default void setFeederBrakeMode(boolean enable) {}
}
