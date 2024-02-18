package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    // All the Inputs for the Right Climber Motor (Should be nearly identical to the Right Climber
    // Motor)
    public double leftClimberPositionMeters = 0.0;
    public double leftClimberVelocityMetersPerSecond = 0.0;
    public double leftClimberAppliedVolts = 0.0;
    public double[] leftClimberCurrentAmps = new double[] {};
    public double[] leftClimberTempCelcius = new double[] {};
    // All the Inputs for the Left Climber Motor (Should be nearly identical to the Left Climber
    // Motor)
    public double rightClimberPositionMeters = 0.0;
    public double rightClimberVelocityMetersPerSecond = 0.0;
    public double rightClimberAppliedVolts = 0.0;
    public double[] rightClimberCurrentAmps = new double[] {};
    public double[] rightClimberTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs for the Climber */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the voltage of BOTH Climber Motors
   *
   * @param volts [-12 to 12]
   */
  public default void setBothClimberVoltage(double volts) {}

  /**
   * Sets the voltage of the Left Climber Motor
   *
   * @param volts [-12 to 12]
   */
  public default void setLeftClimberVoltage(double volts) {}

  /**
   * Sets the voltage of the Right Climber Motor
   *
   * @param volts [-12 to 12]
   */
  public default void setRightClimberVoltage(double volts) {}

  /**
   * Sets BOTH Climber Motors to a percent of their maximum speed
   *
   * @param percent [-1 to 1]
   */
  public default void setBothClimberPercentSpeed(double percent) {}

  /**
   * Sets the Left Climber Motor to a percent of their maximum speed
   *
   * @param percent [-1 to 1]
   */
  public default void setLeftClimberPercentSpeed(double percent) {}

  /**
   * Sets Right Climber Motor to a percent of their maximum speed
   *
   * @param percent [-1 to 1]
   */
  public default void setRightClimberPercentSpeed(double percent) {}
}
