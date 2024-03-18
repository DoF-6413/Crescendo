package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    /** Returns the voltage the Climber Recieves */
    public double climberAppliedVolts = 0.0;
    /** Returns the position of the Climber in Meters */
    public double climberPositionMeters = 0.0;
    /** Returns the velocity of the Climber in m/s */
    public double climberVelocityMetersPerSecond = 0.0;
    /** The Current Drawn from the Climber in Amps */
    public double[] climberCurrentAmps = new double[] {};
    /** The Temperature from the Climber in Celsius */
    public double[] climberTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for the Climber */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the voltage of Climber Motors
   *
   * @param volts -12 to 12
   */
  public default void setClimberVoltage(double volts) {}

  /**
   * Sets the voltage of the Left Climber Motor
   *
   * @param volts -12 to 12
   */
  public default void setLeftClimberVoltage(double volts) {}

  /**
   * Sets the voltage of the Right Climber Motor
   *
   * @param volts -12 to 12
   */
  public default void setRightClimberVoltage(double volts) {}

  /**
   * Sets BOTH Climber Motors to a percent of their maximum speed
   *
   * @param percent -1 to 1
   */
  public default void setClimberPercentSpeed(double percent) {}

  /**
   * Sets the Left Climber Motor to a percent of their maximum speed
   *
   * @param percent -1 to 1
   */
  public default void setLeftClimberPercentSpeed(double percent) {}

  /**
   * Sets Right Climber Motor to a percent of their maximum speed
   *
   * @param percent -1 to 1
   */
  public default void setRightClimberPercentSpeed(double percent) {}

  /**
   * Sets the Brake Mode for the Actuator (Brake means motor holds position, Coast means easy to
   * move)
   *
   * @param enable if enable, it sets brake mode, else it sets coast mode
   */
  public default void setClimberBrakeMode(boolean enable) {}

  public default void setClimberCurrent(int curr) {}
}
