package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    // All the Inputs for the Right Climber Motor (Should be nearly identical to the Right Climber
    // Motor)
    public double leftClimberPositionRad = 0.0;
    public double leftClimberVelocityRadPerSec = 0.0;
    public double leftClimberAppliedVolts = 0.0;
    public double[] leftClimberCurrentAmps = new double[] {};
    public double[] leftClimberTempCelcius = new double[] {};
   
    // All the Inputs for the Left Climber Motor (Should be nearly identical to the Left Climber
    // Motor)
    public double rightClimberPositionRad = 0.0;
    public double rightClimberVelocityRadPerSec = 0.0;
    public double rightClimberAppliedVolts = 0.0;
    public double[] rightClimberCurrentAmps = new double[] {};
    public double[] rightClimberTempCelcius = new double[] {};
  }

  /**
   * Sets the voltage of BOTH Climber Motors
   * 
   * @param volts [-12 to 12]
   */
  public default void setBothClimberMotorsVoltage(double volts) {}

  /**
   * Sets the voltage of the Left Climber Motor
   * 
   * @param volts [-12 to 12]
   */
  public default void setLeftClimberMotorVoltage(double volts) {}

  /**
   * Sets the voltage of the Right Climber Motor
   * 
   * @param volts [-12 to 12]
   */
  public default void setRightClimberMotorVoltage(double volts) {}

  /**
   * Sets BOTH Climber Motors to a percent of their maximum speed
   * 
   * @param percent [-1 to 1]
   */
  public default void setBothClimberMotorsPercentSpeed(double percent) {}

  /**
   * Sets the Left Climber Motor to a percent of their maximum speed
   * 
   * @param percent [-1 to 1]
   */
  public default void setLeftClimberMotorPercentSpeed(double percent) {}
  
  /**
   * Sets Right Climber Motor to a percent of their maximum speed
   * 
   * @param percent [-1 to 1]
   */
  public default void setRightClimberMotorPercentSpeed(double percent) {}
}
