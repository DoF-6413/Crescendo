package frc.robot.Subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    /** Number of volts sent to the Wrist motor */
    public double wristAppliedVolts = 0.0;
    /** The current angle of the Wrist in Radians */
    public double wristPositionRad = 0.0;
    /** The current angle of the Wrist in Degrees */
    public double wristPositionDeg = 0.0;
    /** The velocity of the Wrist in Radians per Second */
    public double wristVelocityRadPerSec = 0.0;
    /** Number of amps used by the Wrist motor */
    public double[] wristCurrentAmps = new double[] {};
    /** Tempature of the Wrist motor */
    public double[] wristTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for the Wrist */
  public default void updateInputs(WristIOInputs inputs) {}

  /**
   * Sets Wrist Percent Speed
   *
   * @param percent [-1 to 1]
   */
  public default void setWristPercentSpeed(double percent) {}

  /**
   * Sets Wrist Voltage
   *
   * @param volts [-12 to 12]
   */
  public default void setWristVoltage(double volts) {}

  /**
   * Sets brake mode
   *
   * @param enable boolean for is brake mode true or false
   */
  public default void setWristBrakeMode(boolean enable) {}

  /**
   * Sets the wrist to a specified angle
   *
   * @param double angle in radians
   */
  public default void setWristPositionRads(double angle) {}
}
