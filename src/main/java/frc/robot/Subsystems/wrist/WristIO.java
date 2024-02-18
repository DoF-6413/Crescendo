package frc.robot.Subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {

    public double wristAppliedVolts = 0.0;
    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double[] wristCurrentAmps = new double[] {};
    public double[] wristTempCelsius = new double[] {};
  }

  /** Updates the set of loggable inputs for the Wrist */
  public default void updateInputs(WristIOInputs inputs) {}

  /** 
   * Sets Wrist Percent Speed
   * @param percent [-1 to 1]
   */
  public default void setWristPercentSpeed(double percent) {}

  /**
   * Sets Wrist Voltage 
   * @param volts [-12 to 12]
   */
  public default void setWristVoltage(double volts) {}
}
