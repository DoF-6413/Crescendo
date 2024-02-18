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

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristPercentSpeed(double percent) {}

  public default void setWristVoltage(double volts) {}
}
