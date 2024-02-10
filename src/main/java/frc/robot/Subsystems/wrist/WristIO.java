package frc.robot.Subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {

    public double wristTurnAppliedVolts = 0.0;
    public double wristTurnPositionRad = 0.0;
    public double wristTurnVelocityRadPerSec = 0.0;
    public double wristTurnCurrentAmps = 0.0;
    public double wristTempCelcius = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristMotorSpeed(double peed) {
}
}
