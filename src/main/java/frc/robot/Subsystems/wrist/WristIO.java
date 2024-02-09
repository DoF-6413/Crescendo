package frc.robot.Subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {

    // the first root of the wrist

    public double firstWristTurnAppliedVolts = 0.0;
    public double firstWristTurnPositionRad = 0.0;
    public double firstWristTurnVelocityRadPerSec = 0.0;
    public double firstWristTurnCurrentAmps = 0.0;
    public double firstWristTempCelcius = 0.0;

    // the second root of the wrist (the closest to the shooter)
    public double secondWristTurnAppliedVolts = 0.0;
    public double secondWristTurnPositionRad = 0.0;
    public double secondWristTurnVelocityRadPerSec = 0.0;
    public double secondWristTurnCurrentAmps = 0.0;
    public double secondWristTempCelcius = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristMotorsSpeed(double firstSpeed, double secondSpeed) {
    /** Sets speed for the first motor(the farest of the shooter) of the wrist */
  }
}
