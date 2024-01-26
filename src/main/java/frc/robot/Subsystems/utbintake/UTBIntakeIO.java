package frc.robot.Subsystems.utbintake;

import org.littletonrobotics.junction.AutoLog;

public class UTBIntakeIO {

  @AutoLog
  public static class UTBIntakeIOInputs {
    public double UTBIntakeVelocityRadPerSec = 0.0;
    public double UTBIntakeAppliedVolts = 0.0;
  }

  public void updateInputs(UTBIntakeIOInputs inputs) {}
}
