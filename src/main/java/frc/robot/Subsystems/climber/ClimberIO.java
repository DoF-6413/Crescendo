package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

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
}
