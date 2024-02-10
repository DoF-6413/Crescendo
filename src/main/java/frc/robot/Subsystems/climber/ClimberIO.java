package frc.robot.Subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {

    // All the Inputs for the Right Climber Motor (Should be nearly identical to the Right Climber
    // Motor)
    public double leftclimberPositionRad = 0.0;
    public double leftclimberVelocityRadPerSec = 0.0;
    public double leftclimberAppliedVolts = 0.0;
    public double[] leftclimberCurrentAmps = new double[] {};
    public double[] leftclimberTempCelcius = new double[] {};
   
    // All the Inputs for the Left Climber Motor (Should be nearly identical to the Left Climber
    // Motor)
    public double rightclimberPositionRad = 0.0;
    public double rightclimberVelocityRadPerSec = 0.0;
    public double rightclimberAppliedVolts = 0.0;
    public double[] rightclimberCurrentAmps = new double[] {};
    public double[] rightclimberTempCelcius = new double[] {};
  }
}
