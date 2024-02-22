package frc.robot.Subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

  @AutoLog
  public static class  ShooterIOInputs {
    /** Velocity of the Feeder Motor in Rotations per Minute */
    public double feederRPM = 0.0;
    /** Number of volts being sent to the Feeder Motor */
    public double feederAppliedVolts = 0.0;
    /** Number of amps used by the Feeder motor */
    public double[] feederCurrentAmps = new double[] {};
    /** Tempature of the Feeder motor in Celsius */
    public double[] feederTempCelsuis = new double[] {};    
    }
}  
    

