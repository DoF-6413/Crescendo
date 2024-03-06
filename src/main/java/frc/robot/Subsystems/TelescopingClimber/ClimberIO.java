package frc.robot.Subsystems.TelescopingClimber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
    /** This returns the voltage the Climber Recieves */
    public double climberAppliedVolts = 0.0;
    /** Returns the position of the Climber in Meters */
    public double climberPositionMeters = 0.0;
    /** Returns the velocity of the Climber in m/s */
    public double climberVelocityMetersPerSecond = 0.0;
    /** The Current Drawn from the Climber in Amps */
    public double[] climberCurrentAmps = new double[] {};
    /** The Temperature from the Climber in Celsius */
    public double[] climberTempCelsius = new double[] {};
    }
}
