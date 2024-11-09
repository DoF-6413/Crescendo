package frc.robot.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BeamBreakConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public final class BeamBreak implements Subsystem {
  private final DigitalInput shooterBeamBreak;
  // private final DigitalInput intakeBeamBreak;

  public BeamBreak() {
    System.out.println("[Init] Creating Beam Breaks");
    shooterBeamBreak = new DigitalInput(BeamBreakConstants.SHOOTER_BEAM_BREAK_PORT);
    // intakeBeamBreak = new DigitalInput(BeamBreakConstants.INTAKE_BEAM_BREAK_PORT);
  }

  public void periodic() {
    getShooterSensor();
    // getIntakeSensor();
  }

  /**
   * Detects if a NOTE is in the Shooter with the Beam Break
   *
   * @return True if NOTE hits the sensor, False if the sensor is unbroken
   */
  @AutoLogOutput(key = "BeamBreaks/ShooterSensor")
  public boolean getShooterSensor() {
    return !shooterBeamBreak.get();
  }

  /**
   * Detects if a NOTE is in the Intake with the Beam Break
   *
   * @return True if NOTE hits the sensor, False if the sensor is unbroken
   */
  // @AutoLogOutput(key = "BeamBreaks/IntakeSensor")
  // public boolean getIntakeSensor() {
  //   return !intakeBeamBreak.get();
  // }
}
