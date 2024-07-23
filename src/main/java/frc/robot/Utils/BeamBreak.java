package frc.robot.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BeamBreakConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class BeamBreak implements Subsystem {
  private final DigitalInput shooterBeamBreak;

  public BeamBreak() {
    System.out.println("[Init] Creating Beam Break");
    shooterBeamBreak = new DigitalInput(BeamBreakConstants.SHOOTER_BEAM_BREAK_PORT);
  }

  public void periodic() {
    getShooterSensor();
  }

  @AutoLogOutput(key = "BeamBreaks/ShooterSensor")
  /** Returns true if nothing is blocking the Shooter Beam Break, false if there is */
  public boolean getShooterSensor() {
    return shooterBeamBreak.get();
  }
}
