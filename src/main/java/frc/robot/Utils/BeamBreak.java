package frc.robot.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BeamBreakConstants;

public class BeamBreak implements Subsystem {
  private DigitalInput shooterBeamBreak;

  public BeamBreak() {
    System.out.println("[Init] Creating Beam Break");
    shooterBeamBreak = new DigitalInput(BeamBreakConstants.SHOOTER_BEAM_BREAK_PORT);
  }

  public void periodic() {
    getShooterSensor();
    SmartDashboard.putBoolean("Shooter BeamBreak Sensor", getShooterSensor());
  }

  public boolean getShooterSensor() {
    return shooterBeamBreak.get();
  }
}
