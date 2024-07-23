package frc.robot.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class BeamBreak implements Subsystem {
  private final DigitalInput shooterBeamBreak;

  public BeamBreak() {
    System.out.println("[Init] Creating Beam Break");
    shooterBeamBreak = new DigitalInput(0);
  }

  public void periodic() {
    getShooterSensor();
  }

  @AutoLogOutput(key = "BeamBreaks/ShooterSensor")
  public boolean getShooterSensor() {
    return shooterBeamBreak.get();
  }
}
