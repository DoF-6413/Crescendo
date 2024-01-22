package frc.robot.Subsystems.mechanism2d;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.shooter.Shooter;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d m_shooterFlywheel;
  private final MechanismRoot2d root;
  private final Shooter m_shooterSub;

  public Mechanisms2d(Shooter shooterSub) {
    m_shooterSub = shooterSub;
    m_shooterFlywheel = new Mechanism2d(3, 3); // only displays rectangular objects?
    root =
        m_shooterFlywheel.getRoot(
            "flywheelRoot", 0, 0); // "root" can only be defined after the mechanism is
  }

  @Override
  public void periodic() {
    // updates values which in this case is just rollers I guess, is sim even necessary for the
    // shooter (not including wrist)?
  }
}
