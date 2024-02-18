package frc.robot.Utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.wrist.Wrist;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d swerveMech;
  private final MechanismRoot2d Wristroot;
  private final MechanismLigament2d wristMech;
  private final Wrist m_wristSub;
  private final Color8Bit red = new Color8Bit(255, 0, 0);

  public Mechanisms2d(Wrist wristSub) {
    swerveMech = new Mechanism2d(5, 5);

    m_wristSub = wristSub;
    Wristroot = swerveMech.getRoot("MechanismRoot", 2, 2);

    wristMech = Wristroot.append(new MechanismLigament2d("wristRoot", 2, 90, 1, red));
  }

  @Override
  public void periodic() {

    wristMech.setAngle(Units.radiansToDegrees(m_wristSub.getAngleRads()));
    Logger.recordOutput("mechanism", swerveMech);
  }
}
