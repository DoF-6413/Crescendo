package frc.robot.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d swerveMech;

  private final MechanismRoot2d Wristroot;
  private final MechanismLigament2d wristMech;
  private final Wrist m_wristSub;

  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armMech;
  private final Arm m_Arm;

  private final Color8Bit red = new Color8Bit(255, 0, 0);
  private final Color8Bit green = new Color8Bit(0, 255, 0);

  public Mechanisms2d(Wrist wristSub, Arm arm) {
    swerveMech = new Mechanism2d(5, 5); // sets the screen

    m_wristSub = wristSub;

    // set the cordenates on the screen where it is
    Wristroot = swerveMech.getRoot("wristRoot", 2, 2);

    // set the dimentions of the mecanism
    wristMech = Wristroot.append(new MechanismLigament2d("wristRoot", 2, 0, 1, red));

    m_Arm = arm;

    armRoot = swerveMech.getRoot("armRoot", 0, 0);
    armMech = armRoot.append(new MechanismLigament2d("armRoot", 1, 120, 5, green));
  }

  @Override
  public void periodic() {

    wristMech.setAngle(Units.radiansToDegrees(m_wristSub.getAngleRads()));
    armMech.setAngle(Units.radiansToDegrees(m_Arm.getAngleRads()));
    Logger.recordOutput("mechanism", swerveMech);
  }
}
