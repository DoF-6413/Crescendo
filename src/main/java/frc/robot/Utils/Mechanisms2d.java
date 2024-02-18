package frc.robot.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d swerveMech;

  private final MechanismRoot2d Wristroot;
  private final MechanismRoot2d armRoot;
  private final MechanismRoot2d actuatorRoot;
  private final MechanismLigament2d wristMech;
  private final MechanismLigament2d armMech;
  private final MechanismLigament2d actuatorMech;
  private final Wrist m_wristSub;
  private final Arm m_Arm;
  private final Actuator m_Actuator;

  private final Color8Bit red = new Color8Bit(255, 0, 0);
  private final Color8Bit green = new Color8Bit(0, 255, 0);

  private final Color8Bit blue = new Color8Bit(0, 0, 255);

  public Mechanisms2d(Wrist wristSub, Arm arm, Actuator actuator) {
    swerveMech = new Mechanism2d(5, 5); // sets the screen

    m_wristSub = wristSub;
    m_Arm = arm;
    m_Actuator = actuator;
    // set the cordenates on the screen where it is
    Wristroot = swerveMech.getRoot("wristRoot", 2, 2);
    armRoot = swerveMech.getRoot("armRoot", 0, 0);
    actuatorRoot = swerveMech.getRoot("actuatorRoot", 1, 2);
    // set the dimentions of the mecanism
    wristMech = Wristroot.append(new MechanismLigament2d("wristRoot", 2, 0, 1, red));
    armMech = armRoot.append(new MechanismLigament2d("armRoot", 1, 120, 5, green));
    actuatorMech = actuatorRoot.append(new MechanismLigament2d("actuatorRoot", 2, 10, 1, blue));
  }

  @Override
  public void periodic() {

    wristMech.setAngle(Units.radiansToDegrees(m_wristSub.getAngleRads()));
    armMech.setAngle(Units.radiansToDegrees(m_Arm.getAngleRads()));
    actuatorMech.setAngle(Units.radiansToDegrees(m_Actuator.getAngleRads()));
    Logger.recordOutput("mechanism", swerveMech);
  }
}
