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
    swerveMech =
        new Mechanism2d(
            1.5, 1.5); // dimentions of the robot inside bumpers without bumpers x+.28 ,y-.32//1,1

    m_wristSub = wristSub;
    m_Arm = arm;
    m_Actuator = actuator;

    // set the cordenates on the screen where it is
    armRoot = swerveMech.getRoot("armRoot", 0.57, .58985); // done
    actuatorRoot = swerveMech.getRoot("actuatorRoot", .35, .2715); // done
    Wristroot = swerveMech.getRoot("wristRoot", 1, 22222);

    // set the dimentions of the mecanism in m
    actuatorMech =
        actuatorRoot.append(
            new MechanismLigament2d( // done
                "actuatorRoot", .2851, 0, 1, blue));

    wristMech = Wristroot.append(new MechanismLigament2d("wristRoot", .2, 0, 1, red));

    armMech =
        armRoot.append(
            new MechanismLigament2d(
                "armRoot", // done
                .39869769, 0, 1, green));
    armMech.append(wristMech);
  }

  @Override
  public void periodic() {

    wristMech.setAngle(Units.radiansToDegrees(m_wristSub.getAngleRads()));
    armMech.setAngle(Units.radiansToDegrees(m_Arm.getAngleRads()));
    actuatorMech.setAngle(Units.radiansToDegrees(m_Actuator.getAngleRads()));

    Logger.recordOutput("mechanism", swerveMech);
  }
}
