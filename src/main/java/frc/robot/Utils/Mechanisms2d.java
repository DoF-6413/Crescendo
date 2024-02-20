package frc.robot.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.climber.Climber;
import frc.robot.Subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class Mechanisms2d extends SubsystemBase {
  private final Mechanism2d swerveMech;

  private final MechanismRoot2d Wristroot;
  private final MechanismRoot2d armRoot;
  private final MechanismRoot2d actuatorRoot;
  private final MechanismRoot2d climberRoot;

  private final MechanismLigament2d wristMech;
  private final MechanismLigament2d armMech;
  private final MechanismLigament2d actuatorMech;
  private final MechanismLigament2d leftClimberMech;
  private final MechanismLigament2d rightClimberMech;

  private final Wrist m_wristSub;
  private final Arm m_Arm;
  private final Actuator m_Actuator;
  private final Climber m_Climber;

  private final Color8Bit red = new Color8Bit(255, 0, 0);
  private final Color8Bit green = new Color8Bit(0, 255, 0);
  private final Color8Bit blue = new Color8Bit(0, 0, 255);
  private final Color8Bit purple = new Color8Bit(255, 0, 255);
  private final Color8Bit loquesea = new Color8Bit(255, 255, 255);

  public Mechanisms2d(Wrist wristSub, Arm arm, Actuator actuator, Climber climber) {
    swerveMech = new Mechanism2d(5, 5); // sets the screen

    m_wristSub = wristSub;
    m_Arm = arm;
    m_Actuator = actuator;
    m_Climber = climber;

    // set the cordenates on the screen where it is
    Wristroot = swerveMech.getRoot("wristRoot", 2, 2);
    armRoot = swerveMech.getRoot("armRoot", 0, 0);
    actuatorRoot = swerveMech.getRoot("actuatorRoot", 1, 2);
    climberRoot = swerveMech.getRoot("climberRoot", 2.5, 2.5);
    // set the dimentions of the mecanism
    wristMech = Wristroot.append(new MechanismLigament2d("wristRoot", 2, 0, 1, red));
    armMech = armRoot.append(new MechanismLigament2d("armRoot", 1, 120, 5, green));
    actuatorMech = actuatorRoot.append(new MechanismLigament2d("actuatorRoot", 2, 10, 1, blue));
    leftClimberMech =
        climberRoot.append(new MechanismLigament2d("leftClimberRoot", 2, 90, 2, purple));
    rightClimberMech =
        climberRoot.append(new MechanismLigament2d("rightClimberRoot", 2, 90, 3, loquesea));
  }

  @Override
  public void periodic() {

    wristMech.setAngle(Units.radiansToDegrees(m_wristSub.getAngleRads()));
    armMech.setAngle(Units.radiansToDegrees(m_Arm.getAngleRads()));
    actuatorMech.setAngle(Units.radiansToDegrees(m_Actuator.getAngleRads()));
    leftClimberMech.setLength(m_Climber.getLeftClimberPose());
    rightClimberMech.setLength(m_Climber.getRightClimberPose());

    Logger.recordOutput("mechanism", swerveMech);
  }
}
