// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.climber.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.utbintake.*;
import frc.robot.Subsystems.vision.*;
import frc.robot.Subsystems.wrist.*;
import frc.robot.Utils.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Arm m_armSubsystem;
  private final Drive m_driveSubsystem;
  private final Gyro m_gyroSubsystem;
  private final Shooter m_shooterSubsystem;
  private final Vision m_visionSubsystem;
  private final UTBIntake m_utbIntakeSubsystem;
  private final OTBIntake m_otbIntakeSubsystem;
  private final Actuator m_actuatorSubsystem;
  private final Climber m_climberSubsystem;
  private final PoseEstimator m_poseEstimator;
  private final PathPlanner m_pathPlanner;
  private final Wrist m_wristSubsystem;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_gyroSubsystem = new Gyro(new GyroIONavX());
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMax(),
                new ModuleIOSparkMax(),
                new ModuleIOSparkMax(),
                new ModuleIOSparkMax(),
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIOSparkMax());
        m_visionSubsystem = new Vision(new VisionIOArduCam());
        m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
        m_climberSubsystem = new Climber(new ClimberIOSparkMax() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSparkMax());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSparkMax());
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
        m_wristSubsystem = new Wrist(new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIOSimNeo(),
                new ModuleIOSimNeo(),
                new ModuleIOSimNeo(),
                new ModuleIOSimNeo(),
                m_gyroSubsystem);
        m_visionSubsystem = new Vision(new VisionIOSim());
        m_armSubsystem = new Arm(new ArmIOSim());
        m_shooterSubsystem = new Shooter(new ShooterIOSim());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSim() {});
        m_climberSubsystem = new Climber(new ClimberIOSim() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSim());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSim());
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
        m_wristSubsystem = new Wrist(new WristIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                m_gyroSubsystem);
        m_visionSubsystem = new Vision(new VisionIO() {});
        m_armSubsystem = new Arm(new ArmIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_climberSubsystem = new Climber(new ClimberIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Default Path", new PathPlannerAuto("ROCK"));
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // A default command always runs unless another command is called
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () ->
                m_driveSubsystem.setRaw(
                    driverController.getLeftX(),
                    -driverController.getLeftY(),
                    driverController.getRightX()),
            m_driveSubsystem));

    driverController.a().onTrue(new InstantCommand(() -> m_driveSubsystem.updateHeading()));

    // TODO: update controls
    // m_utbIntakeSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_utbIntakeSubsystem.setutbIntakePercentSpeed(auxController.getLeftY()),
    //         m_utbIntakeSubsystem));

    // m_climberSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_climberSubsystem.setBothClimberPercentSpeed(auxController.getRightY()),
    //         m_climberSubsystem));

    // m_armSubsystem.setDefaultCommand(
    //       new InstantCommand(
    //           () > m_armSubsystem.setArmMotorSpeed(auxController.getLeftY()), m_armSubsystem));

    // m_otbIntakeSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(auxController.getRightY()),
    //         m_otbIntakeSubsystem));

    // m_actuatorSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_actuatorSubsystem.setActuatorPercentSpeed(auxController.getLeftY()),
    //         m_actuatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return null;
  }
}
