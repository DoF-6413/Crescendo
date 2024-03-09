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

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Commands.TeleopCommands.AmpScore.Backside.*;
import frc.robot.Commands.TeleopCommands.AmpScore.Frontside.*;
import frc.robot.Commands.TeleopCommands.Intakes.*;
import frc.robot.Commands.TeleopCommands.SourcePickup.SourcePickUpBackside;
import frc.robot.Commands.TeleopCommands.SpeakerScore.ShootAtSpeaker;
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.climber.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.photonVision.*;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.utbintake.*;
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
  // Drivetrain
  private final Gyro m_gyroSubsystem;
  private final Drive m_driveSubsystem;

  // Mechanisms
  private final Arm m_armSubsystem;
  private final Vision m_visionSubsystem;
  // private final Climber m_climberSubsystem;
  private final UTBIntake m_utbIntakeSubsystem;
  private final OTBIntake m_otbIntakeSubsystem;
  private final Actuator m_actuatorSubsystem;
  private final Shooter m_shooterSubsystem;
  private final Feeder m_feederSubsystem;
  private final Wrist m_wristSubsystem;

  // Utilities
  private final PoseEstimator m_poseEstimator;
  private final PathPlanner m_pathPlanner;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

  // Autos
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
      case REAL:
        // Real robot, instantiates hardware IO implementations
        m_gyroSubsystem = new Gyro(new GyroIONavX());
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIOSparkMax());
        m_visionSubsystem = new Vision(new VisionIOArduCam());
        // m_climberSubsystem = new Climber(new ClimberIOSparkMax());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSparkMax());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSparkMax());
        m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        m_feederSubsystem = new Feeder(new FeederIOTalonFX());
        m_wristSubsystem = new Wrist(new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiates physics sim IO implementations
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIOSimNeoKraken(),
                new ModuleIOSimNeoKraken(),
                new ModuleIOSimNeoKraken(),
                new ModuleIOSimNeoKraken(),
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIOSim());
        m_visionSubsystem = new Vision(new VisionIOSim());
        // m_climberSubsystem = new Climber(new ClimberIOSim());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSim());
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSim());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSim());
        m_shooterSubsystem = new Shooter(new ShooterIOSim());
        m_feederSubsystem = new Feeder(new FeederIOSim());
        m_wristSubsystem = new Wrist(new WristIOSim());
        break;

      default:
        // Replayed robot, disables IO implementations
        m_gyroSubsystem = new Gyro(new GyroIO() {});
        m_driveSubsystem =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIO() {});
        m_visionSubsystem = new Vision(new VisionIO() {});
        // m_climberSubsystem = new Climber(new ClimberIO() {});
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }
    m_gyroSubsystem = new Gyro(new GyroIO() {});
    m_driveSubsystem =
        new Drive(
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            m_gyroSubsystem);

    // Configure the button bindings
    configureButtonBindings();

    m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);

    // Adds list of autos to Shuffleboard
    autoChooser.addOption("Do Nothing", new InstantCommand());
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
    // The front of the robot is the side where the intakes are located
    // A default command always runs unless another command is called

    /** Driver Contols */

    // Driving the robot
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () ->
                m_driveSubsystem.driveWithDeadband(
                    driverController.getLeftX(), // Forward/backward
                    -driverController
                        .getLeftY(), // Left/Right (multiply by -1 bc controller axis is inverted)
                    driverController.getRightX()), // Rotate chassis left/right
            m_driveSubsystem));

    // Resets robot heading to be wherever the front of the robot is facing
    driverController
        .a()
        .onTrue(new InstantCommand(() -> m_driveSubsystem.updateHeading(), m_driveSubsystem));

    /* UTB Intake */
    // Intake NOTE
    driverController
        .rightTrigger()
        .whileTrue(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(-1), m_utbIntakeSubsystem))
        .whileFalse(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));
    // Outtake NOTE
    driverController
        .rightBumper()
        .whileTrue(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(1), m_utbIntakeSubsystem))
        .whileFalse(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));

    /* All Intakes */
    // Intake NOTE
    driverController
        .leftTrigger()
        .whileTrue(
            new AllIntakesRun(
                m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem, true))
        .whileFalse(
            new AllIntakesStop(m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem));
    // Outtake NOTE
    driverController
        .leftBumper()
        .whileTrue(
            new AllIntakesRun(
                m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem, false))
        .whileFalse(
            new AllIntakesStop(m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem));

    // Brings Actuator back to its default position (all the way up)
    driverController.start().onTrue(new ActuatorToZero(m_actuatorSubsystem));

    /** Aux Controls */

    /* Feeder */
    // Forward
    auxController
        .x()
        .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(2500), m_feederSubsystem))
        .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));
    // Backward
    auxController
        .b()
        .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(-250), m_feederSubsystem))
        .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));

    /* Wrist */
    // Increases angle of the Wrist by 1 degree
    auxController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> m_wristSubsystem.incrementWristSetpoint(Units.degreesToRadians(1)),
                m_wristSubsystem));
    // Decreases angle of the Wrist by 1 degree
    auxController
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> m_wristSubsystem.incrementWristSetpoint(Units.degreesToRadians(-1)),
                m_wristSubsystem));

    /* Arm */
    // Increases angle of the Arm by 1 degree
    auxController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmSetpoint(Units.degreesToRadians(1)),
                m_armSubsystem));
    // Decreases angle of the Arm by 1 degree
    auxController
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmSetpoint(Units.degreesToRadians(-1)),
                m_armSubsystem));

    /* Scoring SPEAKER when up against it */
    auxController
        .leftTrigger()
        .onTrue(new ShootAtSpeaker(m_feederSubsystem, m_shooterSubsystem, m_wristSubsystem))
        .onFalse(
            new ParallelCommandGroup(
                new ArmToZero(m_wristSubsystem, m_armSubsystem),
                new EndEffectorToZero(m_shooterSubsystem, m_feederSubsystem)));

    /* AMP Scoring */
    // Scoring AMP from the frontside
    auxController
        .rightTrigger()
        .onTrue(new PositionAmpScoreFrontSide(m_armSubsystem, m_wristSubsystem))
        .onFalse(
            new ScoreAmpFrontSide(
                m_armSubsystem, m_wristSubsystem, m_feederSubsystem, m_shooterSubsystem));
    // Scoring from the backside
    auxController
        .rightBumper()
        .onTrue(new PositionAmpScoreBackside(m_armSubsystem, m_wristSubsystem))
        .onFalse(new ScoreAmpBackSide(m_armSubsystem, m_wristSubsystem, m_feederSubsystem));

    /* SOURCE Pickup */
    // Picking up from SOURCE, backside
    auxController
        .leftBumper()
        .onTrue(new SourcePickUpBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem))
        .onFalse(
            new ParallelCommandGroup(
                new ArmToZero(m_wristSubsystem, m_armSubsystem),
                new EndEffectorToZero(m_shooterSubsystem, m_feederSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Either Coast or Brake mechanisms depending on Disable or Enable */
  public void mechanismsCoastOnDisable(boolean isDisabled) {
    m_driveSubsystem.coastOnDisable(isDisabled);
    m_armSubsystem.setBrakeMode(!isDisabled);
    m_wristSubsystem.setBrakeMode(!isDisabled);
    m_actuatorSubsystem.setBrakeMode(!isDisabled);
    m_shooterSubsystem.setBrakeMode(!isDisabled);
  }
}
