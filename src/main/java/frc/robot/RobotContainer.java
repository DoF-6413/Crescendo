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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.FourPieceBlue;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.FourPieceRed;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.LeaveAuto;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceAuto;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceLeaveAmpSide;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceLeaveCenter;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceLeaveCoolSide;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.ThreePieceAutoBlue;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.ThreePieceAutoRed;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.TwoPieceReturnSub;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.AutoShoot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PickUp;
import frc.robot.Commands.TeleopCommands.AmpScore.Backside.*;
import frc.robot.Commands.TeleopCommands.AmpScore.Frontside.*;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Commands.TeleopCommands.Intakes.*;
import frc.robot.Commands.TeleopCommands.SourcePickup.SourcePickUpBackside;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.Shoot;
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.climber.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.feeder.*;
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
  // Drivetrain
  private final Gyro m_gyroSubsystem;
  private final Drive m_driveSubsystem;

  // Mechanisms
  private final Arm m_armSubsystem;
  //   private final Vision m_visionSubsystem;
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
  private final CommandXboxController devController = new CommandXboxController(2);

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
        // m_visionSubsystem = new Vision(new VisionIOArduCam());
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
        // m_visionSubsystem = new Vision(new VisionIOSim());
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
        // m_visionSubsystem = new Vision(new VisionIO() {});
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    // Utils
    m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);

    /* Event Marker Commands */
    // Shooter/Feeder
    NamedCommands.registerCommand(
        "Shooter4000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.CLOSE_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Shooter6000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.FAR_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "StopShooter",
        new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Feeder",
        new InstantCommand(
            () -> m_feederSubsystem.setSetpoint(FeederConstants.SPEAKER_RPM), m_feederSubsystem));
    NamedCommands.registerCommand(
        "StopFeeder",
        new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));
    // Angles
    NamedCommands.registerCommand(
        "SpeakerAngle",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(WristConstants.SUBWOOFER_RAD), m_wristSubsystem));
    NamedCommands.registerCommand(
        "ZeroWrist",
        new InstantCommand(
            () ->
                m_wristSubsystem.setGoal(
                    Units.degreesToRadians(WristConstants.DEFAULT_POSITION_DEG)),
            m_wristSubsystem));
    // Pick Ups
    NamedCommands.registerCommand(
        "UTB",
        new InstantCommand(
            () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(-1), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "UTBStop",
        new InstantCommand(
            () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "PickUp",
        new PickUp(m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem, false));
    NamedCommands.registerCommand(
        "PickUpStop",
        new PickUp(m_actuatorSubsystem, m_otbIntakeSubsystem, m_utbIntakeSubsystem, true));
    // Shooting
    NamedCommands.registerCommand(
        "SpeakerShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            WristConstants.SUBWOOFER_RAD,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "PodiumShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            0.5,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "LineShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            3,
            ShooterConstants.FAR_RPM)); // TODO: Update angle
    NamedCommands.registerCommand(
        "LegShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            -7,
            ShooterConstants.FAR_RPM)); // TODO: Update angle
    NamedCommands.registerCommand(
        "WingShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            -8.5,
            ShooterConstants.FAR_RPM));
    NamedCommands.registerCommand(
        "ZeroAll",
        new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* PathPlanner Autos */
    autoChooser.addOption("Do Nothing", new InstantCommand());
    // Test Autos
    autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    autoChooser.addOption("test1", new PathPlannerAuto("test1"));
    autoChooser.addOption("test2", new PathPlannerAuto("test2"));
    autoChooser.addOption("test3", new PathPlannerAuto("test3"));
    // autoChooser.addOption("Midfield Test", new PathPlannerAuto("Midfield Test"));
    // 2 Piece
    autoChooser.addOption("2 Piece Center", new PathPlannerAuto("2P Center"));
    autoChooser.addOption("2 Piece Center 2.0", new PathPlannerAuto("Center"));
    autoChooser.addOption("2 Piece Amp", new PathPlannerAuto("2P Amp"));
    autoChooser.addOption("2 Piece Cool Side", new PathPlannerAuto("2P Cool Side"));
    // 3 Piece
    autoChooser.addOption("3 Piece Center", new PathPlannerAuto("3P Center"));
    autoChooser.addOption("3 Piece Cool Side", new PathPlannerAuto("3P Cool Side"));
    // 4 Piece
    autoChooser.addOption("4 Piece Center", new PathPlannerAuto("4P Center"));
    // 5+ Piece
    autoChooser.addOption("5.5PieceAuto", new PathPlannerAuto("5.5PieceAuto"));
    autoChooser.addOption("Liz3Piece", new PathPlannerAuto("Liz2Piece"));
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());

    // Adds list of deadreckond autos to Shuffleboard
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Leave", new LeaveAuto(m_driveSubsystem, 3, 1));
    autoChooser.addOption(
        "One Piece", new OnePieceAuto(m_wristSubsystem, m_feederSubsystem, m_shooterSubsystem));
    autoChooser.addOption(
        "One Piece Leave Center",
        new OnePieceLeaveCenter(
            m_driveSubsystem,
            m_wristSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            3,
            1,
            m_gyroSubsystem));
    autoChooser.addOption(
        "Better Two Piece",
        new TwoPieceReturnSub(
            m_driveSubsystem,
            m_gyroSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            2,
            1));
    autoChooser.addOption(
        "Blue One Piece Leave Amp Side",
        new OnePieceLeaveAmpSide(
            m_wristSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_driveSubsystem,
            2,
            1,
            m_gyroSubsystem));
    autoChooser.addOption(
        "Blue One Piece Leave Cool Side",
        new OnePieceLeaveCoolSide(
            m_wristSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_driveSubsystem,
            2,
            1,
            m_gyroSubsystem));
    autoChooser.addOption(
        "Red One Piece Leave Cool Side",
        new OnePieceLeaveAmpSide(
            m_wristSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_driveSubsystem,
            2,
            1,
            m_gyroSubsystem));
    autoChooser.addOption(
        " Red One Piece Leave Amp Side",
        new OnePieceLeaveCoolSide(
            m_wristSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_driveSubsystem,
            2,
            1,
            m_gyroSubsystem));
    autoChooser.addOption(
        " Three Piece Leave Amp Side Blue",
        new ThreePieceAutoBlue(
            m_driveSubsystem,
            m_gyroSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            2,
            1));
    autoChooser.addOption(
        " Three Piece Leave Amp Side Red",
        new ThreePieceAutoRed(
            m_driveSubsystem,
            m_gyroSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            2,
            1));
    autoChooser.addOption(
        "4 Piece Blue",
        new FourPieceBlue(
            m_driveSubsystem,
            m_gyroSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            2,
            1));
    autoChooser.addOption(
        "4 Piece Red",
        new FourPieceRed(
            m_driveSubsystem,
            m_gyroSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_feederSubsystem,
            m_shooterSubsystem,
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            2,
            1));

    SmartDashboard.putNumber("Delay", 0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // The front of the robot is the side where the intakes are located
    // A default command always runs unless another command is called

    /** Driver Controls */
    this.driverControllerBindings();

    /** Aux Controls */
    this.auxControllerBindings();

    // this.devContr%ollerBindings();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Curve");
    // return AutoBuilder.followPath(path);
    // return new PathPlannerAuto("Curve");
    return autoChooser.get();
  }

  /** Either Coast or Brake mechanisms depending on Disable or Enable */
  public void mechanismsCoastOnDisable(boolean isDisabled) {
    m_driveSubsystem.coastOnDisable(isDisabled);
    m_armSubsystem.setBrakeMode(!isDisabled);
    m_wristSubsystem.setBrakeMode(!isDisabled);
    m_actuatorSubsystem.setBrakeMode(!isDisabled);
    m_shooterSubsystem.setBrakeMode(!isDisabled);
    m_utbIntakeSubsystem.setUTBIntakeBrakeMode(!isDisabled);
    m_otbIntakeSubsystem.setBrakeMode(!isDisabled);
  }

  public void enablePID(boolean enable) {
    m_armSubsystem.enablePID(enable);
    m_wristSubsystem.enablePID(enable);
    m_shooterSubsystem.enablePID(enable);
  }

  public void enableTesting(boolean enable) {
    m_armSubsystem.enableTesting(enable);
    m_wristSubsystem.enableTesting(enable);
    m_shooterSubsystem.enableTesting(enable);
  }

  public void setAllSetpointsZero() {
    m_shooterSubsystem.setSetpoint(0);
    m_wristSubsystem.setGoal(0);
    m_armSubsystem.setGoal(0);
    m_feederSubsystem.setSetpoint(0);
  }

  public void driverControllerBindings() {
    // Driving the robot
    m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(m_driveSubsystem, driverController));

    // Resets robot heading to be wherever the front of the robot is facing
    driverController
        .a()
        .onTrue(new InstantCommand(() -> m_driveSubsystem.updateHeading(), m_driveSubsystem));

    /* UTB Intake */
    // Intake NOTE
    driverController
        .rightTrigger()
        .onTrue(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, true, false))
        .onFalse(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, true));
    // Outtake NOTE
    driverController
        .rightBumper()
        .onTrue(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, false))
        .onFalse(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, true));

    /* All Intakes */
    // Intake NOTE
    driverController
        .leftTrigger()
        .onTrue(
            new AllIntakesRun(
                m_actuatorSubsystem,
                m_otbIntakeSubsystem,
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                false))
        .onFalse(
            new AllIntakesRun(
                m_actuatorSubsystem,
                m_otbIntakeSubsystem,
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                true));
    // Outtake NOTE
    driverController
        .leftBumper()
        .onTrue(
            new AllIntakesRun(
                m_actuatorSubsystem,
                m_otbIntakeSubsystem,
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                false))
        .onFalse(
            new AllIntakesRun(
                m_actuatorSubsystem,
                m_otbIntakeSubsystem,
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                true));
  }

  public void auxControllerBindings() {
    auxController
        .start()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                0,
                ShooterConstants.FAR_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    auxController
        .button(10)
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                0,
                ShooterConstants.FAR_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    // Shoot
    auxController
        .a()
        .onTrue(new Shoot(m_feederSubsystem, m_armSubsystem, m_shooterSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* Feeder */
    // Forward
    auxController
        .x()
        .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(2500), m_feederSubsystem))
        .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));
    // Backward
    auxController
        .b()
        .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(-500), m_feederSubsystem))
        .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));

    /* Wrist */
    // Increases angle of the Wrist by 1 degree
    auxController
        .povLeft()
        .onTrue(
            new RunCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
                m_wristSubsystem))
        .onFalse(new RunCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));
    ;
    // Decreases angle of the Wrist by 1 degree
    auxController
        .povRight()
        .onTrue(
            new RunCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
                m_wristSubsystem))
        .onFalse(new RunCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));
    ;

    /* Arm */
    // Increases angle of the Arm by 1 degree
    auxController
        .povUp()
        .onTrue(
            new RunCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)), m_armSubsystem))
        .onFalse(new RunCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
    // Decreases angle of the Arm by 1 degree
    auxController
        .povDown()
        .onTrue(
            new RunCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)), m_armSubsystem))
        .onFalse(new RunCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
    ;

    // /* Scoring SPEAKER when up against it */
    auxController
        .leftTrigger()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                WristConstants.SUBWOOFER_RAD,
                ShooterConstants.CLOSE_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // auxController
    //     .button(10)
    //     .onTrue(
    //         new PositionToShoot(
    //             m_feederSubsystem,
    //             m_shooterSubsystem,
    //             m_wristSubsystem,
    //             WristConstants.SUBWOOFER_RAD,
    //             3000))
    //     .onFalse(
    //         new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem,
    // m_feederSubsystem));
    // auxController
    //     .button(9)
    //     .onTrue(
    //         new PositionToShoot(
    //             m_feederSubsystem,
    //             m_shooterSubsystem,
    //             m_wristSubsystem,
    //             WristConstants.SUBWOOFER_RAD,
    //             3000))
    //     .onFalse(
    //         new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem,
    // m_feederSubsystem));

    // /* Scoring SPEAKER when up against the PODIUM */
    auxController
        .rightTrigger()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                WristConstants.PODIUM_RAD,
                ShooterConstants.CLOSE_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* AMP Scoring */
    // Scoring AMP from the frontside
    auxController
        .leftBumper()
        .onTrue(new PositionAmpScoreFrontSide(m_armSubsystem, m_wristSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Scoring from the backside
    auxController
        .rightBumper()
        .onTrue(new PositionAmpScoreBackside(m_armSubsystem, m_wristSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* SOURCE Pickup */
    // Picking up from SOURCE, backside
    auxController
        .y()
        .onTrue(new SourcePickUpBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    auxController
        .back()
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(5500), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem));
  }

  /** Aux Contoller binding for development or manual control purposes */
  //   public void devControllerBindings() {
  //     /* Enable PID for the Wrist, Arm, and Shooter */
  //     devController
  //         .start()
  //         .onTrue(new InstantCommand(() -> this.enablePID(true), m_wristSubsystem,
  // m_armSubsystem));
  //     /* Disable PID for the Wrist, Arm, and Shooter */
  //     devController
  //         .back()
  //         .onTrue(new InstantCommand(() -> this.enablePID(false), m_wristSubsystem,
  // m_armSubsystem));

  //     // Manual Wrist Control
  //     m_wristSubsystem.setDefaultCommand(
  //         new RunCommand(
  //             () -> m_wristSubsystem.setWristPercentSpeed(devController.getRightY()),
  //             m_wristSubsystem));
  //     // Manual Arm Control
  //     m_armSubsystem.setDefaultCommand(
  //         new RunCommand(
  //             () -> m_armSubsystem.setArmPercentSpeed(devController.getLeftY()),
  // m_armSubsystem));

  //     // Subwoofer Shooter RPM w/ PID
  //     auxController
  //         .back()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_shooterSubsystem.setSetpoint(ShooterConstants.CLOSE_RPM),
  //                 m_shooterSubsystem))
  //         .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0),
  // m_shooterSubsystem));

  //     // Subwoofer Shooter RPM w/o PID
  //     devController
  //         .rightTrigger()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_shooterSubsystem.setBothPercentSpeed(0.65), m_shooterSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_shooterSubsystem.setBothPercentSpeed(0.0), m_shooterSubsystem));

  //     // Feeder backwards w/o PID
  //     devController
  //         .a()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_feederSubsystem.setFeederPercentSpeed(-0.2), m_feederSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_feederSubsystem.setFeederPercentSpeed(0), m_feederSubsystem));
  //     // Feeder forwards w/o PID
  //     devController
  //         .b()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_feederSubsystem.setFeederPercentSpeed(0.4), m_feederSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_feederSubsystem.setFeederPercentSpeed(0), m_feederSubsystem));

  //     // Feeder forward w/ PID
  //     devController
  //         .y()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_feederSubsystem.setSetpoint(FeederConstants.SPEAKER_RPM),
  //                 m_feederSubsystem))
  //         .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0),
  // m_feederSubsystem));
  //     // Feeder backwards w/ PID
  //     devController
  //         .x()
  //         .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(-500),
  // m_feederSubsystem))
  //         .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0),
  // m_feederSubsystem));

  //     // pov buttons on aux controller, switch over when best suited

  //     /* Enable to use adjustable PID, FF and Tapezopid values from SmartDashboard */
  //     devController
  //         .button(9)
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> enableTesting(true), m_armSubsystem, m_wristSubsystem,
  // m_shooterSubsystem));
  //     /* Disable to use adjustable PID, FF and Tapezopid values from SmartDashboard */
  //     devController
  //         .button(10)
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> enableTesting(false), m_armSubsystem, m_wristSubsystem,
  // m_shooterSubsystem));
  //   }
}
