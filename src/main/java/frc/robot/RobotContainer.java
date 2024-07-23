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
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.LeaveAuto;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceAuto;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceLeaveCenter;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.TwoPieceReturnSub;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.AutoShoot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PickUp;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootingReady;
import frc.robot.Commands.TeleopCommands.AmpScore.Backside.*;
import frc.robot.Commands.TeleopCommands.AmpScore.Frontside.*;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Commands.TeleopCommands.Intakes.*;
import frc.robot.Commands.TeleopCommands.SourcePickup.SourcePickUpBackside;
import frc.robot.Commands.TeleopCommands.SpeakerScore.OverShot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Commands.TeleopCommands.VisionAutomations.*; // AimShooter, AlignToNote, PickUpNote
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.climber.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.photonVision.Vision;
import frc.robot.Subsystems.photonVision.VisionIO;
import frc.robot.Subsystems.photonVision.VisionIOArduCam;
import frc.robot.Subsystems.photonVision.VisionIOSim;
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
  private final BeamBreak beam;
  int counter = 0;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);
  private final CommandXboxController devController =
      new CommandXboxController(OperatorConstants.DEV_CONTROLLER);

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
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    // Utils
    m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator, m_gyroSubsystem);
    beam = new BeamBreak();

    /* PathPlanner Register Commands */
    // Shooter/Feeder
    NamedCommands.registerCommand(
        "Shooter1000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.SLOW_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Shooter4000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.CLOSE_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Shooter5000",
        new InstantCommand(() -> m_shooterSubsystem.setSetpoint(5000), m_shooterSubsystem));
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
        "FeederReverse",
        new InstantCommand(() -> m_feederSubsystem.setSetpoint(-200), m_feederSubsystem));
    NamedCommands.registerCommand(
        "StopFeeder",
        new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));
    // Angles
    NamedCommands.registerCommand(
        "SubwooferAngle",
        new InstantCommand(
            () -> {
              m_wristSubsystem.setGoal(WristConstants.SUBWOOFER_RAD);
              m_armSubsystem.setGoal(ArmConstants.SUBWOOFER_RAD);
            },
            m_wristSubsystem,
            m_armSubsystem));
    NamedCommands.registerCommand(
        "PodiumAngle",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(WristConstants.PODIUM_RAD), m_wristSubsystem));
    NamedCommands.registerCommand(
        "LineAngle",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(Units.degreesToRadians(9)), m_wristSubsystem));
    NamedCommands.registerCommand(
        "2P Note",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(Units.degreesToRadians(15)), m_wristSubsystem));
    NamedCommands.registerCommand(
        "ChainAngle",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(Units.degreesToRadians(4)), m_wristSubsystem));
    NamedCommands.registerCommand(
        "WingAngle",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(WristConstants.WING_RAD), m_wristSubsystem));
    NamedCommands.registerCommand(
        "AutoAlign",
        new AimShooter(
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_poseEstimator,
            m_feederSubsystem,
            beam));

    // Vision
    NamedCommands.registerCommand("VisionAlign", new AlignToNote(m_driveSubsystem, 0.3));
    NamedCommands.registerCommand(
        "VisionPickUp",
        new PickUpNote(
            m_driveSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            m_feederSubsystem,
            m_actuatorSubsystem));

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
    // Auto Shooting
    NamedCommands.registerCommand(
        "SubwooferShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            WristConstants.SUBWOOFER_RAD,
            ArmConstants.SUBWOOFER_RAD,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "PodiumShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            WristConstants.PODIUM_RAD,
            0,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "ChainShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            WristConstants.CHAIN_RAD,
            0,
            ShooterConstants.MID_RANGE_RPM));
    NamedCommands.registerCommand(
        "LegShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            Units.degreesToRadians(-7), // TODO: Update angle
            0,
            ShooterConstants.FAR_RPM));
    NamedCommands.registerCommand(
        "WingShot",
        new AutoShoot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            m_gyroSubsystem,
            Units.degreesToRadians(-8.5), // TODO: Verify angle
            0,
            ShooterConstants.FAR_RPM));
    // Zero Commands
    NamedCommands.registerCommand(
        "ZeroWrist",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD), m_wristSubsystem));
    NamedCommands.registerCommand(
        "ZeroArm", new InstantCommand(() -> m_armSubsystem.setGoal(0), m_armSubsystem));
    NamedCommands.registerCommand(
        "ZeroShooting",
        new InstantCommand(
            () -> {
              m_armSubsystem.setGoal(0);
              m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD);
              m_shooterSubsystem.setSetpoint(0);
            },
            m_armSubsystem,
            m_wristSubsystem,
            m_shooterSubsystem));
    NamedCommands.registerCommand(
        "ZeroAll",
        new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Gyro heading reset
    NamedCommands.registerCommand(
        "ZeroYaw", new InstantCommand(() -> m_driveSubsystem.updateHeading(), m_driveSubsystem));
    NamedCommands.registerCommand(
        "ChassisAutoAlign",
        new DefaultDriveCommand(
            m_driveSubsystem, auxController, m_gyroSubsystem, m_poseEstimator, 1));
    // WaitUntil Commands
    NamedCommands.registerCommand(
        "ShooterReady",
        new InstantCommand(() -> m_shooterSubsystem.bothAtSetpoint(), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "WristReady", new InstantCommand(() -> m_wristSubsystem.atSetpoint(), m_wristSubsystem));
    NamedCommands.registerCommand(
        "ShootingReady", new ShootingReady(m_shooterSubsystem, m_wristSubsystem, m_armSubsystem));

    /* PathPlanner Autos */
    // Test Autos
    // autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    // autoChooser.addOption("test1", new PathPlannerAuto("test1"));
    // autoChooser.addOption("test2", new PathPlannerAuto("test2"));
    // autoChooser.addOption("test3", new PathPlannerAuto("test3"));
    autoChooser.addOption("2M Test", new PathPlannerAuto("2 meter forwards"));
    // autoChooser.addOption("Command Testing", new PathPlannerAuto("Command Testing"));
    // autoChooser.addOption("Midfield Test", new PathPlannerAuto("Midfield Test"));
    // 2 Piece
    // autoChooser.addDefaultOption("2 Piece Center", new PathPlannerAuto("2P Center"));
    autoChooser.addOption("2 Piece Center 3.0", new PathPlannerAuto("Center 3"));
    autoChooser.addOption("2 Piece Center (2x subwoofer shot)", new PathPlannerAuto("Center"));
    autoChooser.addOption("2 Piece Center", new PathPlannerAuto("Center 2"));
    autoChooser.addOption("2 Piece Amp", new PathPlannerAuto("2P Amp"));
    autoChooser.addOption("2 Piece Cool Side", new PathPlannerAuto("2P Cool Side"));
    autoChooser.addOption("2 Piece Vision", new PathPlannerAuto("2P Vision"));
    // 3 Piece
    // autoChooser.addOption("3 Piece Center", new PathPlannerAuto("3P Center"));
    // autoChooser.addOption("3 Piece Cool Side", new PathPlannerAuto("3P Cool Side"));
    autoChooser.addOption("3 Piece Close Amp", new PathPlannerAuto("3P Center Amp"));
    autoChooser.addOption("3 Piece Close Podium", new PathPlannerAuto("3P Center Podium"));
    autoChooser.addOption(
        "3 Piece VISION Close Podium", new PathPlannerAuto("3P VIS Center Podium"));
    autoChooser.addOption("3 Piece Corner to Midfield", new PathPlannerAuto("3P Corner Mid Right"));
    autoChooser.addOption("3 Piece Source Midfield", new PathPlannerAuto("3P Mid Right"));
    // autoChooser.addOption("Liz3Piece", new PathPlannerAuto("Liz2Piece"));
    // 4 Piece
    // autoChooser.addOption("4 Piece Center", new PathPlannerAuto("4P Center"));
    // autoChooser.addOption("4 Piece Center 2.0", new PathPlannerAuto("4P Center 2"));
    // autoChooser.addOption("4 Piece Center 3.0", new PathPlannerAuto("4P Center 3"));
    autoChooser.addOption("4 Piece Center", new PathPlannerAuto("4P Center 5"));
    // autoChooser.addOption("4 Piece Center 5.0", new PathPlannerAuto("4P Center 5"));
    autoChooser.addOption("4 Piece Left Sub Midfield", new PathPlannerAuto("4P Midfield"));
    autoChooser.addOption("4 Piece Center Midfield", new PathPlannerAuto("4P Center to Midfield"));
    // autoChooser.addOption(
    //     "4 Piece Center Midfield Vision", new PathPlannerAuto("4P Center to Midfield (V)"));
    // 5+ Piece
    autoChooser.addOption("5 Piece Left Sub Midfield", new PathPlannerAuto("5P Midfield"));
    // autoChooser.addOption("5.5PieceAuto", new PathPlannerAuto("5.5PieceAuto"));
    // Adds an "auto" tab on ShuffleBoard
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());

    // Adds list of deadreckond autos to Shuffleboard
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Leave", new LeaveAuto(m_driveSubsystem, 3, 1));
    autoChooser.addDefaultOption(
        "One Piece",
        new OnePieceAuto(m_wristSubsystem, m_armSubsystem, m_feederSubsystem, m_shooterSubsystem));
    autoChooser.addOption(
        "One Piece Leave Center",
        new OnePieceLeaveCenter(
            m_driveSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
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
    // autoChooser.addOption(
    //     "Blue One Piece Leave Amp Side",
    //     new OnePieceLeaveAmpSide(
    //         m_wristSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_driveSubsystem,
    //         2,
    //         1,
    //         m_gyroSubsystem));
    // autoChooser.addOption(
    //     "Blue One Piece Leave Cool Side",
    //     new OnePieceLeaveCoolSide(
    //         m_wristSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_driveSubsystem,
    //         2,
    //         1,
    //         m_gyroSubsystem));
    // autoChooser.addOption(
    //     "Red One Piece Leave Cool Side",
    //     new OnePieceLeaveAmpSide(
    //         m_wristSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_driveSubsystem,
    //         2,
    //         1,
    //         m_gyroSubsystem));
    // autoChooser.addOption(
    //     "Red One Piece Leave Amp Side",
    //     new OnePieceLeaveCoolSide(
    //         m_wristSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_driveSubsystem,
    //         2,
    //         1,
    //         m_gyroSubsystem));
    // autoChooser.addOption(
    //     "Three Piece Leave Amp Side Blue",
    //     new ThreePieceAutoBlue(
    //         m_driveSubsystem,
    //         m_gyroSubsystem,
    //         m_wristSubsystem,
    //         m_armSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_actuatorSubsystem,
    //         m_otbIntakeSubsystem,
    //         m_utbIntakeSubsystem,
    //         2,
    //         1));
    // autoChooser.addOption(
    //     "Three Piece Leave Amp Side Red",
    //     new ThreePieceAutoRed(
    //         m_driveSubsystem,
    //         m_gyroSubsystem,
    //         m_wristSubsystem,
    //         m_armSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_actuatorSubsystem,
    //         m_otbIntakeSubsystem,
    //         m_utbIntakeSubsystem,
    //         2,
    //         1));
    // autoChooser.addOption(
    //     "4 Piece Blue",
    //     new FourPieceBlue(
    //         m_driveSubsystem,
    //         m_gyroSubsystem,
    //         m_wristSubsystem,
    //         m_armSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_actuatorSubsystem,
    //         m_otbIntakeSubsystem,
    //         m_utbIntakeSubsystem,
    //         2,
    //         1));
    // autoChooser.addOption(
    //     "4 Piece Red",
    //     new FourPieceRed(
    //         m_driveSubsystem,
    //         m_gyroSubsystem,
    //         m_wristSubsystem,
    //         m_armSubsystem,
    //         m_feederSubsystem,
    //         m_shooterSubsystem,
    //         m_actuatorSubsystem,
    //         m_otbIntakeSubsystem,
    //         m_utbIntakeSubsystem,
    //         2,
    //         1));

    // Adds a delay onto the deadreakoned autos
    SmartDashboard.putNumber("Delay", 0);

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putBoolean("Is Note Picked Up", false);
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

    /** Driver Controls */
    this.driverControllerBindings();

    /** Aux Controls */
    this.auxControllerBindings();

    // this.devControllerBindings();
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
    m_utbIntakeSubsystem.setUTBIntakeBrakeMode(!isDisabled);
    m_otbIntakeSubsystem.setBrakeMode(!isDisabled);
  }

  /**
   * Toggles PID tuning
   *
   * @param enable True enables PID
   */
  public void enablePID(boolean enable) {
    m_armSubsystem.enablePID(enable);
    m_wristSubsystem.enablePID(enable);
    m_shooterSubsystem.enablePID(enable);
  }

  /**
   * Toggles the use of SmartDashboard PIDFF values
   *
   * @param enable True uses the PIDFF values typed onto SmartDashboard
   */
  public void enableTesting(boolean enable) {
    m_armSubsystem.enableTesting(enable);
    m_wristSubsystem.enableTesting(enable);
    m_shooterSubsystem.enableTesting(enable);
  }

  /** Sets the setpoint/position to zero */
  public void setAllSetpointsZero() {
    m_shooterSubsystem.setSetpoint(0);
    m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD);
    m_armSubsystem.setGoal(0);
    m_feederSubsystem.setSetpoint(0);
  }

  /** Uses the current drawn by the UTB to determine if a NOTE has been picked up */
  //   public void isNotePickedUp() {
  //     if (m_utbIntakeSubsystem.getCurrentDraw() > 20 && m_utbIntakeSubsystem.getCurrentDraw() <
  // 55) {
  //       counter += 1;
  //       if (counter >= 25) {
  //         SmartDashboard.putBoolean("Is Note Picked Up", true);
  //       }
  //     } else if (auxController.a().getAsBoolean() || (m_shooterSubsystem.getSetpoint() > 0 &&
  // m_feederSubsystem.getSetpoint() > 0)) {
  //       counter = 0;
  //       SmartDashboard.putBoolean("Is Note Picked Up", false);
  //     }
  //   }

  /** Controller keybinds for the driver contoller port */
  public void driverControllerBindings() {
    /* Driving the robot */
    m_driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_driveSubsystem, driverController, m_gyroSubsystem, m_poseEstimator, 1));

    /* Reset Gyro heading */
    driverController
        .a()
        .onTrue(new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem));

    /* Auto pickup NOTE */
    // driverController
    //     .x()
    //     .onTrue(
    //         new PickUpNote(
    //             m_driveSubsystem,
    //             m_otbIntakeSubsystem,
    //             m_utbIntakeSubsystem,
    //             m_feederSubsystem,
    //             m_actuatorSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               m_driveSubsystem.setRaw(0, 0, 0);
    //               new AllIntakesRun(
    //                   m_actuatorSubsystem,
    //                   m_otbIntakeSubsystem,
    //                   m_utbIntakeSubsystem,
    //                   m_feederSubsystem,
    //                   true);
    //             },
    //             m_driveSubsystem,
    //             m_actuatorSubsystem,
    //             m_otbIntakeSubsystem,
    //             m_utbIntakeSubsystem,
    //             m_feederSubsystem));

    // /* Rumble */
    // if (m_utbIntakeSubsystem.getCurrentDraw() > 10) {
    //   driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    // } else {
    //   driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    // }

    /* Intakes */
    // All Intakes (Intake)
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
    // UTB Intake (Intake)
    driverController
        .rightTrigger()
        .onTrue(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, true, false))
        .onFalse(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, true));
    // UTB Intake (Outtake)
    driverController
        .rightBumper()
        .onTrue(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, false))
        .onFalse(new UTBIntakeRun(m_utbIntakeSubsystem, m_feederSubsystem, false, true));
  }

  /** Contoller keybinds for the aux contoller port */
  public void auxControllerBindings() {
    /* Release piece */
    auxController
        .a()
        .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(1000), m_feederSubsystem))
        .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem));

    /* SPEAKER Scoring */
    // Subwoofer (w/o vision)
    auxController
        .leftTrigger()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                m_armSubsystem,
                WristConstants.SUBWOOFER_RAD,
                ArmConstants.SUBWOOFER_RAD,
                ShooterConstants.CLOSE_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // PODIUM (w/o vision)
    auxController
        .leftBumper()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                m_armSubsystem,
                WristConstants.PODIUM_RAD,
                0,
                ShooterConstants.MID_RANGE_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // // WING
    // auxController
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_shooterSubsystem.setSetpoint(ShooterConstants.CLOSE_RPM),
    //             m_shooterSubsystem))
    //     .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0),
    // m_shooterSubsystem));
    // Overshot
    auxController
        .button(9)
        .onTrue(
            new OverShot(m_armSubsystem, m_feederSubsystem, m_shooterSubsystem, m_wristSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Position to shoot with Vision
    auxController
        .rightTrigger()
        .onTrue(
            new AimShooter(
                m_shooterSubsystem,
                m_wristSubsystem,
                m_armSubsystem,
                m_poseEstimator,
                m_feederSubsystem,
                beam))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* AMP Scoring */
    // Backside
    auxController
        .rightBumper()
        .onTrue(new PositionAmpScoreBackside(m_armSubsystem, m_wristSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    /* SOURCE */
    // Pick up configuration
    auxController
        .y()
        .onTrue(new SourcePickUpBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Feeding
    auxController
        .x()
        .onTrue(
            new InstantCommand(
                () -> m_shooterSubsystem.setSetpoint(ShooterConstants.FAR_RPM), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem));

    /* Misc */
    // Feeding shot from Midfield
    auxController
        .b()
        .onTrue(
            new PositionToShoot(
                m_feederSubsystem,
                m_shooterSubsystem,
                m_wristSubsystem,
                m_armSubsystem,
                WristConstants.SUBWOOFER_RAD,
                0,
                ShooterConstants.MIDFIELD_FEEDING_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

    // // Rumble when ready to shoot
    // if (m_shooterSubsystem.bothAtSetpoint() && m_shooterSubsystem.getAverageVelocityRPM() >=
    // 3000) {
    //   auxController.getHID().setRumble(RumbleType.kBothRumble, 1);
    // } else {
    //   auxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    // }

    /* Arm */
    // Up by 1 degree on each button press
    auxController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)), m_armSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)), m_armSubsystem));
    // Down by 1 degree on each button press
    auxController
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)), m_armSubsystem))
        .onFalse(
            new InstantCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)), m_armSubsystem));
    /* Wrist */
    // In by 1 degree on each button press
    auxController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
                m_wristSubsystem));
    // Out by 1 degree on each button press
    auxController
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
                m_wristSubsystem));

    // /* Wrist */
    // // Increases angle of the Wrist by 1 degree
    // auxController
    //     .povRight()
    //     .onTrue(
    //         new RunCommand(
    //             () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
    //             m_wristSubsystem))
    //     .onFalse(new RunCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));
    // // Decreases angle of the Wrist by 1 degree
    // auxController
    //     .povLeft()
    //     .onTrue(
    //         new RunCommand(
    //             () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
    //             m_wristSubsystem))
    //     .onFalse(new RunCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));

    // /* Arm */
    // // Increases angle of the Arm by 1 degree
    // auxController
    //     .povUp()
    //     .onTrue(
    //         new RunCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)),
    // m_armSubsystem))
    //     .onFalse(new RunCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
    // // Decreases angle of the Arm by 1 degree
    // auxController
    //     .povDown()
    //     .onTrue(
    //         new RunCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)),
    // m_armSubsystem))
    //     .onFalse(new RunCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
  }

  /** Backup/development button bindings for the Aux Contols */
  //   public void devControllerBindings() {
  //     //       /* Manual Contol (No PID) */
  //     //       // Arm
  //     //       m_armSubsystem.setDefaultCommand(new InstantCommand(()->
  //     //   m_armSubsystem.setArmPercentSpeed(devController.getLeftY()), m_armSubsystem));
  //     //       // Wrist
  //     //       m_wristSubsystem.setDefaultCommand(new InstantCommand(()->
  //     //   m_wristSubsystem.setWristPercentSpeed(devController.getRightY()), m_wristSubsystem));
  //     //       // Shooter
  //     //       devController.rightTrigger().onTrue(new InstantCommand(()->
  //     //   m_shooterSubsystem.setBothPercentSpeed(0.65), m_shooterSubsystem)).onFalse(new
  //     //   InstantCommand(()-> m_shooterSubsystem.setBothPercentSpeed(0), m_shooterSubsystem));
  //     //       // Feeder (Retract)
  //     //       devController.a().onTrue(new InstantCommand(()->
  //     //   m_feederSubsystem.setFeederPercentSpeed(-0.2), m_feederSubsystem)).onFalse(new
  //     //   InstantCommand(()-> m_feederSubsystem.setFeederPercentSpeed(0), m_feederSubsystem));
  //     //       // Feeder (Shoot out)
  //     //       devController.y().onTrue(new InstantCommand(()->
  //     //   m_feederSubsystem.setFeederPercentSpeed(0.4), m_feederSubsystem)).onFalse(new
  //     //   InstantCommand(()-> m_feederSubsystem.setFeederPercentSpeed(0), m_feederSubsystem));

  //     /* Controls with PID */
  //     // Shooter
  //     devController
  //         .leftTrigger()
  //         .onTrue(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(4000),
  // m_shooterSubsystem))
  //         .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0),
  // m_shooterSubsystem));
  //     devController
  //         .rightTrigger()
  //         .onTrue(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(7000),
  // m_shooterSubsystem))
  //         .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0),
  // m_shooterSubsystem));
  //     // Feeder (Retract)
  //     devController
  //         .x()
  //         .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(-500),
  // m_feederSubsystem))
  //         .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0),
  // m_feederSubsystem));
  //     // Feeder (Shoot out)
  //     devController
  //         .b()
  //         .onTrue(new InstantCommand(() -> m_feederSubsystem.setSetpoint(2000),
  // m_feederSubsystem))
  //         .onFalse(new InstantCommand(() -> m_feederSubsystem.setSetpoint(0),
  // m_feederSubsystem));
  //     // Arm (Up)
  //     devController
  //         .povUp()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)),
  // m_armSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)),
  // m_armSubsystem));
  //     // Arm (Down)
  //     devController
  //         .povDown()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)),
  // m_armSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)),
  // m_armSubsystem));
  //     // Wrist (In)
  //     devController
  //         .povLeft()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
  //                 m_wristSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(0)),
  //                 m_wristSubsystem));
  //     // Wrist (Out)
  //     devController
  //         .povRight()
  //         .onTrue(
  //             new InstantCommand(
  //                 () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
  //                 m_wristSubsystem))
  //         .onFalse(
  //             new InstantCommand(
  //                 () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(0)),
  //                 m_wristSubsystem));

  //     /* Toggle PID control */
  //     // Enable
  //     devController.back().onTrue(new InstantCommand(() -> enablePID(true)));
  //     // Disable
  //     devController.start().onTrue(new InstantCommand(() -> enablePID(false)));

  //     //     /* Toggle Testing mode */
  //     //     // Disable
  //     //     devController.button(9).onTrue(new InstantCommand(()-> enableTesting(false),
  //     // m_armSubsystem, m_wristSubsystem, m_shooterSubsystem));
  //     //     // Enable
  //     //     devController.button(10).onTrue(new InstantCommand(()-> enableTesting(true),
  //     // m_armSubsystem, m_wristSubsystem, m_shooterSubsystem));
  //     //   }
  //   }
}
