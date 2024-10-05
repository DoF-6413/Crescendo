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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.LeaveAuto;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.OnePieceAuto;
import frc.robot.Commands.AutonomousCommands.PathPlannerAutos;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PickUp;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PreloadShot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ReverseNote;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootAtAngle;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootWhenReady;
import frc.robot.Commands.TeleopCommands.AmpScore.PositionAmpScoreBackside;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Commands.TeleopCommands.Intakes.ShooterRev;
import frc.robot.Commands.TeleopCommands.Intakes.UTBIntakeRun;
import frc.robot.Commands.TeleopCommands.SourcePickup.SourcePickUpBackside;
import frc.robot.Commands.TeleopCommands.SpeakerScore.*; // Position to Shoot, Overshot, Shoot
import frc.robot.Commands.VisionCommands.*;
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.gyro.*;
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
  private final UTBIntake m_utbIntakeSubsystem;
  private final Shooter m_shooterSubsystem;
  private final Feeder m_feederSubsystem;
  private final Wrist m_wristSubsystem;

  // Utilities
  private final PoseEstimator m_poseEstimator;
  private final PathPlanner m_pathPlanner;
  private final BeamBreak m_beamBreak;

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
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
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
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSim());
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
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    // Utils
    m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
    m_beamBreak = new BeamBreak();

    /* PathPlanner Commands */
    // Shooter
    NamedCommands.registerCommand(
        "Shooter1000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.SLOW_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Shooter4000",
        new InstantCommand(
            () -> m_shooterSubsystem.setSetpoint(ShooterConstants.CLOSE_RPM), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "Shooter5500",
        new RunCommand(
                () -> m_shooterSubsystem.setSetpoint(ShooterConstants.MID_RANGE_RPM),
                m_shooterSubsystem)
            .until(() -> m_shooterSubsystem.bothAtSetpoint()));
    NamedCommands.registerCommand(
        "Shooter6000",
        new InstantCommand(() -> m_shooterSubsystem.setSetpoint(6000), m_shooterSubsystem));
    NamedCommands.registerCommand(
        "StopShooter",
        new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem));

    // Feeder
    NamedCommands.registerCommand(
        "Feeder",
        new InstantCommand(
            () -> m_feederSubsystem.setSetpoint(FeederConstants.SPEAKER_RPM), m_feederSubsystem));
    NamedCommands.registerCommand(
        "FeederReverse", new ReverseNote(m_shooterSubsystem, m_feederSubsystem, m_beamBreak));
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
        "AutoAlignWrist", new AimWrist(m_armSubsystem, m_wristSubsystem, m_poseEstimator));

    // Auto Shooting
    NamedCommands.registerCommand(
        "PreloadShot",
        new PreloadShot(
            m_armSubsystem,
            m_wristSubsystem,
            m_shooterSubsystem,
            m_feederSubsystem,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "SubwooferShot",
        new ShootAtAngle(
            m_armSubsystem,
            m_wristSubsystem,
            m_shooterSubsystem,
            m_feederSubsystem,
            m_beamBreak,
            ArmConstants.SUBWOOFER_RAD,
            WristConstants.SUBWOOFER_RAD,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "SpitNote",
        new ShootWhenReady(
            m_shooterSubsystem, m_feederSubsystem, m_beamBreak, ShooterConstants.SLOW_RPM));
    NamedCommands.registerCommand(
        "ShootWhenReady",
        new ShootWhenReady(
            m_shooterSubsystem, m_feederSubsystem, m_beamBreak, ShooterConstants.MID_RANGE_RPM));

    // Vision
    NamedCommands.registerCommand("NoteAlign", new AlignToNote(m_driveSubsystem, 0.3));
    NamedCommands.registerCommand(
        "VisionPickUp",
        new VisionPickUp(
            m_driveSubsystem,
            m_utbIntakeSubsystem,
            m_armSubsystem,
            m_wristSubsystem,
            m_feederSubsystem,
            m_poseEstimator,
            m_beamBreak));

    // Rotation Override
    NamedCommands.registerCommand(
        "EnableNOTERotationOverride",
        new InstantCommand(
            () -> m_pathPlanner.enableNOTEAlignment(CommandConstants.NOTE_ROTATION_OVERRIDE_ENABLE),
            m_pathPlanner));
    NamedCommands.registerCommand(
        "DisableNOTERotationOverride",
        new InstantCommand(
            () ->
                m_pathPlanner.enableNOTEAlignment(CommandConstants.NOTE_ROTATION_OVERRIDE_DISABLE),
            m_pathPlanner));
    NamedCommands.registerCommand(
        "EnableSpeakerRotationOverride",
        new InstantCommand(
            () ->
                m_pathPlanner.enableSpeakerAlignment(
                    CommandConstants.SPEAKER_ROTATION_OVERRIDE_ENABLE),
            m_pathPlanner));
    NamedCommands.registerCommand(
        "DisableSpeakerRotationOverride",
        new InstantCommand(
            () ->
                m_pathPlanner.enableSpeakerAlignment(
                    CommandConstants.SPEAKER_ROTATION_OVERRIDE_DISABLE),
            m_pathPlanner));

    // Pick Ups
    NamedCommands.registerCommand(
        "UTB",
        new InstantCommand(() -> m_utbIntakeSubsystem.setPercentSpeed(-1), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "UTBStop",
        new InstantCommand(() -> m_utbIntakeSubsystem.setPercentSpeed(0), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "PickUp", new PickUp(m_utbIntakeSubsystem, CommandConstants.RUN_INTAKE));
    NamedCommands.registerCommand(
        "PickUpStop", new PickUp(m_utbIntakeSubsystem, CommandConstants.STOP_INTAKE));

    // Zero Commands
    NamedCommands.registerCommand(
        "ZeroWrist",
        new InstantCommand(
            () -> m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD), m_wristSubsystem));
    NamedCommands.registerCommand(
        "ZeroArm",
        new InstantCommand(
            () -> m_armSubsystem.setGoal(ArmConstants.DEFAULT_POSITION_RAD), m_armSubsystem));
    NamedCommands.registerCommand(
        "ZeroShooting",
        new InstantCommand(
            () -> {
              m_armSubsystem.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
              m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD);
              m_shooterSubsystem.setSetpoint(0);
            },
            m_armSubsystem,
            m_wristSubsystem,
            m_shooterSubsystem));
    NamedCommands.registerCommand(
        "ZeroAll",
        new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Gyro heading update
    NamedCommands.registerCommand(
        "ZeroYaw", new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem));

    /* Autos */
    // ----------Test Autos----------
    // autoChooser.addOption("test1", new PathPlannerAuto("test1"));
    // autoChooser.addOption("test2", new PathPlannerAuto("test2"));
    // autoChooser.addOption("test3", new PathPlannerAuto("test3"));
    // autoChooser.addOption("2M Test", new PathPlannerAuto("2 meter forwards"));
    // autoChooser.addOption("Override Test", new PathPlannerAuto("Speaker"));
    // autoChooser.addOption("Square Test", new PathPlannerAuto("Square"));
    // autoChooser.addOption("Command Testing", new PathPlannerAuto("Command Testing"));
    // autoChooser.addOption("Midfield Test", new PathPlannerAuto("Midfield Test"));
    autoChooser.addOption("Load Path Test", AutoBuilder.followPath(PathPlannerPath.fromPathFile("test2")));
    // ----------0 Piece----------
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Leave", new LeaveAuto(m_driveSubsystem, 3, 1));
    // ----------1 Piece----------
    autoChooser.addDefaultOption(
        "One Piece",
        new OnePieceAuto(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem));
    autoChooser.addOption("One Piece SubSource Leave", new PathPlannerAuto("SubSource Leave"));
    // ----------2 Piece----------
    autoChooser.addOption(
        "2 Piece Center (V) (Return)", new PathPlannerAuto("2P SubCenter-C2-Sub (V)"));
    // ----------3 Piece----------
    autoChooser.addOption(
        "3 Piece SC-C2-C1 (V) (Return)", new PathPlannerAuto("3P SubCenter-C2-Sub-C1-Sub (V)"));
    autoChooser.addOption(
        "3 Piece SC-C2-C3 (V) (Return)", new PathPlannerAuto("3P SubCenter-C2-Sub-C3-Sub (V)"));
    autoChooser.addOption("3 Piece SA-C1-C2", new PathPlannerAuto("SubAmp C1-C2"));
    autoChooser.addOption(
        "3 Piece Midfield SA-C1-C2 (Smart)",
        PathPlannerAutos.SubAmp_C1_C2_Smart(
            m_driveSubsystem,
            m_utbIntakeSubsystem,
            m_armSubsystem,
            m_wristSubsystem,
            m_shooterSubsystem,
            m_feederSubsystem,
            m_beamBreak,
            m_poseEstimator));
    autoChooser.addOption(
        "3 Piece SubSource Spit", new PathPlannerAuto("3P Source-M4-M5 (Spit) (V)"));
    autoChooser.addOption(
        "3 Piece Source Sub Mid Field", new PathPlannerAuto("3P SubSource-M4-M5 (V)"));
    autoChooser.addOption("3 Piece Amp sub Midfield", new PathPlannerAuto("3P SubAmp-M1-M2 (V)"));
    autoChooser.addOption(
        "3 Piece Source-Midfield M2-M4 (Displacement)",
        new PathPlannerAuto("3P Source M2-M4 (Displace)"));
    // ----------4 Piece----------
    autoChooser.addOption(
        "4 Piece (V) (Return)", new PathPlannerAuto("4P SubCenter-C2-C1-C3 (V) (R)"));
    autoChooser.addOption(
        "4 Piece SubAmp Midfield M2-M5 (Displacement)",
        new PathPlannerAuto("4P Source M2-M5 (Displace)"));

    // Adds an "auto" tab on ShuffleBoard
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());

    // Adds a delay to the deadreakoned autos
    // SmartDashboard.putNumber("Delay", 0);

    // Configure the button bindings
    configureButtonBindings();

    // SmartDashboard variable that uses the UTB current to determine if a NOTE has been intaked
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

    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    /** Driver Controls */
    this.driverControllerBindings();

    /** Aux Controls */
    this.auxControllerBindings();
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
    m_shooterSubsystem.setBrakeMode(!isDisabled);
    m_utbIntakeSubsystem.setUTBIntakeBrakeMode(!isDisabled);
  }

  /** Sets the setpoint/position to zero */
  public void setAllSetpointsZero() {
    m_shooterSubsystem.setSetpoint(0);
    m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD);
    m_armSubsystem.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
    m_feederSubsystem.setSetpoint(0);
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

  /**
   * Toggles the use of the back cameras in the Pose Estimator
   *
   * @param enable True = enable, False = disable
   */
  public void enableVision(boolean enable) {
    m_poseEstimator.enableVision(enable);
  }

  /** Controller keybinds for the driver contoller port */
  public void driverControllerBindings() {
    /* Driving the robot */
    m_driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
                m_driveSubsystem,
                m_gyroSubsystem,
                m_poseEstimator,
                driverController,
                1,
                () -> m_armSubsystem.getGoal() >= ArmConstants.SOURCE_BACK_SIDE_RAD)
            .withName("DefaultDriveCommand"));

    /* Reset Gyro heading */
    driverController
        .a()
        .onTrue(
            new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem)
                .withName("ZeroYaw"));

    // UTB Intake (Intake)
    driverController
        .rightTrigger()
        .onTrue(
            new UTBIntakeRun(
                    m_utbIntakeSubsystem,
                    m_feederSubsystem,
                    CommandConstants.INTAKE_INWARDS,
                    CommandConstants.RUN_INTAKE)
                .unless(m_beamBreak::isNoteDetected)
                .withName("UTBIntakeRun"))
        .onFalse(
            new UTBIntakeRun(
                    m_utbIntakeSubsystem,
                    m_feederSubsystem,
                    CommandConstants.INTAKE_INWARDS,
                    CommandConstants.STOP_INTAKE)
                .withName("IntakesStop"))
        .onFalse(
            new ShooterRev(m_feederSubsystem, m_shooterSubsystem, m_beamBreak)
                .withName("ShooterRev"));
    // UTB Intake (Outtake)
    driverController
        .leftBumper()
        .onTrue(
            new UTBIntakeRun(
                    m_utbIntakeSubsystem,
                    m_feederSubsystem,
                    CommandConstants.INTAKE_OUTWARDS,
                    CommandConstants.RUN_INTAKE)
                .withName("UTBIntakeRun"))
        .onFalse(
            new UTBIntakeRun(
                    m_utbIntakeSubsystem,
                    m_feederSubsystem,
                    CommandConstants.INTAKE_OUTWARDS,
                    CommandConstants.STOP_INTAKE)
                .withName("IntakesStop"));

    /* Release NOTE */
    driverController
        .rightBumper()
        .onTrue(
            new Shoot(m_armSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ShootCommand"));
  }

  /** Contoller keybinds for the aux contoller port */
  public void auxControllerBindings() {
    /* Feeder */
    auxController
        .a()
        .onTrue(
            new InstantCommand(
                    () -> m_feederSubsystem.setSetpoint(FeederConstants.SPEAKER_RPM),
                    m_feederSubsystem)
                .withName("FeederRun"))
        .onFalse(
            new InstantCommand(() -> m_feederSubsystem.setSetpoint(0), m_feederSubsystem)
                .withName("FeederStop"));

    auxController
        .start()
        .onTrue(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(-500), m_shooterSubsystem))
        .onFalse(new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem));

    /* SPEAKER Scoring */
    // Subwoofer (w/o vision)
    auxController
        .leftTrigger()
        .onTrue(
            new PositionToShoot(
                    m_armSubsystem,
                    m_wristSubsystem,
                    m_shooterSubsystem,
                    m_feederSubsystem,
                    ArmConstants.SUBWOOFER_RAD,
                    WristConstants.SUBWOOFER_RAD,
                    ShooterConstants.CLOSE_RPM)
                .withName("SubwooferPosition"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));
    // Position to shoot with Vision
    auxController
        .rightTrigger()
        .onTrue(
            new AimShooter(
                    m_armSubsystem,
                    m_wristSubsystem,
                    m_shooterSubsystem,
                    m_feederSubsystem,
                    m_poseEstimator,
                    m_beamBreak,
                    auxController)
                .withName("AimShooter"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));
    // PODIUM (w/o vision)
    auxController
        .leftBumper()
        .onTrue(
            new PositionToShoot(
                    m_armSubsystem,
                    m_wristSubsystem,
                    m_shooterSubsystem,
                    m_feederSubsystem,
                    ArmConstants.DEFAULT_POSITION_RAD,
                    WristConstants.PODIUM_RAD,
                    ShooterConstants.MID_RANGE_RPM)
                .withName("PodiumPosition"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));
    // Overshot
    auxController
        .button(9)
        .onTrue(
            new OverShot(m_armSubsystem, m_wristSubsystem, m_feederSubsystem, m_shooterSubsystem)
                .withName("OvershotPosition"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));

    /* AMP Scoring */
    // Backside
    auxController
        .rightBumper()
        .onTrue(
            new PositionAmpScoreBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem)
                .withName("AmpPosition"))
        .onTrue(
            new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem)
                .withName("ShooterStop"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));

    /* SOURCE */
    // Pick up configuration
    auxController
        .y()
        .onTrue(
            new SourcePickUpBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem)
                .withName("SourcePosition"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));
    // Machine gun feeding
    auxController
        .x()
        .onTrue(
            new InstantCommand(
                    () -> m_shooterSubsystem.setSetpoint(ShooterConstants.MACHINE_GUN_FEEDING_RPM),
                    m_shooterSubsystem)
                .withName("ShooterRPM5000"))
        .onFalse(
            new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem)
                .withName("ShooterStop"));

    /* Misc */
    // Feeding shot from Midfield
    auxController
        .b()
        .onTrue(
            new PositionToShoot(
                    m_armSubsystem,
                    m_wristSubsystem,
                    m_shooterSubsystem,
                    m_feederSubsystem,
                    ArmConstants.DEFAULT_POSITION_RAD,
                    WristConstants.SUBWOOFER_RAD,
                    ShooterConstants.MIDFIELD_FEEDING_RPM)
                .withName("MidfieldFeedingPosition"))
        .onFalse(
            new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
                .withName("ZeroAll"));

    // /* Arm */
    // // Up by 1 degree on each button press
    // auxController
    //     .povUp()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)),
    // m_armSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)),
    // m_armSubsystem));
    // // Down by 1 degree on each button press
    // auxController
    //     .povDown()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)),
    // m_armSubsystem))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(0)),
    // m_armSubsystem));
    // /* Wrist */
    // // In by 1 degree on each button press
    // auxController
    //     .povLeft()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
    //             m_wristSubsystem));
    // // Out by 1 degree on each button press
    // auxController
    //     .povRight()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
    //             m_wristSubsystem));

    /* Arm */
    // Continuously increases angle of the Arm by 1 degree
    auxController
        .povUp()
        .onTrue(
            new RunCommand(
                    () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)),
                    m_armSubsystem)
                .withName("IncrementArm"))
        .onFalse(
            new InstantCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem)
                .withName("ArmStop"));
    // Continuously decreases angle of the Arm by 1 degree
    auxController
        .povDown()
        .onTrue(
            new RunCommand(
                    () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)),
                    m_armSubsystem)
                .withName("DecrementArm"))
        .onFalse(
            new InstantCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem)
                .withName("ArmStop"));
    /* Wrist */
    // Continuously increases angle of the Wrist by 1 degree
    auxController
        .povRight()
        .onTrue(
            new RunCommand(
                    () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
                    m_wristSubsystem)
                .withName("IncrementWrist"))
        .onFalse(
            new InstantCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem)
                .withName("WristStop"));
    // Continuously decreases angle of the Wrist by 1 degree
    auxController
        .povLeft()
        .onTrue(
            new RunCommand(
                    () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
                    m_wristSubsystem)
                .withName("DecrementWrist"))
        .onFalse(
            new InstantCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem)
                .withName("WristStop"));
  }
}
