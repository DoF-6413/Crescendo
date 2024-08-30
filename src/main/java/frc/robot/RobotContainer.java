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
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.BeamBreakPickUp;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PickUp;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PreloadShot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ReverseNote;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootAtAngle;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootWhenReady;
import frc.robot.Commands.TeleopCommands.AmpScore.Backside.*;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Commands.TeleopCommands.Intakes.*;
import frc.robot.Commands.TeleopCommands.SourcePickup.SourcePickUpBackside;
import frc.robot.Commands.TeleopCommands.SpeakerScore.OverShot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.Shoot;
import frc.robot.Commands.VisionCommands.*;
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbIntake.*;
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
  private final OTBIntake m_otbIntakeSubsystem;
  private final Actuator m_actuatorSubsystem;
  private final Shooter m_shooterSubsystem;
  private final Feeder m_feederSubsystem;
  private final Wrist m_wristSubsystem;

  // Utilities
  private final PoseEstimator m_poseEstimator;
  private final PathPlanner m_pathPlanner;
  private final BeamBreak m_beamBreak;
  int notePickUpCounter = 0;

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
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator, m_gyroSubsystem);
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
        "FeederReverse", new ReverseNote(m_feederSubsystem, m_shooterSubsystem, m_beamBreak));
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
        "AutoAlignWrist", new AimWrist(m_wristSubsystem, m_armSubsystem, m_poseEstimator));

    // Auto Shooting
    NamedCommands.registerCommand(
        "PreloadShot",
        new PreloadShot(
            m_feederSubsystem,
            m_shooterSubsystem,
            m_wristSubsystem,
            m_armSubsystem,
            ShooterConstants.CLOSE_RPM));
    NamedCommands.registerCommand(
        "SubwooferShot",
        new ShootAtAngle(
            m_shooterSubsystem,
            m_feederSubsystem,
            m_armSubsystem,
            m_wristSubsystem,
            m_beamBreak,
            ShooterConstants.CLOSE_RPM,
            ArmConstants.SUBWOOFER_RAD,
            WristConstants.SUBWOOFER_RAD));
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
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            m_feederSubsystem,
            m_actuatorSubsystem,
            m_armSubsystem,
            m_wristSubsystem,
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
        new InstantCommand(
            () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(-1), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "UTBStop",
        new InstantCommand(
            () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));
    NamedCommands.registerCommand(
        "PickUp",
        new PickUp(
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            CommandConstants.RUN_INTAKE));
    NamedCommands.registerCommand(
        "PickUpStop",
        new PickUp(
            m_actuatorSubsystem,
            m_otbIntakeSubsystem,
            m_utbIntakeSubsystem,
            CommandConstants.STOP_INTAKE));
    NamedCommands.registerCommand(
        "BeamBreakPickUp",
        new BeamBreakPickUp(
            m_utbIntakeSubsystem, m_feederSubsystem, m_shooterSubsystem, m_beamBreak));

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
        new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Gyro heading update
    NamedCommands.registerCommand(
        "ZeroYaw", new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem));

    /* PathPlanner Autos */
    // Test Autos
    // autoChooser.addOption("Auto1", new PathPlannerAuto("Auto1"));
    // autoChooser.addOption("test1", new PathPlannerAuto("test1"));
    // autoChooser.addOption("test2", new PathPlannerAuto("test2"));
    // autoChooser.addOption("test3", new PathPlannerAuto("test3"));
    // autoChooser.addOption("2M Test", new PathPlannerAuto("2 meter forwards"));
    // autoChooser.addOption("Override Test", new PathPlannerAuto("Speaker"));
    // autoChooser.addOption("Square Test", new PathPlannerAuto("Square"));
    // autoChooser.addOption("Command Testing", new PathPlannerAuto("Command Testing"));
    // autoChooser.addOption("Midfield Test", new PathPlannerAuto("Midfield Test"));
    // 2 Piece
    // autoChooser.addOption("2 Piece Vision", new PathPlannerAuto("2P Vision"));
    autoChooser.addOption("2 Piece (Vision)", new PathPlannerAuto("2P Vision 2.0"));
    // 3 Piece
    autoChooser.addOption("3 Piece (Vision)", new PathPlannerAuto("3P Vision"));
    // 4 Piece
    autoChooser.addOption("4 Piece (Vision)", new PathPlannerAuto("4P Vision"));
    autoChooser.addOption("4 Piece Center", new PathPlannerAuto("4P Center 5"));

    /* Deadreckoned Autos */
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

  /** Uses the current drawn by the UTB to determine if a NOTE has been picked up */
  public void isNotePickedUp() {
    if (m_utbIntakeSubsystem.getCurrentDraw() > 20 && m_utbIntakeSubsystem.getCurrentDraw() < 55) {
      notePickUpCounter += 1;
      if (notePickUpCounter >= 20) {
        SmartDashboard.putBoolean("Is Note Picked Up", true);
      }
    } else if (driverController.rightBumper().getAsBoolean()
        || auxController.a().getAsBoolean()
        || (m_shooterSubsystem.getSetpoint() > 0 && m_feederSubsystem.getSetpoint() > 0)) {
      notePickUpCounter = 0;
      SmartDashboard.putBoolean("Is Note Picked Up", false);
    }
  }

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
                CommandConstants.RUN_INTAKE))
        .onFalse(
            new AllIntakesRun(
                m_actuatorSubsystem,
                m_otbIntakeSubsystem,
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                CommandConstants.STOP_INTAKE))
        .onFalse(new ShooterRev(m_feederSubsystem, m_shooterSubsystem, m_beamBreak));
    // UTB Intake (Intake)
    driverController
        .rightTrigger()
        .onTrue(
            new UTBIntakeRun(
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                CommandConstants.INTAKE_INWARDS,
                CommandConstants.RUN_INTAKE))
        .onFalse(
            new UTBIntakeRun(
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                CommandConstants.INTAKE_INWARDS,
                CommandConstants.STOP_INTAKE))
        .onFalse(new ShooterRev(m_feederSubsystem, m_shooterSubsystem, m_beamBreak));
    // UTB Intake (Outtake)
    driverController
        .leftBumper()
        .onTrue(
            new UTBIntakeRun(
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                CommandConstants.INTAKE_OUTWARDS,
                CommandConstants.RUN_INTAKE))
        .onFalse(
            new UTBIntakeRun(
                m_utbIntakeSubsystem,
                m_feederSubsystem,
                CommandConstants.INTAKE_OUTWARDS,
                CommandConstants.STOP_INTAKE));

    /* Release NOTE */
    driverController
        .rightBumper()
        .onTrue(new Shoot(m_feederSubsystem, m_armSubsystem, m_shooterSubsystem));

    // /* Align to AMP */
    // driverController
    //     .b()
    //     .onTrue(
    //         new ConditionalCommand(
    //             m_pathPlanner.pathFindToPose(PathFindingConstants.AMP_RED_END_POSE),
    //             m_pathPlanner.pathFindToPose(PathFindingConstants.AMP_BLUE_END_POSE),
    //             () -> RobotStateConstants.getAlliance().get() == DriverStation.Alliance.Red));

    // /* Rumble */
    // if (m_utbIntakeSubsystem.getCurrentDraw() > 10) {
    //   driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
    // } else {
    //   driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    // }
  }

  /** Contoller keybinds for the aux contoller port */
  public void auxControllerBindings() {
    /* Release piece */
    // auxController.a().onTrue(new Shoot(m_feederSubsystem, m_armSubsystem, m_shooterSubsystem));

    /* Feeder */
    auxController
        .a()
        .onTrue(
            new InstantCommand(
                () -> m_feederSubsystem.setSetpoint(FeederConstants.SPEAKER_RPM),
                m_feederSubsystem))
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
                auxController,
                m_beamBreak))
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
                ArmConstants.DEFAULT_POSITION_RAD,
                ShooterConstants.MID_RANGE_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));
    // Overshot
    auxController
        .button(9)
        .onTrue(
            new OverShot(m_armSubsystem, m_feederSubsystem, m_shooterSubsystem, m_wristSubsystem))
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
    // Machine gun feeding
    auxController
        .x()
        .onTrue(
            new InstantCommand(
                () -> m_shooterSubsystem.setSetpoint(ShooterConstants.MID_RANGE_RPM),
                m_shooterSubsystem))
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
                ArmConstants.DEFAULT_POSITION_RAD,
                ShooterConstants.MIDFIELD_FEEDING_RPM))
        .onFalse(
            new ZeroAll(m_wristSubsystem, m_armSubsystem, m_shooterSubsystem, m_feederSubsystem));

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
    // Increases angle of the Arm by 1 degree
    auxController
        .povUp()
        .onTrue(
            new RunCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(1)), m_armSubsystem))
        .onFalse(new InstantCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
    // Decreases angle of the Arm by 1 degree
    auxController
        .povDown()
        .onTrue(
            new RunCommand(
                () -> m_armSubsystem.incrementArmGoal(Units.degreesToRadians(-1)), m_armSubsystem))
        .onFalse(new InstantCommand(() -> m_armSubsystem.incrementArmGoal(0), m_armSubsystem));
    /* Wrist */
    // Increases angle of the Wrist by 1 degree
    auxController
        .povRight()
        .onTrue(
            new RunCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(1)),
                m_wristSubsystem))
        .onFalse(
            new InstantCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));
    // Decreases angle of the Wrist by 1 degree
    auxController
        .povLeft()
        .onTrue(
            new RunCommand(
                () -> m_wristSubsystem.incrementWristGoal(Units.degreesToRadians(-1)),
                m_wristSubsystem))
        .onFalse(
            new InstantCommand(() -> m_wristSubsystem.incrementWristGoal(0), m_wristSubsystem));

    // // Rumble when ready to shoot
    // if (m_shooterSubsystem.bothAtSetpoint() && m_shooterSubsystem.getAverageVelocityRPM() >=
    // 3000) {
    //   auxController.getHID().setRumble(RumbleType.kBothRumble, 1);
    // } else {
    //   auxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    // }
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
