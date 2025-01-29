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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces.LeaveAuto;
import frc.robot.Commands.TeleopCommands.DefaultDriveCommand;
import frc.robot.Commands.TeleopCommands.DriveCommands;
import frc.robot.Commands.ZeroCommands.*; // Actuator, Arm, Wrist, Shooter, and Feeder
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.actuator.ActuatorIO;
//import frc.robot.Subsystems.actuator.ActuatorIOSim;
import frc.robot.Subsystems.actuator.ActuatorIOSparkMax;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbroller.OTBRoller;
import frc.robot.Subsystems.otbroller.OTBRollerIO;
//import frc.robot.Subsystems.otbroller.OTBRollerIOSim;
import frc.robot.Subsystems.otbroller.OTBRollerIOSparkMax;
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
  private final OTBRoller m_otbRollerSubsystem;
  private final Actuator m_actuatorSubsystem;
  // private final Shooter m_shooterSubsystem;
  // private final Feeder m_feederSubsystem;
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
        m_gyroSubsystem = new Gyro(new GyroIOPigeon2());
        m_driveSubsystem =
            new Drive(
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIOSparkMax());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
        m_otbRollerSubsystem = new OTBRoller(new OTBRollerIOSparkMax());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSparkMax());
        // m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        // m_feederSubsystem = new Feeder(new FeederIOTalonFX());
        m_wristSubsystem = new Wrist(new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiates physics sim IO implementations
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
        m_otbRollerSubsystem = new OTBRoller(new OTBRollerIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        // m_shooterSubsystem = new Shooter(new ShooterIO() {});
        // m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
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
        m_otbRollerSubsystem = new OTBRoller(new OTBRollerIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        // m_shooterSubsystem = new Shooter(new ShooterIO() {});
        // m_feederSubsystem = new Feeder(new FeederIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    // Utils
    m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem);
    m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
    m_beamBreak = new BeamBreak();

    /* Autos */
    // ----------Test Autos----------
    // autoChooser.addOption("test1", new PathPlannerAuto("test1"));
    // autoChooser.addOption("test2", new PathPlannerAuto("test2"));
    // autoChooser.addOption("test3", new PathPlannerAuto("test3"));
    // autoChooser.addOption("2M Test", new PathPlannerAuto("2 meter forwards"));
    // autoChooser.addOption("Override Test", new PathPlannerAuto("Speaker"));
    // autoChooser.addOption("Square Test", new PathPlannerAuto("Square"));
    // autoChooser.addOption("Command Testing", new PathPlannerAuto("Command Testing"));
    // ----------0 Piece----------
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Leave", new LeaveAuto(m_driveSubsystem, 3, 1));
    
    // Adds an "auto" tab on ShuffleBoard
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
    m_actuatorSubsystem.setBrakeMode(!isDisabled);
    m_armSubsystem.setBrakeMode(!isDisabled);
    m_wristSubsystem.setBrakeMode(!isDisabled);
    // m_shooterSubsystem.setBrakeMode(!isDisabled);
    m_utbIntakeSubsystem.setBrakeMode(!isDisabled);
  }

  /** Sets the setpoint/position to zero */
  public void setAllSetpointsZero() {
    // m_shooterSubsystem.setSetpoint(0);
    m_wristSubsystem.setGoal(WristConstants.DEFAULT_POSITION_RAD);
    m_armSubsystem.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
    // m_feederSubsystem.setSetpoint(0);
  }

  /**
   * Toggles PID tuning
   *
   * @param enable True enables PID
   */
  public void enablePID(boolean enable) {
    m_armSubsystem.enablePID(enable);
    m_wristSubsystem.enablePID(enable);
    // m_shooterSubsystem.enablePID(enable);
  }

  /**
   * Toggles the use of SmartDashboard PIDFF values
   *
   * @param enable True uses the PIDFF values typed onto SmartDashboard
   */
  public void enableTesting(boolean enable) {
    m_armSubsystem.enableTesting(enable);
    m_wristSubsystem.enableTesting(enable);
    // m_shooterSubsystem.enableTesting(enable);
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
        DriveCommands.fieldRelativeDrive(m_driveSubsystem, driverController::getLeftX, driverController::getLeftY, driverController::getRightX)
            .withName("DefaultDriveCommand"));

    /* Reset Gyro heading */
    driverController
        .a()
        .onTrue(
            new InstantCommand(() -> m_gyroSubsystem.zeroYaw(), m_gyroSubsystem)
                .withName("ZeroYaw"));

    driverController.b().onTrue(m_pathPlanner.pathFindToPose(m_poseEstimator.toAprilTag()));

    // All Intakes (Intake)
    // driverController
    //     .leftTrigger()
    //     .onTrue(
    //         new AllIntakesRun(
    //                 m_actuatorSubsystem,
    //                 m_otbRollerSubsystem,
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.RUN_INTAKE)
    //             .unless(m_beamBreak::getShooterSensor)
    //             .withName("AllIntakesRun"))
    //     .onFalse(
    //         new AllIntakesRun(
    //                 m_actuatorSubsystem,
    //                 m_otbRollerSubsystem,
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.STOP_INTAKE)
    //             .withName("AllIntakesStop"))
    //     .onFalse(
    //         new ShooterRev(m_feederSubsystem, m_shooterSubsystem, m_beamBreak)
    //             .withName("ShooterRev"));

    // UTB Intake (Intake)
    // driverController
    //     .rightTrigger()
    //     .onTrue(
    //         new UTBIntakeRun(
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.INTAKE_INWARDS,
    //                 CommandConstants.RUN_INTAKE)
    //             .unless(m_beamBreak::getShooterSensor)
    //             .withName("UTBIntakeRun"))
    //     .onFalse(
    //         new UTBIntakeRun(
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.INTAKE_INWARDS,
    //                 CommandConstants.STOP_INTAKE)
    //             .withName("IntakesStop"))
    //     .onFalse(
    //         new ShooterRev(m_feederSubsystem, m_shooterSubsystem, m_beamBreak)
    //             .withName("ShooterRev"));
    // UTB Intake (Outtake)
    // driverController
    //     .leftBumper()
    //     .onTrue(
    //         new UTBIntakeRun(
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.INTAKE_OUTWARDS,
    //                 CommandConstants.RUN_INTAKE)
    //             .withName("UTBIntakeRun"))
    //     .onFalse(
    //         new UTBIntakeRun(
    //                 m_utbIntakeSubsystem,
    //                 m_feederSubsystem,
    //                 CommandConstants.INTAKE_OUTWARDS,
    //                 CommandConstants.STOP_INTAKE)
    //             .withName("IntakesStop"));

    /* Release NOTE */
    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         new Shoot(m_armSubsystem, m_shooterSubsystem, m_feederSubsystem)
    //             .withName("ShootCommand"));

    /* Brings Actuator back to its default position (all the way up) */
    driverController.start().onTrue(new ActuatorToZero(m_actuatorSubsystem));
  }

  /** Contoller keybinds for the aux contoller port */
  public void auxControllerBindings() {

    // // Position to shoot with Vision
    // auxController
    //     .rightTrigger()
    //     .onTrue(
    //         new AimShooter(
    //                 m_armSubsystem,
    //                 m_wristSubsystem,
    //                 m_shooterSubsystem,
    //                 m_feederSubsystem,
    //                 m_poseEstimator,
    //                 m_beamBreak,
    //                 auxController)
    //             .withName("AimShooter"))
    //     .onFalse(
    //         new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
    //             .withName("ZeroAll"));
    
    // /* AMP Scoring */
    // // Backside
    // auxController
    //     .rightBumper()
    //     .onTrue(
    //         new PositionAmpScoreBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem)
    //             .withName("AmpPosition"))
    //     .onTrue(
    //         new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem)
    //             .withName("ShooterStop"))
    //     .onFalse(
    //         new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
    //             .withName("ZeroAll"));

    // /* SOURCE */
    // // Pick up configuration
    // auxController
    //     .y()
    //     .onTrue(
    //         new SourcePickUpBackside(m_armSubsystem, m_wristSubsystem, m_feederSubsystem)
    //             .withName("SourcePosition"))
    //     .onFalse(
    //         new ZeroAll(m_armSubsystem, m_wristSubsystem, m_shooterSubsystem, m_feederSubsystem)
    //             .withName("ZeroAll"));
    // // Machine gun feeding
    // auxController
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //                 () -> m_shooterSubsystem.setSetpoint(ShooterConstants.MACHINE_GUN_FEEDING_RPM),
    //                 m_shooterSubsystem)
    //             .withName("ShooterRPM5000"))
    //     .onFalse(
    //         new InstantCommand(() -> m_shooterSubsystem.setSetpoint(0), m_shooterSubsystem)
    //             .withName("ShooterStop"));

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
