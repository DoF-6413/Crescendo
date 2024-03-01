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
import frc.robot.Commands.TeleopCommands.AmpScore;
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
  // Subsystems TODO: Add back subsystems as we get them working? If not then just uncomment them
  private final Gyro m_gyroSubsystem;
  private final Drive m_driveSubsystem;

  private final Arm m_armSubsystem;
  //   private final Vision m_visionSubsystem;
  private final Feeder m_feederSubsystem;
  // private final Climber m_climberSubsystem;
  private final UTBIntake m_utbIntakeSubsystem;
  private final OTBIntake m_otbIntakeSubsystem;
  private final Actuator m_actuatorSubsystem;
  private final Shooter m_shooterSubsystem;
  private final Wrist m_wristSubsystem;

  //   private final PoseEstimator m_poseEstimator;
  //   private final PathPlanner m_pathPlanner;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

  // Autos
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
                new ModuleIOSparkMaxTalonFX(0),
                new ModuleIOSparkMaxTalonFX(1),
                new ModuleIOSparkMaxTalonFX(2),
                new ModuleIOSparkMaxTalonFX(3),
                m_gyroSubsystem);
        m_armSubsystem = new Arm(new ArmIOSparkMax());
        // m_visionSubsystem = new Vision(new VisionIOSim());
        m_feederSubsystem = new Feeder(new FeederIOTalonFX());
        // m_climberSubsystem = new Climber(new ClimberIOSparkMax());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSparkMax());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSparkMax());
        m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        m_wristSubsystem = new Wrist(new WristIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
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
        m_feederSubsystem = new Feeder(new FeederIOSim());
        // m_climberSubsystem = new Climber(new ClimberIOSim());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSim());
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSim());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSim());
        m_shooterSubsystem = new Shooter(new ShooterIOSim());
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
        m_armSubsystem = new Arm(new ArmIO() {});
        // m_visionSubsystem = new Vision(new VisionIO() {});
        m_feederSubsystem = new Feeder(new FeederIO() {});
        // m_climberSubsystem = new Climber(new ClimberIO() {});
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_wristSubsystem = new Wrist(new WristIO() {});
        break;
    }

    // Configure the button bindings
    configureButtonBindings();

    // m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
    // m_pathPlanner = new PathPlanner(m_driveSubsystem, m_poseEstimator);
    autoChooser.addOption("Do Nothing", new InstantCommand());
    // autoChooser.addDefaultOption("Default Path", new PathPlannerAuto("ROCK"));
    Shuffleboard.getTab("Auto").add(autoChooser.getSendableChooser());
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
                m_driveSubsystem.driveWithDeadband(
                    driverController.getLeftX(),
                    driverController.getLeftY() * (-1), // Joystick on Xbox Controller is Inverted
                    (driverController.getRightX() * (1))),
            m_driveSubsystem));

    driverController
        .a()
        .onTrue(new InstantCommand(() -> m_driveSubsystem.updateHeading(), m_driveSubsystem));

    // Amp Scoring TODO: Update setpoints
    auxController
        .y()
        .onTrue(
            new AmpScore(m_armSubsystem, m_wristSubsystem, m_feederSubsystem, m_shooterSubsystem)).onFalse(new InstantCommand(
                ()-> {m_feederSubsystem.setFeederPercentSpeed(0);
                m_shooterSubsystem.setBothShooterMotorsVoltage(0);}
            ));

    /** Non PID controls for the mechanisms */
    // NOTE: In sim the angle that the arm stops at changes and isnt near the min/max angles we set
    m_armSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_armSubsystem.setArmPercentSpeed(auxController.getLeftY()), m_armSubsystem));

    m_wristSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_wristSubsystem.setWristPercentSpeed(auxController.getRightY()),
            m_wristSubsystem));

    // m_utbIntakeSubsystem.setDefaultCommand(
    //   new InstantCommand(
    //     ()-> m_utbIntakeSubsystem.enableUTB(driverController.leftBumper().getAsBoolean()),
    //     m_utbIntakeSubsystem
    //   )
    // );

    // m_otbIntakeSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () ->
    // m_otbIntakeSubsystem.enableRollers(driverController.rightBumper().getAsBoolean()),
    //         m_otbIntakeSubsystem));

    // m_actuatorSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_actuatorSubsystem.setActuatorPercentSpeed(auxController.getLeftY() * 0.5),
    //         m_actuatorSubsystem));
    auxController
        .leftTrigger()
        .onTrue(
            new RunCommand(
                () -> m_shooterSubsystem.setShooterMotorPercentSpeed(0.5), m_shooterSubsystem))
        .onFalse(
            new RunCommand(
                () -> m_shooterSubsystem.setShooterMotorPercentSpeed(0.0), m_shooterSubsystem));

    auxController
        .rightTrigger()
        .onTrue(
            new RunCommand(() -> m_feederSubsystem.setFeederPercentSpeed(0.5), m_feederSubsystem))
        .onFalse(
            new RunCommand(() -> m_feederSubsystem.setFeederPercentSpeed(0.0), m_feederSubsystem));

    // m_armSubsystem.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_armSubsystem.setArmPercentSpeed(auxController.getLeftY() * (-1)),
    //         m_armSubsystem));

    m_wristSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_wristSubsystem.setWristPercentSpeed(auxController.getRightY() * (-1)),
            m_wristSubsystem));
    /** PID controls for the mechanisms */
    /** UTB Intake */
    driverController
        .rightTrigger()
        .whileTrue(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(-1), m_utbIntakeSubsystem))
        .whileFalse(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));
    driverController
        .rightBumper()
        .whileTrue(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(1), m_utbIntakeSubsystem))
        .whileFalse(
            new InstantCommand(
                () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem));

    driverController
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(-1), m_utbIntakeSubsystem),
                new InstantCommand(() -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(-0.75))))
        .whileFalse(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem),
                new InstantCommand(() -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(0))));
    driverController
        .leftBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(1), m_utbIntakeSubsystem),
                new InstantCommand(() -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(0.75))))
        .whileFalse(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> m_utbIntakeSubsystem.setUTBIntakePercentSpeed(0), m_utbIntakeSubsystem),
                new InstantCommand(() -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(0))));

    // Actuator
    driverController
        .b()
        .onTrue(
            new InstantCommand(
                () ->
                    m_actuatorSubsystem.setActuatorSetpoint(
                        ActuatorConstants.MAX_ANGLE_RADS), // Extended position
                m_actuatorSubsystem))
        .onFalse(
            new InstantCommand(
                () ->
                    m_actuatorSubsystem.setActuatorSetpoint(
                        ActuatorConstants.MIN_ANGLE_RADS), // Retracted position
                m_actuatorSubsystem));
    // m_actuatorSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_actuatorSubsystem.enableActuator(driverController.x().getAsBoolean()),
    //         m_actuatorSubsystem));

    // m_shooterSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_shooterSubsystem.enableShooter(auxController.a().getAsBoolean()),
    //         m_shooterSubsystem));
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

  /** This Turns the Mechanisms to either Coast or Brake Depending on Disable or Enable */
  public void mechanismsCoastOnDisable(boolean isDisabled) {
    m_driveSubsystem.coastOnDisable(isDisabled);
    m_armSubsystem.setBrakeMode(!isDisabled);
    m_wristSubsystem.setWristBrakeMode(!isDisabled);
    m_actuatorSubsystem.setBrakeMode(!isDisabled);
  }
}
