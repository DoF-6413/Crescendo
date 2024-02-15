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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.actuator.*;
import frc.robot.Subsystems.drive.*;
import frc.robot.Subsystems.gyro.*;
import frc.robot.Subsystems.otbIntake.*;
import frc.robot.Subsystems.pose.*;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.utbintake.*;
import frc.robot.Subsystems.vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Gyro m_gyroSubsystem;
  private final Drive m_driveSubsystem;
  private final Shooter m_shooterSubsystem;
  private final PoseEstimator m_poseEstimator;
  private final Vision m_visionSubsystem;
  private final UTBIntake m_utbIntakeSubsystem;
  private final OTBIntake m_otbIntakeSubsystem;
  private final Actuator m_actuatorSubsystem;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);
  private final CommandXboxController auxController =
      new CommandXboxController(OperatorConstants.AUX_CONTROLLER);

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
        m_visionSubsystem = new Vision(new VisionIOArduCam());
        m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSparkMax());
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSparkMax());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSparkMax());
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
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_shooterSubsystem = new Shooter(new ShooterIOSim());
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIOSim() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIOSim());
        m_actuatorSubsystem = new Actuator(new ActuatorIOSim());

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
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        m_visionSubsystem = new Vision(new VisionIO() {});
        m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_visionSubsystem);
        m_utbIntakeSubsystem = new UTBIntake(new UTBIntakeIO() {});
        m_otbIntakeSubsystem = new OTBIntake(new OTBIntakeIO() {});
        m_actuatorSubsystem = new Actuator(new ActuatorIO() {});
        break;
    }

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

    /*
     * Spins the Shooter motors at a certain percent based off the y-axis value of right Xbox Joystick
     * Up will launch a NOTE outward
     * Down will retract a NOTE inward
     */
    // m_shooterSubsystem.setDefaultCommand(
    //     new InstantCommand(
    //         () ->
    //             m_shooterSubsystem.setShooterMotorPercentSpeed(
    //                 -driverController.getRightY() * 0.75),
    //         m_shooterSubsystem));

    /*
     * Spins the motor that will be running the UTB Intake
     */
    // m_utbIntake.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_utbIntake.setUTBIntakePercentSpeed(auxController.getLeftY()), m_utbIntake));

    m_otbIntakeSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_otbIntakeSubsystem.setOTBIntakePercentSpeed(auxController.getRightY()),
            m_otbIntakeSubsystem));

    m_actuatorSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_actuatorSubsystem.setActuatorPercentSpeed(auxController.getLeftY()),
            m_actuatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
