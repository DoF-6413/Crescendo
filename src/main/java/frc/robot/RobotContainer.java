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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterIO;
import frc.robot.Subsystems.shooter.ShooterIOSim;
import frc.robot.Subsystems.shooter.ShooterIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Gyro m_gyroSubsystem;
  // private final Drive m_driveSubsystem;
  private final Shooter m_shooterSubsystem;
  // private final PoseEstimator m_poseEstimator;
  // private final Vision m_vision;
  // private final UTBIntake m_utbIntake;

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // m_gyroSubsystem = new Gyro(new GyroIONavX());
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIOSparkMax(),
        //         new ModuleIOSparkMax(),
        //         new ModuleIOSparkMax(),
        //         new ModuleIOSparkMax(),
        //         m_gyroSubsystem);
        // m_vision = new Vision(new VisionIOArduCam());
        m_shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        // m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_vision);
        // m_utbIntake = new UTBIntake(new UTBIntakeIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // m_gyroSubsystem = new Gyro(new GyroIO() {});
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIOSimNeo(),
        //         new ModuleIOSimNeo(),
        //         new ModuleIOSimNeo(),
        //         new ModuleIOSimNeo(),
        //         m_gyroSubsystem);
        // m_vision = new Vision(new VisionIOSim());
        // m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_vision);
        m_shooterSubsystem = new Shooter(new ShooterIOSim());
        // m_utbIntake = new UTBIntake(new UTBIntakeIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        // m_gyroSubsystem = new Gyro(new GyroIO() {});
        // m_driveSubsystem =
        //     new Drive(
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         m_gyroSubsystem);
        m_shooterSubsystem = new Shooter(new ShooterIO() {});
        // m_vision = new Vision(new VisionIO() {});
        // m_poseEstimator = new PoseEstimator(m_driveSubsystem, m_gyroSubsystem, m_vision);
        // m_utbIntake = new UTBIntake(new UTBIntakeIO() {});
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
    // m_driveSubsystem.setDefaultCommand(
    //     new RunCommand(
    //         () ->
    //             m_driveSubsystem.setRaw(
    //                 driverController.getLeftX(),
    //                 -driverController.getLeftY(),
    //                 driverController.getRightX()),
    //         m_driveSubsystem));

    // driverController.a().onTrue(new InstantCommand(() -> m_driveSubsystem.updateHeading()));

    /*
     * Spins the Shooter motors at a certain percent based off the y-axis value of right Xbox Joystick
     * Up will launch a NOTE outward
     * Down will retract a NOTE inward
     */
    m_shooterSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_shooterSubsystem.setShooterMotorPercentSpeed(-driverController.getRightY()),
            m_shooterSubsystem));

    /*
     * Spins the motor that will be running the UTB Intake
     */
    // m_utbIntake.setDefaultCommand(
    //     new InstantCommand(
    //         () -> m_utbIntake.setUTBIntakePercentSpeed(driverController.getLeftY()),
    //         m_utbIntake)); // TODO: Update controls
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
