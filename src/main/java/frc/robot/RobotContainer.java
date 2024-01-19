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
import frc.robot.Subsystems.shooter.ShooterIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Shooter shooterSubsystem;

  // Controllers
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotStateConstants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        shooterSubsystem = new Shooter(new ShooterIOTalonFX());
        break;

      default:
        // Replayed robot, disable IO implementations
        shooterSubsystem = new Shooter(new ShooterIOTalonFX());
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
    shooterSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> shooterSubsystem.setShooterMotorPercentSpeed(controller.getRightY() * 0.5), shooterSubsystem));

    controller
        .leftBumper()
        .whileTrue(new InstantCommand(() -> shooterSubsystem.setShooterMotorPercentSpeed(-0.5), shooterSubsystem))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setShooterMotorPercentSpeed(0), shooterSubsystem));
    controller
        .rightBumper()
        .whileTrue(new InstantCommand(() -> shooterSubsystem.setShooterMotorPercentSpeed(0.5), shooterSubsystem))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setShooterMotorPercentSpeed(0), shooterSubsystem));
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
