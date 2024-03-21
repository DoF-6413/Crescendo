// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.First3Pieces;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.TeleopCommands.Intakes.UTBIntakeRun;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.Shoot;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceAuto extends SequentialCommandGroup {
  /** Creates a new ThreePieceAuto. */
  public ThreePieceAuto(Drive drive,
      Gyro gyro,
      Wrist wrist,
      Arm arm,
      Feeder feeder,
      Shooter shooter,
      Actuator actuator,
      OTBIntake otbIntake,
      UTBIntake utbIntake,
      double seconds,
      double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(7), 
      new TwoPieceReturnSub(drive, gyro, wrist, arm, feeder, shooter, actuator, otbIntake, utbIntake, seconds, speed)),
      Commands.runOnce(
            () -> {
              wrist.setSetpoint(0);
            },
            wrist),
        new InstantCommand(() -> shooter.setSetpoint(0)),
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, -speed, 0);
                },
                drive),
            new UTBIntakeRun(utbIntake, feeder, true, false)),
        new WaitCommand(seconds - 1),
                Commands.runOnce(
                () -> {
                  drive.setRaw(speed, 0, 0);
                  System.out.println("running");
                },
                drive),
        new WaitCommand(seconds),
        Commands.runOnce(
                () -> {
                  drive.setRaw(0, 0, 0.1);
                  System.out.println("running");
                },
                drive),
                new WaitCommand(0.1),
                new PositionToShoot(feeder, shooter, wrist, 27, 4000),
                new WaitCommand(seconds),
                new UTBIntakeRun(utbIntake, feeder, false, true),
                Commands.runOnce(
                    () -> {
                      drive.setRaw(0, 0, 0);
                    },
                    drive),
                new WaitUntilCommand(() -> shooter.allAtSetpoint()),
                new Shoot(feeder, arm, shooter),
                Commands.runOnce(() -> shooter.setTolerance(ShooterConstants.RPM_TOLERANCE), shooter)
    );
  }
}
