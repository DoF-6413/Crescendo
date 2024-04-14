// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Commands.TeleopCommands.SpeakerScore.Shoot;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceAutoRed extends SequentialCommandGroup {
  /** Creates a new ThreePieceAuto. */
  public ThreePieceAutoRed(
      Drive drive,
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
        // One Piece ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Commands.runOnce(
            () -> {
              gyro.zeroYaw();
            },
            gyro),
        new OnePieceAuto(wrist, feeder, shooter),
        Commands.runOnce(
            () -> {
              wrist.setGoal(0);
            },
            wrist),
        new InstantCommand(() -> shooter.setSetpoint(0)),
        // Two Piece
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, speed, 0);
                },
                drive),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, false)),
        new WaitCommand(seconds / 2),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, speed / 2, 0);
            },
            drive),
        new WaitCommand(seconds),
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, -speed, 0);
                  shooter.setTolerance(500);
                },
                drive),
            new PositionToShoot(
                feeder,
                shooter,
                wrist,
                arm,
                WristConstants.SUBWOOFER_RAD,
                ArmConstants.SUBWOOFER_RAD,
                ShooterConstants.CLOSE_RPM)),
        new WaitCommand(seconds),
        new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, true),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, 0, 0);
            },
            drive),
        new WaitCommand(1),
        new Shoot(feeder, arm, shooter),
        Commands.runOnce(() -> shooter.setTolerance(ShooterConstants.RPM_TOLERANCE), shooter),
        // 3 Piece
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Commands.runOnce(
            () -> {
              wrist.setGoal(0);
            },
            wrist),
        new InstantCommand(() -> shooter.setSetpoint(0)),
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(-1.0, 0.75, -0.4);
                },
                drive),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, false)),
        new WaitCommand(0.6),
        Commands.runOnce(
            () -> {
              drive.setRaw(-1.0, 0.75, 0.0);
            },
            drive),
        new WaitCommand(1.4),
        new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, true),
        new PositionToShoot(
            feeder, shooter, wrist, arm, WristConstants.PODIUM_RAD, 0, ShooterConstants.CLOSE_RPM),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, 0, 0);
            },
            drive),
        new WaitCommand(1),
        new Shoot(feeder, arm, shooter),
        Commands.runOnce(() -> shooter.setTolerance(ShooterConstants.RPM_TOLERANCE), shooter));
  }
}
