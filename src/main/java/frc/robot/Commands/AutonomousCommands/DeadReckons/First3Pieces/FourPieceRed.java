// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
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

public class FourPieceRed extends SequentialCommandGroup {
  /** Creates a new FourPieceBlue. */
  public FourPieceRed(
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
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
        Commands.runOnce(
            () -> {
              gyro.zeroYaw();
            },
            gyro),
        new ThreePieceAutoRed(
            drive, gyro, wrist, arm, feeder, shooter, actuator, otbIntake, utbIntake, seconds,
            speed),

        // go back to
        Commands.runOnce(
            () -> {
              drive.setRaw(0, -1.0, 0);
            },
            drive),
        new WaitCommand(0.5),

        // go to note and rotate to be staight
        Commands.runOnce(
            () -> {
              drive.setRaw(1.0, 0.0, 0.3);
            },
            drive),
        new WaitCommand(0.1),
        Commands.runOnce(
            () -> {
              drive.setRaw(1.0, 0.0, 0.0);
            },
            drive),
        new WaitCommand(1.9),

        // grab note
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, 1.0, 0.0);
                },
                drive),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, false)),
        new WaitCommand(0.5),

        // rotate to speaker
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0.0, 0.0, 0.3);
                },
                drive),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, true)),
        new WaitCommand(0.1),

        // stop and shoot
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0.0, 0.0, 0.0);
                },
                drive),
            new PositionToShoot(feeder, shooter, wrist, arm, 0.5, 0, ShooterConstants.CLOSE_RPM)));
  }
}
