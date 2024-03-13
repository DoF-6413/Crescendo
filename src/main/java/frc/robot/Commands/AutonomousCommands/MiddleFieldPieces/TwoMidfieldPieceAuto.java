// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.MiddleFieldPieces;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AutonomousCommands.First3Pieces.OnePieceAuto;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Commands.TeleopCommands.SpeakerScore.PositionToShoot;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoMidfieldPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoMidfieldPieceAuto(
      Drive drive,
      Gyro gyro,
      Wrist wrist,
      Feeder feeder,
      Shooter shooter,
      Actuator actuator,
      OTBIntake otbIntake,
      UTBIntake utbIntake,
      double seconds,
      double speedX,
      double speedY) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              gyro.zeroYaw();
            },
            gyro),
        new OnePieceAuto(wrist, feeder, shooter),
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, speedY, 0);
                },
                drive),
            Commands.runOnce(
                () -> {
                  drive.setRaw(speedX, 0, 0);
                }),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, false)),
        new WaitCommand(seconds),
        Commands.runOnce(
            () -> {
              drive.setRaw(0.0, 0, 0);
            },
            drive),
        new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, true),
        Commands.runOnce(
            () -> {
              drive.setRaw(-speedX, 0, 0);
            },
            drive),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, -speedY, 0);
            }),
        new PositionToShoot(feeder, shooter, wrist, WristConstants.PODIUM_RAD));
  }
}
