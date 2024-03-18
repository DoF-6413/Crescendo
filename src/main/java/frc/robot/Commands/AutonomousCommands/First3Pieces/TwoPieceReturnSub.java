// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.First3Pieces;

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
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceReturnSub extends SequentialCommandGroup {
  /** Creates a new TwoPieceReturnSub. */
  public TwoPieceReturnSub(
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
        Commands.runOnce(
            () -> {
              gyro.zeroYaw();
            },
            gyro),
        new OnePieceAuto(wrist, feeder, shooter),
        Commands.runOnce(
            () -> {
              wrist.setSetpoint(0);
            },
            wrist),
        new InstantCommand(() -> shooter.setSetpoint(0)),
        new ParallelCommandGroup(
            Commands.runOnce(
                () -> {
                  drive.setRaw(0, speed, 0);
                },
                drive),
            new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, false)),
        new WaitCommand(seconds),
        new AllIntakesRun(actuator, otbIntake, utbIntake, feeder, true),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, -speed, 0);
            },
            drive),
        new WaitCommand(seconds),
        new PositionToShoot(feeder, shooter, wrist, 27),
        new WaitCommand(0.4),
        new Shoot(feeder, arm, shooter));
  }
}
