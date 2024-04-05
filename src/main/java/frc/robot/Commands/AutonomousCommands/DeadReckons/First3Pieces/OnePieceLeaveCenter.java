// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceLeaveCenter extends SequentialCommandGroup {
  /** Creates a new OnePieceLeaveAuto. */
  public OnePieceLeaveCenter(
      Drive drive,
      Wrist wrist,
      Feeder feeder,
      Shooter shooter,
      double seconds,
      double speed,
      Gyro gyro) {
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
              drive.setRaw(0, speed, 0);
            },
            drive),
        new WaitCommand(seconds),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, 0, 0);
            },
            drive));
  }
}
