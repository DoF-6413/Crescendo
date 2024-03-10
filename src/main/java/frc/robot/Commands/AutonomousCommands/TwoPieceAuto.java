// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesStop;
import frc.robot.Commands.TeleopCommands.SpeakerScore.ShootAtSpeaker;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Drive drive, Wrist wrist, Feeder feeder, Shooter shooter, double seconds, double speed, Actuator actuator, OTBIntake otbIntake, UTBIntake utbIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OnePieceAuto(wrist, feeder, shooter),
      new ParallelCommandGroup(
      Commands.runOnce(
            () -> { drive.setRaw(speed, 0, 0);
            }, drive),
      new AllIntakesRun(actuator, otbIntake, utbIntake, true)),
      new WaitCommand(seconds),
      Commands.runOnce(
            () -> { drive.setRaw(0.0, 0, 0);
            }, drive),
      new AllIntakesStop(actuator, otbIntake, utbIntake),
      new ShootAtSpeaker(feeder, shooter, wrist, 1.5)
    );
  }
}
