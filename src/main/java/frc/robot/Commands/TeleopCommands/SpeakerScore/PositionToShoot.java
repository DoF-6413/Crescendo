// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.wrist.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionToShoot extends SequentialCommandGroup {
  /** Shoots NOTE when robot is against the subwoofer (right in front of the SPEAKER) */
  public PositionToShoot(Feeder feeder, Shooter shooter, Wrist wrist, double angle, double RPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(-500);
              System.out.println("Running 1");
            },
            feeder),
        new WaitCommand(0.3),
        Commands.runOnce(
            () -> {
              // feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
              wrist.setGoal(angle); // TODO: update when shooter interpolation branch is merged to

              System.out.println("Running 2");
              // reference
              // lookup table
              shooter.setSetpoint(RPM);
              feeder.setSetpoint(0);
            },
            feeder,
            wrist,
            shooter));
  }
}
