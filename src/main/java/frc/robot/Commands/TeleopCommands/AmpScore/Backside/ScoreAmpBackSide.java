// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.AmpScore.Backside;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Commands.ZeroCommands.ArmToZero;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpBackside extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreAmpBackside(Arm arm, Wrist wrist, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(-1500);
            },
            feeder),
        new WaitCommand(1),
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(0);
            },
            feeder),
    new ArmToZero(wrist, arm));
  }
}
