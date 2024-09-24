// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.AmpScore;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.wrist.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionAmpScoreBackside extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public PositionAmpScoreBackside(Arm arm, Wrist wrist, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.AMP_BACK_SIDE_RAD);
              feeder.setSetpoint(200);
            },
            arm,
            feeder),
        // new WaitUntilCommand(() -> arm.atGoal()),
        new WaitCommand(0.5),
        new InstantCommand(() -> feeder.setSetpoint(0), feeder),
        new WaitCommand(0.5),
        Commands.runOnce(
            () -> {
              wrist.setGoal(WristConstants.AMP_BACK_SIDE_RAD);
            },
            wrist),
        new WaitUntilCommand(() -> wrist.atGoal()));
  }
}
