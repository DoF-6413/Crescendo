// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.AmpScore.Frontside;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.*;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionAmpScoreFrontSide extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public PositionAmpScoreFrontSide(Arm arm, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.AMP_FRONT_SIDE_RAD);
            },
            arm),
        new WaitUntilCommand(() -> arm.atGoal()),
        Commands.runOnce(
            () -> {
              wrist.setGoal(WristConstants.AMP_FRONT_SIDE_RAD);
            },
            wrist),
        new WaitUntilCommand(() -> wrist.atGoal()));
  }
}
