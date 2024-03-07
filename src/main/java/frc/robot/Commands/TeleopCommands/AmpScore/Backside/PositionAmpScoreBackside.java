// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.AmpScore.Backside;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionAmpScoreBackside extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public PositionAmpScoreBackside(Arm arm, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setSetpoint(Units.degreesToRadians(ArmConstants.AMP_BACKSIDE_DEG));
            },
            arm),
        new WaitUntilCommand(() -> arm.atSetpoint()),
        Commands.runOnce(
            () -> {
              wrist.setSetpoint(Units.degreesToRadians(WristConstants.AMP_BACKSIDE_DEG));
            },
            wrist),
        new WaitUntilCommand(() -> wrist.atSetpoint()));
  }
}
