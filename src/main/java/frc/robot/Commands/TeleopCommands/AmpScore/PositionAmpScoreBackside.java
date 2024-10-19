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
public class PositionAmpScoreBackside extends ParallelCommandGroup {
  /** Creates a new AmpScore. */
  public PositionAmpScoreBackside(Arm arm, Wrist wrist, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                  arm.setGoal(ArmConstants.AMP_BACK_SIDE_RAD);
                  wrist.setSpeedScalar(WristConstants.DEFAULT_SPEED_SCALAR);
                },
                arm,
                wrist),
            new WaitUntilCommand(() -> arm.getPositionDeg() > 6),
            Commands.runOnce(
                () -> {
                  wrist.setGoal(WristConstants.AMP_BACK_SIDE_RAD);
                },
                wrist),
            new WaitUntilCommand(() -> wrist.atGoal())),
        new SequentialCommandGroup(
            Commands.runOnce(() -> feeder.setSetpoint(200), feeder),
            new WaitCommand(1),
            Commands.runOnce(() -> feeder.setSetpoint(0), feeder)));
  }
}
