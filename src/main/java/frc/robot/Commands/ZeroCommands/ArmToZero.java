// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToZero extends SequentialCommandGroup {
  /** Creates a new ArmToZero. */
  public ArmToZero(Arm arm, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
              wrist.setSpeedScalar(WristConstants.REDUCED_SPEED_SCALAR);
              wrist.setGoal(WristConstants.DEFAULT_POSITION_RAD);
            },
            arm,
            wrist),
        new WaitUntilCommand(() -> wrist.getPositionDeg() < 12),
        Commands.runOnce(() -> wrist.setSpeedScalar(WristConstants.DEFAULT_SPEED_SCALAR), wrist));
  }
}
