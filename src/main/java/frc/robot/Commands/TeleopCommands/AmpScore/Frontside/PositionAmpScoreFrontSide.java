// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.AmpScore.Frontside;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

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
              arm.setSetpoint(Units.degreesToRadians(0.0)); //TODO:Update value
            },
            arm),
        new WaitUntilCommand(() -> arm.atSetpoint()),
        Commands.runOnce(
            () -> {
              wrist.setSetpoint(Units.degreesToRadians(0.0)); //TODO:Update value
            },
            wrist),
        new WaitUntilCommand(() -> wrist.atSetpoint()));
  }
}
