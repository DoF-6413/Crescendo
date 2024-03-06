// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SourcePickup;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourcePickUpFrontside extends SequentialCommandGroup {
  /** Creates a new ArmPickUp. */
  public SourcePickUpFrontside(Arm arm, Wrist wrist, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(
        () -> {
          arm.setSetpoint(Units.degreesToRadians(10.0));
          wrist.setSetpoint(Units.degreesToRadians(189));
        },
        arm,
        wrist),
    new WaitUntilCommand(() -> arm.atSetpoint()),
    new WaitUntilCommand(() -> wrist.atSetpoint()),

    Commands.runOnce(
        () -> {
          feeder.setSetpoint(1500);
        },
        feeder)
    );
  }
}
