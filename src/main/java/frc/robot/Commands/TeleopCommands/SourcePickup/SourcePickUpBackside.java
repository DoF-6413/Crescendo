// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SourcePickup;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourcePickUpBackside extends SequentialCommandGroup {
  /** Intakes NOTE from SOURCE, backside */
  public SourcePickUpBackside(Arm arm, Wrist wrist, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setSetpoint(ArmConstants.SOURCE_BACK_SIDE_RAD);
              wrist.setSetpoint(WristConstants.SOURCE_BACK_SIDE_RAD);
            },
            arm,
            wrist),
        new WaitUntilCommand(() -> arm.atSetpoint()),
        new WaitUntilCommand(() -> wrist.atSetpoint()),
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(FeederConstants.SOURCE_RPM);
            },
            feeder));
  }
}
