// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.utbintake.UTBIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUp extends SequentialCommandGroup {
  private double utbIntakePercentSpeed;
  /** Creates a new pickUp. */
  public PickUp(UTBIntake utb, boolean stop) {

    if (stop) {
      utbIntakePercentSpeed = 0;
    } else {
      utbIntakePercentSpeed = -1.0;
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> utb.setUTBIntakePercentSpeed(utbIntakePercentSpeed))));
  }
}
