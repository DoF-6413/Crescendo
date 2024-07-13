// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.Intakes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterRev extends SequentialCommandGroup {

  /** Creates a new ShooterRev. */
  public ShooterRev(
      Actuator actuator, OTBIntake otb, UTBIntake utb, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new UTBIntakeRun(utb, feeder, false, true),
        new ParallelDeadlineGroup(
            new InstantCommand(() -> shooter.getBeamBreak()),
            new InstantCommand(() -> feeder.setSetpoint(-250), feeder)),
        new InstantCommand(() -> shooter.setSetpoint(4000), shooter));
  }
}
