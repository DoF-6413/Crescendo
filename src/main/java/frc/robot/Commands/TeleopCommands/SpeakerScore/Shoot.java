// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(CommandXboxController xbox, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            Commands.runOnce(
                () -> {
                  feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
                },
                feeder),
            new InstantCommand(),
            () -> xbox.leftTrigger().getAsBoolean() || xbox.rightTrigger().getAsBoolean()),
        new ConditionalCommand(
            Commands.runOnce(
                () -> {
                  feeder.setSetpoint(FeederConstants.AMP_RPM);
                },
                feeder),
            new InstantCommand(),
            () -> xbox.leftBumper().getAsBoolean()),
        new ConditionalCommand(
            Commands.runOnce(
                () -> {
                  feeder.setSetpoint(-FeederConstants.AMP_RPM);
                },
                feeder),
            new InstantCommand(),
            () -> xbox.rightBumper().getAsBoolean()));
  }
}
