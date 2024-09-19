// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public Shoot(Arm arm, Shooter shooter, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
            new ConditionalCommand(
                Commands.runOnce(
                    () -> {
                      feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
                    },
                    feeder),
                new ConditionalCommand(
                    Commands.runOnce(
                        () -> {
                          feeder.setSetpoint(FeederConstants.AMP_RPM);
                          shooter.setSetpoint(ShooterConstants.AMP_RPM);
                        },
                        feeder,
                        shooter),
                    new ConditionalCommand(
                        Commands.runOnce(
                            () -> {
                              feeder.setSetpoint(-FeederConstants.AMP_RPM);
                            },
                            feeder),
                        Commands.runOnce(
                            () -> {
                              feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
                            },
                            feeder),
                        () -> arm.getGoal() == ArmConstants.AMP_BACK_SIDE_RAD),
                    () -> arm.getGoal() == ArmConstants.AMP_FRONT_SIDE_RAD),
                () -> arm.getGoal() == ArmConstants.DEFAULT_POSITION_RAD),
            new InstantCommand(),
            () -> shooter.bothAtSetpoint()));
  }
}
