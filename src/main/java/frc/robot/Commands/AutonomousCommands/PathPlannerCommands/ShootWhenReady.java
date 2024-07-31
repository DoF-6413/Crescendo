// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Utils.BeamBreak;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWhenReady extends SequentialCommandGroup {

  /** Creates a new ShootWhenReady. */
  public ShootWhenReady(Shooter shooter, Feeder feeder, BeamBreak beamBreak, double shooterRPM) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> shooter.setSetpoint(shooterRPM), shooter),
      new WaitUntilCommand(()-> shooter.bothAtSetpoint()),
      new InstantCommand(()-> feeder.setSetpoint(FeederConstants.SPEAKER_RPM), feeder)
    );
  }
}
