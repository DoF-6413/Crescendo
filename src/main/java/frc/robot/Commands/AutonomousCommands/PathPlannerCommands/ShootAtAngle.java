// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.BeamBreak;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAtAngle extends SequentialCommandGroup {

  /** Creates a new ShootNote. */
  public ShootAtAngle(
      Arm arm,
      Wrist wrist,
      Shooter shooter,
      Feeder feeder,
      BeamBreak beamBreak,
      double armAngle,
      double wristAngle,
      double shooterSpeed) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  arm.setGoal(armAngle);
                  wrist.setGoal(wristAngle);
                },
                arm,
                wrist),
            new ReverseNote(shooter, feeder, beamBreak)),
        new InstantCommand(() -> shooter.setSetpoint(shooterSpeed), shooter),
        new WaitUntilCommand(() -> shooter.bothAtSetpoint() && wrist.atGoal()),
        new InstantCommand(() -> feeder.setSetpoint(FeederConstants.SPEAKER_RPM), feeder));
  }
}
