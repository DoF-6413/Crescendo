// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

public class ShootingReady extends SequentialCommandGroup {
  /** Creates a new ShootingReady. */
  public ShootingReady(Shooter shooter, Wrist wrist, Arm arm) {
    addCommands(
        Commands.runOnce(
            () -> {
              wrist.atGoal();
              arm.atGoal();
              shooter.bothAtSetpoint();
            },
            wrist,
            arm,
            shooter));
  }
}
