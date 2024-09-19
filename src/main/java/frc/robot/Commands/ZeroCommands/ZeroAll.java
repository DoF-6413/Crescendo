// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ZeroCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroAll extends ParallelCommandGroup {
  /** Creates a new ZeroAll. */
  public ZeroAll(Arm arm, Wrist wrist, Shooter shooter, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmToZero(arm, wrist), new EndEffectorToZero(shooter, feeder));
  }
}
