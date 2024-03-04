// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public AmpScore(Arm arm, Wrist wrist, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(arm, wrist, feeder, shooter);
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setSetpoint(1);
              wrist.setSetpoint(1);
            },
            arm,
            wrist),
        new WaitUntilCommand(() -> arm.atSetpoint()),
        new WaitUntilCommand(() -> wrist.atSetpoint()),
        Commands.runOnce(
            () -> {
              shooter.setShooterMotorPercentSpeed(1);
              feeder.setFeederPercentSpeed(1);
            },
            shooter,
            feeder));
  }
}
