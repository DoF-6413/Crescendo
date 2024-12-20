// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.feeder.FeederConstants;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OverShot extends SequentialCommandGroup {
  /** Creates a new OverShot. */
  public OverShot(Arm arm, Wrist wrist, Feeder feeder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.OVERSHOT_RAD);
            },
            arm),
        new WaitUntilCommand(() -> arm.atGoal()),
        Commands.runOnce(
            () -> {
              // feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
              wrist.setGoal(WristConstants.OVERSHOT_RAD);
            },
            feeder,
            wrist),
        new WaitUntilCommand(() -> wrist.atGoal()),
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(FeederConstants.REVERSE_RPM);
            },
            feeder),
        new WaitCommand(0.3),
        Commands.runOnce(
            () -> {
              shooter.setSetpoint(ShooterConstants.CLOSE_RPM);
              feeder.setSetpoint(0);
            },
            shooter,
            feeder));
  }
}
