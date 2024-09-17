// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.DeadReckons.First3Pieces;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class OnePieceAuto extends SequentialCommandGroup {
  /** Creates a new OnePieceAuto. */
  public OnePieceAuto(Arm arm, Wrist wrist, Shooter shooter, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitCommand(SmartDashboard.getNumber("delay", 0)),
        Commands.runOnce(
            () -> {
              arm.setGoal(ArmConstants.SUBWOOFER_RAD);
              wrist.setGoal(WristConstants.SUBWOOFER_RAD);
              shooter.setSetpoint(ShooterConstants.CLOSE_RPM);
            },
            arm,
            shooter,
            wrist),
        // new WaitUntilCommand(() -> wrist.atGoal()),
        // new WaitUntilCommand(() -> shooter.bothAtSetpoint()),
        new WaitCommand(1.5),
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
            },
            feeder));
  }
}
