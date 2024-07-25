// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutonomousCommands.PathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Commands.TeleopCommands.SpeakerScore.Shoot;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.gyro.Gyro;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new SpeakerShoot. */
  public AutoShoot(
      Feeder feeder,
      Shooter shooter,
      Wrist wrist,
      Arm arm,
      Gyro gyro,
      double angleWrist,
      double angleArm,
      double RPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Commands.runOnce(
        //     () -> {
        //       feeder.setSetpoint(-250);
        //     },
        //     feeder),
        Commands.runOnce(
            () -> {
              wrist.setGoal(angleWrist);
              arm.setGoal(angleArm);
              shooter.setSetpoint(RPM);
            },
            wrist,
            arm,
            shooter),
        new WaitUntilCommand(() -> shooter.bothAtSetpoint()),
        new Shoot(feeder, arm, shooter));
  }
}
