// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerScore;

import edu.wpi.first.math.util.Units;
<<<<<<<< HEAD:src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/OverShot.java
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.wrist.Wrist;
========
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.feeder.*;
import frc.robot.Subsystems.shooter.*;
import frc.robot.Subsystems.wrist.*;
>>>>>>>> a4658a5 (Chore#80 clean dev (#81)):src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/ShootAtSpeaker.java

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
<<<<<<<< HEAD:src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/OverShot.java
public class OverShot extends SequentialCommandGroup {
  /** Creates a new OverShot. */
  public OverShot(Arm arm, Feeder feeder, Shooter shooter, Wrist wrist) {
========
public class ShootAtSpeaker extends SequentialCommandGroup {
  /** Shoots NOTE when robot is against the subwoofer (right in front of the SPEAKER) */
  public ShootAtSpeaker(Feeder feeder, Shooter shooter, Wrist wrist) {
>>>>>>>> a4658a5 (Chore#80 clean dev (#81)):src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/ShootAtSpeaker.java
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(
            () -> {
<<<<<<<< HEAD:src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/OverShot.java
              arm.setGoal(Units.degreesToRadians(83));
            },
            arm),
        new WaitUntilCommand(() -> arm.atGoal()),
        Commands.runOnce(
            () -> {
              // feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
              wrist.setGoal(
                  Units.degreesToRadians(
                      38)); // TODO: update when shooter interpolation branch is merged to
              // reference
========
              feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
              wrist.setSetpoint(
                  Units.degreesToRadians(
                      21)); // TODO: update when shooter interpolation branch is merged to reference
>>>>>>>> a4658a5 (Chore#80 clean dev (#81)):src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/ShootAtSpeaker.java
              // lookup table
            },
            feeder,
            wrist),
        new WaitUntilCommand(() -> wrist.atSetpoint()),
        Commands.runOnce(
            () -> {
<<<<<<<< HEAD:src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/OverShot.java
              feeder.setSetpoint(-600);
========
              feeder.setSetpoint(-FeederConstants.SPEAKER_RPM);
>>>>>>>> a4658a5 (Chore#80 clean dev (#81)):src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/ShootAtSpeaker.java
            },
            feeder),
        new WaitCommand(0.3),
        Commands.runOnce(
            () -> {
<<<<<<<< HEAD:src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/OverShot.java
              shooter.setSetpoint(ShooterConstants.CLOSE_RPM);
              feeder.setSetpoint(0);
            },
            shooter,
========
              shooter.setSetpoint(ShooterConstants.SPEAKER_RPM);
              feeder.setSetpoint(0);
            },
            shooter,
            feeder),
        new WaitCommand(0.8),
        Commands.runOnce(
            () -> {
              feeder.setSetpoint(FeederConstants.SPEAKER_RPM);
            },
>>>>>>>> a4658a5 (Chore#80 clean dev (#81)):src/main/java/frc/robot/Commands/TeleopCommands/SpeakerScore/ShootAtSpeaker.java
            feeder));
  }
}
