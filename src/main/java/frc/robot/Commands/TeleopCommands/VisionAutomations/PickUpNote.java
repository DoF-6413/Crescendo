// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.VisionAutomations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;
import frc.robot.Utils.BeamBreak;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNote extends SequentialCommandGroup {

  /** Creates a new PickUpNote. */
  public PickUpNote(
      Drive drive,
      OTBIntake otb,
      UTBIntake utb,
      Feeder feeder,
      Actuator actuator,
      Shooter shooter,
      Arm arm,
      Wrist wrist,
      BeamBreak beamBreak) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new AllIntakesRun(actuator, otb, utb, feeder, false),
            // new BeamBreakPickUp(utb, feeder, shooter, beamBreak),
            new InstantCommand(
                () -> {
                  arm.setGoal(0);
                  wrist.setGoal(WristConstants.DEFAULT_POSITION_RAD);
                },
                wrist,
                arm)),
        // new UTBIntakeRun(utb, feeder, true, false),
        new DriveToNote(drive, beamBreak, 0, 0.5, 0.3)
        // .until(() -> !beamBreak.getShooterSensor()),
        // new InstantCommand(() -> drive.setRaw(0, 0, 0), drive)
        );
  }
}
