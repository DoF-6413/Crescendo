// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.VisionAutomations;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.TeleopCommands.Intakes.UTBIntakeRun;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.utbintake.UTBIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpNote extends SequentialCommandGroup {
  public Drive drive;
  public OTBIntake otb;
  public UTBIntake utb;
  public Feeder feeder;
  public Actuator actuator;

  /** Creates a new PickUpNote. */
  public PickUpNote(Drive drive, OTBIntake otb, UTBIntake utb, Feeder feeder, Actuator actuator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AlignToNote(drive),
        // new AllIntakesRun(actuator, otb, utb, feeder, false),
        new UTBIntakeRun(utb, feeder, true, false),
        new ParallelRaceGroup(
            Commands.run(() -> drive.setRaw(0, 0.8, 0), drive), new WaitCommand(2)));
  }
}
