// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.TeleopCommands.Intakes.UTBIntakeRun;
import frc.robot.Constants.CommandConstants;
import frc.robot.Subsystems.actuator.Actuator;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.arm.ArmConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.otbIntake.OTBIntake;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Subsystems.wrist.WristConstants;
import frc.robot.Utils.BeamBreak;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, sVisionPickUpee:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionPickUp extends ParallelCommandGroup {

  /** Creates a new VisionPickUp. */
  public VisionPickUp(
      Drive drive,
      OTBIntake otb,
      UTBIntake utb,
      Feeder feeder,
      Actuator actuator,
      Arm arm,
      Wrist wrist,
      BeamBreak beamBreak) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new AllIntakesRun(actuator, otb, utb, feeder, CommandConstants.RUN_INTAKE),
        // new BeamBreakPickUp(utb, feeder, shooter, beamBreak),
        new UTBIntakeRun(utb, feeder, CommandConstants.INTAKE_INWARDS, CommandConstants.RUN_INTAKE),
        new InstantCommand(
            () -> {
              arm.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
              wrist.setGoal(WristConstants.DEFAULT_POSITION_RAD);
            },
            wrist,
            arm),
        new DriveToNote(drive, beamBreak, 0, 0.6, 0.3));
  }
}
