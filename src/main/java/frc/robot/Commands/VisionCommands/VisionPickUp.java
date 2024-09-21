// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.VisionCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.TeleopCommands.Intakes.AllIntakesRun;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.FieldConstants;
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
import frc.robot.Utils.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, VisionPickUp:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionPickUp extends ParallelCommandGroup {

  /** Creates a new VisionPickUp. */
  public VisionPickUp(
      Drive drive,
      Actuator actuator,
      OTBIntake otb,
      UTBIntake utb,
      Arm arm,
      Wrist wrist,
      Feeder feeder,
      PoseEstimator pose,
      BeamBreak beamBreak) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AllIntakesRun(actuator, otb, utb, feeder, CommandConstants.RUN_INTAKE),
        new InstantCommand(
            () -> {
              arm.setGoal(ArmConstants.DEFAULT_POSITION_RAD);
              wrist.setGoal(WristConstants.DEFAULT_POSITION_RAD);
            },
            wrist,
            arm),
        new DriveToNote(drive, beamBreak, 0, 0.6, 0.3)
            .until(
                () ->
                    (pose.getCurrentPose2d().getX() > 4.5
                        && pose.getCurrentPose2d().getX() < FieldConstants.RED_SPEAKER_X - 4.5)));
  }
}
