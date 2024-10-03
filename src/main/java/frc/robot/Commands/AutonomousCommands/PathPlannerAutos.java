package frc.robot.Commands.AutonomousCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PreloadShot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ReverseNote;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootWhenReady;
import frc.robot.Commands.TeleopCommands.SpeakerScore.AimWrist;
import frc.robot.Commands.ZeroCommands.ZeroAll;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.utbintake.UTBIntakeConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.BeamBreak;
import frc.robot.Utils.PoseEstimator;

public class PathPlannerAutos {

  /**
   * Same auto as the one from PathPlanner but with decision making
   *
   * @param cancelFirstNote If C1 is not detected then goes immediately to C2
   * @param pastMidline If the (whole chassis?) is past the Midline then move on to the next path
   */
  public static Command SubAmp_C1_C2_Smart(
      Drive drive,
      UTBIntake utb,
      Arm arm,
      Wrist wrist,
      Shooter shooter,
      Feeder feeder,
      BeamBreak beamBreak,
      PoseEstimator pose) {
    // Normal Paths
    PathPlannerPath SubAmp_to_C1 = PathPlannerPath.fromPathFile("SubAmp to C1");
    PathPlannerPath C1_to_Score = PathPlannerPath.fromPathFile("C1 to Score");
    PathPlannerPath Score_to_C2 = PathPlannerPath.fromPathFile("Score to C2");
    PathPlannerPath C2_to_Score = PathPlannerPath.fromPathFile("C2 to Score");
    // Failsafe Paths
    PathPlannerPath C1_to_C2 = PathPlannerPath.fromPathFile("C1 to C2");
    // NOTEs Picked Up
    Trigger firstNote =
        new Trigger(
            () ->
                (pose.getCurrentPose2d().getX() > 7.5
                    && true)); // TODO: Change false to Intake Beam Break once it exists
    Trigger pastMidline = new Trigger(() -> (pose.getCurrentPose2d().getX() > 8.24));

    // Set start position
    pose.resetPose(new Pose2d(0.80, 6.59, Rotation2d.fromDegrees(60)));

    // Normal Path up to First Note
    Commands.runOnce(
            () -> new PreloadShot(arm, wrist, shooter, feeder, ShooterConstants.CLOSE_RPM),
            arm,
            wrist,
            shooter,
            feeder)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.parallel(Commands.runOnce(() -> AutoBuilder.followPath(SubAmp_to_C1), drive)),
            Commands.runOnce(
                () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb));

    if (pastMidline.getAsBoolean()) {
      return Commands.runOnce(null);
    }

    // Pick up C2 if C1 is not grabbed
    if (true) {
      return Commands.parallel(
              Commands.run(() -> AutoBuilder.followPath(C1_to_C2)),
              Commands.runOnce(
                  () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb))
          .andThen(
              Commands.parallel(Commands.runOnce(() -> AutoBuilder.followPath(C2_to_Score), drive)),
              Commands.runOnce(() -> utb.setPercentSpeed(0), utb))
          .andThen(
              Commands.parallel(
                  new AimWrist(arm, wrist, pose), new ReverseNote(shooter, feeder, beamBreak)))
          .andThen(
              Commands.runOnce(
                  () -> new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM),
                  shooter,
                  feeder,
                  beamBreak));
    } else { // Follow the rest of the Normal path
      return Commands.parallel(
              Commands.runOnce(() -> AutoBuilder.followPath(C1_to_Score), drive),
              Commands.runOnce(() -> utb.setPercentSpeed(0), utb),
              new ZeroAll(arm, wrist, shooter, feeder))
          .andThen(
              Commands.parallel(
                  new AimWrist(arm, wrist, pose), new ReverseNote(shooter, feeder, beamBreak)))
          .andThen(
              Commands.runOnce(
                  () -> new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM),
                  shooter,
                  feeder,
                  beamBreak))
          .andThen(()-> new WaitCommand(0.25))
          .andThen(
              Commands.parallel(
                  Commands.runOnce(() -> AutoBuilder.followPath(Score_to_C2)),
                  Commands.runOnce(
                      () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb)),
              new ZeroAll(arm, wrist, shooter, feeder))
          .andThen(
              Commands.parallel(
                  Commands.runOnce(() -> AutoBuilder.followPath(C2_to_Score)),
                  Commands.runOnce(() -> utb.setPercentSpeed(0))))
          .andThen(
              Commands.parallel(
                  new AimWrist(arm, wrist, pose), new ReverseNote(shooter, feeder, beamBreak)))
          .andThen(
              Commands.runOnce(
                  () -> new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM),
                  shooter,
                  feeder,
                  beamBreak));
    }
  }
}
