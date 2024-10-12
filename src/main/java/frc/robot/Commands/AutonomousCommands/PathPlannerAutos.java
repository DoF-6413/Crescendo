package frc.robot.Commands.AutonomousCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.PreloadShot;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ReverseNote;
import frc.robot.Commands.AutonomousCommands.PathPlannerCommands.ShootWhenReady;
import frc.robot.Commands.TeleopCommands.SpeakerScore.AimWrist;
import frc.robot.Commands.ZeroCommands.ZeroAll;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Subsystems.arm.Arm;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.feeder.Feeder;
import frc.robot.Subsystems.shooter.Shooter;
import frc.robot.Subsystems.shooter.ShooterConstants;
import frc.robot.Subsystems.utbintake.UTBIntake;
import frc.robot.Subsystems.utbintake.UTBIntakeConstants;
import frc.robot.Subsystems.wrist.Wrist;
import frc.robot.Utils.BeamBreak;
import frc.robot.Utils.PathPlanner;
import frc.robot.Utils.PoseEstimator;

/**
 * Autos created using PathPlanner Paths but with functionality that cannot be done with PathPlanner
 */
public class PathPlannerAutos {

  /** Same auto as the one from PathPlanner but with decision making and for the Blue side */
  public static Command SubAmp_M1_M2_Smart_Blue(
      Drive drive,
      UTBIntake utb,
      Arm arm,
      Wrist wrist,
      Shooter shooter,
      Feeder feeder,
      BeamBreak beamBreak,
      PoseEstimator pose,
      PathPlanner pathPlanner) {

    // Normal Paths
    PathPlannerPath SubAmp_to_M1 = PathPlannerPath.fromPathFile("SubAmp to M1");
    PathPlannerPath M1_to_Score = PathPlannerPath.fromPathFile("M1 to Score");
    PathPlannerPath Score_to_M2 = PathPlannerPath.fromPathFile("Score to M2");
    PathPlannerPath M2_to_Score = PathPlannerPath.fromPathFile("M2 to Score");
    // Backup Path
    PathPlannerPath M1_to_M2 = PathPlannerPath.fromPathFile("M1 to M2");
    // Triggers
    double firstNoteXMeters = 7.7;
    double tooFarOverMeters = 8.6;
    Trigger firstNote =
        new Trigger(
            () -> pose.getCurrentPose2d().getX() > firstNoteXMeters && beamBreak.isNoteInIntake());
    Trigger pastMidline = new Trigger(() -> pose.getCurrentPose2d().getX() > tooFarOverMeters);

    return Commands.parallel(
            // Shoot preload, set start position
            new PreloadShot(arm, wrist, shooter, feeder, ShooterConstants.CLOSE_RPM),
            Commands.runOnce(
                () -> pose.resetPose(PathPlannerConstants.SUB_AMP_BLUE_START_POSE), pose),
            Commands.runOnce(() -> pathPlanner.enableNOTEAlignment(false), pathPlanner))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            // Zero Entire Shooter Mech, Run Intake, and Go to M1
            Commands.parallel(
                AutoBuilder.followPath(SubAmp_to_M1).until(pastMidline),
                Commands.runOnce(
                    () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb),
                new ZeroAll(arm, wrist, shooter, feeder),
                Commands.runOnce(() -> pathPlanner.enableNOTEAlignment(true), pathPlanner)
                    .beforeStarting(Commands.waitUntil(() -> pose.getCurrentPose2d().getX() > 5))))
        .andThen(
            // Either Score M1, or go to M2 depending if the M1 NOTE was intaken
            Commands.either(
                /* Normal Path */
                // Zero Shooter Mech, Stop Intake, Go to score position
                Commands.parallel(
                        AutoBuilder.followPath(M1_to_Score),
                        Commands.runOnce(() -> utb.setPercentSpeed(0), utb),
                        new ZeroAll(arm, wrist, shooter, feeder),
                        Commands.runOnce(() -> pathPlanner.enableNOTEAlignment(false), pathPlanner))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM))
                    .andThen(new WaitCommand(0.25))
                    .andThen(
                        // Zero Shooter Mech, Run Intake, Go to M2
                        Commands.parallel(
                            AutoBuilder.followPath(Score_to_M2).until(pastMidline),
                            Commands.parallel(
                                Commands.runOnce(
                                    () ->
                                        utb.setPercentSpeed(
                                            UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                                    utb),
                                new ZeroAll(arm, wrist, shooter, feeder),
                                Commands.runOnce(
                                    () -> pathPlanner.enableNOTEAlignment(true), pathPlanner))),
                        new ZeroAll(arm, wrist, shooter, feeder))
                    .andThen(
                        // Stop Intake, Go to score position
                        Commands.parallel(
                            AutoBuilder.followPath(M2_to_Score),
                            Commands.runOnce(() -> utb.setPercentSpeed(0)),
                            Commands.runOnce(
                                () -> pathPlanner.enableNOTEAlignment(false), pathPlanner)))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE. End of the normal path
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM)),
                Commands.parallel(
                        /* Backup Path */
                        // Run Intake, Go to M2
                        AutoBuilder.followPath(M1_to_M2).until(pastMidline),
                        Commands.runOnce(
                            () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                            utb),
                        Commands.runOnce(() -> pathPlanner.enableNOTEAlignment(true), pathPlanner))
                    .andThen(
                        // Stop Intake, Go to Score
                        Commands.parallel(
                            AutoBuilder.followPath(M2_to_Score),
                            Commands.runOnce(() -> utb.setPercentSpeed(0), utb),
                            Commands.runOnce(
                                () -> pathPlanner.enableNOTEAlignment(false), pathPlanner)))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE. End of Backup path
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM)),
                firstNote));
  }

  /** Same auto as the one from PathPlanner but with decision making and for the Red side */
  public static Command SubAmp_M1_M2_Smart_Red(
      Drive drive,
      UTBIntake utb,
      Arm arm,
      Wrist wrist,
      Shooter shooter,
      Feeder feeder,
      BeamBreak beamBreak,
      PoseEstimator pose) {

    // Normal Paths
    PathPlannerPath SubAmp_to_M1 = PathPlannerPath.fromPathFile("SubAmp to M1");
    PathPlannerPath M1_to_Score = PathPlannerPath.fromPathFile("M1 to Score");
    PathPlannerPath Score_to_M2 = PathPlannerPath.fromPathFile("Score to M2");
    PathPlannerPath M2_to_Score = PathPlannerPath.fromPathFile("M2 to Score");
    // Backup Path
    PathPlannerPath M1_to_M2 = PathPlannerPath.fromPathFile("M1 to M2");
    // Triggers
    double firstNoteXMeters = 8.9;
    double tooFarOverMeters = 8.0;
    Trigger firstNote =
        new Trigger(
            () -> pose.getCurrentPose2d().getX() < firstNoteXMeters && beamBreak.isNoteInIntake());
    Trigger pastMidline = new Trigger(() -> pose.getCurrentPose2d().getX() < tooFarOverMeters);

    return Commands.parallel(
            // Shoot preload, set start position
            new PreloadShot(arm, wrist, shooter, feeder, ShooterConstants.CLOSE_RPM),
            Commands.runOnce(
                () -> pose.resetPose(PathPlannerConstants.SUB_AMP_RED_START_POSE), pose))
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            // Zero Entire Shooter Mech, Run Intake, and Go to M1
            Commands.parallel(
                AutoBuilder.followPath(SubAmp_to_M1).until(pastMidline),
                Commands.runOnce(
                    () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb),
                new ZeroAll(arm, wrist, shooter, feeder)))
        .andThen(
            // Either Score M1, or go to M2 depending if the M1 NOTE was intaken
            Commands.either(
                /* Normal Path */
                // Zero Shooter Mech, Stop Intake, Go to score position
                Commands.parallel(
                        AutoBuilder.followPath(M1_to_Score),
                        Commands.runOnce(() -> utb.setPercentSpeed(0), utb),
                        new ZeroAll(arm, wrist, shooter, feeder))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM))
                    .andThen(new WaitCommand(0.25))
                    .andThen(
                        // Zero Shooter Mech, Run Intake, Go to M2
                        Commands.parallel(
                            AutoBuilder.followPath(Score_to_M2).until(pastMidline),
                            Commands.runOnce(
                                () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                                utb)),
                        new ZeroAll(arm, wrist, shooter, feeder))
                    .andThen(
                        // Stop Intake, Go to score position
                        Commands.parallel(
                            AutoBuilder.followPath(M2_to_Score),
                            Commands.runOnce(() -> utb.setPercentSpeed(0))))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE. End of the normal path
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM)),
                Commands.parallel(
                        /* Backup Path */
                        // Run Intake, Go to M2
                        AutoBuilder.followPath(M1_to_M2).until(pastMidline),
                        Commands.runOnce(
                            () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                            utb))
                    .andThen(
                        // Stop Intake, Go to Score
                        Commands.parallel(
                            AutoBuilder.followPath(M2_to_Score),
                            Commands.runOnce(() -> utb.setPercentSpeed(0), utb)))
                    .andThen(
                        // Aim Wrist to SPEAKER and Reverse NOTE
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak)))
                    .andThen(
                        // Shoot NOTE. End of Backup path
                        new ShootWhenReady(
                            shooter, feeder, beamBreak, ShooterConstants.MID_RANGE_RPM)),
                firstNote));
  }
}
