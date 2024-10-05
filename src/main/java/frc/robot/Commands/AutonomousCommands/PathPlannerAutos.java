package frc.robot.Commands.AutonomousCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.Utils.PoseEstimator;

public class PathPlannerAutos {

  /**
   * Same auto as the one from PathPlanner but with decision making
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
                    && true)); // TODO: Change true/false to Intake Beam Break once it exists
    Trigger pastMidline = new Trigger(() -> (pose.getCurrentPose2d().getX() > 8.24));

    return Commands.parallel(
            new PreloadShot(arm, wrist, shooter, feeder, ShooterConstants.CLOSE_RPM),
            Commands.runOnce(
                () -> pose.resetPose(PathPlannerConstants.SUB_AMP_START_POSE), pose)) // Shoot preload, set start position
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            Commands.parallel(
                AutoBuilder.followPath(SubAmp_to_C1),
                Commands.runOnce(
                    () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED), utb),
                new ZeroAll(arm, wrist, shooter, feeder))) // Zero Entire Shooter Mech, Run Intake, and Go to C1
        .andThen(
            new ConditionalCommand( // Either Score C1, or go to C2 depending if the C1 NOTE was intaken
            /* Normal Path */
                Commands.parallel(
                        AutoBuilder.followPath(C1_to_Score),
                        Commands.runOnce(() -> utb.setPercentSpeed(0), utb),
                        new ZeroAll(arm, wrist, shooter, feeder)) // Zero Shooter Mech, Stop Intake, Go to score position
                    .andThen(
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak))) // Aim Wrist to SPEAKER and Reverse NOTE
                    .andThen(
                        new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM)) // Shoot NOTE
                    .andThen(new WaitCommand(0.25))
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.followPath(Score_to_C2),
                            Commands.runOnce(
                                () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                                utb)),
                        new ZeroAll(arm, wrist, shooter, feeder)) // Zero Shooter Mech, Run Intake, Go to C2
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.followPath(C2_to_Score),
                            Commands.runOnce(() -> utb.setPercentSpeed(0)))) // Stop Intake, Go to score position
                    .andThen(
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak))) // Aim Wrist to SPEAKER and Reverse NOTE
                    .andThen(new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM)), // Shoot NOTE, End of the normal path
                Commands.parallel(
                    /* Backup Path */
                        AutoBuilder.followPath(C1_to_C2),
                        Commands.runOnce(
                            () -> utb.setPercentSpeed(UTBIntakeConstants.INTAKE_PERCENT_SPEED),
                            utb)) // Run Intak, Go to C2
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.followPath(C2_to_Score),
                        Commands.runOnce(() -> utb.setPercentSpeed(0), utb))) // Stop Intake, Go to Score
                    .andThen(
                        Commands.parallel(
                            new AimWrist(arm, wrist, pose),
                            new ReverseNote(shooter, feeder, beamBreak))) // Aim Wrist to SPEAKER and Reverse NOTE
                    .andThen(new ShootWhenReady(shooter, feeder, beamBreak, ShooterConstants.FAR_RPM)), // Shoot NOTE, End of Backup path
                firstNote));
  }
}
