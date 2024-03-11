// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SpeakerAutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Utils.PoseEstimator;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class HeadingController {
  private final PIDController controller;
private Supplier<Rotation2d> goalHeadingSupplier;
private PoseEstimator pose;
private Drive drive;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier, PoseEstimator pose, Drive drive) {
    controller = new PIDController(0, 0, 0);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(3));
    this.goalHeadingSupplier = goalHeadingSupplier;
    this.pose = pose;
    this.drive = drive;
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update() {
    // Update controller
    // controller.setPID(kP.get(), 0, kD.get());
    // controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    // ModuleLimits moduleLimits = RobotState.getInstance().getModuleLimits();
    // double maxAngularAcceleration =
    //     moduleLimits.maxDriveAcceleration()
    //         / DriveConstants.driveConfig.driveBaseRadius()
    //         * maxAccelerationMultipler.get();
    // double maxAngularVelocity =
    //     moduleLimits.maxDriveVelocity()
    //         / DriveConstants.driveConfig.driveBaseRadius()
    //         * maxVelocityMultipler.get();
    // controller.setConstraints(
    //     new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

    var output =
        controller.calculate(
            pose.getCurrentPose2d().getRotation().getRadians(),
            goalHeadingSupplier.get().getRadians());

    // Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
    return output;
    // return 0.0;
  }

  /** Returns true if within tolerance of aiming at speaker */
//   @AutoLogOutput(key = "Drive/HeadingController/AtGoal")
  public boolean atGoal() {
    return controller.atSetpoint();
    // EqualsUtil.epsilonEquals(
    // controller.getSetpoint().position,
    // controller.getGoal().position,
    // Units.degreesToRadians(toleranceDegrees.get()));
  }

  public Rotation2d AngleForSpeaker(){
    Translation2d delta;
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        delta = pose.getCurrentPose2d().getTranslation().minus(FieldConstants.RED_SPEAKER);
        delta = delta.minus(new Translation2d(0, (drive.getChassisSpeed().vyMetersPerSecond * 1/8)  * delta.getNorm()).rotateBy(pose.getCurrentPose2d().getRotation().plus(Rotation2d.fromDegrees(180))));  
        return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX())).rotateBy(new Rotation2d(Math.PI));
    } else {
        delta = pose.getCurrentPose2d().getTranslation().minus(FieldConstants.BLUE_SPEAKER);
        delta = delta.plus(new Translation2d(0, (drive.getChassisSpeed().vyMetersPerSecond * 1/8)  * delta.getNorm()).rotateBy(pose.getCurrentPose2d().getRotation()));  
        return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()));
    }                                 
  }
}
