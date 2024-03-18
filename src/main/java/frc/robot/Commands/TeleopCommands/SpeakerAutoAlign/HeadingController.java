// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TeleopCommands.SpeakerAutoAlign;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.PoseEstimatorLimelight;
import java.util.function.Supplier;

/** Add your docs here. */
public class HeadingController {
  private final PIDController controller;
  private Supplier<Rotation2d> goalHeadingSupplier;
  private PoseEstimatorLimelight pose;

  public HeadingController(
      Supplier<Rotation2d> goalHeadingSupplier, PoseEstimatorLimelight m_poseEstimator) {
    controller = new PIDController(0.1, 0.0, 0.0);
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(5));
    this.goalHeadingSupplier = goalHeadingSupplier;
    this.pose = m_poseEstimator;
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
}
