// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class HeadingController {

  public double kp = 5.0;
  public double kd = 100.0;

  public HeadingController() {
    SmartDashboard.putNumber("Heading Controller Kp", 5.0);
    SmartDashboard.putNumber("Heading Controller Kd", 100.0);
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update(Rotation2d setpoint, Rotation2d gyroAngle, double gyroRate) {

    // var output =
    //     controller.calculate(
    //         pose.getCurrentPose2d().getRotation().getRadians(),
    //         goalHeadingSupplier.get().getRadians());

    kp = SmartDashboard.getNumber("Heading Controller Kp", 5.0);
    kd = SmartDashboard.getNumber("Heading Controller Kd", 100.0);
    SmartDashboard.putNumber("Heading Controller", setpoint.getDegrees());

    double output = (setpoint.minus(gyroAngle).getRadians() * kp + kd * gyroRate);

    // Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
    return output;
    // return 0.0;
  }

  /** Returns true if within tolerance of aiming at speaker */
  //   @AutoLogOutput(key = "Drive/HeadingController/atSetpoint")
  // public boolean atSetpoint() {
  //   return controller.atSetpoint();
  // }
}
