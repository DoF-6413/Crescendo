// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HeadingControllerConstants;

/** Add your docs here. */
public class HeadingController {

  public HeadingController() {
    /** developer tools */
    SmartDashboard.putNumber("Heading Controller Kp", 1.0);
    SmartDashboard.putNumber("Heading Controller Kd", 0.0);
  }

  /**
   * Returns the output of the heading heading controller with the given Setpoint
   *
   * @param setpoint Target angle
   * @param gyroAngle Current angle of the Gyro
   * @param gyroRate Current angular velocity of the Gyro
   */
  public double update(Rotation2d setpoint, Rotation2d gyroAngle, double gyroRate) {
    double output =
        (setpoint.minus(gyroAngle).getRadians() * HeadingControllerConstants.KP
            + HeadingControllerConstants.KD * gyroRate);
    HeadingControllerConstants.KP = SmartDashboard.getNumber("Heading Controller Kp", 1.0);
    HeadingControllerConstants.KD = SmartDashboard.getNumber("Heading Controller Kd", 0.0);
    SmartDashboard.putBoolean(
        "HeadingControllerAtSetpoint",
        gyroAngle.getRadians() <= Units.degreesToRadians(2) + setpoint.getRadians()
            && gyroAngle.getRadians() >= setpoint.getRadians() - Units.degreesToRadians(2));
    return output;
  }
}
