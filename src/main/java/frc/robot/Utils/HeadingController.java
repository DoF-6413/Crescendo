// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.HeadingControllerConstants;

/** Add your docs here. */
public class HeadingController {

  public HeadingController() { 
    /**developer tools  */
    // SmartDashboard.putNumber("Heading Controller Kp", 5.0);
    // SmartDashboard.putNumber("Heading Controller Kd", 100.0);
  }

  /** Returns the rotation rate to turn to aim at speaker */
  public double update(Rotation2d setpoint, Rotation2d gyroAngle, double gyroRate) {
  /**output of the heading controller*/
    double output =
        (setpoint.minus(gyroAngle).getRadians() * HeadingControllerConstants.KP + HeadingControllerConstants.KD * gyroRate);
    // kp = SmartDashboard.getNumber("Heading Controller Kp", 5.0);
    // kd = SmartDashboard.getNumber("Heading Controller Kd", 100.0);
        return output;
  }
}
