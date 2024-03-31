// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.gyro.GyroIO.GyroIOInputs;
import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the Gyro in Every Mode */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    /** Returns whether or not the */
    public boolean connected = false;

    public Rotation2d rollPositionRad = new Rotation2d();
    public Rotation2d pitchPositionRad = new Rotation2d();
    public Rotation2d yawPositionRad = new Rotation2d();
    public Rotation2d rawYawPositionRad = new Rotation2d();
    public Rotation2d anglePositionRad = new Rotation2d();
    /** Gets the angular velocity of the roll */
    public double rollVelocityRadPerSec = 0.0;
    /** Gets the angular velocity of the pitch */
    public double pitchVelocityRadPerSec = 0.0;
    /** Gets the angular velocity of the yaw */
    public double yawVelocityRadPerSec = 0.0;

    public double rateRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Resets the heading to be whereever the front of the robot is facing (front being the intake
   * side)
   */
  public default void zeroHeading() {}
}
