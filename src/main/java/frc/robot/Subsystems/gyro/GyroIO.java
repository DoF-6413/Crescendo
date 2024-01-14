// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import frc.robot.Subsystems.gyro.GyroIO.GyroIOInputs;
import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the Gyro in Every Mode */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double anglePositionRad = 0.0;
    public double rate = 0.0;
    public double rollVelocityDegPerSec = 0.0;
    public double pitchVelocityDegPerSec = 0.0;
    public double yawVelocityDegPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Resets the Heading/ Direction the Robot Considers "Forward" */
  public default void zeroHeading() {}
}
