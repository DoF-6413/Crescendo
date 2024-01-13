// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import frc.robot.Subsystems.gyro.GyroIO.GyroIOInputs;
import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of the Gyro in Every Mode */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
