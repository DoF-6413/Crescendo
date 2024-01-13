// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** All the Loggable Inputs and Outputs of Each Individual Module */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}
}
