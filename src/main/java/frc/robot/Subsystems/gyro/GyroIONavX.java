// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import frc.robot.Subsystems.gyro.GyroIO.GyroIOInputs;

/** Runs Real NavX Gyroscope */
public class GyroIONavX implements GyroIO {

  public GyroIONavX() {
    System.out.println("[Init] Creating GyroIONavX");
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {}
}
