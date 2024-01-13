// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This Runs the Gyro for all Modes of the Robot */
public class Gyro extends SubsystemBase {

  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    System.out.println("[Init] Creating Gyro");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
