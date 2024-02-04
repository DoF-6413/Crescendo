// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPID extends SubsystemBase {
  /** Creates a new ShooterPID. */
  private final PIDController bottomPIDController;

  private final PIDController topPIDController;

  public ShooterPID() {
    bottomPIDController = new PIDController(0, 0, 0);
    topPIDController = new PIDController(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
