// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

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
    Logger.processInputs("Gyro", inputs);
  }

  /**
   * @return the Roll (Y Axis) in Radians (-pi, pi)
   */
  public Rotation2d getRoll() {
    return inputs.rollPositionRad;
  }

  /**
   * @return the Pitch (X Axis) in Radians (-pi, pi)
   */
  public Rotation2d getPitch() {
    return inputs.pitchPositionRad;
  }

  /**
   * @return returns the Yaw (Z Axis) in Radians (-pi, pi)
   */
  public Rotation2d getYaw() {
    return inputs.yawPositionRad;
  }
  /**
   * @return the angle of the robot in Radian
   */
  public Rotation2d getAngle() {
    return inputs.anglePositionRad;
  }

  /** Resets the Heading to the Direction the Gyro is Facing */
  public void zeroYaw() {
    io.zeroHeading();
  }

  /**
   * @return Returns whether or not the gyro is connected
   */
  public boolean isConnected() {
    return inputs.connected;
  }
}
