// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Subsystems.gyro.GyroIO.GyroIOInputs;

/** Runs Real NavX Gyroscope */
public class GyroIONavX implements GyroIO {

  private AHRS gyro;

  public GyroIONavX() {
    System.out.println("[Init] Creating GyroIONavX");
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    gyro.setAngleAdjustment(GyroConstants.HEADING_OFFSET_DEGREES);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.rollPositionRad = new Rotation2d(Units.degreesToRadians(gyro.getRoll()));
    inputs.pitchPositionRad = new Rotation2d(Units.degreesToRadians(gyro.getPitch()));
    // Value is Negative because NavX reads CW and everything else runs CCW
    inputs.yawPositionRad =
        new Rotation2d(
            Units.degreesToRadians(
                -gyro.getYaw() + GyroConstants.HEADING_OFFSET_DEGREES)); // TODO: Make -90 constant
    inputs.rawYawPositionRad = new Rotation2d(Units.degreesToRadians(gyro.getYaw()));
    inputs.anglePositionRad = new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
    inputs.rollVelocityRadPerSec =
        Units.degreesToRadians(
            gyro.getRawGyroY()); // Gets the angular velocity, in degrees per second, of the roll
    // and converts it to radians per second
    inputs.pitchVelocityRadPerSec =
        Units.degreesToRadians(
            gyro.getRawGyroX()); // Gets the angular velocity, in degrees per second, of the pitch
    // and converts it to radians per second
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(
            gyro.getRawGyroZ()); // Gets the angular velocity, in degrees per second, of the yaw and
    // converts it to radians per second

    inputs.rateRadPerSec = Units.degreesToRadians(gyro.getRate());
  }

  @Override
  public void zeroHeading() {
    gyro.zeroYaw();
  }
}
