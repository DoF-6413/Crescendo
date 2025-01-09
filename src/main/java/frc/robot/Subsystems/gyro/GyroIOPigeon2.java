// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Runs Real NavX Gyroscope */
public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 gyro;
  private final StatusSignal<Angle> yawRad;
  private final StatusSignal<AngularVelocity> yawVelocityRadPerSec;

  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");

    gyro = new Pigeon2(GyroConstants.CAN_ID, "CAN2");

    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.optimizeBusUtilization();
    
    yawRad = gyro.getYaw();
    yawVelocityRadPerSec = gyro.getAngularVelocityZWorld();

    yawRad.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
    yawVelocityRadPerSec.setUpdateFrequency(GyroConstants.UPDATE_FREQUENCY_HZ);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yawRad, yawVelocityRadPerSec).isOK();
    inputs.yawPositionRad =
        Rotation2d.fromDegrees(
            MathUtil.inputModulus(yawRad.getValueAsDouble(), 0, 360)
                + GyroConstants.HEADING_OFFSET_DEGREES);
    // and converts it to radians per second
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getAngularVelocityZWorld().getValueAsDouble());
    inputs.rawYawPositionRad = Rotation2d.fromDegrees(yawRad.getValueAsDouble());
  }

  @Override
  public void zeroHeading() {
    gyro.reset();
  }
}
