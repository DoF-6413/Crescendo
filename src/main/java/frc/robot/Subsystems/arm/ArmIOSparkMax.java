// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.RobotStateConstants;

public class ArmIOSparkMax implements ArmIO {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armRelativeEncoder;
  private final DutyCycleEncoder armAbsoluteEncoder;

  /** Runs the real life Arm with CANSpark Speed Controllers and NEO motor */
  public ArmIOSparkMax() {
    armMotor = new CANSparkMax(ArmConstants.CAN_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(ArmConstants.CUR_LIM_A);
    armMotor.setInverted(ArmConstants.IS_INVERTED);
    armRelativeEncoder = armMotor.getEncoder();
    armAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_CHANNEL);

    armMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAppliedVolts = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.armRelativePositionRad =
        Units.rotationsToRadians(Units.rotationsToRadians(armRelativeEncoder.getPosition()))
            / ArmConstants.GEAR_RATIO;
    inputs.armRelativePositionDeg = Units.rotationsToDegrees(armRelativeEncoder.getPosition());
    // The absolute encoder, or a dut cycle encoder, rotates where a full rotation is equal to 1. If
    // 1 rotation is equal to 2pi or 360 degrees, multiply by appropriate to get value
    inputs.armAbsolutePositionRad =
        (1 - armAbsoluteEncoder.getAbsolutePosition()) * 2 * Math.PI - ArmConstants.OFFSET_RAD;
    inputs.armAbsolutePositionDeg =
        (1 - armAbsoluteEncoder.getAbsolutePosition()) * 360
            - Units.radiansToDegrees(ArmConstants.OFFSET_RAD);
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armRelativeEncoder.getVelocity())
            / ArmConstants.GEAR_RATIO;
    inputs.armTempCelsius = armMotor.getMotorTemperature();
    inputs.armCurrentAmps = armMotor.getOutputCurrent();
  }

  @Override
  public void setPercentSpeed(double percent) {
    armMotor.setVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setMotorVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      armMotor.setIdleMode(IdleMode.kBrake);
    } else {
      armMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
