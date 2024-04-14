// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 2 motors
// neo

package frc.robot.Subsystems.wrist;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;

public class WristIOSparkMax implements WristIO {

  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristRelativeEncoder;
  private final SparkAbsoluteEncoder wristAbsoluteEncoder;

  // private final RelativeEncoder wristEncoder;

  public WristIOSparkMax() {
    /** Creates a new Wrist motor and encoder */
    wristMotor = new CANSparkMax(WristConstants.CAN_ID, MotorType.kBrushless);
    wristRelativeEncoder = wristMotor.getEncoder();
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristAbsoluteEncoder.setInverted(WristConstants.IS_INVERTED);
    wristAbsoluteEncoder.setZeroOffset(0.2);

    /** sets default to brake mode, which locks the motor position */
    wristMotor.setIdleMode(IdleMode.kBrake);

    /** sets current limit */
    wristMotor.setSmartCurrentLimit(WristConstants.CUR_LIM_A);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristRelativePositionRad =
        Units.rotationsToRadians(wristRelativeEncoder.getPosition()) / WristConstants.GEAR_RATIO;
    inputs.wristRelativePositionDeg =
        Units.rotationsToDegrees(wristRelativeEncoder.getPosition()) / WristConstants.GEAR_RATIO;
    // The absolute encoder, or a dut cycle encoder, rotates where a full rotation is equal to 1. If
    // 1 rotation is equal to 2pi or 360 degrees, multiply by appropriate to get value
    inputs.wristAbsolutePositionRad =
        (wristAbsoluteEncoder.getPosition() * 2 * Math.PI + WristConstants.ABS_ENCODER_OFFSET_RADS);
    inputs.wristAbsolutePositionDeg =
        (wristAbsoluteEncoder.getPosition() * 360)
            + Units.radiansToDegrees(WristConstants.ABS_ENCODER_OFFSET_RADS);
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristRelativeEncoder.getVelocity());
    inputs.wristTempCelsius = new double[] {wristMotor.getMotorTemperature()};
    inputs.wristCurrentAmps = new double[] {wristMotor.getOutputCurrent()};
  }

  @Override
  public void setPercentSpeed(double percent) {
    wristMotor.set(percent);
  }

  @Override
  public void setVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      wristMotor.setIdleMode(IdleMode.kBrake);
    } else {
      wristMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
