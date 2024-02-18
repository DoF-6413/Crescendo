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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public class WristIONeo implements WristIO {

  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristEncoder;

  public WristIONeo() {
    wristMotor = new CANSparkMax(WristConstants.WRIST_CANID, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CUR_LIM_A);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristPositionRad =
        Units.rotationsToRadians(wristEncoder.getPosition()) / WristConstants.WRIST_GEAR_RATIO;
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());

    inputs.wristTempCelsius = new double[] {wristMotor.getMotorTemperature()};
    inputs.wristCurrentAmps = new double[] {wristMotor.getOutputCurrent()};
  }

  @Override
  public void setWristPercentSpeed(double percent) {
    wristMotor.set(percent);
  }

  @Override
  public void setWristVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  @Override
  public double getAngleRads() {
    return Units.rotationsToRadians(wristEncoder.getPosition() / WristConstants.WRIST_GEAR_RATIO);
  }
}
