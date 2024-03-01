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

public class WristIOSparkMax implements WristIO {

  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristEncoder;

  public WristIOSparkMax() {
    /** creates a new wrist motor and encoder */
    wristMotor = new CANSparkMax(WristConstants.CAN_ID, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    /** sets default to brake mode, which locks the motor position */
    wristMotor.setIdleMode(IdleMode.kBrake);

    /** sets current limit */
    wristMotor.setSmartCurrentLimit(WristConstants.CUR_LIM_A);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristPositionRad =
        Units.rotationsToRadians(wristEncoder.getPosition()) / WristConstants.GEAR_RATIO;
    inputs.wristPositionDeg =
        Units.rotationsToDegrees(wristEncoder.getPosition()) / WristConstants.GEAR_RATIO;
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
  public void setWristBrakeMode(boolean enable) {
    if (enable) {
      wristMotor.setIdleMode(IdleMode.kBrake);
    } else {
      wristMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void setWristPositionRads(double angle) {
    //TODO: implement
  }

  
}
