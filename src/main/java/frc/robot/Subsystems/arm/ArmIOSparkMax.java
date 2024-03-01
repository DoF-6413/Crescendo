// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.RobotStateConstants;

public class ArmIOSparkMax implements ArmIO {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armRelativeEncoder;
  private DutyCycleEncoder armAbsoluteEncoder;

  /** Runs the real life Arm with CANSpark Speed Controllers and NEO motor */
  public ArmIOSparkMax() {
    armMotor = new CANSparkMax(ArmConstants.CAN_ID, MotorType.kBrushless);
    armRelativeEncoder = armMotor.getEncoder();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(ArmConstants.CUR_LIM_A);
    armMotor.setInverted(ArmConstants.IS_INVERTED);
    armAbsoluteEncoder = new DutyCycleEncoder(9);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAppliedVolts = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.armRelativePositionRad =
        Units.rotationsToRadians(Units.rotationsToRadians(armRelativeEncoder.getPosition()))
            / ArmConstants.GEAR_RATIO;
    inputs.armRelativePositionDeg = Units.rotationsToDegrees(armRelativeEncoder.getPosition());
    inputs.armAbsolutePositionRad = armAbsoluteEncoder.getAbsolutePosition(); //Update
    inputs.armAbsolutePositionDeg = armAbsoluteEncoder.getAbsolutePosition(); //Update
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armRelativeEncoder.getVelocity())
            / ArmConstants.GEAR_RATIO;
    inputs.armTempCelsius = new double[] {armMotor.getMotorTemperature()};
    inputs.armCurrentAmps = new double[] {armMotor.getOutputCurrent()};
  }

  @Override
  public void setArmPercentSpeed(double percent) {
    armMotor.setVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setArmVoltage(double volts) {
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
