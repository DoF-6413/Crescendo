// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotStateConstants;

/** Add your docs here. */
public class ArmIONeo implements ArmIO {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;

  public ArmIONeo() {
    armMotor = new CANSparkMax(ArmConstants.ARM_CANID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(ArmConstants.ARM_SMART_CURRENT_LIMIT_A);
  }

  public void setArmMotorSpeed(double percent) {
    armMotor.setVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  public void setArmMotorVoltage(double volts) {
    armMotor.setVoltage(volts);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armTurnAppliedVolts = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.armTurnPositionRad =
        Units.rotationsToRadians(armEncoder.getPosition()) / ArmConstants.ARM_GEAR_RATIO;
    inputs.armTurnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity());
    inputs.armTempCelsius = new double[] {armMotor.getMotorTemperature()};
    inputs.armTurnCurrentAmps = new double[] {armMotor.getOutputCurrent()};
  }
}
