// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotStateConstants;

/** Runs the real life Arm with CANSpark Speed Controllers and NEO motor */
public class ArmIONeo implements ArmIO {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;

  public ArmIONeo() {
    armMotor = new CANSparkMax(ArmConstants.CAN_ID, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(ArmConstants.CUR_LIM_A);
  }

  @Override

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armAppliedVolts = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.armPositionRad =
        Units.rotationsToRadians(Units.rotationsToRadians(armEncoder.getPosition())) / ArmConstants.GEAR_RATIO;
    
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity())/ ArmConstants.GEAR_RATIO;
    inputs.armTempCelsius = new double[] {armMotor.getMotorTemperature()} ;
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
  public void setBrakeMode(boolean enable){
    if(enable){
    armMotor.setIdleMode(IdleMode.kBrake);
    }else{
      armMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
