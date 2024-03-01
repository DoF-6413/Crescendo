// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ActuatorIOSparkMax implements ActuatorIO {
  private final CANSparkMax actuatorMotor;
  private final RelativeEncoder actuatorEncoder;

  /** Runs the real life Actuator with CANSpark Speed Controllers and NEO 550 motor */
  public ActuatorIOSparkMax() {
    System.out.println("[Init] Creating ActuatorIOSparkMax");
    actuatorMotor = new CANSparkMax(ActuatorConstants.CAN_ID, MotorType.kBrushless);
    actuatorEncoder = actuatorMotor.getEncoder();
    actuatorMotor.setSmartCurrentLimit(ActuatorConstants.CUR_LIM_A);
    actuatorMotor.setInverted(ActuatorConstants.IS_INVERTED);
    actuatorMotor.setIdleMode(IdleMode.kBrake);
    // actuatorEncoder.setPosition(ActuatorConstants.MIN_ANGLE_RADS);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.actuatorPositionRad =
        Units.rotationsToRadians(actuatorEncoder.getPosition()) / ActuatorConstants.GEAR_RATIO;
    inputs.actuatorPositionDeg =
        Units.rotationsToDegrees(actuatorEncoder.getPosition()) / ActuatorConstants.GEAR_RATIO;
    inputs.actuatorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(actuatorEncoder.getVelocity())
            / ActuatorConstants.GEAR_RATIO;
    inputs.actuatorAppliedVolts = actuatorMotor.getAppliedOutput() * actuatorMotor.getBusVoltage();
    inputs.actuatorCurrentAmps = new double[] {actuatorMotor.getOutputCurrent()};
    inputs.actuatorTempCelsius = new double[] {actuatorMotor.getMotorTemperature()};
  }

  @Override
  public void setActuatorVoltage(double volts) {
    actuatorMotor.setVoltage(volts);
  }

  @Override
  public void setActuatorPercentSpeed(double percent) {
    actuatorMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      actuatorMotor.setIdleMode(IdleMode.kBrake);
    } else {
      actuatorMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
