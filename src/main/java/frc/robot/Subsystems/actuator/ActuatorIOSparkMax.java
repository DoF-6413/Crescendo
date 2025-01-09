// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

public class ActuatorIOSparkMax implements ActuatorIO {
  private final SparkMax actuatorMotor;
  private final RelativeEncoder actuatorEncoder;
  private final SparkMaxConfig actuatorConfig;

  /** Runs the real life Actuator with CANSpark Speed Controllers and NEO 550 motor */
  public ActuatorIOSparkMax() {
    System.out.println("[Init] Creating ActuatorIOSparkMax");

    // Initalize various motor objects
    actuatorMotor = new SparkMax(ActuatorConstants.CAN_ID, MotorType.kBrushless);
    actuatorEncoder = actuatorMotor.getEncoder();
    actuatorConfig = new SparkMaxConfig();

    // Update configuration
    actuatorConfig.inverted(ActuatorConstants.IS_INVERTED);
    actuatorConfig.idleMode(IdleMode.kBrake);
    actuatorConfig.smartCurrentLimit(ActuatorConstants.CUR_LIM_A);

    // Apply configuration to SparkMAX
    actuatorMotor.configure(actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    inputs.actuatorCurrentAmps = actuatorMotor.getOutputCurrent();
    inputs.actuatorTempCelsius = actuatorMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    actuatorMotor.setVoltage(volts);
  }

  @Override
  public void setPercentSpeed(double percent) {
    actuatorMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    actuatorConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    actuatorMotor.configure(actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setCurrentLimit(int current) {
    actuatorConfig.smartCurrentLimit(current);
    actuatorMotor.configure(actuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void zeroPosition() {
    actuatorEncoder.setPosition(0);
  }
}
