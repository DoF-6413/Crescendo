// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Runs the Real Life Shooter with TalonFX Speed Controllers and Falcon500 Motors */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;

  public ShooterIOTalonFX() {
    System.out.println("[Init] Creating ShooterIOTalonFX");

    // Shooter motor IDs
    topShooterMotor = new TalonFX(ShooterConstants.TOP_MOTOR_ID);
    bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID);

    // Inverts top shooter motor to spin CCW
    topShooterMotor.setInverted(ShooterConstants.TOP_MOTOR_IS_INVERTED);
    bottomShooterMotor.setInverted(ShooterConstants.BOTTOM_MOTOR_IS_INVERTED);

    // sets shooter motors to brake on default
    topShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake);

    // Configures current limits
    final CurrentLimitsConfigs currentLimitsConfig =
        new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.CUR_LIM_A);
    currentLimitsConfig.withSupplyCurrentLimit(ShooterConstants.CUR_LIM_A);
    topShooterMotor.getConfigurator().apply(currentLimitsConfig);
    bottomShooterMotor.getConfigurator().apply(currentLimitsConfig);
    currentLimitsConfig.withStatorCurrentLimitEnable(ShooterConstants.ENABLE_CUR_LIM);
    currentLimitsConfig.withSupplyCurrentLimitEnable(ShooterConstants.ENABLE_CUR_LIM);
  }

  @Override
  /** All the Inputs for the Shooter Motors */
  public void updateInputs(ShooterIOInputs inputs) {
    // Top Shooter Motor Inputs
    inputs.topShooterMotorRPM =
        topShooterMotor.getRotorVelocity().getValueAsDouble()
            * 60
            / ShooterConstants
                .GEAR_RATIO; // Gets the velocity in Rotations per Sec and converts into Rotations
    // Per Min
    // (Gear ratio is 1 so no need to divide the RPM by it)
    inputs.topShooterAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.topShooterCurrentAmps = topShooterMotor.getStatorCurrent().getValueAsDouble();
    inputs.topShooterTempCelsius = topShooterMotor.getDeviceTemp().getValueAsDouble();
    // Bottom Shooter Motor Inputs
    inputs.bottomShooterMotorRPM =
        bottomShooterMotor.getRotorVelocity().getValueAsDouble()
            * 60
            / ShooterConstants
                .GEAR_RATIO; // Gets the velocity in Rotations per Sec and converts into Rotations
    // Per Min
    // (Gear ratio is 1 so no need to divide the RPM by it)
    inputs.bottomShooterAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomShooterCurrentAmps = bottomShooterMotor.getStatorCurrent().getValueAsDouble();
    inputs.bottomShooterTempCelsius = bottomShooterMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    if (enable) {
      topShooterMotor.setNeutralMode(NeutralModeValue.Brake);
      bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
      bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setBothVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void setTopVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
  }

  @Override
  public void setBothPercentSpeed(double percent) {
    topShooterMotor.set(percent);
    bottomShooterMotor.set(percent);
  }

  @Override
  public void setTopPercentSpeed(double percent) {
    topShooterMotor.set(percent);
  }

  @Override
  public void setBottomPercentSpeed(double percent) {
    bottomShooterMotor.set(percent);
  }
}
