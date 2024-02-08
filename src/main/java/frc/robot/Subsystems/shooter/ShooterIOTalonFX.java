// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

/**
 * Runs the Real Life (non-simulation) Shooter with TalonFX Speed Controllers and Falcon500 Motors
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;

  public ShooterIOTalonFX() {
    System.out.println("[Init] Creating ShooterIOTalonFX");

    // Shooter motor IDs
    topShooterMotor = new TalonFX(ShooterConstants.TOP_SHOOTER_MOTOR_ID);
    bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID);

    // Inverts top shooter motor to spin CCW
    topShooterMotor.setInverted(ShooterConstants.TOP_SHOOTER_MOTOR_INVERTED);
    bottomShooterMotor.setInverted(ShooterConstants.BOTTOM_SHOOTER_MOTOR_INVERTED);

    CurrentLimitsConfigs currentLimitsConfig =
        new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.SMART_CURRENT_LIMIT);
    topShooterMotor.getConfigurator().apply(currentLimitsConfig);
    bottomShooterMotor.getConfigurator().apply(currentLimitsConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // All the Inputs for the Top Shooter Motor (Should be nearly identical to the Bottom Shooter
    // Motor)
    inputs.topShooterMotorRPM =
        topShooterMotor.getRotorVelocity().getValueAsDouble()
            * 60; // getVelocity gets rotations per second, by multiplying it by 60 turns it into
    // rotations per minute (RPM)
    inputs.topShooterAppliedVolts = topShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.topShooterCurrentAmps =
        new double[] {topShooterMotor.getStatorCurrent().getValueAsDouble()};
    inputs.topShooterTempCelcius =
        new double[] {topShooterMotor.getDeviceTemp().getValueAsDouble()};

    // All the Inputs for the Bottom Shooter Motor (Should be nearly identical to the Top Shooter
    // Motor)
    inputs.bottomShooterMotorRPM =
        bottomShooterMotor.getRotorVelocity().getValueAsDouble()
            * 60; // getVelocity gets rotations per second, by multiplying it by 60 turns it into
    // rotations per minute (RPM)
    inputs.bottomShooterAppliedVolts = bottomShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomShooterCurrentAmps =
        new double[] {bottomShooterMotor.getStatorCurrent().getValueAsDouble()};
    inputs.bottomShooterTempCelcius =
        new double[] {bottomShooterMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setShooterBreakMode(boolean enable) {
    if (enable) {
      topShooterMotor.setNeutralMode(NeutralModeValue.Brake);
      bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
      bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setBothShooterMotorsVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void setBottomShooterMotorVoltage(double volts) {
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void setTopShooterMotorVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
  }

  @Override
  public void setBothShooterMotorPercentSpeed(double percent) {
    topShooterMotor.set(percent);
    bottomShooterMotor.set(percent);
  }
}
