// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

/**
 * Runs the Real Life (non-simulation) Shooter with TalonFX Speed Controllers and Falcon500 Motors
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX leftShooterMotor;
  // private final Follower rightShooterMotor;
  private final TalonFX rightShooterMotor;

  public ShooterIOTalonFX() {
    System.out.println("[Init] Creating ShooterIOTalonFX");
    leftShooterMotor = new TalonFX(0);
    // rightShooterMotor = new Follower(leftShooterMotor.getDeviceID(), true);
    rightShooterMotor = new TalonFX(1);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // All the Inputs for the Left Shooter Motor (Should be nearly identical to the Right Shooter
    // Motor)
    inputs.leftShooterMotorRPM =
        Units.radiansPerSecondToRotationsPerMinute(
            leftShooterMotor.getDifferentialAverageVelocity().getValueAsDouble());
    inputs.leftShooterAppliedVolts = leftShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftShooterCurrentAmps =
        new double[] {leftShooterMotor.getStatorCurrent().getValueAsDouble()};
    inputs.leftShooterTempCelcius =
        new double[] {leftShooterMotor.getDeviceTemp().getValueAsDouble()};

    // All the Inputs for the Right Shooter Motor (Should be nearly identical to the Left Shooter
    // Motor)
    inputs.rightShooterMotorRPM =
        Units.radiansPerSecondToRotationsPerMinute(
            rightShooterMotor.getDifferentialAverageVelocity().getValueAsDouble());
    inputs.rightShooterAppliedVolts = rightShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterCurrentAmps =
        new double[] {rightShooterMotor.getStatorCurrent().getValueAsDouble()};
    inputs.rightShooterTempCelcius =
        new double[] {rightShooterMotor.getDeviceTemp().getValueAsDouble()};
  }

  @Override
  public void setShooterMotorsVoltage(double volts) {
    leftShooterMotor.setVoltage(volts);
    rightShooterMotor.setVoltage(volts);
  }

  @Override
  public void setShooterBreakMode(boolean enable) {
    if (enable) {
      leftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  // @Override
  public void setShooterMotorPercentSpeed(double percent) {
    leftShooterMotor.setVoltage(12 * percent);
    rightShooterMotor.setVoltage(12 * percent);
  }
}
