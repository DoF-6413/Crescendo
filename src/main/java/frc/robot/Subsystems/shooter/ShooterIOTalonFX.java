// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/**
 * Runs the Real Life (non-simulation) Shooter with TalonFX Speed Controllers and Falcon500 Motors
 */
public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX leftElevatorMotor;
private final Follower rightElevatorMotor;

  public ShooterIOTalonFX() {
    System.out.println("[Init] Creating ShooterIOTalonFX");
    leftElevatorMotor = new TalonFX(0);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
        // All the Inputs for the Left Shooter Motor (Should be nearly identical to the Right Shooter
    // Motor)
    inputs.leftShooterMotorRPM = Units.radiansPerSecondToRotationsPerMinute(leftElevatorMotor.getDifferentialAverageVelocity().getValueAsDouble());
    inputs.leftShooterAppliedVolts = leftElevatorMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftShooterCurrentAmps = new double[] {leftElevatorMotor.getStatorCurrent().getValueAsDouble()};
    inputs.leftShooterTempCelcius = new double[] {leftElevatorMotor.getDeviceTemp().getValueAsDouble()};

    // All the Inputs for the Right Shooter Motor (Should be nearly identical to the Left Shooter
    // Motor)
    inputs.rightShooterMotorRPM = Units.radiansPerSecondToRotationsPerMinute(rightElevatorMotor.getDifferentialAverageVelocity().getValueAsDouble());
    inputs.rightShooterAppliedVolts = rightElevatorMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterCurrentAmps = new double[] {rightElevatorMotor.getStatorCurrent().getValueAsDouble()};
    inputs.rightShooterTempCelcius = new double[] {rightElevatorMotor.getDeviceTemp().getValueAsDouble()};
  }

    @Override
    public void setShooterMotorsVoltage(double volts) {
        leftElevatorMotor.setVoltage(volts);
    }

  
    @Override
    public void setShooterBreakMode(boolean enable) {
        if(enable){
        leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        }else{
        rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }
}
