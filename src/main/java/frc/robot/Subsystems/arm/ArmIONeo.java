// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.wristNeoConstants;

/** Add your docs here. */
public class ArmIONeo implements ArmIO {

  private final CANSparkMax armMotor;
  private final RelativeEncoder armEncoder;

  public ArmIONeo() {
    armMotor = new CANSparkMax(1, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(30);
  }

  public void setWristMotorsSpeed(double Speed) {
    armMotor.setVoltage(Speed);
  }
  public void updateInputs(WristIOInputs inputs) {
    // first wrist motor

    inputs.armTurnAppliedVolts = armMotor.getBusVoltage();
    inputs.armTurnPositionRad =
        Units.rotationsToRadians(armEncoder.getPosition())
            / wristNeoConstants.FIRST_MOTOR_GEAR_RATIO;
    inputs.armTempCelcius = armMotor.getMotorTemperature();
    inputs.armTurnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity());
    inputs.armTurnCurrentAmps = armMotor.getOutputCurrent();



}
}