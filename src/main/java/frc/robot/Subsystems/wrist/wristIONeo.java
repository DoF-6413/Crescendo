// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 2 motors
// neo

package frc.robot.Subsystems.wrist;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.wristNeoConstants;

/** Add your docs here. */
public class wristIONeo implements WristIO {

  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristEncoder;

  public wristIONeo() {
    wristMotor = new CANSparkMax(1, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(30);
  }

  public void setWristMotorsSpeed(double Speed) {
    wristMotor.setVoltage(Speed);
  }

  public void updateInputs(WristIOInputs inputs) {
    // first wrist motor

    inputs.wristTurnAppliedVolts = wristMotor.getBusVoltage();
    inputs.wristTurnPositionRad =
        Units.rotationsToRadians(wristEncoder.getPosition()) / wristNeoConstants.MOTOR_GEAR_RATIO;
    inputs.wristTempCelcius = wristMotor.getMotorTemperature();
    inputs.wristTurnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity());
    inputs.wristTurnCurrentAmps = wristMotor.getOutputCurrent();
  }
}
