// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ActuatorConstants;

/** Runs the real life Actuator with CANSpark Speed Controllers and NEO 550 motor */
public class ActuatorIOSparkMax implements ActuatorIO {
  private CANSparkMax actuatorMotor;
  private RelativeEncoder actuatorEncoder;

  /** Creates the motor and encoder for the actuator */
  public ActuatorIOSparkMax() {
    System.out.println("[Init] Creating Actuator");
    actuatorMotor = new CANSparkMax(ActuatorConstants.ACTUATOR_CANID, MotorType.kBrushless);
    actuatorEncoder = actuatorMotor.getEncoder();
  }

  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.turnAppliedVolts = actuatorMotor.getBusVoltage();
    inputs.turnPositionRad =
        Units.rotationsToRadians(actuatorEncoder.getPosition()) / ActuatorConstants.GEAR_RATIO;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(actuatorEncoder.getVelocity())
            / ActuatorConstants
                .GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.turnCurrentAmps = actuatorMotor.getOutputCurrent();
  }

  public void setActuatorSpeed(double speed) {
    actuatorMotor.setVoltage(speed);
  }
}