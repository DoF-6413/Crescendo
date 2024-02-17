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
  private final CANSparkMax actuatorMotor;
  private final RelativeEncoder actuatorEncoder;

  /** Creates the motor and encoder for the actuator */
  public ActuatorIOSparkMax() {
    System.out.println("[Init] Creating ActuatorIOSparkMax");
    actuatorMotor = new CANSparkMax(ActuatorConstants.ACTUATOR_CANID, MotorType.kBrushless);
    actuatorEncoder = actuatorMotor.getEncoder();
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.actuatorAppliedVolts = actuatorMotor.getAppliedOutput() * actuatorMotor.getBusVoltage();
    inputs.actuatorPositionRad =
        Units.rotationsToRadians(actuatorEncoder.getPosition()) / ActuatorConstants.GEAR_RATIO;
    inputs.actuatorPositionM =
        Units.rotationsToRadians(actuatorEncoder.getPosition())
            * (2 * Math.PI)
            / ActuatorConstants.GEAR_RATIO; // TODO: Double check math
    inputs.actuatorVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(actuatorEncoder.getVelocity())
            / ActuatorConstants
                .GEAR_RATIO; // Converts rotaions to Radians and then divides it by the gear ratio
    inputs.actuatorCurrentAmps = new double[] {actuatorMotor.getOutputCurrent()}; // The amps used by the Actuator motor
    inputs.actuatorTempCelsius = new double[] {actuatorMotor.getMotorTemperature()}; // The tempature of the Actuator motor in Celsius
  }

  @Override
  public void setActuatorVoltage(double volts) {
    actuatorMotor.setVoltage(volts);
  }

  @Override
  public void setActuatorPercentSpeed(double percent) {
    actuatorMotor.set(percent);
  }
}
