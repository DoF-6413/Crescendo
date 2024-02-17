// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Constants.RobotStateConstants;

/** Add your docs here. */
public class ActuatorIOSim implements ActuatorIO {

  private SingleJointedArmSim actuatorMotor;

  /** Creates the sim for the actuator */
  public ActuatorIOSim() {
    System.out.println("[Init] Creating ActuatorIOSim");
    // TODO: Update jKgMetersSquared
    actuatorMotor =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            ActuatorConstants.ACTUATOR_GEAR_RATIO,
            ActuatorConstants.ACTUATOR_MOI_KG_M2,
            ActuatorConstants.ACTUATOR_LENGTH_METERS,
            ActuatorConstants.ACTUATOR_MIN_ANGLE_RADS,
            ActuatorConstants.ACTUATOR_MAX_ANGLE_RADS,
            ActuatorConstants.ACTUATOR_IS_SIMULATE_GRAVITY,
            ActuatorConstants.ACTUATOR_START_ANGLE_RADS);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    actuatorMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.actuatorAppliedVolts = 0.0;
    inputs.actuatorPositionRad = actuatorMotor.getAngleRads();
    inputs.actuatorPositionM =
        actuatorMotor.getAngleRads() * (2 * Math.PI); // TODO: Double check math
    inputs.actuatorVelocityRadPerSec =
        actuatorMotor.getVelocityRadPerSec(); // Converts rotaions to Radians and then
    // divides it by the gear ratio
    inputs.actuatorCurrentAmps = new double[] {actuatorMotor.getCurrentDrawAmps()};
  }

  @Override
  public void setActuatorVoltage(double volts) {
    actuatorMotor.setInputVoltage(volts);
  }

  @Override
  public void setActuatorPercentSpeed(double percent) {
    actuatorMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }
}
