// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.RobotStateConstants;

public class ActuatorIOSim implements ActuatorIO {

  private final SingleJointedArmSim actuatorMotor;

  /**
   * This is a simulation for the actuator. It uses the arm simulation because it rotates around a
   * point
   */
  public ActuatorIOSim() {
    System.out.println("[Init] Creating ActuatorIOSim");
    actuatorMotor =
        new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            ActuatorConstants.GEAR_RATIO,
            ActuatorConstants.MOI_KG_M2,
            ActuatorConstants.LENGTH_M,
            ActuatorConstants.MIN_ANGLE_RADS,
            ActuatorConstants.MAX_ANGLE_RADS,
            ActuatorConstants.IS_SIMULATING_GRAVITY,
            ActuatorConstants.START_ANGLE_RADS);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    actuatorMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    // inputs.actuatorAppliedVolts = 0.0;
    inputs.actuatorPositionRad = actuatorMotor.getAngleRads();
    inputs.actuatorPositionDeg = Units.radiansToDegrees(actuatorMotor.getAngleRads());
    inputs.actuatorVelocityRadPerSec = actuatorMotor.getVelocityRadPerSec();
    // inputs.actuatorCurrentAmps = new double[] {actuatorMotor.getCurrentDrawAmps()};
  }

  // @Override
  // public void setActuatorVoltage(double volts) {
  //   actuatorMotor.setInputVoltage(volts);
  // }

  @Override
  public void setPercentSpeed(double percent) {
    actuatorMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }
}
