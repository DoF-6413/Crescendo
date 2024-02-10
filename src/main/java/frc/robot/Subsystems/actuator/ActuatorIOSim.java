// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
        actuatorMotor = new SingleJointedArmSim(DCMotor.getNeo550(1), ActuatorConstants.GEAR_RATIO, ActuatorConstants.ACTUATOR_JKG_METERS_SQUARED,
                ActuatorConstants.ACTUATOR_LENGTH_METERS, ActuatorConstants.ACTUATOR_MIN_ANGLE_RADS, ActuatorConstants.ACTUATOR_MAX_ANGLE_RADS, ActuatorConstants.IS_SIMULATE_GRAVITY,
                ActuatorConstants.ACTUATOR_START_ANGLE_RADS);
    }

    @Override
    public void updateInputs(ActuatorIOInputs inputs) {
        inputs.turnAppliedVolts = 0.0;
        inputs.turnPositionRad = actuatorMotor.getAngleRads();
        inputs.turnVelocityRadPerSec = actuatorMotor.getVelocityRadPerSec(); // Converts rotaions to Radians and then
                                                                             // divides it by the gear ratio
        inputs.turnCurrentAmps = actuatorMotor.getCurrentDrawAmps();
    }

    @Override
    public void setActuatorVoltage(double volts) {
        actuatorMotor.setInput(volts);
    }

    @Override
    public void setActuatorPercentSpeed(double percent) {
        actuatorMotor.setInput(percent * RobotStateConstants.BATTERY_VOLTAGE);
    }
}