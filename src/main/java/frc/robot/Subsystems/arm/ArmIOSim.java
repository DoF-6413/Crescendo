// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim armMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmConstants.ARM_GEAR_RATIO,
          ArmConstants.ARM_MOI_KG_M2,
          ArmConstants.ARM_LENGTH_M,
          ArmConstants.ARM_MIN_ANGLE_RAD,
          ArmConstants.ARM_MAX_ANGLE_RAD,
          ArmConstants.ARM_IS_SIMULATING_GRAVITY,
          ArmConstants.ARM_STARTING_ANGLE_RAD);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.armTurnPositionRad +=
        armMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.armTurnVelocityRadPerSec = armMotor.getVelocityRadPerSec();
    inputs.armTurnAppliedVolts = 0.0;
    inputs.armTurnCurrentAmps = new double[] {Math.abs(armMotor.getCurrentDrawAmps())};
  }

  public void setArmPercentSpeed(double percent) {
    armMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  public void setArmMotorVoltage(double volts) {
    armMotor.setInputVoltage(volts);
  }

  public double getAngleRads() {
    return armMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
  }
}
