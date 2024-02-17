// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim armMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmConstants.MOTOR_GEAR_RATIO,
          1,
          ArmConstants.MOTOR_LENGTH,
          ArmConstants.MOTOR_MIN_ANGLE,
          ArmConstants.MOTOR_MAX_ANGLE,
          false,
          ArmConstants.MOTOR_STARTING_ANGLE);

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.armTurnPositionRad +=
        armMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.armTurnVelocityRadPerSec = armMotor.getVelocityRadPerSec();
    inputs.armTurnAppliedVolts = 0.0;
    inputs.armTurnCurrentAmps = Math.abs(armMotor.getCurrentDrawAmps());
    inputs.armTempCelcius = 0.0;
  }

  public void setArmMotorSpeed(double Speed) {

    armMotor.setInputVoltage(Speed * ArmConstants.WRIST_APPLIED_VOLTS);

    armMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);
  }
}
