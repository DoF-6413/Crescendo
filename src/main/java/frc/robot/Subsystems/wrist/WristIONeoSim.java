// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

/** Add your docs here. */
public class WristIONeoSim implements WristIO {

  private SingleJointedArmSim wristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          WristConstants.WRIST_GEAR_RATIO,
          1,
          WristConstants.WRIST_LENGTH,
          WristConstants.WRIST_MIN_ANGLE,
          WristConstants.WRIST_MAX_ANGLE,
          false,
          WristConstants.WRIST_STARTING_ANGLE); // TODO: update moment of inertia

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.wristTurnPositionRad +=
        wristMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.wristTurnVelocityRadPerSec = wristMotor.getVelocityRadPerSec();
    inputs.wristTurnAppliedVolts = 0.0;
    inputs.wristTurnCurrentAmps = Math.abs(wristMotor.getCurrentDrawAmps());
    inputs.wristTempCelcius = 0.0;
  }

  @Override
  public void setWristMotorPercent(double percent) {
    wristMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setWristMotorVoltage(double volts) {
    wristMotor.setInputVoltage(volts);
  }
}
