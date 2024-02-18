// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

/** Add your docs here. */
public class WristIOSim implements WristIO {

  private SingleJointedArmSim wristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          WristConstants.WRIST_GEAR_RATIO,
          WristConstants.WRIST_MOI_KG_M2,
          WristConstants.WRIST_LENGTH,
          WristConstants.WRIST_MIN_ANGLE,
          WristConstants.WRIST_MAX_ANGLE,
          WristConstants.WRSIT_SIMULATE_GRAVITY,
          WristConstants.WRIST_STARTING_ANGLE);

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.wristPositionRad +=
        wristMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.wristVelocityRadPerSec = wristMotor.getVelocityRadPerSec();
    inputs.wristAppliedVolts = 0.0;
    inputs.wristCurrentAmps = new double[] {Math.abs(wristMotor.getCurrentDrawAmps())};
  }

  @Override
  public void setWristPercentSpeed(double percent) {
    wristMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setWristVoltage(double volts) {
    wristMotor.setInputVoltage(volts);
  }
}
