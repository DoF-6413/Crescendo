// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.wristNeoConstants;

/** Add your docs here. */
public class wristIOSim implements WristIO {

  private SingleJointedArmSim firstWristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristNeoConstants.FIRST_MOTOR_GEAR_RATIO,
          1,
          16.245309,
          22.360160,
          107,
          false,
          22.360160); // TODO: update moment of inertia change degrees to rad
  private SingleJointedArmSim secondWristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristNeoConstants.SECOND_MOTOR_GEAR_RATIO,
          1,
          10.40410,
          5.715446,
          99.284554,
          false,
          5.715446); // TODO: update moment of inertia change degrees to rad

  @Override
  public void updateInputs(WristIOInputs inputs) {

    // update inputs of the first (the farthest to the shooter)

    inputs.firstWristTurnPositionRad +=
        firstWristMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.firstWristTurnVelocityRadPerSec = firstWristMotor.getVelocityRadPerSec();
    inputs.firstWristTurnAppliedVolts = 0.0;
    inputs.firstWristTurnCurrentAmps = Math.abs(firstWristMotor.getCurrentDrawAmps());
    inputs.firstWristTempCelcius = 0.0;

    // update inputs of the second motor (closest to the shooter)

    inputs.secondWristTurnPositionRad +=
        secondWristMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.secondWristTurnVelocityRadPerSec = secondWristMotor.getVelocityRadPerSec();
    inputs.secondWristTurnAppliedVolts = 0.0;
    inputs.secondWristTurnCurrentAmps = Math.abs(secondWristMotor.getCurrentDrawAmps());
    inputs.secondWristTempCelcius = 0.0;
  }

  public void setFirstWristSpeed(double speed) {
    firstWristMotor.setInputVoltage(speed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    firstWristMotor.update(Constants.RobotStateConstants.LOOP_PERIODIC_SEC);
  }

  public void setSecondWristMotorSpeed(double speed) {
    secondWristMotor.setInputVoltage(speed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    secondWristMotor.update(Constants.RobotStateConstants.LOOP_PERIODIC_SEC);
  }
}
