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
          wristNeoConstants.FIRST_MOTOR_LENGTH,
          wristNeoConstants.FIRST_MOTOR_MIN_ANGLE,
          wristNeoConstants.FIRST_MOTOR_MAX_ANGLE,
          false,
          wristNeoConstants.FIRST_MOTOR_STARTING_ANGLE);
  // TODO: update moment of inertia
  private SingleJointedArmSim secondWristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristNeoConstants.SECOND_MOTOR_GEAR_RATIO,
          1,
          wristNeoConstants.SECOND_MOTOR_LENGTH,
          wristNeoConstants.SECOND_MOTOR_MIN_ANGLE,
          wristNeoConstants.SECOND_MOTOR_MAX_ANGLE,
          false,
          wristNeoConstants.SECOND_MOTOR_STARTING_ANGLE); // TODO: update moment of inertia

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

  public void setWristMotorsSpeed(double firstSpeed, double secondSpeed) {
    secondWristMotor.setInputVoltage(secondSpeed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    secondWristMotor.update(Constants.RobotStateConstants.LOOP_PERIODIC_SEC);

    firstWristMotor.setInputVoltage(firstSpeed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    firstWristMotor.update(Constants.RobotStateConstants.LOOP_PERIODIC_SEC);
  }
}
