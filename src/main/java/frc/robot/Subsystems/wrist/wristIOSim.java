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

  private SingleJointedArmSim wristMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristNeoConstants.MOTOR_GEAR_RATIO,
          1,
          wristNeoConstants.MOTOR_LENGTH,
          wristNeoConstants.MOTOR_MIN_ANGLE,
          wristNeoConstants.MOTOR_MAX_ANGLE,
          false,
          wristNeoConstants.MOTOR_STARTING_ANGLE); // TODO: update moment of inertia

  @Override
  public void updateInputs(WristIOInputs inputs) {

   
    inputs.wristTurnPositionRad +=
        wristMotor.getVelocityRadPerSec() * RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.wristTurnVelocityRadPerSec = wristMotor.getVelocityRadPerSec();
    inputs.wristTurnAppliedVolts = 0.0;
    inputs.wristTurnCurrentAmps = Math.abs(wristMotor.getCurrentDrawAmps());
    inputs.wristTempCelcius = 0.0;

  }

  public void setWristMotorSpeed(double Speed) {
   
    wristMotor.setInputVoltage(Speed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    wristMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);
  }
}
