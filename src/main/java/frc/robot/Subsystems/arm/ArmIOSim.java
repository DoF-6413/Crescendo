// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.wristNeoConstants;
import frc.robot.Subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.Subsystems.wrist.WristIO;

/** Add your docs here. */
public class ArmIOSim implements ArmIO{

  private SingleJointedArmSim armMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          wristNeoConstants.FIRST_MOTOR_GEAR_RATIO,
          1,
          wristNeoConstants.FIRST_MOTOR_LENGTH,
          wristNeoConstants.FIRST_MOTOR_MIN_ANGLE,
          wristNeoConstants.FIRST_MOTOR_MAX_ANGLE,
          false,
          wristNeoConstants.FIRST_MOTOR_STARTING_ANGLE);

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
   
    armMotor.setInputVoltage(Speed * wristNeoConstants.WRIST_APPLIED_VOLTS);

    armMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);
  }
  
}
