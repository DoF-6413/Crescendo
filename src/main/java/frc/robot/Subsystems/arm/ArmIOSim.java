// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

public class ArmIOSim implements ArmIO {

  /** This is a simulation for the arm */
  private final SingleJointedArmSim armMotor;

  /** Creates and initalizes the simulated arm */
  public ArmIOSim() {
    System.out.println("[Init] Creating ArmIOSim");
    armMotor =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            ArmConstants.GEAR_RATIO,
            ArmConstants.MOI_KG_M2,
            ArmConstants.LENGTH_M,
            ArmConstants.MIN_ANGLE_RAD,
            ArmConstants.MAX_ANGLE_RAD,
            ArmConstants.IS_SIMULATING_GRAVITY,
            ArmConstants.STARTING_ANGLE_RAD);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Updates inputs periodically
    armMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.armRelativePositionRad = armMotor.getAngleRads();
    inputs.armRelativePositionDeg = Units.radiansToDegrees(armMotor.getAngleRads());
    inputs.armAbsolutePositionRad = inputs.armRelativePositionRad;
    inputs.armAbsolutePositionDeg = inputs.armRelativePositionDeg;
    inputs.armVelocityRadPerSec = armMotor.getVelocityRadPerSec();
    inputs.armAppliedVolts = 0.0;
    inputs.armCurrentAmps = Math.abs(armMotor.getCurrentDrawAmps());
  }

  @Override
  public void setPercentSpeed(double percent) {
    armMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setMotorVoltage(double volts) {
    armMotor.setInputVoltage(volts);
  }
}
