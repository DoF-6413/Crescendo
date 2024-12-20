// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.*;

public class WristIOSim implements WristIO {

  /** creates a new arm simulation for the wrist of the double jointed arm */
  private final SingleJointedArmSim wristMotor;

  public WristIOSim() {
    System.out.println("[Init] Creating WristIOSim");
    wristMotor =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            WristConstants.GEAR_RATIO,
            WristConstants.MOI_KG_M2,
            WristConstants.LENGTH_M,
            WristConstants.MIN_ANGLE_RAD,
            WristConstants.MAX_ANGLE_RAD,
            WristConstants.IS_SIMULATING_GRAVITY,
            WristConstants.STARTING_ANGLE_RAD);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristMotor.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.wristRelativePositionRad = wristMotor.getAngleRads();
    inputs.wristRelativePositionDeg = Units.radiansToDegrees(wristMotor.getAngleRads());
    inputs.wristAbsolutePositionRad = inputs.wristRelativePositionRad;
    inputs.wristAbsolutePositionDeg = inputs.wristRelativePositionDeg;
    inputs.wristVelocityRadPerSec = wristMotor.getVelocityRadPerSec();
    inputs.wristAppliedVolts = 0.0;
    inputs.wristCurrentAmps = Math.abs(wristMotor.getCurrentDrawAmps());
  }

  @Override
  public void setPercentSpeed(double percent) {
    wristMotor.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }

  @Override
  public void setVoltage(double volts) {
    wristMotor.setInputVoltage(volts);
  }
}
