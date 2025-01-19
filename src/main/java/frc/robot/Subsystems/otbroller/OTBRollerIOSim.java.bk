// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotStateConstants;

/** Add your docs here. */
public class OTBRollerIOSim implements OTBRollerIO {
  private final FlywheelSim otbRollerSim;

  public OTBRollerIOSim() {
    System.out.println("[Init] Creating OTBRollerIOSim");
    otbRollerSim =
        new FlywheelSim(
            DCMotor.getNEO(1), OTBRollerConstants.GEAR_RATIO, OTBRollerConstants.MOI_KG_M2);
  }

  @Override
  public void updateInputs(OTBRollerIOInputs inputs) {
    otbRollerSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.otbRollerRPM = otbRollerSim.getAngularVelocityRPM();
    inputs.otbRollerCurrentAmps = Math.abs(otbRollerSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double voltage) {
    otbRollerSim.setInputVoltage(voltage);
  }

  @Override
  public void setPercentSpeed(double percent) {
    otbRollerSim.setInputVoltage(percent * RobotStateConstants.BATTERY_VOLTAGE);
  }
}
