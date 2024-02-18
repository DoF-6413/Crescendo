// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.*;

/** Add your docs here. */
public class OTBIntakeIOSim implements OTBIntakeIO {

  private FlywheelSim intakeMotorSim =
      new FlywheelSim(
          DCMotor.getNEO(1), OTBIntakeConstants.GEAR_RATIO, OTBIntakeConstants.MOI_KG_M2);

  public OTBIntakeIOSim() {
    System.out.println("[Init] Creating OTBIntakeIOSim");
  }

  @Override
  public void updateInputs(OTBIntakeIOInputs inputs) {
    intakeMotorSim.update(RobotStateConstants.LOOP_PERIODIC_SEC);

    inputs.otbIntakeVelocityRPM =
        intakeMotorSim.getAngularVelocityRPM() / OTBIntakeConstants.GEAR_RATIO;
    inputs.otbIntakeAppliedVolts = 0.0;
    inputs.otbIntakeCurrentAmps = new double[] {Math.abs(intakeMotorSim.getCurrentDrawAmps())};
  }

  @Override
  public void setOTBIntakeVoltage(double volts) {
    intakeMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setOTBIntakePercentSpeed(double percent) {
    intakeMotorSim.setInputVoltage(RobotStateConstants.BATTERY_VOLTAGE * percent);
  }
}
