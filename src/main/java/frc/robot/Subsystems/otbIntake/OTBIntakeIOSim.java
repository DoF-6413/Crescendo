// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class OTBIntakeIOSim implements OTBIntakeIO {
  private FlywheelSim intakeMotorSim;
  private double OTBIntakeAppliedVolts = 0.0;

  public OTBIntakeIOSim() {
    System.out.println("[Init] Creating OTBIntakeIOSim");
    intakeMotorSim =
        new FlywheelSim(
            DCMotor.getNEO(1),
            Constants.OTBIntakeConstants.GEAR_RATIO,
            Constants.OTBIntakeConstants.OTB_MOMENT_OF_INERTIA_KGMETERSSQUARED);
  }

  @Override
  public void updateInputs(OTBIntakeIOInputs inputs) {
    inputs.OTBIntakepositionrad +=
        intakeMotorSim.getAngularVelocityRadPerSec()
            * Constants.RobotStateConstants.LOOP_PERIODIC_SEC;
    inputs.otbIntakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
    inputs.otbIntakeAppliedVolts = 0.0;
    inputs.otbIntakeCurrentAmps = new double[] {Math.abs(intakeMotorSim.getCurrentDrawAmps())};
  }

  @Override
  public void setOTBIntakeVoltage(double Volts) {
    intakeMotorSim.setInputVoltage(Volts);
  }
}
