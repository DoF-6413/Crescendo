// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.otbroller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class OTBRollerIOSim implements OTBRollerIO {
    private final FlywheelSim otbRollerSim;

    public OTBRollerIOSim() {
        otbRollerSim = new FlywheelSim(DCMotor.getNEO(1), OTBRollerConstants.GEAR_RATIO, OTBRollerConstants.MOI_KG_M2);
    }

    @Override
    public void updateInputs(OTBRollerIOInputs inputs) {
        inputs.otbRollerCurrentAmps = otbRollerSim.getCurrentDrawAmps();
        inputs.otbRollerRPM = otbRollerSim.getAngularVelocityRPM();
    }
}
