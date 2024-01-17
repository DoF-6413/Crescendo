// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.shooter;

import frc.robot.Subsystems.drive.ModuleIO.ModuleIOInputs;

/**
 * Runs the Real Life (non-simulation) Shooter with TalonFX Speed Controllers
 * and Falcon500 Motors
 */
public class ShooterIOTalonFX implements ShooterIO {

    public ShooterIOTalonFX() {
        System.out.println("[Init] Creating ShooterIOTalonFX");
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
    }
}
