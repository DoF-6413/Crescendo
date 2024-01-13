// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

/**
 * Runs Simulation for an Individual Swerve Module with the Turn Motor as a Neo and the Drive Motor
 * as a Kraken
 */
public class ModuleIOSimNeoKraken implements ModuleIO {

  public ModuleIOSimNeoKraken() {
    System.out.println("[Init] Creating ModuleIOSimNeoKraken");
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {}
}
