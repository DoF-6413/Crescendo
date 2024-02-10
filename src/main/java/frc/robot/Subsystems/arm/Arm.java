// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

   public Arm(ArmIO arm) {
    System.out.println("[Init] Creating arm");

    this.io = arm;
  }
   @Override
  public void periodic() {

    this.updateInputs();
    Logger.processInputs("arm", armInputs);
  }

  public void updateInputs() {
    io.updateInputs(armInputs);
  }

  /** Sets speed for the first motor of the arm */
  public void setArmMotorSpeed(double Speed) {
    io.setArmMotorSpeed(Speed);
  }
}
