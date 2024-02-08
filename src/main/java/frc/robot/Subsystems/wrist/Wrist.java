// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO wrist) {
    System.out.println("[Init] Creating wrist");

    this.io = wrist;
  }

  @Override
  public void periodic() {

    this.updateInputs();
    Logger.processInputs("wrist", wristInputs);
  }

  public void updateInputs() {
    io.updateInputs(wristInputs);
  }

  /** Sets speed for the first motor of the wrist */
  public void setFirstWristSpeed(double speed) {
    io.setFirstWristSpeed(speed);
  }

  public void setSecondWristMotorSpeed(double speed) {
    io.setSecondWristMotorSpeed(speed);
  }
}
