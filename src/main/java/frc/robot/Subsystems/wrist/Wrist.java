// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

  private final WristIO io;
  private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

  private static PIDController wristPID;

  //temp, move to constants
  private double wristkp = 0.0;
  private double wristki = 0.0;
  private double wristkd = 0.0;


  public Wrist(WristIO io) {
    System.out.println("[Init] Creating wrist");
    this.io = io;

    wristPID = new PIDController(wristkp, wristki, wristkd);
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
  public void setWristMotorSpeed(double Speed) {
    io.setWristMotorSpeed(Speed);
  }
}
