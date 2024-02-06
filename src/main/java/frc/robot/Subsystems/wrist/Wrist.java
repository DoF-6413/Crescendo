// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    
    public Wrist(wristIONeo io) {
        System.out.println("[Init] Creating wrist");

        this.io = io;
    }

    @Override
    public void periodic(){

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
    io.setSecondWristSpeed(speed);
  }
}
