// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.actuator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Actuator extends SubsystemBase {
  /** Creates a new Actuator. */
  public static ActuatorIO actuatorIO;
  public static ActuatorIOInputsAutoLogged actuatorInputs;


  public Actuator(ActuatorIO io) {

    System.out.println("[init] creating Actuator");
    actuatorIO = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actuatorIO.updateInputs(actuatorInputs);// update the inputs 
    Logger.processInputs("Actuator", actuatorInputs);// logg the inputs 
  }
  public double getActuatorPosition(){ //return the position in meters 
    return actuatorInputs.turnPositionRad * 2 * Math.PI;
  }
  public  void  setActuatorSpeed(double voltage){
    ActuatorIO.setActuatorSpeed(voltage);
  }

  public double getActuatorPositionRad(){
    return actuatorInputs.turnPositionRad;
  }
  public void setActuatorPercentSpeed(double percent){
    ActuatorIO.setActuatorSpeed(percent * 12);
  }
  }
