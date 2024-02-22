// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.feeder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.feeder.FeederIO;
import frc.robot.Utils.PIDController;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
  private GenericEntry feederkP;
  private GenericEntry feederkI;
  private GenericEntry feederkD;
  private GenericEntry feederSetpointSetter;
  private double setpointRPM = 0.0;
  private final PIDController feederPIDController;

  /** Creates a new Feeder */
  public Feeder(FeederIO io) {
    System.out.println("[Init] Creating Feeder");
    this.io = io;

    feederPIDController = new PIDController(FeederConstants.KP, FeederConstants.KI, FeederConstants.KD);
    feederPIDController.setSetpoint(setpointRPM);
    feederPIDController.setTolerance(setpointRPM * FeederConstants.TOLERANCE_PERCENT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }
}
