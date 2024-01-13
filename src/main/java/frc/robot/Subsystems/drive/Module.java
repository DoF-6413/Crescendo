// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.DriveConstants;

/** This Runs Each Individual Module of a Swerve Drive for all Modes of the Robot */
public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private PIDController drivePID = new PIDController(0, 0, 0);
  private PIDController steerPID = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);

  public Module(ModuleIO io, int index) {
    System.out.println("[Init] Creating Module");
    this.io = io;
    this.index = index;

    drivePID =
        new PIDController(
            DriveConstants.driveKP(io.isL3()),
            DriveConstants.driveKI(io.isL3()),
            DriveConstants.driveKD(io.isL3()));

    driveFeedforward =
        new SimpleMotorFeedforward(
            DriveConstants.driveKS(io.isL3()), DriveConstants.driveKV(io.isL3()));

    steerPID = new PIDController(DriveConstants.STEER_KP_NEO,DriveConstants.STEER_KI_NEO, DriveConstants.STEER_KD_NEO);

    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {}
}
