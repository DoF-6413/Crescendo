// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/** This Runs Each Individual Module of a Swerve Drive for all Modes of the Robot */
public class Module {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // initialize PID controllers
  private PIDController drivePID = new PIDController(0, 0, 0);
  private PIDController steerPID = new PIDController(0, 0, 0);

  // initialize feedforward
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);

  // construct module
  public Module(ModuleIO io, int index) {
    System.out.println("[Init] Creating Module");
    this.io = io;
    this.index = index;

    // update drive pid values depending on neo or kraken
    drivePID =
        new PIDController(
            DriveConstants.driveKP(io.isL3()),
            DriveConstants.driveKI(io.isL3()),
            DriveConstants.driveKD(io.isL3()));

    // update drive ff values depending on neo or kraken
    driveFeedforward =
        new SimpleMotorFeedforward(
            DriveConstants.driveKS(io.isL3()), DriveConstants.driveKV(io.isL3()));

    // fill steer pid values
    steerPID =
        new PIDController(
            DriveConstants.STEER_KP_NEO, DriveConstants.STEER_KI_NEO, DriveConstants.STEER_KD_NEO);

    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  /** Stops the Robot */
  public void stop() {
    io.setDriveVoltage(0.0);
    io.setTurnVoltage(0.0);
  }

  /** Manually Sets Voltage of the Drive Motor in Individual Module (Max is 12 Volts) */
  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  /** Manually Sets Voltage of the Turn Motor in Individual Module (Max is 12 Volts) */
  public void setTurnVoltage(double volts) {
    io.setTurnVoltage(volts);
  }

  /**
   * Manually Sets the Percent Speed of the Drive Motor in Individual Module (On a -1 to 1 Scale. 1
   * representing 100)
   */
  public void setDrivePercentSpeed(double percent) {
    io.setDriveVoltage(percent * 12);
  }

  /**
   * Manually Sets the Percent Speed of the Turn Motor in Individual Module (On a -1 to 1 Scale. 1
   * representing 100)
   */
  public void setPercentSpeed(double percent) {
    io.setTurnVoltage(percent * 12);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    // Angle Modulus sets the Value Returned to be on a -pi, pi scale
    return new Rotation2d(MathUtil.angleModulus(inputs.turnPositionRad));
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS_M;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS_M;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Sets Breake Mode for Turn and Drive Motors */
  public void setBrakeModeAll(boolean enable) {
    io.setDriveBrakeMode(enable);
    io.setTurnBrakeMode(enable);
  }

  /**
   * Put Values that Should Be Called Periodically for EACH individual Module Here. Module.periodic
   * NEEDS to be in Drive periodic OR it wont run
   */
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    drivePID.setPID(
        DriveConstants.DRIVE_KP_NEO, DriveConstants.DRIVE_KI_NEO, DriveConstants.DRIVE_KD_NEO);
    // System.out.println(drivePID.getP() + " " + drivePID.getI() + " " + drivePID.getD());
    steerPID.setPID(
        DriveConstants.STEER_KP_NEO, DriveConstants.STEER_KI_NEO, DriveConstants.STEER_KD_NEO);
    // System.out.println(steerPID.getP() + " " + steerPID.getI() + " " + steerPID.getD());
    // System.out.println(io.isL3());
  }

  /**
   * Run Setpoint is what Runs a Module based on Chassis Speeds
   *
   * @param Desired Swerve Module State (Desired Velocity and Angle)
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {

    // Optimize state based on current angle (Quickest Path for Wheel to be at Desired Angle (-pi,
    // pi rads))
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Run turn controller
    io.setTurnVoltage(
        steerPID.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    // Update velocity based on turn error
    optimizedState.speedMetersPerSecond *= Math.cos(steerPID.getPositionError());

    // Turn Speed m/s into Vel rad/s
    double velocityRadPerSec = optimizedState.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_M;

    // Run drive controller
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + drivePID.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }
}
