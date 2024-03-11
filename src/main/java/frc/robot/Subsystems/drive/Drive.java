// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*; // Rotation2d and Translation2d
import edu.wpi.first.math.kinematics.*; // ChassisSpeeds, SwerveDriveKinematics, SwerveModuleStates
import edu.wpi.first.wpilibj.*; // Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Subsystems.gyro.Gyro;
import org.littletonrobotics.junction.Logger; // Logger

/** This Runs the full Swerve (All Modules) for all Modes of the Robot */
public class Drive extends SubsystemBase {
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private static final Module[] modules = new Module[4];
  private final Gyro gyro;
  private Twist2d twist = new Twist2d();

  // swerve kinematics library
  public SwerveDriveKinematics swerveKinematics;

  // chassis & swerve modules
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  // Gets previous Gyro position
  Rotation2d lastGyroYaw = new Rotation2d();

  // Gets previous module positions
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

  public Drive(
      ModuleIO FRModuleIO,
      ModuleIO FLModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      Gyro gyro) {

    System.out.println("[Init] Creating Drive");
    this.gyro = gyro;
    modules[0] = new Module(FRModuleIO, 0);
    modules[1] = new Module(FLModuleIO, 1);
    modules[2] = new Module(BLModuleIO, 2);
    modules[3] = new Module(BRModuleIO, 3);

    // Creates Swerve Dimensions in a 2D plan
    swerveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());
  }

  @Override
  public void periodic() {
    // creates four modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    runSwerveModules(getAdjustedSpeeds());
    getMeasuredStates();
  }

  /** Puts robot to coast mode on disable */
  public void coastOnDisable(boolean isDisabled) {
    if (isDisabled) {
      for (var module : modules) {
        module.setBrakeModeAll(false);
      }
    } else {
      for (var module : modules) {
        module.setBrakeModeAll(true);
      }
    }
  }

  /** */
  public SwerveModuleState[] getAdjustedSpeeds() {
    SwerveModuleState[] setpointStates = new SwerveModuleState[4];
    setpointStates = swerveKinematics.toSwerveModuleStates(setpoint);

    // Renormalizes all wheel speeds so the ratio of velocity remains the same but
    // they don't exceed
    // the maximum speed anymore
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC);
    return setpointStates;
  }

  public void runSwerveModules(SwerveModuleState[] setpointStates) {
    // Runs Modules to Run at Specific Setpoints (Linear and Angular Velocity) that
    // is Quick and
    // Optimized for smoothest movement
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Updates setpoint logs
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
  }
  /** Get Swerve Measured States */
  public SwerveModuleState[] getMeasuredStates() {
    // Tracks the state each module is in
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }

    // Updates what states each module is in (Current Velocity, Angular Velocity,
    // and Angle)
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
    return measuredStates;
  }

  /**
   * Sets the Velocity of the Swerve Drive through Passing in a ChassisSpeeds (Can be Field Relative
   * OR Robot Orientated)
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    setpoint = discreteSpeeds;
  }

  /**
   * Runs the drivetrain with raw values on a scale of (-1, 1)
   *
   * @param x velociy in x direction of Entire Swerve Drive
   * @param y velocity in y direction of Entire Swerve Drive
   * @param rot Angular Velocity of Entire Swerve Drive
   */
  public void setRaw(double x, double y, double rot) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, this.getRotation()));
  }

  /** returns a swerveModuleState of chassis speeds */
  public ChassisSpeeds getChassisSpeed() {
    return swerveKinematics.toChassisSpeeds(
        new SwerveModuleState[] {
          modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()
        });
  }

  public void driveWithDeadband(double x, double y, double rot) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);
    double omega = MathUtil.applyDeadband(rot, DriveConstants.DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // The actual run command itself
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC,
            omega * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
            this.getRotation()));
  }

  /** stops the robot (sets velocity to 0 bu inputing empty Chassis Speeds which Default to 0) */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** stops the robot and sets wheels in the shape of an x */
  public void stopWithX() {
    stop();
    // TODO: Update
  }

  /**
   * @return array of all 4 swerve module positions (turn angle and drive position)
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  public SwerveModulePosition[] getWheelDeltas() {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    /* Wheel Deltas or Wheel Positions */
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters()
                  - lastModulePositionsMeters[i]), // This calculates the change in angle
              modules[i].getAngle()); // Gets individual MODULE rotation
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }
    return wheelDeltas;
  }

  /**
   * Combines the Rotation of the Modules AND the rotation of the gyroscope to determine how we have
   * rotated
   */
  public Rotation2d getRotation() {

    var gyroYaw = new Rotation2d(gyro.getYaw().getRadians());

    /*
     * Twist2d is a change in distance along an arc
     * // x is the forward distance driven
     * // y is the distance driven to the side
     * // (left positive), and the component is the change in heading.
     */
    if (gyro.isConnected()) {
      twist =
          new Twist2d(
              twist.dx,
              twist.dy,
              gyroYaw.minus(lastGyroYaw).getRadians()); // Updates twist based on GYRO
    } else {
      twist =
          swerveKinematics.toTwist2d(getWheelDeltas()); // Updates Twist Based on MODULE position
      gyroYaw =
          lastGyroYaw.minus(
              new Rotation2d(twist.dtheta)); // Updates rotation 2d based on robot module position
    }
    lastGyroYaw = gyroYaw;
    return lastGyroYaw;
  }

  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(twist.dx, twist.dy).rotateBy(getRotation());
    return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(), twist.dtheta);
  }

  public void updateHeading() {
    if (gyro.isConnected()) {
      gyro.zeroYaw();
    } else {
      // TODO: ADD HEADING FOR SIM/NO GYRO
    }
  }
}
