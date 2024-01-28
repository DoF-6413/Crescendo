// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import edu.wpi.first.math.geometry.*; // Rotation2d and Translation2d
import edu.wpi.first.math.kinematics.*; // ChassisSpeeds, SwerveDriveKinematics, SwerveModuleStates
import edu.wpi.first.wpilibj.*; // Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Subsystems.gyro.Gyro;
import org.littletonrobotics.junction.Logger; // Logger

/** This Runs the full Swerve (All Modules) for all Modes of the Robot */
public class Drive extends SubsystemBase {
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private static final Module[] modules = new Module[4];
  private final Gyro gyro;
  private Twist2d twist;

  // swerve kinematics library
  public SwerveDriveKinematics swerveKinematics;

  // chassis & swerve modules
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };

  public Drive(
      ModuleIO FLModuleIO,
      ModuleIO FRModuleIO,
      ModuleIO BLModuleIO,
      ModuleIO BRModuleIO,
      Gyro gyro) {

    System.out.println("[Init] Creating Drive");
    modules[0] = new Module(FLModuleIO, 0);
    modules[1] = new Module(FRModuleIO, 1);
    modules[2] = new Module(BLModuleIO, 2);
    modules[3] = new Module(BRModuleIO, 3);

    this.gyro = gyro;

    for (var module : modules) {
      module.setBrakeModeAll(true); // set brake mode to be true by default
    }
  }

  @Override
  public void periodic() {
    // creates four modules
    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    if (DriverStation.isDisabled()) {
      // Stops driving when disabled
      for (var module : modules) {
        module.stop();
      }
    }

    // Creates Swerve Dimensions in a 2D plan
    swerveKinematics = new SwerveDriveKinematics(DriveConstants.getModuleTranslations());

    // Creates setpoint logs
    Logger.recordOutput("SwerveStates/Setpoints", new double[] {});
    Logger.recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    // // Fills setpoints array
    // SwerveModuleState[] setpointStates = swerveKinematics.toSwerveModuleStates(setpoint);
    var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * RobotStateConstants.LOOP_PERIODIC_SEC,
                      setpoint.vyMetersPerSecond * RobotStateConstants.LOOP_PERIODIC_SEC,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * RobotStateConstants.LOOP_PERIODIC_SEC)));
      var adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / RobotStateConstants.LOOP_PERIODIC_SEC,
              setpointTwist.dy / RobotStateConstants.LOOP_PERIODIC_SEC,
              setpointTwist.dtheta / RobotStateConstants.LOOP_PERIODIC_SEC);
      SwerveModuleState[] setpointStates = new SwerveModuleState[4];
      setpointStates = swerveKinematics.toSwerveModuleStates(adjustedSpeeds);

    // Renormalizes all wheel speeds so the ratio of velocity remains the same but
    // they don't exceed
    // the maximum speed anymore
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, DriveConstants.MAX_LINEAR_SPEED_M_PER_SEC);

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
    // Tracks the state each module is in
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }

    // Updates what states each module is in (Current Velocity, Angular Velocity,
    // and Angle)
    Logger.recordOutput("SwerveStates/Measured", measuredStates);
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
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, gyro.getYaw()));
  }

  /** stops the robot (sets velocity to 0 bu inputing empty Chassis Speeds which Default to 0) */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** stops the robot and sets wheels in the shape of an x */
  public void stopWithX() {
    stop();
//TODO: Update
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

  public Twist2d getTwist() {
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition((modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
          modules[i].getAngle());
  lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }

    twist = swerveKinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyro.getYaw().getRadians());
    Rotation2d lastGyroYaw = new Rotation2d();
    if (gyro.isConnected()) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    }
    lastGyroYaw = gyroYaw;
    return twist;
  }
}
